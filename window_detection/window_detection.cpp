#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

Mat draw_square(vector<vector<Point> > squares, Mat image)
{
    for (int i = 0; i < squares.size(); i++)
    {
        // draw contour
        drawContours(image, squares, i, Scalar(255,0,0), 1, 8, vector<Vec4i>(), 0, Point());

        // draw bounding rect
        /*
        Rect rect = boundingRect(Mat(squares[i]));
        rectangle(image, rect.tl(), rect.br(), Scalar(0,255,0), 2, 8, 0);
        */

        // draw rotated rect
        RotatedRect minRect = minAreaRect(Mat(squares[i]));
        Point2f rect_points[4];
        minRect.points( rect_points );
        for ( int j = 0; j < 4; j++ ) 
        {
            // line( image, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 1, 8); // blue 
            line( image, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,0), 1, 8); // green  
        }
    }
    return image;
}

double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 ) 
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void find_squares(Mat& image, vector<vector<Point> >& squares)
{
    //cout << "Entered find_squares\n";
    // blur will enhance edge detection
    Mat blurred(image);
    medianBlur(image, blurred, 9);

    Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
        //cout << "Entered for loop\n";
        //int c = 0;
        int ch[] = {c, 0};
        mixChannels(&blurred, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        const int threshold_level = 2;
        for (int l = 0; l < threshold_level; l++)
        {
            // Use Canny instead of zero threshold level!
            // Canny helps to catch squares with gradient shading
            if (l == 0)
            {
                Canny(gray0, gray, 10, 20, 3); // 

                // Dilate helps to remove potential holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                gray = gray0 >= (l+1) * 255 / threshold_level;
            }

            // Find contours and store them in a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // Test contours
            vector<Point> approx;
            for (size_t i = 0; i < contours.size(); i++)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)))
                {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if (maxCosine < 0.3)
                        squares.push_back(approx);
                }
            }
        }
    }
    //cout << "Exiting find_squares\n";
}

void find_largest_square(const vector<vector<Point> >& squares, vector<Point>& biggest_square)
{
    //cout << "Finding largest square \n";
    //cout << "*******************************************************************\n\n\n";
    if (squares.size() == 0)
    {
        return;
    }
    //cout << "got here \n";

    int max_width = 0;
    int max_height = 0;
    int max_square_idx = 0;
    const int n_points = 4;

    for (size_t i = 0; i < squares.size(); i++)
    {
        //cout << "Entered for loop\n";
        // Convert a set of 4 unordered Points into a meaningful cv::Rect structure.
        Rect rectangle = boundingRect(Mat(squares[i]));

        // cout << "find_largest_square: #" << i << " rectangle x:" << rectangle.x << " y:" << rectangle.y << " " << rectangle.width << "x" << rectangle.height << endl;

        // Store the index position of the biggest square found
        if ((rectangle.width >= max_width) && (rectangle.height >= max_height))
        {
            max_width = rectangle.width;
            max_height = rectangle.height;
            max_square_idx = i;
        }
    }
    biggest_square = squares[max_square_idx];
}

VideoCapture initialize_window_detection()
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
    {
        cout << "Failed to open camera, exiting\n";
        return cap;
    }
}

int detect_window(VideoCapture cap, int frame_size)
{
    Mat frame, edges, quarter_frame;
    //namedWindow("edges",1);
    namedWindow("frame", 1);

    //cout << "entered while loop\n";
    cap >> frame; // get a new frame from camera
    
    /* Gaussian Blur with Canny Edge Detector */
    /*
    cvtColor(frame, edges, CV_BGR2GRAY);
    GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
    Canny(edges, edges, 0, 30, 3);
    */
    //imshow("edges", edges);

    // Detect all regions in the image that are similar to a rectangle
    vector<vector<Point> > squares;

    // Downsample the image to speed up computation
    pyrDown(frame, quarter_frame, Size(frame.cols/2, frame.rows/2));
    if (frame_size == 1) frame = quarter_frame; // change here 

    find_squares(frame, squares);

    cout << "squares.size() = " << squares.size() << "\n";
    if (squares.size() == 0)
    {
        cout << "No squares found.\n";
        //return -1;
    }

    else
    {
        // The largest of them probably represents the window
        vector<Point> largest_square;
        find_largest_square(squares, largest_square);

        // Print the x,y coordinates of the square
        cout << "Point 1: " << largest_square[0] << endl;
        cout << "Point 2: " << largest_square[1] << endl;
        cout << "Point 3: " << largest_square[2] << endl;
        cout << "Point 4: " << largest_square[3] << endl;
        cout << "\n";
        draw_square(squares, frame);
    }
    imshow("frame", frame);

    //if(waitKey(30) >= 0) break;

    return 0;
}

int main(int, char**)
{
    VideoCapture cap = initialize_window_detection();
    int distance;
    int frame_size = 1;
    while(true)
    {
        distance = detect_window(cap, frame_size);
        if(waitKey(30) >= 0) 
            break;
    }
    // camera deinitialized automatically in VideoCapture destructor
    return 0;
}


