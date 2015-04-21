#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

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
    cout << "Entered find_squares\n";
    // blur will enhance edge detection
    Mat blurred(image);
    medianBlur(image, blurred, 9);

    Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point> > contours;


    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
        cout << "Entered for loop\n";
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
    cout << "Exiting find_squares\n";
}

void find_largest_square(const vector<vector<Point> >& squares, vector<Point>& biggest_square)
{
    if (!squares.size())
    {
        // no squares detected
        return;
    }

    int max_width = 0;
    int max_height = 0;
    int max_square_idx = 0;
    const int n_points = 4;

    for (size_t i = 0; i < squares.size(); i++)
    {
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

int main(int, char**)
{
    cout << "Entered main\n";
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
    {
        cout << "Failed to open camera, exiting\n";
        return -1;
    }
    Mat frame, edges;
    namedWindow("edges",1);
    while(true)
    {
        cout << "entered while loop\n";
        cap >> frame; // get a new frame from camera
        cvtColor(frame, edges, CV_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        Canny(edges, edges, 0, 30, 3);
        imshow("edges", edges);
        if(waitKey(30) >= 0) break;

        // Detect all regions in the image that are similar to a rectangle
        vector<vector<Point> > squares;
        find_squares(edges, squares);

        // The largest of them probably represents the paper
        vector<Point> largest_square;
        find_largest_square(squares, largest_square);

        // Print the x,y coordinates of the square
        cout << "Point 1: " << largest_square[0] << endl;
        cout << "Point 2: " << largest_square[1] << endl;
        cout << "Point 3: " << largest_square[2] << endl;
        cout << "Point 4: " << largest_square[3] << endl;
        cout << "\n";
    }
    // camera deinitialized automatically in VideoCapture destructor
    return 0;
}
