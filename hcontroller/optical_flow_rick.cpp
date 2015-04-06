/* --Sparse Optical Flow Demo Program--
 * Written by David Stavens (david.stavens@ai.stanford.edu)
 */
#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <time.h>


//Constants
static const double pi = 3.14159265358979323846;
static const int numFeatures = 400;

float xtot = 0.0f;
float ytot = 0.0f;

CvCapture* input_video;
IplImage* frame;
CvSize frame_size;

/* This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview"). */
CvSize optical_flow_window;

IplImage *lastFrame = NULL;
IplImage *lastFrameBackup = NULL;
IplImage *currentFrameBackup = NULL;
IplImage* eig_image = NULL; 
IplImage* temp_image = NULL;
IplImage* pyramid1 = NULL;
IplImage* pyramid2 = NULL;
IplImage* currentFrame = NULL;
bool demoMode = false; //Specifies whether video demo is enabled

inline static double square(int a)
{
	return a * a;
}

/* this can be bitwise optimized sign bit something etc */
int cmpFloat(const void* a, const void* b) {
    float x = *(float*)a - *(float*)b;
    if (x >= 0.0) {
        return 1;
    }
    return -1;
}

inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;

	*img = cvCreateImage( size, depth, channels );
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}

int opticalFlowInit(void)
{
    input_video = cvCreateCameraCapture(0);// assuming means USB0
    
    
	if (input_video == NULL)
	{
		/* Either the video didn't exist OR it uses a codec OpenCV
		 * doesn't support.
		 */
		return -1;
	}
	/* Read the video's frame size out of the AVI. */
	frame_size.height =
		(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_HEIGHT );
	frame_size.width =
		(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_WIDTH );
    
    
    optical_flow_window = cvSize(3,3);

    //Allocate needed frames
    allocateOnDemand(&lastFrame, frame_size, IPL_DEPTH_8U, 1);
    allocateOnDemand(&currentFrame, frame_size, IPL_DEPTH_8U, 1);
    allocateOnDemand(&eig_image, frame_size, IPL_DEPTH_32F, 1 );
	allocateOnDemand(&temp_image, frame_size, IPL_DEPTH_32F, 1 );
    allocateOnDemand(&pyramid1, frame_size, IPL_DEPTH_8U, 1 );
	allocateOnDemand(&pyramid2, frame_size, IPL_DEPTH_8U, 1 );
    
    //calculateOpticalFlow will assume currentFrame is present 
    IplImage* frame = cvQueryFrame(input_video);
    cvConvertImage(frame, lastFrame, 0);
    return 0;
}

int opticalFlowDemoFrameInit(void) {
	cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
    demoMode = true; 
    allocateOnDemand(&lastFrameBackup, frame_size, IPL_DEPTH_8U, 3);
    allocateOnDemand(&currentFrameBackup, frame_size, IPL_DEPTH_8U, 3);
    cvConvertImage(lastFrame,lastFrameBackup,0);
}

int calculateOpticalFlow(void) {
 
		/* Get the next frame of the video.
		 * IMPORTANT!  cvQueryFrame() always returns a pointer to the _same_
		 * memory location.  So successive calls:
		 * frame1 = cvQueryFrame();
		 * frame2 = cvQueryFrame();
		 * frame3 = cvQueryFrame();
		 * will result in (frame1 == frame2 && frame2 == frame3) being true.
		 * The solution is to make a copy of the cvQueryFrame() output.
		 */
    IplImage* frame = cvQueryFrame( input_video );
    if (frame == NULL)
	{
			/* Why did we get a NULL frame?  We shouldn't be at the end. */
		fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
		return -1;
	}
    
    cvConvertImage(frame, currentFrame, 0);
    
    if (demoMode) {
        cvConvertImage(frame,currentFrameBackup,0);    
    }

		/* Preparation: This array will contain the features found in frame 1. */

		/* Actually run the Shi and Tomasi algorithm!!
		 * "frame1_1C" is the input image.
		 * "eig_image" and "temp_image" are just workspace for the algorithm.
		 * The first ".01" specifies the minimum quality of the features (based on the eigenvalues).
		 * The second ".01" specifies the minimum Euclidean distance between features.
		 * "NULL" means use the entire input image.  You could point to a part of the image.
		 * WHEN THE ALGORITHM RETURNS:
		 * "frame1_features" will contain the feature points.
		 * "number_of_features" will be set to a value <= 400 indicating the number of feature points found.
		 */
    //Make temp variable since modified by cvGoodFeaturesToTrack
    int numFeaturesTemp = numFeatures;
    
    CvPoint2D32f lastFrameFeatures[numFeatures];	
	CvPoint2D32f currentFrameFeatures[numFeatures];
    
    cvGoodFeaturesToTrack(  lastFrame, 
                            eig_image, 
                            temp_image, 
                            lastFrameFeatures, 
                            &numFeaturesTemp, 
                            .01, /* Minimum quality of feature */ 
                            .01, /* Minimum euclidean distance */
                            NULL); /* NULL means use entire input */

		/* Pyramidal Lucas Kanade Optical Flow! */

		/* This array will contain the locations of the points from frame 1 in frame 2. */

		/* The i-th element of this array will be non-zero if and only if the i-th feature of
		 * frame 1 was found in frame 2.
		 */
	char optical_flow_found_feature[numFeatures];

		/* The i-th element of this array is the error in the optical flow for the i-th feature
		 * of frame1 as found in frame 2.  If the i-th feature was not found (see the array above)
		 * I think the i-th entry in this array is undefined.
		 */
	float optical_flow_feature_error[numFeatures];

		
		/* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
		 * epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
		 * work pretty well in many situations.
		 */
	CvTermCriteria optical_flow_termination_criteria  = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

		/* This is some workspace for the algorithm.
		 * (The algorithm actually carves the image into pyramids of different resolutions.)
		 */

		/* Actually run Pyramidal Lucas Kanade Optical Flow!!
		 * "frame1_1C" is the first frame with the known features.
		 * "frame2_1C" is the second frame where we want to find the first frame's features.
		 * "pyramid1" and "pyramid2" are workspace for the algorithm.
		 * "frame1_features" are the features from the first frame.
		 * "frame2_features" is the (outputted) locations of those features in the second frame.
		 * "number_of_features" is the number of features in the frame1_features array.
		 * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
		 * "5" is the maximum number of pyramids to use.  0 would be just one level.
		 * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
		 * "optical_flow_feature_error" is as described above (error in the flow for this feature).
		 * "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
		 * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
		 */
	cvCalcOpticalFlowPyrLK( lastFrame, 
                            currentFrame, 
                            pyramid1, 
                            pyramid2, 
                            lastFrameFeatures, 
                            currentFrameFeatures, 
                            numFeaturesTemp, 
                            optical_flow_window, 
                            5, 
                            optical_flow_found_feature, 
                            optical_flow_feature_error, 
                            optical_flow_termination_criteria, 
                            0 );

    printf("numFeatures: %d, numFeaturesTemp: %d\n",numFeatures,numFeaturesTemp);
     /*
      * Calculate all the vectors and put them in a table
      */
    float feature_vectors[2][numFeatures];
    int num_found_features = 0;
    for (int i = 0; i < numFeatures;i++) {
        if (optical_flow_found_feature[i] == 0) {//feature wasn't present on second image
            feature_vectors[0][i] = -1 * FLT_MAX;
            feature_vectors[1][i] = -1 * FLT_MAX;
        }
        else {
            feature_vectors[0][i] = currentFrameFeatures[i].x - lastFrameFeatures[i].x; //dx = frame2_x - frame1_x
            feature_vectors[1][i] = currentFrameFeatures[i].y - lastFrameFeatures[i].y; //dy = frame2_y - frame1_y 
            num_found_features++;
            printf("Floats?? %f %f \n",feature_vectors[0][i],feature_vectors[1][i]);
        }
    }
        //median-ing filter
    qsort(feature_vectors[0],numFeatures,sizeof(float),cmpFloat);
    qsort(feature_vectors[1],numFeatures,sizeof(float),cmpFloat);
        
    float medianX = feature_vectors[0][(2*numFeatures - num_found_features)/2];
    float medianY = feature_vectors[1][(2*numFeatures - num_found_features)/2];
    printf("medians are X: \n%f\n, Y: \n%f \n",medianX,medianY);
    printf("Absolutes are X: \n%f\n, Y:\n%f \n",xtot,ytot);
    xtot += medianX;
    ytot -= medianY;
           /* !!!!! This is the end of the useful algorithm, the rest is for visualization !!!!!! */
//#if 0
    if (demoMode) {
        //findHomography(frame1_features, frame2_features, 0, 3);

        //#if 0
		/* For fun (and debugging :)), let's draw the flow field. */
		for(int i = 0; i < numFeatures; i++)
		{
			/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
			if ( optical_flow_found_feature[i] == 0 )	continue;

			int line_thickness;				line_thickness = 1;
			/* CV_RGB(red, green, blue) is the red, green, and blue components
			 * of the color you want, each out of 255.
			 */	
			CvScalar line_color;			line_color = CV_RGB(255,0,0);
	
			/* Let's make the flow field look nice with arrows. */

			/* The arrows will be a bit too short for a nice visualization because of the high framerate
			 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
			 */
			CvPoint p,q;
			p.x = (int) lastFrameFeatures[i].x;
			p.y = (int) lastFrameFeatures[i].y;
			q.x = (int) currentFrameFeatures[i].x;
			q.y = (int) currentFrameFeatures[i].y;

			double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

			/* Here we lengthen the arrow by a factor of three. */
			q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
			q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

			/* Now we draw the main line of the arrow. */
			/* "frame1" is the frame to draw on.
			 * "p" is the point where the line begins.
			 * "q" is the point where the line stops.
			 * "CV_AA" means antialiased drawing.
			 * "0" means no fractional bits in the center cooridinate or radius.
			 */
			cvLine( lastFrameBackup, p, q, line_color, line_thickness, CV_AA, 0 );
			/* Now draw the tips of the arrow.  I do some scaling so that the
			 * tips look proportional to the main line of the arrow.
			 */			
			p.x = (int) (q.x + 9 * cos(angle + pi / 4));
			p.y = (int) (q.y + 9 * sin(angle + pi / 4));
			cvLine( lastFrameBackup, p, q, line_color, line_thickness, CV_AA, 0 );
			p.x = (int) (q.x + 9 * cos(angle - pi / 4));
			p.y = (int) (q.y + 9 * sin(angle - pi / 4));
			cvLine( lastFrameBackup, p, q, line_color, line_thickness, CV_AA, 0 );
		}
		/* Now display the image we drew on.  Recall that "Optical Flow" is the name of
		 * the window we created above.
		 */
		cvShowImage("Optical Flow", lastFrameBackup);
		/* And wait for the user to press a key (so the user has time to look at the image).
		 * If the argument is 0 then it waits forever otherwise it waits that number of milliseconds.
		 * The return value is the key the user pressed.
		 */
        int key_pressed;
	    key_pressed = cvWaitKey(1);
        lastFrameBackup = currentFrameBackup;
    }
    lastFrame = currentFrame;
}
