
#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2\opencv.hpp>
#include <math.h>
#include "utils.h"

#define USE_VIDEO 1

#undef MIN
#undef MAX
#define MAX(a,b) ((a)<(b)?(b):(a))
#define MIN(a,b) ((a)>(b)?(b):(a))

void crop(IplImage* src,  IplImage* dest, CvRect rect) {
    cvSetImageROI(src, rect); 
    cvCopy(src, dest); 
    cvResetImageROI(src); 
}

struct BBox {
	BBox():xmin(999999),xmax(-999999),ymin(999999),ymax(-999999){}
	bool contains(CvPoint p) {
		return (p.x > xmin && p.x < xmax && p.y > ymin && p.y < ymax);
	}
	int xmin, xmax, ymin, ymax;
};

struct Lane {
	Lane(){}
	Lane(CvPoint a, CvPoint b, float angle): p0(a),p1(b),angle(angle),
		votes(0),visited(false),found(false)
	{
		// compute bbox
		box.xmin = MIN(MIN(box.xmin, p0.x), p1.x);
		box.xmax = MAX(MAX(box.xmax, p0.x), p1.x);
		box.ymin = MIN(MIN(box.ymin, p0.y), p1.y);
		box.ymax = MAX(MAX(box.ymax, p0.y), p1.y);
	}

	CvPoint p0, p1;
	BBox box;
	int votes;
	bool visited, found;
	float angle;
};

struct Status {
	Status():reset(true),lost(0){}
	MA k, b;
	bool reset;
	int lost;
};

Status laneR, laneL;

enum{
    SCAN_STEP = 5,			 // in pixels
	LINE_REJECT_DEGREES = 5, // in degrees
    BW_TRESHOLD = 250,		 // edge response strength to recognize for 'WHITE'
    BORDERX = 10,			 // px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,	 // px
	
	CANNY_MIN_TRESHOLD = 1,
	CANNY_MAX_TRESHOLD = 100,

	HOUGH_TRESHOLD = 50,
	HOUGH_MIN_LINE_LENGTH = 10,
	HOUGH_MAX_LINE_GAP = 100,
};

#define K_VARY_FACTOR 0.2f
#define B_VARY_FACTOR 20
#define MAX_LOST_FRAMES 10

void FindResponses(IplImage *img, int startX, int endX, int y, std::vector<int>& list)
{
    // scans for single response: /^\_

	const int row = y * img->width * img->depth/8;
	unsigned char* ptr = (unsigned char*)img->imageData;

    int step = (endX < startX) ? -1: 1;
    int range = (endX > startX) ? endX-startX+1 : startX-endX+1;

    for(int x = startX; range>0; x += step, range--)
    {
        if(ptr[row + x] <= BW_TRESHOLD) continue; // skip black: loop until white pixels show up

        // first response found
        int idx = x + step;

        // skip same response(white) pixels
        while(range > 0 && ptr[row+idx] > BW_TRESHOLD){
            idx += step;
            range--;
        }

		// reached black again
        if(ptr[row+idx] <= BW_TRESHOLD) {
            list.push_back(x);
        }

        x = idx; // begin from new pos
    }
}

void vehicleDetection(IplImage* half_frame, CvHaarClassifierCascade* cascade, CvMemStorage* haarStorage) {

	// Haar Car detection
	double scale = 1;
	double scale_factor = 1.05; // every iteration increases scan window by 5%
	int min_neighbours = 2; // minus 1, number of rectangles, that the object consists of
	CvSeq* cars = cvHaarDetectObjects(half_frame, cascade, haarStorage, scale_factor, min_neighbours, CV_HAAR_DO_CANNY_PRUNING);

	/* draw all the rectangles */
	for (int i = 0; i < cars->total; i++) {
		CvRect rc = *(CvRect*)cvGetSeqElem(cars, i);
		cvRectangle(half_frame, 
					cvPoint(rc.x*scale, rc.y*scale),
						cvPoint((rc.x+rc.width)*scale, (rc.y+rc.height)*scale),
						CV_RGB(0,255,0), 2);
	}

	cvShowImage("Half-frame", half_frame);
	cvMoveWindow("Half-frame", half_frame->width*2+10, 0); 
}

void processSide(std::vector<Lane> lanes, IplImage *edges, bool right) {

	Status* side = right ? &laneR : &laneL;

	// response search
	int w = edges->width;
	int h = edges->height;
	const int BEGINY = 0;
	const int ENDY = h-1;
	const int ENDX = right ? (w-BORDERX) : BORDERX;
	int midx = w/2;
	unsigned char* ptr = (unsigned char*)edges->imageData;

	// show responses
	int first_lane = -1;

	for(int y=ENDY; y>=BEGINY; y-=SCAN_STEP) {
		std::vector<int> rsp;
		FindResponses(edges, midx, ENDX, y, rsp);

		for (int i=0; i<rsp.size(); i++) {
			CvPoint p = cvPoint(rsp[i], y);
			bool found = false;
			
			for (int j=0; j<lanes.size(); j++) {
				if (!lanes[j].box.contains(p)) continue;

				float d = dist2line(cvPoint2D32f(lanes[j].p0.x, lanes[j].p0.y), 
						cvPoint2D32f(lanes[j].p1.x, lanes[j].p1.y), 
						cvPoint2D32f(p.x, p.y));

				if (d <= MAX_RESPONSE_DIST) {
					//right[j].votes++;
					found = true;
					if (first_lane == -1) {
						first_lane = j;
					}
					break;
				}
			}

			/*
			if (found) {
				cvCircle(temp_frame, p, 2, CV_RGB(0,255,0), 2);
			}*/
		}	
	}

	if (first_lane != -1) {
		Lane* best = &lanes[first_lane];
		
		float dy = best->p1.y - best->p0.y;
		float dx = best->p1.x - best->p0.x;
		float k = dy/dx;//tanf(atan2(dy,dx));
		float b = best->p0.y - k*best->p0.x;

		float k_diff = fabs(k - side->k.average);
		float b_diff = fabs(b - side->b.average);

		bool update_ok = (k_diff <= K_VARY_FACTOR && b_diff <= B_VARY_FACTOR) || side->reset;

		printf("side: %s, k vary: %.4f, b vary: %.4f, lost: %s\n", 
			(right?"RIGHT":"LEFT"), k_diff, b_diff, (update_ok?"no":"yes"));
		
		if (update_ok) {
			// update is in valid bounds
			side->k.add(k);
			side->b.add(b);
			side->reset = false;
			side->lost = 0;
		} else {
			// can't update, lanes flicker periodically, start counter for partial reset!
			side->lost++;
			if (side->lost >= MAX_LOST_FRAMES && !side->reset) {
				side->reset = true;
			}
		}

	} else {
		printf("no lanes detected - lane tracking lost! counter increased\n");
		side->lost++;
		if (side->lost >= MAX_LOST_FRAMES && !side->reset) {
			// do full reset when lost for more than N frames
			side->reset = true;
			side->k.clear();
			side->b.clear();
		}
	}
}

void processLanes(CvSeq* lines, IplImage* edges, IplImage* temp_frame) {

	// classify lines to left/right side
	std::vector<Lane> left, right;

	for(int i = 0; i < lines->total; i++ )
    {
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
		int dx = line[1].x - line[0].x;
		int dy = line[1].y - line[0].y;
		float angle = atan2f(dy, dx) * 180/CV_PI;

		if (fabs(angle) <= LINE_REJECT_DEGREES) { // reject near horizontal lines
			continue;
		}
		
		int midx = (line[0].x + line[1].x) / 2;
		CvScalar s;
			
		if (midx < temp_frame->width/2 && angle < 0) {
			s = CV_RGB(255,0,0);
			left.push_back(Lane(line[0], line[1], angle));
		} else if (midx > temp_frame->width/2 && angle > 0) {
			s = CV_RGB(0,0,255);
			right.push_back(Lane(line[0], line[1], angle));
		}
    }

	// show Hough lines
	for	(int i=0; i<right.size(); i++) {
		cvLine(temp_frame, right[i].p0, right[i].p1, CV_RGB(0, 0, 255), 2);
	}

	for	(int i=0; i<left.size(); i++) {
		cvLine(temp_frame, left[i].p0, left[i].p1, CV_RGB(255, 0, 0), 2);
	}

	processSide(left, edges, false);
	processSide(right, edges, true);

	// show computed lanes
	int x = temp_frame->width * 0.55f;
	int x2 = temp_frame->width;
	cvLine(temp_frame, cvPoint(x, laneR.k.average*x + laneR.b.average), 
		cvPoint(x2, laneR.k.average * x2 + laneR.b.average), CV_RGB(255, 0, 255), 2);

	x = temp_frame->width * 0;
	x2 = temp_frame->width * 0.45f;
	cvLine(temp_frame, cvPoint(x, laneL.k.average*x + laneL.b.average), 
		cvPoint(x2, laneL.k.average * x2 + laneL.b.average), CV_RGB(255, 0, 255), 2);
}

int main(void)
{

#ifdef USE_VIDEO
	CvCapture *input_video = cvCreateFileCapture("road.avi");
#else
	CvCapture *input_video = cvCaptureFromCAM(0);
#endif

	if (input_video == NULL) {
		fprintf(stderr, "Error: Can't open video\n");
		return -1;
	}

	CvFont font;
	cvInitFont( &font, CV_FONT_VECTOR0, 0.25f, 0.25f);

	CvSize video_size;
	video_size.height = (int) cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_HEIGHT);
	video_size.width = (int) cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_WIDTH);

	/// Create a Trackbar for user to enter threshold
	//createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);
	
	long current_frame = 0;
	int key_pressed = 0;
	IplImage *frame = NULL;

	CvSize frame_size = cvSize(video_size.width, video_size.height/2);
	IplImage *temp_frame = cvCreateImage(frame_size, IPL_DEPTH_8U, 3);
	IplImage *grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *half_frame = cvCreateImage(cvSize(video_size.width/2, video_size.height/2), IPL_DEPTH_8U, 3);

	CvMemStorage* houghStorage = cvCreateMemStorage(0);
	CvMemStorage* haarStorage = cvCreateMemStorage(0);
	CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*)cvLoad("haar/cars3.xml");

	//cvSetCaptureProperty(input_video, CV_CAP_PROP_POS_FRAMES, current_frame);
	while(key_pressed != 27) {

		frame = cvQueryFrame(input_video);
		if (frame == NULL) {
			fprintf(stderr, "Error: null frame received\n");
			return -1;
		}

		cvPyrDown(frame, half_frame, CV_GAUSSIAN_5x5); // Reduce the image by 2	 
		//cvCvtColor(temp_frame, grey, CV_BGR2GRAY); // convert to grayscale

		// we're interested only in road below horizont - so crop top image portion off
		crop(frame, temp_frame, cvRect(0,frame_size.height,frame_size.width,frame_size.height));
		cvCvtColor(temp_frame, grey, CV_BGR2GRAY); // convert to grayscale
		
		// Perform a Gaussian blur ( Convolving with 5 X 5 Gaussian) & detect edges
		cvSmooth(grey, grey, CV_GAUSSIAN, 5, 5);
		cvCanny(grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);

		// do Hough transform to find lanes
		double rho = 1;
		double theta = CV_PI/180;
		CvSeq* lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC, 
			rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);

		processLanes(lines, edges, temp_frame);
		vehicleDetection(half_frame, cascade, haarStorage);

		// show middle line
		cvLine(temp_frame, cvPoint(frame_size.width/2,0), 
			cvPoint(frame_size.width/2,frame_size.height), CV_RGB(255, 255, 0), 1);

		cvShowImage("Grey", grey);
		cvShowImage("Edges", edges);
		cvShowImage("Color", temp_frame);
		
		cvMoveWindow("Grey", 0, 0); 
		cvMoveWindow("Edges", 0, frame_size.height+25);
		cvMoveWindow("Color", 0, 2*(frame_size.height+25)); 

		key_pressed = cvWaitKey(5);
	}

	cvReleaseHaarClassifierCascade(&cascade);
	cvReleaseMemStorage(&haarStorage);
	cvReleaseMemStorage(&houghStorage);

	cvReleaseImage(&grey);
	cvReleaseImage(&edges);
	cvReleaseImage(&temp_frame);
	cvReleaseImage(&half_frame);

	cvReleaseCapture(&input_video);
}