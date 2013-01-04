
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

struct Vehicle {
	CvPoint bmin, bmax;
	int symmetryX;
	bool valid;
	unsigned int lastUpdate;
};

struct VehicleSample {
	CvPoint center;
	float radi;
	unsigned int frameDetected;
	int vehicleIndex;
};

#define GREEN CV_RGB(0,255,0)
#define RED CV_RGB(255,0,0)
#define PURPLE CV_RGB(255,0,255)

Status laneR, laneL;
std::vector<Vehicle> vehicles;
std::vector<VehicleSample> samples;

enum{
    SCAN_STEP = 5,			 // in pixels
	LINE_REJECT_DEGREES = 5, // in degrees
    BW_TRESHOLD = 250,		 // edge response strength to recognize for 'WHITE'
    BORDERX = 10,			 // px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,	 // px
	
	CANNY_MIN_TRESHOLD = 1,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 100, // edge detector maximum hysteresis threshold

	HOUGH_TRESHOLD = 50,
	HOUGH_MIN_LINE_LENGTH = 10,	// join line with smaller than this gaps
	HOUGH_MAX_LINE_GAP = 100,

	CAR_DETECT_LINES = 4,   // minimum lines for a region to pass validation as a 'CAR'
	CAR_H_LINE_LENGTH = 10,  // minimum horizontal line length from car body in px

	MAX_VEHICLE_SAMPLES = 30,      // max vehicle detection sampling history
	CAR_DETECT_POSITIVE_SAMPLES = MAX_VEHICLE_SAMPLES-2, //
	MAX_VEHICLE_NO_UPDATE_FREQ = 15 // in frames
};

#define K_VARY_FACTOR 0.2f
#define B_VARY_FACTOR 20
#define MAX_LOST_FRAMES 10

void FindResponses(IplImage *img, int startX, int endX, int y, std::vector<int>& list)
{
    // scans for single response: /^\_

	const int row = y * img->width * img->nChannels;
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

unsigned char pixel(IplImage* img, int x, int y) {
	return (unsigned char)img->imageData[(y*img->width+x)*img->nChannels];
}

int findSymmetryAxisX(IplImage* half_frame, CvPoint bmin, CvPoint bmax) {
  
  float value = 0;
  int axisX = -1; // not found
  
  int xmin = bmin.x;
  int ymin = bmin.y;
  int xmax = bmax.x;
  int ymax = bmax.y;
  int half_width = half_frame->width/2;
  int maxi = 1;

  for(int x=xmin, j=0; x<xmax; x++, j++) {
	float HS = 0;
    for(int y=ymin; y<ymax; y++) {
		int row = y*half_frame->width*half_frame->nChannels;
        for(int step=1; step<half_width; step++) {
          int neg = x-step;
          int pos = x+step;
		  unsigned char Gneg = (neg < xmin) ? 0 : (unsigned char)half_frame->imageData[row+neg*half_frame->nChannels];
          unsigned char Gpos = (pos >= xmax) ? 0 : (unsigned char)half_frame->imageData[row+pos*half_frame->nChannels];
          HS += abs(Gneg-Gpos);
        }
    }

	if (axisX == -1 || value > HS) { // find minimum
		axisX = x;
		value = HS;
	}
  }

  return axisX;
}

bool hasVertResponse(IplImage* edges, int x, int y, int ymin, int ymax) {
	bool has = (pixel(edges, x, y) > BW_TRESHOLD);
	if (y-1 >= ymin) has &= (pixel(edges, x, y-1) < BW_TRESHOLD);
	if (y+1 < ymax) has &= (pixel(edges, x, y+1) < BW_TRESHOLD);
	return has;
}

int horizLine(IplImage* edges, int x, int y, CvPoint bmin, CvPoint bmax, int maxHorzGap) {

	// scan to right
	int right = 0;
	int gap = maxHorzGap;
	for (int xx=x; xx<bmax.x; xx++) {
		if (hasVertResponse(edges, xx, y, bmin.y, bmax.y)) {
			right++;
			gap = maxHorzGap; // reset
		} else {
			gap--;
			if (gap <= 0) {
				break;
			}
		}
	}

	int left = 0;
	gap = maxHorzGap;
	for (int xx=x-1; xx>=bmin.x; xx--) {
		if (hasVertResponse(edges, xx, y, bmin.y, bmax.y)) {
			left++;
			gap = maxHorzGap; // reset
		} else {
			gap--;
			if (gap <= 0) {
				break;
			}
		}
	}

	return left+right;
}

bool vehicleValid(IplImage* half_frame, IplImage* edges, Vehicle* v, int& index) {

	index = -1;

	// first step: find horizontal symmetry axis
	v->symmetryX = findSymmetryAxisX(half_frame, v->bmin, v->bmax);
	if (v->symmetryX == -1) return false;

	// second step: cars tend to have a lot of horizontal lines
	int hlines = 0;
	for (int y = v->bmin.y; y < v->bmax.y; y++) {		
		if (horizLine(edges, v->symmetryX, y, v->bmin, v->bmax, 2) > CAR_H_LINE_LENGTH) {
#if _DEBUG
			cvCircle(half_frame, cvPoint(v->symmetryX, y), 2, PURPLE);
#endif
			hlines++;
		}
	}

	int midy = (v->bmax.y + v->bmin.y)/2;

	// third step: check with previous detected samples if car already exists
	int numClose = 0;
	float closestDist = 0;
	for (int i = 0; i < samples.size(); i++) {
		int dx = samples[i].center.x - v->symmetryX;
		int dy = samples[i].center.y - midy;
		float Rsqr = dx*dx + dy*dy;
		
		if (Rsqr <= samples[i].radi*samples[i].radi) {
			numClose++;
			if (index == -1 || Rsqr < closestDist) {
				index = samples[i].vehicleIndex;
				closestDist = Rsqr;
			}
		}
	}

	return (hlines >= CAR_DETECT_LINES || numClose >= CAR_DETECT_POSITIVE_SAMPLES);
}

void removeOldVehicleSamples(unsigned int currentFrame) {
	// statistical sampling - clear very old samples
	std::vector<VehicleSample> sampl;
	for (int i = 0; i < samples.size(); i++) {
		if (currentFrame - samples[i].frameDetected < MAX_VEHICLE_SAMPLES) {
			sampl.push_back(samples[i]);
		}
	}
	samples = sampl;
}

void removeSamplesByIndex(int index) {
	// statistical sampling - clear very old samples
	std::vector<VehicleSample> sampl;
	for (int i = 0; i < samples.size(); i++) {
		if (samples[i].vehicleIndex != index) {
			sampl.push_back(samples[i]);
		}
	}
	samples = sampl;
}

void removeLostVehicles(unsigned int currentFrame) {
	// remove old unknown/false vehicles & their samples, if any
	for (int i=0; i<vehicles.size(); i++) {
		if (vehicles[i].valid && currentFrame - vehicles[i].lastUpdate >= MAX_VEHICLE_NO_UPDATE_FREQ) {
			printf("\tremoving inactive car, index = %d\n", i);
			removeSamplesByIndex(i);
			vehicles[i].valid = false;
		}
	}
}

void vehicleDetection(IplImage* half_frame, CvHaarClassifierCascade* cascade, CvMemStorage* haarStorage) {

	static unsigned int frame = 0;
	frame++;
	printf("*** vehicle detector frame: %d ***\n", frame);

	removeOldVehicleSamples(frame);

	// Haar Car detection
	const double scale_factor = 1.05; // every iteration increases scan window by 5%
	const int min_neighbours = 2; // minus 1, number of rectangles, that the object consists of
	CvSeq* rects = cvHaarDetectObjects(half_frame, cascade, haarStorage, scale_factor, min_neighbours, CV_HAAR_DO_CANNY_PRUNING);

	// Canny edge detection of the minimized frame
	if (rects->total > 0) {
		printf("\thaar detected %d car hypotheses\n", rects->total);
		IplImage *edges = cvCreateImage(cvSize(half_frame->width, half_frame->height), IPL_DEPTH_8U, 1);
		cvCanny(half_frame, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);

		/* validate vehicles */
		for (int i = 0; i < rects->total; i++) {
			CvRect* rc = (CvRect*)cvGetSeqElem(rects, i);
			
			Vehicle v;
			v.bmin = cvPoint(rc->x, rc->y);
			v.bmax = cvPoint(rc->x + rc->width, rc->y + rc->height);
			v.valid = true;

			int index;
			if (vehicleValid(half_frame, edges, &v, index)) { // put a sample on that position
				
				if (index == -1) { // new car detected

					v.lastUpdate = frame;

					// re-use already created but inactive vehicles
					for(int j=0; j<vehicles.size(); j++) {
						if (vehicles[j].valid == false) {
							index = j;
							break;
						}
					}
					if (index == -1) { // all space used
						index = vehicles.size();
						vehicles.push_back(v);
					}
					printf("\tnew car detected, index = %d\n", index);
				} else {
					// update the position from new data
					vehicles[index] = v;
					vehicles[index].lastUpdate = frame;
					printf("\tcar updated, index = %d\n", index);
				}

				VehicleSample vs;
				vs.frameDetected = frame;
				vs.vehicleIndex = index;
				vs.radi = (MAX(rc->width, rc->height))/4; // radius twice smaller - prevent false positives
				vs.center = cvPoint((v.bmin.x+v.bmax.x)/2, (v.bmin.y+v.bmax.y)/2);
				samples.push_back(vs);
			}
		}

		cvShowImage("Half-frame[edges]", edges);
		cvMoveWindow("Half-frame[edges]", half_frame->width*2+10, half_frame->height); 
		cvReleaseImage(&edges);
	} else {
		printf("\tno vehicles detected in current frame!\n");
	}

	removeLostVehicles(frame);

	printf("\ttotal vehicles on screen: %d\n", vehicles.size());
}

void drawVehicles(IplImage* half_frame) {

	// show vehicles
	for (int i = 0; i < vehicles.size(); i++) {
		Vehicle* v = &vehicles[i];
		if (v->valid) {
			cvRectangle(half_frame, v->bmin, v->bmax, GREEN, 1);
			
			int midY = (v->bmin.y + v->bmax.y) / 2;
			cvLine(half_frame, cvPoint(v->symmetryX, midY-10), cvPoint(v->symmetryX, midY+10), PURPLE);
		}
	}

	// show vehicle position sampling
	/*for (int i = 0; i < samples.size(); i++) {
		cvCircle(half_frame, cvPoint(samples[i].center.x, samples[i].center.y), samples[i].radi, RED);
	}*/
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

		/*for (int i=0; i<rsp.size(); i++) {
			CvPoint p = cvPoint(rsp[i], y);
			bool found = false;
		}	*/

		if (rsp.size() > 0) {
			CvPoint p = cvPoint(rsp[0], y);

			for (int j=0; j<lanes.size(); j++) {
				if (!lanes[j].box.contains(p)) continue;
				float d = dist2line(cvPoint2D32f(lanes[j].p0.x, lanes[j].p0.y), 
						cvPoint2D32f(lanes[j].p1.x, lanes[j].p1.y), 
						cvPoint2D32f(p.x, p.y));

				if (d <= MAX_RESPONSE_DIST) {
					//right[j].votes++;
					//found = true;
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

		// assume that vanishing point is close to the image horizontal center
		//y = kx + b;
		float k = dy/(float)dx;
		float b = line[0].y - k*line[0].x;
		float x  = -b/k;

		const float w2 = temp_frame->width/2;
		//TODO: does not work always (e.g. when the car is changing lanes!!! HT lines get lost!)
		if (fabs(w2 - x) >= 50) {
			continue;
		}

		// assign lane's side based by its midpoint position 
		int midx = (line[0].x + line[1].x) / 2;
		CvScalar s;
			
		if (midx < temp_frame->width/2) {
			s = CV_RGB(255,0,0);
			left.push_back(Lane(line[0], line[1], angle));
		} else if (midx > temp_frame->width/2) {
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
		
		// process vehicles
		vehicleDetection(half_frame, cascade, haarStorage);
		drawVehicles(half_frame);
		cvShowImage("Half-frame", half_frame);
		cvMoveWindow("Half-frame", half_frame->width*2+10, 0); 

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