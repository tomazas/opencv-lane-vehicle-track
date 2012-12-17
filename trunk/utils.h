#pragma once

//char buf[256];
//sprintf(buf, "%d", k);
//cvPutText(temp_frame, buf , cvPoint(right[k].p0.x+5, right[k].p0.y-5), &font, CV_RGB(0,0,255));

#include <list>

// moving average
// check: http://en.wikipedia.org/wiki/Moving_average
class MA { 
public:

	enum {
		MAX_MA_SIZE = 10,
	};

	MA():average(0),n(0),sum(0){}

	// exponential alpha = 1/N moving average of all time range
	void add_exp(float value) { 
		n++;
		average = ((n-1) * average + value ) / n;
	}

	// cummulative average of all time range
	void add_cummul(float value) { 
		n++;
		average = average + (value - average)/n;
	}

	void add(float value) {
		if (n < MAX_MA_SIZE) {
			values.push_back(value);
			sum += value;
			n++;
			average = sum/n;
		} else {
			float old = values[0]/n;
			values.erase(values.begin());
			average = average - old + value/n;
			values.push_back(value);
		}
	}

	void clear(bool zeroAvrg = false) {
		values.clear();
		n = 0;
		sum = 0;
		if (zeroAvrg) average = 0;
	}

	std::vector<float> values;
	
	float average;
	float sum;
	int n;
};

CvPoint2D32f sub(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x-a.x, b.y-a.y); }
CvPoint2D32f mul(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x*a.x, b.y*a.y); }
CvPoint2D32f add(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x+a.x, b.y+a.y); }
CvPoint2D32f mul(CvPoint2D32f b, float t) { return cvPoint2D32f(b.x*t, b.y*t); }
float dot(CvPoint2D32f a, CvPoint2D32f b) { return (b.x*a.x + b.y*a.y); }
float dist(CvPoint2D32f v) { return sqrtf(v.x*v.x + v.y*v.y); }


CvPoint2D32f point_on_segment(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt){
	CvPoint2D32f v = sub(pt, line0);
	CvPoint2D32f dir = sub(line1, line0);
	float len = dist(dir);
	float inv = 1.0f/(len+1e-6f);
	dir.x *= inv;
	dir.y *= inv;

	float t = dot(dir, v);
	if(t >= len) return line1;
	else if(t <= 0) return line0;

	return add(line0, mul(dir,t));
}

float dist2line(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt){
	return dist(sub(point_on_segment(line0, line1, pt), pt));
}

/*
int determineDominantLine (std::vector<Ptx>& right, Ptx* prev) {

	int max = 1;

	for(int i=0; i<right.size(); i++) {

		//if (fabs(prev.angle - right[i].angle) <= 5.0f) {
		//	right[i].votes += 10;
		//}

		if (right[i].visited) continue;

		for(int j=i+1; j<right.size(); j++) {
			if (right[j].visited) continue;

			if (fabs(right[i].angle - right[j].angle) <= 5.0f) { // parallel
					
				right[i].votes += 1;
				right[j].visited = true;

				if (right[i].votes > max) {
					max = right[i].votes;
				}
			}
		}
	}

	return max;
}

*/
