#ifndef LINE
#define LINE
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class myLine {
public:
	float rho;
	float theta;
	int index;
	myLine(float _rho, float _theta) {
		this->rho = _rho; this->theta = _theta; index = 1;
	}
};

class myLine_arr {
public:
	vector <myLine*> x_lines;
	vector <myLine*> y_lines;
	void insert(float rho, float theta);
	void contourLines(vector < myLine* >& lines, const double rho_threshold, const double theta_threshold);
	void XYcontour(const double rho_threshold, const double theta_threshold);
	int hough_detection(cv::Mat frame);
	void drawLines(Mat& frame);
	void removeLines();
	~myLine_arr() {
		while (this->x_lines.size()) {
			delete this->x_lines.back();
			this->x_lines.pop_back();
		}
		while (this->y_lines.size()) {
			delete this->y_lines.back();
			this->y_lines.pop_back();
		}
	}
};

#endif