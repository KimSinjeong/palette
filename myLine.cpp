#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "myLine.h"

bool lineComparator(myLine* a, myLine* b) { return abs(a->rho) < abs(b->rho); }

void myLine_arr::insert(float rho, float theta) {

	if (theta > (CV_PI * 3 / 4)) {
		theta = theta - CV_PI;
		rho = -rho;
	}

	myLine* new_line = new myLine(rho, theta);
	if (new_line->theta > (CV_PI / 4)) {
		x_lines.push_back(new_line);
		//cout << "x lines : " << theta << endl;
	}

	else {
		y_lines.push_back(new_line);
		//cout << "y lines : " << theta << " / " << rho << endl;
	}
}



void myLine_arr::contourLines(vector < myLine* >& lines, const double rho_threshold, const double theta_threshold) {

	if (lines.size() == 0) return;
	std::sort(lines.begin(), lines.end(), lineComparator);

	vector < myLine* > new_lines;
	new_lines.push_back(lines.back());
	lines.pop_back();

	while (lines.size() != 0) {
		double disDiff = abs(abs((double)new_lines.back()->rho) - abs((double)lines.back()->rho));
		double slopeDiff = abs((double)new_lines.back()->theta - (double)lines.back()->theta);
		// (slopeDiff > CV_PI / 2) slopeDiff = CV_PI - slopeDiff;
		if (slopeDiff > CV_PI / 9) {
			//cout << "obstacles detected : slopeDiff "<< slopeDiff << " / slope : " << new_lines.back()->theta << " / " << lines.back()->theta << endl;
			break;
		}
		else if (disDiff < rho_threshold && slopeDiff < theta_threshold) {
			//else if (disDiff < rho_threshold ) {
			new_lines.back()->rho = (new_lines.back()->rho * new_lines.back()->index + lines.back()->rho) / (new_lines.back()->index + 1);
			new_lines.back()->theta = (new_lines.back()->theta * new_lines.back()->index + lines.back()->theta) / (new_lines.back()->index + 1);
			new_lines.back()->index++;
			lines.pop_back();
		}
		else {
			//if (abs(disDiff) > rho_threshold) cout << "rho    " << disDiff<<endl;
			//if (abs(slopeDiff) > theta_threshold) cout << "theta " << abs(slopeDiff)*180/CV_PI<<endl;
			new_lines.push_back(lines.back());
			lines.pop_back();
		}
	}

	while (new_lines.size() != 0) {
		lines.push_back(new_lines.back());
		new_lines.pop_back();
	}
}





void myLine_arr::XYcontour(const double rho_threshold, const double theta_threshold) {

	contourLines(this->y_lines, rho_threshold, theta_threshold);

	contourLines(this->x_lines, rho_threshold, theta_threshold);

}



int myLine_arr::hough_detection(cv::Mat frame) {

	//Mat c_frame = frame.clone();

	//Mat img_gray, img_dilate, dst;

	//cvtColor(c_frame, img_gray, COLOR_RGB2GRAY);

	//Canny(img_gray, dst, 50, 50, 3);

	Mat img_dilate, dst;

	Canny(frame, dst, 40, 150, 3);

	//....

	imshow("canny", dst);

	//....

	Mat mask = cv::getStructuringElement(cv::MORPH_RECT, Size(3, 3), Point(1, 1));

	dilate(dst, img_dilate, mask, Point(-1, -1), 2);

	erode(img_dilate, dst, mask, Point(-1, -1), 2);



	// Standard Hough Line Transform

	vector<Vec2f> lines; // will hold the results of the detection

	HoughLines(dst, lines, 1, CV_PI / 180, 200); // runs the actual detection



	while (lines.size() != 0) {

		this->insert(lines.back()[0], lines.back()[1]);

		lines.pop_back();

	}

	//this->drawLines(frame);
	//imshow("gridsample", frame);

	double rho_threshold = 5;

	double theta_threshold = 20 * CV_PI / 180;

	this->XYcontour(rho_threshold, theta_threshold);


	if (this->x_lines.size() == 12 && this->y_lines.size() == 12) return 1;

	return 0;

}





void myLine_arr::drawLines(Mat& frame) {

	double a, b, _x, _y;

	Point pt1, pt2;

	for (int i = 0; i < this->x_lines.size(); i++)

	{

		a = cos((double)this->x_lines.at(i)->theta), b = sin((double)this->x_lines.at(i)->theta);

		_x = a * this->x_lines.at(i)->rho, _y = b * this->x_lines.at(i)->rho;

		pt1.x = cvRound(_x + 1000 * (-b));

		pt1.y = cvRound(_y + 1000 * (a));

		pt2.x = cvRound(_x - 1000 * (-b));

		pt2.y = cvRound(_y - 1000 * (a));

		line(frame, pt1, pt2, Scalar(255, 0, 0), 1, LINE_AA);
	}
	for (int i = 0; i < this->y_lines.size(); i++)
	{
		a = cos((double)this->y_lines.at(i)->theta), b = sin((double)this->y_lines.at(i)->theta);
		_x = a * this->y_lines.at(i)->rho, _y = b * this->y_lines.at(i)->rho;
		pt1.x = cvRound(_x + 1000 * (-b));
		pt1.y = cvRound(_y + 1000 * (a));
		pt2.x = cvRound(_x - 1000 * (-b));
		pt2.y = cvRound(_y - 1000 * (a));
		line(frame, pt1, pt2, Scalar(0, 255, 0), 1, LINE_AA);
	}
}

void myLine_arr::removeLines() {
	while (this->x_lines.size()) {
		delete this->x_lines.back();
		this->x_lines.pop_back();
	}
	while (this->y_lines.size()) {
		delete this->y_lines.back();
		this->y_lines.pop_back();
	}
}