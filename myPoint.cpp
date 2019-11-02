#include "myPoint.h"
#include<iostream>

using namespace std;
using namespace cv;

void arr_of_points(Mat frame, myLine_arr& lines_comb, myPoint points[][12]) {
	float a1, a2, b1, b2;
	//solve the point of intersection of two lines
	for (size_t i = 0; i < lines_comb.x_lines.size(); i++)
	{
		float rho_i = lines_comb.x_lines.at(i)->rho, theta_i = lines_comb.x_lines.at(i)->theta;
		double a = cos(theta_i), b = sin(theta_i);
		double x0_i = a * rho_i, y0_i = b * rho_i;


		a1 = -x0_i / y0_i;
		b1 = y0_i - a1 * x0_i;

		for (int j = 0; j < lines_comb.y_lines.size(); j++) {
			float rho_j = lines_comb.y_lines.at(j)->rho, theta_j = lines_comb.y_lines.at(j)->theta;
			double a = cos(theta_j), b = sin(theta_j);
			double x0_j = a * rho_j, y0_j = b * rho_j;

			a2 = -x0_j / y0_j;
			b2 = y0_j - a2 * x0_j;
			points[i][j].x = (b2 - b1) / (a1 - a2);
			points[i][j].y = a1 * points[i][j].x + b1;
			//circle(frame, Point(points[i][j].x, points[i][j].y), 5, Scalar(0, 0, 255));
		}
	}
}

void new_arr_of_points(Mat frame, myPoint points[][12], Mat pap_pix2pap_real, Point2f p_origin_pixel) {
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 12; j++) {
			//Mat p_2 [2][1]= { p.x,p.y };
			Mat p_paper = (Mat_<double>(2, 1) << points[i][j].paper_x, points[i][j].paper_y);
			//cout << p_2 << endl;
			Mat p_pixel = pap_pix2pap_real.inv() * p_paper;
			points[i][j].x = p_origin_pixel.x + p_pixel.at<double>(0);
			points[i][j].y = p_origin_pixel.y + p_pixel.at<double>(1);
			//cout << points[i][j].x <<points[i][j].y << endl;
			//circle(frame, Point(points[i][j].x, points[i][j].y), 5, Scalar(0, 0, 255));
		}
	}
}

void drawDots(Mat frame, myPoint points[][12]) {
	for (size_t i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			circle(frame, Point(points[i][j].x, points[i][j].y), 5, Scalar(0, 0, 255));
		}
	}
	imshow("points", frame);
}

void blobDetection(Mat frame, vector<KeyPoint>& blobPoints) {
	SimpleBlobDetector::Params params;
	params.minThreshold = 100;
	//params.maxThreshold = 220;
	params.filterByArea = true;
	params.minArea = 40;
	params.maxArea = 300;
	params.filterByColor = true;
	params.blobColor = 0; // ¾îµÎ¿î ¾ó·è ÃßÃâ : 0, ¹àÀº ¾ó·è ÃßÃâ : 255
	params.filterByCircularity = false;
	//params.minCircularity = 5;
	//params.filterByCircularity = 5;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	detector->detect(frame, blobPoints);
}

int find_dot_index(myPoint points[][12], KeyPoint blob) {
	int min_distance = 0;
	int index = 0;
	float dx, dy, distance;
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			dx = blob.pt.x - points[i][j].x;
			dy = blob.pt.y - points[i][j].y;
			distance = sqrt(pow(dx, 2) + pow(dy, 2));
			if (distance < blob.size) return i * 12 + j;
		}
	}
	return OUTOFDOTS;
}

int set_dot_state(Mat frame, myPoint& point) {
	if (point.state == EMPTY) {
		point.state = RED;
		point.weight = 1;
		return 1;
	}
	return 0;
}


int dotDetection(Mat frame, myPoint points[][12]) {
	vector<KeyPoint> blobPoints;

	Mat img_erode, img_dilate, img_higher_contrast;
	Mat mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));
	erode(frame, img_erode, mask, cv::Point(-1, -1), 1);
	dilate(img_erode, img_dilate, mask, cv::Point(-1, -1), 2);
	img_dilate.convertTo(img_higher_contrast, -1, 2, 0);

	blobDetection(img_higher_contrast, blobPoints);
	drawKeypoints(img_higher_contrast, blobPoints, frame, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imshow("keypoints", frame);
	int index = OUTOFDOTS;
	int x, y;
	for (int i = 0; i < blobPoints.size(); i++) {
		index = find_dot_index(points, blobPoints.at(i));
		if (index != OUTOFDOTS) {
			x = index / 12;
			y = index % 12;
			if (set_dot_state(img_higher_contrast, points[x][y])) {
				return 1;
			}
			//else cout << i << " the point ("<<x << ", " << y << ") is not empty" << endl;
		}
		//else cout << i << " not a point" << endl;
	}
	return 0;
}

void to_state_arr(myPoint(*points)[12], int(*state)[12]) {
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			state[i][j] = points[i][j].state;
		}
	}
}

void to_point_arr(int(*state)[12], myPoint(*points)[12]) {
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			points[i][j].state = state[i][j];
		}
	}
}
