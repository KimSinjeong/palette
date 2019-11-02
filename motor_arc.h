#ifndef _MOTOR
#define _MOTOR

#include "myPoint.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>



using namespace std;
using namespace cv;



vector<int> joint_arc(myPoint p) {// in mm
	//needed to 0 <= joint <= 4096 (XH, XM) or 0 <= joint <= 1024 (AX)
	int d1 = 120.75 + 5.2, L2 = 201.5, L3 = 201.5, e1 = 37.5, h1 = 45;//  e1: end effector to end of 3rd joint, h1 : floor to pen center

	int p_x, p_y, p_z;
	int q1, q2, q3;
	// q1
	if (p.robot_x > 0)
		q1 = atan(p.robot_y / p.robot_x);
	else if (p.robot_x < 0)
		q1 = atan(p.robot_y / p.robot_x) + CV_PI;
	else
		if (p.robot_y > 0)
			q1 = CV_PI / 2;
		else
			q1 = -CV_PI / 2;

	p_x = p.robot_x - e1 * cos(q1);
	p_y = p.robot_y - e1 * sin(q1);
	p_z = p.robot_z + h1;

	// q3
	q3 = -acos((p_x * p_x + p_y * p_y + (p_z - d1) * (p_z - d1) - L2 * L2 - L3 * L3) / (2 * L2 * L3));

	// q2
	Mat q2_vec_sol = (Mat_<double>(2, 2) << L2 + L3 * cos(q3), -L3 * sin(q3), L3 * sin(q3), L2 + L3 * cos(q3));
	Mat q2_vec = q2_vec_sol.inv() * (Mat_<double>(2, 1) << cos(q1) * p.robot_x + sin(q1) * p.robot_y, p.robot_z - d1);
	q2 = atan(q2_vec.at<double>(1) / q2_vec.at<double>(0));

	vector<int>joint(4); // = zeros(1, 4);

	joint.at(0) = round((180 / CV_PI) * (q1) / 0.0879);
	joint.at(1) = round((180 / CV_PI) * (q2 + CV_PI / 2) / 0.0879);
	joint.at(2) = round((180 / CV_PI) * (q3 + CV_PI) / 0.0879);
	joint.at(3) = round((180 / CV_PI) * (-(q2 + q3) + 5 * CV_PI / 6) / 0.293);

	return joint;
};

vector<char> make_paket(vector<int> joint) {
	vector<char> arr(10);

	arr.at(0) = char('S');
	arr.at(1) = char(joint.at(0) / 64);
	arr.at(2) = char(joint.at(0) % 64);
	arr.at(3) = char(joint.at(1) / 64);
	arr.at(4) = char(joint.at(1) % 64);
	arr.at(5) = char(joint.at(2) / 64);
	arr.at(6) = char(joint.at(2) % 64);
	arr.at(7) = char(joint.at(3) / 64);
	arr.at(8) = char(joint.at(3) % 64);
	arr.at(9) = char('E');

	return arr;
};
#endif