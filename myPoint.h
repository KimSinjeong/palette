#ifndef _POINT
#define _POINT
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "myLine.h"
#include <iostream>

constexpr auto EMPTY = 0;
constexpr auto BLACK = 1; //computer
constexpr auto RED = 2;   //player
constexpr auto OUTOFDOTS = 144;



class myPoint {
public:
	int x, y; //ÇÈ¼¿ÁÂÇ¥
	int global_x, global_y; //¿øÁ¡: global marker, real -> ¿øÁ¡À» ·Îº¿À¸·Î ¹Ù²ã¾ßµÊ
	int paper_x, paper_y; //¿øÁ¡ :paper marker, real 
	int robot_x, robot_y, robot_z;
	int state;
	float weight;
	myPoint(int _x = 0, int _y = 0) {
		x = _x; y = _y;
		state = EMPTY;
		weight = 0;
		robot_z = 20;
	}
};

void arr_of_points(myLine_arr& lines_comb, myPoint points[][12]);
void new_arr_of_points(Mat frame, myPoint points[][12], Mat pap_pix2pap_real, Point2f p_origin_pixel);

void drawDots(Mat frame, myPoint points[][12]);
void blobDetection(Mat frame, vector<KeyPoint>& blobPoints);
int find_dot_index(myPoint points[][12], KeyPoint blob);
int dotDetection(Mat frame, myPoint points[][12]);
void to_state_arr(myPoint (*points)[12], int (*state)[12]);
void to_point_arr(int(*state)[12], myPoint(*points)[12]);

#endif