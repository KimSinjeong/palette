#ifndef _BOARD
#define _BOARD
#include "myLine.h"
#include "myPoint.h"


void draw_board(myPoint point_arr[][12]) {
	Mat board = Mat::zeros(480, 480, CV_8UC3);
	board = Scalar(50, 150, 200);
	rectangle(board, Point(20, 20), Point(460, 460), Scalar(0, 0, 0), 4, LINE_8);
	for (int i = 0; i < 11; i++) {
		line(board, Point(20, 60 + 40 * i), Point(460, 60 + 40 * i), Scalar(0, 0, 0), 2, LINE_8);
		line(board, Point(60 + 40 * i, 20), Point(60 + 40 * i, 460), Scalar(0, 0, 0), 2, LINE_8);
	}
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			if (point_arr[i][j].state != EMPTY) {
				circle(board, Point(20 + 40 * j, 20 + 40 * i), 10, Scalar(0, 0, 180 * (point_arr[i][j].state - 1)), -1, LINE_8);
			}
		}
	}
	imshow("board", board);
}

#endif