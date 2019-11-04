//#include "SerialComm.h"
#include "SerialPort.h"
#include "myLine.h"
#include "myPoint.h"
#include "myBoard.h"
#include "motor_arc.h"
#include "ai.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <predefined_dictionaries.hpp>
#include <opencv2/core/hal/hal.hpp>
#include <iostream>
#include <string>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int screen_width;
int screen_hieght;

Mat dictionary = Mat(250, (6 * 6 + 7) / 8, CV_8UC4, (uchar*)DICT_6X6_1000_BYTES);

Mat getByteListFromBits(const Mat& bits) {
	// integer ceil
	int nbytes = (bits.cols * bits.rows + 8 - 1) / 8;

	Mat candidateByteList(1, nbytes, CV_8UC1, Scalar::all(0));
	unsigned char currentBit = 0;
	int currentByte = 0;

	uchar* rot0 = candidateByteList.ptr();

	for (int row = 0; row < bits.rows; row++) {
		for (int col = 0; col < bits.cols; col++) {
			// circular shift
			rot0[currentByte] <<= 1;

			// set bit
			rot0[currentByte] |= bits.at<uchar>(row, col);

			currentBit++;
			if (currentBit == 8) {
				// next byte
				currentBit = 0;
				currentByte++;
			}
		}
	}
	return candidateByteList;
}

bool identify(const Mat& onlyBits, int& idx, int& rotation) {
	int markerSize = 6;

	//비트 매트릭스를 바이트 리스트로 변환합니다. 
	Mat candidateBytes = getByteListFromBits(onlyBits);

	idx = -1; // by default, not found

	//dictionary에서 가장 근접한 바이트 리스트를 찾습니다. 
	int MinDistance = markerSize * markerSize + 1;
	rotation = -1;
	for (int m = 0; m < dictionary.rows; m++) {

		//각 마커 ID
		for (unsigned int r = 0; r < 4; r++) {
			int currentHamming = hal::normHamming(
				dictionary.ptr(m) + r * candidateBytes.cols,
				candidateBytes.ptr(),
				candidateBytes.cols);

			//이전에 계산된 해밍 거리보다 작다면 
			if (currentHamming < MinDistance) {
				//현재 해밍 거리와 발견된 회전각도를 기록합니다. 
				MinDistance = currentHamming;
				rotation = r;
				idx = m;
			}
		}
	}

	//idx가 디폴트값 -1이 아니면 발견된 것
	return idx != -1;
}

void is_marker(Mat frame, Mat& pap_pix2pap_real, Point2f& p_origin_pixel, Point2f& g_origin_pixel, Point2f& p_x_pixel, Point2f& p_y_pixel, Point2f& g_x_pixel, Point2f& g_y_pixel, Mat& t_g_p);

Mat find_global_real_coor(Mat input_point, Point2f pap_pix2pap_real, Point2f p_origin_pixel, Point2f g_origin_pixel, Point2f p_x_pixel, Point2f p_y_pixel, Point2f g_x_pixel, Point2f g_y_pixel, Mat t_g_p);
//find_global_real_coor 을 이용해서 아웃풋 좌표를 내보내주면 됨. + 로봇 좌표 추가해야됨 +시리얼로 내보내기+ 종이 잘 만들기, 고정시키기 +



int main() {

	char buffer = 0;
	CSerialPort serialPort;
	char port[6] = "COM7";
	char* port_p = port;

	/*
	if (!serialPort.OpenPort(port_p)) //COM25번의 포트를 오픈한다. 실패할 경우 -1을 반환한다.
	{
		cout << "connect faliled" << endl;
		return -1;
	}
	else {
		serialPort.ConfigurePort(9600, 8, 0, 0, 0);
		cout << "connect successed" << endl;
	}
	*/

	VideoCapture cap1(0);
	Size size(640, 360);
	if (!cap1.isOpened())
		cout << "카메라를 열 수 없습니다." << endl;

	Mat frame;
	cap1 >> frame;
	screen_hieght = frame.size().height;
	screen_width = frame.size().width;
	myPoint point_arr[12][12];
	int dot_num = 0;
	draw_board(point_arr);

	Mat pap_pix2pap_real;
	Point2f p_origin_pixel;	Point2f g_origin_pixel;
	Point2f p_x_pixel;	Point2f p_y_pixel;
	Point2f g_x_pixel;	Point2f g_y_pixel;
	Mat t_g_p;

	while (1)
	{
		cap1 >> frame;
		//myLine_arr line_arr;

		while (1) {//2개의 아루코 마커와 3개의 점이 인식될때까지
			cap1 >> frame;
			//imshow("hi", frame); //연산 너무 많아서 안나옴
			waitKey(10);
			//if (is_marker(frame, pap_pix2pap_real, p_origin_pixel, g_origin_pixel, p_x_pixel, p_y_pixel, g_x_pixel, g_y_pixel, t_g_p))
				//continue;
			is_marker(frame, pap_pix2pap_real, p_origin_pixel, g_origin_pixel, p_x_pixel, p_y_pixel, g_x_pixel, g_y_pixel, t_g_p);
		}

		cout << "detected 2 aruco markers and 3 points" << endl;


		/*
		//원점: paper marker, real 12*12 배열 생성(mm 단위)
		int dis_between_lines = 90 / 11;//mm단위
		for (int i = 0; i < 12; i++) {
			for (int j = 0; j < 12; j++) {
				point_arr[i][j].paper_x = Point2f(18, 23).x + Point2f(dis_between_lines * j, dis_between_lines * i).x;
				point_arr[i][j].paper_y = Point2f(18, 23).y + Point2f(dis_between_lines * j, dis_between_lines * i).y;
			}
		}

		for (int i = 0; i < 12; i++) {
			for (int j = 0; j < 12; j++) {
				Mat test_paper_pap = (Mat_<double>(2, 1) << point_arr[i][j].paper_x, point_arr[i][j].paper_y);
				Mat test_global_pap = test_paper_pap - t_g_p;
				//cout << "test_global_pap" << test_global_pap << endl;
				Mat test_global_screen = (Mat_<double>(2, 1) << test_global_pap.at<double>(0) * p_x_pixel.x + test_global_pap.at<double>(1) * p_y_pixel.x, test_global_pap.at<double>(0) * p_x_pixel.y + test_global_pap.at<double>(1) * p_y_pixel.y);
				//Mat test_global_real = (Mat_<double>(2, 1) << Point2f(test_global_screen).dot(g_x_pixel), Point2f(test_global_screen).dot(g_y_pixel));
				Mat g_direction = (Mat_<double>(2, 2) << g_x_pixel.x, g_y_pixel.x, g_x_pixel.y, g_y_pixel.y);
				Mat test_global_real = g_direction.inv() * test_global_screen;

				point_arr[i][j].global_x = Point2f(test_global_real.at<double>(1), test_global_real.at<double>(2)).x;
				point_arr[i][j].global_y = Point2f(test_global_real.at<double>(1), test_global_real.at<double>(2)).y;

				point_arr[i][j].robot_x = point_arr[i][j].global_y;
				point_arr[i][j].robot_y = point_arr[i][j].global_x + 250;
				point_arr[i][j].robot_z = 7;
			}
		}

		cout << "finish setting arr_points_pap_real" << endl;
		//cout << p_origin_pixel << endl;
		//circle(frame, p_origin_pixel, 5, Scalar(0, 255, 255));

		new_arr_of_points(frame, point_arr, pap_pix2pap_real, p_origin_pixel);
		//drawDots(frame, point_arr);
		//cout << point_arr[0][0].x <<" "<<point_arr[0][0].y<< endl;
		//imshow("points", frame);
		//line_arr.drawLines(frame);

		for (int i = 0; i < 12; i++) {
			for (int j = 0; j < 12; j++) {
				circle(frame, Point2f(point_arr[i][j].x, point_arr[i][j].y), 5, Scalar(0, 255, 255));
				//cout << Point2f(point_arr[i][j].x, point_arr[i][j].y) << endl;
			}
		}
		imshow("points", frame);




		cout << "dot detction" << endl;

		cap1 >> frame;
		int index;
		if (dotDetection(frame, point_arr)) {
			cout << ++dot_num << endl;
			draw_board(point_arr);
			waitKey(1000);
			index = ai_turn(point_arr);
			draw_board(point_arr);
			cout << index << endl;
			if (index == -1) {
				cout << "computer win!" << endl;
				break;
			}

			else if (index == -2) {
				cout << "player win!" << endl;
				break;
			}

			if (index == -3) continue;
			myPoint p;

			//////////@@@@@@@@@@@@@@@@@@@@@Serial@@@@@@@@@@@@@@@@@@@@@@@////////////////////
			//myPoint p;
			//@@@@ index : index of ai stone in points_arr
			
			cout << "send the point start" << endl;
			for (int i = 0; i < 4; i++) {
				p.robot_x = point_arr[index / 12][index % 12].robot_x;
				p.robot_y = point_arr[index / 12][index % 12].robot_y;
				p.robot_z = point_arr[index / 12][index % 12].robot_z - 5 * i;

				vector<int> joint;
				joint = joint_arc(p);

				vector <char> packet;
				packet = make_paket(joint);
				for (int i = 0; i < packet.size(); i++) {
					if (serialPort.WriteByte(char(packet.at(i))))
					{
						cout << i << " : " << +char(packet.at(i)) << " ";
						//cout << "input : " << char(packet.at(i)) << endl;
						BYTE byte1;
						serialPort.ReadByte(byte1);
						//cout << "output : " << byte1 << endl;
					}
					//cout << "send Command success" << endl;

				}
				cout << endl;
			}
			//1
			p.robot_x = point_arr[index / 12][index % 12].robot_x + 0.3;
			p.robot_y = point_arr[index / 12][index % 12].robot_y + 0.3;
			p.robot_z = point_arr[index / 12][index % 12].robot_z;

			vector<int> joint;
			joint = joint_arc(p);
			vector <char> packet;
			packet = make_paket(joint);
			for (int i = 0; i < packet.size(); i++) {
				if (serialPort.WriteByte(char(packet.at(i))))
				{
					//cout << "input : " << char(packet.at(i)) << endl;
					BYTE byte1;
					serialPort.ReadByte(byte1);
					//cout << "output : " << byte1 << endl;
				}
				//cout << "send Command success" << endl;

			}
			//2
			p.robot_x = point_arr[index / 12][index % 12].robot_x - 0.3;
			p.robot_y = point_arr[index / 12][index % 12].robot_y + 0.3;
			p.robot_z = point_arr[index / 12][index % 12].robot_z;

			//vector<int> joint;
			joint = joint_arc(p);

			//vector <char> packet;
			packet = make_paket(joint);
			for (int i = 0; i < packet.size(); i++) {
				if (serialPort.WriteByte(char(packet.at(i))))
				{
					//cout << "input : " << char(packet.at(i)) << endl;
					BYTE byte1;
					serialPort.ReadByte(byte1);
					//cout << "output : " << byte1 << endl;
				}
				//cout << "send Command success" << endl;

			}
			//3
			p.robot_x = point_arr[index / 12][index % 12].robot_x - 0.3;
			p.robot_y = point_arr[index / 12][index % 12].robot_y - 0.3;
			p.robot_z = point_arr[index / 12][index % 12].robot_z;

			//vector<int> joint;
			joint = joint_arc(p);

			//vector <char> packet;
			packet = make_paket(joint);
			for (int i = 0; i < packet.size(); i++) {
				if (serialPort.WriteByte(char(packet.at(i))))
				{
					//cout << "input : " << char(packet.at(i)) << endl;
					BYTE byte1;
					serialPort.ReadByte(byte1);
					//cout << "output : " << byte1 << endl;
				}
				//cout << "send Command success" << endl;

			}
			//4
			p.robot_x = point_arr[index / 12][index % 12].robot_x + 0.3;
			p.robot_y = point_arr[index / 12][index % 12].robot_y - 0.3;
			p.robot_z = point_arr[index / 12][index % 12].robot_z;

			//vector<int> joint;
			joint = joint_arc(p);

			//vector <char> packet;
			packet = make_paket(joint);
			for (int i = 0; i < packet.size(); i++) {
				if (serialPort.WriteByte(char(packet.at(i))))
				{
					//cout << "input : " << char(packet.at(i)) << endl;
					BYTE byte1;
					serialPort.ReadByte(byte1);
					//cout << "output : " << byte1 << endl;
				}
				//cout << "send Command success" << endl;

			}
			//5
			p.robot_x = point_arr[index / 12][index % 12].robot_x + 0.3;
			p.robot_y = point_arr[index / 12][index % 12].robot_y + 0.3;
			p.robot_z = point_arr[index / 12][index % 12].robot_z;

			//vector<int> joint;
			joint = joint_arc(p);

			//vector <char> packet;
			packet = make_paket(joint);
			for (int i = 0; i < packet.size(); i++) {
				if (serialPort.WriteByte(char(packet.at(i))))
				{
					//cout << "input : " << char(packet.at(i)) << endl;
					BYTE byte1;
					serialPort.ReadByte(byte1);
					//cout << "output : " << byte1 << endl;
				}
				//cout << "send Command success" << endl;

			}
			for (int i = 0; i < 4; i++) {
				p.robot_x = point_arr[index / 12][index % 12].robot_x;
				p.robot_y = point_arr[index / 12][index % 12].robot_y;
				p.robot_z = point_arr[index / 12][index % 12].robot_z + 5 * i;

				vector<int> joint;
				joint = joint_arc(p);

				vector <char> packet;
				packet = make_paket(joint);
				for (int i = 0; i < packet.size(); i++) {
					if (serialPort.WriteByte(char(packet.at(i))))
					{
						//cout << "input : " << char(packet.at(i)) << endl;
						BYTE byte1;
						serialPort.ReadByte(byte1);
						//cout << "output : " << byte1 << endl;
					}
					//cout << "send Command success" << endl;

				}
			}
			
			//////////@@@@@@@@@@@@@@@@@@@@@Serial End@@@@@@@@@@@@@@@@@@@@@@@////////////////////
			draw_board(point_arr);
			waitKey(1000);
		}
		if (waitKey(10) == 27) break;
		*/
	}
	waitKey(0);
	//serialPort.ClosePort(); //작업이 끝나면 포트를 닫는다

	cout << "end connect" << endl;



	return 0;
}



//--------------------------------------------------------------------------------------------------------------------------//



void is_marker(Mat input_image, Mat& pap_pix2pap_real, Point2f& p_origin_pixel, Point2f& g_origin_pixel, Point2f& p_x_pixel, Point2f& p_y_pixel, Point2f& g_x_pixel, Point2f& g_y_pixel, Mat& t_g_p) {
	if (!input_image.empty()) {
		Mat input_gray_image;
		Mat binary_image;

		cvtColor(input_image, input_gray_image, COLOR_BGR2GRAY);
		adaptiveThreshold(input_gray_image, binary_image, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 91, 7);

		Mat contour_image = binary_image.clone();
		vector<vector<Point>> contours;
		findContours(contour_image, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

		//contour를 근사화한다.
		vector<vector<Point2f> > marker; // 그냥 contour 구해서 사각형인거만 저장
		vector<Point2f> approx;

		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.05, true);

			if (
				approx.size() == 4 && //사각형은 4개의 vertex를 가진다. 
				fabs(contourArea(Mat(approx))) > 1000 && //면적이 일정크기 이상이어야 한다.
				fabs(contourArea(Mat(approx))) < 50000 && //면적이 일정크기 이하여야 한다. 
				isContourConvex(Mat(approx)) //convex인지 검사한다.
				)
			{

				//drawContours(input_image, contours, i, Scalar(0, 255, 0), 1, LINE_AA);

				vector<cv::Point2f> points;
				for (int j = 0; j < 4; j++)
					points.push_back(cv::Point2f(approx[j].x, approx[j].y));

				//반시계 방향으로 정렬
				cv::Point v1 = points[1] - points[0];
				cv::Point v2 = points[2] - points[0];

				double o = (v1.x * v2.y) - (v1.y * v2.x); //벡터 외적해서 시계방향인지 판단, 음수면 반시계방향이라는 거라서 1번, 3번 점 을 바꿔준다.
				if (o < 0.0)
					swap(points[1], points[3]);

				marker.push_back(points);

			}
		}

		//imshow("contour",input_image);

		vector<vector<Point2f> > detectedMarkers; // 테두리가 모두 검정색인 마커의 네 꼭짓점을 식만 저장
		vector<Mat> detectedMarkersImage; //테두리가 모두 검정색인 마커의 
		vector<Point2f> square_points;

		int marker_image_side_length = 80; //마커 6x6크기일때 검은색 테두리 영역 포함한 크기는 8x8
									//이후 단계에서 이미지를 격자로 분할할 시 셀하나의 픽셀너비를 10으로 한다면
									//마커 이미지의 한변 길이는 80
		square_points.push_back(cv::Point2f(0, 0));
		square_points.push_back(cv::Point2f(marker_image_side_length - 1, 0));
		square_points.push_back(cv::Point2f(marker_image_side_length - 1, marker_image_side_length - 1));
		square_points.push_back(cv::Point2f(0, marker_image_side_length - 1));



		Mat marker_image;
		for (int i = 0; i < marker.size(); i++)
		{
			vector<Point2f> m = marker[i];


			//마커를 사각형형태로 바꿀 perspective transformation matrix를 구한다.
			Mat PerspectiveTransformMatrix = getPerspectiveTransform(m, square_points);

			//perspective transformation을 적용한다. 
			warpPerspective(input_gray_image, marker_image, PerspectiveTransformMatrix, Size(marker_image_side_length, marker_image_side_length));

			//otsu 방법으로 이진화를 적용한다. 
			threshold(marker_image, marker_image, 125, 255, THRESH_BINARY | THRESH_OTSU);



			//마커의 크기는 6, 검은색 태두리를 포함한 크기는 8
			//마커 이미지 테두리만 검사하여 전부 검은색인지 확인한다. 
			int cellSize = marker_image.rows / 8;
			int white_cell_count = 0;

			for (int y = 0; y < 8; y++)
			{
				int inc = 7; // 첫번째 열과 마지막 열만 검사하기 위한 값

				if (y == 0 || y == 7) inc = 1; //첫번째 줄과 마지막줄은 모든 열을 검사한다. 


				for (int x = 0; x < 8; x += inc)
				{
					int cellX = x * cellSize;
					int cellY = y * cellSize;

					cv::Mat cell = marker_image(Rect(cellX, cellY, cellSize, cellSize));

					int total_cell_count = countNonZero(cell);

					if (total_cell_count > (cellSize * cellSize) / 2)
						white_cell_count++; //태두리에 흰색영역이 있다면, 셀내의 픽셀이 절반이상 흰색이면 흰색영역으로 본다 

				}
			}

			//검은색 태두리로 둘러쌓여 있는 것만 저장한다.
			if (white_cell_count == 0) {
				detectedMarkers.push_back(m);
				Mat img = marker_image.clone();
				detectedMarkersImage.push_back(img);
			}
		}

		vector<Mat> bitMatrixs;
		for (int i = 0; i < detectedMarkers.size(); i++)
		{
			Mat marker_image = detectedMarkersImage[i];

			//내부 6x6에 있는 정보를 비트로 저장하기 위한 변수
			Mat bitMatrix = Mat::zeros(6, 6, CV_8UC1);

			int cellSize = marker_image.rows / 8;
			for (int y = 0; y < 6; y++)
			{
				for (int x = 0; x < 6; x++)
				{
					int cellX = (x + 1) * cellSize;
					int cellY = (y + 1) * cellSize;
					Mat cell = marker_image(cv::Rect(cellX, cellY, cellSize, cellSize));

					int total_cell_count = countNonZero(cell);

					if (total_cell_count > (cellSize * cellSize) / 2)
						bitMatrix.at<uchar>(y, x) = 1;
				}
			}

			bitMatrixs.push_back(bitMatrix);

		}

		//final_detectedMarkers :: 시계순서로 점을 저장
		vector<int> markerID;
		vector<vector<Point2f> > final_detectedMarkers;
		for (int i = 0; i < detectedMarkers.size(); i++)
		{
			Mat bitMatrix = bitMatrixs[i];
			vector<Point2f> m = detectedMarkers[i];

			int rotation;
			int marker_id;

			if (!identify(bitMatrix, marker_id, rotation))
				cout << "발견안됨" << endl;

			else {

				if (rotation != 0) {
					//회전을 고려하여 코너를 정렬합니다. 
					//마커의 회전과 상관없이 마커 코너는 항상 같은 순서로 저장됩니다.
					std::rotate(m.begin(), m.begin() + 4 - rotation, m.end());
				}

				int sumx = 0, sumy = 0;
				for (int j = 0; j < 4; j++) {
					putText(input_image, to_string(j + 1), Point(m[j].x, m[j].y), QT_FONT_NORMAL, 1, Scalar(255, 0, 0), 1, 1);
					sumx += m[j].x;
					sumy += m[j].y;
				}
				putText(input_image, "id=" + to_string(marker_id), Point(sumx / 4, sumy / 4), QT_FONT_NORMAL, 1, Scalar(255, 0, 0), 1, 1);

				cornerSubPix(input_gray_image, m, Size(5, 5), Size(-1, -1), TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.01));
				markerID.push_back(marker_id);
				final_detectedMarkers.push_back(m);
			}

		}


		//imshow("aruco", input_image);

		SimpleBlobDetector::Params params;
		params.minThreshold = 100;
		//params.maxThreshold = 220;
		params.filterByArea = true;
		params.minArea = 40;
		params.maxArea = 300;
		params.filterByColor = true;
		params.blobColor = 0; // 어두운 얼룩 추출 : 0, 밝은 얼룩 추출 : 255
		params.filterByCircularity = false;

		Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

		vector<KeyPoint> keypoints;
		detector->detect(input_gray_image, keypoints);

		drawKeypoints(input_gray_image, keypoints, input_image, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


		//imshow("blob", input_image);


		for (int i = 0; i < detectedMarkers.size(); i++)
		{
			int sumx = 0, sumy = 0;
			for (int j = 0; j < 4; j++) {
				putText(input_image, to_string(j + 1), Point(final_detectedMarkers.at(i).at(j).x, final_detectedMarkers.at(i).at(j).y), QT_FONT_NORMAL, 1, Scalar(255, 0, 0), 1, 1);
				sumx += final_detectedMarkers.at(i).at(j).x;
				sumy += final_detectedMarkers.at(i).at(j).y;
			}

			putText(input_image, "id=" + to_string(markerID.at(i)), Point(sumx / 4, sumy / 4), QT_FONT_NORMAL, 1, Scalar(255, 0, 0), 1, 1);
			if (markerID.at(i) == 0) {//paper 마커의 x축 y축 방향벡터 & 원점의 좌표 구하기(0번째 점) , pixel 좌표로
				//cout << "paper 원점" << final_detectedMarkers.at(i).at(0) << endl;
				circle(input_image, final_detectedMarkers.at(i).at(0), 5, Scalar(255, 255, 0), -1);
				p_origin_pixel = final_detectedMarkers.at(i).at(0);
				p_x_pixel = final_detectedMarkers.at(i).at(1) - final_detectedMarkers.at(i).at(0);
				p_y_pixel = final_detectedMarkers.at(i).at(3) - final_detectedMarkers.at(i).at(0);
				p_x_pixel = p_x_pixel / sqrt(p_x_pixel.dot(p_x_pixel)); //크기 1로 만들기
				p_y_pixel = p_y_pixel / sqrt(p_y_pixel.dot(p_y_pixel)); //크기 1로 만들기

			}

			else if (markerID.at(i) == 41) {//global 마커의 x축 y축 방향벡터 & 원점의 좌표 구하기(0번째점), pixel 좌표로
				//cout << "global 원점" << final_detectedMarkers.at(i).at(0) << endl;
				circle(input_image, final_detectedMarkers.at(i).at(0), 5, Scalar(0, 0, 0), -1);
				g_origin_pixel = final_detectedMarkers.at(i).at(0);
				g_x_pixel = final_detectedMarkers.at(i).at(1) - final_detectedMarkers.at(i).at(0);
				g_y_pixel = final_detectedMarkers.at(i).at(3) - final_detectedMarkers.at(i).at(0);
				g_x_pixel = g_x_pixel / sqrt(g_x_pixel.dot(g_x_pixel)); //크기 1로 만들기
				g_y_pixel = g_y_pixel / sqrt(g_y_pixel.dot(g_y_pixel)); //크기 1로 만들기
			}

		}

		Point2f p_x_blob;
		Point2f p_y_blob;
		Point2f p_xy_blob;

		for (int i = 0; i < keypoints.size(); i++) {
			//cout << keypoints.at(i).pt << endl;
			circle(input_image, Point2f(keypoints.at(i).pt.x, keypoints.at(i).pt.y), 5, Scalar(0, 0, 255), 1);
			Point2f keyP = keypoints.at(i).pt - p_origin_pixel;
			//Point2f keyP = keypoints.at(i).pt;
			//int min_dot;
			if (abs(keyP.dot(p_x_pixel)) > cos(5 * CV_PI / 180)* sqrt(keyP.dot(keyP))) {
				p_x_blob = keyP + p_origin_pixel;
			}
			else if (abs(keyP.dot(p_y_pixel)) > cos(5 * CV_PI / 180)* sqrt(keyP.dot(keyP))) {
				p_y_blob = keyP + p_origin_pixel;
				//cout << "p_y_blob" <<p_y_blob << endl;
			}
			else {
				p_xy_blob = keyP + p_origin_pixel;
			}
		}

		imshow("keypoints", input_image);

		cout << "점 갯수" << keypoints.size() << endl;

		// 문제!! paper origin 기준 projection 을 했을 때 global marker 가  

		// 첫번째 방법 (paper origin을 기준으로 한 projection matrix 하나만 구한다.)
		// 이후 global origin가 transform된 좌표를 구해서 


		// 두번째 방법(projection matrix 를 두개 구한다.)
		// 사용자가 둔 점은 paper origin 을 기준으로 projection matrix 한걸로 추적
		// 로봇이 둘 실제 좌표는 globla origin 을 기준으로 projection matrix 계산 한 걸로 추적


		// 세번째 방법 (paper origin을 기준으로 projection matrix 구한 image 에서 다시 global 원점을 기준으로 한 transform을 구한다.)

		// transMat_paper 행렬 구하기		
		Point TopLeft = Point(p_origin_pixel.x, p_origin_pixel.y);
		Point TopRight = p_x_blob;
		Point BottomRight = p_xy_blob;
		Point BottomLeft = p_y_blob;

		vector<Point>rect;
		rect.push_back(TopLeft);
		rect.push_back(TopRight);
		rect.push_back(BottomRight);
		rect.push_back(BottomLeft);

		Point2f src[4], dst[4];
		src[0] = Point2f(TopLeft.x, TopLeft.y);
		src[1] = Point2f(TopRight.x, TopRight.y);
		src[2] = Point2f(BottomRight.x, BottomRight.y);
		src[3] = Point2f(BottomLeft.x, BottomLeft.y);

		dst[0] = Point2f(0, 0);
		dst[1] = Point2f(122.0 - 1.0, 0);
		dst[2] = Point2f(122.0 - 1.0, 122.0 - 1.0);
		dst[3] = Point2f(0, 122.0 - 1.0);

		Mat transMat_paper = getPerspectiveTransform(src, dst);
		Mat paperFrame;

		warpPerspective(input_image, paperFrame, transMat_paper, Size(300, 300));
		imshow("perspective", paperFrame);


		// transMat_global




		//calculate the global oringin point with projection matrix




		if (detectedMarkers.size() == 2 && p_x_blob.x && p_y_blob.x) {

			/*
			//종이_픽셀 <-> 종이_실제 ---------------------------------

			Mat p_blob = (Mat_<double>(2, 2) << p_x_blob.x, p_y_blob.x, p_x_blob.y, p_y_blob.y);
			Mat real_blob = (Mat_<double>(2, 2) << 122.0, 0.0, 0.0, 122.0);
			//double pap_pix2pap_real[2][2];
			//Mat pap_pix2pap_real=(Mat_<double>(2,2)<<real_blob*p_blob.inv());
			pap_pix2pap_real = real_blob * p_blob.inv();
			//cout << "pap_pix2pap_real"<<pap_pix2pap_real << endl;
			Mat test_paper_coor = (Mat_<double>(2, 1) << 61, 61);

			//pap_pix2pap_real 잘 되나 확인(paper 중심에 빨간점 찍기)
			Mat test_blob_coor = pap_pix2pap_real.inv() * test_paper_coor;
			//cout <<"test_blob_coor"<< test_blob_coor << endl;
			circle(input_image, Point2f(test_blob_coor.at<double>(0),test_blob_coor.at<double>(1))+p_origin_pixel, 5, Scalar(0, 0, 255), 1);
			//잘 되네 ㅋㅋ




			//종이_실제 <-> 글로벌_실제 ------------------------------------
			Mat t_g_p_pixel = (Mat_<double>(2, 1) << (g_origin_pixel - p_origin_pixel).x, (g_origin_pixel - p_origin_pixel).y);
			t_g_p = pap_pix2pap_real * t_g_p_pixel;


			//실험 1: pixel-> global
			Mat pixel = (Mat_<double>(2, 1) << 100, 100);
			circle(input_image, Point2f(pixel), 5, Scalar(255, 0, 0), 1);
			Mat test_paper_pap_pixel = pixel - (Mat_<double>(2, 1) << p_origin_pixel.x, p_origin_pixel.y);
			Mat test_paper_pap = pap_pix2pap_real * test_paper_pap_pixel;
			//cout << "test_paper_pap" << test_paper_pap << endl;
			Mat test_global_pap = test_paper_pap - t_g_p;
			//cout << "test_global_pap" << test_global_pap << endl;
			Mat test_global_screen = (Mat_<double>(2, 1) << test_global_pap.at<double>(0) * p_x_pixel.x + test_global_pap.at<double>(1) * p_y_pixel.x, test_global_pap.at<double>(0) * p_x_pixel.y + test_global_pap.at<double>(1) * p_y_pixel.y);
			//Mat test_global_real = (Mat_<double>(2, 1) << Point2f(test_global_screen).dot(g_x_pixel), Point2f(test_global_screen).dot(g_y_pixel));
			Mat g_direction = (Mat_<double>(2, 2) << g_x_pixel.x, g_y_pixel.x, g_x_pixel.y, g_y_pixel.y);
			Mat test_global_real = g_direction.inv()*test_global_screen;
			//cout <<"test_global_real"<< Point2f(test_global_real) << endl;


			//test_global_real = (Mat_<double>(2, 1) << 200, 200);
			//실험 2 : global->pixel
			Mat test_global_screen_2 = (Mat_<double>(2, 1) << Point2f(test_global_real).x * (g_x_pixel.x) + Point2f(test_global_real).y * (g_y_pixel.x), Point2f(test_global_real).x * (g_x_pixel.y) + Point2f(test_global_real).y * (g_y_pixel.y));
			Mat p_direction = (Mat_<double>(2, 2) << p_x_pixel.x, p_y_pixel.x, p_x_pixel.y, p_y_pixel.y);
			Mat test_global_pap_2 = p_direction.inv() * test_global_screen_2;
			//Mat test_global_pap_2 = (Mat_<double>(2, 1) << Point2f(test_global_screen_2).dot(p_x_pixel), Point2f(test_global_screen_2).dot(p_y_pixel));// 내적으로 구하기
			Mat test_paper_pap_2 = test_global_pap_2 + t_g_p;
			//cout << "test_paper_pap" << test_paper_pap << endl;
			Mat test_paper_pap_pixel_2 = pap_pix2pap_real.inv() * test_paper_pap_2;
			circle(input_image, Point2f(test_paper_pap_pixel_2.at<double>(0), test_paper_pap_pixel_2.at<double>(1)) + p_origin_pixel, 5, Scalar(0, 0, 255), 1);
			//cout << "pixel point" << Point2f(test_paper_pap_pixel_2.at<double>(0), test_paper_pap_pixel_2.at<double>(1)) + p_origin_pixel << endl;

			imshow("aruco and blob", input_image);

			*/
			//return 1;

		}

		else {
			cout << "no enogh markers ^^" << endl;
			//return 0;
		}
	}

	else {
		cout << "no input" << endl;
		//return 0;
	}
};

Mat find_global_real_coor(Mat input_point, Mat pap_pix2pap_real, Point2f p_origin_pixel, Point2f g_origin_pixel, Point2f p_x_pixel, Point2f p_y_pixel, Point2f g_x_pixel, Point2f g_y_pixel, Mat t_g_p) {
	Mat pixel = (Mat_<double>(2, 1) << 200, 200);
	//circle(input_image, Point2f(pixel), 5, Scalar(255, 0, 0), 1);
	Mat test_paper_pap_pixel = pixel - (Mat_<double>(2, 1) << p_origin_pixel.x, p_origin_pixel.y);
	Mat test_paper_pap = pap_pix2pap_real * test_paper_pap_pixel;
	//cout << "test_paper_pap" << test_paper_pap << endl;
	Mat test_global_pap = test_paper_pap - t_g_p;
	//cout << "test_global_pap" << test_global_pap << endl;
	Mat test_global_screen = (Mat_<double>(2, 1) << test_global_pap.at<double>(0) * p_x_pixel.x + test_global_pap.at<double>(1) * p_y_pixel.x, test_global_pap.at<double>(0) * p_x_pixel.y + test_global_pap.at<double>(1) * p_y_pixel.y);
	Mat test_global_real = (Mat_<double>(2, 1) << Point2f(test_global_screen).dot(g_x_pixel), Point2f(test_global_screen).dot(g_y_pixel));
	//cout << Point2f(test_global_real) << endl;
	return test_global_real;
};