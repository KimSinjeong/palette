#include <iostream>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;


///////////class definition for dot&line///////////
class Dot {
	int x, y;
	int servo;
	Dot(int _x, int _y) {
		x = _x;
		y = _y;
		servo = 0;
	}
}

class Line {
	vector<Dot> dots;
	Dot start;
	Dot end;
	void add(Dot dot) dots.add(dot);
	void add(int index, Dot dot) dots.add(index, dot);
	Dot get(int index) return dots.get(index);
	int size() return dots.size();
	bool isEmpty() return dots.isEmpty();
	Dot removefront() {
		dots.remove(0);
		if (dots.size() == 0) return null;
		return dots.get(0);
	}
	Dot removeback() {
		dots.remove(dots.size() - 1);
		if (dots.size() == 0) return null;
		return dots.get(dots.size() - 1);
	}
}

vector<Dot> dots;
vector<Dot> drawn;

const int interval = 2;
const float t_anchor = 2;
const float t_gradient = 20;
const int VERTICAL = 0;
const int HORIZONTAL = 1;
const int MAX_LINE = 300;
const int UPLEFT = -1;
const int DOWNRIGHT = 1;

const int img_width;
const int img_height;

////////comparator for sorting line_list/////////
//sort lines decreasing order
bool lineComparator(Line l1, Line l2) {
	return l2.size() > l1.size();
}


////////////////////draw//////////////////////////
void draw(Mat img, vector<Dot> dots, vector<Dot> drawn) {
	for (int i = 1; i < dots.size(); i++) {
		if (dots.at(i - 1).servo == 0)
			line(img,Point(dots.at(i).x, dots.at(i).y), Point(dots.at(i - 1).x, dots.at(i - 1).y), Scalar(255,0,0));
	}
	if (!dots.isEmpty() && !drawn.isEmpty() && drawn.get(drawn.size() - 1).servo == 0)
		line(dots.get(0).x, dots.get(0).y, drawn.get(drawn.size() - 1).x, drawn.get(drawn.size() - 1).y);
	for (int i = 1; i < drawn.size(); i++) {
		if (drawn.get(i - 1).servo == 0)
			line(drawn.get(i).x, drawn.get(i).y, drawn.get(i - 1).x, drawn.get(i - 1).y), Scalar(0, 0, 255);
	}
	if (!dots.isEmpty()) {
		circle(img, Point(dots.get(0).x, dots.get(0).y), 3, Scalar(0, 255, 0), -1);
	}
};


//get sorted dots from img.
void processimg(Mat img, vector<Dot> & dots) {
	//load image
	Mat Gimg;
	getGradient(img, Gimg);
	//Extraction of the anchors
	vector<Integer> anchors;
	vector<Line> line_list;
	anchors(Gimg, anchors);
	Edge(Gimg, anchors, line_list);
	sortDots(line_list, dots);
}

//Conneting the anchors
void Edge(Mat Gimg, vector<Integer> anchors, vector<Line> & line_list) {
	int checked[Gimg.size().width][Gimg.size().height];
	//@@@ initialize
	while (anchors.size() != 0) {
		int loc = anchors.back();
		//start at the next point(UPLEFT) of the anchor//
		int tmp_loc = walk(Gimg, loc, UPLEFT);
		int a = UPLEFT;
		if (tmp_loc < 0) {
			a = UPLEFT;
			tmp_loc = -tmp_loc;
		}
		else a = DOWNRIGHT;
		int x = tmp_loc % Gimg.size().width;
		int y = tmp_loc / Gimg.size().width;
		/////////////////////////////////////////////////
		Dot dot = NULL;
		Line line = new Line();
		//add dots to the front of line(UPLEFT)
		while (checked[x][y] == 0 && red(Gimg.pixels[tmp_loc]) >= t_gradient
			&& x > 1 && y > 1 && x < Gimg.size().width - 1 && y < Gimg.size().height - 1) {
			checked[x][y] = 1;
			dot = new Dot(x, y);
			line.add(0, dot);
			tmp_loc = walk(Gimg, tmp_loc, a);
			if (tmp_loc < 0) {
				a = UPLEFT;
				tmp_loc = -tmp_loc;
			}
			else a = DOWNRIGHT;
			x = tmp_loc % Gimg.size().width;
			y = tmp_loc / Gimg.size().width;
		}
		line.start = dot; //set start point
		//change x,y to anchor
		x = loc % Gimg.size().width;
		y = loc / Gimg.size().width;
		a = DOWNRIGHT;
		dot = new Dot(x, y);
		if (line.start == null) //If anchor is start point
			line.start = dot;
		line.add(dot); //add the anchor
		//start at the next point(DOWNRIGHT) of the anchor//
		tmp_loc = walk(Gimg, loc, a);
		if (tmp_loc < 0) {
			a = UPLEFT;
			tmp_loc = -tmp_loc;
		}
		else a = DOWNRIGHT;
		x = tmp_loc % Gimg.size().width;
		y = tmp_loc / Gimg.size().width;
		//add dots to the back of the line(DOWNRIGHT)
		while (checked[x][y] == 0 == 0 && red(Gimg.pixels[tmp_loc]) >= t_gradient
			&& x > 1 && y > 1 && x < Gimg.width - 1 && y < Gimg.height) {
			checked[x][y] = 1;
			dot = new Dot(x, y);
			line.add(dot);
			tmp_loc = walk(Gimg, tmp_loc, a);
			if (tmp_loc < 0) {
				a = UPLEFT;
				tmp_loc = -tmp_loc;
			}
			else a = DOWNRIGHT;
			x = tmp_loc % Gimg.size().width;
			y = tmp_loc / Gimg.size().width;
		}
		line.end = dot;
		if (line.size() > 5) {
			line_list.add(line);
		}
		anchors.pop_back();
	}
	//sort the lines in decreasing order
	std::sort(line_list.begin(), line_list.end(), lineComparator);
	//limit the number of lines(MAX_LINES)
	while (line_list.size() > < AX_LINE)
		line_list.pop_back();
}

///////////////////get next point///////////////////////
int walk(PImage Gimg, int tmp_loc, int a) {
	int loc = 0;
	if (a == UPLEFT) {
		if (green(Gimg.pixels[tmp_loc]) == VERTICAL) { //UP
			float max = max(red(Gimg.pixels[tmp_loc - Gimg.width - 1]),
				red(Gimg.pixels[tmp_loc - Gimg.width]), red(Gimg.pixels[tmp_loc - Gimg.width + 1]));
			if (red(Gimg.pixels[tmp_loc - Gimg.width - 1]) == max)
				loc = tmp_loc - Gimg.width - 1;
			else if (red(Gimg.pixels[tmp_loc - Gimg.width + 1]) == max)
				loc = tmp_loc - Gimg.width + 1;
			else
				loc = tmp_loc - Gimg.width;
		}
		else {  //LEFT
			float max = max(red(Gimg.pixels[tmp_loc - Gimg.width - 1]),
				red(Gimg.pixels[tmp_loc - 1]), red(Gimg.pixels[tmp_loc + Gimg.width - 1]));
			if (red(Gimg.pixels[tmp_loc - Gimg.width - 1]) == max)
				loc = tmp_loc - Gimg.width - 1;
			else if (red(Gimg.pixels[tmp_loc - 1]) == max)
				loc = tmp_loc - 1;
			else
				loc = tmp_loc + Gimg.width - 1;
		}
	}
	else if (a == DOWNRIGHT) { //
		if (green(Gimg.pixels[tmp_loc]) == VERTICAL) { //DOWN
			float max = max(red(Gimg.pixels[tmp_loc + Gimg.width - 1]),
				red(Gimg.pixels[tmp_loc + Gimg.width]), red(Gimg.pixels[tmp_loc + Gimg.width + 1]));
			if (red(Gimg.pixels[tmp_loc + Gimg.width - 1]) == max)
				loc = tmp_loc + Gimg.width - 1;
			else if (red(Gimg.pixels[tmp_loc + Gimg.width + 1]) == max)
				loc = tmp_loc + Gimg.width + 1;
			else
				loc = tmp_loc + Gimg.width;
		}
		else { //RIGHT
			float max = max(red(Gimg.pixels[tmp_loc - Gimg.width + 1]),
				red(Gimg.pixels[tmp_loc + 1]), red(Gimg.pixels[tmp_loc + Gimg.width + 1]));
			if (red(Gimg.pixels[tmp_loc - Gimg.width + 1]) == max)
				loc = tmp_loc - Gimg.width + 1;
			else if (red(Gimg.pixels[tmp_loc + Gimg.width + 1]) == max)
				loc = tmp_loc + Gimg.width + 1;
			else

				loc = tmp_loc + 1;
		}
	}
	int x1 = tmp_loc % Gimg.width;//before
	int y1 = tmp_loc / Gimg.height;
	int x2 = loc % Gimg.width;    //after
	int y2 = loc / Gimg.height;
	float gmax = 0;
	int _a = 0;
	if (green(Gimg.pixels[tmp_loc]) == HORIZONTAL && green(Gimg.pixels[loc]) == VERTICAL) {
		if (y2 > y1)      //down
			_a = DOWNRIGHT;
		else if (y2 < y1) //up
			_a = UPLEFT;
		else {
			gmax = max(max(green(Gimg.pixels[loc - Gimg.width - 1]), green(Gimg.pixels[loc - Gimg.width]), green(Gimg.pixels[loc - Gimg.width + 1])),
				max(green(Gimg.pixels[loc + Gimg.width - 1]), green(Gimg.pixels[loc + Gimg.width]), green(Gimg.pixels[loc + Gimg.width + 1])));
			if (gmax == max(green(Gimg.pixels[loc - Gimg.width - 1]), green(Gimg.pixels[loc - Gimg.width]), green(Gimg.pixels[loc - Gimg.width + 1])))
				_a = UPLEFT;
			else
				_a = DOWNRIGHT;
		}
	}
	else if (green(Gimg.pixels[tmp_loc]) == VERTICAL && green(Gimg.pixels[loc]) == HORIZONTAL) {
		if (x2 > x1) //right
			_a = DOWNRIGHT;
		else if (x2 < x1) //left
			_a = UPLEFT;
		else {
			gmax = max(max(green(Gimg.pixels[loc - Gimg.width + 1]), green(Gimg.pixels[loc + 1]), green(Gimg.pixels[loc + Gimg.width + 1])),
				max(green(Gimg.pixels[loc - Gimg.width - 1]), green(Gimg.pixels[loc - 1]), green(Gimg.pixels[loc + Gimg.width - 1])));
			if (gmax == max(green(Gimg.pixels[loc - Gimg.width + 1]), green(Gimg.pixels[loc + 1]), green(Gimg.pixels[loc + Gimg.width + 1])))
				_a = DOWNRIGHT;
			else
				_a = UPLEFT;
		}
	}
	else
		_a = a;
	if (_a == UPLEFT)
		return -loc;
	return loc;
}

int getTheta1(int _x, int _y) {
	double x = _x / 90.0;
	double y = (img_height - _y) / 90.0;
	double R1 = 15;
	double R2 = 16.07;

	double r = 13.166510146833373;
	double phi = 0.5438614148195791;
	double y1 = y * Math.cos(phi) - x * Math.sin(phi) - r * Math.sin(phi) - 14.5 * Math.cos(phi);
	double x1 = y * Math.sin(phi) + x * Math.cos(phi) + r * Math.cos(phi) - 14.5 * Math.sin(phi);

	double theta2 = -Math.acos((Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(R1, 2) - Math.pow(R2, 2)) / (2 * R1 * R2));
	double theta1 = ((R1 + R2 * Math.cos(theta2)) * y1 - R2 * Math.sin(theta2) * x1) / (R2 * Math.sin(theta2) * y1 + (R1 + R2 * Math.cos(theta2)) * x1);
	theta1 = Math.toDegrees(theta1);
	return (int)Math.round(16.0 * theta1 / 1.8 * 5.0);
}

int getTheta2(int _x, int _y) {
	double x = _x / 90.0;
	double y = (img_height - _y) / 90.0;
	double R1 = 15;
	double R2 = 16.07;

	double r = 13.166510146833373;
	double phi = 0.5438614148195791;
	double y1 = y * Math.cos(phi) - x * Math.sin(phi) - r * Math.sin(phi) - 14.5 * Math.cos(phi);
	double x1 = y * Math.sin(phi) + x * Math.cos(phi) + r * Math.cos(phi) - 14.5 * Math.sin(phi);

	double theta2 = -Math.acos((Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(R1, 2) - Math.pow(R2, 2)) / (2 * R1 * R2));
	theta2 = Math.toDegrees(theta2);
	return (int)Math.round(16.0 * theta2 / 1.8 * 5.0);
}


PImage loadImg(String imagePath) {
	PImage img = loadImage(imagePath);
	//resize image
	if (img.width / 870 > img.height / 450)
		img.resize(870, 0);
	else img.resize(0, 450);
	img_width = img.width;
	img_height = img.height;
	//calcultate gradient
	img.filter(BLUR, 1);
	return img;
}

void getGradient(MAt img, Mat Gimg) {
	for (int y = 1; y < img.height - 1; y++) { // Skip top and bottom edges
		for (int x = 1; x < img.width - 1; x++) { // Skip left and right edges
			int loc = y * img.width + x;
			float Gx = brightness(img.pixels[loc + 1]) - brightness(img.pixels[loc - 1]);
			float Gy = brightness(img.pixels[loc - img.width]) - brightness(img.pixels[loc + img.width]);
			float Gm = sqrt(sq(Gx) + sq(Gy));
			//RED: gradient magnitude, GREEN: gradient direction, BLUE: anchor
			if (abs(Gx) > abs(Gy))  //Vertical Edge
				Gimg.pixels[loc] = color(Gm, VERTICAL, 0);
			else
				Gimg.pixels[loc] = color(Gm, HORIZONTAL, 0);
		}
	}
}

void anchors(Mat Gimg, vector<Integer> & anchors) {
	Gimg.loadPixels();
	for (int y = 1; y < Gimg.height - 1; y++) { // Skip top and bottom edges
		for (int x = 1; x < Gimg.width - 1; x++) { // Skip left and right edges
			int loc = y * Gimg.width + x;
			//get max gradient
			float Gmax = 0;
			if (green(Gimg.pixels[loc]) == VERTICAL) { //vertical
				for (int i = max(1, y - interval); i < min(Gimg.height, y + interval + 1); i++) {
					if (i != y) //if i is not itself
						Gmax = max(Gmax, red(Gimg.pixels[i * Gimg.width + x]));
				}
				//check anchor
				if (red(Gimg.pixels[loc]) >= Gmax + t_anchor &&
					blue(Gimg.pixels[loc - Gimg.width]) == 0 && blue(Gimg.pixels[loc - 1]) == 0)
					anchors.add(loc);
			}
			else { //horizontal
				for (int i = max(1, x - interval); i < min(Gimg.width, x + interval + 1); i++) {
					if (i != x) //if max is not itself
						Gmax = max(Gmax, red(Gimg.pixels[y * Gimg.width + i]));
				}
				if (red(Gimg.pixels[loc]) >= Gmax + t_anchor &&
					blue(Gimg.pixels[loc - Gimg.width]) == 0 && blue(Gimg.pixels[loc - 1]) == 0)
					anchors.add(loc);
			}
		}
	}
}

void emptyImg(Mat _img) {
	Mat img = createImage(_img.width, _img.height, RGB);
	img.loadPixels();
	for (int i = 1; i < img.height - 1; i++) { // Skip top and bottom edges
		for (int j = 1; j < img.width - 1; j++) {
			img.set(0, 0, 0);
		}
	}
	img.updatePixels();
}

void sortDots(vector<Line> line_list, vector<Dot> & dots) {
	Line[][] Edge = new Line[img_width][img_height];
	for (int i = 0; i < img_width; i++)
		Arrays.fill(Edge[i], null);
	for (int i = 0; i < line_list.size(); i++) {
		Line line = line_list.get(i);
		Edge[line.start.x][line.start.y] = line;
		Edge[line.end.x][line.end.y] = line;
	}
	Vector<Dot> dots = new Vector<Dot>();
	Line line = line_list.get(0);
	Dot dot = line.end;
	line_list.remove(0);
	for (int i = 0; i < line.size(); i++)
		dots.add(dots.size(), line.get(i));
	for (int n = 0; n < line_list.size(); n++) {
		if (dot == null) break;
		Edge[line.start.x][line.start.y] = null;
		Edge[line.end.x][line.end.y] = null;
		int x_end = dot.x;
		int y_end = dot.y;
		int i = 1;
		dot = null;
		while (dot == null) {
			for (int x = max(1, x_end - i); x <= min(x_end + i, img_width - 2); x++) {
				if (y_end - i > 0 && Edge[x][y_end - i] != null) {
					line = Edge[x][y_end - i];
					if (i != 1) dots.get(dots.size() - 1).servo = 20;
					if (line.start.x == x && line.start.y == y_end - i) {
						dots.addAll(line.dots);
						dot = line.end;
					}
					else {
						Collections.reverse(line.dots);
						dots.addAll(line.dots);
						dot = line.start;
					}
				}
				else if (y_end + i < img_height - 1 && Edge[x][y_end + i] != null) {
					line = Edge[x][y_end + i];
					if (i != 1) dots.get(dots.size() - 1).servo = 20;
					if (line.start.x == x && line.start.y == y_end + i) {
						dots.addAll(line.dots);
						dot = line.end;
					}
					else {
						Collections.reverse(line.dots);
						dots.addAll(line.dots);
						dot = line.start;
					}
				}
			}
			if (dot == null) {
				for (int y = max(1, y_end - i); y <= min(img_height - 2, y_end + i); y++) {
					if (x_end - i > 0 && Edge[x_end - i][y] != null) {
						line = Edge[x_end - i][y];
						if (i != 1) dots.get(dots.size() - 1).servo = 20;
						if (line.start.x == x_end - i && line.start.y == y) {
							dots.addAll(line.dots);
							dot = line.end;
						}
						else {
							Collections.reverse(line.dots);
							dots.addAll(line.dots);
							dot = line.start;
						}
					}
					else if (x_end + i < img_width - 1 && Edge[x_end + i][y] != null) {
						line = Edge[x_end + i][y];
						if (i != 1) dots.get(dots.size() - 1).servo = 20;
						if (line.start.x == x_end + i && line.start.y == y) {
							dots.addAll(line.dots);
							dot = line.end;
						}
						else {
							Collections.reverse(line.dots);
							dots.addAll(line.dots);
							dot = line.start;
						}
					}
				}
			}
			i++;
		}
	}
	dots.at(dots.size() - 1).servo = 20;
}
