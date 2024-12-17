#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <opencv/highgui.h>
int key = 0;

int main() {
	int width = 1000;
	int height = 1000;

	IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);	
	cvNamedWindow("Output");	
	for (int i = 0; i <= 1000; i += 100) {
		cvLine(img, cvPoint(0, 0+i), cvPoint(1000, 0+i), cvScalar(0, 0, 255), 5, 1);
		cvLine(img, cvPoint(0+i, 0), cvPoint(0+i,1000), cvScalar(0, 0, 255), 5, 1);
	}
	cvShowImage("Output", img);

	while (1) {
		key = cvWaitKey(0);		
		if (key == ' ')
			break;

	}
	cvDestroyAllWindows();			
	cvReleaseImage(&img);			

	return 0;

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

#define GRID_SIZE 10
#define CELL_SIZE 50
#define WINDOW_SIZE (GRID_SIZE * CELL_SIZE)

using namespace cv;
using namespace std;

// グリッド情報
int grid[GRID_SIZE][GRID_SIZE] = {0}; // 0: 通過可能, 1: 障害物
Point start(0, 0);
Point goal(GRID_SIZE - 1, GRID_SIZE - 1);
bool start_set = false, goal_set = false;

// マウスコールバック関数
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        int col = x / CELL_SIZE;
        int row = y / CELL_SIZE;

        if (grid[row][col] == 0) {
            grid[row][col] = 1; // 障害物として設定
        }
    }
}

// ノード情報
struct Node {
    Point pt;
    double cost, heuristic;
    Node* parent;

    Node(Point p, double c, double h, Node* par = nullptr)
        : pt(p), cost(c), heuristic(h), parent(par) {}

    double totalCost() const {
        return cost + heuristic;
    }
};

// A*アルゴリズム
vector<Point> aStar(Point start, Point goal) {
    auto compare = [](Node* a, Node* b) { return a->totalCost() > b->totalCost(); };
    priority_queue<Node*, vector<Node*>, decltype(compare)> openList(compare);
    bool visited[GRID_SIZE][GRID_SIZE] = {false};

    openList.push(new Node(start, 0, norm(goal - start)));

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        Point pt = current->pt;
        if (visited[pt.y][pt.x])
            continue;

        visited[pt.y][pt.x] = true;

        if (pt == goal) {
            vector<Point> path;
            while (current) {
                path.push_back(current->pt);
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        static const Point directions[] = {
            Point(0, -1), Point(0, 1), Point(-1, 0), Point(1, 0)};

        for (const auto& dir : directions) {
            Point next = pt + dir;
            if (next.x >= 0 && next.x < GRID_SIZE && next.y >= 0 && next.y < GRID_SIZE &&
                grid[next.y][next.x] == 0 && !visited[next.y][next.x]) {
                double newCost = current->cost + 1;
                double heuristic = norm(goal - next);
                openList.push(new Node(next, newCost, heuristic, current));
            }
        }
    }

    return {};
}

// 描画
void drawGrid(Mat& img, const vector<Point>& path) {
    img = Mat::zeros(WINDOW_SIZE, WINDOW_SIZE, CV_8UC3);

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            Point topLeft(j * CELL_SIZE, i * CELL_SIZE);
            Point bottomRight((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE);

            if (grid[i][j] == 1) {
                rectangle(img, topLeft, bottomRight, Scalar(0, 0, 255), FILLED);
            } else {
                rectangle(img, topLeft, bottomRight, Scalar(255, 255, 255), 1);
            }
        }
    }

    for (const auto& pt : path) {
        Point topLeft(pt.x * CELL_SIZE, pt.y * CELL_SIZE);
        Point bottomRight((pt.x + 1) * CELL_SIZE, (pt.y + 1) * CELL_SIZE);
        rectangle(img, topLeft, bottomRight, Scalar(0, 255, 255), FILLED);
    }
}

int main() {
    Mat img;
    vector<Point> path;

    namedWindow("A* Pathfinding");
    setMouseCallback("A* Pathfinding", mouseCallback);

    while (true) {
        drawGrid(img, path);
        imshow("A* Pathfinding", img);

        int key = waitKey(1);
        if (key == 's') { // スタート位置設定
            path = aStar(start, goal);
        } else if (key == 27) { // ESCで終了
            break;
        }
    }

    return 0;
}
