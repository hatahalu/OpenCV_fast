// 等速直線運動

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <math.h>
#include <opencv/highgui.h>

typedef struct nodes{
	int cost;
	int pre_cost;
	int result_cost;
	bool available = true;
	double x_left;
	double x_right;
	double y_top;
	double y_bottom;
}Nodes;

void initNode(Nodes *a,int i,int j){
	a->x_left = i;
	a->x_right = i + 100;
	a->y_top = j;
	a->y_bottom = j + 100;
}

int main(void) {
	int width = 1000;
	int height = 1000;
	double dt = 1 / 30;
	int key = 0;
	IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);	// width x height サイズのカラー画像を生成
	cvNamedWindow("Output");

	Nodes node[10][10];

	for (int i = 0; i <= 1000; i += 100){
		for (int j = 0; j <= 1000; j += 100){
			initNode(&node[i][j], i, j);
		}
	}

	Nodes start_node = node[0][0];
	Nodes goal_node = node[10][10];
	Nodes current_node = start_node;

	while (1) {
		cvSetZero(img);
		for (int i = 0; i <= 10; i++) {
		cvLine(img, cvPoint(0, 100 * i), cvPoint(1000, 100 * i), cvScalar(150, 150, 150), 2, 1);
		cvLine(img, cvPoint(100 * i, 0), cvPoint(100 * i,1000), cvScalar(150, 150, 150), 2, 1);
		}	
		cvShowImage("Output", img);		
		// 次フレームまで待つ + キー入力受付
		key = cvWaitKey(dt * 1000);// キー入力待ち(ms 単位)
		if (key == ' ')// スペースキーを押したら終了
			break;
	}

	return 0;
}


///ちゃっとじー
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

#define GRID_SIZE 10
#define CELL_SIZE 100
#define WIDTH (GRID_SIZE * CELL_SIZE)
#define HEIGHT (GRID_SIZE * CELL_SIZE)

typedef struct nodes {
    int cost;
    int heuristic;
    int total_cost;
    bool available;
    int x, y;
    struct nodes* parent;
} Nodes;

IplImage* img;
Nodes node[GRID_SIZE][GRID_SIZE];
Nodes* start_node;
Nodes* goal_node;

bool show_path = false;  // 経路表示フラグ

// マウスコールバック関数
void mouseHandler(int event, int x, int y, int flags, void* param) {
    int grid_x = x / CELL_SIZE;
    int grid_y = y / CELL_SIZE;

    if (grid_x < GRID_SIZE && grid_y < GRID_SIZE) {
        if (event == CV_EVENT_LBUTTONDOWN) {  // 左クリック: 障害物を追加
            node[grid_y][grid_x].available = false;
        }
        else if (event == CV_EVENT_RBUTTONDOWN) {  // 右クリック: 障害物を削除
            node[grid_y][grid_x].available = true;
        }
    }
}

// A*アルゴリズム
void a_star() {
    Nodes* open_list[GRID_SIZE * GRID_SIZE];
    bool closed_list[GRID_SIZE][GRID_SIZE] = { false };
    int open_count = 0;

    start_node->cost = 0;
    start_node->heuristic = abs(goal_node->x - start_node->x) + abs(goal_node->y - start_node->y);
    start_node->total_cost = start_node->heuristic;
    open_list[open_count++] = start_node;

    while (open_count > 0) {
        int min_index = 0;
        for (int i = 1; i < open_count; i++) {
            if (open_list[i]->total_cost < open_list[min_index]->total_cost) {
                min_index = i;
            }
        }

        Nodes* current = open_list[min_index];
        open_list[min_index] = open_list[--open_count];

        if (current == goal_node) {
            return;
        }

        closed_list[current->y][current->x] = true;

        int directions[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };
        for (int i = 0; i < 4; i++) {
            int new_x = current->x + directions[i][0];
            int new_y = current->y + directions[i][1];

            if (new_x < 0 || new_x >= GRID_SIZE || new_y < 0 || new_y >= GRID_SIZE) continue;
            if (!node[new_y][new_x].available || closed_list[new_y][new_x]) continue;

            Nodes* neighbor = &node[new_y][new_x];
            int new_cost = current->cost + 1;

            if (neighbor->parent == NULL || new_cost < neighbor->cost) {
                neighbor->cost = new_cost;
                neighbor->heuristic = abs(goal_node->x - neighbor->x) + abs(goal_node->y - neighbor->y);
                neighbor->total_cost = neighbor->cost + neighbor->heuristic;
                neighbor->parent = current;

                open_list[open_count++] = neighbor;
            }
        }
    }
}

// 経路を描画
void drawPath(Nodes* node) {
    while (node->parent != NULL) {
        cvLine(img,
            cvPoint(node->x * CELL_SIZE + CELL_SIZE / 2, node->y * CELL_SIZE + CELL_SIZE / 2),
            cvPoint(node->parent->x * CELL_SIZE + CELL_SIZE / 2, node->parent->y * CELL_SIZE + CELL_SIZE / 2),
            cvScalar(0, 255, 0), 2, 8);
        node = node->parent;
    }
}

int main(void) {
    img = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
    cvNamedWindow("A* Pathfinding");
    cvSetMouseCallback("A* Pathfinding", mouseHandler, NULL);

    // ノードの初期化
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            node[i][j].x = j;
            node[i][j].y = i;
            node[i][j].available = true;
            node[i][j].parent = NULL;
        }
    }
    start_node = &node[0][0];
    goal_node = &node[GRID_SIZE - 1][GRID_SIZE - 1];

    // フォント設定
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2, 8);

    int key = 0;
    while (key != 27) {  // ESCキーで終了
        cvSet(img, cvScalar(255, 255, 255));  // 背景を白色に設定

        // グリッド線を描画 (黒色)
        for (int i = 0; i <= GRID_SIZE; i++) {
            cvLine(img, cvPoint(0, i * CELL_SIZE), cvPoint(WIDTH, i * CELL_SIZE), cvScalar(0, 0, 0), 1);
            cvLine(img, cvPoint(i * CELL_SIZE, 0), cvPoint(i * CELL_SIZE, HEIGHT), cvScalar(0, 0, 0), 1);
        }

        // 障害物を描画
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (!node[i][j].available) {
                    cvRectangle(img,
                        cvPoint(j * CELL_SIZE, i * CELL_SIZE),
                        cvPoint((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE),
                        cvScalar(0, 0, 255), CV_FILLED);
                }
            }
        }

        // スタートとゴールの文字を描画
        cvPutText(img, "START", cvPoint(10, 40), &font, cvScalar(0, 0, 0));
        cvPutText(img, "GOAL", cvPoint(WIDTH - 90, HEIGHT - 20), &font, cvScalar(0, 0, 0));

        // 経路を描画
        if (show_path) {
            drawPath(goal_node);
        }

        cvShowImage("A* Pathfinding", img);

        key = cvWaitKey(10);
        if (key == 13) {  // Enterキー
            for (int i = 0; i < GRID_SIZE; i++) {
                for (int j = 0; j < GRID_SIZE; j++) {
                    node[i][j].parent = NULL;
                }
            }
            a_star();
            show_path = true;
        }
    }

    cvReleaseImage(&img);
    cvDestroyWindow("A* Pathfinding");
    return 0;
}
