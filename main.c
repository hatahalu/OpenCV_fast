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

void mouseHandler(int event, int x, int y, int flags, void* param);
void a_star();
void drawPath(Nodes* node);


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
        cvCircle(img, cvPoint(50, 50), 50, cvScalar(0, 255, 0),-1);
        cvCircle(img, cvPoint(WIDTH - 50, HEIGHT - 50), 50, cvScalar(0, 255, 0),-1);

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
