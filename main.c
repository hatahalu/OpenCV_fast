
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define GRID_SIZE 10
#define INF 1e9

typedef struct {
    int x, y;
} Point;

typedef struct {
    Point pt;
    double cost;
    double heuristic;
    Point parent;
} Node;

// グリッドデータ
int grid[GRID_SIZE][GRID_SIZE] = {0};

// A*アルゴリズム用
bool visited[GRID_SIZE][GRID_SIZE] = {false};
Point parent[GRID_SIZE][GRID_SIZE];

// 4方向移動用
Point directions[] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

// 距離計算
double distance(Point a, Point b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// A*アルゴリズム実装
bool a_star(Point start, Point goal) {
    double cost[GRID_SIZE][GRID_SIZE];
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            cost[i][j] = INF;
            parent[i][j] = (Point){-1, -1};
            visited[i][j] = false;
        }
    }

    cost[start.y][start.x] = 0;
    Node openList[GRID_SIZE * GRID_SIZE];
    int openListSize = 0;

    openList[openListSize++] = (Node){start, 0, distance(start, goal), {-1, -1}};

    while (openListSize > 0) {
        // 最小コストのノードを取得
        int minIndex = 0;
        for (int i = 1; i < openListSize; i++) {
            if (openList[i].cost + openList[i].heuristic <
                openList[minIndex].cost + openList[minIndex].heuristic) {
                minIndex = i;
            }
        }

        Node current = openList[minIndex];
        Point pt = current.pt;

        // Openリストから削除
        openList[minIndex] = openList[--openListSize];

        if (visited[pt.y][pt.x]) continue;
        visited[pt.y][pt.x] = true;

        if (pt.x == goal.x && pt.y == goal.y) {
            return true;  // ゴールに到達
        }

        // 隣接ノードを処理
        for (int i = 0; i < 4; i++) {
            Point next = {pt.x + directions[i].x, pt.y + directions[i].y};

            if (next.x < 0 || next.x >= GRID_SIZE || next.y < 0 || next.y >= GRID_SIZE ||
                grid[next.y][next.x] == 1 || visited[next.y][next.x]) {
                continue;
            }

            double newCost = cost[pt.y][pt.x] + 1;

            if (newCost < cost[next.y][next.x]) {
                cost[next.y][next.x] = newCost;
                parent[next.y][next.x] = pt;

                openList[openListSize++] = (Node){next, newCost, distance(next, goal), pt};
            }
        }
    }

    return false;  // ゴールに到達できない
}

// 経路描画
void drawPath(Point goal) {
    Point p = goal;
    while (parent[p.y][p.x].x != -1) {
        if (grid[p.y][p.x] == 0) grid[p.y][p.x] = 2;  // 経路マーク
        p = parent[p.y][p.x];
    }
}

// グリッド描画
void drawGrid() {
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (grid[i][j] == 1) {
                printf("# ");  // 障害物
            } else if (grid[i][j] == 2) {
                printf("* ");  // 経路
            } else {
                printf(". ");  // 通常マス
            }
        }
        printf("\n");
    }
}

int main() {
    Point start = {0, 0};
    Point goal = {GRID_SIZE - 1, GRID_SIZE - 1};

    int obstacles;
    printf("障害物の数を入力してください: ");
    scanf("%d", &obstacles);

    for (int i = 0; i < obstacles; i++) {
        int x, y;
        printf("障害物の位置 (x y): ");
        scanf("%d %d", &x, &y);
        if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
            grid[y][x] = 1;
        }
    }

    if (a_star(start, goal)) {
        drawPath(goal);
        printf("\n経路を見つけました:\n");
    } else {
        printf("\n経路が見つかりませんでした。\n");
    }

    drawGrid();
    return 0;
}


#include <opencv2/opencv.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define GRID_SIZE 10
#define CELL_SIZE 50
#define WINDOW_SIZE (GRID_SIZE * CELL_SIZE)

// マスの状態
#define EMPTY 0
#define OBSTACLE 1
#define PATH 2

// グリッド情報
int grid[GRID_SIZE][GRID_SIZE] = {0};

// スタートとゴール
CvPoint start = {0, 0};
CvPoint goal = {GRID_SIZE - 1, GRID_SIZE - 1};

// マウスクリックで障害物を設定
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        int col = x / CELL_SIZE;
        int row = y / CELL_SIZE;

        if (grid[row][col] == EMPTY) {
            grid[row][col] = OBSTACLE; // 障害物にする
        }
    }
}

// ノード情報
typedef struct {
    CvPoint pt;
    double cost, heuristic;
    CvPoint parent;
} Node;

// A*アルゴリズム用のヘルパー関数
double distance(CvPoint a, CvPoint b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool isValid(CvPoint pt) {
    return pt.x >= 0 && pt.x < GRID_SIZE && pt.y >= 0 && pt.y < GRID_SIZE && grid[pt.y][pt.x] != OBSTACLE;
}

// A*アルゴリズム
bool aStar(CvPoint start, CvPoint goal) {
    bool visited[GRID_SIZE][GRID_SIZE] = {false};
    CvPoint parent[GRID_SIZE][GRID_SIZE];
    double cost[GRID_SIZE][GRID_SIZE];

    // 初期化
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            cost[i][j] = INFINITY;
            parent[i][j] = (CvPoint){-1, -1};
        }
    }

    // オープンリスト
    Node openList[GRID_SIZE * GRID_SIZE];
    int openListSize = 0;

    // スタート地点を追加
    cost[start.y][start.x] = 0;
    openList[openListSize++] = (Node){start, 0, distance(start, goal), {-1, -1}};

    while (openListSize > 0) {
        // 最小コストのノードを取得
        int minIndex = 0;
        for (int i = 1; i < openListSize; i++) {
            if (openList[i].cost + openList[i].heuristic <
                openList[minIndex].cost + openList[minIndex].heuristic) {
                minIndex = i;
            }
        }

        Node current = openList[minIndex];
        CvPoint pt = current.pt;

        // オープンリストから削除
        openList[minIndex] = openList[--openListSize];

        if (visited[pt.y][pt.x]) continue;
        visited[pt.y][pt.x] = true;

        // ゴールに到達
        if (pt.x == goal.x && pt.y == goal.y) {
            while (parent[pt.y][pt.x].x != -1) {
                grid[pt.y][pt.x] = PATH;
                pt = parent[pt.y][pt.x];
            }
            return true;
        }

        // 隣接ノードを処理
        CvPoint directions[] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (int i = 0; i < 4; i++) {
            CvPoint next = {pt.x + directions[i].x, pt.y + directions[i].y};

            if (isValid(next) && !visited[next.y][next.x]) {
                double newCost = cost[pt.y][pt.x] + 1;

                if (newCost < cost[next.y][next.x]) {
                    cost[next.y][next.x] = newCost;
                    parent[next.y][next.x] = pt;

                    openList[openListSize++] = (Node){next, newCost, distance(next, goal), pt};
                }
            }
        }
    }

    return false;
}

// グリッド描画
void drawGrid(IplImage* img) {
    cvSet(img, cvScalar(255, 255, 255));

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            CvPoint topLeft = {j * CELL_SIZE, i * CELL_SIZE};
            CvPoint bottomRight = {(j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE};

            if (grid[i][j] == OBSTACLE) {
                cvRectangle(img, topLeft, bottomRight, CV_RGB(255, 0, 0), CV_FILLED);
            } else if (grid[i][j] == PATH) {
                cvRectangle(img, topLeft, bottomRight, CV_RGB(255, 255, 0), CV_FILLED);
            }

            cvRectangle(img, topLeft, bottomRight, CV_RGB(0, 0, 0), 1);
        }
    }
}

// メイン関数
int main() {
    // ウィンドウと画像の作成
    IplImage* img = cvCreateImage(cvSize(WINDOW_SIZE, WINDOW_SIZE), IPL_DEPTH_8U, 3);
    cvNamedWindow("A* Pathfinding", CV_WINDOW_AUTOSIZE);

    cvSetMouseCallback("A* Pathfinding", mouseCallback);

    while (1) {
        drawGrid(img);
        cvShowImage("A* Pathfinding", img);

        int key = cvWaitKey(1);
        if (key == 's') {  // スタートキー
            if (aStar(start, goal)) {
                printf("経路が見つかりました。\n");
            } else {
                printf("経路が見つかりませんでした。\n");
            }
        } else if (key == 27) {  // ESCキー
            break;
        }
    }

    cvReleaseImage(&img);
    cvDestroyAllWindows();
    return 0;
}


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define GRID_SIZE 10
#define CELL_SIZE 50
#define WINDOW_SIZE (GRID_SIZE * CELL_SIZE)

// マスの状態
#define EMPTY 0
#define OBSTACLE 1
#define PATH 2

// グリッド情報
int grid[GRID_SIZE][GRID_SIZE] = {0};

// マウスクリックで障害物を設定
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        int col = x / CELL_SIZE;
        int row = y / CELL_SIZE;

        if (row >= 0 && row < GRID_SIZE && col >= 0 && col < GRID_SIZE) {
            if (grid[row][col] == EMPTY) {
                grid[row][col] = OBSTACLE; // 障害物にする
                printf("Obstacle set at (%d, %d)\n", row, col);
            }
        }
    }
}

// グリッド描画関数
void drawGrid(IplImage* img) {
    cvSet(img, cvScalar(255, 255, 255)); // 背景を白色に

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            CvPoint topLeft = cvPoint(j * CELL_SIZE, i * CELL_SIZE);
            CvPoint bottomRight = cvPoint((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE);

            // 障害物の描画
            if (grid[i][j] == OBSTACLE) {
                cvRectangle(img, topLeft, bottomRight, CV_RGB(255, 0, 0), CV_FILLED);
            } else if (grid[i][j] == PATH) {
                cvRectangle(img, topLeft, bottomRight, CV_RGB(255, 255, 0), CV_FILLED);
            }

            // グリッド線を黒色で描画
            cvRectangle(img, topLeft, bottomRight, CV_RGB(0, 0, 0), 1);
        }
    }
}

int main() {
    // OpenCVウィンドウと画像の作成
    IplImage* img = cvCreateImage(cvSize(WINDOW_SIZE, WINDOW_SIZE), IPL_DEPTH_8U, 3);
    if (!img) {
        fprintf(stderr, "Error: Unable to create image.\n");
        return 1;
    }

    cvNamedWindow("A* Pathfinding", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("A* Pathfinding", mouseCallback, NULL);

    while (1) {
        drawGrid(img);
        cvShowImage("A* Pathfinding", img);

        int key = cvWaitKey(10);
        if (key == 27) { // ESCキーで終了
            break;
        }
    }

    cvReleaseImage(&img);
    cvDestroyAllWindows();
    return 0;
}

