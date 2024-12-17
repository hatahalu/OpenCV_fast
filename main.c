
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

// スタートとゴール
CvPoint start = {0, 0};
CvPoint goal = {GRID_SIZE - 1, GRID_SIZE - 1};

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

// 距離関数（ユークリッド距離）
double distance(CvPoint a, CvPoint b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// A*アルゴリズム用のノード
typedef struct {
    CvPoint pt;
    double cost, heuristic;
    CvPoint parent;
} Node;

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
            // 経路を復元
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

            if (next.x >= 0 && next.x < GRID_SIZE && next.y >= 0 && next.y < GRID_SIZE &&
                grid[next.y][next.x] != OBSTACLE && !visited[next.y][next.x]) {
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
                cvRectangle(img, topLeft, bottomRight, CV_RGB(0, 0, 255), CV_FILLED);
            }

            // グリッド線を描画
            cvRectangle(img, topLeft, bottomRight, CV_RGB(0, 0, 0), 1);
        }
    }
}

int main() {
    // OpenCVウィンドウと画像の作成
    IplImage* img = cvCreateImage(cvSize(WINDOW_SIZE, WINDOW_SIZE), IPL_DEPTH_8U, 3);
    cvNamedWindow("A* Pathfinding", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("A* Pathfinding", mouseCallback, NULL);

    while (1) {
        drawGrid(img);
        cvShowImage("A* Pathfinding", img);

        int key = cvWaitKey(10);
        if (key == 's') {  // スタートキー
            if (aStar(start, goal)) {
                printf("経路が見つかりました。\n");
            } else {
                printf("経路が見つかりませんでした。\n");
            }
        } else if (key == 27) {  // ESCキーで終了
            break;
        }
    }

    cvReleaseImage(&img);
    cvDestroyAllWindows();
    return 0;
}