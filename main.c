#define _CRT_SECURE_NO_WARNINGS // セキュリティ警告を無効化 (Visual Studio用)

#include <stdio.h>             // 標準入出力ライブラリ
#include <stdbool.h>           // ブール型を利用するためのヘッダー
#include <math.h>              // 数学関数を利用するためのヘッダー
#include <opencv2/opencv.hpp>  // OpenCVコアライブラリ
#include <opencv/highgui.h>    // OpenCV GUI操作用ライブラリ

#define GRID_SIZE 10            // グリッドのサイズ (10x10)
#define CELL_SIZE 100           // 各セルのサイズ (100ピクセル)
#define WIDTH (GRID_SIZE * CELL_SIZE)  // ウィンドウの幅
#define HEIGHT (GRID_SIZE * CELL_SIZE) // ウィンドウの高さ

// ノード構造体の定義
typedef struct nodes {
    int cost;              // 開始ノードからのコスト
    int heuristic;         // 推定値（ヒューリスティック）
    int total_cost;        // 合計コスト (cost + heuristic)
    bool available;        // 通行可能かどうか
    int x, y;              // ノードのグリッド座標
    struct nodes* parent;  // 親ノードへのポインタ
} Nodes;

// グローバル変数の宣言
IplImage* img;               // 描画用の画像
Nodes node[GRID_SIZE][GRID_SIZE]; // グリッド内の全ノード
Nodes* start_node;           // スタートノード
Nodes* goal_node;            // ゴールノード
bool show_path = false;      // 経路を表示するフラグ

// 関数の宣言
void mouseHandler(int event, int x, int y, int flags, void* param); // マウスイベントハンドラ
void a_star();                    // A*アルゴリズム
void drawPath(Nodes* node);       // 経路を描画する関数

// メイン関数
int main(void) {
    // 画像の作成
    img = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
    cvNamedWindow("A* Pathfinding"); // ウィンドウを作成
    cvSetMouseCallback("A* Pathfinding", mouseHandler, NULL); // マウスイベントコールバックを設定

    // ノードの初期化
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            node[i][j].x = j;             // ノードのx座標
            node[i][j].y = i;             // ノードのy座標
            node[i][j].available = true; // 通行可能に設定
            node[i][j].parent = NULL;    // 親ノードを初期化
        }
    }
    start_node = &node[0][0];                          // スタートノードを設定
    goal_node = &node[GRID_SIZE - 1][GRID_SIZE - 1];   // ゴールノードを設定

    // フォントの設定
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
                if (!node[i][j].available) { // 通行不能ノードを描画
                    cvRectangle(img,
                        cvPoint(j * CELL_SIZE, i * CELL_SIZE),
                        cvPoint((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE),
                        cvScalar(0, 0, 255), CV_FILLED); // 赤色で塗りつぶし
                }
            }
        }

        // スタートとゴールの文字を描画
        cvCircle(img, cvPoint(50, 50), 50, cvScalar(0, 255, 0), -1); // スタートノード (緑)
        cvCircle(img, cvPoint(WIDTH - 50, HEIGHT - 50), 50, cvScalar(0, 255, 0), -1); // ゴールノード (緑)

        // 経路を描画
        if (show_path) {
            drawPath(goal_node);
        }

        cvShowImage("A* Pathfinding", img); // 画像を表示

        key = cvWaitKey(10); // キー入力を待つ
        if (key == 13) {  // Enterキー
            for (int i = 0; i < GRID_SIZE; i++) {
                for (int j = 0; j < GRID_SIZE; j++) {
                    node[i][j].parent = NULL; // 親ノードをリセット
                }
            }
            a_star();       // A*アルゴリズムを実行
            show_path = true; // 経路を表示
        }
    }

    cvReleaseImage(&img);          // 画像メモリの解放
    cvDestroyWindow("A* Pathfinding"); // ウィンドウを破棄
    return 0;
}

// A*アルゴリズム
void a_star() {
    Nodes* open_list[GRID_SIZE * GRID_SIZE]; // オープンリスト
    bool closed_list[GRID_SIZE][GRID_SIZE] = { false }; // クローズドリスト
    int open_count = 0;

    // スタートノードの初期化
    start_node->cost = 0;
    start_node->heuristic = abs(goal_node->x - start_node->x) + abs(goal_node->y - start_node->y);
    start_node->total_cost = start_node->heuristic;
    open_list[open_count++] = start_node;

    while (open_count > 0) { // オープンリストに要素がある間
        int min_index = 0;
        for (int i = 1; i < open_count; i++) {
            if (open_list[i]->total_cost < open_list[min_index]->total_cost) {
                min_index = i; // 最小コストのノードを探す
            }
        }

        Nodes* current = open_list[min_index]; // 現在のノードを取得
        open_list[min_index] = open_list[--open_count]; // オープンリストを更新

        if (current == goal_node) { // ゴールに到達
            return;
        }

        closed_list[current->y][current->x] = true; // クローズドリストに追加

        int directions[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} }; // 移動方向
        for (int i = 0; i < 4; i++) {
            int new_x = current->x + directions[i][0];
            int new_y = current->y + directions[i][1];

            if (new_x < 0 || new_x >= GRID_SIZE || new_y < 0 || new_y >= GRID_SIZE) continue; // 範囲外
            if (!node[new_y][new_x].available || closed_list[new_y][new_x]) continue; // 通行不能または探索済み

            Nodes* neighbor = &node[new_y][new_x]; // 隣接ノード
            int new_cost = current->cost + 1;

            if (neighbor->parent == NULL || new_cost < neighbor->cost) {
                neighbor->cost = new_cost; // 新しいコストを更新
                neighbor->heuristic = abs(goal_node->x - neighbor->x) + abs(goal_node->y - neighbor->y);
                neighbor->total_cost = neighbor->cost + neighbor->heuristic;
                neighbor->parent = current; // 親ノードを設定

                open_list[open_count++] = neighbor; // オープンリストに追加
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
            cvScalar(0, 255, 0), 2, 8); // 緑色の線で経路を描画
        node = node->parent; // 次のノードに移動
    }
}

// マウスコールバック関数
void mouseHandler(int event, int x, int y, int flags, void* param) {
    int grid_x = x / CELL_SIZE; // グリッド座標x
    int grid_y = y / CELL_SIZE; // グリッド座標y

    if (grid_x < GRID_SIZE && grid_y < GRID_SIZE) {
        if (event == CV_EVENT_LBUTTONDOWN) {  // 左クリック: 障害物を追加
            node[grid_y][grid_x].available = false;
        }
        else if (event == CV_EVENT_RBUTTONDOWN) {  // 右クリック: 障害物を削除
            node[grid_y][grid_x].available = true;
        }
    }
}
