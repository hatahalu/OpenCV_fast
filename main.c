#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

// グリッドのサイズ（10x10のグリッド）とセルのサイズ（100x100ピクセル）
#define GRID_SIZE 10
#define CELL_SIZE 100
#define WIDTH (GRID_SIZE * CELL_SIZE)  // 画面の幅（グリッドの幅 * セルのサイズ）
#define HEIGHT (GRID_SIZE * CELL_SIZE)  // 画面の高さ（グリッドの高さ * セルのサイズ）

// ノード構造体
typedef struct nodes {
    int cost;           // 現在のノードまでのコスト（スタートノードからの移動コスト）
    int heuristic;      // ヒューリスティックコスト（ゴールノードまでの推定コスト）
    int total_cost;     // 総コスト（cost + heuristic）
    bool available;     // このセルが通行可能かどうか（障害物か通路か）
    int x, y;           // ノードの位置（x座標、y座標）
    struct nodes* parent;  // このノードの親ノードへのポインタ（経路をたどるため）
} Nodes;

// 画像の宣言と、ノード、スタートノード、ゴールノード、経路表示フラグの設定
IplImage* img;  // 画像データの格納先
Nodes node[GRID_SIZE][GRID_SIZE];  // グリッド（2D配列）のノード
Nodes* start_node;  // スタートノードのポインタ
Nodes* goal_node;   // ゴールノードのポインタ

bool show_path = false;  // 経路表示フラグ。A*アルゴリズムの結果が表示されるかどうか

// マウスのクリックイベントを処理するコールバック関数
void mouseHandler(int event, int x, int y, int flags, void* param) {
    // マウス座標をグリッド座標に変換（セル内の位置）
    int grid_x = x / CELL_SIZE;
    int grid_y = y / CELL_SIZE;

    // グリッド内の有効なセルを選択した場合
    if (grid_x < GRID_SIZE && grid_y < GRID_SIZE) {
        if (event == CV_EVENT_LBUTTONDOWN) {  // 左クリック: 障害物を追加
            node[grid_y][grid_x].available = false;  // 通行不可（障害物）に設定
        }
        else if (event == CV_EVENT_RBUTTONDOWN) {  // 右クリック: 障害物を削除
            node[grid_y][grid_x].available = true;  // 通行可能（空き地）に設定
        }
    }
}

// A*アルゴリズムによる経路探索
void a_star() {
    // オープンリスト（探索待ちのノード）とクローズリスト（探索済みのノード）
    Nodes* open_list[GRID_SIZE * GRID_SIZE];  // オープンリストの最大サイズ
    bool closed_list[GRID_SIZE][GRID_SIZE] = { false };  // すべてのノードを最初は探索していないと設定
    int open_count = 0;  // オープンリストに入っているノードの数

    // スタートノードのコストとヒューリスティックを設定
    start_node->cost = 0;  // スタートノードの移動コストは0
    start_node->heuristic = abs(goal_node->x - start_node->x) + abs(goal_node->y - start_node->y);  // マンハッタン距離
    start_node->total_cost = start_node->cost + start_node->heuristic;  // 総コスト（移動コスト + ヒューリスティック）
    open_list[open_count++] = start_node;  // スタートノードをオープンリストに追加

    // オープンリストが空でない限り繰り返す
    while (open_count > 0) {
        // オープンリスト内の最小の総コストを持つノードを探す
        int min_index = 0;
        for (int i = 1; i < open_count; i++) {
            if (open_list[i]->total_cost < open_list[min_index]->total_cost) {
                min_index = i;  // 最小コストノードのインデックスを更新
            }
        }

        // 最小コストノードをオープンリストから取り出す
        Nodes* current = open_list[min_index];
        open_list[min_index] = open_list[--open_count];  // 取り出したノードをオープンリストから削除

        // ゴールノードに到達した場合、探索終了
        if (current == goal_node) {
            return;
        }

        // 現在のノードをクローズリストに追加
        closed_list[current->y][current->x] = true;

        // 4方向（上下左右）の隣接ノードをチェック
        int directions[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };
        for (int i = 0; i < 4; i++) {
            int new_x = current->x + directions[i][0];  // 新しいx座標
            int new_y = current->y + directions[i][1];  // 新しいy座標

            // 新しいノードがグリッド外の場合はスキップ
            if (new_x < 0 || new_x >= GRID_SIZE || new_y < 0 || new_y >= GRID_SIZE) continue;

            // 障害物がある場合、または既に探索済みの場合はスキップ
            if (!node[new_y][new_x].available || closed_list[new_y][new_x]) continue;

            Nodes* neighbor = &node[new_y][new_x];  // 隣接ノードのポインタを取得
            int new_cost = current->cost + 1;  // 新しいコスト（1マス進むため）

            // 新しいコストが現在のノードより小さい場合、または親ノードが設定されていない場合に親ノードを更新
            if (neighbor->parent == NULL || new_cost < neighbor->cost) {
                neighbor->cost = new_cost;  // 新しいコストを設定
                neighbor->heuristic = abs(goal_node->x - neighbor->x) + abs(goal_node->y - neighbor->y);  // ヒューリスティックを再計算
                neighbor->total_cost = neighbor->cost + neighbor->heuristic;  // 総コストを更新
                neighbor->parent = current;  // 親ノードを現在のノードに設定

                // 新しいノードをオープンリストに追加
                open_list[open_count++] = neighbor;
            }
        }
    }
}

// 経路を描画する関数
void drawPath(Nodes* node) {
    // 経路を親ノードをたどって描画する
    while (node->parent != NULL) {
        cvLine(img,
            cvPoint(node->x * CELL_SIZE + CELL_SIZE / 2, node->y * CELL_SIZE + CELL_SIZE / 2),  // 現在のノードの中心座標
            cvPoint(node->parent->x * CELL_SIZE + CELL_SIZE / 2, node->parent->y * CELL_SIZE + CELL_SIZE / 2),  // 親ノードの中心座標
            cvScalar(0, 255, 0), 2, 8);  // 緑色の線で描画
        node = node->parent;  // 親ノードに移動
    }
}

int main(void) {
    // 画像の作成とウィンドウの初期設定
    img = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
    cvNamedWindow("A* Pathfinding");  // ウィンドウの作成
    cvSetMouseCallback("A* Pathfinding", mouseHandler, NULL);  // マウスイベントのコールバックを設定

    // ノードの初期化
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            node[i][j].x = j;  // x座標
            node[i][j].y = i;  // y座標
            node[i][j].available = true;  // 通行可能に設定
            node[i][j].parent = NULL;  // 親ノードを初期化
        }
    }

    // スタートノードとゴールノードを設定
    start_node = &node[0][0];  // (0, 0)の位置をスタートノードに設定
    goal_node = &node[GRID_SIZE - 1][GRID_SIZE - 1];  // (9, 9)の位置をゴールノードに設定

    // フォントの設定
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2, 8);  // フォントの種類とサイズを設定

    int key = 0;
    while (key != 27) {  // ESCキーが押されるまでループ
        cvSet(img, cvScalar(255, 255, 255));  // 画面の背景色を白に設定

        // グリッド線を描画（黒色）
        for (int i = 0; i <= GRID_SIZE; i++) {
            cvLine(img, cvPoint(0, i * CELL_SIZE), cvPoint(WIDTH, i * CELL_SIZE), cvScalar(0, 0, 0), 1);  // 横の線
            cvLine(img, cvPoint(i * CELL_SIZE, 0), cvPoint(i * CELL_SIZE, HEIGHT), cvScalar(0, 0, 0), 1);  // 縦の線
        }

        // 障害物を描画
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (!node[i][j].available) {  // 通行不可の場合（障害物）
                    cvRectangle(img,
                        cvPoint(j * CELL_SIZE, i * CELL_SIZE),
                        cvPoint((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE),
                        cvScalar(0, 0, 255), CV_FILLED);  // 赤色で塗りつぶし
                }
            }
        }

        // スタートとゴールの文字を描画
        cvPutText(img, "START", cvPoint(10, 40), &font, cvScalar(0, 0, 0));  // スタート位置に「START」
        cvPutText(img, "GOAL", cvPoint(WIDTH - 90, HEIGHT - 20), &font, cvScalar(0, 0, 0));  // ゴール位置に「GOAL」

        // 経路を描画
        if (show_path) {
            drawPath(goal_node);  // 経路の描画（A*アルゴリズムの結果）
        }

        // 画像を表示
        cvShowImage("A* Pathfinding", img);

        // キー入力を待つ
        key = cvWaitKey(10);
        if (key == 13) {  // Enterキーが押されたらA*を実行
            // ノードの親ポインタをリセット
            for (int i = 0; i < GRID_SIZE; i++) {
                for (int j = 0; j < GRID_SIZE; j++) {
                    node[i][j].parent = NULL;
                }
            }
            // A*アルゴリズムを実行
            a_star();
            show_path = true;  // 経路表示フラグをオン
        }
    }

    // 画像とウィンドウの解放
    cvReleaseImage(&img);
    cvDestroyWindow("A* Pathfinding");
    return 0;
}
