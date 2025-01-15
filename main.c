#define _CRT_SECURE_NO_WARNINGS // Visual Studioでのセキュリティ警告を無効化

#include <stdio.h> // 標準ライブラリ
#include <stdbool.h> // 真偽値（bool型）を扱うためのライブラリ
#include <math.h> // 数学関数（absなど）を扱うためのライブラリ
#include <opencv2/opencv.hpp> // OpenCVの主要な機能を扱うヘッダーファイル
#include <opencv/highgui.h> // OpenCVで画像表示やGUI操作を行うためのヘッダーファイル

// 定数を定義
#define GRID_SIZE 10 // グリッドの縦横のサイズ（10x10のグリッド）
#define CELL_SIZE 100 // 各セルのピクセルサイズ
#define WIDTH (GRID_SIZE * CELL_SIZE) // 画像の横幅（セルサイズ × グリッド数）
#define HEIGHT (GRID_SIZE * CELL_SIZE) // 画像の縦幅（セルサイズ × グリッド数）

// ノード（セル）の情報を表す構造体
typedef struct nodes {
    int cost; // スタートノードから現在ノードまでの移動コスト
    int heuristic; // ヒューリスティック値（ゴールまでの推定コスト）
    int total_cost; // 合計コスト（cost + heuristic）
    bool available; // 通行可能かどうか（true: 通行可能, false: 障害物）
    bool passed; // 探索済みかどうか（true: 探索済み, false: 未探索）
    int x, y; // ノードのグリッド座標（x, y）
    struct nodes* parent; // 親ノード（経路の探索でこのノードに至る直前のノード）
} Nodes;

// グローバル変数
IplImage* img; // 画像データを保持する変数（OpenCV用）
Nodes node[GRID_SIZE][GRID_SIZE]; // グリッド上のノード（10x10のノード）　
Nodes* start_node; // スタートノード
Nodes* goal_node; // ゴールノード

// 関数のプロトタイプ宣言
int heuristic(Nodes a, Nodes b); // ヒューリスティック値を計算する関数
void Setcost(Nodes* a, int cost, int heuristic); // ノードのコストを設定する関数
void mouseHandler(int event, int x, int y, int flags, void* param); // マウスクリック時の処理を定義する関数
void a_star(Nodes* start_node, Nodes* goal_node); // A*アルゴリズムを実行する関数
void drawPath(Nodes* node); // ゴールからスタートノードまでの経路を描画する関数



int main(void) {
    // １００マス分の大きさの画像の生成する
    img = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
    // A＊という名前のwindowを作成
    cvNamedWindow("A*");
    // マウスコールバック関数を設定
    cvSetMouseCallback("A*", mouseHandler, NULL);

    // ノードの初期化をするためのfor文
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            // 各ノードの位置を設定
            node[i][j].x = j;
            node[i][j].y = i;
            // 初期状態ではすべてのノードが通行可能
            node[i][j].available = true;
            // 初期状態ではすべてのノードが未探索
            node[i][j].passed = false;
            // 親ノードをNULLに設定
            node[i][j].parent = NULL;
        }
    }
    // スタートノードをwindowの左上に設定
    start_node = &node[0][0];
    // ゴールノードをwindowの右下に設定（&node[GRID_SIZE][GRID_SIZE]だと存在しないマスがgoal_nodeになってしまうので-1)
    goal_node = &node[GRID_SIZE - 1][GRID_SIZE - 1];
    int key = 0;
    while (key != 27) { // ESCキーが押されるまでループする
        // 背景を白色に設定
        cvSet(img, cvScalar(255, 255, 255));
        // グリッド線を黒線で描画
        for (int i = 0; i <= GRID_SIZE; i++) {
            cvLine(img, cvPoint(0, i * CELL_SIZE), cvPoint(WIDTH, i * CELL_SIZE), cvScalar(0, 0, 0), 1);
            cvLine(img, cvPoint(i * CELL_SIZE, 0), cvPoint(i * CELL_SIZE, HEIGHT), cvScalar(0, 0, 0), 1);
        }
        // 障害物を描画する
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (!node[i][j].available) { // 障害物の場合
                    cvRectangle(img,
                        cvPoint(j * CELL_SIZE, i * CELL_SIZE),
                        cvPoint((j + 1) * CELL_SIZE, (i + 1) * CELL_SIZE),
                        cvScalar(0, 0, 255), CV_FILLED);
                }
            }
        }
        // スタートノードとゴールノードの緑色の円を描画
        cvCircle(img, cvPoint(50, 50), 50, cvScalar(0, 255, 0), -1); //スタートノード
        cvCircle(img, cvPoint(WIDTH - 50, HEIGHT - 50), 50, cvScalar(0, 255, 0), -1);//ゴールノード
        key = cvWaitKey(10); // キー入力を10ミリ秒待機する
        if (key == 13) { // Enterキーが押された場合(13はenterキーのASCIIコード）
            // 今までのノードを初期化する
            for (int i = 0; i < GRID_SIZE; i++) {
                for (int j = 0; j < GRID_SIZE; j++) {
                    node[i][j].parent = NULL; // 親ノードをクリア
                    node[i][j].passed = false; // 探索済みフラグをリセット
                }
            }
            // A*アルゴリズムを実行して経路を探索
            a_star(start_node, goal_node);
        }
        // 経路を描画
        drawPath(goal_node); // 経路が見つかった場合に経路を描画
        // ウィンドウに画像を表示する
        cvShowImage("A*", img);
    }
    // リソースを解放
    cvReleaseImage(&img);
    cvDestroyWindow("A*");
    return 0; // プログラム終了
}

int heuristic(Nodes a, Nodes b) {//ヒューリスティックを返す関数
    int heuristic = (abs(a.x - b.x)) + (abs(a.y - b.y));
    return heuristic;
}

void Setcost(Nodes* a, int cost, int heuristic) {//ノードのコストを設定する関数
    a->cost = cost;
    a->heuristic = heuristic;
    a->total_cost = cost + heuristic;
    return;
}


// A*アルゴリズム
void a_star(Nodes* start_node, Nodes* goal_node) {


    Nodes* open_nodes[GRID_SIZE * GRID_SIZE] = { nullptr };
    //Nodes* close_nodes[GRID_SIZE * GRID_SIZE];
    Nodes* current_node = start_node;//現在のノードをスタートノードで初期化
    int open_number = 0;//オープンされたノードの数(オープンリストに入っているノードの数)
    Setcost(current_node, 0, heuristic(*current_node, *goal_node));//現在のノード(スタートノード)の値を設定

    open_nodes[0] = current_node;//オープンリストの先頭に現在のノードを追加
    open_number++;//open_numberを1増やす

    int roopcount = 0;
    while (open_number > 0 && current_node != goal_node) {//オープンリストにノードがあるかぎり探索を続ける
        roopcount++;
        int mincost_node_index = 0;//最小コストのノードのインデックスを保持する変数
        for (int i = 0; i < open_number - 1; i++) {//for文でopen_nodes内を比較し、最小コストを探す
            if (open_nodes[mincost_node_index]->total_cost > open_nodes[i]->total_cost) {//最小コストノードの総コストi番目のノードの総コストを比較
                mincost_node_index = i;//i番目のノードの総コストがより小さければ、indexを更新
            }
            else if (open_nodes[mincost_node_index]->total_cost == open_nodes[i]->total_cost) {
                //総コストが同じなら
                if (open_nodes[mincost_node_index]->cost > open_nodes[i]->cost) {//コストを比較
                    mincost_node_index = i;//コストがより小さければ、indexを更新
                }

            }
        }

        Nodes* next_node = open_nodes[mincost_node_index];//最小コストのノードを次に移動するノードとする

        //close_nodes[close_number] = current_node;//現在のノードをクローズリストに追加
        current_node->passed = true;//通行済みにする
        open_nodes[mincost_node_index] = open_nodes[open_number - 1];//最小コストノードをオープンリストから削除(リストの最後尾のノードで上書きして"強引に","実質的"な削除をしている)
        open_number--;//通過したノードはもう探索しないので、open_numberをひとつ減らす
        //next_node->parent = current_node;//移動先のノードの親ノードを現在のノードに設定

        current_node = next_node;//現在のノードを移動先のノードに更新



        if (current_node == goal_node)return;//この時点でゴールノードに到達しているなら探索を終了する

        int new_cost = current_node->cost + 1;


        int neighbors_index[4][2] = { {1,0},{-1,0},{0,1},{0,-1} };
        for (int i = 0; i < 4; i++) {
            Nodes* neighbor;
            int neighbor_x = current_node->x + neighbors_index[i][0];
            int neighbor_y = current_node->y + neighbors_index[i][1];
            bool x_is_normal = (neighbor_x >= 0 && neighbor_x < 10);
            bool y_is_normal = (neighbor_y >= 0 && neighbor_y < 10);
            if (x_is_normal && y_is_normal) {
                neighbor = &node[neighbor_y][neighbor_x];
            }
            else
            {
                continue;
            }
            bool already_open = false;
            if (!neighbor->passed && neighbor->available) {
                for (int j = 0; j < open_number - 1; j++) {//オープンリストをforで探す

                    if (open_nodes[j] == neighbor) {//aがすでにオープンリストにあるなら
                        already_open = true;
                        break;
                    }

                }

                if (!already_open) {//already_openがfalse(未オープン)だったなら
                    open_nodes[open_number] = neighbor;//オープンリストに入れる
                    Setcost(neighbor, new_cost, heuristic(*neighbor, *goal_node));
                    neighbor->parent = current_node;
                    open_number++;//オープンしたノードの数を1増やす
                }
            }
        }

    }

}

void drawPath(Nodes* node) {
    // ゴールノードから親ノードをたどって経路を描画する
    while (node->parent != NULL) { //親ノードが空じゃなければ繰り返す
        int start_x_point = node->x * CELL_SIZE + CELL_SIZE / 2;
        int start_y_point = node->y * CELL_SIZE + CELL_SIZE / 2;
        int goal_x_point = node->parent->x * CELL_SIZE + CELL_SIZE / 2;
        int goal_y_point = node->parent->y * CELL_SIZE + CELL_SIZE / 2;
        cvLine(img,
            cvPoint(start_x_point, start_y_point), // 現在ノードの中心点の座標
            cvPoint(goal_x_point, goal_y_point), // 親ノードの中心点の座標
            cvScalar(0, 255, 0), 2, 8); // 緑色の線を描画
        node = node->parent; // 次のノード（親ノード）に進む
    }
}

void mouseHandler(int event, int x, int y, int flags, void* param) {
    // マウスクリック位置のグリッド座標を計算
    int grid_x = x / CELL_SIZE; //xが1だと、1/100=0.01となるがintに変換するのでgrid_xは0になる。
    int grid_y = y / CELL_SIZE; 
    // グリッド範囲内でクリックされた場合にのみ処理を実行
    if (grid_x < GRID_SIZE && grid_y < GRID_SIZE) {
        if (event == CV_EVENT_LBUTTONDOWN) { // 左クリックの場合
            node[grid_y][grid_x].available = false; // 障害物を追加（通行不可に設定）
        }
        else if (event == CV_EVENT_RBUTTONDOWN) { // 右クリックの場合
            node[grid_y][grid_x].available = true; // 障害物を削除（通行可能に設定）
        }
    }
}
