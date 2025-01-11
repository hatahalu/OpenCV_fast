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
    bool available = true;     // このセルが通行可能かどうか（障害物か通路か）
    bool passed = false;
    int x, y;           // ノードの位置（x座標、y座標）
    struct nodes* parent;  // このノードの親ノードへのポインタ（経路をたどるため）

} Nodes;

// 画像の宣言と、ノード、スタートノード、ゴールノード、経路表示フラグの設定
IplImage* img;  画像データの格納先
Nodes node[GRID_SIZE][GRID_SIZE];  // グリッド（2D配列）のノード
Nodes* start_node;  // スタートノードのポインタ
Nodes* goal_node;   // ゴールノードのポインタ

bool show_path = false;  // 経路表示フラグ。A*アルゴリズムの結果が表示されるかどうか

int heuristic(Nodes* a, Nodes* b) {//ヒューリスティックを返す関数
    int heuristic = (abs(a->x - b->x)) + (abs(a->y - b->y));
    return heuristic;
}

void Setcost(Nodes* a, int cost, int heuristic) {//ノードのコストを設定する関数
    a->cost = cost;
    a->heuristic = heuristic;
    a->total_cost = cost + heuristic;
    return;
}

void A_star(){

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            node[i][j].x = j;
            node[i][j].y = i;
            node[i][j].available = true;
            node[i][j].parent = NULL;
        }
    }

    Nodes* open_nodes[GRID_SIZE * GRID_SIZE];
    //Nodes* close_nodes[GRID_SIZE * GRID_SIZE];
    Nodes* current_node = start_node;//現在のノードをスタートノードで初期化
    int open_number = 0;//オープンされたノードの数(オープンリストに入っているノードの数)
    int close_number = 0;//クローズされたノードの数(クローズリストに入っているノードの数)
    current_node->cost = 0;//現在のノード(スタートノード)の値を設定
    current_node->heuristic = heuristic(current_node,goal_node);
    current_node->total_cost = current_node->cost + current_node->heuristic;
    current_node->available = false;//利用不可(通過した)

    open_nodes[0] = current_node;//オープンリストの先頭に現在のノードを追加
    open_number++;//open_numberを1増やす

    while (open_number > 0 || current_node == goal_node) {//オープンリストにノードがあるかぎり探索を続ける
        int mincost_node_index = 0;//最小コストのノードのインデックスを保持する変数
        for (int i = 0;i < open_number - 1;i++) {//for文でopen_nodes内を比較し、最小コストを探す
            if (!open_nodes[i]->passed) {//比較対象のノードが未通過なら(そもそもオープンリストにいれないから使わないかも)
                if (open_nodes[mincost_node_index]->total_cost > open_nodes[i]->total_cost) {//最小コストノードの総コストi番目のノードの総コストを比較
                    mincost_node_index = i;//i番目のノードの総コストがより小さければ、indexを更新
                }
            }
        }

        Nodes* next_node = open_nodes[mincost_node_index];//最小コストのノードを次に移動するノードとする

        //close_nodes[close_number] = current_node;//現在のノードをクローズリストに追加
        current_node->passed = true;//通行済みにする
        open_nodes[mincost_node_index] = open_nodes[open_number - 1];//最小コストノードをオープンリストから削除(リストの最後尾のノードで上書きして"強引に","実質的"な削除をしている)
        open_number--;//通過したノードはもう探索しないので、open_numberをひとつ減らす
        next_node->parent = current_node;//移動先のノードの親ノードを現在のノードに設定
        current_node = next_node;//現在のノードを移動先のノードに更新


        if (current_node == goal_node)return;//この時点でゴールノードに到達しているなら探索を終了する

        int new_cost = current_node->cost + 1;

        Nodes* neighbors[4] = {&node[current_node->x - 1][current_node->y], &node[current_node->x + 1][current_node->y], &node[current_node->x][current_node->y + 1], &node[current_node->x][current_node->y - 1] };
        //隣接するノードの配列。先頭から順番に左、右、上、下のノード

        if (current_node->x < 10 && !neighbors[0]->passed) {//現在のノードが左端にない、かつ次のノードが未通行なら
            open_nodes[open_number] = neighbors[0];//現在のノードから一つ左のノードをオープンリストに追加
            Setcost(open_nodes[open_number], new_cost, heuristic(open_nodes[open_number], goal_node));
            open_number++;
        }

        if (current_node->x > 0 && !neighbors[1]->passed) {//現在のノードが右端にない、かつ次のノードが未通行なら
            open_nodes[open_number] = neighbors[1];//右
            Setcost(open_nodes[open_number], new_cost, heuristic(open_nodes[open_number], goal_node));
            open_number++;
        }

        if (current_node->y < 10 && !neighbors[2]->passed) {//現在のノードが上端にない、かつ次のノードが未通行なら
            open_nodes[open_number] = neighbors[2];//上
            Setcost(open_nodes[open_number], new_cost, heuristic(open_nodes[open_number], goal_node));
            open_number++;
        }

        if (current_node->y > 0 && !neighbors[3]->passed) {//現在のノードが下端にない、かつ次のノードが未通行なら
            open_nodes[open_number] = neighbors[3];//下
            Setcost(open_nodes[open_number], new_cost, heuristic(open_nodes[open_number], goal_node));
            open_number++;
        }
    }    

}

