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

void a_star(Nodes* start_node,Nodes*goal_node) {


    Nodes* open_nodes[GRID_SIZE * GRID_SIZE] = {nullptr};
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
                else if(open_nodes[mincost_node_index]->total_cost == open_nodes[i]->total_cost){
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
