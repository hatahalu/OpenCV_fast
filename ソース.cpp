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