#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <opencv/highgui.h>
int key = 0;

int main() {
	int width = 1000;
	int height = 1000;

	IplImage* img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);	
	cvNamedWindow("Output");	
	for (int i = 0; i <= 1000; i += 100) {
		cvLine(img, cvPoint(0, 0+i), cvPoint(1000, 0+i), cvScalar(0, 0, 255), 5, 1);
		cvLine(img, cvPoint(0+i, 0), cvPoint(0+i,1000), cvScalar(0, 0, 255), 5, 1);
	}
	cvShowImage("Output", img);

	while (1) {
		key = cvWaitKey(0);		
		if (key == ' ')
			break;

	}
	cvDestroyAllWindows();			
	cvReleaseImage(&img);			

	return 0;

}
