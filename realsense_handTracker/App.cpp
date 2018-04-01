#include <opencv2\opencv.hpp>
#include<utilities\pxcsmoother.h>
#include"Hand3D.hpp"
#include "ringbuf.hpp"
#include"HandGesture.hpp"
using namespace std;
using namespace cv;
#define devView(i) imshow(#i,i)

int main()
{
	PXCSenseManager::CreateInstance()->QuerySession()->CreateImpl<PXCSmoother>(&smooth);
	smoother = smooth->Create1DQuadratic(1);
	for (size_t i = 0; i < 6; i++) {
		smootherPoint[i] = smooth->Create1DSpring(0.5);
	}

	Hand3D hand;

	hand.KillDCM();
	hand.Init();
	Mat canvas = Mat::zeros(Size(640, 480), CV_8UC3);
	LARGE_INTEGER t0, t, freq;
	HandGesture HandGs;
	for (; waitKey(15) != 27; hand.Update())
	{
		Mat Colorimage = hand.drawIndicator();
		//Mat Colorimage = hand.QueryColorImage2();

		HandGs.ShowGesture(hand, Colorimage, canvas);
		QueryPerformanceCounter(&t);
		QueryPerformanceFrequency(&freq);
		double fps = freq.QuadPart / ((double)t.QuadPart - t0.QuadPart);
		putText(Colorimage, "fps:" + (std::to_string(smoother->SmoothValue(fps))).substr(0, 4), Point(20, 50), cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 20, 0), 2);

		Mat viewer;
		addWeighted(Colorimage, 0.7, canvas, 1, 1, viewer);
		//resize(viewer, viewer, Size(800,600));
		devView(viewer);
		QueryPerformanceCounter(&t0);
		//	HandGs.numData(HandGs.FingerDataL(hand), HandGs.FingerDataR(hand));
	}
	return 0;
}