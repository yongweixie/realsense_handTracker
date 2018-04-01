#include <opencv2\opencv.hpp>
#include "ringbuf.hpp"
#include"Hand3D.hpp"
#include<utilities\pxcsmoother.h>
using namespace std;
using namespace cv;
#define devView(i) imshow(#i,i)
enum DirectionID {
	x_pos = 0x01 << 0,
	x_unk = 0x00,
	x_neg = 0x01 << 1,
	y_pos = 0x01 << 2,
	y_unk = 0x00,
	y_neg = 0x01 << 3,
	z_pos = 0x01 << 4,
	z_unk = 0x00,
	z_neg = 0x01 << 5,
};
enum state {
	neg = -1,
	unknow = 0,
	pos = 1,
};
enum HandState
{
	hand_unkown = 100,
	hand_zero = 100,
	hand_stone = 100,
	hand_one,
	hand_two,
	hand_three,
	hand_four,
	hand_five,
	hand_six,
	hand_seven,
	hand_eight,
	hand_nine,
	hand_fullopen,
	hand_Little,
	hand_Big,
	hand_OK=113,
};

enum HandTrail
{
	me=200,
	go=201,
	zhengzhou,
	gaotie,
	cesuo,
};
struct state3 {
	int xst, yst, zst;
	//double confidence[3];
};
struct fingerGesture
{
	int left, right ;
};
uchar idTable[3][3][3] = {
	(x_neg | y_neg | z_neg), (x_neg | y_neg | z_unk), (x_neg | y_neg | z_pos),//2a,a,1a
	(x_neg | y_unk | z_neg), (x_neg | y_unk | z_unk), (x_neg | y_unk | z_pos),//22,2,12
	(x_neg | y_pos | z_neg), (x_neg | y_pos | z_unk), (x_neg | y_pos | z_pos),//26,6,16
	(x_unk | y_neg | z_neg), (x_unk | y_neg | z_unk), (x_unk | y_neg | z_pos),//28,8,18
	(x_unk | y_unk | z_neg), (x_unk | y_unk | z_unk), (x_unk | y_unk | z_pos),//20,0,10
	(x_unk | y_pos | z_neg), (x_unk | y_pos | z_unk), (x_unk | y_pos | z_pos),//24,4,14
	(x_pos | y_neg | z_neg), (x_pos | y_neg | z_unk), (x_pos | y_neg | z_pos),//29,9,29
	(x_pos | y_unk | z_neg), (x_pos | y_unk | z_unk), (x_pos | y_unk | z_pos),//21,1,11
	(x_pos | y_pos | z_neg), (x_pos | y_pos | z_unk), (x_pos | y_pos | z_pos),//25,5,15
};
uchar MapStateToID(state3 &in) {
	return idTable[in.xst + 1][in.yst + 1][in.zst + 1];
}
ringbuffer<Point> trailR(30);
ringbuffer<Point> trailL(30);

ringbuffer<float> WorldPosR(40);
ringbuffer<float> WorldPosL(40);

vector<state3> mstseqR;
vector<state3> mstseqL;
Point center(ringbuffer<Point> seq) {
	// 卡尔曼滤波实现中心估计
	Point ct = seq[0];
	for (size_t i = 1; i < seq.size(); i++)
		ct += seq[i];
	ct.x /= seq.size();
	ct.y /= seq.size();
	return ct;
}
float center(ringbuffer<float> WorldPos){
	float Wp = WorldPos[0];
	for (size_t i = 1; i < WorldPos.size(); i++)
		Wp += WorldPos[i];
	Wp /= WorldPos.size();
	return Wp;
}
vector<state3> shrinkState1D(vector<state3> seqid) {
	vector<state3> sseqid;
	for (size_t i = 1; i < seqid.size(); i++)
	{
		if (seqid[i - 1].xst != seqid[i].xst /*||
											 seqid[i - 1].yst != seqid[i].yst ||
											 seqid[i - 1].zst != seqid[i].zst*/
			) {
			sseqid.push_back(seqid[i]);
			//cout << "[" << seqid[i].xst /*<< "," << seqid[i].yst << "," << seqid[i].zst*/ << "]" ;
		}
	}
	//cout << endl;
	//cout << sseqid.size() << endl;
	return sseqid;
}
vector<int> shrinkFinger(vector<int> &Finger)
{

	vector <int> FFinger;
	for (size_t i = 1; i < Finger.size(); i++)
	{
		if (Finger[i - 1] != Finger[i])
		{
			FFinger.push_back(Finger[i]);
		}
	}
	//cout << FFinger.size();
	return FFinger;
}
vector<string> shrinkString(vector<string> str)
{
	vector<string> sstr;
	for (size_t i = 1; i < str.size(); i++)
	{
		if (str[i - 1] != str[i] /*||
											 seqid[i - 1].yst != seqid[i].yst ||
											 seqid[i - 1].zst != seqid[i].zst*/
			) {
			sstr.push_back(str[i]);
			//cout << "[" << seqid[i].xst /*<< "," << seqid[i].yst << "," << seqid[i].zst*/ << "]" ;
		}
	}
	//cout << endl;
	//cout << sseqid.size() << endl;
	return sstr;
}
state3 cvt( ringbuffer<float> WorldPos,ringbuffer<Point> &seq, int thresh) {
	state3 st;
	Point ct = center(seq);
	float Wp = center(WorldPos);
	st.xst = unknow;
	//ct.x = 640 - ct.x;
	if (ct.x - seq.end().x > thresh)st.xst = neg;
	if (ct.x - seq.end().x < -thresh)st.xst = pos;
	st.yst = unknow;
	if (ct.y - seq.end().y > thresh)st.yst = neg;
	if (ct.y - seq.end().y < -thresh)st.yst = pos;
	st.zst = unknow;
	if (Wp - WorldPos.end() > 0.025) st.zst = neg;//往前
	if (Wp - WorldPos.end() < -0.025) st.zst = pos;//往后
	//st.zst = unknow;
	//if (ct.z - seq.end().z > thresh)st.zst = neg;
	//if (ct.z - seq.end().z < -thresh)st.zst = pos;
	return st;
}


PXCSmoother *smooth = NULL;
PXCSmoother::Smoother1D* smoother;
PXCSmoother::Smoother1D* smootherPoint[6];


class HandGesture
{
public:
	void ShowGesture(Hand3D hand, Mat colorImage, Mat canvas);
	double *FingerDataR(Hand3D hand);
	double *FingerDataL(Hand3D hand);
	string HandGesture::Ges2num(state3 cR, state3 cL, double *FingerFolL, double *FingerFolR);
	fingerGesture gesture2num(double *FingerFolL, double *FingerFolR);
	vector<int >numData(double *FingerFolL, double *FingerFolR);

private:
	//PXCHandData::IHand *ihand[3];
};
double *HandGesture::FingerDataR(Hand3D hand)
{
	auto r = hand.QueryHandRight();
	double *foldedness = hand.QueryFingerFoldedness(r);
	/*for (size_t i = 0; i < 5; i++)
	{
		cout << foldedness[i] << "  ";
	}*/
	//cout << endl;
	return foldedness;
}
double *HandGesture::FingerDataL(Hand3D hand)
{
	auto r = hand.QueryHandLeft();
	double *foldedness = hand.QueryFingerFoldedness(r);
	/*for (size_t i = 0; i < 5; i++)
	{
	cout << foldedness[i] << "  ";
	}*/
	//cout << endl;
	return foldedness;
}
string HandGesture::Ges2num(state3 cR,state3 cL, double *FingerFolL, double *FingerFolR)
{
	string handTrail;
	vector<string> handtrailOrder;
	struct HandGestureID{
		int left,right;
	}hgid;

	auto fG = gesture2num(FingerFolL, FingerFolR);
	hgid.left = fG.left; hgid.right = fG.right;
	bool ready_index = false;//我
	/*if ((hgid.right == (hand_one - 100))|| (hgid.right == (hand_nine - 100))||(hgid.right == (hand_seven - 100)))
	{
		ready_index = true;
	}*/
	if ((hgid.right == (hand_one - 100)))
	{
		ready_index = true;
	}
	if (ready_index)
	{
		if (MapStateToID(cR) == (x_unk | y_unk | z_pos))
		{
			handTrail = "我要";
			//ready_index = false;

		//cout << "我" << "  ";
		}
		if (MapStateToID(cR) == (x_neg | y_unk | z_unk))
		{
			handTrail = "郑州";
			//ready_index = false;
			//cout << "郑州" << "  ";
		}
	}
	//if ((hgid.right == hand_six - 100 || (hgid.right != hand_two - 100 && hgid.right != hand_one - 100))&&MapStateToID(cR) == (x_neg | y_unk | z_unk))
	//{
	//	handTrail = "去";
	//	//cout << "去" <<"  ";
	//}
	bool ready_six = false;
	if (hgid.left == (hand_six - 100))
	{
		ready_six;
	}
	if (ready_six)
	{
		if (MapStateToID(cR) == ((x_neg | y_unk | z_unk)))
		{
			handTrail = "去";
		}
	}
	
	
	if (hgid.right!=hand_five-100&&MapStateToID(cR) == (x_unk | y_neg | z_unk))
	{
		handTrail = " 高铁"; 
		//ShellExecute(NULL, L"open", L"http://www.12306.cn/mormhweb/", NULL, NULL, SW_SHOWNORMAL);

		//cout << "高铁"<<" ";
	}	

	if (hgid.right != hand_five - 100 && MapStateToID(cR) == (x_unk | y_pos | z_unk))
	{
		handTrail = "西安";
		//cout << "高铁"<<" ";
	}
	if (hgid.right == hand_OK-100)
	{
		handTrail = "厕所";
		//cout << "厕所" << endl;	
	}
	if (hgid.right == hand_six-100&&MapStateToID(cR) == (x_unk | y_neg | z_unk))
	{
		//handTrail = "飞机";
		//cout << "飞机"<<"  ";
	}
	if (hgid .right== hand_one-100&&MapStateToID(cR) == (x_unk | y_neg | z_unk))
	{
		//handTrail = "上";
		//cout << "上"<<"  ";
	}
	if (hgid.right == hand_two-100&&MapStateToID(cR) == (x_unk | y_neg | z_unk))
	{
		//handTrail = "二楼";
		//cout << "二楼"<<" ";
	}
	if ((MapStateToID(cL) == (x_neg | y_unk | z_unk))&&MapStateToID(cR) == (x_pos | y_unk | z_unk))
	{
	//	handTrail = "今天";
	    //cout << "今天"<<"  ";
	}
	bool ishand_two = false;
	if (hgid.right == hand_two - 100)
	{
		ishand_two == true;
	}
	if (hgid.right==hand_two - 100 &&MapStateToID(cR) == (x_pos | y_unk | z_unk))
	{
		handTrail = "明天";
		//Sleep(500);
		for (size_t i = 0; i < 10; i++)
			{
				//cout << "正在查询" << endl;
			}

		//Sleep(2000);

		//ShellExecute(NULL, L"open", L"http://flights.ctrip.com/?utm_source=baidu&utm_medium=cpc&utm_campaign=baidu0f&campaign=CHNbaidu0f&adid=flight&gclid=&isctrip=T", NULL, NULL, SW_SHOWNORMAL);

	}
	if (hgid.right == hand_two - 100 && MapStateToID(cR) == (x_unk | y_unk | z_pos))
	{
		//右手食指无名指先往后，再画圈
		/*handTrail = "天气";
		Sleep(500);
		for (size_t i = 0; i < 10; i++)
		{
			cout << "西安今天天气" << endl;
		}
		cout << "正在查询.....";
		Sleep(2000);
		ShellExecute(NULL, L"open", L"http://www.weather.com.cn/weather1d/101050311.shtml#input", NULL, NULL,SW_SHOWNORMAL);*/
		//cout << "天气"<<" ";
		//system("pause");
	}
	if (hgid.right == hand_Big-100)
	{
		//右手大拇指
		//handTrail = "好吗";
		//cout << "好吗" << "  ";
	}
	if (MapStateToID(cL) == (x_pos | y_unk | z_unk) && MapStateToID(cR) == (x_neg | y_unk | z_unk))
	{
		//两手较远位置往中间靠
		//handTrail = "房间";
		//cout << "房间" << " ";
	}
	
	return handTrail;
}

fingerGesture HandGesture::gesture2num(double *FingerFolL, double *FingerFolR)
{
	fingerGesture fingerGesture;
	 fingerGesture.left = 100;
	 fingerGesture.right = 100;
	 /*if (FingerFolL[0]=-1)
	 {
		 fingerGesture.left = hand_unkown;
	 }
	 if (FingerFolR[0] =-1)
	 {
		 fingerGesture.right = hand_unkown;
	 }*/
	/* if (FingerFolL[0]<0.3&&FingerFolL[1]<0.3&&FingerFolL[2]<0.3&&FingerFolL[3]<0.3&&FingerFolL[4]<0.3)
	 {
		 fingerGesture.left = hand_zero;
	 }
	 if (FingerFolR[0]<0.3&&FingerFolR[1]<0.3&&FingerFolR[2]<0.3&&FingerFolR[3]<0.3&&FingerFolR[4]<0.3)
	 {
		 fingerGesture.right = hand_zero;
	 }*/
	if (FingerFolL[0]<0.3&&FingerFolL[1]>0.8&&FingerFolL[2]<0.3&&FingerFolL[3]<0.3&&FingerFolL[4]<0.3)
	{
		fingerGesture.left = hand_one;
	}
	if (FingerFolR[0]<0.3&&FingerFolR[1]>0.8&&FingerFolR[2]<0.3&&FingerFolR[3]<0.3&&FingerFolR[4]<0.3)
	{
		fingerGesture.right = hand_one;
	}
	if ((FingerFolL[0]<0.3&&FingerFolL[1]>0.8&&FingerFolL[2]>0.8&&FingerFolL[3]<0.3&&FingerFolL[4]<0.3))
	{
		fingerGesture.left= hand_two;
	}
	if ((FingerFolR[0]<0.3&&FingerFolR[1]>0.8&&FingerFolR[2]>0.8&&FingerFolR[3]<0.3&&FingerFolR[4]<0.3))
	{
		fingerGesture.right = hand_two;
	}
	if ((FingerFolL[0]<0.3&&FingerFolL[1]>0.8&&FingerFolL[2]>0.8&&FingerFolL[3]>0.8&&FingerFolL[4]<0.3))
	{
		fingerGesture.left = hand_three;
	}
	if ((FingerFolR[0]<0.3&&FingerFolR[1]>0.8&&FingerFolR[2]>0.8&&FingerFolR[3]>0.8&&FingerFolR[4]<0.3))
	{
		fingerGesture.right = hand_three;
	}
	if ((FingerFolL[0]<0.3&&FingerFolL[1]>0.8&&FingerFolL[2]>0.8&&FingerFolL[3]>0.8&&FingerFolL[4]>0.8))
	{
		fingerGesture.left = hand_four;
	}
	if ((FingerFolR[0]<0.3&&FingerFolR[1]>0.8&&FingerFolR[2]>0.8&&FingerFolR[3]>0.8&&FingerFolR[4]>0.8))
	{
		fingerGesture.right = hand_four;
	}
	if ((FingerFolL[0]>0.8&&FingerFolL[1]>0.8&&FingerFolL[2]>0.8&&FingerFolL[3]>0.8&&FingerFolL[4]>0.8))
	{
		fingerGesture.left = hand_five;
	}
	if ((FingerFolR[0]>0.8&&FingerFolR[1]>0.8&&FingerFolR[2]>0.8&&FingerFolR[3]>0.8&&FingerFolR[4]>0.8))
	{
		fingerGesture.right = hand_five;
	}
	if ((FingerFolL[0]>0.8&&FingerFolL[1]<0.3&&FingerFolL[2]<0.3&&FingerFolL[3]<0.3&&FingerFolL[4]>0.8) ||
		(FingerFolL[0]>0.8&&FingerFolL[1]<0.3&&FingerFolL[2]<0.3&&FingerFolL[3]>0.8&&FingerFolL[4]<0.3)
		)
	{
		fingerGesture.left = hand_six;
	}
	if ((FingerFolR[0]>0.8&&FingerFolR[1]<0.3&&FingerFolR[2]<0.3&&FingerFolR[3]<0.3&&FingerFolR[4]>0.8) ||
		(FingerFolR[0]>0.8&&FingerFolR[1]<0.3&&FingerFolR[2]<0.3&&FingerFolR[3]>0.8&&FingerFolR[4]<0.3)
		)
	{
		fingerGesture.right = hand_six;
	}
	if ((FingerFolL[0]<0.2&&FingerFolL[1] >0.3&&FingerFolL[2] >0.4&&FingerFolL[3]<0.4&&FingerFolL[4]<0.4) && (FingerFolL[1] <0.9))
	{
		fingerGesture.left = hand_seven;
	}
	if ((FingerFolR[0]<0.2&&FingerFolR[1] >0.3&&FingerFolR[2] >0.4&&FingerFolR[3]<0.4&&FingerFolR[4]<0.4) && (FingerFolR[1] <0.9))
	{
		fingerGesture.right = hand_seven;
	}
	if ((FingerFolL[0]>0.8&&FingerFolL[1]>0.8&&FingerFolL[2]<0.3&&FingerFolL[3]<0.3&&FingerFolL[4]<0.3))
	{
		fingerGesture.left = hand_eight;
	}
	if ((FingerFolR[0]>0.8&&FingerFolR[1]>0.8&&FingerFolR[2]<0.3&&FingerFolR[3]<0.3&&FingerFolR[4]<0.3))
	{
		fingerGesture.right = hand_eight;
	}
	if ((FingerFolL[0]<0.3&&FingerFolL[1]>0.2&&FingerFolL[2]<0.3&&FingerFolL[3]<0.3&&FingerFolL[4]<0.3&&FingerFolL[1]<0.7))
	{
		fingerGesture.left = hand_nine;
	}
	if ((FingerFolR[0]<0.3&&FingerFolR[1]>0.2&&FingerFolR[2]<0.3&&FingerFolR[3]<0.3&&FingerFolR[4]<0.3&&FingerFolR[1]<0.7))
	{
		fingerGesture.right = hand_nine;
	}
	/*if ((FingerFolL[0]>0.8&&FingerFolL[1]>0.8&&FingerFolL[2]>0.8&&FingerFolL[3]>0.8&&FingerFolL[4]>0.8) &&
		(FingerFolR[0]>0.8&&FingerFolR[1]>0.8&&FingerFolR[2]>0.8&&FingerFolR[3]>0.8&&FingerFolR[4]>0.8))
	{
		fingerGesture.right = hand_fullopen;
	}*/
	if ((FingerFolL[0]<0.3&&FingerFolL[1]<0.3&&FingerFolL[2]<0.3&&FingerFolL[3]<0.3&&FingerFolL[4]>0.8) ||
		(FingerFolL[0]<0.3&&FingerFolL[1]<0.3&&FingerFolL[2]<0.3&&FingerFolL[3]>0.8&&FingerFolL[4]<0.3))
	{
		fingerGesture.left = hand_Little;
	}
	if ((FingerFolR[0]<0.3&&FingerFolR[1]<0.3&&FingerFolR[2]<0.3&&FingerFolR[3]<0.3&&FingerFolR[4]>0.8) ||
		(FingerFolR[0]<0.3&&FingerFolR[1]<0.3&&FingerFolR[2]<0.3&&FingerFolR[3]>0.8&&FingerFolR[4]<0.3))
	{
		fingerGesture.right = hand_Little;
	}
	if ((FingerFolR[0]>0.8&&FingerFolR[1] < 0.4&&FingerFolR[2] < 0.4&&FingerFolR[3] < 0.4&&FingerFolR[4] < 0.4))
	{
		fingerGesture.right = hand_Big;
	}
	if ((FingerFolL[0] > 0.8&&FingerFolL[1] < 0.4&&FingerFolL[2] < 0.4&&FingerFolL[3] < 0.4&&FingerFolL[4] < 0.4))
	{
		fingerGesture.left = hand_Big;
	}
	if ((FingerFolR[0]<0.4&&FingerFolR[1] < 0.4&&FingerFolR[2]>0.8&&FingerFolR[3]>0.8&&FingerFolR[4]>0.8))
	{
		fingerGesture.right = hand_OK;
	}
	fingerGesture.left = fingerGesture.left - 100;
	fingerGesture.right = fingerGesture.right - 100;
	return fingerGesture;
}
 

vector<int> fgesOrderL;
vector<int> fgesOrderR;
vector<int> num_data;
vector<string> HandTrailOrder;
void HandGesture::ShowGesture(Hand3D hand, Mat colorImage,Mat canvas)
{
		auto CenterPosR = hand.QueryMassCenterImage(hand.QueryHandRight());
		auto CenterPosL = hand.QueryMassCenterImage(hand.QueryHandLeft());

		auto handPosR = hand.QueryMassCenterWorld(hand.QueryHandRight());
		auto handPosL = hand.QueryMassCenterWorld(hand.QueryHandLeft());

		auto hand_xR = smootherPoint[0]->SmoothValue(CenterPosR.x);
		auto hand_xL = smootherPoint[1]->SmoothValue(CenterPosL.x);
		hand_xR = 640 - hand_xR;
		hand_xL = 640 - hand_xL;
		auto hand_yL = smootherPoint[2]->SmoothValue(CenterPosL.y);
		auto hand_yR = smootherPoint[3]->SmoothValue(CenterPosR.y);

		if (hand_xR ==640&& hand_yR ==0)
		{
			hand_xR = 480;
			hand_yR = 120;
		}
		if (hand_xL == 640 && hand_yL == 0)
		{
			hand_xL = 160;
			hand_yL = 120;
		}
		trailR.push_back(Point(hand_xR, hand_yR));
		trailL.push_back(Point(hand_xL, hand_yL));

		auto hand_zR = smootherPoint[4]->SmoothValue(handPosR.z);
		auto hand_zL = smootherPoint[5]->SmoothValue(handPosL.z);
		WorldPosR.push_back(hand_zR);
		WorldPosL.push_back(hand_zL);

		canvas -= Scalar(64, 64, 64);
		auto cR = cvt(WorldPosR, trailR, 80);
		auto cL= cvt(WorldPosL, trailL, 80);
		mstseqR.push_back(cR);
		shrinkState1D(mstseqR);
		mstseqL.push_back(cL);
		shrinkState1D(mstseqL);
		//line(canvas, trail.begin(), trail.end(), Scalar(127, 127, 127 * (1 + state)));
	
		for (size_t i = 1; i < trailR.size(); i++) {
			auto color = Scalar(127, 127, 255 * ((double)i / trailR.size()));
			line(canvas, trailR[i - 1], trailR[i], color);
		}
		for (size_t i = 1; i < trailL.size(); i++) {
			auto color = Scalar(127, 127, 255 * ((double)i / trailL.size()));
			line(canvas, trailL[i - 1], trailL[i], color);
		}

		//circle(canvas, trail.begin(), 10, Scalar(255, 255, 0));
		circle(canvas, trailR.end(), 10, Scalar(0, 0, 255),-1);
		//circle(canvas, center(trail), 10, Scalar(0, 255, 255));
		circle(canvas, trailL.end(), 10, Scalar(0, 255, 0),-1);


		line(canvas, trailR.end() + Point(127 * cR.xst, 127 * cR.yst), trailR.end(), Scalar(255, 255, 255), 1);
		line(canvas, trailL.end() + Point(127 * cL.xst, 127 * cL.yst), trailL.end(), Scalar(255, 255, 255), 1);

		//devView(canvas);
		

	/*	if (MapStateToID(cR)==(x_unk|y_unk|z_pos))
		{
			cout << "z_pos" << endl;
		}
		else
		{
			cout << "else" << endl;
		}
*/

		auto FingerFolL = FingerDataL(hand);
		auto FingerFolR = FingerDataR(hand);
		auto fingerGesture = gesture2num(FingerFolL, FingerFolR);
			fgesOrderL.push_back(fingerGesture.left);
			fgesOrderR.push_back(fingerGesture.right);
		auto FgesOrderL=shrinkFinger(fgesOrderL);
		auto FgesOrderR= shrinkFinger(fgesOrderR);

		if (!FgesOrderL.empty())
		{
			putText(colorImage, (std::to_string(FgesOrderL[FgesOrderL.size() - 1])), Point(120, 150), cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255), 2);
			//cout << FgesOrder[FgesOrder.size() - 1] << "  ";
		}
		if (!FgesOrderR.empty())
		{
			putText(colorImage, (std::to_string(FgesOrderR[FgesOrderR.size() - 1])), Point(300, 150), cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255), 2);
			//cout << FgesOrder[FgesOrder.size() - 1] << "  ";
		}
		auto gesShow=Ges2num( cR,cL, FingerFolL, FingerFolR);
		HandTrailOrder.push_back(gesShow);
		for (size_t i = 0; i < shrinkString(HandTrailOrder).size(); i++)
		{
			//cout << shrinkString(HandTrailOrder)[i];
		}

		if (gesture2num(FingerFolL, FingerFolR).right==hand_Little)
		{
			//shrinkString(HandTrailOrder).clear();
		}
		cout << endl;
		//putText(colorImage, gesShow, Point(20, 170), cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255), 2);
	}
vector <int> HandGesture::numData(double *FingerFolL, double *FingerFolR)
{
	auto numL = gesture2num(FingerFolL, FingerFolR).left;
	auto numR = gesture2num(FingerFolL, FingerFolR).right;
	if (numL==numR)
	{
		num_data.push_back(numL);
		//cout << numL << " ";
		////putText(colorImage, (std::to_string(numL)).substr(0, 4), Point(100, 170), cv::HersheyFonts::FONT_HERSHEY_COMPLEX, 1, Scalar(0, 20, 0), 2);
	}
	return shrinkFinger(num_data);
}