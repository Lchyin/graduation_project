
/******************************************************
 * 摄像头使用kinect2
 * 结合camshiftdemo.cpp
 * 追使用霍夫变换选中矩形框
 *
 ******************************************************/

#include <iostream>
#include <ctype.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <cstdlib>
#include "ur_modern_driver/ur_realtime_communication.h"
#include "string"
using namespace std;
using namespace cv;
UrRealtimeCommunication * Ur;
void MyLine( Mat img, Point start, Point end );

typedef struct{
	float x=0.5;
	float y;
	float z;
}Pose;
std::vector<Pose> poins_lines;
std::vector<Pose> poins_ag;
std::vector<Pose> poins_rect;
std::vector<Pose> poins_circle;



bool Init_Pose()
{
	std::string cmd = "movel(p[0.5,-0.1,0.45,0.35,1.6,0.4],a=1.4, v=1.05, t=0, r=0)\n";

	return Ur->addCommandToQueue(cmd);
}



std::vector<Pose> cc_pose(std::vector<cv::Point> points_lines){

	std::vector<Pose> pose;
	Pose a;
	for (int i =0;i < points_lines.size();i++){
	
		a.z = 0.55 - points_lines[i].y*0.324/1000*1.3;
		a.y = -0.6 + points_lines[i].x*0.625/1000;
		pose.push_back(a);
		
	}
	return pose;

}

bool line(std::vector<Pose> poins_lines)
{
	char cmd[2048];
	std::sprintf(cmd, "def line(): \
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\nend\n", poins_lines[0].x, poins_lines[0].y, poins_lines[0].z,
			poins_lines[1].x, poins_lines[1].y, poins_lines[1].z);
	cout << cmd <<endl;
	return Ur->addCommandToQueue((std::string) (cmd));

}


bool angel(std::vector<Pose> poins_ag)
{
	char cmd[2048];
	std::sprintf(cmd, "def angel(): \
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\nend\n", poins_ag[0].x, poins_ag[0].y, poins_ag[0].z,
			poins_ag[1].x, poins_ag[1].y, poins_ag[1].z,
			poins_ag[2].x, poins_ag[2].y, poins_ag[2].z,
			poins_ag[0].x, poins_ag[0].y, poins_ag[0].z);
	std::cout << cmd << std::endl;
	return Ur->addCommandToQueue((std::string) (cmd));
}

bool Rec(std::vector<Pose> poins_rect)
{
	char cmd[2048];
	std::sprintf(cmd, "def rect(): \
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movep(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\nend\n", poins_rect[0].x, poins_rect[0].y, poins_rect[0].z,
			poins_rect[1].x, poins_rect[1].y, poins_rect[1].z,
			poins_rect[2].x, poins_rect[2].y, poins_rect[2].z,
			poins_rect[3].x, poins_rect[3].y, poins_rect[3].z,
			poins_rect[0].x, poins_rect[0].y, poins_rect[0].z);
	std::cout << cmd << std::endl;
	return Ur->addCommandToQueue((std::string) (cmd));
}


void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 1;
	int thickness = 5;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(255, 0, 0), thickness, 8);

}
void  cross(Mat& src,cv::Point center)
{
	// circle center
	Point on(center.x - 10, center.y);
	Point under(center.x + 10, center.y);
	Point left(center.x, center.y - 10);
	Point right(center.x, center.y + 10);
	MyLine(src, on, under);
	MyLine(src, left, right);
}

void cal_pose(Pose center, float r)
{
	Pose pose1, pose2, pose3, pose4, pose5;
	pose1 = center;
	pose2 = center;
	pose2.z = pose2.z - r;
	pose3 = center;
	pose3.y = pose3.y - r;
	pose4 = center;
	pose4.z = pose4.z + r;
	pose5 = center;
	pose5.y = pose5.y + r;

	poins_circle.push_back(pose1);
	poins_circle.push_back(pose2);
	poins_circle.push_back(pose3);
	poins_circle.push_back(pose4);
	poins_circle.push_back(pose5);
	poins_circle.push_back(pose2);
}

bool circle(std::vector<Pose> poins_circle)
{
	char cmd[2048];
	std::sprintf(cmd, "def circle(): \
			\n movej(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movej(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movec(p[%lf,%lf,%lf,0.35,1.6,0.4],p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movej(p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\n movec(p[%lf,%lf,%lf,0.35,1.6,0.4],p[%lf,%lf,%lf,0.35,1.6,0.4])\
			\nend\n", poins_circle[0].x, poins_circle[0].y, poins_circle[0].z,
			poins_circle[1].x, poins_circle[1].y, poins_circle[1].z,
			poins_circle[2].x, poins_circle[2].y, poins_circle[2].z,
			poins_circle[3].x, poins_circle[3].y, poins_circle[3].z,
			poins_circle[3].x, poins_circle[3].y, poins_circle[3].z,
			poins_circle[4].x, poins_circle[4].y, poins_circle[4].z,
			poins_circle[5].x, poins_circle[5].y, poins_circle[5].z);
	std::cout << cmd << std::endl;
	return Ur->addCommandToQueue((std::string) (cmd));
}

std::vector<cv::Point> points;

int main( int argc, const char** argv )
{
	Mat image;
	//std::condition_variable msg_cond;
	//std::string host("172.16.252.129");
	//Ur = new UrRealtimeCommunication(msg_cond, host);
	//if (Ur->start()==0){
	//	std::cout <<"通信失败"<<std::endl;
	//}
	//Ur->robot_state_->setVersion(3.3);
	//Init_Pose();
	
	VideoCapture capture(0);
	namedWindow("image",CV_WINDOW_AUTOSIZE);
	while(!(waitKey(30)=='q'))
	{	
		Mat frame;
		capture>>frame;
		cv::cvtColor(frame,image,cv::COLOR_RGB2GRAY);
		cv::medianBlur(image,image,3);
		imshow("image",image);
		 cv::threshold(image,image,0,255,cv::THRESH_OTSU);
		 cv::imshow("threshold",image);
   		vector<Vec3f> circles;
		HoughCircles(image, circles, HOUGH_GRADIENT, 1,
				image.rows,  // change this value to detect circles with different 
				55,40, 0,0 // change the last two parameters
				// (min_radius & max_radius) to detect larger circles
				);
	    	for( size_t i = 0; i < circles.size(); i++ )
	   	 {
			Vec3i c = circles[i];
			Point center = Point(c[0], c[1]);
			// circle center
			circle( frame, center, 1, Scalar(0,255,0), 3, LINE_AA);
			// circle outline
			int radius = c[2];
			circle( frame, center, radius, Scalar(255,0,0), 3, LINE_AA);
	  	  }
	   	 imshow("detected circles", frame);
	}
	return 0;

}

void MyLine( Mat img, Point start, Point end )
{
	int thickness = 2;
	int lineType = LINE_8;
	line( img,
			start,
			end,
			Scalar( 255, 0, 255 ),
			8,
			lineType );
}


