#include <iostream>
#include <fstream>
#include <opencv2/rgbd.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include <Eigen/Dense>
//#include <eigen3/Eigen/Core>
//#include <Eigen/Geometry>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
Ptr<rgbd::OdometryFrame> frame_curr = Ptr<rgbd::OdometryFrame>(new rgbd::OdometryFrame());
Ptr<rgbd::OdometryFrame> frame_prev = Ptr<rgbd::OdometryFrame>(new rgbd::OdometryFrame());
Ptr<rgbd::ICPOdometry> odometry;
Ptr<rgbd::RgbdICPOdometry> odometry2;
Ptr<rgbd::RgbdOdometry> odometry3;
cv::rgbd::FastICPOdometry odometry4;


//from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
//calculate the euler rotation angles for non fixed ZYX Rotation which are the same angles as for rotation around fixed axis of XYZ
//so we can directly use this angles
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
	float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
		y = atan2(-R.at<double>(2,0), sy);
		z = atan2(R.at<double>(1,0), R.at<double>(0,0));
	}
	else
	{
		x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
		y = atan2(-R.at<double>(2,0), sy);
		z = 0;
	}
	return Vec3f(x, y, z);
}
Mat quatToMat(double x,double y,double z,double w){
	double nom=(w*w + x*x + y*y + z*z);
	w=w/nom;
	x=x/nom;
	y=y/nom;
	z=z/nom;
	Mat RT=Mat::eye(3,3, CV_64F);
	//Zeile,Spalte
	RT.at<double>(0,0)=1-2*y*y-2*z*z;
	RT.at<double>(0,1)=2*x*y - 2*w*z;
	RT.at<double>(0,2)=2*x*z+2*w*y;
	RT.at<double>(1,0)=2*x*y+2*w*z;
	RT.at<double>(1,1)=1-2*x*x-2*z*z;
	RT.at<double>(1,2)=2*y*z-2*w*x;
	RT.at<double>(2,0)=2*x*z-2*w*y;
	RT.at<double>(2,1)=2*y*z+2*w*x;
	RT.at<double>(2,2)=1-2*x*x-2*y*y;
	return RT;
}
class SubscribeAndPublish
{
public:
	//
	SubscribeAndPublish(int choose_):tf2_(buffer_),sync(MySyncPolicy(50), image_sub, image_sub2){
		cout<<"in creation"<<endl;
		choose=choose_;
		float minDepth = 0.3f; //in meters
		n.getParam("/mind",minDepth);
		float maxDepth = 10.0f; //in meters
		n.getParam("/maxd",maxDepth);
		float MAX_DEPTH_DIFF = 0.09f;  // in meters
		n.getParam("/mdd",MAX_DEPTH_DIFF);
		float MAX_POINTS_PART = 0.06f;
		n.getParam("/mpp",MAX_POINTS_PART);
		string topicname;
		if(choose == 1){
			odometry2->setMaxDepth(maxDepth);
			odometry2->setMinDepth(minDepth);
			odometry2->setMaxDepthDiff(MAX_DEPTH_DIFF);
			odometry2->setMaxPointsPart(MAX_POINTS_PART);
			topicname="/RgbdICP";
		}
		else if(choose==2){
			odometry3->setMinDepth(minDepth);
			odometry3->setMaxDepth(maxDepth);
			odometry3->setMaxDepthDiff(MAX_DEPTH_DIFF);
			odometry3->setMaxPointsPart(MAX_POINTS_PART);
			topicname="/Rgbd";
		}
		else if(choose==3){
			odometry4.setMaxDistDiff(MAX_DEPTH_DIFF);
		}
		else{
			odometry->setMinDepth(minDepth);
			odometry->setMaxDepth(maxDepth);
			odometry->setMaxDepthDiff(MAX_DEPTH_DIFF);
			odometry->setMaxPointsPart(MAX_POINTS_PART);
			topicname="/ICP";
		}
		cout<<"mind"<<minDepth<<"maxd"<<maxDepth<<"mdd"<<MAX_DEPTH_DIFF<<"mpp"<<MAX_POINTS_PART<<endl;
		cout<<"Creating Subscribes"<<endl;
		string col;
		n.getParam("/color",col);
                cout<<col<<endl;
		//https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
		image_sub.subscribe(n, col, 1);
		n.getParam("/depth",col);
                cout<<col<<endl;
		image_sub2.subscribe(n, col, 1);
		//image_sub2.subscribe(n,""/camera/aligned_depth_to_color/image_raw",1);
		//sync2.registerCallback(boost::bind(&SubscribeAndPublish::callback,this, _1, _2));
		sync.registerCallback(boost::bind(&SubscribeAndPublish::callback,this, _1, _2));
		odom_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped >(topicname, 50);
		n.getParam("/optical",col);
                cout<<col<<endl;
		string base;
                n.getParam("/base",base);
                cout<<base<<endl;
                cout<<"Waiting for message tf message"<<endl;
		base_link_to_leap_motion = buffer_.lookupTransform(base,col, ros::Time(0), ros::Duration(1.0) );
		cout<<base_link_to_leap_motion;
		n.getParam("/colorinfo",col);
                cout<<col<<endl;
                cout<<"Waiting for message to get the camera_info"<<endl;
		boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(col,n);
		cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(col,n);
		Mat cameraMatrix = Mat::eye(3,3,CV_32FC1);
		cout<<"Creating of camera Matrix"<<endl;
		//cout<<(*(cam_info)).K[0]<<endl;
		cameraMatrix.at<float>(0,0) = (*(cam_info)).K[0];
		cameraMatrix.at<float>(1,1) = (*(cam_info)).K[4];
		cameraMatrix.at<float>(0,2) =(*(cam_info)).K[2];
		cameraMatrix.at<float>(1,2) = (*(cam_info)).K[5];
		/*cameraMatrix.at<float>(0,0) = 617.1514282226562;
		cameraMatrix.at<float>(1,1) =  617.4064331054688;
		cameraMatrix.at<float>(0,2) = 328.6275939941406;
		cameraMatrix.at<float>(1,2) = 239.1651611328125;*/
		cout<<cameraMatrix<<endl;
		if(choose_ == 1){
			odometry2->setCameraMatrix(cameraMatrix);
		}
		else if(choose_ == 2){
			odometry3->setCameraMatrix(cameraMatrix);
		}
		else if(choose_ == 3){
			odometry4.setCameraMatrix(cameraMatrix);
		}
		else{
			odometry->setCameraMatrix(cameraMatrix);
		}
		cout<<"Done Cam Matrix"<<endl;
		n.getParam("/scalefactor",scalefactor);
                n.getParam("/debugfilepath",debugfilename);
                n.getParam("/debugmode",dbmod);
                cout<<"Debugmode: "<<dbmod<<endl;
	}
	void callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::ImageConstPtr& msg2){
		cout<<"in callback"<<cbcount<<endl;
		//cout<<msg->header.stamp<<endl;
		cbcount++;
		//auto start = chrono::steady_clock::now();
		//Mat depth = cv_bridge::toCvShare(msg2, "16UC1")->image;would be used if directly accessing kinect topic
		depth = cv_bridge::toCvShare(msg2, "32FC1")->image;
		//Mat depth_flt;
                //already done in other remap
                //depth.convertTo(depth, CV_32FC1,(1.0/scalefactor));
                //depth.setTo(std::numerics_limits<float>::quiet_NaN(), depth == 0);
		//cout<<depth.at<float>(300,200);
		//depth = depth_flt.clone();
		//cout<<first.type();
		cvtColor(cv_bridge::toCvShare(msg,"rgb8")->image, gray, COLOR_RGB2GRAY);
		/*Size size(640,480);
		resize(gray,gray,size);
		resize(depth,depth,size);*/
		//gray.convertTo(gray, CV_8UC1, 1.0 / 255.0);
		if(frame_prev->image.empty()){
			//https://docs.opencv.org/3.4/df/ddc/classcv_1_1rgbd_1_1Odometry.html needs metre
			frame_prev->image = gray.clone();
			frame_prev->depth = depth.clone();
		}
		else{
			frame_curr->image = gray.clone();
			frame_curr->depth = depth.clone();
			bool res;
			if(choose == 2){
				res = odometry3->compute(frame_curr,frame_prev, Rt);
			}
			else if(choose == 1){
				res = odometry2->compute(frame_curr,frame_prev, Rt);
			}
			else if(choose == 3){
				res = odometry4.compute(frame_curr,frame_prev, Rt);
			}
			else {
				res = odometry->compute(frame_curr,frame_prev, Rt);			
			}
			if(res==true){
				cout<<"Computation Done"<<endl;
				//Mat rotationMat = Rt(cv::Rect(0, 0, 3, 3)).clone();
				//new translation always apply from the left hand side
				//apply it from the right hand side when you compute it like shown above so
				//also it seems not to work when compute(prev,cur) is used and then transformation is applied to the previous one from left hand side
				//reason might be  Given a point \mathbf{(x, y, z)},
				//you can think of this point as a row vector [ x, y, z] or a column vector [ x, y, z]^T.
				//If you use a row vector, you have to post-multiply the 3×3 rotation matrix
				//and if you use the column vector representation you have to pre-multiply the rotation matrix to rotate the point.
                                Rt1 = Rt1*Rt;
                                //prevRt=Rt.clone();
				/*Mat translateMat = Rt1(cv::Rect(3, 0, 1, 3)).clone();
				 * ofstream myfile;
				 * for testing data before transforms
				 * myfile.open ("example2.txt",ios::app);
				myfile <<translateMat.at<double>(0)<<"   "<<translateMat.at<double>(2)<< "\n";
				myfile.close();*/
				//cout<<Rt1<<endl;
				//converting from left handside into right handside system
				/*Mat Rt2=Mat::eye(4,4,CV_64F)*-1;
				Rt2.at<double>(3,3)=1;flips around 0,0
                                rotate for 90 degree around x
                                Mat Rt3=Mat::zeros(4,4,CV_64F);
				Rt3.at<double>(0,2)=1;
				Rt3.at<double>(1,1)=1;
				Rt3.at<double>(2,0)=-1;
				Rt2=Rt2*Rt1;*/
				/*Mat abrot= Rt1(cv::Rect(0, 0, 3, 3));
				 * prove that the quaternions are correct by crosschecking with the result from eigen library
				 //not MatrixXd because it requires a 3x3 matrix
				Eigen::Matrix3d m(3,3);
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						m(j,i)=abrot.at<double>(j,i);
					}
				}
				Eigen::Quaterniond q(m);
				q.normalize();
				cout<<q.norm();
				std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << q.vec()<<std::endl;
				Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> b;*/
				//https://answers.opencv.org/question/129648/cvvec3f-vs-cvpoint3f/
				//this will apply the transform from color optical into kinect
				/*example for reading out transform msg of ros
				 * base_link_to_leap_motion.transform.translation.x=0;
				base_link_to_leap_motion.transform.translation.y=0;
				base_link_to_leap_motion.transform.translation.z=0;
				base_link_to_leap_motion.transform.rotation.x=	0.505;
				base_link_to_leap_motion.transform.rotation.y=	-0.498;
				base_link_to_leap_motion.transform.rotation.z=	 0.503;
				base_link_to_leap_motion.transform.rotation.w=	 -0.494;*/
				//we need to determine how the coordinates in rgbdKOS would look like in the cameralinkKOS
				//for that we create the Matrix which would transform from rgbd system into cameralink system
				//and use its inverse to get the matrix which will tell us how coordinates from rgbdKOSsystem would look like in cameralinkKOSsystem
				/*Mat Rr=quatToMat(base_link_to_leap_motion.transform.rotation.x,base_link_to_leap_motion.transform.rotation.y,base_link_to_leap_motion.transform.rotation.z,base_link_to_leap_motion.transform.rotation.w);
				Rr=Rr.t();
				abrot= Rr(cv::Rect(0, 0, 3, 3));
				Rr.copyTo(Rt(cv::Rect(0,0,3,3)));
				Rt.at<double>(0,3)=-base_link_to_leap_motion.transform.translation.x;
				Rt.at<double>(1,3)=-base_link_to_leap_motion.transform.translation.y;
				Rt.at<double>(2,3)=-base_link_to_leap_motion.transform.translation.z;
				Rt=Rt*Rt1;
				odom.pose.pose.position.x = Rt.at<double>(0,3);
				odom.pose.pose.position.y = Rt.at<double>(1,3);
				odom.pose.pose.position.z = Rt.at<double>(2,3);
				abrot= Rt(cv::Rect(0, 0, 3, 3));
				Vec3f angles= rotationMatrixToEulerAngles(abrot);
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(angles[0],angles[1],angles[2]);
				odom.pose.pose.orientation = odom_quat;
				 */
				//the other way to do this is directly use the transformation from cameralinkKOS into rgbdKOS which describes how coordinates from rgbdKOS would look like in cameraKOS
				Rt= Rt1(cv::Rect(0, 0, 3, 3));
				angles= rotationMatrixToEulerAngles(Rt);
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(angles[0],angles[1],angles[2]);
				//cout<<"Before:";
				/*cout<<"x:"<<Rt1.at<double>(0,3)<<endl;
				cout<<"y:"<<Rt1.at<double>(1,3)<<endl;
				cout<<"z:"<<Rt1.at<double>(2,3)<<endl;*/
				geometry_msgs::PoseWithCovarianceStamped odom;
				odom.pose.pose.position.x = Rt1.at<double>(0,3);
				odom.pose.pose.position.y = Rt1.at<double>(1,3);
				odom.pose.pose.position.z = Rt1.at<double>(2,3);
				odom.pose.pose.orientation = odom_quat;
				tf2::doTransform(odom, odom, base_link_to_leap_motion);
				cout<<"transform \n";
				odom.header.stamp = msg->header.stamp;
				//new stuff
				//odom.header.stamp = ros::Time::now();
				odom.header.frame_id = "camera_link";
				odom.pose.covariance[0]  = 1e-6;
				odom.pose.covariance[7]  = 1e-6;
				odom.pose.covariance[14] = 1e-6;
				odom.pose.covariance[21] = 1e-6;
				odom.pose.covariance[28] = 1e-6;
				odom.pose.covariance[35] = 1e-6;
				odom_pub.publish(odom);
				/*cout<<"x:"<<odom.pose.pose.position.x<<endl;
				cout<<"y:"<<odom.pose.pose.position.y<<endl;
				cout<<"z:"<<odom.pose.pose.position.z<<endl;*/
                                if(dbmod == true){
                                debugfile.open (debugfilename, ios::out | ios::app);
                                debugfile<<Rt1.at<double>(0,3)<<" "<<Rt1.at<double>(1,3)<<" "<<Rt1.at<double>(2,3)<<endl;
                                debugfile.close();
                                }                            
			}
			else{
                                if(dbmod == true){
                                    debugfile.open (debugfilename, ios::out | ios::app);
                                    cout<<"failed keeping old values"<<endl;
                                    debugfile<<"failed to calculate skipping frame e.g. publishing old values again"<<endl;
                                    debugfile.close();
                                }
                                Rt= Rt1(cv::Rect(0, 0, 3, 3));
                                angles= rotationMatrixToEulerAngles(Rt);
                                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(angles[0],angles[1],angles[2]);
                                //cout<<"Before:";
                                /*cout<<"x:"<<Rt1.at<double>(0,3)<<endl;
                                cout<<"y:"<<Rt1.at<double>(1,3)<<endl;
                                cout<<"z:"<<Rt1.at<double>(2,3)<<endl;*/
                                geometry_msgs::PoseWithCovarianceStamped odom;
                                odom.pose.pose.position.x = Rt1.at<double>(0,3);
                                odom.pose.pose.position.y = Rt1.at<double>(1,3);
                                odom.pose.pose.position.z = Rt1.at<double>(2,3);
                                odom.pose.pose.orientation = odom_quat;
				cout<<"keeping old "<<base_link_to_leap_motion<<"\n";
                                tf2::doTransform(odom, odom, base_link_to_leap_motion);
                                cout<<"transform \n";
                                odom.header.stamp = msg->header.stamp;
                                //new stuff
                                //odom.header.stamp = ros::Time::now();
                                odom.header.frame_id = "camera_link";
                                odom.pose.covariance[0]  = 1e-6;
                                odom.pose.covariance[7]  = 1e-6;
                                odom.pose.covariance[14] = 1e-6;
                                odom.pose.covariance[21] = 1e-6;
                                odom.pose.covariance[28] = 1e-6;
                                odom.pose.covariance[35] = 1e-6;
                                odom_pub.publish(odom);
                        }
                        std::swap(frame_prev, frame_curr);
                        if(!frame_curr.empty())
                                frame_curr->release();
                }
		//auto end = std::chrono::steady_clock::now();
		//cout << "Elapsed time in milliseconds : "
		//		<< chrono::duration_cast<chrono::milliseconds>(end - start).count()
		//		<< " ms" << std::endl;
	}
private:
	ros::NodeHandle n;
	ros::Publisher odom_pub;
	tf2_ros::Buffer buffer_;
	tf2_ros::TransformListener tf2_;
	message_filters::Subscriber<Image> image_sub;
	message_filters::Subscriber<Image> image_sub2;
	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;
	geometry_msgs::TransformStamped base_link_to_leap_motion;
        int choose;
        int cbcount=0;
        bool dbmod;
	double scalefactor;
        ofstream debugfile;
	Mat gray;
	Mat depth;
        Mat Rt;
        Mat Rt1=Mat::eye(4, 4, CV_64F);
        string debugfilename;
	Vec3f angles;
};

int test(void);
int test2(void);
int main(int argc, char **argv){
	//test2();
	//test();
	int choose=0;
	if(argc > 1){
		if((int)argv[1][0]-48 == 3){
			odometry4 = rgbd::FastICPOdometry();
			cout<<"Using FastICPOdometry"<<endl;
			choose=3;
		}
		else if((int)argv[1][0]-48  == 2){
			odometry3 = rgbd::RgbdOdometry::create();
			cout<<"Using RgbdOdometry"<<endl;
			choose=2;
		}
		else if((int)argv[1][0]-48  == 1){
			odometry2 = rgbd::RgbdICPOdometry::create();
			cout<<"Using RgbdICPOdometry"<<endl;
			choose=1;
		}
		else{
			odometry= rgbd::ICPOdometry::create();
			cout<<"Using ICPOdometry "<<endl;
			choose=0;
		}
	}
	else{
		odometry= rgbd::ICPOdometry::create();
		cout<<"Using ICPOdometry "<<endl;
		choose=0;
	}
	cout<<"In init"<<endl;
	ros::init(argc, argv,  "talker");
	cout<<"Finished init"<<endl;
	SubscribeAndPublish test(choose);
	ros::spin();
	return 1;
}
int test2(void){
	ofstream myfile;
	myfile.open ("example.txt",ios::trunc);
	myfile.close();
	Mat traj = Mat::zeros(800, 800, CV_8UC3);
	const int timestampLength = 17;
	const int rgbPathLehgth = 17+8;
	const int depthPathLehgth = 17+10;
	const string filename = "/home/irobot/Downloads/rgbd_dataset_freiburg3_long_office_household/assoc.txt";
	const string dirname = "/home/irobot/Downloads/rgbd_dataset_freiburg3_long_office_household/";
	ifstream file( filename.c_str() );
	if( !file.is_open() )
		return -1;
	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);
	Mat Rt;
	Mat Rt1=Mat::eye(4, 4, CV_64F);
	const float minDepth = 0.8f; //in meters
	const float maxDepth = 4.0f; //in meters
	const float MAX_DEPTH_DIFF = 0.08f;  // in meters
	const float MAX_POINTS_PART = 0.09f;
	float fx = 535.43310546875, // default
			fy =  539.212524414062,
			cx =  320.106652814575,
			cy = 247.632132204719;
	//				fx = 517.3f; fy = 516.5f; cx = 318.6f; cy = 255.3f;
	Mat cameraMatrix = Mat::eye(3,3,CV_32FC1);
	{
		cameraMatrix.at<float>(0,0) = fx;
		cameraMatrix.at<float>(1,1) = fy;
		cameraMatrix.at<float>(0,2) = cx;
		cameraMatrix.at<float>(1,2) = cy;
	}
	odometry3 = rgbd::RgbdOdometry::create();
	odometry3->setCameraMatrix(cameraMatrix);
	odometry3->setMaxDepth(maxDepth);
	odometry3->setMinDepth(minDepth);
	odometry3->setMaxDepthDiff(MAX_DEPTH_DIFF);
	odometry3->setMaxPointsPart(MAX_POINTS_PART);
	bool first=true;
	for(int i = 0; !file.eof(); i++)
	{
		string str;
		std::getline(file, str);
		if(str.empty()) break;
		string rgbFilename = str.substr(timestampLength + 1, rgbPathLehgth );
		string timestap = str.substr(0, timestampLength);
		string depthFilename = str.substr(2*timestampLength + rgbPathLehgth + 3, depthPathLehgth );
		Mat image2 = imread(dirname + rgbFilename);
		Mat depth2 = imread(dirname + depthFilename, -1);
		Mat gray2;
		Mat depth_flt;
		if(! image2.data )  // Check for invalid input
		{
			cout <<  "Could not open or find the image" << std::endl ;
			return -1;
		}
		cvtColor(image2, gray2, COLOR_BGR2GRAY);
		depth2.convertTo(depth_flt, CV_32FC1,1.f/5000.f);
		depth_flt.setTo(std::numeric_limits<float>::quiet_NaN(), depth2 == 0);
		depth2 = depth_flt;
		if(first == true){
			frame_prev->image = gray2;
			frame_prev->depth = depth2;
			first=false;
		}
		else{
			frame_curr->image = gray2;
			frame_curr->depth = depth2;
			bool res = odometry3->compute(frame_curr,frame_prev, Rt);
			if(res==true){
				Rt1 = Rt1*Rt;
				Mat rotationMat = Rt1(cv::Rect(0, 0, 3, 3)).clone();
				Mat translateMat = Rt1(cv::Rect(3, 0, 1, 3)).clone();
				//cout << "Rr " << rotationMat << endl;
				//cout << "Rt " << translateMat << endl;
				cout<< "Coordinates: x = "<<translateMat.at<double>(0)<<" "<<translateMat.at<double>(1)<<" "<<translateMat.at<double>(2)<<endl;
				myfile.open ("example.txt",ios::app);
				myfile <<translateMat.at<double>(0)<<"   "<<translateMat.at<double>(2)<< "\n";
				int x =     int(60.0f * translateMat.at<double>(0)) +
						800 / 2;
				int y =       int(60.0f * translateMat.at<double>(2)) +
						800 / 2;
				circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
				rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0),
						-1);
				sprintf(text, "Coordinates: x = %04fm y = %04fm z = %04fm",
						translateMat.at<double>(0),
						translateMat.at<double>(1),
						translateMat.at<double>(2));
				putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255),
						thickness, 8);
				myfile.close();
			}
			else{
			}
			if(!frame_prev.empty())
				frame_prev->release();
			std::swap(frame_prev, frame_curr);
			imshow("RGBD Trajectory", traj);
			namedWindow( "window", 0x00000001 ); // Create a window for display.
			cv::imshow( "window2", image2 ); // Show our image inside it.
		}
		waitKey(1);}
	myfile.close();
        return 1;
}
int test(void){
	Mat image, depth,image2,depth2;
	image = imread("/home/irobot/Downloads/pictures/R10QV2/python.png",1);
	//image2 = cv::imread("/home/irobot/Downloads/images/rgbd_dataset_freiburg1_desk/rgb/1305031453.391690.png",1);
	image2 = imread("/home/irobot/Downloads/pictures/R10QV2/python2.png",1);
	//depth2 = imread("/home/irobot/Downloads/images/rgbd_dataset_freiburg1_desk/depth/1305031453.404816.png", IMREAD_ANYDEPTH);
	depth = imread("/home/irobot/Downloads/pictures/R10QV2/pythond.png", IMREAD_ANYDEPTH);
	depth2 = imread("/home/irobot/Downloads/pictures/R10QV2/pythond2.png", IMREAD_ANYDEPTH);
	if(! image.data ||  ! image2.data || ! depth.data || ! depth2.data)                              // Check for invalid input
	{
		cout <<  "Could not open or find the image" << std::endl ;
		return -1;
	}
	Mat depth_flt;
	depth.convertTo(depth_flt, CV_32FC1,1.f/1000.f);
	depth_flt.setTo(std::numeric_limits<float>::quiet_NaN(), depth == 0);
	depth = depth_flt.clone();
	depth2.convertTo(depth_flt, CV_32FC1,1.f/1000.f);
	depth_flt.setTo(std::numeric_limits<float>::quiet_NaN(), depth2 == 0);
	depth2 = depth_flt.clone();
	Mat gray,gray2;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	cvtColor(image2, gray2, COLOR_BGR2GRAY);
	frame_curr->image = gray.clone();
	frame_curr->depth = depth.clone();
	frame_prev->image = gray2.clone();
	frame_prev->depth = depth2.clone();
	cout<<"Pictures loaded"<<endl;
	Mat cameraMatrix = Mat::eye(3,3,CV_32FC1);
	cameraMatrix.at<float>(0,0) = 925.72705078125f;
	cameraMatrix.at<float>(1,1) = 926.109619140625f;
	cameraMatrix.at<float>(0,2) = 652.9413452148438f;
	cameraMatrix.at<float>(1,2) = 358.74774169921875f;
	odometry= rgbd::ICPOdometry::create();
	odometry2 = rgbd::RgbdICPOdometry::create();
	odometry3 = rgbd::RgbdOdometry::create();
	odometry->setCameraMatrix(cameraMatrix);
	odometry2->setCameraMatrix(cameraMatrix);
	odometry3->setCameraMatrix(cameraMatrix);
	Mat Rt;
	bool res = odometry->compute(frame_curr,frame_prev, Rt);
	if(res==true){
		cout<<"worked"<<endl;
	}
	cout<<"ICPOdometry"<<endl;
	cout << "Rt " << Rt << endl;
	Mat rotationMat2 = Rt(cv::Rect(0, 0, 3, 3)).clone();
	Mat translateMat2 = Rt(cv::Rect(3, 0, 1, 3)).clone();
	cout << "Rr " << rotationMat2 << endl;
	cout << "Rt " << translateMat2 << endl;
        Mat Rt1=Mat::eye(4, 4, CV_64F);
	Rt1=Rt1*Rt;
	cout<<Rt1<<endl;
	cout<<rotationMatrixToEulerAngles(rotationMat2)<<endl;
	cout<<"RgbdICPOdometry"<<endl;
	res = odometry2->compute(frame_curr,frame_prev, Rt);
	if(res==true){
		cout<<"worked"<<endl;
	}
	cout << "Rt " << Rt << endl;
	rotationMat2 = Rt(cv::Rect(0, 0, 3, 3)).clone();
	translateMat2 = Rt(cv::Rect(3, 0, 1, 3)).clone();
	cout << "Rr " << rotationMat2 << endl;
	cout << "Rt " << translateMat2 << endl;
	cout<<"RgbdOdometry"<<endl;
	res = odometry3->compute(frame_curr,frame_prev, Rt);
	if(res==true){
		cout<<"worked"<<endl;
	}
	cout << "Rt " << Rt << endl;
	rotationMat2 = Rt(cv::Rect(0, 0, 3, 3)).clone();
	translateMat2 = Rt(cv::Rect(3, 0, 1, 3)).clone();
	cout << "Rr " << rotationMat2 << endl;
	cout << "Rt " << translateMat2 << endl;
	return 0;
	/*

	return 1;*/
}
