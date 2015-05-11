/**
* @file single_board.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief ROS version of the example named "simple_board" in the Aruco software package.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/highlyreliablemarkers.h>
#include <aruco/chromaticmask.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_sys/utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
 
#include <std_msgs/Float32MultiArray.h>

#include <memory>

using namespace aruco;
using namespace cv;
using namespace std;



void getHueImg(const cv::Mat &rgbImg, cv::Mat &hImg)
{
  cv::Mat hsvImg;
  vector<cv::Mat> hsvImg_channels;
  cvtColor(rgbImg, hsvImg, CV_RGB2HSV);
  cv::split(hsvImg, hsvImg_channels);
  hsvImg_channels[0].copyTo(hImg);
}

class ArSysSingleBoard
{
	private:
		cv::Mat inImage, resultImg, TheInputImage,TheInputImageCopy, TheInputImageH;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
		MarkerDetector *mDetector;
		vector<Marker> markers;
		BoardConfiguration the_board_config;
		BoardDetector the_board_detector;
		//Board the_board_detected;
		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher occlusion_mask_pub;
		image_transport::Publisher debug_pub;
		ros::Publisher pose_pub;
		ros::Publisher transform_pub; 
		ros::Publisher position_pub;
		ros::Publisher modelView_pub;
		std::string board_frame;
	
		// added by me
		std::string dictPath; 
		ChromaticMask TheChromaticMask;
		Dictionary D;
		vector<cv::Point3f> corners;
        int index=0;
        int waitTime=5;
        //pair<double,double> AvrgTime(0,0) ;
		// end added by me

		double marker_size;
		std::string board_config;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		tf::TransformListener _tfListener;


	public:
		ArSysSingleBoard()
			: cam_info_received(false),
			nh("~"),
			it(nh)
		{
			image_sub = it.subscribe("/image", 1, &ArSysSingleBoard::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysSingleBoard::cam_info_callback, this);

			modelView_pub = nh.advertise<std_msgs::Float32MultiArray>("modelview",5);

			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);
			occlusion_mask_pub = it.advertise("occlusion_mask", 1); // ADDED BY ME
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

			nh.param<double>("marker_size", marker_size, 0.05);
			nh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
			
			nh.param<std::string>("board_frame", board_frame, "");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, false);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, false);

			the_board_config.readFromFile(board_config.c_str());


			ROS_INFO("ArSys node started with marker size of %f m and board configuration: %s",
					 marker_size, board_config.c_str());
		}

		void processKey(char key) {
			//cout << "Key pressed: " << key << endl;
		    switch (key) {
			    case 's':
			        if (waitTime==0) waitTime=10;
			        else waitTime=0;
			        break;

			    // calibrate mask
			    case 'm':
			    	cout << "Training" << endl;
					float prob = (float)the_board_detector.getDetectedBoard().size() / (float)the_board_detector.getDetectedBoard().conf.size();
					//TheChromaticMask.detectBoard( TheInputImageH );
					if(prob>0.2) TheChromaticMask.train(TheInputImageH, the_board_detector.getDetectedBoard());
					//if(prob>0.2) TheVisibilityMask.calibrate(TheBoardDetector.getDetectedBoard(), TheInputImageH, TheCameraParameters, TheMarkerSize);
					break;
				//default: 
				//	return;
		    }
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			if(!cam_info_received) return;

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
				resultImg = cv_ptr->image.clone();



				//detection results will go into "markers"
				markers.clear();
				//Ok, let's detect
				//mDetector->detect(inImage, markers);//, camParam, marker_size, false);
				//Detection of the boade
				//float probDetect=the_board_detector.detect(markers, the_board_config, the_board_detected, camParam, marker_size);
				
				// ADDED BY ME - HRM STUFF

				//TheInputImage = resultImg;
				resultImg.copyTo(TheInputImageCopy);
				getHueImg(resultImg, TheInputImageH);

				index++;
				//float probDetect=0;
				//cout << "================" << endl;
				//cout << TheInputImageH << endl;
				float probDetect=the_board_detector.detect(TheInputImageH);
				//float probDetect=the_board_detector.detect(inImage);
				Board& the_board_detected = the_board_detector.getDetectedBoard();
				
				//the_board_detector.getMarkerDetector().detect(TheInputImageH, markers);
				markers = the_board_detector._vmarkers;
				//cout << "After detection" << endl;
				// END ADDED BY ME
				if (probDetect > 0.0)
				{
					tf::Transform transform = ar_sys::getTf(the_board_detected.Rvec, the_board_detected.Tvec);

					tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, board_frame);

					geometry_msgs::PoseStamped poseMsg;
					tf::poseTFToMsg(transform, poseMsg.pose);
					poseMsg.header.frame_id = msg->header.frame_id;
					poseMsg.header.stamp = msg->header.stamp;
					pose_pub.publish(poseMsg);

					geometry_msgs::TransformStamped transformMsg;
					tf::transformStampedTFToMsg(stampedTransform, transformMsg);
					transform_pub.publish(transformMsg);

					geometry_msgs::Vector3Stamped positionMsg;
					positionMsg.header = transformMsg.header;
					positionMsg.vector = transformMsg.transform.translation;
					position_pub.publish(positionMsg);

					// PUBLISH MODELVIEW MATRIX
					std_msgs::Float32MultiArray array;
					array.data.clear();

					double modelview[16];
					the_board_detected.glGetModelViewMatrix(modelview);

					for(int k=0; k<16; k++){ 
						array.data.push_back(modelview[k]);
					}

					modelView_pub.publish(array);

				} else {
					//cout << "prob is 0" << endl;
				}
				//for each marker, draw info and its boundaries in the image
				for(size_t i=0; draw_markers && i < markers.size(); ++i)
				{
					markers[i].draw(resultImg,cv::Scalar(0,0,255),2);
				}


				if(camParam.isValid() && marker_size != -1)
				{
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); ++i)
					{
						if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
					}
					//draw board axis
					if (probDetect > 0.0) CvDrawingUtils::draw3dAxis(resultImg, the_board_detected, camParam);
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0)
				{
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.frame_id = msg->header.frame_id;
					debug_msg.header.stamp = msg->header.stamp;
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector->getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}

				// ADDED BY ME - HRM STUFF 
				
				Mat dummy = cv::Mat::zeros(camParam.CamSize.height, camParam.CamSize.width, CV_8UC1);
				Mat mask;
				// create mask
				cv::imshow("mask",dummy);
			    if(TheChromaticMask.isValid()) { 
			    	//cout << "is valid" << endl;
			      	if(probDetect>0.2) {
				
		 	   		 	
						TheChromaticMask.classify2(TheInputImageH,the_board_detector.getDetectedBoard());
		             	//AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency(); AvrgTime.second++;
			            //cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;		
				
						if(index%200==0) {
				  			cout << "Updating Mask" << endl;
				  			TheChromaticMask.update(TheInputImageH);
						}
			     	} else {
			     		TheChromaticMask.resetMask();
			     	}
					// TheVisibilityMask.createMask(TheInputImageH);
		     		
			     	// closing 
			     	int morph_size = 5;

			     	Mat inv = Mat::ones(camParam.CamSize.height, camParam.CamSize.width, CV_8UC1);

			     	Mat invMask = inv - TheChromaticMask.getMask();

			     	Mat element = getStructuringElement(0, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
			     	morphologyEx( invMask, mask, cv::MORPH_OPEN, element );

			     	//cv::morphologyEx(TheChromaticMask.getMask(), mask, cv::MORPH_OPEN, Mat::zeros(5,5, CV_8UC1), Point(-1,-1), 5 );

			      	cv::imshow("mask",invMask*255);
			      	cv::imshow("mask closed",mask*255);
					//cv::imshow("maskcells",TheChromaticMask.getCellMap()*10);

		    	} else { 
		    		//cout << "is invalid" << endl;
		    	}

		    	//if (  TheOutVideoFilePath!="") {
                	//create a beautiful compiosed image showing the mask
                	//first create a small version of the mask image
                	if(TheChromaticMask.isValid()) {
					  	cv::Mat smallMask;
					  	cv::resize( TheChromaticMask.getMask()*255,smallMask,cvSize(TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
					  	cv::Mat small3C;
					  	cv::cvtColor(smallMask,small3C,CV_GRAY2BGR);
					  	cv::Mat roi=TheInputImageCopy(cv::Rect(0,0,TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
					  	small3C.copyTo(roi);
					}
                	//VWriter<<TheInputImageCopy;
            	//}

            	char key = 0;
            	key=cv::waitKey(waitTime);//wait for key to be pressed
            	processKey(key);

            	if(occlusion_mask_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage occlusion_msg;
					occlusion_msg.header.frame_id = msg->header.frame_id;
					occlusion_msg.header.stamp = msg->header.stamp;
					occlusion_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
					occlusion_msg.image = mask;
					occlusion_mask_pub.publish(occlusion_msg.toImageMsg());
				}

				// END ADDED BY ME

			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();

			cv::Size fixed; 
			fixed.width = camParam.CamSize.height;
			fixed.height = camParam.CamSize.width;
			camParam.CamSize = fixed;

			// BEGIN HRM STUFF
			nh.param<std::string>("dictionary", dictPath, "");
			if (!D.fromFile(dictPath)) {
            	ROS_ERROR("Could not open dictionary file");
            } else { 
            	ROS_INFO("Dictionary loaded.");
            }
            
        	HighlyReliableMarkers::loadDictionary(D);
			//ROS_INFO("Dictionary loaded0.");
			TheChromaticMask.setParams(5,5,0.0001,camParam,the_board_config, marker_size);
			//ROS_INFO("Dictionary loaded0.5.");
			the_board_detector.setYPerperdicular(false); 
			//ROS_INFO("Dictionary loaded1.");
			the_board_detector.setParams(the_board_config,camParam, marker_size);
			//ROS_INFO("Dictionary loaded2.");
			the_board_detector.getMarkerDetector().setThresholdParams( 21,7); // for blue-green markers, the window size has to be larger
			//ROS_INFO("Dictionary loaded3.");
			//the_board_detector.getMarkerDetector().getThresholdParams( ThresParam1,ThresParam2);
			the_board_detector.getMarkerDetector().setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
			//the_board_detector.getMarkerDetector().setMakerDetectorFunction(aruco::FiducidalMarkers::detect);
			//ROS_INFO("Dictionary loaded4.");
			the_board_detector.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::LINES);
			//ROS_INFO("Dictionary loaded5.");
			//the_board_detector.getMarkerDetector().setWarpSize((D[0].n()+2)*8);
			//ROS_INFO("Dictionary loaded6.");
			the_board_detector.getMarkerDetector().setMinMaxSize(0.005, 0.5);	
			//ROS_INFO("Dictionary loaded7.");
			mDetector = &the_board_detector.getMarkerDetector();

			ROS_INFO("HRM finished.");
        	// END HRM STUFF
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_single_board");

	ArSysSingleBoard node;

	ros::spin();
}
