// Author: Mohamed Ali Abdelhafez
// Email:  ma.abdelhafez@gmail.com
// Description: A pub-sub node. 
// subscriber gets the image msgs from topic 1 and republishes them on topic 2 
// after editing 1) the header of every imag msg for synchronization and 2) rectifying the images from topic 1.




#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "std_msgs/Header.h"
 

using namespace std;
using namespace cv;

 
 
class RepubRightCam
{

  ros::NodeHandle repubRightCamNodeHandle;
  image_transport::ImageTransport repubRightCamImageTransport;
  image_transport::Subscriber rightCamTopicSub;
  image_transport::Publisher rightCamTopicRepub;


public:

  RepubRightCam()
    : repubRightCamImageTransport(repubRightCamNodeHandle)
  {
    rightCamTopicSub = repubRightCamImageTransport.subscribe ("/front_right/image_raw", 1, &RepubRightCam::republishRightCamTopic, this);
    rightCamTopicRepub = repubRightCamImageTransport.advertise ("/points/right_image", 1);
  }


  ~RepubRightCam()
  {
     
  }



 void republishRightCamTopic(const sensor_msgs::ImageConstPtr& msg)
 {

   //std::cout << "Hamada beyel3ab" << std::endl;
      
      
    cv_bridge::CvImageConstPtr rightSrcImagePtr;
    try
    {
      rightSrcImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      // otherwise use cv_bridge::CvImagePtr and toCvCopy and pass also the image encoding sensor_msgs::image_encodings::MONO8 or BGR8
    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Image could not be converted from '%s' to 'mono8'.", msg->encoding.c_str());
      return;
    }


    cv::Mat rightSrcImage = rightSrcImagePtr->image;
   // cv::imshow("original right image", rightSrcImage);
   // cv::waitKey(3);

  
 //-------------------------------Image rectification----------------------------//

    cv::Mat leftCamMatrix(3, 3, cv::DataType<float>::type);

    leftCamMatrix.at<float>(0, 0) = 659.3598; // fx
    leftCamMatrix.at<float>(0, 1) = 0.0f; 
    leftCamMatrix.at<float>(0, 2) = 605.0000; // cx

    leftCamMatrix.at<float>(1, 0) = 0.0f;
    leftCamMatrix.at<float>(1, 1) = 659.0103; //fy
    leftCamMatrix.at<float>(1, 2) = 501.6295 ; // cy

    leftCamMatrix.at<float>(2, 0) = 0.0f;
    leftCamMatrix.at<float>(2, 1) = 0.0f;
    leftCamMatrix.at<float>(2, 2) = 1.0f;

     leftCamMatrix.convertTo(leftCamMatrix, CV_64F);


    cv::Mat leftCamDistCoeffs(5, 1, cv::DataType<float>::type);  

    leftCamDistCoeffs.at<float>(0, 0) =  -0.3307 ; //k1
    leftCamDistCoeffs.at<float>(1, 0) = 0.1350 ;    // k2   
    leftCamDistCoeffs.at<float>(2, 0) =  0.0004 ;  // p1
    leftCamDistCoeffs.at<float>(3, 0) = 0.0006;  // p2
    leftCamDistCoeffs.at<float>(4, 0) =  -0.0283;   // k3

     leftCamDistCoeffs.convertTo(leftCamDistCoeffs, CV_64F);


    cv::Mat rightCamDistCoeffs(5, 1, cv::DataType<float>::type);  

    rightCamDistCoeffs.at<float>(0, 0) = -0.3124; //k1
    rightCamDistCoeffs.at<float>(1, 0) =  0.1094;    // k2   
    rightCamDistCoeffs.at<float>(2, 0) =  0.0007 ;  // p1
    rightCamDistCoeffs.at<float>(3, 0) =  0.0012 ;  // p2
    rightCamDistCoeffs.at<float>(4, 0) =  -0.0180 ;   // k3

    rightCamDistCoeffs.convertTo(rightCamDistCoeffs, CV_64F);


    cv::Mat rightCamMatrix(3, 3, cv::DataType<float>::type);

    rightCamMatrix.at<float>(0, 0) = 657.6514 ; // fx
    rightCamMatrix.at<float>(0, 1) = 0.0f; 
    rightCamMatrix.at<float>(0, 2) =  594.7421 ; // cx

    rightCamMatrix.at<float>(1, 0) = 0.0f;
    rightCamMatrix.at<float>(1, 1) = 658.0538; //fy
    rightCamMatrix.at<float>(1, 2) = 500.2612 ; // cy

    rightCamMatrix.at<float>(2, 0) = 0.0f;
    rightCamMatrix.at<float>(2, 1) = 0.0f;
    rightCamMatrix.at<float>(2, 2) = 1.0f;

     rightCamMatrix.convertTo(rightCamMatrix, CV_64F);



 cv::Mat rotMatrixLeftToRight(3, 3, cv::DataType<float>::type);

    rotMatrixLeftToRight.at<float>(0, 0) = 1.0000; 
    rotMatrixLeftToRight.at<float>(0, 1) = -0.0052; 
    rotMatrixLeftToRight.at<float>(0, 2) =  0.0051 ;

    rotMatrixLeftToRight.at<float>(1, 0) =  0.0051;
    rotMatrixLeftToRight.at<float>(1, 1) =  0.9997 ;
    rotMatrixLeftToRight.at<float>(1, 2) = 0.0232; 

    rotMatrixLeftToRight.at<float>(2, 0) = -0.0052;
    rotMatrixLeftToRight.at<float>(2, 1) = -0.0232;
    rotMatrixLeftToRight.at<float>(2, 2) = 0.9997;

    rotMatrixLeftToRight.convertTo(rotMatrixLeftToRight, CV_64F);

  

    cv::Mat transVecLeftToRight(3, 1, cv::DataType<float>::type);  

    transVecLeftToRight.at<float>(0, 0) = 310.8613 ; //x in mm
    transVecLeftToRight.at<float>(1, 0) = 1.3962 ; // y  in mm
    transVecLeftToRight.at<float>(2, 0) =   0.1188 ;  // z   in mm
     
     transVecLeftToRight.convertTo(transVecLeftToRight, CV_64F);


    cv::Size frameSize(1224, 1024);
    cv::Mat  R1, R2, P1, P2, Q, rightMapX, rightMapY;
    R1.convertTo(R1, CV_64F);
    R2.convertTo(R2, CV_64F);
    P1.convertTo(P1, CV_64F);
    P2.convertTo(P2, CV_64F);
    Q.convertTo(Q, CV_64F);


//------------------only for right images----------------------------------//


    // from stereoRectify we get output R1, R2, P1, P2, Q
    cv::stereoRectify(leftCamMatrix, leftCamDistCoeffs, rightCamMatrix, rightCamDistCoeffs, rightSrcImage.size(), rotMatrixLeftToRight, transVecLeftToRight, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0.0, rightSrcImage.size() ); // CALIB_ZERO_DISPARITY, 0.0,

    // now we pass the R2, P2 we got from stereoRectify to init the Maps
    cv::initUndistortRectifyMap(rightCamMatrix, rightCamDistCoeffs, R2, P2, rightSrcImage.size(), CV_32FC1, rightMapX, rightMapY);

    cv::remap(rightSrcImage, rightSrcImage , rightMapX, rightMapY, cv::INTER_LINEAR); // should be cv::remap(sourceImg, undistortedImg,...etc  ) but I overwrite the original image i.e. source image == undistorted image

    // cv::imshow("rectified right image", rightSrcImage);
    // cv::waitKey(33);

//--------------------------------------end of rectification-------------------------------------------//
    




//-----------------------------------------crop the rectified image if desired---------------------------------//

cv::Rect cropped_region(50, 0, 1124, 724);

rightSrcImage = rightSrcImage (cropped_region);

//cv::imshow("cropped image ",rightSrcImage);
//cv::waitKey(33);
//-------------------------------------------------end of crop-----------------------//

	//to be republished Msg
    sensor_msgs::Image rightCamImageMsg;
  
  	//editing of Msg Header
	  std_msgs::Header rightImageHeader;
    
    rightImageHeader.stamp = ros::Time::now();
    rightImageHeader.frame_id = "/repub_right_image" ;
   
    
 	  //Reconverting to ROS Msg and applying the new Header
    cv_bridge::CvImage rightImgRebridge;
    rightImgRebridge = cv_bridge::CvImage(rightImageHeader, "mono8", rightSrcImage);
    rightImgRebridge.toImageMsg(rightCamImageMsg);

 
    ros::Rate loop_rate(20); // no. of msgs per sec
  
    // publishing
    rightCamTopicRepub.publish(rightCamImageMsg);
    

    ros::spinOnce(); // process incoming messages
    loop_rate.sleep(); 


 }


};



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "repubRightCam");
   
  RepubRightCam rrc ;
   
  ros::spin();
  
  return 0;
   
}

