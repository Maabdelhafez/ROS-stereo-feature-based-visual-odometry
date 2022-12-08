// Author: Mohamed Ali Abdelhafez
// Email:  ma.abdelhafez@gmail.com
// Description: A ROS pub-sub node. 
// The node subscribes to left and right rectified and synchronized camera image topics simultaneously. 
// Within the callback function a disparity image is calculated and published on a third topic. 

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
 

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
//#include "std_msgs/Header.h"

#include <geometry_msgs/Twist.h>

#include <sstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include "opencv2/ximgproc/disparity_filter.hpp"

// #include <Eigen/Core>
//#include <chrono>
#include <unistd.h>
#include <vector>
#include <string>



using namespace std;
using namespace cv;
using namespace message_filters;
//using namespace Eigen;
using namespace cv::ximgproc;


class MyClass {

public:
  MyClass() :
    pointsImageTransport(pointsNodeHandle), 
 
    disparityLeftCamSub( pointsImageTransport, "/points/left_image" , 1 ), 
    disparityRightCamSub( pointsImageTransport, "/points/right_image" , 1 ),  
 
    sync( MySyncPolicy( 10 ), disparityLeftCamSub, disparityRightCamSub )
    {
      sync.registerCallback( boost::bind( &MyClass::rightCamCallbackToGetDisparityMap, this, _1, _2 ) );
    }


  void rightCamCallbackToGetDisparityMap(const sensor_msgs::ImageConstPtr& rightImageMsg, const sensor_msgs::ImageConstPtr& leftImageMsg)
  {
      cv_bridge::CvImagePtr sourceRightImagePtr;
      cv_bridge::CvImagePtr sourceLeftImagePtr;
    
      try
        { 
          sourceLeftImagePtr = cv_bridge::toCvCopy(leftImageMsg, sensor_msgs::image_encodings::MONO8);
          sourceRightImagePtr = cv_bridge::toCvCopy(rightImageMsg, sensor_msgs::image_encodings::MONO8); 
        }

      catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Right Image could not be converted ");
          return;
        }


      cv::Mat currentLeftImageFrame = sourceLeftImagePtr->image;
      cv::Mat currentRightImageFrame = sourceRightImagePtr->image;
      
      // cv::imshow("left cam rectified frame", currentLeftImageFrame );
      // cv::waitKey(3);
      //cv::imshow("right cam rectified frame", currentRightImageFrame );
      //cv::waitKey(3);


      cv::Mat sgbmDisparity, disparity, disparityMap;

  
  //--------------------------------------------------------------------------//

      //parameters for stereo matching and filtering
      double vis_mult = 7.0;
      int wsize = 3;
      int max_disp = 16 * 6;
      double lambda = 70000.0;
      double sigma = 7;


      //Some object instantiation that can be done only once
      Mat left_for_matcher,right_for_matcher;
      Mat left_disp, right_disp;
      Mat filtered_disp;
      Ptr<DisparityWLSFilter> wls_filter;
      Mat filtered_disp_vis;

      // downsample images to speed up results on cost of quality
      max_disp/=2;
      // if(max_disp%16 != 0) max_disp += 16-(max_disp%16);
      resize(currentLeftImageFrame, left_for_matcher,Size(),1,1);
      resize(currentRightImageFrame, right_for_matcher,Size(),1,1);

      Ptr<StereoSGBM> left_matcher  = StereoSGBM::create (0, max_disp, wsize, 240*wsize*wsize, 960*wsize*wsize, 0, 63, 0, 50, 5); // (0, max_disp,wsize,2*wsize*wsize, 15*wsize*wsize, 10, 5, 10, 50, 8) wsize==3 lambda 20000 sigma 3
      
      // According to opencv filtering disparity guide
      left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
      wls_filter = createDisparityWLSFilter(left_matcher);
      Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

      left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
      right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

      //Weighted least square filter
      wls_filter->setLambda(lambda);
      wls_filter->setSigmaColor(sigma);
      wls_filter->filter(left_disp,currentLeftImageFrame,filtered_disp,right_disp);
    
      Mat finalDisparity = filtered_disp;
      filtered_disp.convertTo (finalDisparity, CV_32F, 1.0 / 1.0f) ;

      getDisparityVis(finalDisparity,filtered_disp_vis,vis_mult);
      imshow("filtered disparity", filtered_disp_vis );

      cv::waitKey(35);


      sensor_msgs::Image disparityImageMsg; // >> message to be sent

      std_msgs::Header disparityHeader;
      disparityHeader.stamp = ros::Time::now();
      disparityHeader.frame_id = "/disparity_image";
      
      cv_bridge::CvImage img_bridge;
      img_bridge = cv_bridge::CvImage(disparityHeader,sensor_msgs::image_encodings::TYPE_32FC1 , finalDisparity);// sensor_msgs::image_encodings::TYPE_32FC1
      img_bridge.toImageMsg(disparityImageMsg);
      
      disparityImagePublisher.publish(disparityImageMsg);
        
  }

  private:
  ros::NodeHandle pointsNodeHandle;
  image_transport::ImageTransport pointsImageTransport;
  
  //subcriber
  typedef image_transport::SubscriberFilter PointsSubscriber;
  PointsSubscriber disparityLeftCamSub;
  PointsSubscriber disparityRightCamSub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;
  //publisher
  image_transport::Publisher disparityImagePublisher = pointsImageTransport.advertise("/points/disparity_image", 1);

  
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth");
  
  MyClass mc;  
       
  ros::spin();

  
  return 0;
 
}


