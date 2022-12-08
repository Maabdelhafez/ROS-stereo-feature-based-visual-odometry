// Author: Mohamed Ali Abdelhafez
// Email:  ma.abdelhafez@gmail.com
// Description: A ROS pub-sub node. 
// The node subscribes to two synchronized image topics simultaneously (left cam image and disparity image). 
// Within the callback function, relative motion is estimated betw. two successive frames k and k+1 for getting the 2D feature point and ( 3D with the help of the disparity image) 
// a rotation matrix and a translation vector are calculated and displayed on the terminal. 


#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include <geometry_msgs/Twist.h>

#include <sstream>
#include <iostream>
 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/imgcodecs/legacy/constants_c.h>


#include <opencv2/line_descriptor.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include <unistd.h>
#include <vector>
#include <string>


#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <iomanip>
#include <fstream>


using namespace std;
using namespace cv;
using namespace cv::line_descriptor;
using namespace message_filters;



//------Public Variables
bool currentLeftImageFrameIsSaved = false, nextLeftImageFrameIsSaved = false;
Mat nextLeftImageFrame; 
Mat currentLeftImageFrame ;
int leftCamFrameCount = 0;
cv::Mat disparityImage; 



//-----Function Prototypes
void pointsFeatureMatcher (Mat &currentFrame,  Mat &nextFrame, std::vector<KeyPoint> &currentFramePoints, std::vector<KeyPoint> &nextFramePoints, std::vector<DMatch> &matches);

Point2d normalized2DPixelPointToCamCoordinates(const Point2d &p, const Mat &K);


class MyClass 
{
public:
    MyClass() :
    
      pointsImageTransport(pointsNodeHandle),
  
      leftCamSub( pointsImageTransport, "/points/left_image", 10 ),
      disparityImageSub( pointsImageTransport,"/points/disparity_image", 10 ),  
  
      sync( MySyncPolicy( 10 ), leftCamSub, disparityImageSub )
    {
      sync.registerCallback( boost::bind( &MyClass::localizationWithPointFeatures, this, _1, _2 ) );
    }

    //--- cam to world transformed R and t
    Mat Rw = (Mat_<double>(3, 3) << 1, 0, 0,  0, 1, 0, 0, 0, 1);
    Mat tw = (Mat_<double>(3, 1) << 0,0,0);



    void localizationWithPointFeatures (const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& disparityImageMsg)
    {
      cv_bridge::CvImagePtr sourceLeftImagePtr;
      cv_bridge::CvImagePtr disparityImagePtr;
  
    try
      {	
        disparityImagePtr = cv_bridge::toCvCopy (disparityImageMsg, sensor_msgs::image_encodings::TYPE_32FC1); 
        sourceLeftImagePtr = cv_bridge::toCvCopy (leftImageMsg, sensor_msgs::image_encodings::MONO8); 
        // otherwise use cv_bridge::CvImagePtr and toCvCopy and pass also the image encoding sensor_msgs::image_encodings::MONO8 or BGR8
        
        leftCamFrameCount ++;
      //  std::cout << "Frame count =" << leftCamFrameCount << std::endl;
      }


    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Left Republished Image could not be converted");
        return;
      }

    if (leftCamFrameCount == 1)
    {
      nextLeftImageFrame = sourceLeftImagePtr->image;

      disparityImage = disparityImagePtr->image.clone();
    }

    else if (leftCamFrameCount >1)
    {
      currentLeftImageFrame = nextLeftImageFrame ;
      currentLeftImageFrameIsSaved = true;
      nextLeftImageFrame = sourceLeftImagePtr->image;
      nextLeftImageFrameIsSaved = true;
      
      disparityImage = disparityImagePtr->image;
    }
  
    if ( currentLeftImageFrameIsSaved == true && nextLeftImageFrameIsSaved == true )
    {
      
      //-------------now we have captured two successive frames.. lets estimate the relative motion between them----------//

      Mat K = (Mat_<double>(3, 3) << 659.3598, 0, 605.000, 0, 659.0103, 501.6295, 0, 0, 1);
      float b = 0.310 ; float fx = 659.3598;

      std::vector<KeyPoint> currentFramePoints;
      std::vector<KeyPoint> nextFramePoints;
      std::vector<DMatch> matches;
      vector<Point3f> pts_3d;
      vector<Point2f> pts_2d;
      vector<float> dds;
 
    
      pointsFeatureMatcher (currentLeftImageFrame, nextLeftImageFrame, currentFramePoints, nextFramePoints, matches);
      cout << "Found matches:" << matches.size() << endl;  
      

      // getting the matched 2D points and their 3D correspondences
      for (DMatch m:matches) 
      {
        // getting depth by accessing disparity map
        float depth = ( fx * b ) / (disparityImage.at<float>(currentFramePoints[m.queryIdx].pt.y , currentFramePoints[m.queryIdx].pt.x));
        
        //Reliable depth by practice
        if (depth <= 0.4 || depth > 14 )
        continue;        

      dds.push_back(depth);
      Point2d p1 = normalized2DPixelPointToCamCoordinates(currentFramePoints[m.queryIdx].pt, K);
      pts_3d.push_back(Point3f(p1.x * depth, p1.y * depth, depth));
      pts_2d.push_back(nextFramePoints[m.trainIdx].pt);
     
      }


  //==================== adding 3d-2d points extracted from detected and matched lines =====================================

  //-----create a random binary mask
    Mat mask1 = Mat::ones( currentLeftImageFrame.size(), CV_8UC1 );
    Mat mask2 = Mat::ones( nextLeftImageFrame.size(), CV_8UC1 );
   
    Ptr<BinaryDescriptor> binDesc = BinaryDescriptor::createBinaryDescriptor(); 

    vector<KeyLine> foundLines_1;
    vector<KeyLine> foundLines_2;

    binDesc->detect(currentLeftImageFrame, foundLines_1, mask1); // detect lines
    binDesc->detect(nextLeftImageFrame, foundLines_2, mask2); // detect lines

   

  //------ Descriptor and matching-------------//

    cv::Mat descriptor_1, descriptor_2;
    vector<DMatch> goodLineMatches;
    
    binDesc->compute( currentLeftImageFrame, foundLines_1, descriptor_1);
    binDesc->compute( nextLeftImageFrame, foundLines_2, descriptor_2);

  ( *binDesc )( currentLeftImageFrame, mask1, foundLines_1, descriptor_1, false, false );
    ( *binDesc )( nextLeftImageFrame, mask2, foundLines_2, descriptor_2, false, false );

    Mat img_out_1 = currentLeftImageFrame.clone();
    Mat img_out_2 = nextLeftImageFrame.clone();
    cvtColor( img_out_1, img_out_1, COLOR_GRAY2BGR );
    cvtColor( img_out_2, img_out_2, COLOR_GRAY2BGR );

  /* select keylines from first octave and their descriptors */
    std::vector<KeyLine> lbd_octave1, lbd_octave2;
    Mat current_lbd, next_lbd;
   
    for ( int i = 0; i < (int) foundLines_1.size(); i++ )
    {
       KeyLine currentLine = foundLines_1[i];
      if( foundLines_1[i].octave == 0 )
      {
        lbd_octave1.push_back( foundLines_1[i] ); 
        current_lbd.push_back( descriptor_1.row( i ) );
      }
      Point p1 = Point2d (currentLine.startPointX, currentLine.startPointY); // x1,y1
      Point p2 = Point2d (currentLine.endPointX, currentLine.endPointY); // x2,y2
      line( img_out_1, p1, p2, Scalar( 0,0,255), 2);

    }
  
    for ( int j = 0; j < (int) foundLines_2.size(); j++ )
    {
      KeyLine currentLine2 = foundLines_2[j];

      if( foundLines_2[j].octave == 0 )
      {
        lbd_octave2.push_back( foundLines_2[j] );
        next_lbd.push_back( descriptor_2.row( j ) );
      }
      Point p3 = Point2d (currentLine2.startPointX, currentLine2.startPointY); // x1,y1
      Point p4 = Point2d (currentLine2.endPointX, currentLine2.endPointY); // x2,y2
      line( img_out_2, p3, p4, Scalar( 0,0,255), 2);

    }
  
    /* create a BinaryDescriptorMatcher object */
    Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
  
    /* require match */
    std::vector<DMatch> lineMatches;
    bdm->match( current_lbd, next_lbd, lineMatches );
  
    /* select best matches */
    for ( int i = 0; i < (int) lineMatches.size(); i++ )
    {
      if( lineMatches[i].distance < 25 )
        goodLineMatches.push_back( lineMatches[i] );
    }
  
    cout<< " No of good Line Matches:  " << goodLineMatches.size() <<endl;

    //---------------drawing line matches
    std::vector<char> mask( lineMatches.size(), 1 );
    cv::Mat outImg; 
    
    // show lines on image
    imshow( "Line Segments in previous frame", img_out_1 );
    imshow( "Line Segments in current frame", img_out_2 );
 
    
    drawLineMatches( img_out_1, lbd_octave1, img_out_2, lbd_octave2, goodLineMatches, outImg, Scalar::all( -1 ), Scalar( 0,0,255), mask, DrawLinesMatchesFlags::DEFAULT );
    
    cv::namedWindow("Line Matches", cv::WINDOW_KEEPRATIO);
    imshow( "Line Matches", outImg );
    resizeWindow("Line Matches", 1800,800);
    waitKey(3);

//--------- Now get the points 3d-2d pair
  for (DMatch l:goodLineMatches) {
    
    KeyLine currentLine_1 = foundLines_1[l.queryIdx];
    KeyLine currentLine_2 = foundLines_2[l.trainIdx];
    
    vector<double> depth_stPts;
    vector<double> depth_endPts; 

    vector<double> pts_2d_fromLines;
    vector<double> pts_3d_fromLines;

    float depth_stPt = ( fx * b ) / (disparityImage.at<float>(currentLine_1.startPointY , currentLine_1.startPointX));
    if (depth_stPt <= 2 || depth_stPt > 25)  continue;  // bad depth       
    depth_stPts.push_back(depth_stPt);  

    float depth_endPt = ( fx * b ) / (disparityImage.at<float>(currentLine_1.endPointY , currentLine_1.endPointX));
    if (depth_endPt <= 2 || depth_endPt > 25)    continue;  // bad depth
    depth_endPts.push_back(depth_endPt);  
  
    //---add the matched 2D-3D START Points  
    Point2d my2dStartPoint = normalized2DPixelPointToCamCoordinates(foundLines_2[l.trainIdx].getStartPoint(), K);
    pts_2d.push_back(my2dStartPoint);  
    Point3f my3dStartPoint (my2dStartPoint.x * depth_stPt, my2dStartPoint.y* depth_stPt, depth_stPt);
    pts_3d.push_back(my3dStartPoint);  
          
    //---add the matched 2D-3D END Points      
    Point2d my2dEndPoint   = normalized2DPixelPointToCamCoordinates(foundLines_2[l.trainIdx].getEndPoint(), K);
    pts_2d.push_back(my2dEndPoint);
    Point3f my3dEndPoint (my2dEndPoint.x * depth_endPt, my2dEndPoint.y* depth_endPt, depth_endPt);
    pts_3d.push_back(my3dEndPoint); 

    }
     cout<< "pts_3d size After lines "  << pts_3d.size() <<endl; 
   //==========================END of 3d-2d points from lines======================================



      // relative rotation 
      Mat R; // relative rotation matrix
      cv::Mat r(3,1,cv::DataType<double>::type); // relative rotation vector
      cv::Mat t(3,1,cv::DataType<double>::type);  // relative translation vector 
      

      cv::solvePnPRansac(pts_3d, pts_2d, K, Mat(), r, t);

      cv::Rodrigues(r, R);  

      //---- inverse transformation of relative motion
      Mat R_transpose; // inverse of R
      transpose(R, R_transpose); // inverse (R) = transpose (R)
      Mat t_inverse = -R_transpose*t;
      

      //----cam to world transformation
      Rw = Rw * R_transpose;
      tw = tw + R_transpose * t_inverse;
     // tw = tw*1000;

      cv::Mat rw(3,1,cv::DataType<double>::type);
      cv::Rodrigues(Rw, rw);
      
 
      Mat eulerAngles = rw*180.0/M_PI; // absolute // rad to degree as column vector
      
      cout << "World pose: eulerAngles=" << eulerAngles << ", tw=" << tw << endl; 
      
      //---- save to log as row vector
      Mat ew1, tw1; 
      transpose(eulerAngles, ew1);
      transpose(tw, tw1); 
      //poseLog << ew1 << ",   " << tw1 << endl; 
      

     //----draw depth labels on left frame (old) 
      Mat depthLables = currentLeftImageFrame;
      cv::cvtColor(depthLables, depthLables, cv::COLOR_GRAY2BGR);

      for(int i=0; i<dds.size(); i++)
      {
        Point2f featurePoint = pts_2d[i];
        float depthOfFeaturePoint = dds[i];
        //cout << depthOfFeaturePoint << " , " ;
        
        stringstream drawDepthLables;
        drawDepthLables << std::setprecision(2) <<  depthOfFeaturePoint;
        cv::putText(depthLables, drawDepthLables.str(), featurePoint, FONT_HERSHEY_COMPLEX, 1,{0,165,255}, 2 );//lable depth
      }
        cout << endl;
        cv::namedWindow("Depth", cv::WINDOW_KEEPRATIO);
        imshow("Depth", depthLables);
        waitKey(1);
            
    }

   }


  ros::NodeHandle pointsNodeHandle;
  image_transport::ImageTransport pointsImageTransport;
  
  //subcriber
  typedef image_transport::SubscriberFilter PointsSubscriber;
  PointsSubscriber leftCamSub;
  PointsSubscriber disparityImageSub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;

  //if desired we can also publish estimated motion on this topic
  ros::Publisher pointsPosePublisher = pointsNodeHandle.advertise<geometry_msgs::Twist>("/pose_estimation/pointss", 100);
  
};





int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "points");

  MyClass mc;  

  ros::spin();

  return 0;
 
}

 

 
void pointsFeatureMatcher ( Mat &currentLeftImageFrame,  Mat &nextLeftImageFrame, std::vector<KeyPoint> &currentFramePoints, std::vector<KeyPoint> &nextFramePoints, std::vector<DMatch> &matches) 
{
    
  Mat currentFrameDescriptor, nextFrameDescriptor;
    
  Ptr<FeatureDetector> detector = ORB::create(1000);
  Ptr<DescriptorExtractor> descriptor = ORB::create(1000);
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    
  //Detect Oriented FAST corner positions
  detector->detect(currentLeftImageFrame, currentFramePoints);
  detector->detect(nextLeftImageFrame, nextFramePoints);

  //Calculate the BRIEF descriptor based on the corner position
  descriptor->compute(currentLeftImageFrame, currentFramePoints, currentFrameDescriptor);
  descriptor->compute(nextLeftImageFrame, nextFramePoints, nextFrameDescriptor);

  //Match the Brief descriptors in the two images, using the Hamming distance
  vector<DMatch> pointsMatch;
  matcher->match(currentFrameDescriptor, nextFrameDescriptor, pointsMatch);   // BFMatcher matcher ( NORM_HAMMING );

  //set min/max distance for matching
  double min_dist = 10000, max_dist = 0;

  //Find the minimum and maximum distances between all matches, that is, 
  //the distances between the most similar and least similar two sets of points
  for (int i = 0; i < currentFrameDescriptor.rows; i++) 
  {
    double dist = pointsMatch[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
    
  //When the distance between descriptors is greater than twice the minimum distance, 
  //it is considered that the matching is wrong. But sometimes the minimum distance will be very small, 
  //and an empirical value of 30 is set as the lower limit.
  for (int i = 0; i < currentFrameDescriptor.rows; i++) 
  {
   if (pointsMatch[i].distance <= max(2*min_dist, 30.0)) 
   {
    matches.push_back(pointsMatch[i]);
   }
  }
  
  Mat matchedPoints;
  drawMatches(currentLeftImageFrame, currentFramePoints, nextLeftImageFrame, nextFramePoints, matches, matchedPoints);
  // cv::namedWindow("Matched 2D ORB Features", cv::WINDOW_KEEPRATIO);
  // imshow("Matched 2D ORB Features", matchedPoints);
  // resizeWindow("Matched 2D ORB Features", 1800,400);
  // waitKey(1);
    
}


Point2d normalized2DPixelPointToCamCoordinates(const Point2d &p, const Mat &K) 
{
  return Point2d ( (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0) , (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1) );
}




