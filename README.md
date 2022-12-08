# ROS-stereo-feature-based-visual-odometry

Back when I started learning about visual motion estimation for my master thesis, there were too few resources on the internet and it seemed to be a dream that will never happen to find a single piece of information that explains the main idea in a very clear and simple way or to find some Github repo that has a minimal amount of code to explain the idea with no “fancy” coding. 
Today I make this dream happen. 


![efeu_comb](https://user-images.githubusercontent.com/50604118/206424575-81c58070-fe9e-4b86-bf13-770bf518eb23.jpg)

This is a mobile robot called Efeu. Efeu has a rigidly fixed stereo camera and this repo is about estimating the motion of Efeu via estimating the motion of the fixed stereo camera. The camera motion estimation is done using features extracted from images taken by the camera itself. Awesome, isn´t it? 
 

The theory behind the system is self-explainatory as described in the following figure: 

![sloution_scenario_2](https://user-images.githubusercontent.com/50604118/206425471-2f1de1eb-e423-4814-9db2-64f754feb427.jpg)

Images from left and right camera are captured then converted to grayscale and rectified. Features in successive rectified left images are detected and matched. 
Simultaneously, the rectified left and right images are then used to generate a disparity map and hence calculate the
depth. After that, the motion is estimated using the extracted 2D features and their corresponding 3D points which we get using depth information. 

The method that uses such a number n of 2d-3d point corresspondences is 
called the perspective-n-point i.e. a number n of matched 2d features from two successive left camera frames and their 3d correspondences. The following figure shows point features matching between two successive frames.. On the left is the
previous Efeu’s left camera image frame (the older frame). On the right is the current frame or the new frame from the same left camera.
Those matched pairs are what we mean by "matched 2d points".  

![point_matches](https://user-images.githubusercontent.com/50604118/206459381-6105b576-62de-4ded-ac22-d933ff879eb6.png)


Here we consider only the VO module of the visual SLAM. There are however other modules. For example, a map representing a reconstruction
of the world scene is generated using the 3D points. Also the estimated motion can be even optimized using techniques such as bundle adjustment. 
Finally, If the robot finds that it has revisited the same place, then
this means a closure of the loop, and this information is used to reduce the error of the
estimated relative ego-motion. However, as i explained above, here i want to keep stuff as minimal as possible and just get the motion estimation module done!


![ros_structure](https://user-images.githubusercontent.com/50604118/206437896-fdb5014d-21ab-4d82-bf7b-0013c19517cf.jpg)


Now we explain the ROS implementation. Each circle represents a ROS node, and each arrow represents a ROS topic. The system consists of four nodes
and six topics. All the nodes are pub-sub nodes that subscribe to a topic or several topics
and publish information on another topic. 
Node 1 (repubLeftCam.cpp) and node 2 (repubRightCam.cpp) receive the raw unrectified left and right camera images, respectively.
In these two nodes, the images are rectified, and timing is adjusted and required for future
synchronization, as we will explain.

Node 3 (depth.cpp) subscribes to both rectified left image sequence and right image sequence and uses
stereo matching and the SGBM algorithm to generate a disparity map.

Node 4 (points.cpp) receives a sequence of rectified left camera images and detects, describes, and
matches features between two consecutive images. Line features are also detected, and the
two peripheral points of every matched 2d line segment are extracted and added to the
matched point features.

Node 4 also receives a disparity map for the first image and uses the positions of the 2D
features to access the disparity map to obtain the disparity of each point feature. After
that, the depth for each feature is calculated using the disparity, the previously calculated
Efeu’s stereo camera baseline, and the left camera focal length.
Using the depth information and left camera matrix K, calculated in the calibration process,
we get a set of 3D points that correspond to the matched 2D points. 

Finally, the 3D-2D pairs are used for motion estimation using the PnP method. The estimated rotation matrix
and the 3D translation vector are finally published on a topic.
For the case of Node 3 and Node 4, the nodes subscribe to two topics; hence a synchronization
of the data included in those two pics is required. If the two topics are not
synchronized, node 3, for example, will do stereo matching between a left camera image
and a right camera image that do not represent the same 3D scene. In the case of node 4,
if no synchronization is done between the rectified left image sequence and the disparity
map generated by node 3, then the algorithm will access the wrong disparity map and will
obtain disparity values that do not correspond to the features detected in the rectified left
image.
Even a small error in the synchronization will lead to the stereo matching of different frame
counts or accessing the wrong disparity map, which will lead to wrong odometry results.
Hence a sufficient amount of time has been dedicated to ensure a good synchronization
policy. Furthermore, the algorithm has been tested numerous times in different testing
scenarios to verify the correctness of the synchronization policy.



Now let#s talk a little about stereo camera calibration and image rectification. 

Technically known as camera aperture, the camera hole must be small enough to get a sharp
image and large enough to get sufficient light. In order to achieve those two contradicting
goals, the use of a lens is essential.

A lens enables collecting as many rays of light as possible, while at the same time, we can
keep the camera aperture small to get a sharp image as desired. Also, a lens can help the
camera have a larger field of view (FOV). Hence, the use of a lens is essential.
However, using a lens causes image distortion, and a distortion model is needed. So the
pinhole model must be edited to include such a distortion model of the lens.
The manufacturer could have provided camera parameters, but now it is clear that the
manufacturer does not know which lens will be used with the camera and to what extent the image will be distorted. Because of the distortion caused by the lens, a calibration of
the camera is still needed.

The calibration process yields two important things : a new camera matrix K and the distortion coefficients of both radial and tangential distortion.

 The camera matrix K and the distortion coefficients are then used to rectify the images. The rectified images are vertically
aligned, which is a must for the stereo matching process used to generate the disparity map.

A chess board pattern with a known square size is used in the calibration process. A person
holds the chess pattern in front of the camera while an algorithm is capturing images every
three seconds, for example, to give the person enough time to rotate the chess pattern and
move it in various directions. The pattern should also be tilted from time to time.
The algorithm responsible for rectifying the images is included node 1 and node 2. The algorithm for camera calibration is not included in this ros package as it is used only once for determining K and the distortion coefficients. It is also possible to use MATLAB computer vision toolbox which i recommend. If your lens has a very wide angle, you can also use fisheye module which is provided by openCV. 

The calibration process is not straight forward and it takes a lot of time and effort to get the best results, but the effort will pay off when you get a good estimation of K. 
You have to know that the matrix K MUST be as accurate as possible , because it is not only used for rectification but it is directly involved in the moption estimation equation, so be patient. A good estimation of K and the distortion coefficients will help the rectification algorithm get a result such as the following image.


![b4__after_rectification](https://user-images.githubusercontent.com/50604118/206438473-53ebbf0d-2180-45b9-965b-e617b48fec8a.png)

As you can see, there is a fence behind me. The fence is curved in the left image (before rectification) but in the right is now straight (rectified) after the rectification. The colorful lines are called the epipolar lines which, in short, must have the same vertical level in both left and right camera.. 
Consider the following figure, and compare the position of any random pixel in both left and right
images. For example, the tip of my nose in the left image lies on the same horizontal line
as the tip of my nose in the right image. You can also pick any other feature in the
left image and check the corresponding pixel in the right image, and they should lie on the
same horizontal line. This is what we mean by vertical alignment.. If, however, this is not the case, the stereo matching algorithm will
return a wrong match which may lead to a poor desparity and a poor depth estimation and hence wrong motion estimation results.

![stereo_matching](https://user-images.githubusercontent.com/50604118/206458031-08a0233f-bd7c-4ee9-b6e1-ff7eeedec259.png)


Results of the algorithm are outstanding specially in simulation using the KITTI dataset as shown in the following figure: 

![seq_kitti](https://user-images.githubusercontent.com/50604118/206524424-a0dbfcd3-60ab-4dba-8611-fe8740b7c82b.png)


As you can see those results are excellent especially that there is no motion optimization and the used KITTI sequences simulate a car motion of several kilometers! Just a final thought: you need to use the K matrix provided with the dataset and not the one resulting from your calibration.. pretty straightforward but just in case. 


Hope I have helped you understand the main methodology behind the algorithm and if you have any question, just shoot me an email on:  ma [dot] abdelhafez [ät] gmail [dot] com. 

