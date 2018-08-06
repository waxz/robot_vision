This code is an example of how to run OpenCV Code inside of a ROS node.

Basically, you create a standard ROS node and then just write some OpenCV
code inside of it and it'll do what you say.

1.ROS cv_bridge
https://github.com/ros-perception/vision_opencv

CMakeLists

    find_package(OpenCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})

You can also use OpenCV3: in that case, add a dependency to opencv3. But make sure that none of your dependencies depends on OpenCV2 (as you would get linked to both the OpenCVs which would most likely create a symbol conflict).
If you have OpenCV2 installed and the the ROS OpenCV3, OpenCV3 will be find_package-ed first. If you do not want to compile against OpenCV3 but still wish to have it installed, just find_package OpenCV2 as follows:

    find_package(OpenCV 2 REQUIRED)

install opencv3 above indigo: http://wiki.ros.org/opencv3




2. cv_bridge
http://wiki.ros.org/cv_bridge/Tutorials
http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
http://wiki.ros.org/image_transport/Tutorials/ExaminingImagePublisherSubscriber
http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages

cv_bridge::CvImagePtr cv_ptr
sensor_msgs::Image Image_msg
cv::Mat frame

Mat <--> CvImagePtr <-->Image

        cv_bridge::CvImage cv_image= cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame);
        msg = cv_image.toImageMsg();


        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
- path error; fix compile with vision_opencv
CMake Error at /opt/ros/kinetic/share/cv_bridge/cmake/cv_bridgeConfig.cmake:113 (message):
  Project 'cv_bridge' specifies '/opt/ros/kinetic/include/opencv-3.3.1-dev'
  as an include dir, which is not found.  It does neither exist as an
  absolute directory nor in
  '/opt/ros/kinetic//opt/ros/kinetic/include/opencv-3.3.1-dev'.


3. calibration
https://blog.csdn.net/heyijia0327/article/details/43538695
driver: https://github.com/ros-drivers/usb_cam.git
calibration: https://github.com/ros-perception/image_pipeline
    1.run usb camera
     roslaunch usb_cam usb_cam-test.launch
    2.看是否有如下消息，并记住名称

    /usb_cam/camera_info
    /usb_cam/image_raw

    3.To start the calibration you will need to load the image topics that will be calibrated:

    rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.05 image:=/usb_cam/image_raw camera:=/usb_cam

    4.move board wait progressbar reach goal
    click calibration, then save yaml file to

4. run camera driver
driver: https://github.com/ros-drivers/usb_cam.git

    roslaunch usb_cam usb_cam-test.launch

5.load calibration file using the camera_info_manager
http://wiki.ros.org/camera_info_manager

This package contains no ROS nodes or utility commands.
It provides a C++ class used by many camera drivers to manage the camera calibration data required by the ROS image pipeline.

For camera drivers written in Python, the camera_info_manager_py package provides a similar interface.

- usb_cam wiil load calibration file automatically

6.Rectifying an image
Simply loading a calibration file does not rectify the image. For rectification, use the image_proc package
http://wiki.ros.org/image_proc

ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
    rosrun image_view image_view image:=usb_cam/image_raw

using as nodelet


5.
https://github.com/ros-drivers/video_stream_opencv.git

6. line fitting and display in gui

7.webcam on iphone
https://github.com/shenyaocn/IP-Camera-Bridge.git
python test_video_resource.py http://admin:admin@192.168.0.120:8081/video
