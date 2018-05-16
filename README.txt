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

cv_bridge::CvImagePtr cv_ptr
sensor_msgs::Image Image_msg
cv::Mat frame

Mat <--> CvImagePtr <-->Image

        cv_bridge::CvImage cv_image= cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame);
        msg = cv_image.toImageMsg();


        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
