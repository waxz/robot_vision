#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


static const std::string OPENCV_WINDOW = "Image window";

class OpenCVWebCam {
private:
    ros::NodeHandle nh_;

    // image subscriber, enable to receive compresess image format
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher image_pub2_;

    void imageCb(const sensor_msgs::Image::ConstPtr &msg);

    bool matToimage(cv::Mat &frame, sensor_msgs::ImagePtr &msg);

    bool imageTomat(const sensor_msgs::ImageConstPtr &image, cv_bridge::CvImagePtr &cv_ptr);

public:
    OpenCVWebCam();

    ~OpenCVWebCam();

    void start_record();
};

OpenCVWebCam::OpenCVWebCam() : it_(nh_) {

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
                               &OpenCVWebCam::imageCb, this);
    image_pub_ = it_.advertise("/camera/image_raw", 1);
    image_pub2_ = it_.advertise("/camera/image", 1);

}

OpenCVWebCam::~OpenCVWebCam() {
    cv::destroyWindow(OPENCV_WINDOW);
}


// convert cv2:mat to sensor::Image
bool OpenCVWebCam::matToimage(cv::Mat &frame, sensor_msgs::ImagePtr &msg) {
    try {
        // convert mat to sensor::Image
        cv_bridge::CvImage cv_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame);
        msg = cv_image.toImageMsg();

    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    return true;

}

// convert sensor::Image to cv2:mat
bool OpenCVWebCam::imageTomat(const sensor_msgs::ImageConstPtr &image, cv_bridge::CvImagePtr &cv_ptr) {
    try {
        // convert sensor::Image to mat
        // copy mode, youcan modify  new mat data
        // if share mode , you can't modify data
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    return true;


}


// start record frame form camera
void OpenCVWebCam::start_record() {
    cv::VideoCapture cap(CV_CAP_ANY); // open any camera
    //In this case you can hard-code the index/address of the device and directly pass it to the video capturing structure in OpenCV
    // (example: cv::VideoCapture(0) if /dev/video0 is used).
    if (!cap.isOpened()) {
        std::cout << "Could not open camera\n";
    }

    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::startWindowThread();

    while (1) {
        cv::Mat frame;
        if (cap.read(frame)) {
            cv::imshow("Webcam", frame);

            // convert mat to image msg
            sensor_msgs::ImagePtr msg;
            if (matToimage(frame, msg))
                image_pub_.publish(msg);
        }
        // process callback
        ros::spinOnce();
        if (cv::waitKey(30) == 27) {
            // if "esc" is pressed end the program
            std::cout << "Closing the program because esc pressed";
            break;
        }
    }
}


void OpenCVWebCam::imageCb(const sensor_msgs::Image::ConstPtr &msg) {
    ROS_INFO("get image");
    cv_bridge::CvImagePtr cv_ptr;

    try {
        imageTomat(msg, cv_ptr);
        cv::imshow("view", cv_ptr->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }


    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    image_pub2_.publish(cv_ptr->toImageMsg());

}

int main(int argc, char** argv) {
	// set up ros
	ros::init(argc, argv, "opencv_test");
	OpenCVWebCam webcam;
    webcam.start_record();
	ROS_INFO("Webcam Tested");
	return 0;
}