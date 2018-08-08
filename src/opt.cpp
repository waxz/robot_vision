//
// Created by waxz on 18-8-2.
//
#include <cpp_utils/levmarq.h>
#include <cpp_utils/time.h>
#include <cpp_utils/listener.h>
#include <cpp_utils/parse.h>
#include <cpp_utils/types.h>
#include <cpp_utils/container.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#include <vector>
#include <valarray>
using std::cout;
using std::endl;
using std::vector;
using std::valarray;

namespace sm=sensor_msgs;
int main(int argc, char **argv) {


#if 0
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::NodeHandle nh_pravite("~");
    rosnode::Listener l(nh,nh_pravite);
    std::shared_ptr<sm::LaserScan> scan_data__;
    string scan_topic_ = "/scan";
    auto res = l.createSubcriber<sm::LaserScan>(scan_topic_,1);

    scan_data__ = std::get<0>(res);

    l.getOneMessage(scan_topic_, -1);

    ROS_INFO_STREAM("ranges"<<(*scan_data__).header);

    // laser to mat
    // first remove independent points
    auto laserRanges = container_util::createValarrayFromVector((*scan_data__).ranges);
    auto size = (*scan_data__).ranges.size();
    valarray<float> laserAngles(size);
    float angle_min = (*scan_data__).angle_min;
    float incre = (*scan_data__).angle_increment;
    for(int i=0;i<size;i++){
        laserAngles[i] = angle_min+i*incre;
    }

    valarray<float> laserXs = laserRanges*cos(laserAngles);
    valarray<float> laserYs = laserRanges*sin(laserAngles);



    // filter
    // range filter [range_min,range_max]
    float range_filter_min_ = 0.1;
    float range_filter_max_ = 2.0;

    valarray<bool> mask = laserRanges>range_filter_min_ && laserRanges<range_filter_max_;

    valarray<float> maskXs = laserXs[mask];
    valarray<float> maskYs = laserYs[mask];
    size = maskYs.size();

    valarray<float> maskXsL = maskXs[std::slice(0, size - 1, 1)];
    valarray<float> maskXsR = maskXs[std::slice(1, size - 1, 1)];

    valarray<float> maskYsL = maskYs[std::slice(0, size - 1, 1)];
    valarray<float> maskYsR = maskYs[std::slice(1, size - 1, 1)];

    // get distance sequence
    valarray<float> distance = sqrt(pow(maskXsR - maskXsL, 2) + pow(maskYsR - maskYsL, 2));

    float edgeMin_ = 0.05;

    mask = distance>edgeMin_;
    auto Ids = container_util::createRangeValarray(distance.size(),0);
    valarray<int> edgeIds = Ids[mask];
//    valarray<float> edgeXs = maskXsL[mask];
//    valarray<float> edgeYs = maskYsL[mask];
    size = edgeIds.size();
    valarray<int> edgeIdL = edgeIds[std::slice(0, size - 1, 1)];
    valarray<int> edgeIdR = edgeIds[std::slice(1, size - 1, 1)];
    auto edgeDist = edgeIdR - edgeIdL;

    // push all point to a vector
    vector<type_util::Point2d> continiousPoints;
    int distMin_ = 5;
    vector<float> continiousPointsYs;

    vector<float> continiousPointsXs;
    double x,y;
    type_util::Point2d p;
    for(int i=0;i<edgeDist.size();i++){
        if(edgeDist[i]>distMin_){
            for(int j = edgeIds[i]+1;j<edgeIds[i+1];j++){

                p.x =  maskXsL[j];
                p.y = maskYsL[j];
                continiousPointsXs.push_back(p.x);
                continiousPointsYs.push_back(p.y);

                continiousPoints.push_back(p);
            }
        }
    }

    // find xmax,xmin,ymax,ymin
    auto continiousPointsXs_tmp = container_util::createValarrayFromVector(continiousPointsXs);
    auto continiousPointsYs_tmp = container_util::createValarrayFromVector(continiousPointsYs);
    double normXmin, normYmin, normXmax,normYmax;
    normXmin = continiousPointsXs_tmp.min();
    normXmax = continiousPointsXs_tmp.max();
    normYmin = continiousPointsYs_tmp.min();
    normYmax = continiousPointsYs_tmp.max();
    double normXlen = normXmax - normXmin;
    double normYlen = normYmax - normYmin;


    int resolution_ = 100;
    int width = int(100*(normXmax - normXmin));
    int height = int(100*(normYmax - normYmin));


    cv::Mat grid(height, width, CV_8U,255);

    size = continiousPoints.size();
    int idx ,idy;
    for(int i=0;i<size;i++){

        idx = std::max(0,int((continiousPoints[i].x-normXmin )*(width-1)/normXlen));
        idy = std::max(0,int((continiousPoints[i].y-normYmin )*(height-1)/normYlen));

        grid.at<uchar>(idx, idy) = 0;


    }
    // convert point to Mat


    cv::imshow("source", grid);

#endif





    // finally get [startId,endId] vector


    //push all points to SelectPoints

    // convert selectPoints to Mat







    // window filter



    // select region
    // convert to Mat
    // normalise range




#if 1
    time_util::Timer timer;
    timer.start();

    // 'm' is the number of data points.
    int m = 100;
    Point2DVector Points = opt_util::SampleGen(-1, 1, m);

    // laser scan to grid

    // apply hough
    // 0.01 cm
    //2m*2m --> 200*200
    cv::Mat grid(300, 300, CV_8U);
    grid = 255;
    for (int i = 0; i < 1 * Points.size(); i++) {
        // convert points to index
        double x = Points[i](0);
        double y = Points[i](1);
//        printf("x,y = %d, [%.3f,%.3f]\n", i, x, y);

        int idxi = int(x * 100) + 0.5 * grid.cols;
        int idxj = int(y * 100) + 0.5 * grid.rows;
//        printf("idxi,idxj = [%d,%d]\n", idxi, idxj);



        grid.at<uchar>(idxj, idxi) = 0;

    }
    printf("point to mat time %f ms\n", timer.elapsedMicroseconds() / 1000.0);


    cv::Mat cdst;
    cv::Mat dst = grid.clone();

    //Apply blur to smooth edges and use adapative thresholding
    cv::Size size(3, 3);
    cv::GaussianBlur(dst, dst, size, 5);


    adaptiveThreshold(dst, dst, 255, CV_ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, 10);
    printf("adaptiveThreshold time %f ms\n", timer.elapsedMicroseconds() / 1000.0);

    cv::bitwise_not(dst, dst);
    printf("bitwise_not time %f ms\n", timer.elapsedMicroseconds() / 1000.0);

    auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));


    decltype(dst) dilateMat;
    cv::dilate(dst, dilateMat, kernel, cv::Point(-1, -1), 1);
    printf("dilate time %f ms\n", timer.elapsedMicroseconds() / 1000.0);


    decltype(dst) erodeMat;

    auto kernel2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::erode(dilateMat, erodeMat, kernel2, cv::Point(-1, -1), 1);
    printf("erode time %f ms\n", timer.elapsedMicroseconds() / 1000.0);


    cvtColor(erodeMat, cdst, CV_GRAY2BGR);

    vector<cv::Vec4i> lines;


    HoughLinesP(dst, lines, 1, CV_PI / 100, 50, 80, 1);

    int sz = int(lines.size());
    printf("get line = %d\n", sz);
#if 0
    cv::Vec4i l = lines[1];
    cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
#endif
#if 1
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, CV_AA);
    }
#endif

    timer.stop();
    printf("time %f ms\n", timer.elapsedMicroseconds() / 1000.0);
    cout << endl;
    cv::imshow("source", grid);
    cv::imshow("dst", dst);

    cv::imshow("dilateMat", dilateMat);
    cv::imshow("erodeMat", erodeMat);

    cv::imshow("detected lines", cdst);


#endif


    cv::waitKey();


#if 0
    // Move the data into an Eigen Matrix.
    // The first column has the input values, x. The second column is the f(x) values.
    Eigen::MatrixXd measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = Points[i](0);
        measuredValues(i, 1) = Points[i](1);
    }


// 'n' is the number of parameters in the function.
    // f(x) = a(x^2) + b(x) + c has 3 parameters: a, b, c
    int n = 3;

    // 'x' is vector of length 'n' containing the initial values for the parameters.
    // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
    // The LM optimization inputs should not be confused with the x input values.
    Eigen::VectorXd x(n);
    x(0) = 0.0;             // initial value for 'a'
    x(1) = 1.0;             // initial value for 'b'
    x(2) = 0.5;             // initial value for 'c'



    // Run the LM optimization
    // Create a LevenbergMarquardt object and pass it the functor.
    //

    LMFunctor functor;
    functor.measuredValues = measuredValues;
    functor.m = m;
    functor.n = n;

    Eigen::LevenbergMarquardt<LMFunctor, double> lm(functor);
    int status = lm.minimize(x);
    std::cout << "LM optimization status: " << status << std::endl;

    //
    // Results
    // The 'x' vector also contains the results of the optimization.
    //
    std::cout << "Optimization results" << std::endl;
    std::cout << "\tx0: " << x(0) << std::endl;
    std::cout << "\ty0: " << x(1) << std::endl;
    std::cout << "\tk1: " << x(2) << std::endl;
#endif
    return 0;
}