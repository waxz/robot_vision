//
// Created by waxz on 18-8-2.
//
#include <cpp_utils/levmarq.h>
#include <cpp_utils/time.h>
#include <cpp_utils/listener.h>
#include <cpp_utils/parse.h>
#include <cpp_utils/types.h>
#include <cpp_utils/container.h>
#include <cpp_utils/svdlinefitting.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


#include <boost/bind.hpp>
#include <vector>
#include <valarray>
using std::cout;
using std::endl;
using std::vector;
using std::valarray;

namespace sm=sensor_msgs;

void f(int x) {
    printf("xx = %d \n", x);
}



int main(int argc, char **argv) {
    time_util::Timer timer;


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
    auto lasersize = (*scan_data__).ranges.size();
    valarray<float> laserAngles(lasersize);
    float angle_min = (*scan_data__).angle_min;
    float incre = (*scan_data__).angle_increment;
    for(int i=0;i<lasersize;i++){
        laserAngles[i] = angle_min+i*incre;
    }

    valarray<float> laserXs = laserRanges*cos(laserAngles);
    valarray<float> laserYs = laserRanges*sin(laserAngles);

    // plot all points to mat
    valarray<float> dataXs;
    valarray<float> dataYs ;

    // plot



    // filter
    // range filter [range_min,range_max]
    float range_filter_min_ = 0.1;
    float range_filter_max_ = 2.0;

    valarray<bool> mask = laserRanges>range_filter_min_ && laserRanges<range_filter_max_;

    valarray<float> maskXs = laserXs[mask];
    valarray<float> maskYs = laserYs[mask];

    // plot





    int size = maskYs.size();

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
    int distMin_ = 4;
    vector<float> continiousPointsYs;

    vector<float> continiousPointsXs;
    double x,y;
    type_util::Point2d p;
    for(int i=0;i<edgeDist.size();i++){
        int startId , endId;
        bool getLine = false;
        if(i==0){
            if(edgeIds[i] >distMin_){
                getLine = true;
                startId = 0;
                endId = edgeIds[i];
                if(getLine){
                    for(int j = startId;j<endId;j++){

                        p.x =  maskXsL[j];
                        p.y = maskYsL[j];
                        continiousPointsXs.push_back(p.x);
                        continiousPointsYs.push_back(p.y);

                        continiousPoints.push_back(p);
                    }
                }
            }
        }
        if(i==edgeDist.size()-1){
            if(Ids.size() - edgeIds[edgeIds.size()-1] >distMin_){
                getLine = true;
                startId = edgeIds[i];
                endId = Ids.size();
                if(getLine){
                    for(int j = startId;j<endId;j++){

                        p.x =  maskXsL[j];
                        p.y = maskYsL[j];
                        continiousPointsXs.push_back(p.x);
                        continiousPointsYs.push_back(p.y);

                        continiousPoints.push_back(p);
                    }
                }
            }
        }



        if(edgeDist[i]>distMin_){
            getLine= true;
            startId = edgeIds[i]+1;
            endId = edgeIds[i+1];
            if(getLine){
                for(int j = startId;j<endId;j++){

                    p.x =  maskXsL[j];
                    p.y = maskYsL[j];
                    continiousPointsXs.push_back(p.x);
                    continiousPointsYs.push_back(p.y);

                    continiousPoints.push_back(p);
                }
            }

        }


    }

    // find xmax,xmin,ymax,ymin
    auto continiousPointsXs_tmp = container_util::createValarrayFromVector(continiousPointsXs);
    auto continiousPointsYs_tmp = container_util::createValarrayFromVector(continiousPointsYs);


    // plot
    // ============================================
    dataXs = continiousPointsXs_tmp;

    dataYs = continiousPointsYs_tmp;

    double normXmin, normYmin, normXmax,normYmax;
    normXmin = dataXs.min();
    normXmax = dataXs.max();
    normYmin = dataYs.min();
    normYmax = dataYs.max();
    double normXlen = normXmax - normXmin;
    double normYlen = normYmax - normYmin;
    int resolution_ = 100;
    int height= int(resolution_*(normXmax - normXmin));
    int width = int(resolution_*(normYmax - normYmin));
    cv::Mat grid(1*height, 1*width, CV_8U,255);
    printf("w h = %d,%d\n",width,height);

    int idx ,idy;
    for(int i=0;i<dataYs.size();i++){

        idx = int(resolution_*(dataXs[i] - normXmin));
        idy = int(resolution_*(dataYs[i] - normYmin));

        printf("id %d,%d\n",idx,idy);
        grid.at<uchar>(idy ,idx) = 0;


    }


#if 0
    cv::GaussianBlur(grid, grid, cv::Size(3, 3), 5);


    adaptiveThreshold(grid, grid, 255, CV_ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, 10);
    cv::imshow("dataXs dataYs",grid);
#endif
    // ============================================




#endif


#if 0


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
#endif
#if 0
    timer.start();

    printf("point to mat time %f ms\n", timer.elapsedMicroseconds() / 1000.0);


    cv::Mat cdst;
    cv::Mat dst = grid.clone();

    //Apply blur to smooth edges and use adapative thresholding
    cv::GaussianBlur(dst, dst,  cv::Size(3,3), 9);


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


    auto detectMat  = dst;
    HoughLinesP(detectMat, lines, 1, CV_PI / 100, 10, 12, 1);
    //    HoughLinesP(dst, lines, 1, CV_PI / 100, 50, 80, 1);


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

    cv::waitKey();
#endif


#if 1
    // 'm' is the number of data points.
    int m = 100;
    vector<type_util::Point2d> LinePoints;
    Point2DVector Points = opt_util::SampleGen(-1, 1, m);
    // Move the data into an Eigen Matrix.
    // The first column has the input values, x. The second column is the f(x) values.
    Eigen::MatrixXd measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = Points[i](0);
        measuredValues(i, 1) = Points[i](1);
        if (i < 0.5 * m)
            LinePoints.push_back(type_util::Point2d(Points[i](0), Points[i](1)));
    }

    Eigen::VectorXd x(3);
    x(0) = 0.0;             // initial value for 'a'
    x(1) = 1.0;             // initial value for 'b'
    x(2) = 0.5;             // initial value for 'c'


    opt_util::SimpleSolver<opt_util::LineFunctor> sm;
    decltype(x) model(1);
    model(0) = 0.5 * M_PI;
    sm.updataModel(model);
    sm.setParams(x);
    sm.feedData(measuredValues);
    timer.start();
    int status = sm.solve();
    x = sm.getParam();



    std::cout << "LM optimization status: " << status << std::endl;

    std::cout << "Optimization results" << std::endl;
    std::cout << "\tx0: " << x(0) << std::endl;
    std::cout << "\ty0: " << x(1) << std::endl;
    std::cout << "\tk1: " << x(2) << std::endl;
    printf("opt time = %f ms\n", timer.elapsedMicroseconds() / 1000);
    double a, b, c;
    fit_util::svdfit(LinePoints, a, b, c);
    printf("svd time = %f ms\n", timer.elapsedMicroseconds() / 1000);

    printf("get line ");
#endif
    return 0;
}