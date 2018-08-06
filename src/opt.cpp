//
// Created by waxz on 18-8-2.
//
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <unsupported/Eigen/NonLinearOptimization>

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;


#include <string>
#include <iostream>
#include <vector>
#include <cpp_utils/time.h>

using std::cout;
using std::endl;
using std::vector;

// generator
// given 3 parameter [x0,y0,yaw1], (x,y) as turn point, yaw as one side angle
// for x<x0, y = k1x+b, for x>x0 , y= k2x+b
Point2DVector SampleGen(double start, double end, int num) {
    double x0, y0, k1, k2, angle;
    // line function ax + by = c
    angle = 0.5 * M_PI;
    k1 = 0.3;
    k2 = -0.3;//-1;tan(atan(k1+angle));
    x0 = 0.0;
    y0 = 1.1;

    // sample range
    Point2DVector points;

    for (int cnt = 0; cnt < num; cnt++) {
        double random_ratio = 0.4;
        double x = static_cast<double>(start + cnt * (end - start) / num );
        Eigen::Vector2d point;
        point(0) = x;
        double noise = random_ratio * drand48() / 10.0;
        point(1) = (x > x0) ? y0 + (x - x0) * k1 : y0 + (x - x0) * k2;
        //2.0 * x + 5.0 + drand48() / 10.0;
        points.push_back(point);


    }


    return points;


}

struct LMFunctor {
    // 'm' pairs of (x, f(x))
    Eigen::MatrixXd measuredValues;

    // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.

        // 'fvec' has dimensions m x 1
        // It will contain the error for each data point.

        double x0 = x(0);
        double y0 = x(1);
        double k1 = x(2);

        double k2, angle;
        angle = 0.6 * M_PI;
        k2 = tan(atan(k1 + angle));

        for (int i = 0; i < values(); i++) {
            double xValue = measuredValues(i, 0);
            double yValue = measuredValues(i, 1);

            fvec(i) = yValue - ((xValue > x0) ? y0 + (xValue - x0) * k1 : y0 + (xValue - x0) * k2);
        }
        return 0;
    }

    // Compute the jacobian of the errors
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const {
        // 'x' has dimensions n x 1
        // It contains the current estimates for the parameters.

        // 'fjac' has dimensions m x n
        // It will contain the jacobian of the errors, calculated numerically in this case.

        double epsilon;
        epsilon = 1e-5f;

        for (int i = 0; i < x.size(); i++) {
            Eigen::VectorXd xPlus(x);
            xPlus(i) += epsilon;
            Eigen::VectorXd xMinus(x);
            xMinus(i) -= epsilon;

            Eigen::VectorXd fvecPlus(values());
            operator()(xPlus, fvecPlus);

            Eigen::VectorXd fvecMinus(values());
            operator()(xMinus, fvecMinus);

            Eigen::VectorXd fvecDiff(values());
            fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

            fjac.block(0, i, values(), 1) = fvecDiff;
        }

        return 0;
    }

    // Number of data points, i.e. values.
    int m;

    // Returns 'm', the number of values.
    int values() const { return m; }

    // The number of parameters, i.e. inputs.
    int n;

    // Returns 'n', the number of inputs.
    int inputs() const { return n; }

};

int main(int argc, char **argv) {
    time_util::Timer timer;
    timer.start();

    // 'm' is the number of data points.
    int m = 100;
    Point2DVector Points = SampleGen(-1, 1, m);

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
        printf("x,y = %d, [%.3f,%.3f]\n", i, x, y);

        int idxi = int(x * 100) + 0.5 * grid.cols;
        int idxj = int(y * 100) + 0.5 * grid.rows;
        printf("idxi,idxj = [%d,%d]\n", idxi, idxj);


#if 0
        int w = 4;
        for(int d =0 ;d<w;d++){

            for(int k=0;k<w;k++){
                grid.at<uchar>(idxi-d,idxj-k) = 255;
                grid.at<uchar>(idxi+d,idxj+k) = 255;
            }



        }
#endif
        grid.at<uchar>(idxj, idxi) = 0;

    }
    cv::Mat cdst;
    cv::Mat dst = grid.clone();

    //Apply blur to smooth edges and use adapative thresholding
    cv::Size size(3, 3);
    cv::GaussianBlur(dst, dst, size, 0);
    adaptiveThreshold(dst, dst, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 75, 10);


    cv::dilate(dst, dst, 0, cv::Point(-1, -1), 2, 1, 1);
    cv::bitwise_not(dst, dst);


//    Canny(grid, dst, 50, 200, 3);

    cvtColor(dst, cdst, CV_GRAY2BGR);

    vector<cv::Vec4i> lines;

    HoughLinesP(dst, lines, 1, CV_PI / 50, 50, 80, 1);

    int sz = int(lines.size());
    printf("get line = %d\n", sz);
#if 0
    cv::Vec4i l = lines[0];
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

    cv::imshow("detected lines", cdst);

#if 0
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