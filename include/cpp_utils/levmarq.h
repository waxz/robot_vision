//
// Created by waxz on 18-7-21.
//

#ifndef LOCATE_REFLECTION_LEVMARQ_H
#define LOCATE_REFLECTION_LEVMARQ_H
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

namespace opt_util {
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
            double noise = random_ratio * drand48() / 50.0;
            point(1) = (x > x0) ? y0 + (x - x0) * k1 + noise : y0 + (x - x0) * k2 + noise;
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
}


#endif //LOCATE_REFLECTION_LEVMARQ_H
