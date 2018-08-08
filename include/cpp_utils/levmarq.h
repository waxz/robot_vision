//
// Created by waxz on 18-7-21.
//

#ifndef LOCATE_REFLECTION_LEVMARQ_H
#define LOCATE_REFLECTION_LEVMARQ_H
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "types.h"
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
    Point2DVector SampleGen(double start, double end, int num, double random_ratio = 0.4) {
        double x0, y0, k0, k1, k2, angle;
        // line function ax + by = c
        angle = 0.5 * M_PI;
        k0 = 0.3;
        k1 = tan(atan(k0) + 0.5 * angle);
        k2 = tan(atan(k0) - 0.5 * angle);

        x0 = 0.0;
        y0 = 1.1;

        // sample range
        Point2DVector points;

        for (int cnt = 0; cnt < num; cnt++) {
            double x = static_cast<double>(start + cnt * (end - start) / num );
            Eigen::Vector2d point;
            point(0) = x;
            double noise = random_ratio * drand48() / 100.0;
            point(1) = (x > x0) ? y0 + (x - x0) * k1 + noise : y0 + (x - x0) * k2 + noise;
            //2.0 * x + 5.0 + drand48() / 10.0;
            points.push_back(point);


        }


        return points;


    }


    struct SimpleFunctor {

        // Compute 'm' errors, one for each data point, for the given parameter values in 'x'
        virtual int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const = 0;

        virtual void updataModel(const Eigen::VectorXd &model) = 0;
        // 'm' pairs of (x, f(x))
        Eigen::MatrixXd measuredValues;



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

        void feedData(Eigen::MatrixXd &data) {
            measuredValues = data;
            m = static_cast<int>(data.rows());

        }

    };

    template<class T>
    struct SimpleSolver {
        T functor_;
        Eigen::VectorXd x_;

        void setParams(const Eigen::VectorXd &x) {
            // 'x' is vector of length 'n' containing the initial values for the parameters.
            // The parameters 'x' are also referred to as the 'inputs' in the context of LM optimization.
            // The LM optimization inputs should not be confused with the x input values.
            x_ = x;
            functor_.n = x.rows();
        }

        void feedData(Eigen::MatrixXd &data) {
            functor_.feedData(data);

        }

        void updataModel(const Eigen::VectorXd &model) {
            functor_.updataModel(model);
        }

        int solve() {
            // Run the LM optimization
            // Create a LevenbergMarquardt object and pass it the functor_.
            Eigen::LevenbergMarquardt<T, double> lm(functor_);
            int status = lm.minimize(x_);
            printf("LM optimization status: %d\n", status);
            return status;
        }

        Eigen::VectorXd getParam() {
            return x_;
        }

    };

    struct LineFunctor : SimpleFunctor {
        // define model param
        double paramAngle_;

        LineFunctor() {
            paramAngle_ = 0.0;
        }

        //update model param
        void updataModel(const Eigen::VectorXd &model) {
            paramAngle_ = model(0);
        }

        // predict
        int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
            // 'x' has dimensions n x 1
            // It contains the current estimates for the parameters.

            // 'fvec' has dimensions m x 1
            // It will contain the error for each data point.

            double x0 = x(0);
            double y0 = x(1);
            double k0 = x(2);
            double k1, k2;

            k1 = tan(atan(k0) + 0.5 * paramAngle_);
            k2 = tan(atan(k0) - 0.5 * paramAngle_);


            for (int i = 0; i < values(); i++) {
                double xValue = measuredValues(i, 0);
                double yValue = measuredValues(i, 1);

                // error = y_true - y_predict
                fvec(i) = yValue - ((xValue > x0) ? y0 + (xValue - x0) * k1 : y0 + (xValue - x0) * k2);
            }
            return 0;
        }


    };


}

#if 0
Eigen::MatrixXd measuredValues(m, 2);
    for (int i = 0; i < m; i++) {
        measuredValues(i, 0) = Points[i](0);
        measuredValues(i, 1) = Points[i](1);
    }

    Eigen::VectorXd x(3);
    x(0) = 0.0;             // initial value for 'a'
    x(1) = 1.0;             // initial value for 'b'
    x(2) = 0.5;             // initial value for 'c'

    opt_util::SimpleSolver<opt_util::LineFunctor> sm;
    sm.functor_.updataModel(0.5*M_PI);
    sm.setParams(x);
    sm.feedData(measuredValues);
    int status = sm.solve();
    x = sm.getParam();



#endif

#endif //LOCATE_REFLECTION_LEVMARQ_H
