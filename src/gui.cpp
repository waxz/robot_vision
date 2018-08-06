//
// Created by waxz on 18-8-2.
//

// use cvui
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CVUI_IMPLEMENTATION

#include <cpp_utils/cvui.h>

#include <string>
#include <vector>
#include <valarray>

#define WINDOW_NAME "Sparkline"

using std::vector;
using std::string;
using std::valarray;
enum Pen {
    //point
            O,
    // o
            o,
    // line
            line,
    // triangle
            triangle,
    // rectangle
            rectangle,
    // X
            X

};

// a gui wrapper
class CvPlot {
private:
    string name_;
    cv::Mat frame_;

    cv::Mat getPen(Pen pen);

    int width_;
    int height_;
    int row_;
    int col_;
    int margin_x_;
    int margin_y_;
    int penWidth_;

public:
    CvPlot(string name, int width, int height, int row = 1, int col = 1);

    // plot 1d array , given 1d data, marker type
    void plot(vector<double> points1d, int pltId = 0);

    // plot 2d array
    void plot(vector<cv::Point2d> point2d, int pltId = 0);

    //
    cv::Mat getFrame();

    //
    void show();


};

cv::Mat CvPlot::getFrame() {
    return frame_;
}

void CvPlot::show() {
    cv::imshow(name_, frame_);
}

CvPlot::CvPlot(string name, int width, int height, int row, int col) {
    // normalise width and height
    height_ = int(height / row) * row;
    width_ = int(width / col) * col;

    // ctrate frame
    frame_ = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));
    row_ = row;
    col_ = col;
    name_ = name;
    margin_x_ = 25;
    margin_y_ = 25;

    penWidth_ = 1;

    cvui::init(name_);


}


cv::Mat CvPlot::getPen(Pen pen) {

    cv::Mat penMat(penWidth_, penWidth_, CV_8UC3);
    switch (pen) {
        case Pen::O :
            break;
        case Pen::o :
            break;
        case Pen::rectangle :
            break;
        case Pen::triangle :
            break;
        case Pen::X :
            break;
        case line:
            break;
    }
    penMat = cv::Scalar(0, 55, 66);
    return penMat;
}

void CvPlot::plot(vector<double> points1d, int pltId) {
    cv::Mat fig = cv::Mat(height_ / row_, width_ / col_, CV_8UC3, cv::Scalar(255, 255, 255));

    auto num = points1d.size();

    valarray<double> points1d_(&(points1d[0]), num);
    // normalise y

    points1d_ = (fig.rows - 2 * margin_y_) * (points1d_ - points1d_.min()) / (points1d_.max() - points1d_.min()) +
                margin_y_;

    // select one region
    // local point to global point
    auto penMat = getPen(Pen::X);

    cv::Rect roi;
    vector<cv::Rect> roiVec;
    vector<cv::Point2d> pointVec;
    roi.width = penMat.cols;
    roi.height = penMat.rows;
    cv::Mat marker;
    // show position
    double mouseDist = 1000;
    int showId;
    double mouseX = cvui::mouse().x, mouseY = cvui::mouse().y;

    //plot with line
    // plot with marker

    for (decltype(num) i = 0; i < num; i++) {
        roi.x = int(margin_x_ + (fig.cols - 2 * margin_x_) * i / (num - 1) -
                    0.5 * penWidth_);//int((0.5*(1-max_ration) + max_ration*i/num)*width_/col_);
        roi.y = int(fig.rows - points1d_[i] - 0.5 * penWidth_);
        marker = fig(roi);
#if 0
        // marker
        penMat.copyTo(marker);
#endif
        roiVec.push_back(roi);
        cv::Point2d point = cv::Point2d(int(margin_x_ + (fig.cols - 2 * margin_x_) * i / (num - 1)),
                                        int(fig.rows - points1d_[i]));
        pointVec.push_back(point);
//        cvui::printf(fig,roi.x,roi.y,0.3,0xff00ff,"(%d,%.1f)",i,points1d[i]);

        double dist = sqrt(pow(roi.x - mouseX, 2) + pow(roi.y - mouseY, 2));
        if (dist < mouseDist) {
            showId = i;
            mouseDist = dist;
        }

        // plot circle
        cv::circle(fig, point, 3, cv::Scalar(00, 244, 0), 3);
        if (i > 0) {
            cv::line(fig, pointVec[i - 1], point, cv::Scalar(234, 244, 0), 1);
        }

    }
//    cvui::printf(fig,int(0.2*roiVec[showId].x + 0.8*mouseX) ,int(0.2*roiVec[showId].y + 0.8*mouseY),0.3,0xff00ff,"(%d,%.3f)",showId,points1d[showId]);
    penMat = cv::Scalar(0, 0, 233);
    marker = fig(roiVec[showId]);
    penMat.copyTo(marker);


    // copy fig to fram
    roi.x = fig.cols * (pltId % col_);
    roi.y = fig.rows * int(pltId / col_);
    roi.width = fig.cols;
    roi.height = fig.rows;
    cv::Mat fig_ = frame_(roi);
    fig.copyTo(fig_);

    // plot axis

}

void CvPlot::plot(vector<cv::Point2d> point2d, int pltId) {

}

int main(int argc, char **argv) {
    printf("hello !!");

    vector<double> sample{1, 2, 3, 4};
    CvPlot plt("test", 300, 300, 2);
    int i = 0;
    while (1) {
        auto frame = plt.getFrame();
        frame = cv::Scalar(255, 255, 255);
        cvui::printf(frame, cvui::mouse().x, cvui::mouse().y, 0.6, 0xff00ff, "mouse at (%d,%d)", cvui::mouse().x,
                     cvui::mouse().y);

        plt.plot(sample, 0);
        plt.plot(sample, 1);

        plt.show();
        cvui::update();


        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27) {
            break;
        }
    }


#if 0
    cv::Mat frame = cv::Mat(600, 1000, CV_8UC3);


    // Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
    cvui::init(WINDOW_NAME);
    cv::Point anchor;
    cv::Rect roi(0, 0, 0, 0);

    while (1){
        frame = cv::Scalar(255, 255, 255);
        cvui::printf(frame,cvui::mouse().x,cvui::mouse().y,0.6,0xff00ff,"mouse at (%d,%d)",cvui::mouse().x,cvui::mouse().y);

        cv::Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.width = cvui::mouse().x;
        rect.height = cvui::mouse().y;
        int w = cvui::mouse().x, h = cvui::mouse().y;


        cv::Mat m = cv::Mat(h, w, CV_8UC3);
//        m.setTo(cv::Scalar(44, 55, 66));
        m = cv::Scalar(44, 55, 66);

        cv::Mat roi_mat = frame(rect);

//        cv::Mat roi_mat(frame(rect));
        m.copyTo(roi_mat);
        // all the behind the scenes magic to handle mouse clicks, etc.
        cvui::update();

        // Show everything on the screen
        cv::imshow(WINDOW_NAME, frame);

        // Check if ESC key was pressed
        if (cv::waitKey(20) == 27) {
            break;
        }
    }


#endif
    return 0;
}
