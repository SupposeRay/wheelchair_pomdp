#ifndef PLOT_SCAN_HPP
#define PLOT_SCAN_HPP

#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace plot_scan
{

class PlotScan
{
    public:
        PlotScan();
        ~PlotScan();

        /**
        * @brief Draw laser scan data onto an image
        * @param scan_msg ROS LaserScan msg
        * @param img output image
        */
        void drawScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg, cv::Mat &img);

        /**
        * @brief Set laser scan color
        * @param b blue value 0-255
        * @param g green value 0-255
        * @param r red value 0-255
        */
        void setColor(int b, int g, int r);

        /**
        * @brief Set image size
        * @param w width - pixels
        * @param h height - pixels
        */
        void setImgSize(int w, int h);

        /**
        * @brief Set focal length of the projected view
        * @param f focal length - pixels
        */
        void setFocal(double f);

        /**
        * @brief Option to draw X-Y coordinate on image
        * @param draw set true to draw grid
        */
        void setGrid(bool draw);

        /**
        * @brief Set view height
        * @param z view height - meters
        */
        void setHeight(double z);

    private:
        bool draw_grid_ = true;
        int img_w_ = 720;
        int img_h_ = 720;
        double focal_ = 450;
        double z_ = 3;
        cv::Vec3b bg_color_ = cv::Vec3b(0,0,0);
        cv::Vec3b color_ = cv::Vec3b(0,0,255);
        cv::Mat imgView_;

        /**
        * @brief Initializa image canvas
        */
        void initViewer();

        /**
        * @brief Convert laser scan msg into vector of 3D points
        * @param scan ROS LaserScan msg
        * @param points output vector
        */
        void cvtScanToPoints(const sensor_msgs::LaserScan::ConstPtr& scan, std::vector<Eigen::Vector3d>& points);

        /**
        * @brief Plot a single point onto the image
        * @param pt point coordinate
        * @param color point color
        */
        void plotViewer(Eigen::Vector3d pt, cv::Vec3b color);
};

inline PlotScan::PlotScan()
{
}

inline PlotScan::~PlotScan()
{
}

inline void PlotScan::drawScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg, cv::Mat &img)
{
    initViewer();

    std::vector<Eigen::Vector3d> scan_points;
    cvtScanToPoints(scan_msg, scan_points);

    for (auto pt : scan_points) 
        plotViewer(pt, color_);

    img = imgView_.clone();
}

inline void PlotScan::setColor(int b, int g, int r)
{
    color_ = cv::Vec3b(b, g, r);
}

inline void PlotScan::setImgSize(int w, int h)
{
    img_w_ = w;
    img_h_ = h;
}

inline void PlotScan::setFocal(double f)
{
    focal_ = f;
}

inline void PlotScan::setGrid(bool draw)
{
    draw_grid_ = draw;
}

inline void PlotScan::setHeight(double z)
{
    z_ = z;
}

inline void PlotScan::initViewer()
{
    imgView_ = cv::Mat(img_h_, img_w_, CV_8UC3, bg_color_);

    Eigen::Vector3d pt(0,0,0);
    int col = (int)(pt.x() / z_ * focal_ + img_w_/2);
    int row = (int)(- pt.y() / z_ * focal_ + img_h_/2);

    if (draw_grid_)
    {
        cv::circle(imgView_, cv::Point(col,row), 3, cv::Scalar(0,255,255), -1);
        cv::line(imgView_, cv::Point(col,row), cv::Point(img_w_,row), cv::Scalar(0,50,255));
        cv::line(imgView_, cv::Point(col,row), cv::Point(col,0), cv::Scalar(0,255,50));

        cv::line(imgView_, cv::Point(col,row), cv::Point(0,row), cv::Scalar(0,0,100));
        cv::line(imgView_, cv::Point(col,row), cv::Point(col,img_h_), cv::Scalar(0,100,0));
    }

}

inline void PlotScan::cvtScanToPoints(const sensor_msgs::LaserScan::ConstPtr& scan, std::vector<Eigen::Vector3d>& points)
{
    size_t n_pts = scan->ranges.size();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);
    Eigen::ArrayXXd cos_sin_map (n_pts, 2);

    for (size_t i = 0; i < n_pts; ++i)
    {
        ranges(i, 0) = (double)scan->ranges[i];
        ranges(i, 1) = (double)scan->ranges[i];

        cos_sin_map(i, 0) = cos (scan->angle_min + (double) i * scan->angle_increment);
        cos_sin_map(i, 1) = sin (scan->angle_min + (double) i * scan->angle_increment);
    }

    output = ranges * cos_sin_map;

    for (size_t i = 0; i < n_pts; ++i) 
    {
        double range_cutoff = 30.0;
        const float range = scan->ranges[i];
        if (range < range_cutoff && range >= scan->range_min)
        {
            Eigen::Vector3d pt = Eigen::Vector3d(output(i,0), output(i,1), 0);
            points.push_back(pt);
        } else
        {
            points.push_back(Eigen::Vector3d(1000.0,1000.0, 0) );
        }
    }
}

inline void PlotScan::plotViewer(Eigen::Vector3d pt, cv::Vec3b color)
{
    Eigen::Vector3d pt_ = pt;
    int col = (int)(pt_.x() / z_ * focal_ + img_w_/2);
    int row = (int)(-pt_.y() / z_ * focal_ + img_h_/2);
    if(col > img_w_-1 || col< 0 || row > img_h_-1 || row < 0)
        return;

    imgView_.at<cv::Vec3b>(row, col) = color;
}

} // namespace plot_scan


#endif // PLOT_SCAN_HPP