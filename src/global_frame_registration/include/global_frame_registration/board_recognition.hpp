#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <typeinfo>

class BoardCalibration
{

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber image_sub_, image_depth_sub_, camera_info_sub_;
    ros::Publisher  pose_array_pub;

    image_transport::Publisher pub_rgb, pub_mask;
    std::string depth_topic, cam_info_topic, image_topic, camera_color_frame, camera_tof_frame, pose_array_topic;

    cv::Mat image_rgb, image_depth;

    sensor_msgs::CameraInfo cam_info;
    bool                    tof_3d;
    int                     radius;

    cv::Ptr<cv::MSER> ms;

public:
    BoardCalibration(ros::NodeHandle nh);

    ~BoardCalibration();

    void camInfoCb(const sensor_msgs::CameraInfoPtr& msg);

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void imageDepthCb(const sensor_msgs::ImageConstPtr& msg);

    std_msgs::Header makeHeader(std::string id);

    void PublishRenderedImage(image_transport::Publisher& pubImage,
                              cv::Mat&                    image,
                              std::string                 encoding,
                              std_msgs::Header            header);

    void findCircles(cv::Mat& img, int minArea, std::vector<cv::Point2f>& corners);

    void poseFromImage(cv::Mat                   image_depth,
                       std::vector<cv::Point2f>  corners,
                       float*                    K,
                       geometry_msgs::PoseArray& pose_array);

    void poseFromImageToF(cv::Mat                   image_depth,
                          std::vector<cv::Point2f>  corners,
                          float*                    K,
                          geometry_msgs::PoseArray& pose_array);

    void computeVersor(geometry_msgs::PoseArray& pose_array);

    void getMatDepthValue(cv::Mat& src, cv::Mat& dst);

    cv::Mat hsv_blue_mask(cv::Mat color);

    cv::Mat hsv_black_mask(cv::Mat color);

    void morphOperation(cv::Mat img, cv::Mat& morphMask);

    void filterContour(cv::Mat& img, cv::Mat &mask);

    void detectRegions(cv::Mat& img, cv::Rect& roi);

    void update();
};