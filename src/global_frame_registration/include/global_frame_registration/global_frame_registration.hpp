#include "ros/ros.h"
#include <aruco_msgs/MarkerArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <unordered_map>
#include <vector>

class GlobalFrameRegistrator
{
    struct Frame {
        std::string        frame_id;
        std::string        parent_frame;
        std::vector<float> position;
        std::vector<float> orientation;
    };

public:
    GlobalFrameRegistrator(ros::NodeHandle&);
    ~GlobalFrameRegistrator() = default;

    void initialize();
    void toolPoseUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string);
    void arrayPoseUpdate(const geometry_msgs::PoseArray::ConstPtr& msg, std::string);
    void cameraPoseUpdate(const aruco_msgs::MarkerArray::ConstPtr& msg, std::string);

private:
    void loadRegistration();
    void storeRegistration(std::unordered_map<std::string, tf::StampedTransform> calib);
    void broadcastTransform();
    void computeReferenceSystem(std::string, std::string, std::vector<geometry_msgs::PoseStamped>, geometry_msgs::Point);

    std::pair<tf::Vector3, tf::Vector3>    bestPlaneFromPoints(const std::vector<geometry_msgs::PoseStamped>&);
    std::pair<tf::Vector3, tf::Quaternion> bestPlaneFromPlanes(const std::vector<geometry_msgs::PoseStamped>&);

    ros::NodeHandle              nh;
    std::vector<ros::Subscriber> pose_subs;

    std::vector<std::string> tool_subs_topic, camera_subs_topic, rgbd_subs_topic;
    XmlRpc::XmlRpcValue      registration_data;
    std::string              global_frame_name, camera_color_frame, camera_ref_frame, camera_ref_base_frame;
    int                      num_points;
    bool                     use_existing, eye_in_hand;

    std::thread* t;
    std::mutex   mtx;

    std::unordered_map<std::string, geometry_msgs::PoseStamped>              tool_poses;
    std::unordered_map<std::string, std::vector<geometry_msgs::PoseStamped>> cam_poses;
    std::unordered_map<std::string, geometry_msgs::PoseArray>                rgbd_poses;
    std::unordered_map<std::string, tf::StampedTransform>                    calib_transform;
};