#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "instrument_pose_publisher");
    ros::NodeHandle private_nh("~"), public_nh;

    geometry_msgs::PoseStamped pose_msg;
    if (!private_nh.getParam("header/frame_id", pose_msg.header.frame_id)) {
        ROS_ERROR_STREAM("Unable to get param header/frame_id");
    }
    if (!private_nh.getParam("pose/position/x", pose_msg.pose.position.x)) {
        ROS_ERROR_STREAM("Unable to get param pose/position/x");
    }
    if (!private_nh.getParam("pose/position/y", pose_msg.pose.position.y)) {
        ROS_ERROR_STREAM("Unable to get param pose/position/y");
    }
    if (!private_nh.getParam("pose/position/z", pose_msg.pose.position.z)) {
        ROS_ERROR_STREAM("Unable to get param pose/position/z");
    }
    if (!private_nh.getParam("pose/orientation/x", pose_msg.pose.orientation.x)) {
        ROS_ERROR_STREAM("Unable to get param pose/orientation/x");
    }
    if (!private_nh.getParam("pose/orientation/y", pose_msg.pose.orientation.y)) {
        ROS_ERROR_STREAM("Unable to get param pose/orientation/y");
    }
    if (!private_nh.getParam("pose/orientation/z", pose_msg.pose.orientation.z)) {
        ROS_ERROR_STREAM("Unable to get param pose/orientation/z");
    }
    if (!private_nh.getParam("pose/orientation/w", pose_msg.pose.orientation.w)) {
        ROS_ERROR_STREAM("Unable to get param pose/orientation/w");
    }

    std::string child_frame_id;
    if (!private_nh.getParam("child_frame_id", child_frame_id)) {
        ROS_ERROR_STREAM("Unable to get param child_frame_id");
    }

    double rate;
    if (!private_nh.getParam("rate", rate)) {
        ROS_ERROR_STREAM("Unable to get param rate");
    }

    bool        use_filt = private_nh.param<bool>("use_filt", false);
    std::string frame_id = pose_msg.header.frame_id;

    if (use_filt) {
        if (!private_nh.getParam("filt_frame_id", frame_id)) {
            ROS_ERROR_STREAM("Unable to get param filt_frame_id");
        }
    }

    ros::Publisher pub = public_nh.advertise<geometry_msgs::PoseStamped>(child_frame_id, 1);

    tf::Transform tf(tf::Quaternion(pose_msg.pose.orientation.x,
                                    pose_msg.pose.orientation.y,
                                    pose_msg.pose.orientation.z,
                                    pose_msg.pose.orientation.w),
                     tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener    listener;

    tf::StampedTransform worldToToolTf;

    while (ros::ok()) {

        try {
            listener.waitForTransform("optitrack_world", frame_id, ros::Time(0), ros::Duration(10));
            listener.lookupTransform("optitrack_world", frame_id, ros::Time(0), worldToToolTf);
        }
        catch (std::exception& e) {
            ROS_ERROR("%s", e.what());
        }

        auto pose_tf = worldToToolTf * tf;

        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "optitrack_world";
        pose_msg.pose.position.x = pose_tf.getOrigin().getX();
        pose_msg.pose.position.y = pose_tf.getOrigin().getY();
        pose_msg.pose.position.z = pose_tf.getOrigin().getZ();

        pose_msg.pose.orientation.x = pose_tf.getRotation().getX();
        pose_msg.pose.orientation.y = pose_tf.getRotation().getY();
        pose_msg.pose.orientation.z = pose_tf.getRotation().getZ();
        pose_msg.pose.orientation.w = pose_tf.getRotation().getW();


        pub.publish(pose_msg);
        broadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), frame_id, child_frame_id));
        ros::Rate(rate).sleep();
    }
}