#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseStamped::Ptr pose      = nullptr;
geometry_msgs::PoseStamped::Ptr prev_pose = nullptr;

void pose_callback(geometry_msgs::PoseStamped::Ptr msg)
{
    pose = msg;
}

double lowpass(const double prev, const double curr, const double cutoff, const double rate)
{
    return prev + (curr - prev) * (1.0 - exp(-(1.0 / rate) / (1.0 / (2.0 * M_PI * cutoff))));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lowpass_filter_pose");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    double      cutoff         = private_nh.param<double>("cutoff", 10);
    double      rate           = private_nh.param<double>("rate", 100);
    std::string child_frame_id = private_nh.param<std::string>("child_frame_id", "left_tool_filt");

    ros::Subscriber pose_sub = nh.subscribe("pose_sub_topic", 1, pose_callback);
    ros::Publisher  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_pub_topic", 1);

    tf::TransformBroadcaster broadcaster;

    ros::Rate looprate(rate);

    while (ros::ok()) {
        auto now = ros::Time::now();

        if (prev_pose != nullptr) {
            prev_pose->header.stamp       = now;
            prev_pose->header.frame_id    = pose->header.frame_id;
            prev_pose->pose.position.x    = lowpass(prev_pose->pose.position.x, pose->pose.position.x, cutoff, rate);
            prev_pose->pose.position.y    = lowpass(prev_pose->pose.position.y, pose->pose.position.y, cutoff, rate);
            prev_pose->pose.position.z    = lowpass(prev_pose->pose.position.z, pose->pose.position.z, cutoff, rate);
            // prev_pose->pose.orientation.x = lowpass(prev_pose->pose.orientation.x, pose->pose.orientation.x, cutoff, rate);
            // prev_pose->pose.orientation.y = lowpass(prev_pose->pose.orientation.y, pose->pose.orientation.y, cutoff, rate);
            // prev_pose->pose.orientation.z = lowpass(prev_pose->pose.orientation.z, pose->pose.orientation.z, cutoff, rate);
            // prev_pose->pose.orientation.w = lowpass(prev_pose->pose.orientation.w, pose->pose.orientation.w, cutoff, rate);

            pose_pub.publish(prev_pose);

            tf::Transform filtered_tf;
            filtered_tf.setOrigin(
                tf::Vector3(prev_pose->pose.position.x, prev_pose->pose.position.y, prev_pose->pose.position.z));

            filtered_tf.setRotation(tf::Quaternion(prev_pose->pose.orientation.x,
                                                   prev_pose->pose.orientation.y,
                                                   prev_pose->pose.orientation.z,
                                                   prev_pose->pose.orientation.w));

            broadcaster.sendTransform(tf::StampedTransform(filtered_tf, now, pose->header.frame_id, child_frame_id));
        }

        prev_pose = pose;

        ros::spinOnce();
        looprate.sleep();
    }
}