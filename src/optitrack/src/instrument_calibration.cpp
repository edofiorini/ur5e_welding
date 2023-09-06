#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

geometry_msgs::PoseStamped trakcable_pose;
bool received = false;

void pose_callback(geometry_msgs::PoseStamped msg)
{
    trakcable_pose = msg;
    received = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "instument_calibration");
    ros::NodeHandle private_nh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // get parameters
    std::string ref_frame;
    if(!private_nh.getParam("optitrack_ref_frame", ref_frame)){
        ROS_ERROR("Unable to parse ref_frame param");
    }

    std::string tool_frame;
    if(!private_nh.getParam("optitrack_tool_frame", tool_frame)){
        ROS_ERROR("Unable to parse tool_frame param");
    }

    std::string base_frame;
    if(!private_nh.getParam("tool_base_frame", base_frame)){
        ROS_ERROR("Unable to parse base_frame param");
    }

    std::string ns;
    if(!private_nh.getParam("ns", ns)){
        ROS_ERROR("Unable to parse ns param");
    }

    double rate = 100;
    if(!private_nh.getParam("rate", rate)){
        ROS_ERROR("Unable to parse rate param");
    }

    std::string output_file;
    if(!private_nh.getParam("output_file", output_file)){
        ROS_ERROR("Unable to parse output_file param");
    }

    tf::StampedTransform tool_tf;
    tf::StampedTransform base_calib;

    tf::TransformListener listener;
    listener.waitForTransform(ref_frame, tool_frame, ros::Time(0), ros::Duration(10));
    listener.lookupTransform(ref_frame, tool_frame, ros::Time(0), tool_tf);

    listener.waitForTransform(ref_frame, base_frame, ros::Time(0), ros::Duration(10));
    listener.lookupTransform(ref_frame, base_frame, ros::Time(0), base_calib);

    auto tool_tip_tf = tool_tf.inverse() * base_calib;

    tool_tip_tf.setRotation(tool_tip_tf.getRotation() * tf::Quaternion(tf::Vector3(1,0,0), M_PI_2));

    std::ofstream of(output_file);
    of << "header:\n    frame_id: \"" << tool_frame << "\"\n";
    of << "pose:\n    position:" << std::endl;
    of << "        x: " << tool_tip_tf.getOrigin().getX() << std::endl;
    of << "        y: " << tool_tip_tf.getOrigin().getY() << std::endl;
    of << "        z: " << tool_tip_tf.getOrigin().getZ() << std::endl;
    of << "    orientation:" << std::endl;
    of << "        x: " << tool_tip_tf.getRotation().getX() << std::endl;
    of << "        y: " << tool_tip_tf.getRotation().getY() << std::endl;
    of << "        z: " << tool_tip_tf.getRotation().getZ() << std::endl;
    of << "        w: " << tool_tip_tf.getRotation().getW() << std::endl;
    of.close();

    ROS_INFO_STREAM("Calibration DONE. You can kill this node");

    ros::waitForShutdown();
}