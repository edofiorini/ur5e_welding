#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

std::vector<double> joint_des;
bool new_joint_des = false;

void setJointPosition(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    joint_des[0] = msg->data[0];
    joint_des[1] = msg->data[1];
    joint_des[2] = msg->data[2];
    joint_des[3] = msg->data[3];
    joint_des[4] = msg->data[4];
    joint_des[5] = msg->data[5];

    new_joint_des = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_ros");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // starting teleoperation
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    // rtde interface
    ros::Publisher jointsPub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher tpcPosPub = nh.advertise<geometry_msgs::PoseStamped>("ee_pose", 1);
    ros::Publisher tpcVelPub = nh.advertise<geometry_msgs::TwistStamped>("ee_twist", 1);
    ros::Publisher tpcWrenchPub = nh.advertise<geometry_msgs::WrenchStamped>("ee_wrench", 1);

    ros::Subscriber jointPosSub = nh.subscribe<std_msgs::Float32MultiArray>("joint_pos_cmd", 1, &setJointPosition);

    std::string robot_ip = pnh.param("robot_ip", std::string(""));
    std::string robot_ns = pnh.param("robot_ns", std::string(""));
    bool tracking_ctrl_mode = pnh.param("tracking_ctrl_mode", true);
    int freq = pnh.param("freq", 100);
    int urcap_port = pnh.param("urcap_port", 50002);
    ros::Rate loopRate(freq);

    ROS_INFO_STREAM("Connecting to robot " << robot_ip << " on urcap port " << urcap_port);

    ur_rtde::RTDEReceiveInterface rtde_receive(robot_ip);
    ur_rtde::RTDEControlInterface rtde_control(robot_ip);//, freq, ur_rtde::RTDEControlInterface::FLAG_USE_EXT_UR_CAP, urcap_port);

    ROS_INFO_STREAM("Connected to robot " << robot_ip << " on urcap port " << urcap_port);
    ROS_INFO_STREAM("Tracking control mode enabled: " << tracking_ctrl_mode);

    sensor_msgs::JointState joints_msg;
    geometry_msgs::PoseStamped tcpPose_msg;
    geometry_msgs::TwistStamped tcpVel_msg;
    geometry_msgs::WrenchStamped tcpWrench_msg;

    joint_des.assign(6, 0);
    rtde_control.zeroFtSensor();

    while (ros::ok())
    {
        // reading current joint state
        joints_msg.header.stamp = ros::Time::now();
        joints_msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joints_msg.position = rtde_receive.getActualQ();
        joints_msg.velocity = rtde_receive.getActualQd();
        joints_msg.effort = rtde_receive.getActualCurrent();

        jointsPub.publish(joints_msg);

        // reading current cartesian state
        // the rotation is encoded as axis-angle notation
        auto tcpPose = rtde_receive.getActualTCPPose();
        tf::Vector3 axisAngle(tcpPose[3], tcpPose[4], tcpPose[5]);
        tf::Quaternion quat(axisAngle.normalized(), axisAngle.length());

        tcpPose_msg.header.frame_id = robot_ns + "/base";
        tcpPose_msg.header.stamp = ros::Time::now();
        tcpPose_msg.pose.position.x = tcpPose[0];
        tcpPose_msg.pose.position.y = tcpPose[1];
        tcpPose_msg.pose.position.z = tcpPose[2];
        tcpPose_msg.pose.orientation.x = quat.x();
        tcpPose_msg.pose.orientation.y = quat.y();
        tcpPose_msg.pose.orientation.z = quat.z();
        tcpPose_msg.pose.orientation.w = quat.w();

        auto tpcvel = rtde_receive.getActualTCPSpeed();

        tcpVel_msg.header.frame_id = robot_ns + "/base";
        tcpVel_msg.header.stamp = ros::Time::now();
        tcpVel_msg.twist.linear.x = tpcvel[0];
        tcpVel_msg.twist.linear.y = tpcvel[1];
        tcpVel_msg.twist.linear.z = tpcvel[2];
        tcpVel_msg.twist.angular.x = tpcvel[3];
        tcpVel_msg.twist.angular.y = tpcvel[4];
        tcpVel_msg.twist.angular.z = tpcvel[5];

        auto tpcwrench = rtde_receive.getActualTCPForce();

        tcpWrench_msg.header.frame_id = robot_ns + "/base";
        tcpWrench_msg.header.stamp = ros::Time::now();
        tcpWrench_msg.wrench.force.x = tpcwrench[0];
        tcpWrench_msg.wrench.force.y = tpcwrench[1];
        tcpWrench_msg.wrench.force.z = tpcwrench[2];
        tcpWrench_msg.wrench.torque.x = tpcwrench[3];
        tcpWrench_msg.wrench.torque.y = tpcwrench[4];
        tcpWrench_msg.wrench.torque.z = tpcwrench[5];

        tpcPosPub.publish(tcpPose_msg);
        tpcVelPub.publish(tcpVel_msg);
        tpcWrenchPub.publish(tcpWrench_msg);

        // publishing the tcp pose as a transformation
        // tf::StampedTransform tcpTf;
        // tcpTf.stamp_ = ros::Time::now();
        // tcpTf.frame_id_ = robot_ns + "/base";
        // tcpTf.child_frame_id_ = robot_ns + "/end_effector";

        // tcpTf.setOrigin(tf::Vector3(
        //     tcpPose_msg.pose.position.x,
        //     tcpPose_msg.pose.position.y,
        //     tcpPose_msg.pose.position.z));
        // tcpTf.setRotation(quat);

        // broadcaster.sendTransform(tcpTf);

        if (new_joint_des)
        {
            if(tracking_ctrl_mode)
            {
                double lookahead_time = 0.17;
                double gain = 1500;
                // servoJ is used to provide continuos target (like in teleoperation or trajectory tracking)
                // https://www.universal-robots.com/articles/ur/programming/servoj-command/
                rtde_control.servoJ(joint_des, 0, 0, 1.0 / freq, lookahead_time, gain);
            }
            else
            {
                rtde_control.moveJ(joint_des, 3.14, 2, false);
            }
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}