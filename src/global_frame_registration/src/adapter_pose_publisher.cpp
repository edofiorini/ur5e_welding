#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#define TF_TIMEOUT 2.0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adapter_pose_publisher");

    ros::NodeHandle privateNh("~");
    ros::NodeHandle publicNh;

    auto adapterLen = privateNh.param<double>("adapter_len", 0.15);
    auto rateVal = privateNh.param<double>("rate", 100.0);

    auto adapterPosePub = publicNh.advertise<geometry_msgs::PoseStamped>("/dvrk/ECM/adapter_position", 1);

    tf::TransformListener tfListener;
    tf::StampedTransform stfEcm_EcmBase;
    tf::Transform tfEcm_Adapter(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, adapterLen));

    geometry_msgs::PoseStamped adapterPoseMsg;

    ros::Rate rate(rateVal);
    while (ros::ok())
    {
        try
        {
            tfListener.waitForTransform("ECM", "ECM_base", ros::Time(0), ros::Duration(TF_TIMEOUT));
            tfListener.lookupTransform("ECM", "ECM_base", ros::Time(0), stfEcm_EcmBase);
        }
        catch (std::exception &e)
        {
            ROS_ERROR("%s", e.what());
        }
        auto tfAdapter_EcmBase = stfEcm_EcmBase.inverse() * tfEcm_Adapter;

        adapterPoseMsg.header.stamp = ros::Time::now();
        adapterPoseMsg.header.frame_id = "ECM_base";

        adapterPoseMsg.pose.position.x = tfAdapter_EcmBase.getOrigin().getX();
        adapterPoseMsg.pose.position.y = tfAdapter_EcmBase.getOrigin().getY();
        adapterPoseMsg.pose.position.z = tfAdapter_EcmBase.getOrigin().getZ();
        adapterPoseMsg.pose.orientation.x = tfAdapter_EcmBase.getRotation().getX();
        adapterPoseMsg.pose.orientation.y = tfAdapter_EcmBase.getRotation().getY();
        adapterPoseMsg.pose.orientation.z = tfAdapter_EcmBase.getRotation().getZ();
        adapterPoseMsg.pose.orientation.w = tfAdapter_EcmBase.getRotation().getW();

        adapterPosePub.publish(adapterPoseMsg);

        rate.sleep();
        ros::spinOnce();
    }
}