// SPDX-License-Identifier: BSD-2-Clause

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace hdl_graph_slam {

class OdomRepublisherNodelet : public nodelet::Nodelet {
public:
  OdomRepublisherNodelet() {}
  virtual ~OdomRepublisherNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    target_parent_frame = private_nh.param<std::string>("target_parent_frame", "");
    target_child_frame = private_nh.param<std::string>("target_child_frame", "");

    // subscribers
    odom_sub = nh.subscribe("/gps/geopoint", 32, &OdomRepublisherNodelet::poseCallback, this);
    filtered_points_sub = nh.subscribe("/filtered_points", 32, &OdomRepublisherNodelet::filteredPointsCallback, this);

    // publishers
    odom_pub = mt_nh.advertise<nav_msgs::Odometry>("/hdl_graph_slam/odom_repub", 32);
    conv_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_conv", 32);
  }

private:
  void poseCallback(const nav_msgs::OdometryConstPtr& msg){

      ros::Time current_time = ros::Time::now();
      nav_msgs::Odometry pose_gt_frame;

      pose_gt_frame.header.frame_id = target_parent_frame;
      pose_gt_frame.header.stamp = current_time;
      pose_gt_frame.child_frame_id = target_child_frame;

      pose_gt_frame.twist = msg->twist;
      pose_gt_frame.pose = msg->pose;

      //publish the message
      odom_pub.publish(pose_gt_frame);

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = target_parent_frame;
      odom_trans.child_frame_id = target_child_frame;

      odom_trans.transform.translation.x = msg->pose.pose.position.x;
      odom_trans.transform.translation.y = msg->pose.pose.position.y;
      odom_trans.transform.translation.z = msg->pose.pose.position.z;
      odom_trans.transform.rotation = msg->pose.pose.orientation;

      br.sendTransform(odom_trans);

  }

  void filteredPointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    sensor_msgs::PointCloud2 conv_filtered_pts_msg;

    conv_filtered_pts_msg = *msg;
    conv_filtered_pts_msg.header.stamp = ros::Time::now();

    conv_points_pub.publish(conv_filtered_pts_msg);

  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Subscriber odom_sub;
  ros::Subscriber filtered_points_sub;

  ros::Publisher odom_pub;
  ros::Publisher conv_points_pub;

  tf::TransformBroadcaster br;

  std::string target_parent_frame;
  std::string target_child_frame;

};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::OdomRepublisherNodelet, nodelet::Nodelet)