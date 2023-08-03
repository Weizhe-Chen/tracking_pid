#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

using namespace std;

/////////////////
// Topic Names //
/////////////////
string odom_topic = "/odom";
string goal_topic = "/goal";
string cmd_topic = "/cmd_vel";
string goal_reached_topic = "/goal_reached";

/////////////
// Options //
/////////////
bool print_odom = false;
bool print_goal = true;
bool print_cmd = true;
bool rotate_in_place = false;
int sub_queue_size = 5;
int pub_queue_size = 2;

///////////////
// Variables //
///////////////
double odom_x = 0.0;
double odom_y = 0.0;
double odom_z = 0.0;
double odom_roll = 0.0;
double odom_pitch = 0.0;
double odom_yaw = 0.0;

double goal_x = 0.0;
double goal_y = 0.0;
double goal_z = 0.0;
double goal_roll = 0.0;
double goal_pitch = 0.0;
double goal_yaw = 0.0;

double proportional_gain_x = 0.8;
double proportional_gain_yaw = 1.0;

double max_vel_x = 1.0;
double min_vel_x = -1.0;

double rotate_dist_threshold = 0.1;
int odom_waiting_count = 1;

double yaw_tolerance = 20.0; // degrees
double goal_tolerance = 0.3; // meters

////////////////
// Publishers //
////////////////
ros::Publisher pub_cmd;
ros::Publisher pub_goal_reached;

//////////////
// Messages //
//////////////
std_msgs::Bool goal_reached_msg;
geometry_msgs::Twist cmd_msg;

///////////////////////////////////////////////////////////////////////////////

void stopCmdSigintHandler(int sig) {
  ROS_INFO("Stopping the robot and shutting down the tracking_node...");
  cmd_msg.linear.x = 0.0;
  cmd_msg.linear.y = 0.0;
  cmd_msg.linear.z = 0.0;
  cmd_msg.angular.z = 0.0;
  cmd_msg.angular.y = 0.0;
  cmd_msg.angular.x = 0.0;
  pub_cmd.publish(cmd_msg);
  if (print_cmd) {
    ROS_INFO("Command | X: % .2f | Y: % .2f | Z: % .2f | Roll: % .2f | "
             "Pitch: % .2f | Yaw: % .2f ",
             cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z,
             cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
  }
  ROS_INFO("Done!");
  ros::shutdown();
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &msg) {
  if (odom_waiting_count >= 0) {
    ROS_INFO("Waiting for stable odom data -> step %d", odom_waiting_count);
    if (odom_waiting_count == 0) {
      geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
      tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w))
          .getRPY(odom_roll, odom_pitch, odom_yaw);
      goal_x += msg->pose.pose.position.x;
      goal_y += msg->pose.pose.position.y;
      goal_yaw += odom_yaw;
      if (print_goal) {
        ROS_INFO("Init Goal | X: % .2f | Y: % .2f", goal_x, goal_y);
      }
    }
    odom_waiting_count--;
    return;
  }

  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w))
      .getRPY(odom_roll, odom_pitch, odom_yaw);
  odom_x = msg->pose.pose.position.x;
  odom_y = msg->pose.pose.position.y;
  odom_z = msg->pose.pose.position.z;
  if (print_odom) {
    ROS_INFO("Odometry | X: % .2f | Y: % .2f | Z: % .2f | Roll: % .2f | "
             "Pitch: % .2f | Yaw: % .2f ",
             odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw);
  }

  double dx = goal_x - odom_x;
  double dy = goal_y - odom_y;
  double dist = sqrt(dx * dx + dy * dy);

  double dx_odom = cos(odom_yaw) * dx + sin(odom_yaw) * dy;
  double dy_odom = -sin(odom_yaw) * dx + cos(odom_yaw) * dy;

  double vel_x = proportional_gain_x * dx_odom;
  vel_x = max(min(vel_x, max_vel_x), min_vel_x);

  double dyaw = atan2(dy_odom, dx_odom);

  if (dist < rotate_dist_threshold) {
    // vel_yaw = 0.0;
    dyaw = goal_yaw - odom_yaw;
    if (dyaw > M_PI) {
      dyaw -= 2 * M_PI;
    } else if (dyaw < -M_PI) {
      dyaw += 2 * M_PI;
    }
  }

  double vel_yaw = proportional_gain_yaw * dyaw;

  if (dist < goal_tolerance && abs(dyaw) < yaw_tolerance * M_PI / 180.0) {
    goal_reached_msg.data = true;
    pub_goal_reached.publish(goal_reached_msg);
  } else {
    goal_reached_msg.data = false;
    pub_goal_reached.publish(goal_reached_msg);
  }

  cmd_msg.linear.x = vel_x;
  cmd_msg.linear.y = 0.0;
  cmd_msg.linear.z = 0.0;
  cmd_msg.angular.z = vel_yaw;
  cmd_msg.angular.y = 0.0;
  cmd_msg.angular.x = 0.0;
  pub_cmd.publish(cmd_msg);

  if (print_cmd) {
    ROS_INFO("Command | X: % .2f | Y: % .2f | Z: % .2f | Roll: % .2f | "
             "Pitch: % .2f | Yaw: % .2f | Dist: %.2f | Dyaw: % .2f",
             cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z,
             cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z, dist,
             dyaw);
  }
}

void goalHandler(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;
  goal_z = msg->pose.position.z;

  geometry_msgs::Quaternion quat = msg->pose.orientation;
  tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w))
      .getRPY(goal_roll, goal_pitch, goal_yaw);

  if (print_goal) {
    ROS_INFO("Goal | X: % .2f | Y: % .2f | Z: % .2f | Roll: % .2f | "
             "Pitch: % .2f | Yaw: % .2f ",
             goal_x, goal_y, goal_z, goal_roll, goal_pitch, goal_yaw);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_tracking_node");
  ros::NodeHandle nh;

  // Stop the robot if this node is killed.
  signal(SIGINT, stopCmdSigintHandler);

  /////////////////
  // Subscribers //
  /////////////////
  ros::Subscriber sub_odom =
      nh.subscribe<nav_msgs::Odometry>(odom_topic, sub_queue_size, odomHandler);
  ros::Subscriber sub_goal = nh.subscribe<geometry_msgs::PoseStamped>(
      goal_topic, sub_queue_size, goalHandler);

  ////////////////
  // Publishers //
  ////////////////
  pub_cmd = nh.advertise<geometry_msgs::Twist>(cmd_topic, pub_queue_size);
  pub_goal_reached =
      nh.advertise<std_msgs::Bool>(goal_reached_topic, pub_queue_size);

  ros::spin();
  return 0;
}
