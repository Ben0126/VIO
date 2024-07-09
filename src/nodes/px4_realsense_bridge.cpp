#include "../../include/PX4_realsense_bridge/PX4_realsense_bridge.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <Eigen/Eigen>
#include <math.h>


namespace bridge {

PX4_Realsense_Bridge::PX4_Realsense_Bridge(const ros::NodeHandle& nh)
    : nh_(nh) {

  // initialize subscribers
  odom_sub_ = nh_.subscribe<const nav_msgs::Odometry&>("/camera/odom/sample_throttled", 10, &PX4_Realsense_Bridge::odomCallback, this);

  tfmini_sub = nh_.subscribe<const sensor_msgs::Range&>("/mavros/distance_sensor/hrlv_ez4_pub", 10, &PX4_Realsense_Bridge::tfmini_cb, this);

  att_sub = nh_.subscribe<const sensor_msgs::Imu&>("/mavros/imu/data", 10, &PX4_Realsense_Bridge::att_cb, this);

  // publishers
  mavros_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);

  mavros_system_status_pub_ =  nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);

  last_callback_time = ros::Time::now();

  status_mutex_.reset(new std::mutex);
  worker_ = std::thread(&PX4_Realsense_Bridge::publishSystemStatus, this);


};

PX4_Realsense_Bridge::~PX4_Realsense_Bridge() { }

void PX4_Realsense_Bridge::tfmini_cb(const sensor_msgs::Range& msg)
{
	tfmini_z = msg.range;
}

Eigen::Vector3d PX4_Realsense_Bridge::quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}


void PX4_Realsense_Bridge::att_cb(const sensor_msgs::Imu& msg)
{
	 q_fcu= Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
     euler_fcu = PX4_Realsense_Bridge::quaternion_to_euler(q_fcu);
     attitude[0] = euler_fcu[0];
     attitude[1] = euler_fcu[1];
     attitude[2] = euler_fcu[2];
}


void PX4_Realsense_Bridge::odomCallback(const nav_msgs::Odometry& msg) {

  vrt_h = tfmini_z*cos(atan(sqrt( tan(attitude[0])*tan(attitude[0]) + tan(attitude[1])*tan(attitude[1]) )));
  vrt_h = double(int(vrt_h*100)/100.0);
  // publish odometry msg
  nav_msgs::Odometry output = msg;
  output.header.frame_id = msg.header.frame_id;
  output.child_frame_id = msg.child_frame_id;
  output.pose.pose.position.z = vrt_h;
  mavros_odom_pub_.publish(output);

  flag_first_pose_received = true;

  { // lock mutex
    std::lock_guard<std::mutex> status_guard(*(status_mutex_));

    last_system_status_ = system_status_;

    // check confidence in vision estimate by looking at covariance
    if( msg.pose.covariance[0] > 0.1 ) // low confidence -> reboot companion
    {
      system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
    }
    else if( msg.pose.covariance[0] == 0.1 ) // medium confidence
    {
      system_status_ = MAV_STATE::MAV_STATE_CRITICAL;
    }
    else if( msg.pose.covariance[0] == 0.01 ) // high confidence
    {
      system_status_ = MAV_STATE::MAV_STATE_ACTIVE;
    }
    else
    {
      ROS_WARN_STREAM("Unexpected vision sensor variance");
    }  

    // publish system status immediately if it changed
    if( last_system_status_ != system_status_ )
    {
      mavros_msgs::CompanionProcessStatus status_msg;

      status_msg.header.stamp = ros::Time::now();
      status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

      status_msg.state = (int)system_status_;

      mavros_system_status_pub_.publish(status_msg);
    }  

  last_callback_time = ros::Time::now();
    
  }
}


void PX4_Realsense_Bridge::publishSystemStatus(){
  

  while(ros::ok()){
    
    ros::Duration(1).sleep();

    if(flag_first_pose_received == true) { // only send heartbeat if we receive pose estimates at all

      // check if we received an recent update
      // otherwise let the companion computer restart
      if( (ros::Time::now()-last_callback_time) > ros::Duration(0.5) ){
        ROS_WARN_STREAM("Stopped receiving data from T265");
        system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
      }

      mavros_msgs::CompanionProcessStatus status_msg;

      status_msg.header.stamp = ros::Time::now();
      status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
    
      { // lock mutex
        std::lock_guard<std::mutex> status_guard(*(status_mutex_));

        status_msg.state = (int)system_status_;

        mavros_system_status_pub_.publish(status_msg);
      }
    }
  }

}

}
