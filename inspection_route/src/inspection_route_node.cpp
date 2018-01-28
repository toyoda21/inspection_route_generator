/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "inspection_route/guidance.h"
#include "aruco_msgs/Marker.h"//マーカ情報取得に必要な変数がパッケージ独自のため引用
#include "aruco_msgs/MarkerArray.h"//マーカ情報取得に必要な変数がパッケージ独自のため引用
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>

int state = 0;
bool offb_flag = false;
bool arm_flag = false;
int task;

//autopilotの現在の状態を保存するコールバックを作成する
//接続・アーム・オフボードフラグを確認する
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//現在のlocal poseを保存するコールバックを作成する
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
    current_pose = *pose_msg;
}

//マーカ検出
aruco_msgs::Marker marker_data;
geometry_msgs::PoseStamped fixation_pose;
geometry_msgs::PoseStamped current_target_pose;
void marker_cb(const aruco_msgs::MarkerArray::ConstPtr& mk_msg){
    if(state == 0){
	  state = 1;
      marker_data.id = mk_msg->markers[0].id;
	  marker_data.pose.pose.position.x = mk_msg->markers[0].pose.pose.position.x;
	  marker_data.pose.pose.position.y = mk_msg->markers[0].pose.pose.position.y;
	  marker_data.pose.pose.position.z = mk_msg->markers[0].pose.pose.position.z;
	  
	  fixation_pose.pose.position.x = current_target_pose.pose.position.x;
	  fixation_pose.pose.position.y = current_target_pose.pose.position.y;
	  fixation_pose.pose.position.z = 1.5;
	  fixation_pose.pose.orientation.w = 0;
	  fixation_pose.pose.orientation.x = 0;
	  fixation_pose.pose.orientation.y = 0;
	  fixation_pose.pose.orientation.z = 0;
	  
      if(marker_data.id == 2){
		task = 3;//task1~2
		ROS_INFO("ID:2");
	  }
	  if(marker_data.id == 26){
		task = 3;//task1~2
		ROS_INFO("ID:26");
	  }
      if(marker_data.id == 110){
		task = 3;//task1~2
		ROS_INFO("ID:110");
	  }
	  if(marker_data.id == 1000){
		task = 3;//task1~2
		ROS_INFO("ID:1000");
	  }
    }
	
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspection_route_node");
    ros::NodeHandle nh;

    //命令したローカルポジションとクライアントを発行する
    //アームとモード変更を要求する
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber marker_sub = nh.subscribe<aruco_msgs::MarkerArray>
            ("aruco_marker_publisher/markers", 5, marker_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist twist;

    //オフボードモードに設定する
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //アームコマンドを送信する
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // wait for OFFBOARD mode connection
    while(ros::ok() && current_state.mode != "OFFBOARD"){
	ROS_INFO("Waiting ...AAA");
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0.8;
        pose.pose.orientation.w = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        local_pos_pub.publish(pose);
		ros::spinOnce();
        rate.sleep();
	}
	ROS_INFO("Start offboard mode!");
    
    //main loop
    //geometry_msgs::PoseStamped current_target_pose;
    double start_request;
	double now_request;
	bool timer = false;
    while (ros::ok()){
	  if(state < task && state > 0){
		if(   fabs(pose.pose.position.x - current_pose.pose.position.x) < 0.08
		   && fabs(pose.pose.position.y - current_pose.pose.position.y) < 0.08
		   && fabs(pose.pose.position.z - current_pose.pose.position.z) < 0.08
			   ){
			  if(timer == false ){
			    start_request = ros::Time::now().toSec();
				timer = true;
			  }else{
				now_request = ros::Time::now().toSec();
                if(now_request - start_request > 10.0){
                  state++;
                  timer = false;
				  ROS_INFO("next state");
				}
			  }
		    }
			pose = guidance_marker_route(state, current_target_pose, marker_data, fixation_pose);
	  }else{state = 0;
        pose = guidance_marker_route(state, current_target_pose, marker_data, fixation_pose);
	  }
	  current_target_pose = pose;
	  local_pos_pub.publish(pose);
	  ros::spinOnce();
	  rate.sleep();
    }

    return 0;
}
