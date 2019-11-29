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

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    //while(ros::ok() && !current_state.connected){
    //    ros::spinOnce();
    //    rate.sleep();
    //}
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    //send a few setpoints before starting
    //オフボードモードに入る前にsetpointのストリーミングを開始する
    for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    //オフボードモードに設定する
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //アームコマンドを送信する
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    int state = 0;
    bool offb_flag = false;
    bool arm_flag = false;
    /*while (ros::ok() && !offb_flag){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){//サービスコールは5s間隔で送る
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");//offbが有効になっていたら表示される
                offb_flag = true;
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    while (ros::ok() && !arm_flag){
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
            ROS_INFO("Vehicle armed");//armコマンドを受け取ったら表示される
            arm_flag = true;
                }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }//OFFBOARDモード後armしていることを確認*/
    
    ros::Time set_request = ros::Time::now();
    while (ros::ok()){
        if(state < 7){
            if(fabs(pose.pose.position.x - current_pose.pose.position.x) < 0.1
            && fabs(pose.pose.position.y - current_pose.pose.position.y) < 0.1
            && fabs(pose.pose.position.z - current_pose.pose.position.z) < 0.1 ){state++;}
        }else{state = 0;}
        pose = guidance_position(state);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
