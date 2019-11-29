#include "inspection_route/guidance.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

geometry_msgs::Twist guidance_velocity(int state){
    geometry_msgs::Twist twist;
    if( state == 0 ){
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.x = 0;
        }
    if ( state == 1 ){
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
    }
    if ( state == 2 ){
        twist.linear.x = 0;
        twist.linear.y = 1;
        twist.linear.x = 1;
    }
    if( state == 3 ){
        twist.linear.x = -1;
        twist.linear.y = 0;
        twist.linear.x = 1;
    }
    if( state == 4 ){
        twist.linear.x = 0;
        twist.linear.y = -1;
        twist.linear.z = 1;
    }
    return twist;
}
/*
geometry_msgs::PoseStamped insp_pose;
insp_pose.pose.position.x = 0;
insp_pose.pose.position.y = 0;
insp_pose.pose.position.z = 2;
*/
geometry_msgs::PoseStamped guidance_position(int state){
    geometry_msgs::PoseStamped insp_pose;
    if( state == 0 ){
        insp_pose.pose.position.x = 0;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 2;
    }else if( state == 1 ){
        insp_pose.pose.position.x = 0;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 2;
    }else if( state == 2 ){
        insp_pose.pose.position.x = 2;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 2;
    }else if( state == 3 ){
        insp_pose.pose.position.x = 2;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 3;
    }else if( state == 4 ){
        insp_pose.pose.position.x = 2;
        insp_pose.pose.position.y = 2;
        insp_pose.pose.position.z = 3;
    }else if( state == 5){
        insp_pose.pose.position.x = 2;
        insp_pose.pose.position.y = 2;
        insp_pose.pose.position.z = 2;
    }else if( state == 6 ){
        insp_pose.pose.position.x = 0;
        insp_pose.pose.position.y = 2;
        insp_pose.pose.position.z = 2;
    }else{}
    return insp_pose;
}
//circle route
geometry_msgs::PoseStamped guidance_position_circle(double theta, double diameter){
    geometry_msgs::PoseStamped circle_pose;
    circle_pose.pose.position.x = (diameter/2) * sin(theta);
    circle_pose.pose.position.y = (diameter/2) * cos(theta);
    circle_pose.pose.position.z = 2;//高度2mで固定
    return circle_pose;
}