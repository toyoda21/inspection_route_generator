#include "inspection_route/guidance.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "aruco_msgs/Marker.h"//マーカ情報取得に必要な変数がパッケージ独自のため引用
#include "aruco_msgs/MarkerArray.h"//マーカ情報取得に必要な変数がパッケージ独自のため引用
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>

geometry_msgs::Twist guidance_velocity(int state, geometry_msgs::PoseStamped change_mode_pose){
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

geometry_msgs::PoseStamped guidance_position(int state){
    geometry_msgs::PoseStamped insp_pose;
    if( state == 0 ){
        insp_pose.pose.position.x = 0;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 0.8;
    }else if( state == 1 ){
        insp_pose.pose.position.x = 0;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 0.8;
    }else if( state == 2 ){
        insp_pose.pose.position.x = 0.5;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 0.8;
    }else if( state == 3 ){
        insp_pose.pose.position.x = 0.5;
        insp_pose.pose.position.y = 0;
        insp_pose.pose.position.z = 1.0;
    }else if( state == 4 ){
        insp_pose.pose.position.x = 0.5;
        insp_pose.pose.position.y = 0.5;
        insp_pose.pose.position.z = 1;
    }else if( state == 5){
        insp_pose.pose.position.x = 0.5;
        insp_pose.pose.position.y = 0.5;
        insp_pose.pose.position.z = 0.8;
    }else if( state == 6 ){
        insp_pose.pose.position.x = 0;
        insp_pose.pose.position.y = 0.5;
        insp_pose.pose.position.z = 0.8;
    }else{}
        insp_pose.pose.orientation.w = 0;
        insp_pose.pose.orientation.x = 0;
        insp_pose.pose.orientation.y = 0;
        insp_pose.pose.orientation.z = 0;
    return insp_pose;
}
//マーカguidance
geometry_msgs::PoseStamped guidance_marker_route(int state, geometry_msgs::PoseStamped current_target_pose, aruco_msgs::Marker mk_msg, geometry_msgs::PoseStamped fixation_pose){
    geometry_msgs::PoseStamped insp_pose;
    if(state == 0){ROS_INFO("state: 0");
	    insp_pose.pose.position.x = fixation_pose.pose.position.x;
        insp_pose.pose.position.y = fixation_pose.pose.position.y;
        insp_pose.pose.position.z = 0.8;
    }
	
    if(mk_msg.id == 2 ){
        if( state == 1 ){ROS_INFO("state: 1");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 2.0;
        }else if( state == 2 ){ROS_INFO("state: 2");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.x + 1;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 2.0;
        }else{ROS_INFO("state return: 0");}
    }
	if(mk_msg.id == 26 ){
        if( state == 1 ){ROS_INFO("state: 1");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 2.0;
        }else if( state == 2 ){ROS_INFO("state: 2");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 3.0;
		}else if( state == 2 ){ROS_INFO("state: 2");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y + 1;
		  insp_pose.pose.position.z = 3.0;
		}else if( state == 2 ){ROS_INFO("state: 2");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y + 1;
		  insp_pose.pose.position.z = 2.0;
        }else{ROS_INFO("state return: 0");}
    }
	if(mk_msg.id == 110 ){
        if( state == 1 ){ROS_INFO("state: 1");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 2.0;
        }else if( state == 2 ){ROS_INFO("state: 2");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x - 1;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 2.0;
        }else{ROS_INFO("state return: 0");}
    }
	if(mk_msg.id == 1000 ){
        if( state == 1 ){ROS_INFO("state: 1");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y;
		  insp_pose.pose.position.z = 2.0;
        }else if( state == 2 ){ROS_INFO("state: 2");
		  insp_pose.pose.position.x = -mk_msg.pose.pose.position.y + fixation_pose.pose.position.x;
		  insp_pose.pose.position.y = -mk_msg.pose.pose.position.x + fixation_pose.pose.position.y - 1;
		  insp_pose.pose.position.z = 2.0;
        }else{ROS_INFO("state return: 0");}
	}
        insp_pose.pose.orientation.w = 0;
        insp_pose.pose.orientation.x = 0;
        insp_pose.pose.orientation.y = 0;
        insp_pose.pose.orientation.z = 0;
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
