#ifndef GUIDANCE_H
#define GUIDANCE_H
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
geometry_msgs::Twist guidance_velocity(int state);
geometry_msgs::PoseStamped guidance_position(int state);
geometry_msgs::PoseStamped guidance_position_circle(double theta, double diameter);
#endif