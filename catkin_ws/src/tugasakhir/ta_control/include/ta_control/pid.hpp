#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <ta_control/math.hpp>

class PID
{
public:
// private:
    ros::NodeHandle *nh;
    mavros_msgs::PositionTarget *setpoint;

    ros::Publisher reset_x_pub;
    ros::Publisher reset_y_pub;
    ros::Publisher reset_z_pub;
    ros::Publisher reset_yaw_pub;
    ros::Publisher enable_x_pub;
    ros::Publisher enable_y_pub;
    ros::Publisher enable_z_pub;
    ros::Publisher enable_yaw_pub;
    ros::Publisher state_x_pub;
    ros::Publisher state_y_pub;
    ros::Publisher state_z_pub;
    ros::Publisher state_yaw_pub;
    ros::Publisher setpoint_x_pub;
    ros::Publisher setpoint_y_pub;
    ros::Publisher setpoint_z_pub;
    ros::Publisher setpoint_yaw_pub;
    ros::Subscriber control_x_sub;
    ros::Subscriber control_y_sub;
    ros::Subscriber control_z_sub;
    ros::Subscriber control_yaw_sub;

    void control_x_cb(const std_msgs::Float64::ConstPtr&);
    void control_y_cb(const std_msgs::Float64::ConstPtr&);
    void control_z_cb(const std_msgs::Float64::ConstPtr&);
    void control_yaw_cb(const std_msgs::Float64::ConstPtr&);

    const uint8_t queue_size = 10;
    std_msgs::Bool enabled;
    std_msgs::Float64 state_x;
    std_msgs::Float64 state_y;
    std_msgs::Float64 state_z;
    std_msgs::Float64 state_yaw;
    std_msgs::Float64 setpoint_x;
    std_msgs::Float64 setpoint_y;
    std_msgs::Float64 setpoint_z;
    std_msgs::Float64 setpoint_yaw;
    std_msgs::Float64 control_x;
    std_msgs::Float64 control_y;
    std_msgs::Float64 control_z;
    std_msgs::Float64 control_yaw;
public:
    PID(ros::NodeHandle*, mavros_msgs::PositionTarget*);
    ~PID();

    void init(double_t, double_t, double_t, double_t, double_t, double_t, double_t, double_t);
    void update_x(double_t, double_t);
    void update_x(double_t);
    void update_y(double_t, double_t);
    void update_y(double_t);
    void update_z(double_t, double_t);
    void update_z(double_t);
    void update_yaw(double_t, double_t);
    void update_yaw(double_t);
    void enable();
    void disable();
    void reset();

    void publish();
};

PID::PID(ros::NodeHandle *parent, mavros_msgs::PositionTarget *target) :
    nh(parent), setpoint(target)
{
    reset_x_pub = nh->advertise<std_msgs::Bool>("controller_x/pid_reset", queue_size);
    reset_y_pub = nh->advertise<std_msgs::Bool>("controller_y/pid_reset", queue_size);
    reset_z_pub = nh->advertise<std_msgs::Bool>("controller_z/pid_reset", queue_size);
    reset_yaw_pub = nh->advertise<std_msgs::Bool>("controller_yaw/pid_reset", queue_size);
    enable_x_pub = nh->advertise<std_msgs::Bool>("controller_x/pid_enable", queue_size);
    enable_y_pub = nh->advertise<std_msgs::Bool>("controller_y/pid_enable", queue_size);
    enable_z_pub = nh->advertise<std_msgs::Bool>("controller_z/pid_enable", queue_size);
    enable_yaw_pub = nh->advertise<std_msgs::Bool>("controller_yaw/pid_enable", queue_size);
    state_x_pub = nh->advertise<std_msgs::Float64>("controller_x/state", queue_size);
    state_y_pub = nh->advertise<std_msgs::Float64>("controller_y/state", queue_size);
    state_z_pub = nh->advertise<std_msgs::Float64>("controller_z/state", queue_size);
    state_yaw_pub = nh->advertise<std_msgs::Float64>("controller_yaw/state", queue_size);
    setpoint_x_pub = nh->advertise<std_msgs::Float64>("controller_x/setpoint", queue_size);
    setpoint_y_pub = nh->advertise<std_msgs::Float64>("controller_y/setpoint", queue_size);
    setpoint_z_pub = nh->advertise<std_msgs::Float64>("controller_z/setpoint", queue_size);
    setpoint_yaw_pub = nh->advertise<std_msgs::Float64>("controller_yaw/setpoint", queue_size);
    control_x_sub = nh->subscribe<std_msgs::Float64>("controller_x/control_effort", queue_size, &PID::control_x_cb, this);
    control_y_sub = nh->subscribe<std_msgs::Float64>("controller_y/control_effort", queue_size, &PID::control_y_cb, this);
    control_z_sub = nh->subscribe<std_msgs::Float64>("controller_z/control_effort", queue_size, &PID::control_z_cb, this);
    control_yaw_sub = nh->subscribe<std_msgs::Float64>("controller_yaw/control_effort", queue_size, &PID::control_yaw_cb, this);
}

PID::~PID()
{
}

void PID::init(
    double_t state_x = 0.0f,
    double_t state_y = 0.0f,
    double_t state_z = 0.0f,
    double_t state_yaw = 0.0f,
    double_t setpoint_x = 0.0f,
    double_t setpoint_y = 0.0f,
    double_t setpoint_z = 0.0f,
    double_t setpoint_yaw = 0.0f
    ){
    
    this->disable();

    this->state_x.data = state_x;
    this->state_y.data = state_y;
    this->state_z.data = state_z;
    this->state_yaw.data = state_yaw;
    this->setpoint_x.data = setpoint_x;
    this->setpoint_y.data = setpoint_y;
    this->setpoint_z.data = setpoint_z;
    this->setpoint_yaw.data = setpoint_yaw;
}

void PID::control_x_cb(const std_msgs::Float64::ConstPtr& msg) {
    setpoint->velocity.x = msg->data;
    control_x = *msg;
}

void PID::control_y_cb(const std_msgs::Float64::ConstPtr& msg) {
    setpoint->velocity.y = msg->data;
    control_y = *msg;
}

void PID::control_z_cb(const std_msgs::Float64::ConstPtr& msg) {
    setpoint->velocity.z = msg->data;
    control_z = *msg;
}

void PID::control_yaw_cb(const std_msgs::Float64::ConstPtr& msg) {
    setpoint->yaw_rate = msg->data;
    control_yaw = *msg;
}

void PID::update_x(double_t state, double_t setpoint){
    state_x.data = state;
    setpoint_x.data = setpoint;
}

void PID::update_x(double_t state){
    state_x.data = state;
}

void PID::update_y(double_t state, double_t setpoint){
    state_y.data = state;
    setpoint_y.data = setpoint;
}

void PID::update_y(double_t state){
    state_y.data = state;
}

void PID::update_z(double_t state, double_t setpoint){
    state_z.data = state;
    setpoint_z.data = setpoint;
}

void PID::update_z(double_t state){
    state_z.data = state;
}

void PID::update_yaw(double_t state, double_t setpoint){
    // state_yaw.data = state / 180.0 * M_PI;
    // setpoint_yaw.data = setpoint / 180.0 * M_PI;
    state_yaw.data = state;
    setpoint_yaw.data = setpoint;
}

void PID::update_yaw(double_t state){
    state_yaw.data = state;
}

void PID::reset() {
    std_msgs::Bool msg;

    reset_x_pub.publish(msg);
    reset_y_pub.publish(msg);
}

void PID::enable(){
    std_msgs::Bool msg;
    msg.data = true;

    enable_x_pub.publish(msg);
    enable_y_pub.publish(msg);
    enable_z_pub.publish(msg);
    enable_yaw_pub.publish(msg);
}

void PID::disable(){
    std_msgs::Bool msg;
    msg.data = false;

    enable_x_pub.publish(msg);
    enable_y_pub.publish(msg);
    enable_z_pub.publish(msg);
    enable_yaw_pub.publish(msg);
}

void PID::publish(){
    state_x_pub.publish(state_x);
    state_y_pub.publish(state_y);
    state_z_pub.publish(state_z);
    state_yaw_pub.publish(state_yaw);
    setpoint_x_pub.publish(setpoint_x);
    setpoint_y_pub.publish(setpoint_y);
    setpoint_z_pub.publish(setpoint_z);
    setpoint_yaw_pub.publish(setpoint_yaw);
}

// END
