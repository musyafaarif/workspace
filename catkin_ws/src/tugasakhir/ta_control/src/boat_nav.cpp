#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <math.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "boat_nav");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("boat_mavros/state", 10, state_cb);
    ros::Publisher raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("boat_mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("boat_mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("boat_mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);

    // wait for FCU connection
    ROS_INFO("Wait for Connection..");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    mavros_msgs::PositionTarget tgt;
    tgt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    tgt.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    tgt.velocity.x = 0.0f;
    tgt.velocity.x = 15.0f;
    // tgt.yaw_rate = 1.0f;

    //send a few setpoints before starting
    ROS_INFO("Send a few setpoints..");
    for(int i = 5; ros::ok() && i > 0; --i){
        raw_pub.publish(tgt);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_course = ros::Time::now();

    int course = 1;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (ros::Time::now() - last_course > ros::Duration(90.0)) {
            course = (course + 1) % 4;
            tgt.velocity.x = 15.0f * sin(course * M_PI / 2.0f);
            tgt.velocity.y = 15.0f * cos(course * M_PI / 2.0f);
            ROS_INFO("Changed to Course: %d", course);
            ROS_INFO("Set Velocity X to: %0.1f", tgt.velocity.x);
            ROS_INFO("Set Velocity Y to: %0.1f", tgt.velocity.y);

            last_course = ros::Time::now();
        }

        raw_pub.publish(tgt);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
