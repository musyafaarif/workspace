#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::NavSatFix current_pos;
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "boat_com");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("boat_mavros/state", 10, state_cb);
    
    // Boat Communication
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("boat_mavros/global_position/global", 10, global_pos_cb);
    ros::Publisher global_pos_pub = nh.advertise<sensor_msgs::NavSatFix>
            ("boat_global_pos", 10);

    // Publish Global Position in 10Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    ROS_INFO("Wait for Connection..");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");

    while(ros::ok()){
        // Boat Communication
        global_pos_pub.publish(current_pos);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
