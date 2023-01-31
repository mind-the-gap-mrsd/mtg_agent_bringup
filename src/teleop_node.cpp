#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mtg_messages/agents_status.h>
#include <vector>
#include <string>
#include <iostream>

class active_agent{
public:
    int robot_id;
    std::string ip_address;

    active_agent(int id, std::string address){
        robot_id = id;
        ip_address = address;
    }
};

class MtgTeleop{
public:
    MtgTeleop();

private:

    void gamepad_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void status_msg_callback(const mtg_messages::agents_status::ConstPtr& status);

    ros::NodeHandle nh;
    int linear_joy_axes {1}; 
    int angular_joy_axes {3};
    double axes_threshold {0.3};
    double const_linear_velocity {0.5}; 
    double const_angular_velocity {1.0};
    int current_agent {0};
    int velocity_inc_button, velocity_dec_button;
    int agent_change_button_right, agent_change_button_left;
    std::vector<active_agent> active_agent_list;
    std::vector<ros::Publisher> vel_pub_list;
    ros::Subscriber gamepad_subscriber;
    ros::Subscriber status_subscriber;
};

MtgTeleop::MtgTeleop(){
    nh.getParam("velocity_inc_button", velocity_inc_button);
    nh.getParam("velocity_dec_button", velocity_dec_button);
    nh.getParam("agent_change_button_right", agent_change_button_right);
    nh.getParam("agent_change_button_left", agent_change_button_left);

    for (int i=0; i<10; i++){
        std::string publisher_topic = "mtg_agent_bringup_node/agent_" + std::to_string(i) + "/cmd_vel";
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>(publisher_topic, 1);
        vel_pub_list.push_back(vel_pub);
    }

    gamepad_subscriber = nh.subscribe<sensor_msgs::Joy>("joy", 10, &MtgTeleop::gamepad_callback, this);
    status_subscriber = nh.subscribe<mtg_messages::agents_status>("/mtg_agent_bringup_node/all_agent_status", 10, &MtgTeleop::status_msg_callback, this);
}

int get_sign(double x){
    return x>0 ? 1:-1;
}

void MtgTeleop::gamepad_callback(const sensor_msgs::Joy::ConstPtr& joy){

    if (active_agent_list.size()==0){
        return;
    }
    geometry_msgs::Twist twist;

    if(joy->buttons[agent_change_button_left]==1){
        current_agent = std::max(0, current_agent -1 );
        std::cout << "Agent changed to " << current_agent << std::endl;
    }
    if(joy->buttons[agent_change_button_right]==1){
        int size = active_agent_list.end() - active_agent_list.begin();
        current_agent = std::min(size - 1, current_agent +1 );

        std::cout << "Agent changed to " << current_agent << std::endl;
    }


    if(joy->buttons[velocity_inc_button]==1){
        const_linear_velocity = std::min(10.0, const_linear_velocity+0.2);
        const_angular_velocity = std::min(10.0, const_angular_velocity+0.2);
        std::cout << "Linear and angular velocity changed to " << const_linear_velocity << " " << const_angular_velocity << std::endl; 
    }

    if(joy->buttons[velocity_dec_button]==1){
        const_linear_velocity = std::max(0.2, const_linear_velocity-0.2);
        const_angular_velocity = std::max(0.2, const_angular_velocity-0.2);
        std::cout << "Linear and angular velocity changed to " << const_linear_velocity << " " << const_angular_velocity << std::endl; 
    }

    twist.linear.x = std::abs(joy->axes[linear_joy_axes]) > axes_threshold ? get_sign(joy->axes[linear_joy_axes]) * const_linear_velocity : 0;
    twist.angular.z = std::abs(joy->axes[angular_joy_axes]) > axes_threshold ? get_sign(joy->axes[angular_joy_axes]) * const_angular_velocity: 0;

    vel_pub_list[current_agent].publish(twist);


}

void MtgTeleop::status_msg_callback(const mtg_messages::agents_status::ConstPtr& status){
    active_agent_list.clear();
    int size = status->battery_lvl.size();
    for(int i=0;i<size; i++){
        if(status->status[i] == "ROBOT_STATUS_ACTIVE"){
            active_agent_list.push_back(active_agent(atoi(status->robot_id[i].c_str()), status->ip_adress[i]));
        }
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mtg_agent_bringup_teleop_node",
                            ros::init_options::AnonymousName);


  MtgTeleop mtg_teleop;

  ros::spin();

  return (0);
}