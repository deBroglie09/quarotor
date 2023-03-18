#pragma once

#include "QuadrotorFeedbackController.h"
#include "QuadrotorAggressiveController.h"
#include <vector>
#include <fstream>
#include <iostream>

class OffboardWrapper{
    private:       
        ros::NodeHandle nh;
        geometry_msgs::PoseStamped start_position_setpoint_, end_position_setpoint_;
        ros::Time start_planning_t_, start_hover_t;

        // std::string current_status_;
        std::string uav_id;
        std::string dataset_address;
        int current_status_;
        
        //flag
        bool hover_flag;
        bool planning_flag;
        bool end_flag;
        bool ready_flag;

        void isAtSetpoint();
        void isAtEndpoint();
        void topicPublish();

    public:
        OffboardWrapper(geometry_msgs::PoseStamped position_setpoint, std::string id, std::string node_id, std::string dataset);
        ~OffboardWrapper();

        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient arming_client; 

        mavros_msgs::State wrapper_current_state_;

        struct SubsciberWrapper{
            ros::Subscriber wrapper_state_sub_;   
            ros::Subscriber wrapper_current_sub_;
            ros::Subscriber wrapper_vrpn_sub_;
            ros::Subscriber wrapper_velocity_sub_;
            ros::Subscriber wrapper_status_sub;
        }m_Subscriber;

        struct ffSubsciberWrapper
        {
            ros::Subscriber ff_position_sub_;
            ros::Subscriber ff_velocity_sub_;
            ros::Subscriber ff_attitude_sub_;
            ros::Subscriber ff_rate_sub_;
            ros::Subscriber ff_command_sub_;
        }f_Subscriber;        

        struct PublisherWrapper{
            ros::Publisher wrapper_local_pos_pub_;
            ros::Publisher wrapper_attitude_pub_;
            ros::Publisher wrapper_vision_pos_pub_;
            ros::Publisher wrapper_status_pub;  
                      
            ros::Publisher position_setpoint_pub;
            ros::Publisher velocity_setpoint_pub;
            ros::Publisher attitude_setpoint_pub;
            ros::Publisher attitude_cureuler_pub;
        }m_Publisher;

        struct DataCentre wrap_data;
        struct NNFeedforwardCmd ff_data;        
        
        void subscriber();
        void ffSubscriber();
        void run();
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        // void visualCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void visualCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void statusCallback(const std_msgs::Bool::ConstPtr& msg);
        void getEndPoint();

        void ffpositionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void ffvelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void ffattitudeCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void ffrateCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void ffcommandCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
};

