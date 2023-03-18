#pragma once

#include "QuadrotorFeedbackController.h"
#include <vector>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

struct NNFeedforwardCmd{
            Eigen::Vector3d position_cmd, velocity_cmd, attitude_cmd, rate_cmd;
            double thrust, lambda0, lambda1;
        };

class QuadrotorAggressiveController{
    private:
        int data_ptr_;
        int csv_size_;
        double csv_data_[200][18];

        // std::string current_state_;

        Eigen::Vector3d current_position_, current_velocity_, current_attitude_;
        Eigen::Vector3d position_error_sum_, velocity_error_sum_;
        mavros_msgs::AttitudeTarget thrust_attitude_cmd_;

        double kp_planning_x_, kp_planning_y_, kp_planning_z_, kp_planning_vx_, kp_planning_vy_, kp_planning_vz_;
        double ki_planning_x_, ki_planning_y_, ki_planning_z_, ki_planning_vx_, ki_planning_vy_, ki_planning_vz_;
        double kd_planning_x_, kd_planning_y_, kd_planning_z_, kd_planning_vx_, kd_planning_vy_, kd_planning_vz_;
        double kp_attitude_;

    public:
        struct DataCentre *data_ptr;
        struct NNFeedforwardCmd *ff_ptr;
        struct FeedforwardCmd{
            Eigen::Vector3d position_cmd, velocity_cmd, attitude_cmd, rate_cmd;
            double thrust;
            bool is_done;
        }ff_cmd;

        double current_time_;
        double thrust_ave_;

        void readCsvData(std::string dataset);
        void loadFeedforwardData();
        void loadLatestData();
        void aggressiveControl();
        QuadrotorAggressiveController(struct DataCentre* wrap_data_ptr, struct NNFeedforwardCmd* ff_cmd_ptr);
        ~QuadrotorAggressiveController();
};