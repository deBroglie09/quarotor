#include "QuadrotorFeedbackController.h"

QuadrotorFeedbackController::QuadrotorFeedbackController(geometry_msgs::PoseStamped position_setpoint, struct DataCentre* wrap_data_ptr){

  data_ptr = wrap_data_ptr;
  position_setpoint_ = position_setpoint;

  current_status_ = "HOVER";

  // thrust eval
  eval_ptr_ = 0;

  // hover PID params init
  kp_hover_x_ = 1.05;
  kp_hover_y_ = 1.1;
  kp_hover_z_ = 2;
  kp_hover_vx_ = 0.3;
  kp_hover_vy_ = -0.3;
  kp_hover_vz_ = 0.5;

  ki_hover_x_ = 0;
  ki_hover_y_ = 0;
  ki_hover_z_ = 0.01;
  ki_hover_vx_ = 0;
  ki_hover_vy_ = 0;
  ki_hover_vz_ = 0.01;

  kd_hover_x_ = 0;
  kd_hover_y_ = 0;
  kd_hover_z_ = 0;
  kd_hover_vx_ = 0;
  kd_hover_vy_ = 0;
  kd_hover_vz_ = 0;
}

QuadrotorFeedbackController::~QuadrotorFeedbackController(){
}

void QuadrotorFeedbackController::positionControlFeedback(){
  Eigen::Vector3d position_cmd_(position_setpoint_.pose.position.x, position_setpoint_.pose.position.y, position_setpoint_.pose.position.z);
  Eigen::Vector3d position_error_ = position_cmd_ - current_position_;

  if(current_state_== OFFBOARD)
    position_error_sum_ += position_error_ / LOOP_FREQUENCY;
  else
    position_error_sum_ = Eigen::Vector3d(0,0,0);

  double position_i_z_ = ki_hover_z_ * position_error_sum_[2]; 
  if(position_i_z_ > 2)
    position_i_z_ = 2;
  if(position_i_z_ < -2)
    position_i_z_ = -2;

  velocity_setpoint_.twist.linear.x = kp_hover_x_ * position_error_[0];
  velocity_setpoint_.twist.linear.y = kp_hover_y_ * position_error_[1];
  velocity_setpoint_.twist.linear.z = kp_hover_z_ * position_error_[2] + position_i_z_;

  position_setpoint_.header.stamp = ros::Time::now();
  position_setpoint_.header.frame_id = "odom";
  velocity_setpoint_.header.stamp = ros::Time::now();
  velocity_setpoint_.header.frame_id = "odom";

  data_ptr->pub_setpoint_position_ = position_setpoint_;
  data_ptr->pub_setpoint_velocity_ = velocity_setpoint_;

  // std::cout << data_ptr->pub_setpoint_position_ << std::endl;
  
}

void QuadrotorFeedbackController::velocityControlFeedback(){
  Eigen::Vector3d velocity_cmd_(velocity_setpoint_.twist.linear.x, velocity_setpoint_.twist.linear.y, velocity_setpoint_.twist.linear.z);
  Eigen::Vector3d velocity_error_  = velocity_cmd_ - current_velocity_;

  double psi_ = current_attitude_[2];
  Eigen::Matrix3d R_E_B_;
  R_E_B_ << cos(psi_),sin(psi_),ZERO,
          -sin(psi_),cos(psi_),ZERO,
          ZERO,ZERO,ONE;
  velocity_error_ = R_E_B_ * velocity_error_;
  
  if(current_state_ == OFFBOARD)
    velocity_error_sum_ += velocity_error_ / LOOP_FREQUENCY;
  else
    velocity_error_sum_ = Eigen::Vector3d(0,0,0); 

  double velocity_i_y_ = ki_hover_vy_ * velocity_error_sum_[1]; 
  if(velocity_i_y_ > 0.4)
    velocity_i_y_ = 0.4;
  if(velocity_i_y_ < -0.4)
    velocity_i_y_ = -0.4; 

  double velocity_i_z_ = ki_hover_vz_ * velocity_error_sum_[2]; 
  if(velocity_i_z_ > 0.2)
    velocity_i_z_ = 0.2;
  if(velocity_i_z_ < -0.2)
    velocity_i_z_ = -0.2;

  double thrust_cmd_ = 0.68 + kp_hover_vz_ * velocity_error_[2] + velocity_i_z_;
  if(thrust_cmd_ >= 0.95)
    thrust_cmd_ = 0.95;
  if(thrust_cmd_ <= 0.01)
    thrust_cmd_ = 0;
  printf("thrust_cmd: %lf \n", thrust_cmd_);

  double theta_cmd_ = kp_hover_vx_ * velocity_error_[0];
  double phi_cmd_ = kp_hover_vy_ * velocity_error_[1] +  velocity_i_y_;
  double psi_cmd_ = 0;
  if(theta_cmd_ > 0.5)
    theta_cmd_ = 0.5;
  if(theta_cmd_ < -0.5)
    theta_cmd_ = -0.5;
  if(phi_cmd_ > 0.5)
    phi_cmd_ = 0.5;
  if(phi_cmd_ < -0.5)
    phi_cmd_ = -0.5;

  data_ptr->thrust_eval[eval_ptr_] = thrust_cmd_;
  if(eval_ptr_ == 4)
    eval_ptr_ = 0;
  else
    eval_ptr_ += 1;

  tf::Quaternion oq_;
  oq_.setRPY(phi_cmd_, theta_cmd_, psi_cmd_);
  thrust_attitude_cmd_.orientation.w = oq_.w();
  thrust_attitude_cmd_.orientation.x = oq_.x();
  thrust_attitude_cmd_.orientation.y = oq_.y();
  thrust_attitude_cmd_.orientation.z = oq_.z();
  thrust_attitude_cmd_.thrust = thrust_cmd_;
  // thrust_attitude_cmd.body_rate = Vector3d(0,0,0);
  thrust_attitude_cmd_.type_mask = 7;
  
  data_ptr->thrust_attitude_cmd_ = thrust_attitude_cmd_;

  data_ptr->pub_euler_attitude_.vector.x = current_attitude_[0]*180/PI;
  data_ptr->pub_euler_attitude_.vector.y = current_attitude_[1]*180/PI;
  data_ptr->pub_euler_attitude_.vector.z = current_attitude_[2]*180/PI;
  data_ptr->pub_euler_attitude_.header.stamp = ros::Time::now();

  data_ptr->pub_setpoint_attitude_.vector.x = phi_cmd_*180/PI;
  data_ptr->pub_setpoint_attitude_.vector.y = theta_cmd_*180/PI;
  data_ptr->pub_setpoint_attitude_.vector.z = psi_cmd_*180/PI;
  data_ptr->pub_setpoint_attitude_.header.stamp = ros::Time::now();


  std::cout << "phi_cmd: " << phi_cmd_*180/PI << std::endl; 
  std::cout << "theta_cmd: " << theta_cmd_*180/PI << std::endl; 
  std::cout << "psi_cmd: " << psi_cmd_*180/PI << std::endl;
}

void QuadrotorFeedbackController::loadLatestData(){
  current_position_ = data_ptr->wrapper_current_position_;
  current_velocity_ = data_ptr->wrapper_current_velocity_;
  current_attitude_ = data_ptr->wrapper_current_attitude_;
  current_state_ = data_ptr->current_state_;
}

void QuadrotorFeedbackController::get_params(const ros::NodeHandle &nh){
  
  read_yaml_param(nh,"/offb_node/PID/kp", kp_hover_x_);
  read_yaml_param(nh,"/offb_node/PID/ki", ki_hover_x_);
  read_yaml_param(nh,"/offb_node/PID/kd", kd_hover_x_);

  std::cout << "kp_x: " << kp_hover_x_ << std::endl;
  std::cout << "ki_x: " << ki_hover_x_ << std::endl;  
  std::cout << "kd_x: " << kd_hover_x_ << std::endl;
  std::cout << "kp_vx: " << kp_hover_vx_ << std::endl;
}
