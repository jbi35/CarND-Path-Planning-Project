#include<math.h>
#include "VehicleController.h"
#include "EgoTrajectoryGenerator.h"


// define some constants
const double VehicleController::SPEED_LIMIT           = 49.5;
const double VehicleController::OVERTAKE_THRESHOLD    = 45.0;
const int    VehicleController::NUM_TRAJECTORY_POINTS = 50;
const double VehicleController::TIME_STEP_SIZE        = 0.02;
const double VehicleController::TRAJECTORY_LENGTH     = 30.0;
const double VehicleController::SAFETY_DISTANCE       = 30.0;

VehicleController::VehicleController()
:trajectory_generator_(NUM_TRAJECTORY_POINTS,TIME_STEP_SIZE,TRAJECTORY_LENGTH),
target_vel_(0.0), // simulator starts with zero velocity
target_lane_(1) // simulator always start with lane 1
{}
// feed data to controller
void VehicleController::UpdateState(
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  double end_path_s,
  double end_path_d,
  double car_x,
  double car_y,
  double car_yaw,
  double car_s,
  vector <vector<double>> sensor_fusion_data,
  vector <double> map_waypoints_s,
  vector <double> map_waypoints_x,
  vector <double> map_waypoints_y)
  {
    previous_path_x_ = previous_path_x;
    previous_path_y_ = previous_path_y;
    end_path_s_ = end_path_s;
    end_path_d_ = end_path_d;
    car_x_ = car_x;
    car_y_ = car_y;
    car_yaw_ = car_yaw;
    car_s_ = car_s;
    sensor_fusion_data_ = sensor_fusion_data;
    map_waypoints_s_ = map_waypoints_s;
    map_waypoints_x_ = map_waypoints_x;
    map_waypoints_y_ = map_waypoints_y;
  }

  // get new trajectory
void VehicleController::ComputeNewTrajectory(
  vector<double> &ego_trajectory_x,
  vector<double> &ego_trajectory_y)
  {
    int previous_size = previous_path_x_.size();

    // TODO do really want this
    if(previous_size > 0)
    {
      car_s_ = end_path_s_;
    }

    // check for obstacles
    bool too_close = false;

    // go through sensor fusion data
    for(int i = 0; i< sensor_fusion_data_.size(); i++)
    {
      // check for cars that are in my lane
      float d = sensor_fusion_data_[i][6];
      if(d< (2+4*target_lane_+2) && d> (2+4*target_lane_-2))
      {
        double vx = sensor_fusion_data_[i][3];
        double vy = sensor_fusion_data_[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion_data_[i][5];
        // project cars position into the future
        check_car_s+=((double)previous_size*TIME_STEP_SIZE*check_speed);

        // add very simply check for collision
        if((check_car_s > car_s_) && (check_car_s-car_s_ < SAFETY_DISTANCE))
        {
          // take some action
          too_close = true;
        }
      }
    }

    if(too_close)
    {
      // slow down by roughly 5 m/s^2 (under max acceleration)
      target_vel_ -= 0.224;
    }
    else if (target_vel_ < SPEED_LIMIT)
    {
      target_vel_ += 0.224;
    }

    trajectory_generator_.GenerateEgoTrajectory(
      ego_trajectory_x,
      ego_trajectory_y,
      previous_path_x_,
      previous_path_y_,
      car_x_,
      car_y_,
      car_yaw_,
      car_s_,
      target_vel_,
      target_lane_,
      map_waypoints_s_,
      map_waypoints_x_,
      map_waypoints_y_);
  }
