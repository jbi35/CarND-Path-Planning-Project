#include <math.h>
#include "VehicleController.h"
#include "EgoTrajectoryGenerator.h"
#include <iostream>

// define some constants
const double VehicleController::SPEED_LIMIT               = 49.5;
const double VehicleController::OVERTAKE_THRESHOLD        = 45.0;
const int    VehicleController::NUM_TRAJECTORY_POINTS     = 50;
const double VehicleController::TIME_STEP_SIZE            = 0.02;
const double VehicleController::TRAJECTORY_LENGTH         = 50.0;
const double VehicleController::SAFETY_DISTANCE           = 50.0;
const double VehicleController::EMERGENCY_BREAK_THRESHOLD = 30.0;
const double VehicleController::SAFETY_DISTANCE_BACK      = -10.0;

//==============================================================================
VehicleController::VehicleController()
:trajectory_generator_(NUM_TRAJECTORY_POINTS,TIME_STEP_SIZE,TRAJECTORY_LENGTH),
target_vel_(0.0), // simulator starts with zero velocity
target_lane_(1) // simulator always start with lane 1
{}

//==============================================================================
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

//==============================================================================
void VehicleController::ComputeNewTrajectory(
  vector<double> &ego_trajectory_x,
  vector<double> &ego_trajectory_y)
  {
    DetermineOptimalLane();

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

//==============================================================================
void VehicleController::DetermineOptimalLane()
{
  // check to see if the car is too close to the car ahead of it,
  // and if the lanes to the left and right of it are clear and a lane change
  // makes sense

  bool need_to_slow_down = false;
  bool lane_change_left_possible = true;
  bool lane_change_right_possible = true;

  // make sure the car does never drive into wrong side of the highway
  if (target_lane_ - 1 < 0)
  {
    lane_change_left_possible  = false;
  }
  // make sure the car does not drive of the road
  else if (target_lane_ + 1 > 2)
  {
    lane_change_right_possible = false;
  }

  // loop over all cars in traffic
  double current_traffic_pace = SPEED_LIMIT;

  double distance_to_next_car_in_lane = 100;
  for (int i = 0; i < sensor_fusion_data_.size(); ++i)
  {

    double traffic_car_s = sensor_fusion_data_[i][5];
    double traffic_car_d = sensor_fusion_data_[i][6];
    double traffic_car_speed_x = sensor_fusion_data_[i][3];
    double traffic_car_speed_y = sensor_fusion_data_[i][4];

    double traffic_speed = sqrt(traffic_car_speed_x*traffic_car_speed_x
                                +traffic_car_speed_y*traffic_car_speed_y);
    double s_distance = traffic_car_s - car_s_;

    // to determine optimal cruising speed get speed of car in front of us
    if( traffic_car_d < (2+4*target_lane_+2)
        && traffic_car_d > (2+4*target_lane_-2)
        && s_distance < distance_to_next_car_in_lane
        && s_distance > 0.0)
    {
      distance_to_next_car_in_lane = s_distance;
      current_traffic_pace = traffic_speed;
    }
    // check if there is traffic in our lane and we are getting close
    if( traffic_car_d < (2+4*target_lane_+2)
        && traffic_car_d > (2+4*target_lane_-2)
        && s_distance > 0
        && s_distance < SAFETY_DISTANCE)
    {
      need_to_slow_down = true;
      current_traffic_pace = traffic_speed;
    }

    if( traffic_car_d > 4.0 * (target_lane_ + 1)
        && traffic_car_d < 4.0 * (target_lane_ + 2)
        && (s_distance < SAFETY_DISTANCE
        && s_distance > SAFETY_DISTANCE_BACK))
    {
      lane_change_right_possible = false;
    }

    if( traffic_car_d > 4.0 * (target_lane_ - 1)
        && traffic_car_d < 4.0 * target_lane_
        && (s_distance < SAFETY_DISTANCE
        && s_distance > SAFETY_DISTANCE_BACK) )
    {
      lane_change_left_possible = false;
    }
  }

  // only make a lane change if we cannot keep moving close to the speed limit
  // in our current lane
  if (need_to_slow_down)
  {
    if (lane_change_right_possible)
    {
      target_lane_ += 1;
    }
    else if (lane_change_left_possible)
    {
      target_lane_ -= 1;
    }
    else // lanne change is not safely possible, thus we need to slow down
    {
      // compute acceleration
      if(current_traffic_pace < target_vel_)
      {
        double acc;
        // if some car pulled infront of us initiate emergency break
        if(distance_to_next_car_in_lane < EMERGENCY_BREAK_THRESHOLD)
        {
          acc = -9.8;
        }
        // otherwise break according to the difference in speed to the car in front of us
        else
        {
          acc = max((current_traffic_pace-target_vel_)*0.44,-9.8);
        }
        target_vel_ += acc*TIME_STEP_SIZE;
      }
    }
  }
  //  if the road ahead is clear, speed up to the speed limit
  else if (target_vel_ < SPEED_LIMIT)
  {
    target_vel_ += 0.424;
  }
}
