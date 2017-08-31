#include "EgoTrajectoryGenerator.h"
#include <math.h>
#include "utils.h"
#include "spline.h"
#include <iostream>

using namespace std;


//==============================================================================

EgoTrajectoryGenerator::EgoTrajectoryGenerator(
  int num_trajectory_points,
  double time_step_size,
  double trajectory_length)
{
  num_trajectory_points_ = num_trajectory_points;
  time_step_size_ = time_step_size;
  trajectory_length_ = trajectory_length;
}

void EgoTrajectoryGenerator::GenerateEgoTrajectory(
  vector<double> &ego_trajectory_x,
  vector<double> &ego_trajectory_y,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  double car_x,
  double car_y,
  double car_yaw,
  double car_s,
  double target_vel,
  int lane,
  vector<double> map_waypoints_s,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y)
{
  int previous_size = previous_path_x.size();

  vector<double> ptsx;
  vector<double> ptsy;

  // store reference state
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // if previous path does not contain eneough points
  if(previous_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // use previous path
  else
  {
    ref_x = previous_path_x[previous_size-1];
    ref_y = previous_path_y[previous_size-1];

    double ref_x_prev = previous_path_x[previous_size-2];
    double ref_y_prev = previous_path_y[previous_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  // add some points way down the road
  vector<double> next_wp0 = getXY(car_s+trajectory_length_,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+trajectory_length_+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+trajectory_length_+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // transformation to cars local cosy
  for(int i = 0;i<ptsx.size(); i++)
  {
    double shift_x =  ptsx[i] - ref_x;
    double shift_y =  ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y* sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y* cos(0-ref_yaw));

  }

  // create spline
  tk::spline s;
  s.set_points(ptsx,ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // use points from previous path first
  for(int i = 0; i< previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = trajectory_length_;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

  double x_add_on = 0;

  for(int i = 1; i<=num_trajectory_points_-previous_path_x.size();i++)
  {

    double N = (target_dist/(time_step_size_*target_vel/2.24));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    // undo cosy transformation
    x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

    ego_trajectory_x = next_x_vals;
    ego_trajectory_y = next_y_vals;
  }
}
