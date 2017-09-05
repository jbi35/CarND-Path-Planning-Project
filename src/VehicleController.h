//==============================================================================
#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H
//==============================================================================
#include <vector>
# include "EgoTrajectoryGenerator.h"
using namespace std;
//==============================================================================


class VehicleController
{
public:
  VehicleController();

  void UpdateState(
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
    vector <double> map_waypoints_y);

    // get new trajectory
  void ComputeNewTrajectory(
    vector<double> &ego_trajectory_x,
    vector<double> &ego_trajectory_y);

private:

  void DetermineOptimalLane();

private:

  // current target velocity
  double target_vel_;

  // lane to drive in
  int target_lane_;

  // trajectory generator for ego vehicle
  EgoTrajectoryGenerator trajectory_generator_;

  // state data
  vector<double> previous_path_x_;
  vector<double> previous_path_y_;
  double end_path_s_;
  double end_path_d_;
  double car_x_;
  double car_y_;
  double car_yaw_;
  double car_s_;
  vector <vector<double>> sensor_fusion_data_;
  vector <double> map_waypoints_s_;
  vector <double> map_waypoints_x_;
  vector <double> map_waypoints_y_;

  // declare all constants defining the vehicle behaviour here
  static const double SPEED_LIMIT;
  static const int    NUM_TRAJECTORY_POINTS;
  static const double TIME_STEP_SIZE;
  static const double TRAJECTORY_LENGTH;
  static const double SAFETY_DISTANCE;
  static const double EMERGENCY_BREAK_THRESHOLD;
  static const double SAFETY_DISTANCE_BACK;

};
//==============================================================================
#endif // VEHICLECONTROLLER_H
//==============================================================================
