//==============================================================================
#ifndef EGOTRAJECTORYGENERATOR_H
#define EGOTRAJECTORYGENERATOR_H
//==============================================================================
#include <vector>
using namespace std;
//==============================================================================


class EgoTrajectoryGenerator
{
public:

  EgoTrajectoryGenerator(
    int num_trajectory_points,
    double time_step_size,
    double trajectory_length);


  void GenerateEgoTrajectory(
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
    vector <double> map_waypoints_s,
    vector <double> map_waypoints_x,
    vector <double> map_waypoints_y);

private:

  int num_trajectory_points_;

  double time_step_size_;

  double trajectory_length_;

};
//==============================================================================
#endif //EGOTRAJECTORYGENERATOR_H
//==============================================================================
