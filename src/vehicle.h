#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "helpers.h"

using std::map;
using std::string;
using std::vector;


struct trajectory{
    vector<vector<double>> points;
    string  state;
};

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float d, float v, float a, float x, float y, float yaw, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  trajectory choose_next_state( vector<vector<double>> &predictions);

  vector<string> successor_states();

  trajectory  generate_trajectory(string state, 
                                      vector<vector<double>> &predictions);

  trajectory constant_speed_trajectory();

  trajectory keep_lane_trajectory(vector<vector<double>> &predictions);

  trajectory  lane_change_trajectory(string state, 
                                         vector<vector<double>> &predictions);


  void update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> &previous_pathX, vector<double> &previous_pathY, double end_paths, double end_pathd, vector<vector<double>> &sensor_fusion);

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, goal_lane, lanes_available;

  string state;

  //changes from here

  private:

    vector<vector<double>> generate_spline_trajectory(float meters_ahead, int lane_goal, float ref_vel);
    

    double v, target_speed, a, max_acceleration, s, d, goal_s, goal_d, x, y, yaw, ref_velocity, end_path_s, end_path_d;

    bool too_close = false;

    vector<double> previous_path_x;

    vector<double> previous_path_y;

    //map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

};

#endif  // VEHICLE_H

