#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include "spline.h"

using std::string;
using std::vector;
//TODO: incorporate cost functions and functions from vehicle class to main

// Initializes Vehicle
Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float s, float d, float v, float a, float x, float y, float yaw, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy, string state)
{
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v = v;
    this->a = a;
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->state = state;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
}

Vehicle::~Vehicle() {}

trajectory Vehicle::choose_next_state(vector<vector<double>> &predictions)
{
    /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   * TODO: Your solution here.
   */

  //define mref_velocity from predictions
  too_close = false;
  std::cout<<"state: "<<state<<", ref_velocity: "<<ref_velocity<<", lane: "<<lane<<std::endl;
    int prev_size = previous_path_x.size();
    if (prev_size > 0)
    {
        s = end_path_s;
    }

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
        //car is in my lane
        float d = predictions[i][6];
        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
        {
            double vx = predictions[i][3];
            double vy = predictions[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = predictions[i][5];

            check_car_s += ((double)prev_size * 0.02 * check_speed);
            if ((check_car_s > s) && ((check_car_s - s) < 25))
            {
                //do something
                too_close = true;
            }
        }
    }
    if (too_close)
    {
        if(ref_velocity > 0.0)
            ref_velocity -= 0.224;
    }
    else
    {
        if (ref_velocity < 49.5)
            ref_velocity += 0.224;
    }

    vector<string> successor_states_vector = successor_states();
    vector<float> costs;
    vector<trajectory> paths;
    for (unsigned int i = 0; i < successor_states_vector.size(); ++i)
    {
        //calculate trajectory for each possible successor state
        trajectory path = generate_trajectory(successor_states_vector.at(i), predictions);
        paths.push_back(path);

        //calculate cost for each given trajectory
        costs.push_back(calculate_cost(*this, predictions, path));
    }
    int min_cost = 0;

    if (costs.size() > 0)
        min_cost = std::min_element(costs.begin(), costs.end()) - costs.begin();

    /**
   * TODO: Change return value here:
   */
    return paths.at(min_cost);
}

vector<string> Vehicle::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        states.push_back("LCL");
        states.push_back("LCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

trajectory Vehicle::generate_trajectory(string state,
                                        vector<vector<double>> &predictions)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    trajectory path;
    if (state.compare("CS") == 0)
    {
        path = constant_speed_trajectory();
    }
    else if (state.compare("KL") == 0)
    {
        path = keep_lane_trajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        path = lane_change_trajectory(state, predictions);
    }

    return path;
}

vector<vector<double>> Vehicle::generate_spline_trajectory(float meters_ahead, int lane_goal, float ref_vel)
{
    //size of the previous path
    int prev_size = previous_path_x.size();
    //Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    //these points will be interpolated by using splines
    vector<double> ptsx;
    vector<double> ptsy;
    // Current car position
    double current_x = x;
    double current_y = y;
    double current_yaw = yaw;
    // if the path size is still too small, we should take the current car position instead of the previous path
    // to generate a new spline
    if (prev_size < 2)
    {
        //go backwards in time based on cars position and angle
        double prev_car_x = x - cos(yaw);
        double prev_car_y = y - sin(yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(y);
    }
    else
    {
        //redifine current state from previous path end points
        current_x = previous_path_x[prev_size - 1];
        current_y = previous_path_y[prev_size - 1];

        double prev_x = previous_path_x[prev_size - 2];
        double prev_y = previous_path_y[prev_size - 2];
        current_yaw = atan2(current_y - prev_y, current_x - prev_x);

        ptsx.push_back(prev_x);
        ptsx.push_back(current_x);

        ptsy.push_back(prev_y);
        ptsy.push_back(current_y);
    }

    // in frenet add evenly Xm spaced points ahead of the starting reference

    vector<double> next_wp0 = getXY(s + meters_ahead, (2 + 4 * lane_goal), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(s + 2.0 * meters_ahead, (2 + 4 * lane_goal), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(s + 3.0 * meters_ahead, (2 + 4 * lane_goal), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        //transform to local frame
        double shift_x = ptsx[i] - current_x;
        double shift_y = ptsy[i] - current_y;

        ptsx[i] = (shift_x * cos(0 - current_yaw) - shift_y * sin(0 - current_yaw));
        ptsy[i] = (shift_x * sin(0 - current_yaw) + shift_y * cos(0 - current_yaw));
    }

    //create spline
    tk::spline s;
    s.set_points(ptsx, ptsy);

    //define the points we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //start with points from previous path
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    //calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = meters_ahead;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    //Fill up the rest of our path planner
    for (unsigned int i = 1; i <= 50 - previous_path_x.size(); i++)
    {
        double N = (target_dist / (0.02 * ref_vel / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //std::cout<<x_ref<< ", "<<y_ref << std::endl;

        // rotate back to global frame
        x_point = (x_ref * cos(current_yaw) - y_ref * sin(current_yaw));
        y_point = (x_ref * sin(current_yaw) + y_ref * cos(current_yaw));

        x_point += current_x;
        y_point += current_y;

        //std::cout << x_point << ", " << y_point << std::endl;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double>> next_path;
    next_path.push_back(next_x_vals);
    next_path.push_back(next_y_vals);

    return next_path;
}

trajectory Vehicle::constant_speed_trajectory()
{
    // Generate a constant speed trajectory.
    trajectory path;
    path.points = generate_spline_trajectory(30.0, lane, ref_velocity);
    path.state = "KL";
    return path;
}

trajectory Vehicle::keep_lane_trajectory(vector<vector<double>> &predictions)
{
    // Generate a constant speed trajectory.
    trajectory path;
    path.points = generate_spline_trajectory(30.0, lane, ref_velocity);
    path.state = "KL";
    return path;
}

trajectory Vehicle::lane_change_trajectory(string state,
                                           vector<vector<double>> &predictions)
{
    vector<trajectory> paths;
    float distance_ahead_step_size = 5.0f;
    float min_distance_ahead = 10.0f;
    int num_distance_steps = 5;
    int lane_goal = lane;
    if (state.compare("LCL") == 0)
    {
        lane_goal--;
    }
    else if (state.compare("LCR") == 0)
    {
        lane_goal++;
    }
    for (int i = 0; i < num_distance_steps; ++i)
    {
        trajectory path;
        path.points = generate_spline_trajectory(min_distance_ahead + (float)i * distance_ahead_step_size,
                                                 lane_goal, ref_velocity);
        path.state = state;
        paths.push_back(path);
    }
    // Generate a lane change trajectory based on the trajectory with the lower cost:
    vector<float> costs;
    for (unsigned int i = 0; i < paths.size(); ++i)
    {
        //calculate cost for each given trajectory
        costs.push_back(calculate_cost(*this, predictions, paths.at(i)));
    }
    int min_cost = 0;

    if (costs.size() > 0)
        min_cost = std::min_element(costs.begin(), costs.end()) - costs.begin();

    return paths.at(min_cost);
}

void Vehicle::update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> &previous_pathX, vector<double> &previous_pathY, double end_paths, double end_pathd, vector<vector<double>> &sensor_fusion)
{
    x = car_x;
    y = car_y;
    s = car_s;
    d = car_d;
    yaw = deg2rad(car_yaw);
    v = car_speed;
    previous_path_x = previous_pathX;
    previous_path_y = previous_pathY;
    end_path_s = end_paths;
    end_path_d = end_pathd;
}