#ifndef COST_H
#define COST_H

#include "vehicle.h"
#include <iostream>

using std::map;
using std::string;
using std::vector;


bool get_vehicle_ahead(const Vehicle vehicle, const vector<vector<double>> &predictions, 
                       const int lane, vector<double> &ahead_vehicle, const int trajectory_size);

bool get_vehicle_behind(const Vehicle vehicle, const vector<vector<double>> &predictions, 
                        const int lane, vector<double> &behind_vehicle, const int trajectory_size);

float lane_speed(const vector<vector<double>> &predictions, const int lane, 
                const int trajectory_size, const Vehicle vehicle);

float obstacle_cost(const Vehicle vehicle, const vector<vector<double>> &predictions, 
                    const trajectory &path);

float calculate_cost(const Vehicle &vehicle,
                     const vector<vector<double>> &predictions,
                     const trajectory &path);

float crash_cost(const Vehicle vehicle, const vector<vector<double>> &predictions, 
                 const trajectory &path);

float distance_to_front_cost(const Vehicle vehicle, const vector<vector<double>> &predictions, 
                             const trajectory &path);



#endif  // COST_H