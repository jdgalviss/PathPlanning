#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle &vehicle, 
                     const vector<vector<double>> &predictions, 
                     const trajectory &path);

#endif  // COST_H