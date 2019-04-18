#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);


float calculate_cost(const Vehicle &vehicle, 
                     const vector<vector<double>> &predictions, 
                     const trajectory &path) {

  float cost = 0.0;
  return cost;
}
