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

// Calculate vehicle ahead of my vehicle (passed by reference)
bool get_vehicle_ahead(const Vehicle vehicle, const vector<vector<double>> &predictions, const int lane, vector<double> &ahead_vehicle, const int trajectory_size)
{
    float min_s = 100000.0;
    bool found_vehicle = false;
    vector<double> temp_vehicle;
    // Check all data from sensor fusion
    for (int i = 0; i < predictions.size(); ++i)
    {
        // calculate vehicles position
        temp_vehicle = predictions.at(i);
        double vx_front = temp_vehicle[3];
        double vy_front = temp_vehicle[4];
        double check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
        double check_car_s_front = temp_vehicle[5] + 0.02 * check_speed_front * trajectory_size;
        int lane_prediction = floor(temp_vehicle[6] / 4.0);
        // Calculate the closest vehicle in front of my vehicle (same lane)
        if (lane_prediction == lane && vehicle.s < check_car_s_front && check_car_s_front < min_s)
        {
            min_s = check_car_s_front;
            ahead_vehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

// Calculate vehicle behind my vehicle (passed by reference)
bool get_vehicle_behind(const Vehicle vehicle, const vector<vector<double>> &predictions, const int lane, vector<double> &behind_vehicle, const int trajectory_size)
{
    float max_s = 0.0;
    bool found_vehicle = false;
    vector<double> temp_vehicle;
    for (int i = 0; i < predictions.size(); ++i)
    {
        temp_vehicle = predictions.at(i);
        double vx_back = temp_vehicle[3];
        double vy_back = temp_vehicle[4];
        double check_speed_back = sqrt(vx_back * vx_back + vy_back * vy_back);
        double check_car_s_back = temp_vehicle[5] + 0.02 * check_speed_back * trajectory_size;
        int lane_prediction = floor(temp_vehicle[6] / 4.0);
        if (lane_prediction == lane && vehicle.s > check_car_s_back && check_car_s_back > max_s)
        {
            max_s = check_car_s_back;
            behind_vehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

//Calculate speed of the given lane based on the next vehicle within the vision range on the given lane
float lane_speed(const vector<vector<double>> &predictions, const int lane,
                 const int trajectory_size, const Vehicle vehicle)
{
    //lane_speed is set to max (50mph = 22.352m/s) if no vehicle is found in front on the given lane
    float lane_speed = 22.352f;
    vector<double> front_vehicle;
    bool vehicle_found = get_vehicle_ahead(vehicle, predictions, lane, front_vehicle, trajectory_size);
    if (vehicle_found)
    {
        //check vehicle's position and speed
        double vx_front = front_vehicle[3];
        double vy_front = front_vehicle[4];
        double check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
        double check_car_s_front = front_vehicle[5] + 0.02 * check_speed_front * trajectory_size;
        //the lane_speed corresponds to the speed of the vehicle in front
        lane_speed = check_speed_front;
        //check that the vehicle is within the vision range we've defined (50m). If out of range, assume max speed
        double my_car_s = vehicle.s;
        if ((check_car_s_front < my_car_s) || ((check_car_s_front - my_car_s) > 50))
        {
            lane_speed = 22.352f;
        }
    }
    return lane_speed;
}

//cost function given by the distance to obstacles
//(used to define which lane change trajectory avoids in a more appropiate way, the vehicles that our car might crash with)
float obstacle_cost(const Vehicle vehicle, const vector<vector<double>> &predictions, const trajectory &path)
{

    // calculate current lane and goal lane
    int current_lane = floor(vehicle.d / 4.0);
    float goal_lane = (float)path.goal_lane;
    // calculate vehicle in front same lane, back goal lane and front goal lane
    vector<double> front_vehicle;
    vector<double> back_vehicle;
    vector<double> front_vehicle2;
    bool found_front_vehicle = get_vehicle_ahead(vehicle, predictions, current_lane, front_vehicle, path.points.at(0).size());
    bool found_rear_vehicle = get_vehicle_behind(vehicle, predictions, goal_lane, back_vehicle, path.points.at(0).size());
    bool found_front_vehicle2 = get_vehicle_ahead(vehicle, predictions, goal_lane, front_vehicle2, path.points.at(0).size());

    //Calculate these 3 vehicles position and speed
    double vx_front = 0;
    double vy_front = 0;
    double check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
    double check_car_s_front = vehicle.s + 75;
    double check_car_d_front = vehicle.d;
    if (found_front_vehicle)
    {
        vx_front = front_vehicle[3];
        vy_front = front_vehicle[4];
        check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
        check_car_s_front = front_vehicle[5];
        check_car_d_front = front_vehicle[6];
    }

    double vx_back = 0.0;
    double vy_back = 0.0;
    double check_speed_back = sqrt(vx_back * vx_back + vy_back * vy_back);
    double check_car_s_back = vehicle.s - 75;
    double check_car_d_back = goal_lane * 4.0f + 2.0f;
    if (found_rear_vehicle)
    {
        vx_back = back_vehicle[3];
        vy_back = back_vehicle[4];
        check_speed_back = sqrt(vx_back * vx_back + vy_back * vy_back);
        check_car_s_back = back_vehicle[5];
        check_car_d_back = back_vehicle[6];
    }

    double vx_front2 = 0;
    double vy_front2 = 0;
    double check_speed_front2 = sqrt(vx_front2 * vx_front2 + vy_front2 * vy_front2);
    double check_car_s_front2 = vehicle.s + 75;
    double check_car_d_front2 = goal_lane * 4.0f + 2.0f;
    if (found_front_vehicle2)
    {
        vx_front2 = front_vehicle2[3];
        vy_front2 = front_vehicle2[4];
        check_speed_front2 = sqrt(vx_front2 * vx_front2 + vy_front2 * vy_front2);
        check_car_s_front2 = front_vehicle2[5];
        check_car_d_front2 = front_vehicle2[6];
    }

    double cost = 0.0;

    //calculate distance to wach point of the trajectory and add, so we can determine which trajectory reduces the distance to obstacles
    for (int i = 0; i < path.points.at(0).size(); ++i)
    {
        //std::cout<<i<<std::endl;
        //get 'current' point from trajectory
        double path_x = path.points.at(0).at(i);
        double path_y = path.points.at(1).at(i);
        //get future x y of the cars:
        check_car_s_front += 0.02 * check_speed_front;
        check_car_s_back += 0.02 * check_speed_back;
        //calculate distance
        vector<double> front_current = getXY(check_car_s_front, check_car_d_front, vehicle.map_waypoints_s, vehicle.map_waypoints_x, vehicle.map_waypoints_y);
        vector<double> back_current = getXY(check_car_s_back, check_car_d_back, vehicle.map_waypoints_s, vehicle.map_waypoints_x, vehicle.map_waypoints_y);
        double distance_front = (pow(path_x - front_current.at(0), 2.0) + pow(path_y - front_current.at(1), 2.0));
        double distance_back = (pow(path_x - back_current.at(0), 2.0) + pow(path_y - back_current.at(1), 2.0));
        cost += distance_front + distance_back;
    }
    return cost;
}

//Cost function associated with the distance of the vehicle to vehicles in the goal lane (it defines when it is safe to perform a lane change)
float crash_cost(const Vehicle vehicle, const vector<vector<double>> &predictions, const trajectory &path)
{

    // calculate current lane and goal lane
    int current_lane = floor(vehicle.d / 4.0);
    float goal_lane = (float)path.goal_lane;
    // calculate vehicle in front and back on the goal lane
    vector<double> front_vehicle;
    vector<double> back_vehicle;
    vector<double> front_vehicle2;
    bool found_front_vehicle = get_vehicle_ahead(vehicle, predictions, current_lane, front_vehicle, path.points.at(0).size());
    bool found_rear_vehicle = get_vehicle_behind(vehicle, predictions, goal_lane, back_vehicle, path.points.at(0).size());
    bool found_front_vehicle2 = get_vehicle_ahead(vehicle, predictions, goal_lane, front_vehicle2, path.points.at(0).size());

    //Calculate these 3 vehicles position and speed
    double vx_front = 0;
    double vy_front = 0;
    double check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
    double check_car_s_front = vehicle.s + 75;
    double check_car_d_front = vehicle.d;
    if (found_front_vehicle)
    {
        vx_front = front_vehicle[3];
        vy_front = front_vehicle[4];
        check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
        check_car_s_front = front_vehicle[5] + 0.02 * check_speed_front * path.points.at(0).size();
        check_car_d_front = front_vehicle[6];
    }

    double vx_back = 0.0;
    double vy_back = 0.0;
    double check_speed_back = sqrt(vx_back * vx_back + vy_back * vy_back);
    double check_car_s_back = vehicle.s - 75;
    double check_car_d_back = goal_lane * 4.0f + 2.0f;
    if (found_rear_vehicle)
    {
        vx_back = back_vehicle[3];
        vy_back = back_vehicle[4];
        check_speed_back = sqrt(vx_back * vx_back + vy_back * vy_back);
        check_car_s_back = back_vehicle[5] + 0.02 * check_speed_back * path.points.at(0).size();
        check_car_d_back = back_vehicle[6];
    }

    double vx_front2 = 0;
    double vy_front2 = 0;
    double check_speed_front2 = sqrt(vx_front2 * vx_front2 + vy_front2 * vy_front2);
    double check_car_s_front2 = vehicle.s + 75;
    double check_car_d_front2 = goal_lane * 4.0f + 2.0f;
    if (found_front_vehicle2)
    {
        vx_front2 = front_vehicle2[3];
        vy_front2 = front_vehicle2[4];
        check_speed_front2 = sqrt(vx_front2 * vx_front2 + vy_front2 * vy_front2);
        check_car_s_front2 = front_vehicle2[5] + 0.02 * check_speed_front2 * path.points.at(0).size();
        ;
        check_car_d_front2 = front_vehicle2[6];
    }

    double cost = 0.0;
    //calculate distance to objects
    double distance_front = sqrt(pow(vehicle.s - check_car_s_front, 2.0));
    double distance_back = sqrt(pow(vehicle.s - check_car_s_back, 2.0));
    double distance_front2 = sqrt(pow(vehicle.s - check_car_s_front2, 2.0));

    //calculate cost function only for lane changing states
    float min_dist = 75.0;
    if (distance_back < min_dist && distance_back < 15) //take vehicle in the back into account only if too cloes
    {
        min_dist = distance_back;
    }

    if (distance_front2 < min_dist && distance_front2 < 10)
    {
        min_dist = distance_front2;
    }
    //std::cout << "state: " << path.state << " - min_distance_back: " << distance_back << std::endl;
    if (path.state.compare("KL") != 0)
        cost = 75.0 / (min_dist)-1.0;
    else
    {
        cost = 0.0;
    }
    cost = cost >= 0.0 ? cost : 0.0;
    return cost;
}

//Calculte cost depending on the distance to the vehicle closest to the front of each goal lane (measures free space in goal lane)
float distance_to_front_cost(const Vehicle vehicle, const vector<vector<double>> &predictions, const trajectory &path)
{
    // calculate current lane and goal lane
    int current_lane = floor(vehicle.d / 4.0);
    float goal_lane = (float)path.goal_lane;
    // calculate vehicle in front on the goal lane
    vector<double> front_vehicle;
    bool found_front_vehicle = get_vehicle_ahead(vehicle, predictions, goal_lane, front_vehicle, path.points.at(0).size());
    //calculate speed and position of the vehicle in front
    double vx_front = 0;
    double vy_front = 0;
    double check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
    double check_car_s_front = vehicle.s + 75;
    double check_car_d_front = vehicle.d;

    if (found_front_vehicle)
    {
        vx_front = front_vehicle[3];
        vy_front = front_vehicle[4];
        check_speed_front = sqrt(vx_front * vx_front + vy_front * vy_front);
        check_car_s_front = front_vehicle[5] + 0.02 * check_speed_front * path.points.at(0).size();
        check_car_d_front = front_vehicle[6];
    }
    //calculate cost depending on free space on the goal lane
    double cost = 0.0;
    double distance_front = sqrt(pow(vehicle.s - check_car_s_front, 2.0));
    cost = 75.0 / distance_front - 1.0f;
    cost = cost >= 0.0 ? cost : 0.0;
    return cost;
}

//calculate weighted total cost
float calculate_cost(const Vehicle &vehicle,
                     const vector<vector<double>> &predictions,
                     const trajectory &path)
{
    int goal_lane = path.goal_lane;
    int trajectory_size = path.points.at(0).size();

    //calculate lane_speed cost
    float lane_speed_cost = 1.0 - (lane_speed(predictions, path.goal_lane, trajectory_size, vehicle)) / 22.352f;

    //calculate crash cost
    float crash = crash_cost(vehicle, predictions, path);

    //Associate cost to not being in the center lane
    float center_cost = (path.goal_lane == 1) ? 0.0 : 0.1;

    //calculate cost associated with free space on goal lane
    float distance_front_cost = distance_to_front_cost(vehicle, predictions, path);

    //cost of being in the middle lane, so the vehicle doesnt state on the lane for too long
    float on_lane_cost = 0.0;
    if(((vehicle.d >3.25 && vehicle.d <4.75) || (vehicle.d >7.75 && vehicle.d <8.25)) && (path.state.compare("KL") == 0) && vehicle.state.compare("KL") != 0 )
        on_lane_cost = 20.0;

    //total cost
    return lane_speed_cost + distance_front_cost * 0.1 + crash * 3.0 + center_cost + on_lane_cost;
}