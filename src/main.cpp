#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

//--JD code
      int lane = 1;
      double ref_vel = 0.0;
      // end

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      

      auto s = hasData(data);

      if (s != "") {
        std::cout << "ref vel: " << ref_vel<<std::endl;
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          std::cout << "ref vel: " << ref_vel<< "Initial pose: " << car_x<<", "<<car_y<<std::endl;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          //size of the previous path
          int prev_size = previous_path_x.size();

          bool too_close = false;
          // avoid obstacles

          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          

          for(unsigned int i = 0; i < sensor_fusion.size(); i++){
            //car is in my lane
            float d = sensor_fusion[i][6];
            if(d < (2 + 4*lane+2) && d > (2 + 4*lane - 2) )
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size*0.02*check_speed);
              if((check_car_s > car_s) && ((check_car_s-car_s)<30))
              {
                //do something
                too_close = true;
                if(lane > 0)
                  lane = 0;
                else if (lane == 0)
                  lane = 1;
              }

            }
          }

          
          if(too_close){
            ref_vel -= 0.224;
          }
          else
          {
            if(ref_vel < 49)
              ref_vel += 0.224;
          }
          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          //Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          //these points will be interpolated by using splines
          vector<double> ptsx;
          vector<double> ptsy;


          // Current car position
          double current_x = car_x;
          double current_y = car_y;
          double current_yaw = deg2rad(car_yaw);

          // if the path size is still too small, we should take the current car position instead of the previous path
          // to generate a new spline
          if(prev_size < 2)
          {
            //go backwards in time based on cars position and angle
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
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
          // in frenet add evenly 30m spaced points ahead of the starting reference

          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i = 0; i < ptsx.size(); i++){
            //transform to local frame
            double shift_x = ptsx[i]-current_x;
            double shift_y = ptsy[i]-current_y;

            ptsx[i] = (shift_x * cos(0 - current_yaw) - shift_y*sin(0 - current_yaw));
            ptsy[i] = (shift_x * sin(0 - current_yaw) + shift_y*cos(0 - current_yaw));
          }

          //create spline
          tk::spline s;
          s.set_points(ptsx,ptsy);

          //define the points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //start with points from previous path
          for(int i = 0; i < previous_path_x.size();i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          //Fill up the rest of our path planner
          for(unsigned int i = 1; i<=50-previous_path_x.size(); i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //std::cout<<x_ref<< ", "<<y_ref << std::endl;

            // rotate back to global frame
            x_point = (x_ref*cos(current_yaw)-y_ref*sin(current_yaw));
            y_point = (x_ref*sin(current_yaw)+y_ref*cos(current_yaw));

            x_point += current_x;
            y_point += current_y;

            std::cout<<x_point<< ", "<<y_point << std::endl;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }
          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}