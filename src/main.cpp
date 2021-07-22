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

using namespace std;

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
  
  // Add lane value
  int lane = 1;
  
  // Add target velocity
  double target_vel = 0.0; // in mph

  h.onMessage([&lane,&target_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          json msgJson;

          // Define list of x and y points for path planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Add size of previous path
          int prev_size = previous_path_x.size(); 
          
          // Connect waypoints to end of previous path
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          // Perception: Use sensor fusion data to check for other cars
          // The data format for each car is: [id, x, y, vx, vy, s, d]
          
          // Initialize checks
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          
          // Initialize variables
          double car_ahead_speed = 50;
          double car_left_speed = 50;
          double car_right_speed = 50;
          
          int check_car_lane = 5;
          
          // Target velocity in mph, other values are m or m/s
          // Target velocity is converted later in path planning section
          
          // Check for other cars nearby
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_car_s = sensor_fusion[i][5];
            double check_car_speed = sqrt(vx*vx + vy*vy);
                        
            // Check where car will be at a later timestep
            check_car_s += (double)prev_size * 0.02 * check_car_speed;
            
            // Check which lane the other car is in
            if (d > 0 && d < 4) {
              check_car_lane = 0;
            }
            else if (d > 4 && d < 8) {
              check_car_lane = 1;
            }
            else if (d > 8 && d < 12) {
              check_car_lane = 2;
            }
            
            // Check if car is in the same lane
            if (check_car_lane == lane) {
              
              // Check if car ahead is within 25 m
              if ((check_car_s > car_s) && (check_car_s - car_s < 25)) {
                car_ahead = true;
                car_ahead_speed = 2.24 * check_car_speed; // convert m/s to mph
                cout << "Car speed: " << car_speed << endl;
                cout << "Car ahead speed: " << car_ahead_speed << endl;
              }
            }
            
            // Check if left lane is clear
            if ((check_car_lane + 1 == lane) && (fabs(car_s - check_car_s) < 20)) {
              car_left = true; // false when lane is clear
              car_left_speed = 2.24 * check_car_speed; // convert m/s to mph
              cout << "Left car in lane: " << check_car_lane << endl;
              cout << "Car s: " << car_s << endl;
              cout << "Left car s: " << check_car_s << endl;
              cout << "Car speed: " << car_speed << endl;
              cout << "Left car speed: " << car_left_speed << endl;
            }
            
            // Check if right lane is clear
            if ((check_car_lane - 1 == lane) && (fabs(car_s - check_car_s) < 20)) {
              car_right = true; // false when lane is clear
              car_right_speed = 2.24 * check_car_speed; // convert m/s to mph
              cout << "Right car in lane: " << check_car_lane << endl;
              cout << "Car s: " << car_s << endl;
              cout << "Right car s: " << check_car_s << endl;
              cout << "Car speed: " << car_speed << endl;
              cout << "Right car speed: " << car_right_speed << endl;
            }
            
          }
          
          // Behavior planning
          
          // Max acceleration is 10 m/s^2, or changing velocity by 22.4 mph/s
          // Each timestep is 0.02 s, so max velocity change is 22.4 mph/s * 0.02 s
          // Max velocity change is 0.448 mph (for acceleration of 10 m/s^2) 
          
          // Set max speed and acceleration values
          double max_speed = 49.5; // mph
          double max_acc = 0.224; // approximately 5 m/s^2
          
          // Slow down if car ahead is too close
          if (car_ahead == true) {
            if (target_vel > car_ahead_speed * 0.85) { // buffer speed
              target_vel -= max_acc;
              cout << "Car ahead, slowing down" << endl;
            }
            // If left lane is clear, move left
            if (car_left == false && lane > 0) {
              lane--;
              cout << "Left lane clear, moving left" << endl;
            }
            // Otherwise, move right if right lane is clear
            else if (car_right == false && lane < 2) {
              lane++;
              cout << "Right lane clear, moving right" << endl;
            }
          }
          // If there is no car ahead, speed up to target velocity
          else if (target_vel < max_speed) {
            target_vel += max_acc;
            cout << "Lane clear, speeding up" << endl;
          }
          
          // Spline path planner from Project Q&A video
          
          // Create list of waypoints spaced further apart for spline
          vector<double> pts_x;
          vector<double> pts_y;
          
          // Reference x, y, and yaw values
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If previous size is close to empty, use car values
          if (prev_size < 2) {

            // Create points that would make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            // Add to pts_x and pts_y lists
            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);
            
            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
          }
          
          else {
            
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            
            // Angle of rotation towards end of previous path
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // Add to pts_x and pts_y lists
            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);
            
            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }
          
          // Create waypoints spaced further apart
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // Add values to pts_x and pts_y lists
          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          
          // Shift car's reference angle to zero degrees (from map frame to car's frame)
          for (int i = 0; i < pts_x.size(); i++) {
            // Difference between waypoints and end of previous path
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            
            // Translate and rotate waypoints to origin of the car's coordinate system
            pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          
          // Generate spline
          tk::spline s;
          
          // Add list of waypoints to spline
          s.set_points(pts_x, pts_y);
          
          // Separate path list using previous points
          for (int i = 0; i < previous_path_x.size(); i++) {  
            next_x_vals.push_back(previous_path_x[i]);  
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Set up distance between spline points to match target velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          
          double x_add_on = 0.0;
          
          // Fill up the remaining points for path planning list
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            double N = target_dist / (0.02 * target_vel / 2.24); // velocity converted to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate waypoints back to normal
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            // Add to path planning list
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          // End of path planning

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