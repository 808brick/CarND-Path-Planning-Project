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
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


////////////////////
//  Helper Functions
////////////////////

double mph_to_m_s(double mph_speed){
  return mph_speed * 0.44704;
}

double m_s_to_mph(double m_s_speed){
  return m_s_speed / 0.44704;
}



////////////////////
//  Variables
////////////////////

int max_lane_num = 2; // On this highway, there are lanes 3 lanes, [0,1,2]
double lane_width = 4.0; // Width of highway lane in meters

double speed_limit_mph = 50; // Speed limit in MPH
//double speed_limit = mph_to_m_s(speed_limit_mph); // Speed limit in m/s
double speed_limit = 50;
double speed_limit_buffer = 1; // Buffer to ensure we do not accidentally exceed speed limit
double desired_speed = speed_limit - speed_limit_buffer;
double max_speed_change = 3.2; // Do not accelerate too fast to try minimize jerk

double safe_lane_change_dist = 30; // Distance away in meters cars in other lane should be to safely change lane

double max_cost = 1;
double slow_down_default_cost = 0.5;
double speed_up_default_cost = 0.7;
double stay_in_lane_default_cost = 0.4;
double turning_min_cost = 0.25; // Default cost of turning, prioritize staying in lane unless there is a slow car in front


////////////////////
//  Cost Functions
////////////////////

double stay_in_lane_cost(double car_front_dist){
  double cost = max_cost;
  if (car_front_dist > safe_lane_change_dist*1.5){
    cost *= 0.2;
  }
  else { // Increase cost as vehicle in front gets closer
    double slope = 1.0 * (0.8 - 0) / (safe_lane_change_dist*1.5 - 0);
    double mapped_value = 0 + slope * (car_front_dist);
    cost *= 0.2 + (1 - mapped_value);
  }
  return cost;
}

double slow_down_cost(double car_front_dist, double current_speed){
  double cost = slow_down_default_cost;

  if (current_speed > speed_limit || car_front_dist < 15){
    cost = 0;
  }

  else if (car_front_dist < 20){
    cost *= 0.5;
  }
  else if (current_speed > desired_speed){
    cost *= 0.3;
  }
  return cost;
}

double turn_left_cost(bool car_on_left, double car_left_front_dist, int current_lane){
  double cost = max_cost;

  if (current_lane != 0 && car_on_left == false){

    cost -= car_left_front_dist / (2 * safe_lane_change_dist);
    // cost *= safe_lane_change_dist /2 / car_left_front_dist);

    // Ensure cost never goes below turning_min_cost
    if (cost < turning_min_cost){
      cost = turning_min_cost;
    }

  }

  return cost;


}

double turn_right_cost(bool car_on_right, double car_right_front_dist, int current_lane){
  double cost = max_cost;

  if (current_lane != max_lane_num && car_on_right == false){
    cost -= car_right_front_dist / (2 * safe_lane_change_dist);

    // Ensure cost never goes below turning_min_cost
    if (cost < turning_min_cost){
      cost = turning_min_cost;
    }
  }

  return cost;
}

////////////////////
//  Main
////////////////////

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
    double s;
    double d_x;
    double d_y;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

           ////////////////////
           //  Variables
           ////////////////////

           int prev_size = previous_path_x.size();

           double delta_t = 0.02;

           vector<double> x_path_points, y_path_points;

           double min_cost = 1;
           int decision = 0;
           // decision = 0 = Stay in same lane at max velocity
           // decision = 1 = Slow Down
           // decision = 2 = Turn Left
           // decision = 3 = Turn right

           int current_lane = car_d / lane_width; // 0 if in left lane, 1 if in middle lane, 2
           int desired_lane = current_lane; // Will change if decide to change lanes

           double speed_diff = 0;

           vector<double> cost_vector;
           double car_front_dist = 100;
           double car_left_front_dist = 100;
           double car_right_front_dist = 100;
           double car_front_speed = 100;
           bool car_on_left = false;
           bool car_on_right = false;


           ////////////////////
           //  Make Decision
           ////////////////////
           std::cout << std::endl;

           std::cout << "Current Lane: " << current_lane << std::endl;


           if (prev_size > 0){
             car_s = end_path_s;
           }

           for (int i=0; i < sensor_fusion.size(); i++){ // Loop through each car in the simulation
             auto car_i_data = sensor_fusion[i];
             double car_i_s = car_i_data[5];
             double car_i_d = car_i_data[6];
             
             double car_vel_x = car_i_data[3];
             double car_vel_y = car_i_data[4];
             
             double car_i_speed = sqrt(car_vel_x * car_vel_x + car_vel_y * car_vel_y ) * 2.24;
             
             int car_i_lane_num = car_i_d / lane_width;
             double dist_from_car_i = car_i_s - car_s;
             bool car_i_in_front = false;

             if (dist_from_car_i > 0){ // Car is in front of us
               car_i_in_front = true;
             }

             if (car_i_lane_num == current_lane && car_i_in_front && car_front_dist > dist_from_car_i){ // Check if car is in same lane, in front
               car_front_dist = dist_from_car_i;
               car_front_speed = car_i_speed;
               
             }

             if (car_i_lane_num == current_lane - 1){ // Car is in lane to left of us
               if (abs(dist_from_car_i) < safe_lane_change_dist){
                 car_on_left = true;
               }
               if (car_i_in_front && dist_from_car_i < car_left_front_dist) {
                 car_left_front_dist = dist_from_car_i;
               }
               if (car_i_d > car_i_lane_num * lane_width + lane_width * 0.75 && dist_from_car_i < car_front_dist){ // Car in left lane is trying to merge into our lane
                car_front_dist = dist_from_car_i;

               }
             }

             if (car_i_lane_num == current_lane + 1){ // Car is in lane to right of us
               if (abs(dist_from_car_i) < safe_lane_change_dist){
                 car_on_right = true;
               }
               if (car_i_in_front && dist_from_car_i < car_right_front_dist) {
                 car_right_front_dist = dist_from_car_i;
               }
               if (car_i_d < car_i_lane_num * lane_width + lane_width * 0.25 && dist_from_car_i < car_front_dist){ // Car in right lane is trying to merge into our lane
                car_front_dist = dist_from_car_i;

               }
             }

           }

           cost_vector.push_back(stay_in_lane_cost(car_front_dist));
           cost_vector.push_back(slow_down_cost(car_front_dist, car_speed));
           cost_vector.push_back(turn_left_cost(car_on_left, car_left_front_dist, current_lane));
           cost_vector.push_back(turn_right_cost(car_on_right, car_right_front_dist, current_lane));

           std::cout << std::endl;

           std::cout << "Car Front Dist: " << car_front_dist << std::endl;
          std::cout << "Car Front Speed: " << car_front_speed << std::endl;
           std::cout << "Car Front Left Dist: " << car_left_front_dist << std::endl;
           std::cout << "Car Front Right Dist: " << car_right_front_dist << std::endl;
           std::cout << "Car On Left: " << car_on_left << std::endl;
           std::cout << "Car On Right: " << car_on_right << std::endl;
           std::cout << "Cost Function: " << std::endl;
           std::cout << cost_vector[0] << std::endl;
           std::cout << cost_vector[1] << std::endl;
           std::cout << cost_vector[2] << std::endl;
           std::cout << cost_vector[3] << std::endl;
           std::cout << std::endl;

           double prev_decision = decision;

           for (int i = 0; i < cost_vector.size(); ++i) { // Sort through the min cost in the vector to make decision
             double i_cost = cost_vector[i];
             if (i_cost < min_cost) {
                min_cost = i_cost;
                decision = i;
              }
            }

            std::cout << "Decision: " << decision << std::endl;

           if (decision == 0) { // Continue straight in lane
             desired_lane = current_lane;
             if (car_speed < desired_speed - speed_limit_buffer && car_front_speed > car_speed + 2){ // Increase speed if not going at desired speed and car in front not going slow
               speed_diff += max_speed_change;
               if (speed_diff + car_speed > desired_speed){
                 speed_diff = desired_speed - car_speed;
               }

               if (prev_decision == 1){ // If suddenly switching from slowing down to speeding up, do not accelerate too fast
                 speed_diff /= 3;
               }
               std::cout << "Increasing Speed..." << std::endl;
             }
           }

           else if (decision == 1) { // Slow Down
              speed_diff -= max_speed_change;
              std::cout << "Decreasing Speed..." << std::endl;
              if (prev_decision == 0){ // If suddenly switching from speeding up to slowing down, do not deaccelerate too fast
                 speed_diff /= 3;
               }
           }

           else if (decision == 2) { // Turn Left
              desired_lane = current_lane - 1;
              std::cout << "Attempting Left Turn..." << std::endl;
           }

           else if (decision == 3) { // Turn Right
              desired_lane = current_lane + 1;
              std::cout << "Attempting Right Turn..." << std::endl;
           }


           ////////////////////
           //  Generate Path Based Off Decision
           ////////////////////

          vector<double> points_x;
          vector<double> points_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Check to see if enough previous points from a previous path are availble, if not generate points
          if (prev_size < 2) {

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              points_x.push_back(prev_car_x);
              points_y.push_back(car_y);

              points_x.push_back(car_x);
              points_y.push_back(prev_car_y);


          } else {
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              points_x.push_back(ref_x_prev);
              points_y.push_back(ref_y_prev);

              points_x.push_back(ref_x);
              points_y.push_back(ref_y);


          }

          // Create new waypoints based using future coordinates
          for (int j=0; j < 3; j++){
            vector<double> gen_next_point = getXY(car_s + safe_lane_change_dist * (1+j), 0.5 * lane_width + lane_width * desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            points_x.push_back(gen_next_point[0]);
            points_y.push_back(gen_next_point[1]);
          }

          // vector<double> gen_next_point_1 = getXY(car_s + safe_lane_change_dist, 0.5 * lane_width + lane_width * desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // vector<double> gen_next_point_2 = getXY(car_s + safe_lane_change_dist * 2, 0.5 * lane_width + lane_width * desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // vector<double> gen_next_point_3 = getXY(car_s + safe_lane_change_dist * 3, 0.5 * lane_width + lane_width * desired_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //
          // points_x.push_back(gen_next_point_1[0]);
          // points_y.push_back(gen_next_point_1[1]);
          //
          // points_x.push_back(gen_next_point_2[0]);
          // points_y.push_back(gen_next_point_2[1]);
          //
          // points_x.push_back(gen_next_point_3[0]);
          // points_y.push_back(gen_next_point_3[1]);

          // Shift the coordinate with respect to the car instead of the map
          for (int i = 0; i < points_x.size(); i++) {
              double shift_x = points_x[i] - ref_x;
              double shift_y = points_y[i] - ref_y;

              points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Utilize the cubic spline library to generate a smooth path to reduce jerk
          tk::spline spline_path;
          spline_path.set_points(points_x, points_y); // using 2 previous points and 3 furture points to initialize the spline

          // Create vectors to store next path points to be sent to the simulation
          vector<double> next_x_vals, next_y_vals;

          for (int i = 0; i < prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // Generate a path using the x_coordinates (relative to the car) up to 30 meters away
          double target_x = 30.0;
          double target_y = spline_path(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_counter = 0;

          double set_velocity = car_speed + speed_diff;
          
          int num_points_needed = 50-prev_size;

          for (int i = 1; i < num_points_needed; i++) {
              //double N = target_dist / (0.02 * set_velocity / 2.24); // divided into N segments
              double N = target_dist / (0.02 * (car_speed + (1-i/num_points_needed) * speed_diff) / 2.24);
              double x_point = x_counter + target_x / N;
              double y_point = spline_path(x_point); // To calculate distance y position .

              x_counter = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

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
