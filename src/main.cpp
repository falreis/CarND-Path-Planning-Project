#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"
#include "path_planing.h"

using namespace std;
using namespace help;
using namespace plan;

// for convenience
using json = nlohmann::json;

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
	//string map_file_ = "/home/falreis/Me/self_driving_car/term3/CarND-Path-Planning-Project/data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
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

	//State machine that controls the vehicle
	PathPlaning control;

	h.onMessage([&map_waypoints_x,
				&map_waypoints_y,
				&map_waypoints_s,
				&map_waypoints_dx,
				&map_waypoints_dy,
				&control]
		(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
	{
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
		auto s = hasData(data);
		if (s != "") {
        	auto j = json::parse(s);
        	string event = j[0].get<string>();
        	if (event == "telemetry") {
				// j[1] is the data JSON object

        		// Main car's localization Data
				Car my_car;
				my_car.x = j[1]["x"];
				my_car.y = j[1]["y"];
				my_car.s = j[1]["s"];
				my_car.d = j[1]["d"];
				my_car.yaw = j[1]["yaw"];
				my_car.speed = j[1]["speed"];

				// Previous path data given to the Planner
				auto previous_path_x = j[1]["previous_path_x"];
				auto previous_path_y = j[1]["previous_path_y"];

				// Previous path's end s and d values
				double end_path_s = j[1]["end_path_s"];
				double end_path_d = j[1]["end_path_d"];

				// Sensor Fusion Data, a list of all other cars on the same side of the road.
				auto sensor_fusion = j[1]["sensor_fusion"];

				int prev_size = previous_path_x.size();
				
				if(prev_size == 0){
					if(end_path_s > 0){
						my_car.s = end_path_s;
					}
					control.start();
				}
				
				control.setPosition(my_car);
				control.checkLane(sensor_fusion, prev_size);
				control.doActions();

				//list of widely spaced (x,y) waypoints
				vector<double> ptsx, ptsy;

				//reference x, y and yaw states
				double ref_x = my_car.x;
				double ref_y = my_car.y;
				double ref_yaw = deg2rad(my_car.yaw);

				//IF without values, add some information, ELSE use previous waypoints
				if(prev_size < 2){
					ptsx.push_back(my_car.x - cos(my_car.yaw)); //previous car_x point
					ptsx.push_back(my_car.x);

					ptsy.push_back(my_car.y - sin(my_car.yaw)); //previous car_y point
					ptsy.push_back(my_car.y);
				}
				else{
					//add x points
					ref_x = previous_path_x[prev_size - 1];
					double prev_ref_x = previous_path_x[prev_size - 2];

					ptsx.push_back(prev_ref_x);
					ptsx.push_back(ref_x);

					//add y points
					ref_y = previous_path_y[prev_size - 1];
					double prev_ref_y = previous_path_y[prev_size - 2];

					ptsy.push_back(prev_ref_y);
					ptsy.push_back(ref_y);

					//yaw
					ref_yaw = atan2((ref_y - prev_ref_y), (ref_x - prev_ref_x));
				}

				//points ahead starting reference
				vector<double> next_wp0 = getXY((my_car.s + 1*PathPlaning::HORIZON), 2 + (4*control.getLane()), map_waypoints_s, map_waypoints_x, map_waypoints_y);
				vector<double> next_wp1 = getXY((my_car.s + 2*PathPlaning::HORIZON), 2 + (4*control.getLane()), map_waypoints_s, map_waypoints_x, map_waypoints_y);
				vector<double> next_wp2 = getXY((my_car.s + 3*PathPlaning::HORIZON), 2 + (4*control.getLane()), map_waypoints_s, map_waypoints_x, map_waypoints_y);

				ptsx.push_back(next_wp0[0]);
				ptsx.push_back(next_wp1[0]);
				ptsx.push_back(next_wp2[0]);

				ptsy.push_back(next_wp0[1]);
				ptsy.push_back(next_wp1[1]);
				ptsy.push_back(next_wp2[1]);

				for(int i=0; i<ptsx.size(); i++){
					double shift_x = ptsx[i] - ref_x;
					double shift_y = ptsy[i] - ref_y;

					ptsx[i] = ((shift_x * cos(0-ref_yaw)) - (shift_y * sin(0-ref_yaw)));
					ptsy[i] = ((shift_x * sin(0-ref_yaw)) + (shift_y * cos(0-ref_yaw)));
				}

				//create spline, set (x,y) points and define points to the planner
				tk::spline spl;
				spl.set_points(ptsx, ptsy);

				vector<double> next_x_vals;
				vector<double> next_y_vals;

				for(int i=0; i< previous_path_x.size(); i++){
					next_x_vals.push_back(previous_path_x[i]);
					next_y_vals.push_back(previous_path_y[i]);
				}

				//use spline to travel at our desired reference velocity
				double target_x = PathPlaning::HORIZON;
				double target_y = spl(target_x);
				double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));

				double x_add_on = 0;	//to use in local transformation - starts with 0

				//fill the rest of the path planner
				for(int i=1; i<= (PathPlaning::HORIZON-previous_path_x.size()); i++){
					double N = (target_dist / (.02 * control.getVelocity() / 2.24)); //2.24 is meters/sec
					double x_point = x_add_on + (target_x / N);
					double y_point = spl(x_point);

					x_add_on = x_point;

					double x_ref = x_point;
					double y_ref = y_point;

					//rotate back from previous rotation (to global coordinates)
					x_point = ((x_ref*cos(ref_yaw)) - (y_ref*sin(ref_yaw)));
					y_point = ((x_ref*sin(ref_yaw)) + (y_ref*cos(ref_yaw)));

					x_point += ref_x;
					y_point += ref_y;

					next_x_vals.push_back(x_point);
					next_y_vals.push_back(y_point);
				}

				json msgJson;
				msgJson["next_x"] = next_x_vals;
				msgJson["next_y"] = next_y_vals;

				auto msg = "42[\"control\","+ msgJson.dump()+"]";

				//this_thread::sleep_for(chrono::milliseconds(100));
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        	}
		}
		else {
			// Manual driving
			std::string msg = "42[\"manual\",{}]";
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
