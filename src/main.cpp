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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
  const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double avg_lane_speed(vector<double> v) {
  // no car in this lane - speed is max
  if (v.empty()) return 49;
  return accumulate(v.begin(), v.end(), 0.0)/v.size(); 
}

double inefficiency_cost(double target_speed, int intended_lane, int final_lane, vector<double> lane_speeds) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
    */
    
    double intended_lane_speed = lane_speeds[intended_lane];
    double final_lane_speed = lane_speeds[final_lane];
    int max_index = distance(lane_speeds.begin(), max_element(lane_speeds.begin(), lane_speeds.end()));
    double delta_d = abs(final_lane - max_index) + abs(intended_lane - max_index);
    //TODO: Replace cost = 0 with an appropriate cost function.
    double cost = 1-exp(-2*target_speed / (float)(lane_speeds[intended_lane] + lane_speeds[final_lane]));
    
    return cost;
}

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

  // start in lane 1 (middle lane)
  int lane = 1;

  // reference velocity
  double ref_vel = 0; // mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();
            bool too_close = false;
            bool left_lane_safe = true;
            bool right_lane_safe = true;
            double target_speed = 49;

            // use sensor fusion
            // to determine behavior
            vector<double> left_lane_speeds, mid_lane_speeds, right_lane_speeds;
            for (int i = 0; i < sensor_fusion.size(); i++) {
              // record lane speed for each lane
              float vx = sensor_fusion[i][3];
              float vy = sensor_fusion[i][4];
              float obj_speed = sqrt(vx*vx+vy*vy);
              float d = sensor_fusion[i][6];
              float obj_s = sensor_fusion[i][5];
              float future_s;
              // only record speeds for cars in front
              if (obj_s > car_s) {
                if (d <= 4) left_lane_speeds.push_back(obj_speed);
                if (d > 4 && d <= 8) mid_lane_speeds.push_back(obj_speed);
                if (d > 8) right_lane_speeds.push_back(obj_speed);
              }
              // if car is in my lane
              if (d > 4*lane && d < 4*lane+4) {
                future_s = obj_s + prev_size*.02*obj_speed;
                // if car in front and future distance too close
                if (obj_s > car_s && future_s - car_s < 50) {
                  too_close = true;
                  cout << "TOO CLOSE!" << endl;
                }
              } 
              // car not in left lane and check left safe
              if (lane != 0 && d > 4*(lane-1) && d < 4*(lane-1)+4) {
                if (-5 < obj_s-car_s && obj_s-car_s < 20) left_lane_safe = false;
              }
              // car not in right lane and check right safe
              if (lane != 2 && d > 4*(lane+1) && d < 4*(lane+1)+4) {
                if (-5 < obj_s-car_s && obj_s-car_s < 20) right_lane_safe = false;
              }
            }
            vector<double> lane_speeds;
            lane_speeds.push_back(avg_lane_speed(left_lane_speeds));
            lane_speeds.push_back(avg_lane_speed(mid_lane_speeds));
            lane_speeds.push_back(avg_lane_speed(right_lane_speeds));

            if (!left_lane_safe || !right_lane_safe ) cout << "LEFT OR RIGHT LANE NOT SAFE" << endl;
            if (too_close) {
              // behavior planning
              vector<double> state_costs; // LK, LCL, LCR, PLCL, PLCR
              double lk_cost, lcl_cost, lcr_cost, plcl_cost, plcr_cost;
              // lane keep
              lk_cost = inefficiency_cost(target_speed, lane, lane, lane_speeds);
              // left lane exists
              if (lane-1>=0) {
                lcl_cost = inefficiency_cost(target_speed, lane-1, lane-1, lane_speeds);
                plcl_cost = inefficiency_cost(target_speed, lane, lane-1, lane_speeds);
              } else {
                lcl_cost = 9999.;
                plcl_cost = 9999.;
              }
              // right lane exists
              if (lane+1<=2) {
                lcr_cost = inefficiency_cost(target_speed, lane+1, lane+1, lane_speeds);
                plcr_cost = inefficiency_cost(target_speed, lane, lane+1, lane_speeds);
              } else {
                lcr_cost = 9999.;
                plcr_cost = 9999.;
              }
              state_costs.push_back(lk_cost);
              state_costs.push_back(lcl_cost);
              state_costs.push_back(lcr_cost);
              state_costs.push_back(plcl_cost);
              state_costs.push_back(plcr_cost);

              float min_value = 9999.;
              int min_index;
              for (int i = 0; i < state_costs.size(); i++) {
                if (state_costs[i] < min_value) {
                  min_value = state_costs[i];
                  min_index = i;
                }
              }

              // if not safe to change lanes
              if (min_index == 1 && !left_lane_safe) min_index = 3;
              if (min_index == 2 && !right_lane_safe) min_index = 4;
              cout << "DECISION BEHAVIOR: " << min_index << endl;
              if (min_index == 0) {
                target_speed = lane_speeds[lane];
              } else if (min_index == 1) {
                lane -= 1;
                target_speed = lane_speeds[lane-1];
              } else if (min_index == 2) {
                lane += 1;
                target_speed = lane_speeds[lane+1];
              } else if (min_index == 3) {
                target_speed = lane_speeds[lane-1];
              } else if (min_index == 4) {
                target_speed = lane_speeds[lane+1];
              }
            }
            
            // create evenly spaced waypoints
            vector<double> ptsx;
            vector<double> ptsy;

            // starting ref points
            double ref_x, ref_y, ref_yaw;
            if (prev_size < 2) {
              // starting out
              ref_x = car_x;
              ref_y = car_y; 
              ref_yaw = deg2rad(car_yaw);
              double prev_car_x = car_x - cos(ref_yaw);
              double prev_car_y = car_y - sin(ref_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(ref_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(ref_y);
            } else {
              // take the last two points in prev path
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double second_to_last_x = previous_path_x[prev_size-2];
              double second_to_last_y = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-second_to_last_y, ref_x-second_to_last_x);

              ptsx.push_back(second_to_last_x);
              ptsx.push_back(ref_x);
              ptsy.push_back(second_to_last_y);
              ptsy.push_back(ref_y);
            }

            // add 30m even spaced waypoints (in Frenet!)
            vector<double> next_wp0 = getXY(car_s+50, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+100, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+120, 4*lane+2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsy.push_back(next_wp0[1]);
            ptsx.push_back(next_wp1[0]);
            ptsy.push_back(next_wp1[1]);
            ptsx.push_back(next_wp2[0]);
            ptsy.push_back(next_wp2[1]);

            // shift to local coordinates
            for (int i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x*cos(-ref_yaw)-shift_y*sin(-ref_yaw);
              ptsy[i] = shift_x*sin(-ref_yaw)+shift_y*cos(-ref_yaw);
            }

            // spline calculation
            tk::spline s;
            
            s.set_points(ptsx, ptsy);
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            // save prev points
            for (int i = 0; i < prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

            }

            // fill the rest with spline
            double target_x = 50;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            double x_add_on = 0;
            double delta;
            for (int i = 0; i < 50-prev_size; i++) {
              if (target_speed < ref_vel) {
                delta = (ref_vel-target_speed)/50;
                if (delta > .224) delta = .224;
                ref_vel -= delta;
              } else {
                delta = (target_speed-ref_vel)/50;
                if (delta > .224) delta = .224;
                ref_vel += delta;
              }
              double N = target_dist/(.02*ref_vel/2.24);
              double x_point = target_x/N + x_add_on;
              double y_point = s(x_point);
              x_add_on = x_point;

              // back to normal coordinates
              double x_ref = x_point;
              double y_ref = y_point;
              x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);
        
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
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
