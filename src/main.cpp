#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <tuple>
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

double speed_limit = 49.5*0.447;
double anchor_spacing = 30.0;
double collision_check_margin = 30.0;
double timestep = 0.02;
double max_acceleration = 5; // m/s^2
int lanes_available = 3;

//double target_speed = speed_limit;
//int target_lane = 1;
//
//double current_speed = 0.0;
//int current_lane = 1;

struct Car {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double speed;
  double yaw = 0.0;
  int lane = 0;
};
typedef struct Car Car;
struct Map {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};
typedef struct Map Map;
struct Trajectory {
  vector<double> x;
  vector<double> y;

  inline size_t size() const{
    return x.size();
  }
};
typedef struct Trajectory Trajectory;
struct Command {
  double speed;
  int lane;
};
typedef struct Command Command;

void from_json(const json& j, Car& c) {
  c.id = j[0];
  c.x = j[1];
  c.y = j[2];
  c.vx = j[3];
  c.vy = j[4];
  c.speed = sqrt(c.vx*c.vx + c.vy*c.vy);
  c.s = j[5];
  c.d = j[6];
}

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

double distance (double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const Map& map)
{
	int next_wp = NextWaypoint(x,y, theta, map.x,map.y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map.x.size()-1;
	}

	double n_x = map.x[next_wp]-map.x[prev_wp];
	double n_y = map.y[next_wp]-map.y[prev_wp];
	double x_x = x - map.x[prev_wp];
	double x_y = y - map.y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-map.x[prev_wp];
	double center_y = 2000-map.y[prev_wp];
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
		frenet_s += distance(map.x[i],map.y[i],map.x[i+1],map.y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const Map& map)
{
	int prev_wp = -1;

	while(s > map.s[prev_wp+1] && (prev_wp < (int)(map.s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map.x.size();

	double heading = atan2((map.y[wp2]-map.y[prev_wp]),(map.x[wp2]-map.x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map.s[prev_wp]);

	double seg_x = map.x[prev_wp]+seg_s*cos(heading);
	double seg_y = map.y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

inline const tuple<double, double> GlobalToLocal(const tuple<double, double> input, const tuple<double, double, double> ref)
{
  // First find relative coordinates and then rotate them
  double x, y, x0, y0, theta0;
  std::tie(x, y) = input;
  std::tie(x0, y0, theta0) = ref;

  // Translate
  x = x - x0;
  y = y - y0;

  // Rotate
  return std::make_tuple(x * cos(theta0) + y * sin(theta0),
                        -x * sin(theta0) + y * cos(theta0));
}

inline const tuple<double, double> LocalToGlobal(const tuple<double, double> input, const tuple<double, double, double> ref)
{
  // First find relative coordinates and then rotate them
  double x, y, x0, y0, theta0;
  std::tie(x, y) = input;
  std::tie(x0, y0, theta0) = ref;

  // Rotate and translate
  return std::make_tuple(x0 + x * cos(theta0) - y * sin(theta0),
                         y0 + x * sin(theta0) + y * cos(theta0));
}

// Checks if car will collide with ego car, `delta_t` seconds in future
const bool CheckCollision(const Car& ego, const Car& car, int lane, const double delta_t)
{
  const auto future_s = car.s + car.speed*delta_t;
  const auto in_same_lane = (car.d > (2 + (4*lane-2)) && (car.d < (2+4*lane+2)));

  if (in_same_lane)
  {
    const auto in_front = (future_s > ego.s);
    const auto too_close= ((future_s - ego.s) < collision_check_margin);
    return in_front && too_close;
  }
  return false;
}

const Trajectory GenerateTrajectory(int target_lane,
                                    double target_speed,
                                    const Car& ego,
                                    const Map& map,
                                    const Trajectory& prev_path)
{
  // Lane shift one at a time
  auto delta_lane = target_lane - ego.lane;
  delta_lane = min(1, max(-1, delta_lane)); // Change one lane at a time
  auto cmd_lane = max(0, ego.lane + delta_lane);
  // Gentle acceleration
  auto delta_speed = target_speed - ego.speed;
  delta_speed = min(max_acceleration*timestep, max(-max_acceleration*timestep, delta_speed));
  auto cmd_speed = ego.speed + delta_speed;

  vector<double> spline_anchor_x, spline_anchor_y;

  auto num_prev_points = prev_path.size();
  // Reference state for coordinate transformation
  // Start with car state as reference
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  if (num_prev_points > 2) {
    // Else start at the end of prior waypoints
    // and use them as reference for coordinate transform
    ref_x = prev_path.x[num_prev_points-1];
    ref_y = prev_path.y[num_prev_points-1];

    // Estimate past heading
    double prev_wp_x = prev_path.x[num_prev_points-2];
    double prev_wp_y = prev_path.y[num_prev_points-2];
    ref_yaw = atan2(ref_y - prev_wp_y,
                    ref_x - prev_wp_x);

    spline_anchor_x.push_back(prev_path.x[num_prev_points-2]);
    spline_anchor_y.push_back(prev_path.y[num_prev_points-2]);
  }
  // position at t
  spline_anchor_x.push_back(ref_x);
  spline_anchor_y.push_back(ref_y);

  // Add equally spaced points in Frenet space
  for(int i = 0; i < 3; i++)
  {
    auto wp = getXY(ego.s + anchor_spacing*(i+1),
                    (2+ 4*cmd_lane),
                    map);
    spline_anchor_x.push_back(wp[0]);
    spline_anchor_y.push_back(wp[1]);
  }

  // Convert spline points to local coordinates
  auto ref_state = std::make_tuple(ref_x, ref_y, ref_yaw);
  for(int i=0; i<spline_anchor_x.size(); i++)
  {
    auto x = spline_anchor_x[i];
    auto y = spline_anchor_y[i];
    std::tie(x, y) = GlobalToLocal(std::make_tuple(x, y), ref_state);
    spline_anchor_x[i] = x;
    spline_anchor_y[i] = y;
  }

  tk::spline s;
  // Fit spline to anchor points
  s.set_points(spline_anchor_x, spline_anchor_y);

  // Copy old waypoints first
  Trajectory output;

  for(int i = 0; i < num_prev_points; i++)
  {
    output.x.push_back(prev_path.x[i]);
    output.y.push_back(prev_path.y[i]);
  }

  // Find waypoint spacing based on speed
  const auto target_x = anchor_spacing;
  const auto target_y = s(target_x);
  const auto target_dist = sqrt(target_x*target_x + target_y*target_y);
  double N = target_dist / (timestep*cmd_speed);

  // Find intermediate points and transform back to global space
  double last_x = 0;
  for(int i = 1; i <= 50 - num_prev_points; i++)
  {
    // double wp_x = last_x + anchor_spacing/num_points;
    double wp_x = last_x + target_x/N;
    double wp_y = s(wp_x);

    last_x = wp_x;
    tie(wp_x, wp_y) = LocalToGlobal(make_tuple(wp_x, wp_y), ref_state);
    output.x.push_back(wp_x);
    output.y.push_back(wp_y);
  }
  return output;
}

// Behavior planning stuff
#include "behavior.h"

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map map;
  Trajectory prev_path, future_path;
  Behavior behavior;

//  fsm.add_transitions({
//    //  from state ,to state  ,triggers        ,guard                    ,action
//    { States::KL, States::LCL, Triggers::DoLaneChangeLeft, [&]{return true;}, [&]{} },
//  });

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
  	map.x.push_back(x);
  	map.y.push_back(y);
  	map.s.push_back(s);
  	map.dx.push_back(d_x);
  	map.dy.push_back(d_y);
  }

  h.onMessage([&map, &behavior, &prev_path, &future_path](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	json msgJson;

            Car ego = {0, car_x, car_y, 0, 0, car_s, car_d, car_speed, deg2rad(car_yaw), 0};
            prev_path = {previous_path_x, previous_path_y};

            int num_prev_points = prev_path.size();

            if(num_prev_points > 0)
            {
              ego.s = end_path_s; // Use the "right" s value for collision prediction
            }

            vector<Car> traffic(sensor_fusion.size());
            for(int i=0; i<sensor_fusion.size(); i++)
            {
              Car car(sensor_fusion[i]);
              traffic.push_back(car);
            }
            // Loop over other detected cars

            // Do trajectory computation
            future_path = behavior.UpdateState(ego, traffic, map, prev_path);
          	msgJson["next_x"] = future_path.x;
          	msgJson["next_y"] = future_path.y;

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
