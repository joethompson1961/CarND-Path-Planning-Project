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

// Transform a pose from car local coordinate system to global map perspective.
// The anchor coordinates are the pose of the local map within the global map.
vector<double> local_to_global(double anchor_x, double anchor_y, double anchor_theta, double local_x, double local_y) {
    double global_x = anchor_x + (local_x * cos(anchor_theta)) - (local_y * sin(anchor_theta));
    double global_y = anchor_y + (local_x * sin(anchor_theta)) + (local_y * cos(anchor_theta));

    return {global_x, global_y};
}

// Transform a pose from global map perspective to car's local coordinate system (front of car faces positive x)
// The anchor coordinates are the pose of the local map within the global map.
vector<double> global_to_local(double anchor_x, double anchor_y, double anchor_theta, double global_x, double global_y) {
	double local_x = (global_x - anchor_x) * cos(-anchor_theta) - (global_y - anchor_y) * sin(-anchor_theta);
	double local_y = (global_x - anchor_x) * sin(-anchor_theta) + (global_y - anchor_y) * cos(-anchor_theta);

    return {local_x, local_y};
}

// return waypoint that's closest to the 
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

// Return next waypoint, which is the closest waypoint that's in front (not in back)
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	// heading from ego position to closest waypoint
	double heading = atan2((map_y-y),(map_x-x)); // radians

	double angle = fabs(theta-heading);  // difference between car's current heading and the heading to waypoint 
	angle = min(2*pi() - angle, angle);  // normalize

	// if more than 90 degrees angle to closest waypoint, i.e. it's behind you, then advance to next waypoint
	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
			closestWaypoint = 0;
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

#define MPH2MPS(i) (i * 0.44704)
#define MPS2MPH(i) (i * 2.24)
#define SF_VX   (3)
#define SF_VY   (4)
#define SF_S    (5)
#define SF_LANE (6)

#define FOLLOW_GAP (50)
#define TIME_STEP (0.02) // 0.02 seconds (50 Hz)
#define SPEED_LMT (49.5) // speed limit

bool have_clearance(double sf_s, double sf_s_pred, double sf_v,
                    double car_s, double car_s_pred, double car_v)
{
    bool clear = true;
    if ( (sf_s > (car_s - FOLLOW_GAP/3)) && (sf_s < (car_s + FOLLOW_GAP)) ) {
        // The other car is within the protected space next us
        clear = false;
//    } else if ((sf_s_pred > (car_s_pred - FOLLOW_GAP/3)) && (sf_s_pred < (car_s_pred + FOLLOW_GAP))) {
//        // The other car's predicted location is within the predicted protected space next to us
//        clear = false;
//    } else if ( (sf_s > car_s) && (sf_v < car_v) ) {
//        // The other car is in front but going slower
//        clear = false;
//    } else if ( (sf_s < car_s) && (car_s - sf_s < FOLLOW_GAP)
//            && (sf_v > car_v + 5) )  {
//        // The other car is behind protected space but close and approaching fast
//        clear = false;
    }
    return clear;
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

  int lane = 1;
  int prev_lane = 1;
  double ref_vel = 0.0;
  bool lane_change_in_progress = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&prev_lane,&ref_vel,&lane_change_in_progress]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	int path_size = previous_path_x.size();

          	double car_s_pred = car_s;
          	if (path_size > 0)
          	    car_s_pred = end_path_s;

            // Scan sensor fusion data for cars in front of us
            // If car in front, adjust speed/distance to stay behind car
            double sf_vx;
            double sf_vy;
            double sf_v = SPEED_LMT;
            double sf_s;
            double sf_s_pred;
            double tracking_speed = SPEED_LMT;
            bool following = false;
            double gap = FOLLOW_GAP;
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
                float sf_d = sensor_fusion[i][SF_LANE];
                // Normally this checks for car in front of our current lane.
                // During lane changes it checks the target lane and the previous lane.
                if ( (sf_d < (2 + 4*lane + 2) && sf_d > (2 + 4*lane - 2))
                    || (sf_d < (2 + 4*prev_lane + 2) && sf_d > (2 + 4*prev_lane - 2)) )
                {
                    // A car is in our lane
                    sf_vx = sensor_fusion[i][SF_VX];
                    sf_vx = MPS2MPH(sf_vx);
                    sf_vy = sensor_fusion[i][SF_VY];
                    sf_vy = MPS2MPH(sf_vy);
                    sf_v = sqrt(sf_vx*sf_vx + sf_vy*sf_vy);
                    sf_s = sensor_fusion[i][SF_S];

                    // Predict the vehicle's s value out to the end of our path
                    sf_s_pred = sf_s + ((double)path_size*TIME_STEP*sf_v);

                    // If the sensed car is in front of us and it's predicted location is in front of us
                    // and it's too close to us, then slow down.
                    if ((sf_s > car_s) && (sf_s_pred > car_s_pred) && ((sf_s_pred - car_s_pred) < FOLLOW_GAP))
                    {
                        following = true;
                        if ((sf_s - car_s) < gap)
                            gap = sf_s - car_s;     // keep track of smallest gap.
                        if (tracking_speed > sf_v) {
                            tracking_speed = sf_v;  // match the slowest car in front
                        }
                    }
                }
            }

            // If following, adjust speed to match car in front.
            // To smoothly fall-in behind the car in front, adjust the target velocity
            // proportional to the distance to the car.
            //
            // If not following then follow the speed limit.
            // To reach target velocity smoothly, adjustments to velocity are proportional to the needed acceleration.
            if (following) {
                if (ref_vel > tracking_speed)
                    ref_vel -= (0.333 * (1 - gap/FOLLOW_GAP));
                else
                    ref_vel += (0.224 * gap/FOLLOW_GAP);
            } else if (ref_vel > tracking_speed) {
                ref_vel -= (0.224 * (1 - tracking_speed/ref_vel));
            } else if (ref_vel < tracking_speed) {
                ref_vel += (0.224 * (1 - ref_vel/tracking_speed));  // increase speed without exceeding max acceleration
            }

            // Manage Lane Changes
            // If a lane change is already in progress don't consider change until it completes.
            if (lane_change_in_progress)
            {
                if ( (car_d > (4.0*(float)lane+1.5)) && ((car_d < (4*(float)lane+2.5))) )
                {
                    lane_change_in_progress = false;
                    prev_lane = lane;
//                    cout << endl << "Change to lane " << lane << " complete." << endl;
                }
            }
            else
            {
                // If following a car then figure out if we can pass
                if (following)
                {
                    bool change_lane_left = false;
                    if (lane > 0)   // if there's a lane to the left
                    {
                        change_lane_left = true; // assume true and reset in not safe in loop below.
                        for (int i = 0; i < sensor_fusion.size(); i++)
                        {
                            float sf_d = sensor_fusion[i][SF_LANE];
                            if ((sf_d > (4*(lane-1))) && (sf_d < (4*(lane-1)+4)))
                            {
                                // A car is sensed in lane to left
                                sf_vx = sensor_fusion[i][SF_VX];
                                sf_vx = MPS2MPH(sf_vx);
                                sf_vy = sensor_fusion[i][SF_VY];
                                sf_vy = MPS2MPH(sf_vy);
                                sf_v = sqrt(sf_vx*sf_vx + sf_vy*sf_vy);
                                sf_s = sensor_fusion[i][SF_S];

                                // Predict the vehicle's s value out to the end of our path
                                sf_s_pred = sf_s + ((double)path_size*TIME_STEP*sf_v);

                                // Clear the flag if prediction that it's unsafe to change lanes.
                                change_lane_left &= have_clearance(sf_s, sf_s_pred, sf_v, car_s, car_s_pred, tracking_speed);
                            }
                        }
                    }

                    bool change_lane_right = false;
                    if (lane < 2)  // if there's a lane to the right
                    {
                        change_lane_right = true; // assume true and reset in not safe in loop below.
                        for (int i = 0; i < sensor_fusion.size(); i++)
                        {
                            float sf_d = sensor_fusion[i][SF_LANE];
                            if ((sf_d > (4*(lane+1))) && (sf_d < (4*(lane+1)+4)))
                            {
                                // A car is sensed in lane to right
                                sf_vx = sensor_fusion[i][SF_VX];
                                sf_vx = MPS2MPH(sf_vx);
                                sf_vy = sensor_fusion[i][SF_VY];
                                sf_vy = MPS2MPH(sf_vy);
                                sf_v = sqrt(sf_vx*sf_vx + sf_vy*sf_vy);
                                sf_s = sensor_fusion[i][SF_S];

                                // Predict the vehicle's s value out to the end of our path
                                sf_s_pred = sf_s + ((double)path_size*TIME_STEP*sf_v);

                                // Clear the flag if prediction that it's unsafe to change lanes.
                                change_lane_right &= have_clearance(sf_s, sf_s_pred, sf_v, car_s, car_s_pred, tracking_speed);
                            }
                        }
                    }

                    if (change_lane_left) {
                        prev_lane = lane;
                        lane -= 1;
                        lane_change_in_progress = true;
//                        cout << "Lane change left in progress." << endl;
                    } else if (change_lane_right) {
                        prev_lane = lane;
                        lane += 1;
                        lane_change_in_progress = true;
//                        cout << "Lane change right in progress." << endl;
                    }
                }
            }

			// Prepare a spline for generating new points to extend the path
          	tk::spline s;
          	vector<double> anchor_x;
          	vector<double> anchor_y;
          	double ref_x;
          	double ref_y;
          	double ref_yaw;
          	double prev_x;
          	double prev_y;

          	// Initialize spline anchor points with a couple of points from the end of the previous path
          	// or using the current pose if the previous path doesn't have at least 2 points.
          	if (path_size < 2)
          	{
          		ref_x = car_x;
          		ref_y = car_y;
          		prev_x = car_x - cos(ref_yaw);
          		prev_y = car_y - sin(ref_yaw);
              	ref_yaw = deg2rad(car_yaw);
          	}
          	else
          	{
          		// Starting the spline with previous path and ending it based on actual pose helps
          		// create a smooth transition from previous path to the new path (in case the
          		// pose of the car has drifted some from the previous path).
          		ref_x = previous_path_x[path_size-1];
          		ref_y = previous_path_y[path_size-1];
          		prev_x = previous_path_x[path_size-2];
          		prev_y = previous_path_y[path_size-2];
              	ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
          	}
          	anchor_x.push_back(prev_x);
          	anchor_y.push_back(prev_y);
          	anchor_x.push_back(ref_x);
          	anchor_y.push_back(ref_y);

          	// Extend spline anchor points with a few points that are based on but well ahead of the current pose.
          	double d =  2.0 + lane * 4.0;
          	vector<double> next;
          	for (int i=1 ; i<=3 ; i++)
          	{
          		next = getXY(car_s + i*(FOLLOW_GAP), d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              	anchor_x.push_back(next[0]);
              	anchor_y.push_back(next[1]);
          	}

          	// Convert spline anchor points to car's local frame of reference
          	for (int i = 0 ; i < anchor_x.size() ; i++)
          	{
          		next = global_to_local(ref_x, ref_y, ref_yaw, anchor_x[i], anchor_y[i]);
          		anchor_x[i] = next[0];
          		anchor_y[i] = next[1];
          	}

          	// Initialize spline with anchor points
          	s.set_points(anchor_x, anchor_y);

          	// copy the remaining previous trajectory to the new trajectory
			for (int i = 0 ; i < path_size ; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double dist = distance(ref_x, s(ref_x), ref_x+FOLLOW_GAP, s(ref_x+FOLLOW_GAP));
          	double x = 0;
			double y;
			double x_part = (FOLLOW_GAP / dist);  // ratio of x distance to total distance, i.e. the x portion of the velocity vector.
			for (int i = path_size ; i < 50 ; i++)
			{
				x += (MPH2MPS(ref_vel) * TIME_STEP * x_part);
				y = s(x);

				next = local_to_global(ref_x, ref_y, ref_yaw, x, y);

				next_x_vals.push_back(next[0]);
				next_y_vals.push_back(next[1]);
			}

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
