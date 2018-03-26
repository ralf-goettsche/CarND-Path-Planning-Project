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

int main() {
  uWS::Hub h;

  enum fsm_state {keep_lane, prep_4_left_lane, prep_4_right_lane, change_2_left_lane, change_2_right_lane};
  fsm_state car_state = keep_lane;
  int target_lane = 1;
  double target_vel  = 0.0;
  
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&target_lane,&target_vel,&car_state]
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

                // Definition of constants
                //
		const float max_speed  = 49.5;      // MPH
                const float maxacc_vel = 0.3;       // MPH, => ~ 7.1 m/sˆ2 = ~10/sqrt(2)
                
                const float red_range = 20;  // m, closest range
                const float mid_range = 40;  // m, middle range
                const float far_range = 60;  // m, far range
                

                // Determination of current car status
                //
                int path_size = previous_path_x.size();
		double car_mod_x = car_x;
		double car_mod_y = car_y;
		double car_mod_yaw = car_yaw;
		double car_mod_rad = deg2rad(car_yaw);

                // Needed for spline calculation
                //
		vector<double> sp_break_pts_x;
		vector<double> sp_break_pts_y;

                // Boolean variables determining if any of the foreign cars
                // are in a certain range to the car
                //
		bool too_close = false;  // foreign car is in the closest range to the car => maximum possible down acceleration
                                         //                                                => only taking the breaks and keeping lane
		bool very_close = false; // foreign car is in the middle  range to the car => linear increasing down acceleration
                                         //                                                   between zero and max possbile down acceleration
                                         //                                                => prep for lane change and lane change possible
                bool car_ahead = false;  // foreign car is in the far     range to the car => prepare for lange change if possible
                                         //                                                => prep for lane change
                bool car_ahead_right = false;  // foreign car is in the far range to the car in the nex right lane
                                               //                                          => no lane change
                
                // Boolean variables describing additional car stati
                //
                bool in_lane = false;        // car is between 1m and 3m of the according lane => lane change is finalized
                bool ready_4_change = false; // environment of the car is ready for lane change

                // Help variables
                //
		double front_car_vel = 0.0;      // velocity of closest car in the target_lane
                double closest_dist_s = 100.0;   // distance of closest car in the target_lane
		double break_vel = 0.0;          // down acceleration value for middle range, linearly calculated, range = [0, maxacc_vel]
                double min_front_dist_s = 100.0; // min distance to car in the front on the next lane which is target of lane change
                double min_back_dist_s = 100.0;  // min distance to car in the back  on the next lane which is target of lane change
                

                // Determining position of car at the beginning, the far future of the trajectory
		if (path_size > 0)
		{
		  car_s = end_path_s;
		}

		// #####
                // # Prediction
		// #####
		// # Lane Space Exploration
		for (int i = 0; i < sensor_fusion.size(); i++)
		{
                    // Get the data of each foreign car
                    double d = sensor_fusion[i][6];
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    // real velocity
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];
                    // distance of foreign car at the end of trajectory
                    check_car_s += ((double)path_size * 0.02 * check_speed);
                    // distance of foreign car to car
                    double delta_car_s = check_car_s - car_s;

                    // If foreign car is in the target_lane,
                    // check for range the foreign car might be in and set according variables
                    if ((target_lane*4 < d) && (d < (4 + target_lane*4)))
                    {
                        // closest range
                        if ((check_car_s > car_s) && (delta_car_s < red_range))
                        {
                            too_close = true;
                        }
                        // middle range
                        // calculate linear break value in accordance to distance and velocity difference
                        else if ((check_car_s > car_s) && (delta_car_s < mid_range))
                        {
                            very_close = true;
                            break_vel = (1 - (delta_car_s - red_range)/(mid_range - red_range)) * (abs(check_speed - car_speed)/fmax(check_speed, car_speed)) * maxacc_vel;
                        }
                        // far range
                        else if ((check_car_s > car_s) && (delta_car_s < far_range))
                        {
                            car_ahead = true;
                        }
                        // Determine closest car in front of target_lane
                        if ((check_car_s > car_s) && (delta_car_s < closest_dist_s))
                        {
                            closest_dist_s = delta_car_s;
                            front_car_vel = check_speed;
                        }
                    }

                    // Check for additional stuff in dependence of car state
                    switch (car_state)
                    {
                        case keep_lane:
                            // Check for cars in right lane if one is in far_range
                            if (((target_lane + 1)*4 < d) && (d < (4 + (target_lane + 1)*4)))
                            {
                                if ((check_car_s > car_s) && (delta_car_s < far_range))
                                {
                                    car_ahead_right = true;
                                }
                            }
                            break;
                        case prep_4_left_lane:
                            // Determining distance window between cars in the left lane
                            if (((target_lane - 1)*4 < d) && (d < (4 + (target_lane - 1)*4)))
                            {
                                if (delta_car_s >= 0.0)
                                {
                                    if (delta_car_s < min_front_dist_s)
                                    {
                                        min_front_dist_s = delta_car_s;
                                    }
                                }
                                else 
                                {
                                    if (fabs(delta_car_s) < min_back_dist_s)
                                    {
                                        min_back_dist_s = fabs(delta_car_s);
                                    }
                                }
                            }
                            break;
                        case prep_4_right_lane:
                            // Determining distance window between cars in the right lane
                            // Additional check if cars are in far_range in the right lane
                            if (((target_lane + 1)*4 < d) && (d < (4 + (target_lane + 1)*4)))
                            {
                                if (delta_car_s >= 0.0)
                                {
                                    if (delta_car_s < min_front_dist_s)
                                    {
                                        min_front_dist_s = delta_car_s;
                                    }
                                    if (delta_car_s < far_range)
                                    {
                                        car_ahead_right = true;
                                    }
                                }
                                else 
                                {
                                    if (fabs(delta_car_s) < min_back_dist_s)
                                    {
                                        min_back_dist_s = fabs(delta_car_s);
                                    }
                                }
                            }
                            break;
                        case change_2_left_lane:
                            // Determining distance window between cars in the left lane
                            if ((target_lane * 4 < d) && (d < (4 + target_lane*4)))
                            {
                                if (delta_car_s >= 0.0)
                                {
                                    if (delta_car_s < min_front_dist_s)
                                    {
                                        min_front_dist_s = delta_car_s;
                                    }
                                }
                                else 
                                {
                                    if (fabs(delta_car_s) < min_back_dist_s)
                                    {
                                        min_back_dist_s = fabs(delta_car_s);
                                    }
                                }
                            }
                            break;
                        case change_2_right_lane:
                            // Determining distance window between cars in the right lane
                            if ((target_lane * 4 < d) && (d < (4 + target_lane*4)))
                            {
                                if (delta_car_s >= 0.0)
                                {
                                    if (delta_car_s < min_front_dist_s)
                                    {
                                        min_front_dist_s = delta_car_s;
                                    }
                                }
                                else 
                                {
                                    if (fabs(delta_car_s) < min_back_dist_s)
                                    {
                                        min_back_dist_s = fabs(delta_car_s);
                                    }
                                }
                            }
                           break;
                    }
                    
		}
                // Checking the distance window
                if ((car_state == prep_4_left_lane)  || (car_state == prep_4_right_lane))
                {
                    if ((min_back_dist_s > red_range) && (min_front_dist_s > red_range))
                    {
                        ready_4_change = true;
                    }
                }
                else if ((car_state == change_2_left_lane) || (car_state == change_2_right_lane))
                {
                    if ((min_back_dist_s > red_range) && (min_front_dist_s > red_range))
                    {
                        ready_4_change = true;
                    }
                }
                
                // Checking if car is in a lane again during changing the lane
                if (((1 + target_lane*4) < car_d) && (car_d < (3 + target_lane*4)))
                {
                    in_lane = true;
                }
                
   
                // ####
		// # Behavioral control
                // ####
                //
		switch (car_state)
		{
                    case keep_lane:
                        // foreign car in closest range => just reduce velocity with max value
                        if (too_close)
                        {
                            if (target_vel > front_car_vel)
                            {
                                target_vel -= maxacc_vel;
                            }
                        }
                        // foreign car in middle range => evtl. reduce speed, prep. for lane change
                        // either to left lane if possible or to the right lane if possible
                        else if (very_close)
                        {
                            if (target_vel > front_car_vel)
                            {
                                target_vel -= break_vel;
                            }
                            else if ((target_vel < front_car_vel) && (target_vel <= max_speed))
                            {
                                target_vel += break_vel;
                            }
                            if (target_lane > 0)
                            {
                                car_state = prep_4_left_lane;
                            }
                            else if ((target_lane < 2) && !car_ahead_right)
                            {
                                car_state = prep_4_right_lane;
                            }
                        }
                        // keep up speed,
                        // if foreign car in far range, prep. for left lane change
                        // if nothing is on the right lane, prep. for right lane change
                        else
                        {
                            if (target_vel <= max_speed)
                            {
                                target_vel += maxacc_vel;
                            }
                            if (car_ahead && (target_lane > 0))
                            {
                                car_state = prep_4_left_lane;
                            }
                            else if ((target_lane < 2) && !car_ahead_right)
                            {
                                car_state = prep_4_right_lane;
                            }
                        }
                        break;    
		    case prep_4_left_lane:
                        // foreign car in closest range => just reduce velocity with max value
                        if (too_close)
                        {
                            if (target_vel > front_car_vel)
                            {
                                target_vel -= maxacc_vel;
                            }
                            car_state = keep_lane;
                            break;
                        }
                        // foreign car in middle range => evtl. reduce speed
                        else if (very_close)
                        {
                            if (target_vel > front_car_vel)
                            {
                                target_vel -= break_vel;
                            }
                            else if ((target_vel < front_car_vel) && (target_vel <= max_speed))
                            {
                                target_vel += break_vel;
                            }
                        }
                        // keep up speed
                        else if (target_vel <= max_speed)
                        {
                            target_vel += maxacc_vel;
                        }
                        // change lane if ready
                        if (ready_4_change)
                        {
                            target_lane -= 1;
                            car_state = change_2_left_lane;
                        }
                        break;
		    case prep_4_right_lane:
                        // foreign car in closest range => just reduce velocity with max value
                        if (too_close)
                        {
                            if (target_vel > front_car_vel)
                            {
                                target_vel -= maxacc_vel;
                            }
                            car_state = keep_lane;
                            break;
                        }
                        // foreign car in middle range => evtl. reduce speed
                        else if (very_close)
                        {
                            if (target_vel > front_car_vel)
                            {
                                target_vel -= break_vel;
                            }
                            else if ((target_vel < front_car_vel) && (target_vel <= max_speed))
                            {
                                target_vel += break_vel;
                            }
                            // if a foreign car is in far range in the right lane, skip lane change
                            if (car_ahead_right)
                            {
                                car_state = keep_lane;
                                break;
                            }
                        }
                        // keep up speed
                        else if (target_vel <= max_speed)
                        {
                            target_vel += maxacc_vel;
                        }
                        // change lane if ready
                        if (ready_4_change)
                        {
                            target_lane += 1;
                            car_state = change_2_right_lane;
                        }
                        break;
		    case change_2_left_lane:
                        // continue change if nothing is blocking
                        if (ready_4_change)
                        {
                            // foreign car in closest range of target lane => just reduce velocity with max value
                            if (too_close)
                            {
                                if (target_vel > front_car_vel)
                                {
                                    target_vel -= maxacc_vel;
                                }
                            }
                            // foreign car in middle range of target lane => just reduce velocity
                            else if (very_close)
                            {
                                if (target_vel > front_car_vel)
                                {
                                    target_vel -= break_vel;
                                }
                                else if ((target_vel < front_car_vel) && (target_vel <= max_speed))
                                {
                                    target_vel += break_vel;
                                }
                            }
                            // we reached the lane, turning to keep_lane state
                            if (in_lane)
                            {
                                car_state = keep_lane;
                            }
                            // otherwise check for speed
                            else if (target_vel <= max_speed)
                            {
                                target_vel += maxacc_vel;
                            }
                        }
                        // something is blocking the car => turning back to lane
                        else
                        {
                            car_state = prep_4_right_lane;
                        }
                        break;
		    case change_2_right_lane:
                        // continue change if nothing is blocking
                        if (ready_4_change)
                        {
                            // foreign car in closest range of target lane => just reduce velocity with max value
                            if (too_close)
                            {
                                if (target_vel > front_car_vel)
                                {
                                    target_vel -= maxacc_vel;
                                }
                            }
                            // foreign car in middle range of target lane => just reduce velocity
                            else if (very_close)
                            {
                                if (target_vel > front_car_vel)
                                {
                                    target_vel -= break_vel;
                                }
                                else if ((target_vel < front_car_vel) && (target_vel <= max_speed))
                                {
                                    target_vel += break_vel;
                                }
                            }
                            // we reached the lane, turning to keep_lane state
                            if (in_lane)
                            {
                                car_state = keep_lane;
                            }
                            // otherwise check for speed
                            else if (target_vel <= max_speed)
                            {
                                target_vel += maxacc_vel;
                            }
                        }
                        // something is blocking the car => turning back to lane
                        else
                        {
                            car_state = prep_4_left_lane;
                        }
                        break;
		}

                
		// #####
		// # Trajectory Generation
		// #####
		// Past (in xy-coordinates)
                for(int i = 0; i < path_size; i++)
                {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }

		// Calculation of spline-trajectory
                // from xy- into car-coordinates
                //
		// Determination of spline-breakpoints in xy-coordinates (Past), for all states the same
		if(path_size < 2)
                {
		  sp_break_pts_x.push_back(car_mod_x - cos(car_mod_yaw));
		  sp_break_pts_x.push_back(car_mod_x);
		  sp_break_pts_y.push_back(car_mod_y - sin(car_mod_yaw));
		  sp_break_pts_y.push_back(car_mod_y);
                }
                else
                {
		  car_mod_x = previous_path_x[path_size-1];
		  car_mod_y = previous_path_y[path_size-1];

		  double car_mod_x_tm1 = previous_path_x[path_size-2];
		  double car_mod_y_tm1 = previous_path_y[path_size-2];
		  
		  sp_break_pts_x.push_back(car_mod_x_tm1);
		  sp_break_pts_x.push_back(car_mod_x);
		  sp_break_pts_y.push_back(car_mod_y_tm1);
		  sp_break_pts_y.push_back(car_mod_y);
		  car_mod_rad = atan2(car_mod_y - car_mod_y_tm1,
				      car_mod_x - car_mod_x_tm1);
	        }

                // Calculation of 'future' spline breakpoints
		vector<double> wp1 = getXY(car_s + 30, 2 + target_lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> wp2 = getXY(car_s + 60, 2 + target_lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		vector<double> wp3 = getXY(car_s + 90, 2 + target_lane * 4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

		// Adding future spline-breakpoints to list
		sp_break_pts_x.push_back(wp1[0]);
		sp_break_pts_x.push_back(wp2[0]);
		sp_break_pts_x.push_back(wp3[0]);
		sp_break_pts_y.push_back(wp1[1]);
		sp_break_pts_y.push_back(wp2[1]);
		sp_break_pts_y.push_back(wp3[1]);

		// Transformation from xy- to car-coordinates
		for (int i = 0; i < sp_break_pts_x.size(); ++i)
		{
		  double shift_x = sp_break_pts_x[i] - car_mod_x;
		  double shift_y = sp_break_pts_y[i] - car_mod_y;
		  
		  sp_break_pts_x[i] = shift_x * cos(0-car_mod_rad) - shift_y * sin(0-car_mod_rad);
		  sp_break_pts_y[i] = shift_x * sin(0-car_mod_rad) + shift_y * cos(0-car_mod_rad);
		}

		// Determination of spline in car-coordinates
		tk::spline s;
                s.set_points(sp_break_pts_x, sp_break_pts_y);

                // Calculation of the remaining trajectory points
                //
		// Future (starting in car-coord with determined spline, transformed into xy-coordinates)
		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

		double N = target_dist/(0.02 * target_vel/2.23694);
		double x_add_on = 0.0;
		for (int i = 0; i < 50-path_size; i++)
                {
		  // Calculation of spline values
		  double x_point = x_add_on + target_x/N;
		  double y_point = s(x_point);

		  x_add_on = x_point;

		  // Transformation from car- to xy-coordinates
		  double x_ref = x_point;
		  double y_ref = y_point;
		  x_point = x_ref * cos(car_mod_rad) - y_ref * sin(car_mod_rad);
		  y_point = x_ref * sin(car_mod_rad) + y_ref * cos(car_mod_rad);
		  x_point += car_mod_x;
		  y_point += car_mod_y;
		  
                  next_x_vals.push_back(x_point);
                  next_y_vals.push_back(y_point);
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
