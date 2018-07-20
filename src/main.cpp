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
#include <string>
#include "LaneChange.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//lane of the vehicle
int lane = 0;

Lane LeftLane = Lane(0);
Lane MiddleLane = Lane(1);
Lane RightLane = Lane(2);
std::vector<Lane> lanes{ LeftLane , MiddleLane , RightLane };

//actual max acceleraation is 10 but we take 9 to prevent going beyond 10
//max acceleration.
double Max_Accn = 9.0;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

	//std::cout << "works till here0!!";

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
	  //std::cout << "works till here1!!";

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();

		//std::cout << "works till here2!!";
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
			//std::cout << "works till here3!!";
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

			//std::cout << "sensor fusion size: " << sensor_fusion.size() << endl;

			//std::cout << sensor_fusion[2] << endl;

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			//previous path is a vector recieved by the simulator. It consists of the previous path points the car has
			//not yet covered. Say the car has recieved 50 points for its path, and by the time the next path has been
			//generated the car has only been able to cover 35 points of its previous way points, so now the vector of
			//remaining 15 x and y points will be included in the previous_path_x, and the previous _path_y vectors 

			//std::cout << "car_s: " << car_s << endl;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			int prev_size = previous_path_x.size();

			//std::cout << "prev size: " << prev_size << endl;

			//use this if you want to truncate the length of previous path.
			bool truncate = false;
			if (truncate == true)
			{
				int prev_points_max = std::min(30, prev_size);

				previous_path_x.erase(previous_path_x.begin() + prev_points_max, previous_path_x.end());
				previous_path_y.erase(previous_path_y.begin() + prev_points_max, previous_path_y.end());
			}


			//in miles per hour
			double target_velocity = 49.0;

			//Keep velocity with reference to this vehicle.
			int target_vehicle;
			
			bool maintain_speed;
			for (int i = 0; i < sensor_fusion.size(); i++)
			{
				int current_lane = car_d / 4;
				//std::cout << current_lane << endl;
				double sensor_s = sensor_fusion[i][5];
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];

				double sensor_velocity = sqrt(pow(vx, 2) + pow(vy, 2));

				double sensor_d = sensor_fusion[i][6];

				//Ego Vehicle Lane
				if ((sensor_d > 4 * current_lane) && (sensor_d < 4 * (current_lane + 1)))
				{

					//A good way to think about is to assume the previous end path of the vehicle to be like
					// the eyes of the car and if it sees an issue upcoming then it tries to slow down.
					if ((sensor_s - end_path_s) < 15.0 && (sensor_s - end_path_s) > 0.0)
					{
						//lane = (lane + 1);
						//break;
						if (sensor_velocity < target_velocity)
						{
							target_velocity = sensor_velocity;
							target_vehicle = i;
							std::cout << "			Noticing Car within 20m" << endl;
						}

					}
				}
			}


			//to smoothen our path we generate a spline ahead of the end of the previous path. We will include the last
			//2 points from the previous path so that our newly generated path will be tangential to the previous path
			// and there will be a smoooth transition between the previous path and the new path.
			//The values that will be pushed to the next_x_vals, and the next_y_vals will first consist of the 
			//previous path points which havent been covered so far and then points which will be taken from the spline.
			
			//std::cout << "works till here!!";
		
			//These two vectors are used to make the spline.
			vector<double> ptsx;
			vector<double> ptsy;

			//at the start or at any other point if the previous path is empty, even then the car should start moving.
			double ref_x = car_x;
			double ref_y = car_y;
			//std::cout << "car_yaw: " << car_yaw << endl;
			double ref_yaw = deg2rad(car_yaw);
			//std::cout << "car_yaw in deg: " << ref_yaw*180/3.14159 << endl;

			//expected to be in metres per second
			double ref_vel;

			double prev_ref_y;
			double prev_ref_x;

			if (prev_size < 2)
			{
				prev_ref_x = car_x - cos(ref_yaw);
				prev_ref_y = car_y - sin(ref_yaw);

				ptsx.push_back(prev_ref_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_ref_y);
				ptsy.push_back(car_y);

				ref_vel = car_speed * 4 / 9;
			}

			else
			{
				ref_x = previous_path_x[prev_size - 1];
				prev_ref_x = previous_path_x[prev_size - 2];

				ref_y = previous_path_y[prev_size - 1];
				prev_ref_y = previous_path_y[prev_size - 2];

				ptsx.push_back(prev_ref_x);
				ptsx.push_back(ref_x);

				ptsy.push_back(prev_ref_y);
				ptsy.push_back(ref_y);

				//atan2 outputs values in radians.
				ref_yaw = atan2((ref_y - prev_ref_y), (ref_x - prev_ref_x));

				//gives ref_vel in metres per second
				ref_vel = (sqrt(pow((ref_x - prev_ref_x), 2) + pow((ref_y - prev_ref_y), 2))) / 0.02;

				//std::cout << "ref_yaw" << ref_yaw << endl;
				//std::cout << "car_yaw" << deg2rad(car_yaw) << endl;
				//std::cout << "difference: " << deg2rad(car_yaw) - ref_yaw << endl;
			}

			//std::cout << "ignore:		" << end_path_s << endl;

			//If the ref_vel i.e velocity of the car at the end of the path is less than 38 mph than look for
			// a new lane to shoft in which might have a higher lane speed.
			if (ref_vel < 38 * 4 / 9)
			{
				int cur_lane = end_path_d / 4;
				std::vector<std::vector<double>> adjoining{ { 1 },{ 0,2 },{ 1 } };
				std::vector<std::vector<bool>> possible_shifts;
				std::vector<std::vector<double>> lane_speeds;

				for (int i = 0; i < adjoining[cur_lane].size(); i++)
				{
					double lane_number = adjoining[cur_lane][i];
					Lane new_lane = lanes[lane_number];

					bool possible = new_lane.LaneShift(sensor_fusion, end_path_s, end_path_d);
					possible_shifts.push_back({ lane_number, possible });

					double lane_speed = new_lane.LaneSpeed(sensor_fusion, end_path_s, end_path_d);
					lane_speeds.push_back({ lane_number, lane_speed });
				}

				double max_lane_speed = 0;
				double new_lane;

				for (int j = 0; j < possible_shifts.size(); j++)
				{
					if (possible_shifts[j][1] == true && (lane_speeds[j][1]>max_lane_speed))
					{
						max_lane_speed = lane_speeds[j][1];
						new_lane = possible_shifts[j][0];
					}
				}

				lane = new_lane;
			}

			int closest_s = ClosestWaypoint(ref_x, ref_y, map_waypoints_x, map_waypoints_y);

			vector<double> next_wp0 = getXY(car_s + 55, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s + 70, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s + 85, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

			for (int i = 0; i < ptsx.size(); i++)
			{
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = shift_x*cos(ref_yaw) + shift_y*sin(ref_yaw);
				ptsy[i] = shift_y*cos(ref_yaw) - shift_x*sin(ref_yaw);
			}

			//std::cout << ptsx[0];
			//std::cout << ptsy[0];

			//std::cout << "ptsx size: " << ptsx.size() << "ptsy: " << ptsy.size() << endl;
			
			tk::spline s;
			s.set_points(ptsx, ptsy);


			//In this part we are supposed to divide the path in distinct points which can be used to give a velocity
			// to the car, Since this is a little computationally expensive we instead divide the direct distance
			// to the target in distinct points as such an approximation will hardly make any difference.
			//target distance is a much better approximation of the path then target_x, so we divide target_dist in
			//N small increments, and use these increments to create x, and y values from the spline.

			//in metres
			double target_x = 30.0;
			double target_y = s(target_x);
			//std::cout << "target_y: " << target_y << endl;
			double target_dist = sqrt(target_x*target_x + target_y*target_y);

			//std::cout << "car speed: " << car_speed << endl;
			//std::cout << "ref vel : " << ref_vel << endl;

			//max accn * 0.02 is the maximum speed that can 
			double max_increase_in_speed = Max_Accn*0.02;

			double x_add_on = 0.0;
			//std::cout << x_add_on;

			for (int i = 0; i < prev_size; i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}


			//Calculate Acceleration for the vehicle to slow down in time.

			//The distance the vehicle must travel while slowing down to the target_speed from the
			//previous path end.
			//In this scenario 50 is the distance the car in front of us will travel befor we reach the 
			// target speed and 10m is the gap that will be maintained between the vehicle in front and us.
			double target_s = sensor_fusion[target_vehicle][5];

			double distance_to_target_speed = target_s + (100) - end_path_s;

			double accn_to_slowdown = (abs((pow((target_velocity * 4 / 9), 2) - pow(ref_vel, 2))) / (2 * distance_to_target_speed))*0.02;
			if (accn_to_slowdown > Max_Accn*0.02)
			{
				accn_to_slowdown = Max_Accn*0.02;
				std::cout << "max slowdown acceleration" << endl;
			}
			
			double accn_to_speedup = Max_Accn*0.02;
			
			//Everything beyond this is in SI
			for (int i = 0; i < 50 - prev_size; i++)
			{
				//If you want to increase or decrease the speed we have to do it by using the path end speed
				// and not on the current car speed. The previous path points are already added, what we need
				// to do is add new path points and if the ref_vel that is the velocity at the end of the 
				// previous path is less than the desired velocity then we may increase or decrease the
				// ref_vel and generate new points using the changed ref_vel.

				//15 is working
				if (ref_vel < ((target_velocity * 4 / 9) - 9*(Max_Accn*0.02)))
				{
					std::cout << "speeding up" << endl;
					//std::cout << "ref_vel: " << ref_vel << endl;
					//std::cout << "target velocity in metres pe second: " << (target_velocity * 4 / 9) << endl;
					ref_vel += accn_to_speedup;
				}

				else if (ref_vel >((target_velocity * 4 / 9) + 15*accn_to_slowdown))
				{
					std::cout << "slowing down" << endl;
					//std::cout << "ref_vel: " << ref_vel << endl;
					//std::cout << "target velocity in metres pe second: " << (target_velocity * 4 / 9) << endl;
					ref_vel -= accn_to_slowdown;
				}

				else
				{
					std::cout << "	maintaining speed." << endl;
				}



				double N = target_dist / (0.02*(ref_vel));

				double x_point = x_add_on + (target_dist/N)*cos(target_y/target_x);
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// Rotate and shift back to normal
				x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
				y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

				//translate back to global coordinates.
				x_point += ref_x;
				y_point += ref_y;

				
				/*
				double test_yaw = atan2((y_point - y_point_prev), (x_point_prev - x_point));

				double x_point_prev = x_point;
				double y_point_prev = y_point;

				vector<double> sd_test;

				sd_test = getFrenet(x_point, y_point, test_yaw, map_waypoints_x, map_waypoints_y);

				*/


				//std::cout << "s value: " << sd_test[0] << "d value: " << sd_test[1] << endl;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}
			
			/*

			double ang_vel = 0.0052;
			double dist_incr = 0.5;

			
			for (int i = 0; i < 50; i++)
				{

				
					double next_s = car_s + dist_incr*i;
					double next_d = 6;

					vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					next_x_vals.push_back(xy[0]);
					next_y_vals.push_back(xy[1]);

					
					double next_x = car_x + dist_incr*i*cos(deg2rad(car_yaw) + ang_vel*i);
					double next_y = car_y + dist_incr*i*sin(deg2rad(car_yaw) + ang_vel*i);	
				}
			
			*/

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(150));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  }
  
  
  );

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
