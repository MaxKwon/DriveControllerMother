/*
 * MotionProfiler.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: maxkwon
 */

#include "MotionProfiler.h"

namespace std {

double PI = 3.1415;

double ramp_time = 0.0;
double ramp_dis = 0.0;

double max_acceleration = 0.0;
double max_velocity = 0.0;

double yaw_rate = 0.0;
double yaw_acc = 0.0;

double iterations = 0.0;
double time_dt = 0.00001; //this is the interval that the profiler will run the simulation at,
//needs to be faster for accurate integration (area calculation) since this is a reiman sum, it is in seconds
double interval = 0.0;

double robot_radius = 0;
double wheel_diameter = 0;

MotionProfiler::MotionProfiler(double max_vel, double max_acc,
		double time_step) {

	max_velocity = max_vel;
	max_acceleration = max_acc;

	interval = time_step / time_dt; //frequency of when points should be recorded

}

MotionProfiler::MotionProfiler(double max_vel, double max_acc,
		double max_yaw_rate, double max_yaw_acc, double time_step) {

	max_velocity = max_vel;
	max_acceleration = max_acc;

	yaw_rate = max_yaw_rate;
	yaw_acc = max_yaw_acc;

	interval = time_step / time_dt; //frequency of when points should be recorded

}

//works off basic triangle geometry calculating times through area calculations under velocity time curves (acceleration is known and constant)
std::vector<std::vector<double> > MotionProfiler::CreateProfile1D(
		double init_pos, std::vector<double> waypoints) {

	double ref = 0;

	double acc = 0.0;
	double vel = 0.0;
	double pos = init_pos;

	double last_vel = 0.0;
	double last_pos = init_pos;

	double time = 0.0;

	int counter = 0;

	//cant initialize any vectors outside of the function or their previous values will carry over into the next profiles made. Don't pull a ChezyChamps2k17
	std::vector<std::vector<double> > matrix; //new matrix every time because .push_back adds rows, moved from the top of the class
	std::vector<double> positions; //first points will be 0
	std::vector<double> velocity;

	int length_waypoint = waypoints.size();

	for (int i = 0; i < length_waypoint; i++) { //will make a profile between each waypoint

		ref = waypoints.at(i);

		if (ref >= init_pos) {
			while (pos < ref) {

				ramp_time = vel / max_acceleration;
				ramp_dis = 0.5 * (vel * ramp_time);

				if ((ref - ramp_dis) <= pos) { //should
					acc = -1.0 * max_acceleration;
				} else if (vel < max_velocity) {
					acc = max_acceleration;
				} else {
					acc = 0.0;
				}

				pos = last_pos + (vel * time_dt);
				last_pos = pos;

				vel = last_vel + (acc * time_dt);
				last_vel = vel;

				counter++;
				time += time_dt; //time not used, can be used to graph (not supported)

				if (counter % (int) interval == 0) { //only add the points to the motion profile every so often to keep the profile at the desired tick rate
					positions.push_back(pos); //interval is the frequency of when points should be recorded.
					velocity.push_back(vel);
					counter = 0;
				}
			}
		} else if (ref < init_pos) {
			while (pos > ref) {

				ramp_time = vel / max_acceleration;
				ramp_dis = 0.5 * (vel * ramp_time);

				if ((ramp_dis - ref) >= pos) {
					acc = 1.0 * max_acceleration;
				} else if (vel > (-1.0 * max_velocity)) {
					acc = -1.0 * max_acceleration;
				} else {
					acc = 0.0;
				}

				pos = last_pos + (vel * time_dt);
				last_pos = pos;

				vel = last_vel + (acc * time_dt);
				last_vel = vel;

				counter++;
				time += time_dt; //time not used, can be used to graph (not supported)

				if (counter % (int) interval == 0) {
					positions.push_back(pos);
					velocity.push_back(vel);
					counter = 0;
				}
			}
		}

		init_pos = ref; //have to redefine the initial position after each waypoint as the waypoint (which is equal to the reference [ref])

	}

	matrix.push_back(positions); //first vector,  row 0
	matrix.push_back(velocity); //second vector, row 1

	return matrix;

}

//include 0,0 as first waypoint
std::vector<std::vector<double>> MotionProfiler::CreateWCProfile(
		std::vector<double> init_pos,
		std::vector<std::vector<double> > waypoints) {

	double max_yaw_rate = yaw_rate;

	std::vector<double> p1;
	std::vector<double> p2;

	double last_yaw = 0.0;
	double start_index = 0;

	double current_pos_right = init_pos.at(0);
	double current_pos_left = init_pos.at(1);
	double current_angle = init_pos.at(2);

	std::vector<std::vector<double> > profile;
	std::vector<std::vector<double> > profile_right;
	std::vector<std::vector<double> > profile_left;
	std::vector<std::vector<double> > profile_yaw;

	for (int i = 0; i < waypoints.size() - 1; i++) {

		//create 1D profile for turn
		//take that profile that is in radians and make it into rotations
		//prob jus find the distance the motor needs to turn and then make your profile

		p1 = waypoints.at(i);
		p2 = waypoints.at(i + 1);

		double yaw = FindAngle(p1, p2) - last_yaw; //or plus?
		double rotations = (2.0 * PI * robot_radius) * (1.0 / ((2 * PI) / yaw))
				* (1 / (wheel_diameter));

		std::vector<double> yaw_waypoints_right, yaw_waypoints_left;
		yaw_waypoints_right.push_back(rotations);
		yaw_waypoints_left.push_back(rotations * -1.0);

		last_yaw = yaw;

		std::vector<std::vector<double> > profile_right_yaw = CreateProfile1D(
				current_pos_right, yaw_waypoints_right);
		std::vector<std::vector<double> > profile_left_yaw = CreateProfile1D(
				current_pos_left, yaw_waypoints_left);

		//set the current position to the end of the turning profile
		current_pos_right = profile_right_yaw.at(profile_right_yaw.size() - 1);
		current_pos_left = profile_left_yaw.at(profile_left_yaw.size() - 1);

		//add the turning profile to the over all side specific profile
		profile_right.insert(profile_right.end(), profile_right_yaw.begin(),
				profile_right_yaw.end());
		profile_left.insert(profile_left.end(), profile_left_yaw.begin(),
				profile_left_yaw.end());

		//create 1D profile for move straight

		//find the distance between the points
		double distance = FindDistance(p1, p2);
		std::vector<double> distance_waypoints;
		distance_waypoints.push_back(distance);

		//create the profile for going straight to the point
		std::vector<std::vector<double> > profile_right_dis = CreateProfile1D(
				current_pos_right, distance_waypoints);
		std::vector<std::vector<double> > profile_left_dis = CreateProfile1D(
				current_pos_left, distance_waypoints);

		//set the current position to the last profiled position
		current_pos_right = profile_right_dis.at(profile_right_dis.size() - 1);
		current_pos_left = profile_left_dis.at(profile_left_dis.size() - 1);

		//add the profile to the right and left profiles
		profile_right.insert(profile_right.end(), profile_right_dis.begin(),
				profile_right_dis.end());
		profile_left.insert(profile_left.end(), profile_left_dis.begin(),
				profile_left_dis.end());

		//add to final profile
		for (int j = start_index; j < profile_right.size(); j++) {
			for (int k = 0; k < profile_right.at(0).size(); k++) {
				profile[j][k] = profile_right[j][k];
			}
		}

		for (int j = start_index; j < profile_left.size(); j++) {
			for (int k = profile_right.at(0).size; k < profile_left.at(0).size(); k++) {
				profile[j][k] = profile_left[j][k];
			}
		}

		start_index += profile_right.size();

	}

	return profile;

}

void MotionProfiler::SetRobotRadius(double length) {

	robot_radius = length;

}

void MotionProfiler::SetMaxYawRate(double rate) {

	yaw_rate = rate;

}

void MotionProfiler::SetWheelDiamter(double d) {

	wheel_diameter = d;

}

double MotionProfiler::FindDistance(std::vector<double> p1,
		std::vector<double> p2) {

	double y1 = p1.at(1);
	double x1 = p1.at(0);
	double y2 = p2.at(1);
	double x2 = p2.at(0);

	//formula for distance is sqrt((x2 - x1)^2 + (y2 - y1)^2)
	double distance_root = ((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1));
	double distance = sqrt(distance_root);

	return distance;

}

//returns the angle between two waypoints in radians
double MotionProfiler::FindAngle(std::vector<double> p1,
		std::vector<double> p2) {

	double y1 = p1.at(1);
	double x1 = p1.at(0);
	double y2 = p2.at(1);
	double x2 = p2.at(0);

	//inverse tangent used
	double angle = atan2(y1 - y2, x1 - x2) - (PI / 2.0); //angle is in radians

	return angle;

}

} /* namespace std */
