/*
 * MotionProfiler.h
 *
 *  Created on: Jan 3, 2018
 *      Author: maxkwon
 */


#include <list>
#include <vector>
#include <iostream>
#include <cmath>

#ifndef SRC_MOTIONPROFILER_H_
#define SRC_MOTIONPROFILER_H_

namespace std {

class MotionProfiler {
public:

	//maximum velocity acceleration of the arm or system and the time step for the controller is needed
	MotionProfiler(double max_vel, double max_acc, double time_step);
	MotionProfiler(double max_vel, double max_acc, double max_yaw_rate, double max_yaw_acc, double time_step);

	//will generate a 1 dimensional profile for appendages
	std::vector<std::vector<double> > CreateProfile1D(double init_pos, std::vector<double> waypoints);

	double FindAngle(std::vector<double> p1, std::vector<double> p2);
	double FindDistance(std::vector<double> p1, std::vector<double> p2);

	//for generating 2 dimensional profiles for robot drives (specifically west coast.
	//Abstract method that will have to be written in the inherited class
	std::vector<std::vector<double>> CreateWCProfile(std::vector<double> init_pos, std::vector<std::vector<double> > waypoints) = 0;

	void SetMaxYawRate(double rate);
	void SetRobotRadius(double length);
	void SetWheelDiamter(double d);



};

} /* namespace std */

#endif /* SRC_MOTIONPROFILER_H_ */
