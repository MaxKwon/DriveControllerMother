#include <iostream>
#include <memory>
#include <string>
#include <WPILib.h>
#include <CANTalon.h>
#include<vector>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public frc::IterativeRobot {
public:

	double A[2][2] = {{0 , 1}, {0 , 1}};

	void RobotInit() {

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {


	}

	void TeleopPeriodic() {

	}

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
