#include <iostream>
#include <memory>
#include <string>
#include <WPILib.h>
#include <CANTalon.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public frc::IterativeRobot {
public:
	CANTalon cantalon;

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
