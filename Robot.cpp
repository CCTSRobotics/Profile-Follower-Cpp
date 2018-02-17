/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <Commands/ProfileFollower.h>
#include <string>
#include <Subsystems/DriveTrain.h>
#include <WPILib.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>

class Robot : public frc::TimedRobot {
public:
	void RobotInit() {

	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		SmartDashboard::PutString("DB/String 0","0");
		Command *AutoCommand = new ProfileFollower("/home/lvuser/Profiles/Test/l_left_detailed.csv",
				"/home/lvuser/Profiles/Test/l_right_detailed.csv");
				AutoCommand->Start();
	}

	void AutonomousPeriodic() {
		Scheduler::GetInstance()->Run();
	}

	void TeleopInit() {
		SmartDashboard::PutString("DB/String 0","0");

	}

	void TeleopPeriodic() {
				//SmartDashboard::PutData("Trial",AutoCommand);
		m_DriveTrain.RCArcadeDrive();
	}

	void TestInit(){

	}
	void TestPeriodic() {


	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
	DriveTrain m_DriveTrain{};
	Command *AutoCommand;
};

START_ROBOT_CLASS(Robot)
