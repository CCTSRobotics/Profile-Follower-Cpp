/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <AnalogGyro.h>
#include <ctre/phoenix.h>
#include <Commands/Subsystem.h>
#include <Drive/DifferentialDrive.h>
#include <Encoder.h>
#include <WPILib.h>
#include <Spark.h>
#include "AHRS.h"
#include "Subsystems/EncoderGroup.h"
#include <SpeedControllerGroup.h>

namespace frc {
class Joystick;
}  // namespace frc

/**
 * The DriveTrain subsystem controls the robot's chassis and reads in
 * information about it's speed and position.
 */
class DriveTrain : public frc::Subsystem {
public:
	DriveTrain();

	/**
	 * When other commands aren't using the drivetrain, allow tank drive
	 * with
	 * the joystick.
	 */

	/**
	 * @param leftAxis Left sides value
	 * @param rightAxis Right sides value
	 */
	void TankDrive(double leftAxis, double rightAxis);

	void ArcadeDrive(double Y, double X);

	void RCArcadeDrive();
	void RCTankDrive();
	void RealTankDrive(double left, double right);

	void SingleStickArcadeDrive();


	/**
	 * Stop the drivetrain from moving.
	 */
	void Stop();

	/**
	 * @return The encoder getting the distance and speed of left side of
	 * the drivetrain.
	 */


	double GetLeftDistance();
	double GetLeftPulses();
	double GetLeftRate();
	/**
	 * @return The encoder getting the distance and speed of right side of
	 * the drivetrain.
	 */

	double GetRightDistance();
	double GetRightPulses();
	double GetRightRate();

	double GetAveDistance();
	double GetAvePulses();
	double GetAveRate();

	/**
	 * @return The current angle of the drivetrain.
	 */
	double GetAngle();

private:
	// Subsystem devices

	Joystick m_stick{0};



	WPI_VictorSPX m_frontLeftCIM{1};
	WPI_VictorSPX m_rearLeftCIM{2};
	frc::SpeedControllerGroup m_leftCIMs{m_frontLeftCIM, m_rearLeftCIM};

	WPI_VictorSPX m_frontRightCIM{3};
	WPI_VictorSPX m_rearRightCIM{4};
	frc::SpeedControllerGroup m_rightCIMs{m_frontRightCIM, m_rearRightCIM};

	frc::DifferentialDrive m_robotDrive{m_leftCIMs, m_rightCIMs};

	frc::Encoder m_RightEncoder{1, 2, true, frc::Encoder::k4X};
	frc::Encoder m_LeftEncoder{8, 9, false, frc::Encoder::k4X};
	EncoderGroup m_AveEncoder{&m_RightEncoder,&m_LeftEncoder};

	AHRS *m_gyro = new AHRS{frc::SPI::Port::kMXP};
};
