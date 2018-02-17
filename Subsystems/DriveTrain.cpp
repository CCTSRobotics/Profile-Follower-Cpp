/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriveTrain.h"

#include <cmath>
#include <CamdenCountyLibrary.h>
#include <WPILib.h>
#include <Joystick.h>


DriveTrain::DriveTrain()
    : frc::Subsystem("DriveTrain") {
	// AddChild("Front Left CIM", m_frontLeftCIM);
	// AddChild("Front Right CIM", m_frontRightCIM);
	// AddChild("Back Left CIM", m_backLeftCIM);
	// AddChild("Back Right CIM", m_backRightCIM);

	// Configure the DifferentialDrive to reflect the fact that all our
	// motors are wired backwards and our drivers sensitivity preferences.
	m_robotDrive.SetSafetyEnabled(false);
	m_robotDrive.SetExpiration(0.1);
	m_robotDrive.SetMaxOutput(1.0);
//	m_leftCIMs.SetInverted(true);
//	m_rightCIMs.SetInverted(true);

	// Configure encoders
	m_RightEncoder.SetPIDSourceType(PIDSourceType::kDisplacement);
	m_LeftEncoder.SetPIDSourceType(PIDSourceType::kDisplacement);

}


void DriveTrain::TankDrive(double leftAxis, double rightAxis) {
	m_robotDrive.TankDrive(leftAxis, rightAxis);
}

void DriveTrain::ArcadeDrive(double Y, double X) {
	m_robotDrive.ArcadeDrive(Y, X);
}

void DriveTrain::RCArcadeDrive() {
	m_robotDrive.ArcadeDrive(m_stick.GetRawAxis(LY), m_stick.GetRawAxis(RX));
}

void DriveTrain::RCTankDrive() {
	m_robotDrive.TankDrive(m_stick.GetRawAxis(LY), m_stick.GetRawAxis(RY));
}
void DriveTrain::RealTankDrive(double left, double right) {
	m_leftCIMs.Set(left);
	m_rightCIMs.Set(right);
}


void DriveTrain::SingleStickArcadeDrive() {
	m_robotDrive.ArcadeDrive(m_stick.GetRawAxis(LY), m_stick.GetRawAxis(LX));
}


void DriveTrain::Stop() {
	m_robotDrive.TankDrive(0.0, 0.0);
}

double DriveTrain::GetLeftDistance(){
	return m_LeftEncoder.GetDistance();
}
double DriveTrain::GetLeftPulses(){
	return m_LeftEncoder.Get();
}
double DriveTrain::GetLeftRate(){
	return m_LeftEncoder.GetRate();
}

double DriveTrain::GetRightDistance(){
	return m_RightEncoder.GetDistance();
}
double DriveTrain::GetRightPulses(){
	return m_RightEncoder.Get();
}
double DriveTrain::GetRightRate(){
	return m_RightEncoder.GetRate();
}

double DriveTrain::GetAveDistance(){
	return m_AveEncoder.GetDistance();
}
double DriveTrain::GetAvePulses(){
	return m_AveEncoder.Get();
}
double DriveTrain::GetAveRate(){
	return m_AveEncoder.GetRate();
}


double DriveTrain::GetAngle() {
	return m_gyro->GetAngle();
}
