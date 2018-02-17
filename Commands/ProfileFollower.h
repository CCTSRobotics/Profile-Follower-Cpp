#ifndef ProfileFollower_H
#define ProfileFollower_H

#include <Joystick.h>
#include <WPILib.h>
#include <ctre/phoenix.h>
#include <Spark.h>
#include <AHRS.h>
#include <Pathfinder.h>
#include <iostream>
#include<string>

#include <fstream>
using namespace std;

class ProfileFollower : public Command {
private:
	double Circumference = 6*PI;
	WPI_VictorSPX LeftOne{3};
	WPI_VictorSPX LeftTwo{4};
	SpeedControllerGroup LeftMotors{LeftOne,LeftTwo};
	WPI_VictorSPX RightOne{2};
	TalonSRX Trial{0};
	WPI_VictorSPX RightTwo{1};
	SpeedControllerGroup RightMotors{RightOne,RightTwo};
	DifferentialDrive Train{LeftMotors,RightMotors};
	double lengthL;
	double lengthR;
	bool AmDone = false;
	double MAX_VELOCITY= 7;
	AHRS *gyro = new AHRS(SPI::kMXP);
	Encoder *LeftEncoder =  new Encoder(0, 1, false, Encoder::EncodingType::k4X);;
	Encoder *RightEncoder = new Encoder(8, 9, true, Encoder::EncodingType::k4X);
	Segment LeftTrajectory[1024];
	Segment RightTrajectory[1024];
	EncoderConfig LC;
	EncoderFollower *LeftFollower;
	EncoderFollower *RightFollower;
	EncoderConfig RC;
	void InitConfig(EncoderConfig &c, double pos,
			double dpp, double circ, double p, double i,
			double d, double v, double a);
public:
	ProfileFollower(const char *leftCSV, const char *rightCSV);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ProfileFollower_H
