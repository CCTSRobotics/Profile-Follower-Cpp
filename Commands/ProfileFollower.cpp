#include "ProfileFollower.h"
#include "WPILib.h"
#include <Pathfinder.h>

#include <ctre/phoenix.h>
ProfileFollower::ProfileFollower(const char *leftCSV, const char *rightCSV)
 {

	FILE *LeftProfile = fopen(leftCSV, "r");
	FILE *RightProfile = fopen(rightCSV, "r");




	 lengthL = pathfinder_deserialize_csv(LeftProfile, LeftTrajectory);
	 lengthR = pathfinder_deserialize_csv(RightProfile, RightTrajectory);
		LeftFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));
		RightFollower = (EncoderFollower*)malloc(sizeof(EncoderFollower));

	fclose(LeftProfile);
	fclose(RightProfile);


}

void ProfileFollower::InitConfig(EncoderConfig &c, double pos,double dpp,
		double circ, double p, double i, double d, double v, double a){
	c.initial_position= pos; c.ticks_per_revolution=dpp;c.wheel_circumference=circ;
	c.kp=p;c.ki=i;c.kd=d;c.kv=v;c.ka=a;
}
// Called just before this Command runs the first time
void ProfileFollower::Initialize() {
	gyro->ZeroYaw();
	RightEncoder->Reset();
	LeftEncoder->Reset();
	this->InitConfig(RC,RightEncoder->Get(),256*3,6*PI*.0254,1,0,0,1/MAX_VELOCITY,0);
	this->InitConfig(LC,LeftEncoder->Get(),256*3,6*PI*.0254,1,0,0,1/MAX_VELOCITY,0);
	LeftFollower->last_error=0;LeftFollower->segment=0;LeftFollower->finished=0;
	RightFollower->last_error=0;RightFollower->segment=0;RightFollower->finished=0;


}

// Called repeatedly when this Command is scheduled to run
void ProfileFollower::Execute() {
		double L = pathfinder_follow_encoder(LC, LeftFollower,  LeftTrajectory,  1024,LeftEncoder->Get() );
		double R = pathfinder_follow_encoder(RC, RightFollower,  RightTrajectory,  1024,RightEncoder->Get() );
		double Gyro_Heading = gyro->GetAngle();
		double Heading_Goal = r2d(LeftFollower->heading);
		double angle = Heading_Goal-Gyro_Heading;
		double turn = .8*(-1/80)*angle;
		SmartDashboard::PutNumber("Left Speed Prof",L);//+turn);
		SmartDashboard::PutNumber("Right Speed Prof",R);//-turn);

//		Train.TankDrive(L,R);
		LeftMotors.Set(L-turn);
		RightMotors.Set(-(R+turn));
		AmDone = L==0&&R==0;

}

// Make this return true when this Command no longer needs to run execute()
bool ProfileFollower::IsFinished() {
	return AmDone;
}

// Called once after isFinished returns true
void ProfileFollower::End() {
	LeftMotors.Set(0);
	RightMotors.Set(0);
	free(LeftTrajectory);
	free(RightTrajectory);
	free(LeftFollower);
	free(RightFollower);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ProfileFollower::Interrupted() {
	End();
}
