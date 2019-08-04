/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Odometry.h"
#include "Robot.h"
#include "math.h"

#define PI 3.14159265

Odometry::Odometry() : Subsystem("Odometry") 
{
}

void Odometry::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

//********************************************
void Odometry::Periodic(void)
{

    //Read Encoders and find delta distance traveled in inches
    int left_enc  = Robot::m_drivetrain->GetLeftEncoder();
    int right_enc = Robot::m_drivetrain->GetRightEncoder();
    double timestamp = Robot::m_timer->GetFPGATimestamp();    

    int delta_left_enc  = ( left_enc  - prev_left_enc);
    int delta_right_enc = ( right_enc - prev_right_enc);

    double distance = (delta_left_enc + delta_right_enc)/( 2.0 * Robot::m_drivetrain->ENC_TICKS_PER_INCH );

    //Calculate new X and Y based on Gyro
    double angle = Robot::m_drivetrain->GetGyroAngle();

    curr_x += distance * sin( (angle * PI)/180.0 );
    curr_y += distance * cos( (angle * PI)/180.0 );

    //Calculate Velocity
    double delta_time = timestamp-prev_timestamp;

    curr_v =  distance / delta_time;



    //Update parameters for next run
    prev_left_enc  = left_enc;
    prev_right_enc = right_enc;
    prev_timestamp = timestamp;

}

//********************************************
void Odometry::Reset(void)
{
    prev_left_enc  = 0;
    prev_right_enc = 0;
    prev_timestamp = 0.0;

    curr_x = 0.0;
    curr_y = 0.0;
    curr_v = 0.0;

}

//********************************************
double  Odometry::GetX(void)
{
    return curr_x;
}
//********************************************
double  Odometry::GetY(void)
{
    return curr_y;
}
//********************************************
double  Odometry::GetVel(void)
{
    return curr_v;
}