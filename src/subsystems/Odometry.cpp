/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Odometry.h"
#include "../Robot.h"
#include "math.h"

#define PI 3.14159265

Odometry::Odometry()
{
    std::cout<<"In Odometry"<<std::endl;
    std::cout<<"Odometry: Warning - simulated timestamp"<<std::endl;

    Reset();

    m_curr_x = 0;
    m_curr_y = 0;

    m_curr_v  = 0;
    m_curr_Lv = 0;
    m_curr_Rv = 0;

    m_prev_left_enc  = 0;
    m_prev_right_enc = 0;
    m_prev_timestamp = 0;

}


// Put methods for controlling this subsystem
// here. Call these from Commands.

//********************************************
void Odometry::OdometryPeriodic(void)
{

    //Read Encoders and find delta distance traveled in inches
    int left_enc  = Robot::m_drivetrain->GetLeftEncoder();
    int right_enc = Robot::m_drivetrain->GetRightEncoder();

    //SIMULATED TIMESTAMP
    //double timestamp = Robot::m_timer->GetFPGATimestamp();
    double timestamp = m_prev_timestamp + 0.020;  //20ms


    int delta_left_enc  = ( left_enc  - m_prev_left_enc);
    int delta_right_enc = ( right_enc - m_prev_right_enc);

    double distance = (delta_left_enc + delta_right_enc)/( 2.0 * Robot::m_drivetrain->ENC_TICKS_PER_INCH );

    //Calculate new X and Y based on Gyro
    double angle = Robot::m_drivetrain->GetGyroAngle();

    m_curr_x += distance * sin( (angle * PI)/180.0 );
    m_curr_y += distance * cos( (angle * PI)/180.0 );

    //Calculate Velocity
    double delta_time = timestamp - m_prev_timestamp;

    m_curr_v  =  distance / delta_time;
    m_curr_Lv =  delta_left_enc /(Robot::m_drivetrain->ENC_TICKS_PER_INCH*delta_time);
    m_curr_Rv =  delta_right_enc/(Robot::m_drivetrain->ENC_TICKS_PER_INCH*delta_time);

    //frc::SmartDashboard::PutNumber("delta d",  distance  );
    //frc::SmartDashboard::PutNumber("delta t",  delta_time );

    //Update parameters for next run
    m_prev_left_enc  = left_enc;
    m_prev_right_enc = right_enc;
    m_prev_timestamp = timestamp;


    // std::cout<< "od coord = " << m_curr_x <<" "<< m_curr_y << std::endl;
    // std::cout<< "od Vel   = " << m_curr_Lv <<" "<< m_curr_Rv << std::endl;


}

//********************************************
void Odometry::Reset(void)
{
    m_prev_left_enc  = 0;
    m_prev_right_enc = 0;
    m_prev_timestamp = 0.0;

    m_curr_x  = 0.0;
    m_curr_y  = 0.0;
    m_curr_v  = 0.0;
    m_curr_Lv = 0.0;
    m_curr_Rv = 0.0;

}

//********************************************
double  Odometry::GetX(void)
{
    return m_curr_x;
}
//********************************************
double  Odometry::GetY(void)
{
    return m_curr_y;
}
//********************************************
double  Odometry::GetVel(void)
{
    return m_curr_v;
}
//********************************************
double  Odometry::GetLVel(void)
{
    return m_curr_Lv;
}
//********************************************
double  Odometry::GetRVel(void)
{
    return m_curr_Rv;
}
//********************************************
double  Odometry::GetHeading(void)
{
    return Robot::m_drivetrain->GetGyroAngle();
}






