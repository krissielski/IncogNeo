/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "GamepadMap.h"
#include "util/PurePursuit.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>


//Subsystem Instantiation
OI *Robot::m_oi;
Drivetrain *Robot::m_drivetrain;
Odometry   *Robot::m_odometry;


frc::Timer *Robot::m_timer;

//local function prototypes
void Write2Dashboard(void);

void Robot::RobotInit() {


    //*************************** INIT ******************************
    std::cout<<"RobotInit"<<std::endl;
    std::cout<<"IncogSparks: Offseason 2019"<<std::endl;
    std::cout<<"Version: " << __DATE__ <<"  "<<__TIME__<<std::endl<<std::endl; 


    //******Subsystems******
    m_drivetrain    = new Drivetrain();
    m_odometry      = new Odometry();

    //OI **MUST** be after all subsystem constructors
    m_oi = new OI();
    m_timer = new frc::Timer();

    //Subsystem Inits
    m_odometry->Reset();


}

void Robot::RobotPeriodic() 
{
    //m_drivetrain->DriveWithGamepad(); 
    m_drivetrain->DrivetrainPeriodic();
    m_odometry->OdometryPeriodic();

    Write2Dashboard();
}


void Robot::DisabledInit() 
{
    std::cout<<"Disabled Init"<<std::endl;
}

void Robot::DisabledPeriodic() 
{ 

}


void Robot::AutonomousInit() 
{
    std::cout<<"Auto Init"<<std::endl;
    std::cout<<"****NOTHING TO DO!!!*****"<<std::endl;
}

void Robot::AutonomousPeriodic() 
{ 
    frc::Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
    // Must kill any lingering autonomous commands here

    std::cout<<"Teleop Init"<<std::endl;

    m_drivetrain->ResetEncoders();
    m_odometry->Reset();

}

void Robot::TeleopPeriodic() 
{

    frc::Scheduler::GetInstance()->Run(); 
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif



void Write2Dashboard(void)
{

    frc::SmartDashboard::PutNumber("L_Motor",  Robot::m_drivetrain->GetLeftMotor()  );
    frc::SmartDashboard::PutNumber("R_Motor",  Robot::m_drivetrain->GetRightMotor()  );

    frc::SmartDashboard::PutNumber("D_L_Y_axis",  Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_Y)  );
    frc::SmartDashboard::PutNumber("D_R_Y_axis",  Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_Y)  );
    frc::SmartDashboard::PutNumber("D_L_X_axis",  Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_X)  );
    frc::SmartDashboard::PutNumber("D_R_X_axis",  Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_X)  );

    frc::SmartDashboard::PutNumber("D_L_Trig",    Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_TRIG)  );
    frc::SmartDashboard::PutNumber("D_R_Trig",    Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_TRIG)  );

	frc::SmartDashboard::PutNumber("LeftEnc",    Robot::m_drivetrain->GetLeftEncoder());
	frc::SmartDashboard::PutNumber("RightEnc",   Robot::m_drivetrain->GetRightEncoder());  


	frc::SmartDashboard::PutBoolean("navx_IsConn", Robot::m_drivetrain->IsGyroConnected() );
	frc::SmartDashboard::PutNumber("navx_Yaw",     Robot::m_drivetrain->GetGyroYaw() );
   	frc::SmartDashboard::PutNumber("navx_Angle",   Robot::m_drivetrain->GetGyroAngle() );
 
    frc::SmartDashboard::PutNumber("navx_Rate",    Robot::m_drivetrain->GetGyroRate() );

    frc::SmartDashboard::PutNumber("Curr_X",    Robot::m_odometry->GetX() );
    frc::SmartDashboard::PutNumber("Curr_Y",    Robot::m_odometry->GetY() );
    frc::SmartDashboard::PutNumber("Curr_Vel",  Robot::m_odometry->GetVel() );


    //Time
    //frc::SmartDashboard::PutNumber("FPGATime2",  Robot::m_timer->GetFPGATimestamp() );   //(double) sec
    //frc::SmartDashboard::PutNumber("FPGATime1",  frc::Timer::GetFPGATimestamp() );   //(double) sec
    //frc::SmartDashboard::PutNumber("Timer",      Robot::m_timer->Get() );                //Manual Timer sec




}