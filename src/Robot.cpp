/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

//Subsystem Instantiation
OI *Robot::m_oi;
Drivetrain* Robot::m_drivetrain;

void Robot::RobotInit() {


    //*************************** INIT ******************************
    std::cout<<"RobotInit"<<std::endl;
    std::cout<<"IncogSparks: Offseason 2019"<<std::endl;
    std::cout<<"Version: " << __DATE__ <<"  "<<__TIME__<<std::endl<<std::endl; 


    //******Subsystems******
    m_drivetrain      = new Drivetrain();

    //OI **MUST** be after all subsystem constructors
    m_oi = new OI();

    //Subsystem Inits


}

void Robot::RobotPeriodic() 
{
    m_drivetrain->DriveWithGamepad(); 
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

}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
