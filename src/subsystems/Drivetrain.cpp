/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "GamepadMap.h"
#include "subsystems/Drivetrain.h"

#include "commands/CmdDriveWithGamepad.h"

Drivetrain::Drivetrain() : Subsystem("Drivetrain") 
{
	leftMotor         = new frc::Spark(0);
	rightMotor        = new frc::Spark(1);
	differentialDrive = new frc::DifferentialDrive(*leftMotor, *rightMotor);
}

void Drivetrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new CmdDriveWithGamepad() );
}

// Put methods for controlling this subsystem
// here. Call these from Commands.



//**************************************************************
void Drivetrain::DriveWithGamepad( void )
{
	const double DEADBAND = 0.08;
	
	double yL = Robot::m_oi->GetGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_Y);
	double xL = Robot::m_oi->GetGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_X);
	double yR = Robot::m_oi->GetGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_Y);
	double xR = Robot::m_oi->GetGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_X); 
	double tL = Robot::m_oi->GetGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_TRIG);
	double tR = Robot::m_oi->GetGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_TRIG);

	if (fabs(yL)<= DEADBAND) yL = 0;
	if (fabs(xL)<= DEADBAND) xL = 0;
	if (fabs(yR)<= DEADBAND) yR = 0;
	if (fabs(xR)<= DEADBAND) xR = 0;


	//TankDrive
  differentialDrive->TankDrive( yL,  yR,  false);

	//Arcade Drive
	//differentialDrive->ArcadeDrive(yL,xR,  true);

}

//**************************************************************
void Drivetrain::Drive( double left, double right )
{
	//Neg=Fwd.   Pos=Rev
	differentialDrive->TankDrive( (-1.0)*left,  (-1.0)*right,  false);
}
void Drivetrain::Stop( void )
{
	differentialDrive->TankDrive(0.0, 0.0, false);
  	std::cout << "STOP!" << std::endl;
}

