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

//Drivetrain Constants
//const double Drivetrain::ENC_TICKS_PER_INCH = 41.9;		//Calibrated IncogSparks 8/25/19
//Encoder TICKS PER INCH Calibration
const double Drivetrain::LEFT_ENCODER_TPI  = 40.6;			//Calibrated IncogNeo 10/13/19
const double Drivetrain::RIGHT_ENCODER_TPI = 41.45;			//Calibrated IncogNeo 10/13/19

#define MAX_DRIVE       0.95


Drivetrain::Drivetrain() : Subsystem("Drivetrain") 
{
	//leftMotor         = new frc::Spark(0);
	//rightMotor        = new frc::Spark(1);

	m_leftNeoMaster   = new rev::CANSparkMax( 4, rev::CANSparkMax::MotorType::kBrushless );
	m_leftNeoSlave 	  = new rev::CANSparkMax( 5, rev::CANSparkMax::MotorType::kBrushless );
	m_rightNeoMaster  = new rev::CANSparkMax( 7, rev::CANSparkMax::MotorType::kBrushless );
	m_rightNeoSlave   = new rev::CANSparkMax( 6, rev::CANSparkMax::MotorType::kBrushless );

	differentialDrive = new frc::DifferentialDrive(*m_leftNeoMaster, *m_rightNeoMaster);

	rightEncoder      = new frc::Encoder(2, 3, false, frc::Encoder::k4X);
	leftEncoder       = new frc::Encoder(0, 1, true,  frc::Encoder::k4X);

	ahrs  			  = new AHRS(SPI::Port::kMXP);

	//Turn motor Safety off (!)
	differentialDrive->SetSafetyEnabled(false);


}

void Drivetrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new CmdDriveWithGamepad() );
}

// Put methods for controlling this subsystem
// here. Call these from Commands.




//**************************************************************
void Drivetrain::NeoSetup(void)
{
	std::cout << "Drivetrain::NeoSetup" << std::endl;
    //Follower
    m_leftNeoSlave->Follow(*m_leftNeoMaster);
    m_rightNeoSlave->Follow(*m_rightNeoMaster);

    //Inverted
    m_leftNeoMaster->SetInverted( false );
    m_leftNeoSlave->SetInverted( false );
    m_rightNeoMaster->SetInverted( false );
    m_rightNeoSlave->SetInverted( false );
}

//**************************************************************
void Drivetrain::DrivetrainPeriodic(void)
{
	
}

//Mentor drive limiter
#define MAX_DRIVE_POWER		0.8
#define MAX_TURN_POWER		0.95

//**************************************************************
void Drivetrain::DriveWithGamepad( void )
{
	const double DEADBAND = 0.08;
	
	double yL = Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_Y);
	double xL = Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_X);
	double yR = Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_Y);
	double xR = Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_X); 
	double tL = Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_L_TRIG);
	double tR = Robot::m_oi->GetDriverGamepad()->GetRawAxis(GAMEPADMAP_AXIS_R_TRIG);

	if (fabs(yL)<= DEADBAND) yL = 0;
	if (fabs(xL)<= DEADBAND) xL = 0;
	if (fabs(yR)<= DEADBAND) yR = 0;
	if (fabs(xR)<= DEADBAND) xR = 0;




	// //TankDrive
	// yL *= MAX_DRIVE_POWER;	
	// yR *= MAX_DRIVE_POWER;	
  	// differentialDrive->TankDrive( yL,  yR,  false);

	//Arcade Drive
	yL *= MAX_DRIVE_POWER;
	xR *= MAX_TURN_POWER;
	differentialDrive->ArcadeDrive(yL,-xR,  false);

}

//**************************************************************
void Drivetrain::Drive( double left, double right )
{
	double leftDrive  = left;
	double rightDrive = right;

    if ( left < -MAX_DRIVE )  leftDrive = -MAX_DRIVE;
    if ( left >  MAX_DRIVE )  leftDrive =  MAX_DRIVE;

    if ( right < -MAX_DRIVE )  rightDrive = -MAX_DRIVE;
    if ( right >  MAX_DRIVE )  rightDrive =  MAX_DRIVE;

	//Neg=Fwd.   Pos=Rev
	differentialDrive->TankDrive( (-1.0)*leftDrive,  (-1.0)*rightDrive,  false);
	//differentialDrive->TankDrive(0.0, 0.0, false);
}
//**************************************************************
void Drivetrain::Stop( void )
{
	differentialDrive->TankDrive(0.0, 0.0, false);
  	std::cout << "STOP!" << std::endl;
}
//**************************************************************
double Drivetrain::GetRightMotor(void)
{
	return m_rightNeoMaster->Get();
}
double Drivetrain::GetLeftMotor(void)
{
	return m_leftNeoMaster->Get();
}


//**************** TRANNY *********************


//**************** ENCODERS *********************
int Drivetrain::GetLeftEncoder(void)
{
	return -leftEncoder->GetRaw();
}
int Drivetrain::GetRightEncoder(void)
{
	return -rightEncoder->GetRaw();
}

void Drivetrain::ResetEncoders(void)
{
	leftEncoder->Reset();
	rightEncoder->Reset();
}


//**************** AHRS (NavX) *********************

bool Drivetrain::IsGyroConnected(void)
{
	return ahrs->IsConnected();
}
double Drivetrain::GetGyroYaw(void)
{
    //Returns Relative Yaw:  -180 to +180
	return (double) ahrs->GetYaw();
}
double Drivetrain::GetGyroAngle(void)
{
    //returns total accumulated angle -inf to +inf  (continuous through 360deg)
	return (double) ahrs->GetAngle();
}
double Drivetrain::GetGyroRate(void)
{
	return ahrs->GetRate();
}
void Drivetrain::ZeroGyro(void)
{
  	std::cout<<"ZeroGyro"<<std::endl;
	ahrs->ZeroYaw();
	//**OR
	//ahrs->Reset();//????
}



//**************** Robot Control *********************
double Drivetrain::V2P_calc( double velocity )
{
	// *** Velocity to power calculator ***
	// Velocity supplied in inches/sec
	// Power returned as a percentage output (0 - 1.0)
	// Calculated by taking a few runs at different powers
	//    and finding max velocity
	//  then solving formula for a line (y-mx=b)
	//  where x=power and y=velocity
	// *** Calibrated for IncogNeo 10/13/2019
	const double m = 250.0;		//Slope
	const double b = 7.5;		//Y-intercept

	//y=mx+b  ==>  x = (y-b)/m  
	double power = (velocity - b)/m;	//power calc
	if(power<0.0) power = 0;			//sanity check
	if(power>1.0) power = 1.0;			//sanity check
	return power;
}

