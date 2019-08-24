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
const double Drivetrain::ENC_TICKS_PER_INCH = 40.0;

#define MAX_DRIVE       0.95


Drivetrain::Drivetrain() : Subsystem("Drivetrain") 
{
	leftMotor         = new frc::Spark(0);
	rightMotor        = new frc::Spark(1);
	differentialDrive = new frc::DifferentialDrive(*leftMotor, *rightMotor);

	rightEncoder      = new frc::Encoder(0, 1, true , frc::Encoder::k4X);
	leftEncoder       = new frc::Encoder(2, 3, false, frc::Encoder::k4X);

	//Turn motor Safety off (!)
	differentialDrive->SetSafetyEnabled(false);

	//Simulation
	std::cout<<"Drivetrain: Warning - simulated drivetrain"<<std::endl;
    m_Ldrive = 0;
    m_Rdrive = 0;

    m_Lencoder = 0;
    m_Rencoder = 0;

    m_sim_Lvelocity = 0;
    m_sim_Rvelocity = 0;
    m_sim_distance = 0;
    m_sim_Lencoder = 0;
    m_sim_Rencoder = 0;

}

void Drivetrain::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new CmdDriveWithGamepad() );
}

// Put methods for controlling this subsystem
// here. Call these from Commands.


//Simulation constants
#define PROFILE_RATE    50                              //hertz
#define PROFILE_PERIOD  (1/PROFILE_RATE)                //seconds

#define MAX_ACCEL_RATE  200                             //inches per second per second
#define MAX_ACCEL_TICK  (MAX_ACCEL_RATE/PROFILE_RATE)   //inches per second per tick

//**************************************************************
void Drivetrain::DrivetrainPeriodic(void)
{
	
    //Calc New Velocity....
    double drive_Lv = fabs(m_Ldrive) >= 0.5 ?  (m_Ldrive-0.5)*200 : 0;
    double drive_Rv = fabs(m_Rdrive) >= 0.5 ?  (m_Rdrive-0.5)*200 : 0;

    double delta_Lv = drive_Lv - m_sim_Lvelocity;
    double delta_Rv = drive_Rv - m_sim_Rvelocity;

    double calc_Ldv = 0;
    double calc_Rdv = 0;

    if      ( delta_Lv < -MAX_ACCEL_TICK )  calc_Ldv = -MAX_ACCEL_TICK;
    else if ( delta_Lv >  MAX_ACCEL_TICK )  calc_Ldv =  MAX_ACCEL_TICK;
    else                                    calc_Ldv =  delta_Lv;

    if      ( delta_Rv < -MAX_ACCEL_TICK )  calc_Rdv = -MAX_ACCEL_TICK;
    else if ( delta_Rv >  MAX_ACCEL_TICK )  calc_Rdv =  MAX_ACCEL_TICK;
    else                                    calc_Rdv =  delta_Rv;

    m_sim_Lvelocity += (calc_Ldv);
    m_sim_Rvelocity += (calc_Rdv);

    m_sim_distance += (m_sim_Lvelocity + m_sim_Rvelocity)/(2*PROFILE_RATE) ;

    m_sim_Lencoder += (m_sim_Lvelocity * ENC_TICKS_PER_INCH)/PROFILE_RATE;
    m_sim_Rencoder += (m_sim_Rvelocity * ENC_TICKS_PER_INCH)/PROFILE_RATE;

    //Encoders update based on velocity
    m_Lencoder = (int)m_sim_Lencoder;
    m_Rencoder = (int)m_sim_Rencoder;

    // //**** DEBUG ****
    // std::cout<< "dt drive = " << m_Ldrive <<" "<< m_Rdrive << std::endl;
    // std::cout<< "dt drive = " << drive_Lv <<" "<< drive_Rv << std::endl;
    // std::cout<< "dt calc  = " << calc_Ldv <<" "<< calc_Rdv << std::endl;
    // std::cout<< "dt dist  = " << m_sim_distance  << std::endl;

    // std::cout<< "dt Veloc = " << m_sim_Lvelocity <<" "<< m_sim_Rvelocity << std::endl;
    // std::cout<< "dt Enc   = " << m_sim_Lencoder <<" "<< m_sim_Rencoder << std::endl;
    // std::cout<< "dt Enc   = " << m_Lencoder <<" "<< m_Rencoder << std::endl;

}

//Mentor drive limiter
#define MAX_DRIVE_POWER		0.8
#define MAX_TURN_POWER		0.8

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
	differentialDrive->ArcadeDrive(yL,-xR,  true);

}

//**************************************************************
void Drivetrain::Drive( double left, double right )
{

    if      ( left < -MAX_DRIVE )  m_Ldrive = -MAX_DRIVE;
    else if ( left >  MAX_DRIVE )  m_Ldrive =  MAX_DRIVE;
    else                           m_Ldrive =  left;

    if      ( right < -MAX_DRIVE )  m_Rdrive = -MAX_DRIVE;
    else if ( right >  MAX_DRIVE )  m_Rdrive =  MAX_DRIVE;
    else                            m_Rdrive =  right;

	//Neg=Fwd.   Pos=Rev
	differentialDrive->TankDrive( (-1.0)*left,  (-1.0)*right,  false);
	//differentialDrive->TankDrive(0.0, 0.0, false);
}
//**************************************************************
void Drivetrain::Stop( void )
{
	differentialDrive->TankDrive(0.0, 0.0, false);
  	std::cout << "STOP!" << std::endl;

	m_Ldrive = 0;
	m_Rdrive = 0;
}
//**************************************************************
double Drivetrain::GetRightMotor(void)
{
	return rightMotor->Get();
	//return m_Ldrive;
}
double Drivetrain::GetLeftMotor(void)
{
	return leftMotor->Get();
	//return m_Rdrive;
}


//**************** TRANNY *********************


//**************** ENCODERS *********************
int Drivetrain::GetLeftEncoder(void)
{
	return -leftEncoder->GetRaw();
	//return m_Lencoder;
}
int Drivetrain::GetRightEncoder(void)
{
	return -rightEncoder->GetRaw();
	//return m_Rencoder;
}

void Drivetrain::ResetEncoders(void)
{
	leftEncoder->Reset();
	rightEncoder->Reset();
    m_Lencoder = 0;
    m_Lencoder = 0;

}


//**************** AHRS (NavX) *********************

bool Drivetrain::IsGyroConnected(void)
{
	//return ahrs->IsConnected();
	return true;
}
double Drivetrain::GetGyroYaw(void)
{
    //Returns Relative Yaw:  -180 to +180
	//return (double) ahrs->GetYaw();
	return 0.0;
}
double Drivetrain::GetGyroAngle(void)
{
    //returns total accumulated angle -inf to +inf  (continuous through 360deg)
	//return (double) ahrs->GetAngle();
	return 0.0;
}


double Drivetrain::GetGyroRate(void)
{
	//return ahrs->GetRate();
	return 0.0;
}
void Drivetrain::ZeroGyro(void)
{
  	std::cout<<"ZeroGyro"<<std::endl;
	//ahrs->ZeroYaw();
	//**OR
	//ahrs->Reset();//????
}
