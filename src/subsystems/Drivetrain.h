/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc/WPILib.h"
#include "AHRS.h"
#include "rev/CANSparkMax.h"

class Drivetrain : public frc::Subsystem {
private:

    //Components
    //frc::SpeedController    *leftMotor;
    //frc::SpeedController    *rightMotor;
    rev::CANSparkMax *m_leftNeoMaster;
    rev::CANSparkMax *m_leftNeoSlave ;
    rev::CANSparkMax *m_rightNeoMaster;
    rev::CANSparkMax *m_rightNeoSlave;

    frc::DifferentialDrive  *differentialDrive;

    frc::Encoder            *rightEncoder;
    frc::Encoder            *leftEncoder;    

    AHRS *ahrs;	    //NavX



public:
    Drivetrain();
    void InitDefaultCommand() override;

    //Drivetrain Constants
    //const static double ENC_TICKS_PER_INCH;
    const static double LEFT_ENCODER_TPI;
    const static double RIGHT_ENCODER_TPI;

    //Our Functions
    void DrivetrainPeriodic(void);
    void NeoSetup(void);


    //Drive
    void   DriveWithGamepad( void );
    void   Drive( double left, double right );
    void   Stop( void );
    double GetRightMotor(void);
    double GetLeftMotor(void);

    //Encoders
	int  GetLeftEncoder(void);
	int  GetRightEncoder(void);
	void ResetEncoders(void);
    
    //NavX
	bool   IsGyroConnected(void);
	double GetGyroYaw(void);            //yaw: Relative -180 to +180
	double GetGyroAngle(void);          //angle: absolute -inf to +inf
	double GetGyroRate(void);
	void   ZeroGyro(void);

    //Robot Control
    double V2P_calc( double velocity ); //velocity = inches/sec

};
