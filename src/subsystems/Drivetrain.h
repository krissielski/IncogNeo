/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc/WPILib.h"

class Drivetrain : public frc::Subsystem {
private:

    //Components
    frc::SpeedController    *leftMotor;
    frc::SpeedController    *rightMotor;
    frc::DifferentialDrive  *differentialDrive;

    frc::Encoder            *rightEncoder;
    frc::Encoder            *leftEncoder;    

    double encsim_time;
    
public:
    Drivetrain();
    void InitDefaultCommand() override;

    //Drivetrain Constants
    const static double ENC_TICKS_PER_INCH;

    //Our Functions


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

};
