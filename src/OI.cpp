/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include "commands/CmdRunPurePursuit.h"
#include "commands/GrpVelocityRampTest.h"
#include "commands/CmdDriveClearAll.h"
#include "commands/GrpPathfindingTest.h"

OI::OI() 
{
    //Init Gamepad
    drivergamepad = new frc::Joystick(0);

    //SmartDashboard buttons
    frc::SmartDashboard::PutData("CmdDriveClearAll",    new CmdDriveClearAll( ) );
    frc::SmartDashboard::PutData("CmdRunPurePursuit",   new CmdRunPurePursuit( "output.csv") );
    frc::SmartDashboard::PutData("GrpVelocityRampTest", new GrpVelocityRampTest( ) );

    frc::SmartDashboard::PutData("GrpPathfindingTest", new GrpPathfindingTest( ) );


}

frc::Joystick* OI::GetDriverGamepad() {
  return drivergamepad;
}