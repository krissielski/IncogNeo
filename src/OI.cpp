/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"


OI::OI() 
{
    //Init Gamepad
    drivergamepad = new frc::Joystick(0);


}


frc::Joystick* OI::GetDriverGamepad() {
  return drivergamepad;
}