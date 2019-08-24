/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CmdLoggingEnable.h"
#include "Robot.h"

CmdLoggingEnable::CmdLoggingEnable(bool enable) {
  m_enable = enable;
}

// Called once when the command executes
void CmdLoggingEnable::Initialize() 
{
  Robot::m_logfile->LogFileEnable(m_enable);
}
