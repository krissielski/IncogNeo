/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/GrpPathfindingTest.h"

#include "commands/CmdDriveManual.h"
#include "commands/CmdPrintAutoText.h"
#include "commands/CmdDriveFwdEncoder.h"
#include "commands/CmdDriveFwdGyro.h"
#include "commands/CmdDriveClearAll.h"
#include "commands/CmdLoggingEnable.h"

#include "commands/CmdRunPurePursuit.h"


GrpPathfindingTest::GrpPathfindingTest() 
{
	AddSequential(new CmdPrintAutoText("GrpPathfindingTest Begin"));
    AddSequential(new CmdDriveClearAll());
    //***************************************************

    AddSequential(new CmdRunPurePursuit( "LftRtTurn_pp.csv") );  
    AddSequential(new CmdRunPurePursuit( "LftRt_then_RtLftTurn_pp.csv") );  


    AddSequential(new frc::WaitCommand(1.0));           //Let it finish whatever it's doing
    AddSequential(new CmdLoggingEnable(false));
    //***************************************************
    AddSequential(new CmdDriveManual(0,0,0) );          //Safety.  All Off
    AddSequential(new CmdPrintAutoText("GrpPathfindingTest Complete"));
}