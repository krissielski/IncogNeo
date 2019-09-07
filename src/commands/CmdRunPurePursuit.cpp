/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CmdRunPurePursuit.h"
#include "Robot.h"

using namespace std;


CmdRunPurePursuit::CmdRunPurePursuit( std::string  profile_filename )
{

    cout << "CmdRunPurePursuit: "<< profile_filename << endl;

    pp = new PurePursuit(profile_filename);


    // Use Requires() here to declare subsystem dependencies
    Requires( Robot::m_drivetrain);

}

// Called just before this Command runs the first time
void CmdRunPurePursuit::Initialize() 
{
    cout << "CmdRunPurePursuit Init"<<endl;
    pp->PurePursuitInit();

}

// Called repeatedly when this Command is scheduled to run
void CmdRunPurePursuit::Execute() 
{
    pp->PurePursuitPeriodic();
}

// Make this return true when this Command no longer needs to run execute()
bool CmdRunPurePursuit::IsFinished() 
{ 
    if( pp->PurePursuitIsDone() )
    {
       cout << " Path at end"<<endl;
       return true;
    }
    if( pp->PurePursuitIsError() )
    {
        cout << " ** ABORT **  Path Error!"<<endl;

        //** TODO:  Abort Autonomous code on path error!!!!! ***********************
        //robot::auto_abort = true;
        return true;
    }
    return false;
}

// Called once after isFinished returns true
void CmdRunPurePursuit::End() 
{
    cout << "CmdRunPurePursuit END"<<endl;
    Robot::m_drivetrain->Stop();
    pp->PurePursuitEnd();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdRunPurePursuit::Interrupted() 
{
    cout << "CmdRunPurePursuit Interrupted"<<endl;
    End();
}
