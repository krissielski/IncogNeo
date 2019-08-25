
#include "commands/CmdDriveVelRampTest.h"
#include "subsystems/Drivetrain.h"
#include "Robot.h"

CmdDriveVelRampTest::CmdDriveVelRampTest(void) 
{
    Requires(Robot::m_drivetrain);

    frc::SmartDashboard::PutNumber("VRTest_Power",  0.5  );
    frc::SmartDashboard::PutNumber("VRTest_Time",   2.0  );
}

void CmdDriveVelRampTest::Initialize() 
{
    double power = frc::SmartDashboard::GetNumber("VRTest_Power",0.0);
    double time  = frc::SmartDashboard::GetNumber("VRTest_Time",0.0);

    Robot::m_drivetrain->Drive( power,  power );
	SetTimeout (time);
}


void CmdDriveVelRampTest::Execute() {}


bool CmdDriveVelRampTest::IsFinished() { 
     return IsTimedOut();
 }

// Called once after isFinished returns true
void CmdDriveVelRampTest::End() {
    Robot::m_drivetrain->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdDriveVelRampTest::Interrupted() 
{
    End();
}
