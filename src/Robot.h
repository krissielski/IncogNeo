/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "OI.h"

//Subsystems
#include "subsystems/Drivetrain.h"
#include "subsystems/Odometry.h"


#include <iostream>	
#include <string>

class Robot : public frc::TimedRobot {
 public:
    //Subsystems
    static OI *m_oi;
    static Drivetrain *m_drivetrain;
    static Odometry   *m_odometry;

    static frc::Timer *m_timer;

    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;



 private:


};
