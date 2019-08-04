/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>

class Odometry : public frc::Subsystem {
private:

    int    prev_left_enc;
    int    prev_right_enc;
    double prev_timestamp;

    double curr_x;  //inches
    double curr_y;  //inches
    double curr_v;  //inches/sec



public:
    Odometry();
    void InitDefaultCommand() override;

    //Our Functions
    void    Periodic(void);
    void    Reset(void);

    double  GetX(void); //inches
    double  GetY(void);
    double  GetVel(void); //in/sec

    



};
