/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


class Odometry
{
private:

    int    m_prev_left_enc;
    int    m_prev_right_enc;
    double m_prev_timestamp;

    double m_curr_x;  //inches
    double m_curr_y;  //inches
    double m_curr_v;  //inches/sec
    double m_curr_Lv;
    double m_curr_Rv;



public:
    Odometry();


    //Our Functions
    void    OdometryPeriodic(void);
    void    Reset(void);

    double  GetX(void); //inches
    double  GetY(void);
    double  GetVel(void); //in/sec
    double  GetLVel(void); //in/sec
    double  GetRVel(void); //in/sec

    double  GetHeading(void); //degrees

};
