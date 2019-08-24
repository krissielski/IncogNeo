#include "Robot.h"
#include "LogFile.h"


#include <cmath>
#include <string>
#include <ctime>
#include <iomanip>      // std::setprecision

using namespace std;

#define LOGFILEDIRNAME "/media/sda1/logs/"          //mounted USB Drive

//Static File handler
static std::ofstream logfile;


/*******************************
 ** LogFile (Constructor)
 *******************************/
LogFile::LogFile( void )
{

    cout<<"In LogFile" << endl;


}

/*******************************
 ** LogFilePeriodic
 *******************************/
void LogFile::LogFilePeriodic( void )
{
    if( !logfile.is_open() ) return;


    //Gather some data 
    double curr_time = frc::Timer::GetFPGATimestamp() - m_logStartTime;
    //double curr_x    = Robot::m_odometry->GetX();
    //double curr_y    = Robot::m_odometry->GetY();

    logfile << std::fixed << std::setprecision(3);

    logfile << curr_time                                 << ","; // 1:  ms since start
   
    logfile << Robot::m_drivetrain->GetGyroAngle()      << ","; // 4:  Yaw
    logfile << Robot::m_drivetrain->GetLeftMotor()      << ","; // 5:  LeftMotor
    logfile << Robot::m_drivetrain->GetRightMotor()     << ","; // 6:  RightMotor
    logfile << Robot::m_drivetrain->GetLeftEncoder()    << ","; // 7:  Left Encoder
    logfile << Robot::m_drivetrain->GetRightEncoder()   << ","; // 8:  Right Encoder
//    logfile << Robot::m_drivetrain->GetLeftEncVel()     << ","; // 9:  Left Encoder Velocity
//    logfile << Robot::m_drivetrain->GetRightEncVel()    << ","; // 10: Right Encoder Velocity


    //Must be last
    logfile << "\n";
}



/*******************************
 ** LogFileEnable
 *******************************/
void LogFile::LogFileEnable( bool enable )
{
    if( enable ) LogfileOpen();
    else         LogfileClose();
}

/*******************************
 ** LogFileStop
 *******************************/
void LogFile::LogFileStop( void )
{

}


/*******************************
 ** LogfileOpen (private)
 *******************************/
void LogFile::LogfileOpen(void)
{
    if( logfile.is_open() ) return;


    std::string filename;
    char tbuf[100];

    //Get current time and format into a filename -> mmdd_HHMMSS
    std::time_t time = std::time(0); 
    std::tm*    ts   = std::localtime(&time);
    strftime(tbuf, sizeof(tbuf), "%m%d_%H%M%S", ts);

    //Generate path and filename into a string
    filename  = LOGFILEDIRNAME;     //Start with directory
    filename += "log_";             //Add log prefix    
    filename += tbuf;               //Add time file name
    filename += ".csv";             //Add CSV as the extention

    //Finally, open the file
	logfile.open(filename, std::ios::out | std::ios::trunc );

	if( logfile.is_open() )
    {
        std::cout<<"LogFile Opened: " << tbuf <<std::endl;
        //Write header
        logfile << "Time "      << ","; // 1:  ms since start

        logfile << "Yaw"        << ","; // 4:  Yaw
        logfile << "LMotor"     << ","; // 5:  LeftMotor
        logfile << "RMotor"     << ","; // 6:  RightMotor
        logfile << "LEnc"       << ","; // 7:  Left Encoder
        logfile << "REnc"       << ","; // 8:  Right Encoder
        logfile << "LEncV"      << ","; // 9:  Left Encoder Velocity
        logfile << "REncV"      << ","; // 10: Right Encoder Velocity

        //Must be last
        logfile << "\n";
    }     
	else
        std::cout<<"*** Could NOT Open Logfile!!!!"<<std::endl;
		

    m_logStartTime = frc::Timer::GetFPGATimestamp();

}
/*******************************
 ** LogfileClose (private)
 *******************************/
void LogFile::LogfileClose(void)
{
    if( !logfile.is_open() ) return;

    logfile.close();
    cout<<"LogFile Closed"<<endl;
}