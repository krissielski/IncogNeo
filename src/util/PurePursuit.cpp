#include "Robot.h"
#include "PurePursuit.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <ctime>
#include <iomanip>      // std::setprecision


using namespace std;

//Paths to Profiles
#ifdef __WIN32__
#define PATH_TO_PROFILE ""
#define PATH_TO_LOGS    ""
#endif
#ifdef __linux__
//#define PATH_TO_PROFILE "/media/sda1/"            //USB Drive
#define PATH_TO_PROFILE "/home/lvuser/deploy/"      //User deploy directory
#define PATH_TO_LOGS    "/media/sda1/logs/"         //USB Drive
#endif

#define PINDENT "    "

#define NUM_PPARAM  3                               //Number of parameters to read from each profile line

//****** PARAMETERS WE NEED TO DEFINE
#define LOOKAHEAD_DISTANCE      25.0          //inches (radius)
#define ROBOT_TRACK_WIDTH       30

const double PI = 3.14159;

//MACROS
#define SQUARE(x)   ((x)*(x))
#define DEG2RAD(x)  ((x)*PI/180.0)



/*******************************
 ** PurePursuit (Constructor)
 *******************************/
PurePursuit::PurePursuit( std::string profile_filename )
{
    cout<<"PP Profile: " << profile_filename <<  endl;

    LoadProfile( profile_filename );

    //Init
    m_isPathDone  = false;
    m_isPathError = false;
    m_logfile = new std::ofstream();

}


/*******************************
 ** LoadProfile
 ** Parse path profile from file
 *******************************/
void PurePursuit::LoadProfile( std::string profile_filename )
{
    string   filename = PATH_TO_PROFILE + profile_filename;
    ifstream openfile;
    string   line;
    bool     first_line = true;

    profile_point_t ppoint;

    openfile.open(filename, ifstream::in );

	if( !openfile.is_open() )
    {
        std::cout<< PINDENT << "*** Could not open profile!!!!" <<std::endl;
        return;
    }

    //Read file reading each line in and parsing for parameters
    while (!openfile.eof())
    {

        //Read line from file
        getline(openfile,line);
        // cout<<line<<endl;

        //Are we are end of file?
        if(openfile.eof() )
        {
            cout<< PINDENT << "Imported " << m_profile.size() << " lines" << endl;
            return;
        }

        //Check for first line: Comment line
        if( first_line )
        {
            first_line = false;
            cout<< PINDENT << line;//  << endl;
            continue;
        }

        unsigned int line_index = 0;
        unsigned int col_index  = 0;
        string results[NUM_PPARAM];

        //Read each character of the line and place in proper bin
        while( (line_index < line.size() ) && (col_index<NUM_PPARAM) )
        {

            if( line[line_index] == ',' )                   //if hit ','
                col_index++;                                //move to next bin
            else if( line[line_index] == ' ' )              //ignore spaces
                ;   //pass
            else
                results[col_index] += line[line_index];     //add char to current bin

            line_index++;
        }


        //Sanity Check that we had the correct amount of data
        if( col_index != (NUM_PPARAM-1) )
        {
            cout << PINDENT << "**** Invalid Index Size!" << endl;
            return;
        }

        //Use a try-block to insure we don't crash trying to convert strings to float
        try
        {
            ppoint.x          = (double)stof(results[0]);
            ppoint.y          = (double)stof(results[1]);
            ppoint.velocity   = (double)stof(results[2]);
        }
        catch(...)
        {
            cout<< PINDENT << "***** Exception!!" << endl;
            return;
        }

        //Add new point to profile vector
        m_profile.push_back(ppoint);


        //cout << endl;
     }



    //Cleanup
    openfile.close();

}



/*******************************
 ** PrintProfile
 ** Output profile vector
 *******************************/
void PurePursuit::PrintProfile( void )
{
    for(unsigned int i=0; i< m_profile.size(); i++)
    {
        cout << m_profile[i].x << " ";
        cout << m_profile[i].y << " ";
        cout << m_profile[i].velocity << " ";
        cout << endl;
    }
    cout << endl;
}


/*******************************
 ** PurePursuitInit
 *******************************/
void PurePursuit::PurePursuitInit(void)
{
    cout<<"PP Init" << endl;

    LogfileOpen();
    m_logStartTime = frc::Timer::GetFPGATimestamp();

}
/*******************************
 ** PurePursuitEnd
 *******************************/
void PurePursuit::PurePursuitEnd(void)
{
    cout<<"PP End" << endl;
    LogfileClose();

}
/*******************************
 ** PurePursuitIsDone
 *******************************/
bool PurePursuit::PurePursuitIsDone(void)
{
    return m_isPathDone;
}
/*******************************
 ** PurePursuitIsError
 *******************************/
bool PurePursuit::PurePursuitIsError(void)
{
    return m_isPathError;
}


/*******************************
 ** PurePursuitPeriodic
 ** Run the PureProfile Algorithm
 *******************************/
void PurePursuit::PurePursuitPeriodic(void)
{

    //Sanity Check
    if( m_isPathDone || m_isPathError )
    {
        //Path done or is and error.  Exit immediately
        return;
    }

    //Running Pure Pursuit Algorithm
    unsigned int ppindex = FindClosestPoint();

    coord_t lookahead_pt = FindLookaheadPoint();

    //Check to see if we left path
    if( m_isPathError  ) return;
    
    double curvature     = CalcCurvature(lookahead_pt );


    //Get velocities
    double req_velocity  = m_profile[ppindex].velocity; //requested from profile
    double curr_velocity = Robot::m_odometry->GetVel(); //current

    //Velocity Rate Limiter
    const double VEL_RAMP_LIMIT = 20;    //(500/50);      //500 in/sec @ 50hz
    if( req_velocity > (curr_velocity + VEL_RAMP_LIMIT) )   req_velocity = curr_velocity + VEL_RAMP_LIMIT;

    //Calculate Left and Right target velocities
    //  +curve=Right turn = higher Left drive
    //  -curve=Left turn  = higher right drive
    double target_Lv = req_velocity * (2 + curvature*ROBOT_TRACK_WIDTH)/2;
    double target_Rv = req_velocity * (2 - curvature*ROBOT_TRACK_WIDTH)/2;

    double curr_Lv   = Robot::m_odometry->GetLVel();
    double curr_Rv   = Robot::m_odometry->GetRVel();


    //Get FF Power required for target velocity
    double calcLff = Robot::m_drivetrain->V2P_calc(target_Lv);
    double calcRff = Robot::m_drivetrain->V2P_calc(target_Rv);


    //Calculate FB parameters
    const double kP = 0.01;
    double calcLfb = kP * (target_Lv-curr_Lv);
    double calcRfb = kP * (target_Rv-curr_Rv);

    //Drive power = FF + FB
    double calcLdrive = calcLff + calcLfb;
    double calcRdrive = calcRff + calcRfb;

    //Constraints check.    0.1 < drive < 1.0
    //if( calcLdrive < 0.1 ) calcLdrive = 0.1;
    if( calcLdrive > 1.0 ) calcLdrive = 1.0;
    //if( calcRdrive < 0.1 ) calcRdrive = 0.1;
    if( calcRdrive > 1.0 ) calcRdrive = 1.0;



    // *** ToDo:
    //Lff, Rff
    //Lfb, Rfb
    //kV,  kA, kP

    //Finally, write to drivetrain
    Robot::m_drivetrain->Drive(calcLdrive,calcRdrive);

    // *******   LOG FILE ********************************************
    if( !m_logfile->is_open() ) return;


    //Gather some data 
    double curr_time = frc::Timer::GetFPGATimestamp() - m_logStartTime;
    double curr_x    = Robot::m_odometry->GetX();
    double curr_y    = Robot::m_odometry->GetY();



    *m_logfile << std::fixed << std::setprecision(3);

    *m_logfile << curr_time                                 << ","; // A:  Time
    *m_logfile << curr_x  <<","<< curr_y                    << ","; // BC: X,Y
    *m_logfile << Robot::m_drivetrain->GetGyroAngle()       << ","; // D:  Yaw
    *m_logfile << curr_Lv <<","<< curr_Rv                   << ","; // EF: Vel

    *m_logfile << ppindex                                  << ","; // G:   index
    *m_logfile <<  lookahead_pt.x <<","<< lookahead_pt.y   << ","; // HI:  lookahead x,y
    *m_logfile << curvature                                << ","; // J:   curve
    *m_logfile << target_Lv  <<","<< target_Rv             << ","; // KL:  target vel
    *m_logfile << calcLdrive <<","<< calcRdrive                  ; // MN:  calc drive



    *m_logfile << "\n";
}




/*******************************
 ** FindClosestPoint
 ** Find the closest point on the profile
 ** and return it's index
 *******************************/
unsigned int PurePursuit::FindClosestPoint(void)
{
    double x = Robot::m_odometry->GetX();
    double y = Robot::m_odometry->GetY();

    int    min_index = -1;
    double min_dist  = 1000000.0;    //Init very far


    //Find the distance to each point in the profile and store the shortest
    //Distance = sqrt(  (x1-x2)^2 + (y1-y2)^2 )
    for(unsigned int i=0; i< m_profile.size();i++)
    {
        double dist = sqrt( SQUARE(x-m_profile[i].x) + SQUARE(y-m_profile[i].y) );
        if( dist < min_dist )
        {
            min_dist  = dist;
            min_index = i;
        }
    }

    return min_index;
}


/*******************************
 ** FindLookaheadPoint
 ** Find the lookahead point where circle intercepts profile path
 ** Crazy math algorithm straight from Pure Pursuit paper
 ** Work backwards from end point so we always know first point found is forwards
 ** Returns (x,y) of point!!!?????
 *******************************/
coord_t PurePursuit::FindLookaheadPoint(void)
{

    coord_t d,f;
    coord_t intercept_pt;

    double curr_x = Robot::m_odometry->GetX();
    double curr_y = Robot::m_odometry->GetY();

    for(unsigned int i = m_profile.size()-2;  i>0;  i-- )
    {

        //Sanity check - check for rolled over count
        if( i > m_profile.size() )
        {
            m_isPathError = true;
            cout<< PINDENT << "*** Invalid LAP index" << endl;
            return {-1,-1}; //????
        }

        //L-E:  Profile line segment angle vector
        d.x = m_profile[i+1].x - m_profile[i].x;
        d.y = m_profile[i+1].y - m_profile[i].y;

        //E-C: Center of lookahead sphere to ray start vector
        f.x = m_profile[i].x - curr_x;
        f.y = m_profile[i].y - curr_y;

        //Crazy vector math!
        double a = SQUARE(d.x) + SQUARE(d.y);                               // d dot d
        double b = 2.0 * ( d.x * f.x + d.y * f.y);                          // 2 * d dot f
        double c = SQUARE(f.x) + SQUARE(f.y) - SQUARE(LOOKAHEAD_DISTANCE);  //f dot f - r^2

        double dis = SQUARE(b) - (4.0*a*c);                                 //Discriminant


        if( dis > 0 )
        {
            dis = sqrt(dis);
            double t1 = (-b - dis)/(2*a);
            double t2 = (-b + dis)/(2*a);


            if( (t1>=0) && (t1<=1) )
            {
//                cout<<"T1 Hit!" << endl;
//                cout<<"  index " << i << endl;
//                cout<<"  t     " << t1 << endl;

                //Check for End
                if( i == m_profile.size()-2 )
                    m_isPathDone = true;

                intercept_pt.x = m_profile[i].x + t1*d.x;
                intercept_pt.y = m_profile[i].y + t1*d.y;
                return intercept_pt;
            }
            if( (t2>=0) && (t2<=1) )
            {
//                cout<<"T2 Hit!" << endl;
//                cout<<"  index " << i << endl;
//                cout<<"  t     " << t2 << endl;

                //Check for End
                if( i == m_profile.size()-2 )
                    m_isPathDone = true;

                intercept_pt.x = m_profile[i].x + t2*d.x;
                intercept_pt.y = m_profile[i].y + t2*d.y;
                return intercept_pt;
            }
        }

    }

    //If here, no intersection was found.  Robot Left the path!
    m_isPathError = true;
    cout<< PINDENT << "*** No LAP Found!!" << endl;
    return {-1,-1}; 

}




/*******************************
 ** CalcCurvature
 ** Calculate the arc path required to navagate the robot toward the lookahead point
 ** Crazy math algorithm straight from Pure Pursuit paper
 ** Returns signed curvature (<0 = Left Turn;  >0 = Right Turn)
 ** (Note:  Curvature is 1/radius)
 *******************************/
double PurePursuit::CalcCurvature( coord_t aim_pt )
{

    double curr_x = Robot::m_odometry->GetX();
    double curr_y = Robot::m_odometry->GetY();

    //convert degrees to radians
    //Note:  Offset angle by 90 to account for axis rotaion of the field
    double heading_rad = DEG2RAD(90 - Robot::m_odometry->GetHeading() );


    double a = -tan( heading_rad );
    double c =  tan( heading_rad ) * curr_x - curr_y;
    double x =  fabs( a * aim_pt.x + aim_pt.y + c)/sqrt( SQUARE(a) + 1);

    //which side calc?
    double x_side = sin( heading_rad ) * ( aim_pt.x - curr_x );
    double y_side = cos( heading_rad ) * ( aim_pt.y - curr_y );
    double side   = (x_side-y_side) > 0.0 ? (1.0):(-1.0);

    return( side * (2*x)/SQUARE(LOOKAHEAD_DISTANCE) );
}



//*********************************************************
//Logfile handlers
void PurePursuit::LogfileOpen(void)
{
    std::string filename;
    char tbuf[100];

    //Get current time and format into a filename -> mmdd_HHMMSS
    std::time_t time = std::time(0); 
    std::tm*    ts   = std::localtime(&time);
    strftime(tbuf, sizeof(tbuf), "%m%d_%H%M%S", ts);

    //Generate path and filename into a string
    filename  = PATH_TO_LOGS;       //Start with directory
    filename += "pp";              //Add pp prefix
    //filename += tbuf;               //Add time file name  **Remove this during development**
    filename += ".csv";             //Add CSV as the extention

    //Finally, open the file
	m_logfile->open(filename, std::ios::out | std::ios::trunc );    

    if( !m_logfile->is_open() )
    {
        std::cout<<"*** Could NOT Open m_logfile!!!!"<<std::endl;
        return;
    }


    //Write Header
    *m_logfile << "Time"        << ","; // A:  Time
    *m_logfile << "X,Y"         << ","; // BC: X,Y
    *m_logfile << "Gyro"        << ","; // D:  Yaw
    *m_logfile << "Lv,Rv"       << ","; // EF: Vel
    *m_logfile << "ppindex"     << ","; // G:  index
    *m_logfile << "lapX,lapY"   << ","; // HI: lookahead x,y
    *m_logfile << "curve"       << ","; // J:  curve
    *m_logfile << "tLv,tRv"     << ","; // KL: target vel
    *m_logfile << "LD,RD"             ; // MN: calc drive

    *m_logfile << "\n";

}
void PurePursuit::LogfileClose(void)
{
    if( m_logfile->is_open() )
    {
		m_logfile->close();
        std::cout<<"m_logfile Closed"<<std::endl;
    }
}



