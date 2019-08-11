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

//****** PARAMETERS WE NEED TO DEFINE
#define LOOKAHEAD_DISTANCE      20.0          //inches (radius)
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

    cout<<"In Pure Pursuit" << endl;

    cout<<"Filename: " << profile_filename <<  endl;

    LoadProfile( profile_filename );

    //Init
    m_isPathDone = false;
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

    profile_point_t ppoint;

    cout<<"Filename: " << filename <<  endl;

    openfile.open(filename, ifstream::in );

	if( !openfile.is_open() )
    {
        std::cout<<"*** Could NOT Open m_logfile!!!!" <<std::endl;
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

            cout<<" Profile Imported, " << m_profile.size() << "Lines!" << endl;
            cout<<" All Done!!!!"<<endl;
            return;
        }

        unsigned int line_index = 0;
        unsigned int col_index  = 0;
        string results[6];

        //Read each character of the line and place in proper bin
        while( (line_index < line.size() ) && (col_index<6) )
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
        if( col_index != 5)
        {
            cout << "Invalid Index Size!" << endl;
            return;
        }

        // cout << results[0] << " ";
        // cout << results[1] << " ";
        // cout << results[2] << " ";
        // cout << results[3] << " ";
        // cout << results[4] << " ";
        // cout << results[5] << endl;


        //Use a try-block to insure we don't crash trying to convert strings to float
        try
        {
            ppoint.x          = (double)stof(results[0]);
            ppoint.y          = (double)stof(results[1]);
            ppoint.distance   = (double)stof(results[2]);
            ppoint.curvature  = (double)stof(results[3]);
            ppoint.max_v      = (double)stof(results[4]);
            ppoint.velocity   = (double)stof(results[5]);
        }
        catch(...)
        {
            cout<<"Exception!! "<<endl;
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
        cout << m_profile[i].distance << " ";
        cout << m_profile[i].curvature << " ";
        cout << m_profile[i].max_v << " ";
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
 ** PurePursuitPeriodic
 ** Run the PureProfile Algorithm
 *******************************/
void PurePursuit::PurePursuitPeriodic(void)
{

    //Sanity Check
    if( m_isPathDone )
    {
        //Path done.  Don't do anymore
        Robot::m_drivetrain->Stop();
        return;
    }

    //Running Pure Pursuit Algorithm
    unsigned int ppindex = FindClosestPoint();
    coord_t lookahead_pt = FindLookaheadPoint();
    double curvature     = CalcCurvature(lookahead_pt );


    //Calculate Left and Right target velocities
    //  +curve=Right turn = higher Left drive
    //  -curve=Left turn  = highrt right drive
    double target_Lv = m_profile[ppindex].velocity * (2 + curvature*ROBOT_TRACK_WIDTH)/2;
    double target_Rv = m_profile[ppindex].velocity * (2 - curvature*ROBOT_TRACK_WIDTH)/2;

    double curr_Lv   = Robot::m_odometry->GetLVel();
    double curr_Rv   = Robot::m_odometry->GetRVel();


    double calcLdrive = target_Lv/200 + 0.5;

    double calcRdrive = target_Rv/200 + 0.5;


    Robot::m_drivetrain->Drive(calcLdrive,calcRdrive);

    //Lff, Rff
    //Lfb, Rfb
    //kV,  kA, kP


    // //**** DEBUG ****
    // std::cout<< "pp index = " << ppindex << std::endl;
    // std::cout<< "pp la pt = " << lookahead_pt.x <<" "<< lookahead_pt.y << std::endl;
    // std::cout<< "pp curve = " << curvature  << std::endl;
    // std::cout<< "pp Veloc = " << target_Lv <<" "<< target_Rv << std::endl;

    // std::cout<< "pp Drive = " << calcLdrive <<" "<< calcRdrive << std::endl;

    if( !m_logfile->is_open() ) return;


    //Gather some data 
    double curr_time = frc::Timer::GetFPGATimestamp() - m_logStartTime;
    double curr_x    = Robot::m_odometry->GetX();
    double curr_y    = Robot::m_odometry->GetY();



    *m_logfile << std::fixed << std::setprecision(3);

    *m_logfile << curr_time                                 << ","; // 4:  Yaw
    *m_logfile << curr_x  <<" "<< curr_y                    << ","; // 5:  LeftMoto
    *m_logfile << curr_Lv <<" "<< curr_Rv                   << ","; // 5:  LeftMoto


    *m_logfile << ppindex                                  << ","; // 4:  Yaw
    *m_logfile <<  lookahead_pt.x <<" "<< lookahead_pt.y   << ","; // 5:  LeftMoto
    *m_logfile << curvature                                << ","; // 4:  Yaw
    *m_logfile << target_Lv  <<" "<< target_Rv             << ","; // 4:  Yaw
    *m_logfile << calcLdrive <<" "<< calcRdrive            << ","; // 4:  Yaw



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

    for(unsigned int i = m_profile.size()-2;  i>=0;  i-- )
    {

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
    //Need a better way of returning failed searches ??
    return {-1,-1}; //????

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
    filename += "pp_";              //Add pp prefix
    filename += tbuf;               //Add time file name
    filename += ".csv";             //Add CSV as the extention

    //Finally, open the file
	m_logfile->open(filename, std::ios::out | std::ios::trunc );    

    if( !m_logfile->is_open() )
        std::cout<<"*** Could NOT Open m_logfile!!!!"<<std::endl;

}
void PurePursuit::LogfileClose(void)
{
    if( m_logfile->is_open() )
    {
		m_logfile->close();
        std::cout<<"m_logfile Closed"<<std::endl;
    }
}



