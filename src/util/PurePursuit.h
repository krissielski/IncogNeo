#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include<vector>



//****************************
//Coordinate struct
typedef struct coordinate
{
    double x;
    double y;
}coord_t;


class PurePursuit
{

    private:

        //Private functions
        void LoadProfile( std::string profile_filename );
        unsigned int FindClosestPoint(void);
        coord_t FindLookaheadPoint(void);
        double  CalcCurvature( coord_t aim_pt );


        //Robot Track profile Struct
        typedef struct profile_point
        {
            double x;           //[0]
            double y;           //[1]
            double distance;    //[2]
            double curvature;   //[3]
            double max_v;       //[4]
            double velocity;    //[5]
        }profile_point_t;

        //THE PROFILE VECTOR
        std::vector<profile_point_t> m_profile; //** DOES THIS NEED TO BE A POINTER AND DYNAMIC??

        //Local Variables
        bool m_isPathDone;


        //File Logging
        std::ofstream *m_logfile;
        void LogfileOpen(void);
        void LogfileClose(void);
        double m_logStartTime;

    public:
        PurePursuit( std::string profile_filename );
        void PrintProfile( void );


        void PurePursuitPeriodic(void);
        void PurePursuitInit(void);
        void PurePursuitEnd(void);
        bool PurePursuitIsDone(void);


};

#endif // PUREPURSUIT_H
