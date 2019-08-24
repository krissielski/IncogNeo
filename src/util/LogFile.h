#ifndef LOGFILE_H
#define LOGFILE_H

#include <ostream>
#include <fstream>

class LogFile
{
    private:
        void LogfileOpen(void);
        void LogfileClose(void);
        double m_logStartTime;

    public:
        LogFile(void);      //Constructor

        void LogFileEnable( bool enable );
        void LogFileStop( void );
        void LogFilePeriodic( void );

};




#endif //LOGFILE_H