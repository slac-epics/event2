#include<epicsTime.h>

extern  int timesyncGetFifo(epicsTimeStamp     *epicsTime_ps,
                            long long          *fid, 
                            unsigned int        eventCode,
                            unsigned long long *idx,
                            int                 incr);
extern long long timesyncGetLastFiducial( );         /* Returns lastfid, the last fiducial set by ISR */
