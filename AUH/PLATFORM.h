
// DEFINE THE TYPE OF PLATFORM
//#define PLATFORM LARA
#define PLATFORM AUH
//#define PLATFORM SeaGlider
//#define PLATFORM RAOSTop
//#define PLATFORM RAOSBottom

// AUH
#define PROG_VERSION 3.0 // Keep this up to date!!!

#define POWERLOGGING
#define DEBUG
#define WISPR
//#define IRIDIUM
//#define CTDSENSOR
//#define SEAGLIDER
#define REALTIME // AUH is not transmitting real time data. But the
                 // DataXInterval Timer is set for ~1 day periods inwhich
                 // filenum will be incremented.

// Enabling REALTIME mode will initiate the call land protocol to transfer
// realtime data.
// It will also use a filenumber based .log system rather than activity log
#ifdef REALTIME
#define RTS(X) X
#else
#define RTS(X)
#endif

//#define WINCH

// DEBUG
#ifdef DEBUG
#define DBG(X) X
#else
#define DBG(X)
#endif

// CTD
#ifdef CTDSENSOR
#define CTD(X) X
#else
#define CTD(X)
#endif

#ifdef TIMING
#define CLK(X) X
#else
#define CLK(X)
#endif

// PowerLogging
#ifdef POWERLOGGING
#define PWR(X) X
// Crucial to ADS Timing of Program. explained in ads power consumption
// calcation excel file
#define BITSHIFT 11
#else
#define PWR(X)
#endif

extern short LARA_PHASE;

#define MIN_FREESPACE 0.5
#define WISPRNUMBER 2
#define MAX_DETECTIONS 20
#define MAX_DUTYCYCLE 100
#define MIN_DUTYCYCLE 0
#define WTMODE nsStdSmallBusAdj // choose: nsMotoSpecAdj or nsStdSmallBusAdj
#define SYSCLK 16000 // Clock speed: 2000 works 160-32000 kHz Default: 16000
#define MAX_GPS_CHANGE 1.0
#define MAX_UPLOAD 30000 // bytes
#define MAX_STARTUPS 1000
#define MIN_DETECTION_INTERVAL 10
#define MAX_DETECTION_INTERVAL 60
#define MAX_BLOCK_SIZE 2000
#define MIN_DATAX_INTERVAL 30
#define MAX_DATAX_INTERVAL 2880
