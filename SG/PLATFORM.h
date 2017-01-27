
// DEFINE THE TYPE OF PLATFORM
//#define PLATFORM LARA
//#define PLATFORM AUH
#define PLATFORM SeaGlider
//#define PLATFORM RAOSTop
//#define PLATFORM RAOSBottom

// SG PROGRAM
#define PROG_VERSION 3.0 // Keep this up to date!!!

#define POWERLOGGING

#define WISPR
//#define IRIDIUM
//#define CTDSENSOR
#define SEAGLIDER

// Enabling REALTIME mode will initiate the call land protocol to transfer
// realtime data.
// It will also use a filenumber based .log system rather than activity log

//#define REALTIME
#ifdef REALTIME
#define RTS(X) X
#else
#define RTS(X)
#endif

//#define WINCH

// DEBUG
//#define DEBUG
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

#define MAX_UPLOAD 30000 // bytes
#define MAX_STARTUPS 1000
#define MIN_DETECTION_INTERVAL 15
#define MAX_DETECTION_INTERVAL 360
#define MAX_BLOCK_SIZE 2000
#define MIN_DATAX_INTERVAL 60
#define MAX_DATAX_INTERVAL 1440

#define WTMODE nsStdSmallBusAdj // choose: nsMotoSpecAdj or nsStdSmallBusAdj
#define SYSCLK 2000 // Clock speed: 2000 works 160-32000 kHz Default: 16000

#ifdef SEAGLIDER
#define MIN_OFF_DEPTH 25
#define MIN_ON_DEPTH 15
#endif

#define WISPRNUMBER 1
#define MAX_DETECTIONS 20
#define MIN_DUTYCYCLE 0
#define MAX_DUTYCYCLE 100
#define MIN_FREESPACE 2.0 // represents the cutoff percentage for WISPR
                          // recording

// TIMING
#ifdef TIMING
#define CLK(X) X
#else
#define CLK(X)
#endif

// PowerLogging
#ifdef POWERLOGGING

#define BATTERYLOG

// #define POWERERROR 1.05
#define MIN_BATTERY_VOLTAGE                                                    \
  11.0 // volts or 11.0 for 15V Battery System of Seaglider
#define INITIAL_BATTERY_CAPACITY 5000 // kiloJoules
#define MINIMUM_BATTERY_CAPACITY INITIAL_BATTERY_CAPACITY * 0.1

#define PWR(X) X
#define BITSHIFT                                                               \
  11 // Crucial to ADS Timing of Program. explained in ads power consumption
     // calcation excel file
/***************************************************
10: 25.6seconds/file write 843.75 bytes/hour
11: 51.2secs/file write 421.875bytes/hr
12: 102.4secs/file 201.937bytes/hr
13: 204.8secs/file 105.468
14: 409.6 52.734
15: 819.2 26.367
16: 1638.4 13.183
*************************************************/
#else
#define PWR(X)
#endif
