// PLATFORM.h - common things
#define PLATFORM LARA
#define PROG_VERSION 4.0

// the DEBUG's each turn on a macro named DBGx()
// - blocks of code
#define DEBUG
// 0 print func names
#define DEBUG0
// 1 print more interesting things
#undef DEBUG1
// 2 print details
#define DEBUG2
// debug related to wispr
#define DEBUGWISPR

#define POWERLOGGING
#define SYSTEMDIAGNOSTICS
#define WISPR
#define IRIDIUM
#define CTDSENSOR
#define CONSOLEINPUT
#define REALTIME
#define NIGKWINCH

#define MAX_GPS_CHANGE 1.0
#define MAX_UPLOAD 30000 // bytes
#define MAX_STARTUPS 1000
#define MIN_DETECTION_INTERVAL 10
#define MAX_DETECTION_INTERVAL 60
#define MAX_BLOCK_SIZE 2000
#define MIN_DATAX_INTERVAL 30
#define MAX_DATAX_INTERVAL 2880

#ifdef DEBUGWISPR
#define WISPRNUMBER 0
#else
#define WISPRNUMBER 4
#endif
#define MAX_DETECTIONS 20
#define MIN_DUTYCYCLE 0
#define MAX_DUTYCYCLE 100
#define MIN_FREESPACE 1.0 // represents the cutoff percentage for WISPR
                          // recording

// Enabling REALTIME mode will initiate the call land protocol to transfer
// realtime data.
// It will also use a filenumber based .log system rather than activity log

#ifdef REALTIME
#define RTS(X) X
#else
#define RTS(X)
#endif

// levels of debug
#ifdef DEBUG
#define DBG(X) X
#else
#define DBG(X)
#endif
// DEBUG0
#ifdef DEBUG0
#define DBG0(X) X
#else
#define DBG0(X)
#endif
// DEBUG1
#ifdef DEBUG1
#define DBG1(X) X
#else
#define DBG1(X)
#endif
// DEBUG2
#ifdef DEBUG2
#define DBG2(X) X
#else
#define DBG2(X)
#endif

/*
//CTD
#ifdef CTDSENSOR
   #define CTD(X)    X
#else
   #define CTD(X)
#endif*/

// TIMING
#ifdef TIMING
#define CLK(X) X
#else
#define CLK(X)
#endif

// PowerLogging
#ifdef POWERLOGGING
// LARA has 4 sets of Lithium-Primary Batteries. Rated at 14.4V (Initial at
// 14.74V) and 117Amp-Hour
// This Provides 24MJ of energy!
#define BATTERYLOG

#define POWERERROR 1.02
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
   11: 51.2secs/file write    421.875bytes/hr
   12: 102.4secs/file         201.937bytes/hr
   13: 204.8secs/file         105.468
   14: 409.6                  52.734
   15: 819.2                  26.367
   16: 1638.4                 13.183
*************************************************/
#else
#define PWR(X)
#endif

#define Delayms(X) RTCDelayMicroSeconds((long) 1000 * (long) X)

// shared buffers, defined in common.c
#define BUFSZ 1024
