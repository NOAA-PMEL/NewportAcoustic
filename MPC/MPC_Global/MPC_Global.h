/*********************************************************
** SYSTEM PARAMETER STRUCTURES
***********************************************************/
// System Parameters//Always defined // used as MPC.name
typedef struct {
  char PROGNAME[20]; // added HM
  char LONG[17];     // 123:45.67 West
  char LAT[17];      // 45:67.8900 North
  char PROJID[6];    // rudicsland
  char PLTFRMID[6];  // rudicsland
  char LOGFILE[13]; // File Name: activity.log
  long FILENUM; // current number for 00000000.dat
  short STARTUPS;
  short STARTMAX; //-s
  short DETINT;   //-D      //Minutes   //WISPR DET INTERVAL
  short DATAXINT; // VEE:DATAXINTERVAL_NAME
} SystemParameters;

// This structure is System status, used as LARA.*
typedef struct {
  short PHASE; // 1=WISPR, 2=Ascent, 3=Surface, 4=Descent, 5=deployment
  bool ON;       // While "ON", continue running program
  bool DATA;     // ?? Data is triggered true when the timing interval goes off.
  bool SURFACED; // Set to true when Antenna is estimated to be above water
  short BUOYMODE;  // 0=stopped 1=ascend 2=descend 3=careful ascent
  float DEPTH;     // Most recent depth measurement from CTD
  float MOORDEPTH; // Depth at beginning of LARA.PHASE==1. Averaged Samples
  float TOPDEPTH;
  short ASCENTTIME;
  short DESCENTTIME;
  short STATUS;
  bool LOWPOWER;
  bool RESTART;
  TUPort AntModport; // port for antenna module
  TUPort CTDport; // port for the buoy ctd
  bool CTDsync; // ctd sync mode
} SystemStatus;

// Seaglider Structure Parameters:
typedef struct {
  short ONDEPTH;  // Depth at which SG powers on during the descent.
  short OFFDEPTH; // Depth at which SG power off during ascent.
  short DIVENUM;  // For use instead of FileNumber: Might just use FILENUM.
} SeagliderParameters;

typedef struct {
  char BATCAP[9];
  short BATLOG;    // t logging change in battery capacity
  char MINVOLT[6]; //-v %.2f  minimum system voltage
} PowerParameters;
