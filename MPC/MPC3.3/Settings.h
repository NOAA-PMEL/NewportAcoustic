

/******************************************************************************\
** VEEPROM Settings Definitions
**
**
\******************************************************************************/
void settings(void); // renamed main from pre-FAT32 code

typedef struct // Used to access runtime settings
{
  char *optName;    // VEE name for lookup
  char *optDefault; // default value
  char *optDesc;    // detailed description text
  char *optCurrent; // current value
} Settings;

#define PROG_NAME "MPC.PROGNAME"
#define PROG_DEFAULT "RAOS"
#define PROG_DESC                                                              \
  \
"Program name\n"

#define PROJID_NAME "MPC.PROJID"
#define PROJID_DEFAULT "QUEH" // QUEH for IRIDIUM RUDICS Login
#define PROJID_DESC                                                            \
  \
"Project ID\n"

#define PLTFRMID_NAME "MPC.PLTFRMID"
#define PLTFRMID_DEFAULT "RA01" // RA01 for RUDICS Login
#define PLTFRMID_DESC                                                          \
  \
"Platform ID\n"

#define LONGITUDE_NAME "MPC.LONG"
#define LONGITUDE_DEFAULT "125:00.00W"
#define LONGITUDE_DESC                                                         \
  \
"10 character string in format above of degree and decimal minute longitude\n"

#define LATITUDE_NAME "MPC.LAT"
#define LATITUDE_DEFAULT "45:00.00N"
#define LATITUDE_DESC                                                          \
  \
"9 character string in format above of degree and decimal minute latitude\n"

#define STARTUPS_NAME "MPC.STARTUPS"
#define STARTUPS_DEFAULT                                                       \
  "0" // Variable will be written to every software restart
#define STARTUPS_DESC                                                          \
  \
"The total number of reboots/startups\n"

#define STARTMAX_NAME                                                          \
  "MPC.STARTMAX" //-s to control this valuable through SYSTEM.CFG file
#define STARTMAX_DEFAULT "9999" //
#define STARTMAX_DESC                                                          \
  \
"Maximum number of startups allowed.\n"

#define DETECTIONINT_NAME "MPC.DETINT" //-i
#define DETECTIONINT_DEFAULT "10"
#define DETECTIONINT_DESC                                                      \
  \
"System interval time used for calling detections and calculating dutycycle.\n"

#define FILENUM_NAME "MPC.FILENUM"
#define FILENUM_DEFAULT "00000000"
#define FILENUM_DESC                                                           \
  \
"File Naming System\n"

#define LOGFILE_NAME "MPC.LOGFILE"
#define LOGFILE_DEFAULT "ACTIVITY.LOG"
#define LOGFILE_DESC                                                           \
  \
"Log file for long term moorings that don't transmit data"

#define DATAXINTERVAL_NAME "MPC.DATAXINT" //-a currently...
#define DATAXINTERVAL_DEFAULT "360"
#define DATAXINTERVAL_DESC                                                     \
  \
"The time in minutes between data transfers. Whether AModem or just IRIDIUM.\n"

// POWER LOGGING SETTINGS
#define BATTERYLOGGER_NAME "ADS.BATLOG" // T
#define BATTERYLOGGER_DEFAULT "0"
#define BATTERYLOGGER_DESC                                                     \
  \
"To log the energy use of battery or not. 1=calculate.\n"

#define BATTERYCAPACITY_NAME "ADS.BATCAP" // Calculated by hand before
                                          // deployment
#define BATTERYCAPACITY_DEFAULT "12000.0"
#define BATTERYCAPACITY_DESC                                                   \
  \
"Number of Kilojoules of supplied battery. To be subtracted from and reported to land.\n"

#define MINSYSVOLT_NAME "ADS.MINVOLT" //-v
#define MINSYSVOLT_DEFAULT "11.5"
#define MINSYSVOLT_DESC                                                        \
  \
"The voltage at which the system will go into only recovery mode"

// IRIDIUM VEEPROM SETTINGS
#define IRIDPHONE_NAME "IRID.PHONE"       //-p?
#define IRIDPHONE_DEFAULT "0088160000519" // PMEL Rudics
#define IRIDPHONE_DESC                                                         \
  \
"The 13 character phone number the iridium modem uses to call in.\n"

#define MINSIGQ_NAME "IRID.MINSIGQ"
#define MINSIGQ_DEFAULT "3"
#define MINSIGQ_DESC                                                           \
  \
"The minimum signal qualtiy of Iridium call.\n"

#define WARMUP_NAME "IRID.WARMUP"
#define WARMUP_DEFAULT "27"
#define WARMUP_DESC                                                            \
  \
"GPS IRID warm-up period in seconds.\n"

#define MAXUPLOAD_NAME "IRID.MAXUPL"
#define MAXUPLOAD_DEFAULT "30000"
#define MAXUPLOAD_DESC                                                         \
  \
"This is the size of the largest file to upload in kilobytes.\n"

#define ANTSW_NAME "IRID.ANTSWT"
#define ANTSW_DEFAULT "0"
#define ANTSW_DESC                                                             \
  \
"0: no antenna switch is necessary, 1=Need switch.\n"

#define IRIDREST_NAME "IRID.REST"
#define IRIDREST_DEFAULT "30"
#define IRIDREST_DESC                                                          \
  \
"Rest period for Iridium to make another call when the call fails.\n"

#define OFFSET_NAME "IRID.OFFSET"
#define OFFSET_DEFAULT "4"
#define OFFSET_DESC                                                            \
  \
"GPS and UTC time offset in sec.\n"

#define MAXCALLS_NAME "IRID.MAXCALLS"
#define MAXCALLS_DEFAULT "4"
#define MAXCALLS_DESC                                                          \
  \
"Maximum Iridium calls per session.\n"

#define CALLHOUR_NAME "IRID.CALLHOUR"
#define CALLHOUR_DEFAULT "20" // UTC Hour
#define CALLHOUR_DESC                                                          \
  \
"Hour at which to call iridium\n"

#define CALLMODE_NAME "IRID.CALLMODE"
#define CALLMODE_DEFAULT "0"
#define CALLMODE_DESC                                                          \
  \
"Induce call from Call Hour or Data Transfer Interval\n"

// WISPR VEEPROM SETTINGS
#define DETECTIONNUM_NAME "WISP.DETNUM" //-n
#define DETECTIONNUM_DEFAULT "0"
#define DETECTIONNUM_DESC                                                      \
  \
"The number of detections at one inquiry to trigger AModem Call"

#define DETECTIONMAX_NAME "WISP.DETMAX" //-d
#define DETECTIONMAX_DEFAULT "10"
#define DETECTIONMAX_DESC                                                      \
  \
"[-1-20]Maximum number of detections to be returned at the end of every DETECTIONINTerval.\n"

#define WISPRGAIN_NAME "WISP.GAIN" //-g
#define WISPRGAIN_DEFAULT "0"
#define WISPRGAIN_DESC                                                         \
  \
"[0-3] Gain setting to apply to WISPR which controls EOS_HM1 Pre-amp.\n"

#define WISPRNUM_NAME "WISP.NUM" //-w?
#define WISPRNUM_DEFAULT "1"
#define WISPRNUM_DESC                                                          \
  \
"[1-4] The Wispr board number to be operated while WISP.ON = 1. \n"

#define DUTYCYCLE_NAME "WISP.DUTYCYCL" //-c
#define DUTYCYCLE_DEFAULT "100"
#define DUTYCYCLE_DESC                                                         \
  \
"[0-100] Percentage of DETINT duration which WISPR is on. The later half of the DETINT cycle.\n"

#define WISPRMODE_NAME "WISP.MODE"
#define WISPRMODE_DEFAULT "2"
#define WISPRMODE_DESC                                                         \
  \
"[1-5] Mode of WISPR DSP Program\n"

// SEAGLIDER LOGGER SETTINGS
#define DIVENUM_NAME "SEAG.DIVENUM"
#define DIVENUM_DEFAULT "0001"
#define DIVENUM_DESC                                                           \
  \
"The current dive number of the seaglider\n"

#define POWERONDEPTH_NAME "SEAG.ONDEPTH"
#define POWERONDEPTH_DEFAULT "25"
#define POWERONDEPTH_DESC                                                      \
  \
"The minimum depth/floating point at which the WISPR will power on\n"

#define POWEROFFDEPTH_NAME "SEAG.OFFDEPTH"
#define POWEROFFDEPTH_DEFAULT "25"
#define POWEROFFDEPTH_DESC                                                     \
  \
"The minimum depth/floating point at which the WISPR will power off\n"

// BLUETOOTH SETTINGS
#define BLUETOOTH_NAME "BT.ON"
#define BLUETOOTH_DEFAULT "0"
#define BLUETOOTH_DESC                                                         \
  \
"Define whether bluetooth is on or off.\n"

// LINKQUEST ACOUSTIC MODEM UWM2000 VEEPROM SETTINGS
#define AMODEMOFFSET_NAME "AMDM.OFFSET"
#define AMODEMOFFSET_DEFAULT "6"
#define AMODEMOFFSET_DESC                                                      \
  \
"The offset in seconds for which it takes to send and process gps\n"

#define AMODEMMAXUPLOAD_NAME "AMDM.MAXUPL"
#define AMODEMMAXUPLOAD_DEFAULT "4000" // BYTES
#define AMODEMMAXUPLOAD_DESC                                                   \
  \
"The number of bytes inwhich to limit AModem Data Transfers\n"

#define AMODEMBLOCKSIZE_NAME "AMDM.BLKSIZE"
#define AMODEMBLOCKSIZE_DEFAULT "1000"
#define AMODEMBLOCKSIZE_DESC                                                   \
  \
"The number of bytes for each pass of data\n"

// NIGK Winch Settings
#define NIGKDELAY_NAME "NIGK.DELAY"
#define NIGKDELAY_DEFAULT "4"
#define NIGKDELAY_DESC                                                         \
  \
"Time in seconds it takes for Winch to respond to command from Profiling Buoy\n"

#define NIGKANTENNALENGTH_NAME "NIGK.ANTLEN"
#define NIGKANTENNALENGTH_DEFAULT "14"
#define NIGKANTENNALENGTH_DESC                                                 \
  \
"Length in meters from CTD depth sensor to antenna\n"

#define NIGKTARGETDEPTH_NAME "NIGK.TDEPTH"
#define NIGKTARGETDEPTH_DEFAULT "10"
#define NIGKTARGETDEPTH_DESC                                                   \
  \
"Depth in meters at which CTD position is optimum for gps/iridium comms\n"

#define NIGKRISERATE_NAME "NIGK.RRATE"
#define NIGKRISERATE_DEFAULT "15"
#define NIGKRISERATE_DESC                                                      \
  \
"Rate in meters per minute at which winch ascends profiling float\n"

#define NIGKFALLRATE_NAME "NIGK.FRATE"
#define NIGKFALLRATE_DEFAULT "9"
#define NIGKFALLRATE_DESC                                                      \
  \
"Rate in meters per minute at which winch descends profliling float\n"

#define NIGKPROFILES_NAME "NIGK.PROFILES"
#define NIGKPROFILES_DEFAULT "0"
#define NIGKPROFILES_DESC                                                      \
  \
"Keeping track of the number of profiles on winch\n"

#define NIGKRECOVERY_NAME "NIGK.RECOVERY"
#define NIGKRECOVERY_DEFAULT "0"
#define NIGKRECOVERY_DESC                                                      \
  \
"1 to set Profiling BUoy in Recovery Mode\n"

// CTD Sensor Settings
#define CTDUPLOADFILE_NAME "CTD.UPLOAD"
#define CTDUPLOADFILE_DEFAULT "1"
#define CTDUPLOADFILE_DESC                                                     \
  \
"\"1\" to upload file via Iridium else \"0\"\n" \
/*
#define  CTDSAMPLINGMODE_NAME		"CTD.POLLED"
#define  CTDSAMPLINGMODE_DEFAULT "1"
#define  CTDSAMPLINGMODE_DESC		\
"1 for Polled, 0 for Autonomous\n" \

#define 	CTDSAMPLEDELAY_NAME		"CTD.DELAY"
#define  CTDSAMPLEDELAY_DEFAULT  "2"
#define  CTDSAMPLEDELAY_DESC 		\
"Number of seconds to wait before taking another sample. Polled Mode only \n" \

#define  CTDSAMPLEINTERVAL_NAME 	"CTD.SAMPINT"
#define  CTDSAMPLEINTERVAL_DEFAULT "6"
#define  CTDSAMPLEINTERVAL_DESC 		\
"Number of seconds between samples. Autonomous sampling mode only \n" 
*/