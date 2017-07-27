/******************************************************************************\
** Settings.c (1/27/2015)
**
**	1.1 release: 01/27/2015
*****************************************************************************
**
**	Licensed by:	NOAA, PMEL Newport for the Peristor CFX
**	info@persistor.com - http://www.persistor.com
**
*****************************************************************************
**
**	Developed by:	Haru Matsumoto
**
\******************************************************************************/
#define PD_CMD_BUILD_LINKS // this must be at the top of the file !!!

#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h>
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <limits.h>
#include <stat.h> // PicoDOS POSIX-like File Status Definitions
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // PicoDOS POSIX-like UNIX Function Definitions
#ifdef ACOUSTICMODEM
#include <AMODEM.h>
#endif
#ifdef CTDSENSOR
#include <CTD.h>
#endif
#ifdef POWERLOGGING
#include <ADS.h>
#endif
#ifdef IRIDIUM
#include <GPSIRID.h>
#endif
#ifdef BLUETOOTH
#include <BT.h>
#endif
#ifdef WISPR
#include <WISPR.h>
#endif

bool ModifyPermission = true; // gets updated when password entered
char CFXNum[6];               // 5 digits + terminating zero

Settings *SettingsPointer;
// For all Platforms
Settings SYSSettings[] = {PROG_NAME,
                          PROG_DEFAULT,
                          PROG_DESC,
                          "",
                          PROJID_NAME,
                          PROJID_DEFAULT,
                          PROJID_DESC,
                          "",
                          PLTFRMID_NAME,
                          PLTFRMID_DEFAULT,
                          PLTFRMID_DESC,
                          "",
                          LATITUDE_NAME,
                          LATITUDE_DEFAULT,
                          LATITUDE_DESC,
                          "",
                          LONGITUDE_NAME,
                          LONGITUDE_DEFAULT,
                          LONGITUDE_DESC,
                          "",
                          STARTUPS_NAME,
                          STARTUPS_DEFAULT,
                          STARTUPS_DESC,
                          "",
                          STARTPHASE_NAME,
                          STARTPHASE_DEFAULT,
                          STARTPHASE_DESC,
                          "",
                          STARTMAX_NAME,
                          STARTMAX_DEFAULT,
                          STARTMAX_DESC,
                          "",
                          MINSYSVOLT_NAME,
                          MINSYSVOLT_DEFAULT,
                          MINSYSVOLT_DESC,
                          "",
                          FILENUM_NAME,
                          FILENUM_DEFAULT,
                          FILENUM_DESC,
                          "",
                          BATTERYCAPACITY_NAME,
                          BATTERYCAPACITY_DEFAULT,
                          BATTERYCAPACITY_DESC,
                          "",
                          DETECTIONINT_NAME,
                          DETECTIONINT_DEFAULT,
                          DETECTIONINT_DESC,
                          "",
                          DATAXINTERVAL_NAME,
                          DATAXINTERVAL_DEFAULT,
                          DATAXINTERVAL_DESC,
                          "",
                          DETECTIONINT_NAME,
                          DETECTIONINT_DEFAULT,
                          DETECTIONINT_DESC,
                          "",
                          BATTERYLOGGER_NAME,
                          BATTERYLOGGER_DEFAULT,
                          BATTERYLOGGER_DESC,
                          "",
                          0,
                          0,
                          0,
                          0};
#ifdef POWERLOGGING
Settings PowerSettings[] = {BATTERYLOGGER_NAME,
                            BATTERYLOGGER_DEFAULT,
                            BATTERYLOGGER_DESC,
                            "",
                            MINSYSVOLT_NAME,
                            MINSYSVOLT_DEFAULT,
                            MINSYSVOLT_DESC,
                            "",
                            BATTERYCAPACITY_NAME,
                            BATTERYCAPACITY_DEFAULT,
                            BATTERYCAPACITY_DESC,
                            "",
                            0,
                            0,
                            0,
                            0};
#endif
#ifdef CTDSENSOR
Settings CTDSettings[] = {
    CTDUPLOADFILE_NAME, CTDUPLOADFILE_DEFAULT, CTDUPLOADFILE_DESC, "",
    /*	CTDSAMPLINGMODE_NAME,CTDSAMPLINGMODE_DEFAULT, 	CTDSAMPLINGMODE_DESC,
       "",
            CTDSAMPLEDELAY_NAME, CTDSAMPLEDELAY_DEFAULT,
       CTDSAMPLEDELAY_DESC, "",
            CTDSAMPLEINTERVAL_NAME, CTDSAMPLEINTERVAL_DEFAULT,
       CTDSAMPLEINTERVAL_DESC, "",*/
    0,
    0, 0, 0};
#endif

#ifdef WISPR
Settings WISPSettings[] = {WISPRNUM_NAME,
                           WISPRNUM_DEFAULT,
                           WISPRNUM_DESC,
                           "",
                           DETECTIONMAX_NAME,
                           DETECTIONMAX_DEFAULT,
                           DETECTIONMAX_DESC,
                           "",

                           DETECTIONNUM_NAME,
                           DETECTIONNUM_DEFAULT,
                           DETECTIONNUM_DESC,
                           "",
                           WISPRGAIN_NAME,
                           WISPRGAIN_DEFAULT,
                           WISPRGAIN_DESC,
                           "",
                           DUTYCYCLE_NAME,
                           DUTYCYCLE_DEFAULT,
                           DUTYCYCLE_DESC,
                           "",
                           0,
                           0,
                           0,
                           0};
#endif
#ifdef IRIDIUM
Settings IRIDSettings[] = {ANTSW_NAME,
                           ANTSW_DEFAULT,
                           ANTSW_DESC,
                           "",
                           IRIDPHONE_NAME,
                           IRIDPHONE_DEFAULT,
                           IRIDPHONE_DESC,
                           "",
                           WARMUP_NAME,
                           WARMUP_DEFAULT,
                           WARMUP_DESC,
                           "",
                           MAXUPLOAD_NAME,
                           MAXUPLOAD_DEFAULT,
                           MAXUPLOAD_DESC,
                           "",
                           IRIDREST_NAME,
                           IRIDREST_DEFAULT,
                           IRIDREST_DESC,
                           "",
                           OFFSET_NAME,
                           OFFSET_DEFAULT,
                           OFFSET_DESC,
                           "",
                           MAXCALLS_NAME,
                           MAXCALLS_DEFAULT,
                           MAXCALLS_DESC,
                           "",
                           MINSIGQ_NAME,
                           MINSIGQ_DEFAULT,
                           MINSIGQ_DESC,
                           "",
                           CALLHOUR_NAME,
                           CALLHOUR_DEFAULT,
                           CALLHOUR_DESC,
                           "",
                           CALLMODE_NAME,
                           CALLMODE_DEFAULT,
                           CALLMODE_DESC,
                           "",
                           LOWFIRST_NAME,
                           LOWFIRST_DEFAULT,
                           LOWFIRST_DESC,
                           "",
                           0,
                           0,
                           0,
                           0};
#endif
#ifdef SEAGLIDER
Settings SEAGSettings[] = {POWERONDEPTH_NAME,
                           POWERONDEPTH_DEFAULT,
                           POWERONDEPTH_DESC,
                           "",
                           POWEROFFDEPTH_NAME,
                           POWEROFFDEPTH_DEFAULT,
                           POWEROFFDEPTH_DESC,
                           "",
                           DIVENUM_NAME,
                           DIVENUM_DEFAULT,
                           DIVENUM_DESC,
                           "",
                           0,
                           0,
                           0,
                           0};
#endif

#ifdef ACOUSTICMODEM
Settings AMDMSettings[] = {AMODEMMAXUPLOAD_NAME,
                           AMODEMMAXUPLOAD_DEFAULT,
                           AMODEMMAXUPLOAD_DESC,
                           "",
                           AMODEMOFFSET_NAME,
                           AMODEMOFFSET_DEFAULT,
                           AMODEMOFFSET_DESC,
                           "",
                           AMODEMBLOCKSIZE_NAME,
                           AMODEMBLOCKSIZE_DEFAULT,
                           AMODEMBLOCKSIZE_DESC,
                           "",
                           0,
                           0,
                           0,
                           0};

#endif
#ifdef BLUETOOTH
Settings BTSettings[] = {
    BLUETOOTH_NAME, BLUETOOTH_DEFAULT, BLUETOOTH_DESC, "", 0, 0, 0, 0};
#endif
#ifdef WINCH
Settings NIGKSettings[] = {NIGKDELAY_NAME,
                           NIGKDELAY_DEFAULT,
                           NIGKDELAY_DESC,
                           "",
                           NIGKFALLRATE_NAME,
                           NIGKFALLRATE_DEFAULT,
                           NIGKFALLRATE_DESC,
                           "",
                           NIGKRISERATE_NAME,
                           NIGKRISERATE_DEFAULT,
                           NIGKRISERATE_DESC,
                           "",
                           NIGKANTENNALENGTH_NAME,
                           NIGKANTENNALENGTH_DEFAULT,
                           NIGKANTENNALENGTH_DESC,
                           "",
                           NIGKTARGETDEPTH_NAME,
                           NIGKTARGETDEPTH_DEFAULT,
                           NIGKTARGETDEPTH_DESC,
                           "",
                           NIGKPROFILES_NAME,
                           NIGKPROFILES_DEFAULT,
                           NIGKPROFILES_DESC,
                           "",
                           NIGKRECOVERY_NAME,
                           NIGKRECOVERY_DEFAULT,
                           NIGKRECOVERY_DESC,
                           "",
                           0,
                           0,
                           0,
                           0};
#endif

// PROTOTYPES

void FetchSettings(Settings *);
void ResetDefaultSettings(Settings *);
char *SetupHelpCmd(CmdInfoPtr cip);
char *SetupListCmd(CmdInfoPtr cip);
void SystemSettings();
void SetupAModemCmd();
void SetupSettings();
void SetupPWRCmd();
void SamplePWR();
void StartPWRCmd();
void StopPWRCmd();
void SetupIridiumCmd();
void SetupCTDCmd();
void StartCTDCmd();
void StopCTDCmd();
void SetupWISPRCmd();
void WISPROn();
void WISPROff();
static void IRQ5_ISR(void);
char *SetupDefaultsCmd(CmdInfoPtr cip);
char *AllDirsCmd(CmdInfoPtr cip);
void DisplayParameters(FILE *);

CmdTable SetupCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "SETUP>", PDCCmdStdInteractive, 0, 0, 1, 1,
        "MPC System Settings Commands\n"
#ifdef SYSTEMDIAGNOSTICS
        ,
        "SYSTEM", SystemSettings, 0, 0, 0, 0, "System Diagnostics Menu\n"
#endif
        ,
        "LIST", SetupListCmd, 0, 0, 0, 0, "List settings [/V] (verbose)", "SET",
        PDCSetCmd, 0, 0, 0, 0, "[var=[str]] [/SLFE?]", "DEFAULTS",
        SetupDefaultsCmd, 0, 10, 0, 0, "Reset to default settings", "ALLDIRS",
        AllDirsCmd, 0, 10, 0, 0, "Get directories from all drives", "DATE",
        PDCDateCmd, 0, 0, 0, 0, "[mm-dd-yy[yy] [time]", "BREAK", PDCCmdStdBreak,
        0, 0, 0, 0, "Return to Program", "QUIT", PDCResetCmd, 0, 0, 0, 0,
        "QUIT, RES, EXIT all reset the CFX", "RES", PDCResetCmd, 0, 0, 0, 0, "",
        "EXIT", PDCResetCmd, 0, 0, 0, 0, "", "HELP", SetupHelpCmd, 0, 0, 0, 0,
        "HELP, HE, H, ? all show help", "HE", SetupHelpCmd, 0, 0, 0, 0, "", "H",
        SetupHelpCmd, 0, 0, 0, 0, "", "?", SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};

CmdTable SystemCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "SYSTEM>", PDCCmdStdInteractive, 0, 0, 1, 1,
        "System Diagnostic Commands\n", "LIST", SetupListCmd, 0, 0, 0, 0,
        "List settings [/V] (verbose)", "SET", PDCSetCmd, 0, 0, 0, 0,
        "[var=[str]] [/SLFE?]", "GETSET", GetSettings, 0, 0, 0, 0,
        "get all veeprom settings"
#ifdef ACOUSTICMODEM
        ,
        "AMODEM", SetupAModemCmd, 0, 0, 0, 0, "Acoustic Modem Menu"
#endif
#ifdef CTDSENSOR
        ,
        "CTD", SetupCTDCmd, 0, 0, 0, 0, "CTD Menu"
#endif
#ifdef IRIDIUM
        ,
        "IRID", SetupIridiumCmd, 0, 0, 0, 0, "Iridium Menu"
#endif
#ifdef BLUETOOTH
        ,
        "BTLP", Bluetooth_LPWait, 0, 0, 0, 0, "Wait in Bluetooth Low Power Mode"
#endif
#ifdef POWERLOGGING
        ,
        "PWR", SetupPWRCmd, 0, 0, 0, 0, "Power Menu"
#endif
#ifdef WISPR
        ,
        "WISPR", SetupWISPRCmd, 0, 0, 0, 0, "WISPR Menu"
#endif
        ,
        "BREAK", PDCCmdStdBreak, 0, 0, 0, 0, "Return to Program", "QUIT",
        PDCResetCmd, 0, 0, 0, 0, "QUIT, RES, EXIT all reset the CFX", "RES",
        PDCResetCmd, 0, 0, 0, 0, "", "EXIT", PDCResetCmd, 0, 0, 0, 0, "",
        "HELP", SetupHelpCmd, 0, 0, 0, 0, "HELP, HE, H, ? all show help", "HE",
        SetupHelpCmd, 0, 0, 0, 0, "", "H", SetupHelpCmd, 0, 0, 0, 0, "", "?",
        SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};
#ifdef ACOUSTICMODEM
CmdTable AModemCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "AMODEM>", PDCCmdStdInteractive, 0, 0, 1, 1,
        "Acoustic Modem Command menu\n", "LIST", SetupListCmd, 0, 0, 0, 0,
        "List settings [/V] (verbose)", "SET", PDCSetCmd, 0, 0, 0, 0,
        "[var=[str]] [/SLFE?]", "SYSTEM", SystemSettings, 0, 0, 0, 0,
        "System Menu", "SETUP", SetupSettings, 0, 0, 0, 0,
        "Settings Setup Menu", "BREAK", PDCCmdStdBreak, 0, 0, 0, 0,
        "Return to Program", "QUIT", PDCResetCmd, 0, 0, 0, 0,
        "QUIT, RES, EXIT all reset the CFX", "RES", PDCResetCmd, 0, 0, 0, 0, "",
        "EXIT", PDCResetCmd, 0, 0, 0, 0, "", "HELP", SetupHelpCmd, 0, 0, 0, 0,
        "HELP, HE, H, ? all show help", "HE", SetupHelpCmd, 0, 0, 0, 0, "", "H",
        SetupHelpCmd, 0, 0, 0, 0, "", "?", SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};
#endif
#ifdef IRIDIUM
CmdTable IridiumCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "IRID>", PDCCmdStdInteractive, 0, 0, 1, 1, "Iridium Command Menu\n",
        "SYSTEM", SystemSettings, 0, 0, 0, 0, "System Menu", "LIST",
        SetupListCmd, 0, 0, 0, 0, "List settings [/V] (verbose)", "SET",
        PDCSetCmd, 0, 0, 0, 0, "[var=[str]] [/SLFE?]", "GPS", GPSstartup, 0, 0,
        0, 0, "Gathering GPS Time", "GETSET", GetIRIDIUMSettings, 0, 0, 0, 0,
        "Get IRIDUM Settings", "IRIDGPS", IRIDGPS, 0, 0, 0, 0, "Full Test"
        //   ,"UPLOAD"		, UploadFiles,			0, 0, 0, 0, "Upload
        //   Files"
        ,
        "BREAK", PDCCmdStdBreak, 0, 0, 0, 0, "Return to Program", "QUIT",
        PDCResetCmd, 0, 0, 0, 0, "QUIT, RES, EXIT all reset the CFX", "RES",
        PDCResetCmd, 0, 0, 0, 0, "", "EXIT", PDCResetCmd, 0, 0, 0, 0, "",
        "HELP", SetupHelpCmd, 0, 0, 0, 0, "HELP, HE, H, ? all show help", "HE",
        SetupHelpCmd, 0, 0, 0, 0, "", "H", SetupHelpCmd, 0, 0, 0, 0, "", "?",
        SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};
#endif
#ifdef CTDSENSOR
CmdTable CTDCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "CTD>", PDCCmdStdInteractive, 0, 0, 1, 1, "CTD Command Mneu\n",
        "SYSTEM", SystemSettings, 0, 0, 0, 0, "System Menu", "START",
        StartCTDCmd, 0, 0, 0, 0, "Power On CTD", "TS", CTD_Sample, 0, 0, 0, 0,
        "CTD_Sample: TS, SAMPLE", "SAMPLE", CTD_Sample, 0, 0, 0, 0, "", "SYNC",
        CTD_SyncMode, 0, 0, 0, 0, "CTD_SyncMode", "SYNCSTOP", CTD_SampleBreak,
        0, 0, 0, 0, "CTD_SampleBreak", "STOP", StopCTDCmd, 0, 0, 0, 0,
        "Power Down CTD", "BREAK", PDCCmdStdBreak, 0, 0, 0, 0,
        "Return to Program", "QUIT", PDCResetCmd, 0, 0, 0, 0,
        "QUIT, RES, EXIT all reset the CFX", "RES", PDCResetCmd, 0, 0, 0, 0, "",
        "EXIT", PDCResetCmd, 0, 0, 0, 0, "", "HELP", SetupHelpCmd, 0, 0, 0, 0,
        "HELP, HE, H, ? all show help", "HE", SetupHelpCmd, 0, 0, 0, 0, "", "H",
        SetupHelpCmd, 0, 0, 0, 0, "", "?", SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};
#endif
#ifdef POWERLOGGING
ulong ADSStartTime;

CmdTable PWRCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "PWR>", PDCCmdStdInteractive, 0, 0, 1, 1, "Power Command Menu\n",
        "SYSTEM", SystemSettings, 0, 0, 0, 0, "System Menu"

        ,
        "SAMPLE", SamplePWR, 0, 0, 0, 0, "Sample Voltage", "START", StartPWRCmd,
        0, 0, 0, 0, "Start AtoD Power Logging", "STOP", StopPWRCmd, 0, 0, 0, 0,
        "Stop AtoD Power Logging", "BREAK", PDCCmdStdBreak, 0, 0, 0, 0,
        "Return to Program", "QUIT", PDCResetCmd, 0, 0, 0, 0,
        "QUIT, RES, EXIT all reset the CFX", "RES", PDCResetCmd, 0, 0, 0, 0, "",
        "EXIT", PDCResetCmd, 0, 0, 0, 0, "", "HELP", SetupHelpCmd, 0, 0, 0, 0,
        "HELP, HE, H, ? all show help", "HE", SetupHelpCmd, 0, 0, 0, 0, "", "H",
        SetupHelpCmd, 0, 0, 0, 0, "", "?", SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};
#endif
#ifdef WISPR
CmdTable WISPRCmdTable[] =
    {
        //	PROMPT			HANDLER
        //2COLS DEF  CR  ABV	HEADER TEXT
        "WISPR>", PDCCmdStdInteractive, 0, 0, 1, 1, "Power Command Menu\n",
        "SYSTEM", SystemSettings, 0, 0, 0, 0, "System Menu"

        ,
        "LIST", SetupListCmd, 0, 0, 0, 0, "List settings [/V] (verbose)", "SET",
        PDCSetCmd, 0, 0, 0, 0, "[var=[str]] [/SLFE?]", "GETSET",
        GetWISPRSettings, 0, 0, 0, 0, "GetWISPRSettings", "ON", WISPROn, 0, 0,
        0, 0, "Start AtoD Power Logging", "OFF", WISPROff, 0, 0, 0, 0,
        "Stop AtoD Power Logging", "BREAK", PDCCmdStdBreak, 0, 0, 0, 0,
        "Return to Program", "QUIT", PDCResetCmd, 0, 0, 0, 0,
        "QUIT, RES, EXIT all reset the CFX", "RES", PDCResetCmd, 0, 0, 0, 0, "",
        "EXIT", PDCResetCmd, 0, 0, 0, 0, "", "HELP", SetupHelpCmd, 0, 0, 0, 0,
        "HELP, HE, H, ? all show help", "HE", SetupHelpCmd, 0, 0, 0, 0, "", "H",
        SetupHelpCmd, 0, 0, 0, 0, "", "?", SetupHelpCmd, 0, 0, 0, 0, ""

        // TERMINATING ENTRY (!!!!This must be present at end of table !!!!)
        ,
        "", 0, 0, 0, 0, 0, 0};
#endif

/******************************************************************************\
**	settings		Main Entry Point
**
**	This is just the launching point for the major portions of the program.
\******************************************************************************/
void settings(void) {

  CmdInfo ci, *cip = &ci;

  char *ProgramDescription = {
      "\n"
      "Type HELP from the SETUP> prompt for a list of the commands available\n"
      "in the program. Type QUIT from the SETUP> to exit the program.\n"
      "\n"
      "Type LIST /V from the SETUP> prompt for a complete description\n"
      "of each of the settings names, values, and purpose.\n"
      "\n"
      "Type QUIT from the SETUP> to exit the program.\n"
      "\n"};
  sprintf(CFXNum, "%05ld", BIOSGVT.CF1SerNum);

  printf("\nProgram: %s: %s %s \n", __FILE__, __DATE__, __TIME__);
  printf(ProgramDescription);

  FetchSettings(SYSSettings);

#ifdef IRIDIUM
  FetchSettings(IRIDSettings);
#endif
#ifdef WISPR
  FetchSettings(WISPSettings);
#endif
#ifdef SEAGLIDER
  FetchSettings(SEAGSettings);
#endif
#ifdef ACOUSTICMODEM
  FetchSettings(AMDMSettings);
#endif
#ifdef BLUETOOTH
  FetchSettings(BTSettings);
#endif
#ifdef POWERLOGGING
  FetchSettings(PowerSettings);
#endif
#ifdef CTDSENSOR
  FetchSettings(CTDSettings);
#endif
#ifdef WINCH
  FetchSettings(NIGKSettings);
#endif

  //
  //	ENTER INTERACTIVE COMMAND PROCESSING MODE
  //
  CmdStdSetup(&ci, SetupCmdTable, 0);
  cip->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&ci) != CMD_BREAK) {
    PWR(AD_Check();)

    printf("\n%s", ci.errmes);
    fflush(stdout);
  }

  // BIOSReset();	// clean restart

} //____ main() ____//
/******************************************************************************\
** SystemSettings(
\******************************************************************************/
void SystemSettings() {

  CmdInfo cia, *cipa = &cia;

  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};
  SettingsPointer = SYSSettings;
  printf(ProgramDescription);

  CmdStdSetup(&cia, SystemCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {
    PWR(AD_Check();)

    printf("\n%s", cia.errmes);
    fflush(stdout);
  }

  SetupSettings();
}
/******************************************************************************\
** SystemSettings(
\******************************************************************************/
void SetupSettings() {

  CmdInfo cia, *cipa = &cia;

  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};

  // printf(ProgramDescription);

  CmdStdSetup(&cia, SetupCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {
    PWR(AD_Check();)

    printf("\n%s", cia.errmes);
    fflush(stdout);
  }
}
/******************************************************************************\
** WISPRSettings
\******************************************************************************/
#ifdef WISPR
void SetupWISPRCmd() {

  CmdInfo cia, *cipa = &cia;

  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};

  // printf(ProgramDescription);
  SettingsPointer = WISPSettings;
  CmdStdSetup(&cia, WISPRCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {

    PWR(AD_Check();)
    if (tgetq(PAMPort))
      WISPR_Data();
    printf("\n%s", cia.errmes);
    fflush(stdout);
  }

  SystemSettings();
}
void WISPROn() {
  OpenTUPort_WISPR(true);
  WISPRPower(true);
}
void WISPROff() {
  WISPRPower(false);
  OpenTUPort_WISPR(false);
}

/*************************************************************************\
**  static void IRQ5_ISR(void) WISPR
\*************************************************************************/
static void IRQ5_ISR(void) {
  PinIO(IRQ5);
  RTE();
} //____ Irq5ISR() ____//

#endif
/******************************************************************************\
** SystemSettings(
\******************************************************************************/
#ifdef ACOUSTICMODEM

void SetupAModemCmd() {

  CmdInfo cia, *cipa = &cia;

  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};
  SettingsPointer = AMDMPointer;
  // printf(ProgramDescription);

  CmdStdSetup(&cia, AModemCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {
    PWR(AD_Check();)

    printf("\n%s", cia.errmes);
    fflush(stdout);
  }
}
#endif
/******************************************************************************\
** SystemSettings(
\******************************************************************************/
void SetupPWRCmd() {

  CmdInfo cia, *cipa = &cia;
  ulong time;
  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};
  if (!ADS_Status()) {
    Time(&time);
    ADSStartTime = time;
    printf("time now: %ld", ADSStartTime);
    Setup_ADS(true, 0L, 8); // Bitshift of 8 gives us a 13second power buffer
  }
  // printf(ProgramDescription);

  CmdStdSetup(&cia, PWRCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {
    AD_Check();
    printf("\n%s", cia.errmes);
    fflush(stdout);
  }

  SystemSettings();
}
void SamplePWR() { printf("\nVoltage: %5.2f", Voltage_Now()); }
void StartPWRCmd() {
  if (!ADS_Status())
    Setup_ADS(true, 0L, 8);
}
void StopPWRCmd() {
  static char filename[] = "c:00000000.dat";
  ulong timenow;
  int filehandle;
  Time(&timenow);
  sprintf(&filename[2], "diagnost.log");
  timenow = timenow - ADSStartTime; //-timedifference from gps;
  printf("\nTime Now: %ld", timenow);
  cdrain();
  coflush();
  filehandle = open(filename, O_WRONLY | O_CREAT | O_TRUNC);
  Power_Monitor(timenow, filehandle, 0);
  close(filehandle);
}

/******************************************************************************\
** SystemSettings(
\******************************************************************************/
#ifdef IRIDIUM
void SetupIridiumCmd() {

  CmdInfo cia, *cipa = &cia;

  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};
  // OpenTUPort_AntMod(true);
  // printf(ProgramDescription);
  SettingsPointer = IRIDSettings;
  CmdStdSetup(&cia, IridiumCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {
    PWR(AD_Check();)

    printf("\n%s", cia.errmes);
    fflush(stdout);
  }

  // OpenTUPort_AntMod(false);
  SystemSettings();
}
#endif
/******************************************************************************\
** SystemSettings(
\******************************************************************************/
#ifdef CTDSENSOR
void SetupCTDCmd() {

  CmdInfo cia, *cipa = &cia;

  char *ProgramDescription = {"\n"
                              "Type HELP to view commands\n"};
  // SettingsPointer=IRIDSettings;

  // printf(ProgramDescription);

  CmdStdSetup(&cia, CTDCmdTable, 0);
  cipa->privLevel = ModifyPermission ? 100 : 1;

  while (CmdStdRun(&cia) != CMD_BREAK) {
    PWR(AD_Check();)
    if (tgetq(CTDPort))
      CTD_Data(1);
    printf("\n%s", cia.errmes);
    fflush(stdout);
  }

  SystemSettings();
}
void StartCTDCmd() { CTD_Start_Up(1, false); }
void StopCTDCmd() { OpenTUPort_CTD(false); }
#endif
/******************************************************************************\
** FetchSettings(Settings)
\******************************************************************************/
void FetchSettings(Settings *VEESettings) {

  Settings *setp = VEESettings;
  VEEData vdp;

  //
  //	READ IN THE CURRENT VARIABLES
  //
  // printf("\nVARIABLE:           CURRENT:            DEFAULT: DIFF:\n");
  while (setp->optName) {
    vdp = VEEFetchData(setp->optName);
    if (vdp.str == 0) // not found, create it
    {
      if (!ModifyPermission) {
        printf("Didn't find %s, do not use this system\n", setp->optName);
        setp++;
        continue;
      }
      printf("Creating %s with default setting...\n", setp->optName);
      if (!VEEStoreStr(setp->optName, setp->optDefault)) {
        printf("!!! VEE store failed, do not use this system\n");
        break;
      } else
        continue; // without increment, to see setting
    }
    setp->optCurrent = vdp.str;
    //	printf("%-20s%-20s%-20s%s\n", setp->optName, setp->optCurrent,
    //setp->optDefault,
    //	strcmp(setp->optCurrent, setp->optDefault) ? "*!*" : "");
    setp++;
  }
}
/******************************************************************************\
**	SetupHelpCmd
\******************************************************************************/
char *SetupHelpCmd(CmdInfoPtr cip) {
  char *HELPhelpText = {
      "\n"
      "Type HELP without parameters to display the list of Setup commands\n"
      "and abbreviated syntax. Type HELP followed by a command or setting\n"
      "name to displaycomprehensive usage information.\n"
      "\n"};
  enum { help, cmd };
  char cmdbuf[32];
  Settings *setp = SYSSettings;

  DosSwitch QMsw = {"/", '?', 0, 0}; // help

  CmdExtractCIDosSwitches(cip, "?", &QMsw);

  // HELP SWITCH SELECTED (common format to many commands)
  if (QMsw.pos)
    return HELPhelpText;

  if (ARGS == help)
    return PDCCmdStdHelp(cip);

  //	FIRST CHECK SETTING MATCH
  while (setp->optName) {
    if (strcmp(setp->optName, cip->argv[cmd].str) == 0) {
      printf("\n%s\n", setp->optDesc);
      return 0;
    }
    setp++;
  }

  //	THAT FAILEDS, TRY EXTENDED COMMAND HELP

  strncpy(cmdbuf, cip->argv[cmd].str, sizeof(cmdbuf));
  sprintf(cip->line, "%s /?", cmdbuf);

  CmdParse(cip); // gives us args for future initargs call
  CmdDispatch(cip);
  CmdSetNextCmd(cip, 0); // don't repeat
  if (cip->errmes != CmdErrUnknownCommand)
    return cip->errmes;

  return 0;

} //____ SetupHelpCmd() ____//

/******************************************************************************\
**	SetupListCmd
\******************************************************************************/
char *SetupListCmd(CmdInfoPtr cip) {
  char *LISThelpText = {
      "\n"
      "Type LIST to display a list of the current settings\n"
      "\n"
      "  /V          Show detailed descriptions of each settings function.\n"
      "\n"};

  Settings *setp = SettingsPointer;
  VEEData vdp;

  DosSwitch QMsw = {"/", '?', 0, 0}; // help
  DosSwitch vsw = {"/", 'V', 0, 0};  // verbose

  CmdExtractCIDosSwitches(cip, "?v", &QMsw, &vsw);

  // HELP SWITCH SELECTED (common format to many commands)
  if (QMsw.pos)
    return LISThelpText;

  printf(
      "\nVARIABLE:           CURRENT:            DEFAULT:            DIFF:\n");
  while (setp->optName) {
    vdp = VEEFetchData(setp->optName);
    if (vdp.str == 0) // not found, create it
    {
      printf("Didn't find %s, do not use this system\n", setp->optName);
      setp++;
      continue;
    }

    setp->optCurrent = vdp.str;

    printf("%-20s%-20s%-20s%s\n", setp->optName, setp->optCurrent,
           setp->optDefault,
           strcmp(setp->optCurrent, setp->optDefault) ? "*!*" : "");
    if (vsw.pos)
      printf("%s\n", setp->optDesc);

    setp++;
  }

  return 0;

} //____ SetupListCmd() ____//
/******************************************************************************\
**	SetupDefaultsCmd
\******************************************************************************/
char *SetupDefaultsCmd(CmdInfoPtr cip) {

  char *SEThelpText = {
      "\n"
      "Resets one or all of the settings to their default states\n"
      "\n"};

  DosSwitch QMsw = {"/", '?', 0, 0}; // help

  CmdExtractCIDosSwitches(cip, "?", &QMsw);

  // HELP SWITCH SELECTED (common format to many commands)
  if (QMsw.pos)
    return SEThelpText;

  ResetDefaultSettings(SYSSettings);
#ifdef WISPR
  ResetDefaultSettings(WISPSettings);
#endif
#ifdef IRIDIUM
  ResetDefaultSettings(IRIDSettings);
#endif
#ifdef BLUETOOTH
  ResetDefaultSettings(BTSettings);
#endif

  printf("\n");
  return 0;

} //____ SetupDefaultsCmd() ____//
/******************************************************************************\
** ResetDefaultSettings()
\******************************************************************************/
void ResetDefaultSettings(Settings *VEESettings) {

  Settings *setp = VEESettings;
  VEEData vdp;
  bool reset;

  printf(
      "\nVARIABLE:           CURRENT:            DEFAULT:            RESET:\n");
  while (setp->optName) {
    vdp = VEEFetchData(setp->optName);
    if (vdp.str == 0) // not found, create it
    {
      printf("Creating %s with default setting...\n", setp->optName);
      if (!VEEStoreStr(setp->optName, setp->optDefault)) {
        printf("!!! VEE store failed, do not use this system\n");
        break;
      } else
        continue; // without increment, to see setting
    }
    reset = false; // assume we won't need to
    if (strcmp(setp->optCurrent, setp->optDefault) != 0) {
      if (!VEEStoreStr(setp->optName, setp->optDefault))
        printf("!!! VEE store failed, do not use this system\n");
      else {
        reset = true;
        setp->optCurrent = setp->optDefault;
      }
    }
    printf("%-20s%-20s%-20s%s\n", setp->optName, setp->optCurrent,
           setp->optDefault, reset ? "*!*" : "");
    setp++;
  }
  return;
}
/******************************************************************************\
**	AllDirsCmd
\******************************************************************************/
char *AllDirsCmd(CmdInfoPtr) {
  short i;

  cprintf("\n\n ============================= ALL DIRS "
          "============================\n");

  execstr("DIR C: /V");

  for (i = 0; i < 10; i++) {
    if (kbhit())
      if (cgetc() == '.')
        break;
    if (execstr("MOUNT BIHD-%u /V", i) == 0)
      execstr("DIR C: /V");
    else
      break;
  }

  return 0;

} //____ AllDirsCmd() ____//
/**********************************************************************************************\
**	void DisplayParameters()
\**********************************************************************************************/
void DisplayParameters(FILE *stream) {

  VEEData vdp;
  Settings *setp = SYSSettings;

  RTCDelayMicroSeconds(10000L);

  fprintf(stream, "SETTING NAME:       SETTING VALUE:\n");

  while (setp->optName) {

    vdp = VEEFetchData(setp->optName);
    if (vdp.str == 0) {
      fprintf(stream, "Didn't find %s, do not use this system\n",
              setp->optName);
      setp++;
      continue;
    }
    setp->optCurrent = vdp.str;

    fprintf(stream, "%-20s%-20s\n", setp->optName, setp->optCurrent);
    setp++;
  }

  RTCDelayMicroSeconds(10000L);
}
