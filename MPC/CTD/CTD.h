#define SBE19

int CTD_OpenTUPort(int sbe); // 0=off, 1=buoy, 2=ant, returns 0 on success
bool CTD_Start_Up(int sbe, bool settime);
bool CTD_GetPrompt();
void CTD_DateTime();
bool CTD_Data();
void CTD_Sample();
void CTD_SampleBreak();
void CTD_CreateFile(long);
void CTD_SyncMode();
void CTD_GetSettings();
float CTD_CalculateVelocity();
float CTD_AverageDepth(int, float *);

extern void SelectAntMod(char); // GPSIRID.c

typedef struct {

  short UPLOAD; // A boolean 1 or 0 to decide whether to upload CTD data at
                // surface.
  //	short POLLED;		//1 for polled sampling ("TS") or 0 for
  //autnomous
  // sampling defined by SAMPINT schedule
  //	short DELAY;		//Delay in seconds between polled samples
  //	short SAMPINT;		//Autonomous sampling interval.

} CTDParameters;
