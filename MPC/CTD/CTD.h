#define SBE19

bool CTD_Start_Up(int sbe, bool settime);
bool CTD_GetPrompt(int sbe);
void CTD_DateTime(int sbe);
bool CTD_Data(int sbe);
void CTD_Sample(int sbe);
void CTD_SampleBreak(int sbe);
void CTD_CreateFile(int sbe, long);
void CTD_SyncMode(int sbe);
void GetCTDSettings(int sbe);
void OpenTUPort_CTD(int sbe, bool);
float CTD_CalculateVelocity(int sbe);
float CTD_AverageDepth(int sbe, int, float *);
// void SwitchTD(char);
#define BAUD 9600L

typedef struct {

  short UPLOAD; // A boolean 1 or 0 to decide whether to upload CTD data at
                // surface.
  //	short POLLED;		//1 for polled sampling ("TS") or 0 for
  //autnomous
  // sampling defined by SAMPINT schedule
  //	short DELAY;		//Delay in seconds between polled samples
  //	short SAMPINT;		//Autonomous sampling interval.

} CTDParameters;
