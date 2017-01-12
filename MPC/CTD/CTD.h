#define SBE19

bool CTD_Start_Up(bool settime);
bool CTD_GetPrompt();
void CTD_DateTime();
bool CTD_Data(void);
void CTD_Sample();
void CTD_SampleBreak();
void CTD_CreateFile(long);
void CTD_SyncMode();
void GetCTDSettings();
void OpenTUPort_CTD(bool);
float CTD_CalculateVelocity();
float CTD_AverageDepth(int, float*);
extern TUPort* CTDPort;
#define  BAUD        9600L

typedef struct
{

	short UPLOAD;		//A boolean 1 or 0 to decide whether to upload CTD data at surface.
//	short POLLED;		//1 for polled sampling ("TS") or 0 for autnomous sampling defined by SAMPINT schedule
//	short DELAY;		//Delay in seconds between polled samples
//	short SAMPINT;		//Autonomous sampling interval. 

} CTDParameters;