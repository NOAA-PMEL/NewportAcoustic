// IRIDIUM Structure Parameters
typedef struct {
  char PHONE[14]; // Rudics phone number 13 char long
  short MINSIGQ;  // Min Irid signal quality to proceed
  short MAXCALLS; // Maximum Iridium calls per session
  short MAXUPL;   // Max upload try per call
  short WARMUP; // IRID GPS Unit warm up in sec//Does this really need to be in
                // here?
  short ANTSW;    //=1: antenna switch; =0: no antenna switch
  short OFFSET;   // GPS and UTC time offset in sec
  short REST;     // Rest period for Iridium to call again
  short CALLHOUR; // Hour at which to call
  short CALLMODE; // 0==call on Dataxinterval, 1== call at set hour everyday.
  bool LOWFIRST;  // send file with lowest value first
} IridiumParameters;

void GetIRIDIUMSettings();
int DevSelect(int);
int AntMode(char);
int GPSIRID_Init();
bool GPSstartup();
short IRIDGPS();
long GetStringWait(char *str, short wait);

extern TUPort *devicePort;
