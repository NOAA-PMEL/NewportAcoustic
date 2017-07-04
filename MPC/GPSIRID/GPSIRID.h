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

/*******************
**IRIDIUM FUNCTIONS
*****************/
/* IRIDGPS:
   Automated call into RUDICS for uploading files and downloading new parameters
   return: 	-1 on failed GPS
                        -2 on failed IRID
                        1 on successful IRID && GPS
   */
short IRIDGPS();

/* GetIRIDIUMSettings:
   if #define IRIDIUM then grab necessary VEEPROM Parameters for IRIDIUM use.
   */
void GetIRIDIUMSettings();

void OpenTUPort_IRIDGPS(bool);
bool PowerOn_GPS();

#define IRIDBAUD 19200L

// if IRIDGPSMODEM defined
#define IRIDGPSPWR 23 // Iridium Power pin (1=ON, 0=OFF)
#define IRIDGPSCOM 22
#define IRIDGPSRX 32
#define IRIDGPSTX 31

// char* GetFileName(bool, bool, long*, const char*);
// INT_MAX
