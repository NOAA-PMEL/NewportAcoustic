

// AModemPort and WISPR Transmission
void AModem_Data();

ulong Winch_Ascend();
ulong Winch_Descend();
ulong Winch_Stop();
void Buoy_Status();
void WinchConsole();
void OpenTUPort_NIGK(bool);
void GetWinchSettings();
ulong Winch_Status();
void Winch_Monitor(int);
extern TUPort *NIGKPort;

extern bool Surfaced;

#define NIGK_MIN_DEPTH 6

// if ACOUSTICMODEM defined
#define AMODEMPWR 21
#define AMODEMRX 33
#define AMODEMTX 35
#define AMODEMBAUD 4800L

typedef struct {

  short TDEPTH; // CTD depth at optimal position for iridium/gps comms (Antenna
                // at surface when winch cable angle <10')
  short DELAY; // Time in seconds post TUTxAcousticModem when the Winch actually
               // accepts character. Strictly for timing & Calculation of cable
               // length
  short RRATE;  // Velocity in meters/minute of the rise (ascent) rate
  short FRATE;  // Velocity in Meters/minute of the fall (descent) rate
  short ANTLEN; // Length from CTD to antenna. More specifically: From CTD Depth
                // Sensor to water surface when antenna is out of water.
  short PROFILES; // Keep record of number of profiles
  short RECOVERY; // If 1, call in repeatedly @ specified interval. 'A' reset to
                  // 30minutes.

} WINCHParameters;

// Tracking number of calls
typedef struct {
  short ASCENTCALLS;
  short ASCENTRCV;
  short DESCENTCALLS;
  short DESCENTRCV;
  short STOPCALLS;
  short STOPRCV;
  short BUOYRCV;
  short WINCHCALLS;
} WinchCalls;