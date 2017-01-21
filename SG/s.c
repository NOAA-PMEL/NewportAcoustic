/******************************************************************************\
**	Seaglider 1.9.1 Program 
** 6/1/2015
** Alex Turpin
** Plans to update DBG to include all console use. (.APP won't also be looking for 
**    CONS input)
** New sleep mode for Seaglider when at Surface.
*****************************************************************************
**	STATUS descriptions:
**    1  Waiting to Leave Surface/Waiting for cat/PAM/GPS commands
**    2  Waiting for MIN_WISPR_DEPTH before turning on.
**    3  WISPR on for Descent
**    4  WISPR on for Ascent while >SG.WISPROFFDEP
**    5  Turn WISPR OFF 
**    6  Waiting to Reach Surface.
**

SEAGLIDER PARAMS
** TPU 1    22 1=MAX3223 ON
** TPU 2    23 0=SG Comm Select
** TPU 10   31 SEAGLIDER TX
** TPU 11   32 SEAGLIDER RX

WISPR BOARD
** TPU 6    27 PAM WISPR TX
** TPU 7    28 PAM WISPR RX
** TPU 8    29 PAM1&2 Enable
** TPU 9    30 PAM1=0 PAM2=1
** TPU 15   37 PAM Pulse On
** MODCK    42 PAM Pulse Off

SETTINGS                                                 Parse Character, Bytes Taken, min and max values
** SG.PWRONDEPTH  Depth of SG at which PAM powers up     I  3  005,  999
** SG.PWROFFDEP   Depth of SG at which PAM powers off    E  3  005,  999
** SG.MAXSTARTS   Maximum number of Bootups              S  4  1000, 9999
** SG.MAXUPL      Maximum number of kB to upload to SG   U  2  01,   30
** SG.DETMAX      Maximum number of Detections to call   d  2  00,   50
** SG.DETINT      Detection Interval in minutes          D  2  05,   60
** SG.WGAIN       WISPR Gain Value                       G  1  0,    3
** SG.MINVOLT     Minimum Voltage for PAM Logger to run  v  4  10.0, 15.0
** SG.DIVENUM     For File System Creation


INTERRUPTS
** IRQ4 Pin 43 Console Serial Interrupt
** IRQ2 Pin 39 SG Serial Interrupt
** IRQ5 Pin 31 PAM Serial Interrupt
**	
\******************************************************************************/

#include	<cfxbios.h>		// Persistor BIOS and I/O Definitions
#include	<cfxpico.h>		// Persistor PicoDOS Definitions

#include	<assert.h>
#include	<ctype.h>
#include	<errno.h>
#include	<limits.h>
#include	<locale.h>
#include	<math.h>
#include	<setjmp.h>
#include	<signal.h>
#include	<stdarg.h>
#include	<stddef.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<time.h>

#include	<dirent.h>		// PicoDOS POSIX-like Directory Access Defines
#include	<dosdrive.h>	// PicoDOS DOS Drive and Directory Definitions
#include	<fcntl.h>		// PicoDOS POSIX-like File Access Definitions
#include	<stat.h>		   // PicoDOS POSIX-like File Status Definitions
#include	<termios.h>		// PicoDOS POSIX-like Terminal I/O Definitions
#include	<unistd.h>		// PicoDOS POSIX-like UNIX Function Definitions


#include <Settings.h>   // For VEEPROM settings definitions
#include <ADS.h>
#include <Global.h>
#include <WISPR.h>





// WDT Watch Dog Timer definition
// Not sure if this watchdog is even working You have to define 
short	CustomSYPCR = WDT105s | HaltMonEnable | BusMonEnable | BMT32;
#define CUSTOM_SYPCR CustomSYPCR  //Enable watch dog  HM 3/6/2014


//Define Settings Maximums
#define MIN_WISPR_DEPTH  5  //Meters
#define MIN_DETECTION_INTERVAL 0   //minutes
#define MAX_DETECTION_INTERVAL 60  //minutes
#define MIN_BATTERY_VOLTAGE 11.0 //volts
#define MAX_DETECTIONS 50  //#of detections per DETINT call
#define MAX_UPLOAD 30 //kilobytes


//Define unused pins here
uchar    mirrorpins[] = { 1, 15, 16, 17, 18, 19, 21, 24, 25, 26, 28, 31, 32, 35, 36, 37, 42, 0 };




void IncrementDetections(int);
bool AD_Check();
void SetupHardware(void);
void PreRun(void);
void SGSystemStart();
int Incoming_Data();
int Console(char);
void System_Timer();
int Check_Timers();
void SGGetSettings();
char* Time(ulong*);
void Sleep();
int WriteFile(float, float, float, char*, ulong ,ulong);
int UploadFile(char*, TUPort*);
void WriteDivenum(short);
void WriteGain();
void DisplayParameters(FILE *stream);
void ParseParam();


//Not Currently Used
void Open_File(long*);
char*GetNextRECFileName(bool,bool,long*);
static char *BIRNextFileName(bool, bool);
long     FileNumber;
char     logfile[16];
char     ParamFile[sizeof "RAOBParams.txt"];

static void IRQ2_ISR(void);
static void IRQ4_ISR(void);
static void IRQ5_ISR(void);
IEV_C_PROTO(ExtFinishPulseRuptHandler);





extern void AD_Log(void);
extern float PowerMonitor(float* ,ulong);
extern float SetUpADS(bool, short);
extern int SeaGlider_Depth_Check();
extern void compileerrors(int);
extern   SGSettings	VEESettings[];
extern bool Power_on;


FILE     *file;
SGData   SG;
bool     Shutdown;
bool     Surfaced;
bool     PutInSleepMode;
int      TotalDetections;
bool     SystemOn=false;
float    GetMaxDepth();
char     time_chr[21];



bool AD_Check(){if((data==true)||(data2==true)){AD_Log(); return true;} else return false;}
void IncrementDetections(int detections){TotalDetections +=detections;}
/******************************************************************************\
**	Main
\******************************************************************************/
void main(){

   ulong PwrOn, PwrOff, TimeOn, UploadTime, UploadStop;
   char fname[sizeof "SG0000.dat"];

	float Ah, voltage;
	float max_depth;
   int ret;
   bool Uploaded;
   	
   // Initialize LogFile
	Initflog("activity.log", true);

   // Get the system settings running    
   SetupHardware();
  
   //Count Down Timer to Start Program or go to PICO dos/Settings
   PreRun();
  

	
   //Initialize system
   SGSystemStart();

   
   //Main Program Loop
   while(!Shutdown){
   
    	//Get Power On Time
	   Time(&PwrOn);
	   
      fname[0]='\0';
      Uploaded=false;  
      TotalDetections=0;
      
      while(Surfaced){ 	   
      	if(PutInSleepMode ==true)      	
            Sleep();                   //Sleep to save power

         else
            ret=Incoming_Data();           //Grabs data from TUPort Devices or logs AD_Data
         
         if(Shutdown) 
            break;
         
         if(ret==2&&!Uploaded){
            sprintf(fname, "SG%04d.dat", SG.DIVENUM);
            UploadFile(fname, SeaGlider);
            ret=0;
            Uploaded=true;
            compileerrors(5);
            }
                          
         }
         
      cprintf("\n%s|Out of Surface Upload Loop\n", Time(NULL));   
      
      //Calculate time sitting idle at Surface. No WISPR   
      Time(&UploadStop);
      UploadTime=UploadStop-PwrOn;
      
      while(!Surfaced){
         
         if(PutInSleepMode)
            Sleep();
         
         else 
            Incoming_Data();
             
         System_Timer();
         
         if(Shutdown) break;
         
         }
         
      Time(&PwrOff);
      TimeOn=PwrOff-PwrOn;
      Ah = PowerMonitor(&voltage, TimeOn);           
      
      
      SetUpADS(true, SG.DIVENUM+1);
      Time(&PwrOn);
      
      sprintf(fname, "SG%04d.dat", SG.DIVENUM);
      max_depth=GetMaxDepth();
      WriteFile(Ah, voltage, max_depth, fname, TimeOn, UploadTime);
      
      TUPortSettings("PAM", false);
      RTCDelayMicroSeconds(100000L);
      TUPortSettings("PAM", true);
      
      flogf("\n%s|SG Upload ready", Time(NULL));
      Surfaced=true;
      }
      
   flogf(". Program Resetting");
   
   //Shut off WISPR for Data Transmission
   if(Power_on){
      if(!WISPRExit()){
         if(!WISPRExit()){
            flogf(": Forcing Off");    
            WISPRPower(false);  
            }
         else
            flogf(": Found Exit");
         }   
      else
         flogf(": Found Exit");
      }      
            

   //Turn off WISPR Port   
   TUPortSettings("PAM", false);
   
   TUPortSettings("SG", false);
   
   TURelease();
   
   RTCDelayMicroSeconds(100000L);   

   BIOSReset();

}	//____ Main() ____//
/*********************************************************************************\
** SetupHardware
** Set IO pins, set SYSCLK, if surface tries to open GPS receiver and set RTC time.
** Set gain.  Return the current phase.
** 10/25/2004 H. Matsumoto
\*********************************************************************************/
void SetupHardware(void)
	{
	short		err = 0;
	short		result=0;
	short		waitsFlash, waitsRAM, waitsCF;
	ushort	nsRAM, nsFlash, nsCF;
	short		nsBusAdj;
	
	float    SG_Version;
		
	DBG( flogf("SetupHardware");  cdrain();)
	CLEAR_OBJECT(SG);	// zero entire structure (see <cf1bios.h>
	SGGetSettings();
	SG_Version= ((float)SG.VERSION/10.0);
 //  SG.VERSION = SG_VERSION * 10; 

   //	Setup runtime clock speed and optimize wait states
	CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
	TMGSetSpeed(SYSCLK);

   PZCacheSetup('C'-'A',calloc,free);


	PIOMirrorList(mirrorpins);


	flogf("\n----------------------------------------------------------------\n");
	flogf("\nProgram: %s,  Version: %3.1f,  Build: %s %s", 
		__FILE__, SG_Version, __DATE__, __TIME__);
	CSGetSysAccessSpeeds(&nsFlash, &nsRAM, &nsCF, &nsBusAdj);
	flogf("\nSystem Parameters: CF2 SN %05ld, PicoDOS %d.%02d, BIOS %d.%02d",
		BIOSGVT.CF1SerNum, BIOSGVT.PICOVersion, BIOSGVT.PICORelease,
		BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease);
	CSGetSysWaits(&waitsFlash, &waitsRAM, &waitsCF);	// auto-adjusted
	flogf("\n%ukHz nsF:%d nsR:%d nsC:%d adj:%d WF:%-2d WR:%-2d WC:%-2d SYPCR:%02d",
		TMGGetSpeed(), nsFlash, nsRAM, nsCF, nsBusAdj,
		waitsFlash, waitsRAM, waitsCF, * (uchar *) 0xFFFFFA21); fflush(NULL); coflush(); ciflush();		
	

		

  	// Enable RXTX buffered operation
	SCIRxSetBuffered(true);
	SCITxSetBuffered(true);
		
	// Identify IDs
	flogf("\nProject ID: %s   Platform ID: %s   Location: %s   \nBoot ups: %d", SG.PROJID, SG.PLTFRMID, SG.LOCATION, SG.STARTUPS);


}        //____ SetupHardware() ____//  
/******************************************************************************\
**	PreRun		Exit opportunity before we start the program
\******************************************************************************/
void PreRun(void)
	{
	short		ndelay=5;
	short		i;
	char		c;
	char 		*ProgramDescription = 
		{
		"\n"
		"The program will start in five seconds unless an operator keys in a\n"
		"period '.' character. If a period is received, you have about\n"
		"### seconds to respond to access other software functions on this\n"
		"system before a watchdog reset occurs.\n"
		"\n"
		};
	time_t		nowsecs;
	char		   strbuf[64],strbuf2[64];
	struct tm 	t;
	ushort		Ticks;


	nowsecs = RTCGetTime(NULL,&Ticks);		// get RTC clock right now	
	t = *localtime(&nowsecs);
	strftime(strbuf, sizeof(strbuf), "%m/%d/%Y  %H:%M:%S", &t);
	
	flogf("\n\nProgram start time: %s.%.3d [ctime: %lu]\n", strbuf, Ticks/40,nowsecs);
	flogf(ProgramDescription);
	flogf("You have %d seconds to launch\n", ndelay);
	

		
	for (i = ndelay; i > 0; i--)
		{
		flogf("%u..", i);
		c = SCIRxGetCharWithTimeout(1000);	// 1 second
		if (c == '.')
			break;
		if (c == -1)
			continue;
		i = 0;		// any other key ends the timeout
		}
	
	DBG(flogf("\nSTARTUPS %d..", SG.STARTUPS);)
	sprintf(strbuf2, "%u", ++SG.STARTUPS);
	DBG(flogf(".STARTUPS %d\n", atoi(strbuf2));)
	VEEStoreStr(STARTUPS_NAME, strbuf2);

	if(SG.STARTUPS > 0) 
		flogf("\nSystem reboot # %d occured at %s", SG.STARTUPS, Time(NULL));	
	
		
	if (i <= 0)
		{
		flogf("\nStarting...\n");
		return;		// to start acquisition
		}

	TickleSWSR();	// another reprieve

	QRchar("\nWhat next?(P=PicoDOS, S=Settings)",
		"%c", false, &c, "PS", true);
	if (c == 'S')
		{
		settings();  
		BIOSReset();	// force clean restart
		}
	else if (c == 'P')
		BIOSResetToPicoDOS();
		
		


	return;		// to start acquisition
	
	
}	//____ PreRun() ____//
/****************************************************************************\
** void RAOBSystemStart()
\****************************************************************************/
void SGSystemStart(){

   //Turn System on. Only Exit upon console '1' input or Parsing 'R' from "RAOB(R)"
	SystemOn=true;
	Surfaced=true;
	
	//Turn on A to D Logging for Power Consumption and return system time interval in minutes
	SetUpADS(true, SG.DIVENUM);
	DetectionTimer=0.0;
	
	// Initialize and open TPU UART
	TUInit(calloc, free);
   
   //Open PAM Tuport
   TUPortSettings("PAM", true);
   TUPortSettings("SG", true); 
   

}        //____ RAOBSystemStart() ____//
/****************************************************************************\
** void Incoming_Data()
\****************************************************************************/
int Incoming_Data(void){
   bool incoming=true;
   int ret=0;
   
   while(incoming){  
      //Check A to D Power Sampling Buffers
      AD_Check();    
            
      //Console Wake up.
      if(cgetq()){ 
         DBG(flogf("CONSOLE INCOMING");)
         Console(cgetc()); 
         incoming =true;
         }
      else if(tgetq(PAM)>5){
         DBG(flogf("WISPR INCOMING");)
         ret = WISPR_Data();

         incoming=true;
         }
      else if(tgetq(SeaGlider)>5){
         DBG(flogf("SG INCOMING");)
         ret=SeaGlider_Depth_Check();
         incoming=true;
         }
      else{ 
         incoming =false; 
         PutInSleepMode=true;
         }
   }
   return ret;

}        //_____ Incoming_SeaGlider_Data() _____//
/************************************************************************************************************************\
** void Console      Testing Opportunity for Debugging
\************************************************************************************************************************/
int Console(char in){

   short first;
   short second;
   short gain;
   char* returnnum="00";
   short detections;

  DBG(  flogf("Incoming Char: %c", in);)
    RTCDelayMicroSeconds(2000L);
   if(in == 'G'){
      WISPRGPS(124.5, 45.5);
      }
   else if(in== 'I')
      WISPRPower(true);
      
   else if(in=='N'){
      gain=SCIRxGetByte(true);
      WISPRGain(gain-48);
      }
      //'D' followed by two numbers to be multiplied to get return detections.
   else if(in == 'D'){
      first =cgetc();
      second = cgetc();
      detections= (first-48)*(second-48);

      if(WISPRDet(detections)<0)
         WISPRPowerCycle();
   }
   else if(in == 'F')
      WISPRDFP();
   
   else if(in == 'E'){
   
      if(Power_on){
         TUTxFlush(PAM);
         TURxFlush(PAM);
         if(!WISPRExit()){
            TUTxFlush(PAM);
            TURxFlush(PAM);
            if(!WISPRExit()){
               flogf(": Forcing Off");    
               WISPRPower(false);  
               }
            }   
         }  
   
      }
   else if(in == '1'){
      DBG(flogf("Shutting down");)
      Shutdown=true;
      Surfaced=true;
      RTCDelayMicroSeconds(20000L);   
      return 2;
   }

  
   PutInSleepMode=true;      
   return 0;
   

}
/************************************************************************************************************************\
** void System_Timer();
** This function deals with different triggers to our alarm clock function and sets them active. 
** Variables:
** DetectionTimer=FWT 5minutes (Based off of ADS(File Write Time)
** DETINT (mins)=    The frequency we call the detection request from WISPR
\************************************************************************************************************************/
void System_Timer(){
 
   if(Power_on){
      if(SG.DETINT==0){ DetectionTimer=0.0; return;}
      if(SG.DETINT/DetectionTimer==1.0){     
         flogf("\n%s|Detection Alarm, DETMAX:%d", Time(NULL), SG.DETMAX);
         WISPRDet(SG.DETMAX);
         
         DetectionTimer=0.0;
         PutInSleepMode=false;
         }
      }
  
}        //____ System_Timer() _____//
/**********************************************************************************************************************\
** void CheckTimerIntervals()
** Check the input values from cmdfile() and makes sure  they are divisible by SysTimeInt. If not, rounds them up.
/**********************************************************************************************************************/
int Check_Timers(unsigned int dividened){
   
   short result;
   char strbuf1[10];
   char *p;
   unsigned int rounded;
   
   rounded= (dividened + 4)/5;
   result=  rounded*5;
   DBG(flogf("result of check timer rounding: %d", result);)
   
   if(result>MAX_DETECTION_INTERVAL)
      result=MAX_DETECTION_INTERVAL;
   else if(result<MIN_DETECTION_INTERVAL)
      result=MIN_DETECTION_INTERVAL;
   else if(SG.DETINT!=result){
      sprintf(strbuf1, "%u", result);
      VEEStoreStr(DETECTINT_NAME, strbuf1);
      RTCDelayMicroSeconds(100000L);
      p = VEEFetchData(DETECTINT_NAME).str;
   	SG.DETINT = atoi(p ? p : DETECTINT_DEFAULT);
      flogf("\n%s|Check_Timers(DETINT): %d", Time(NULL), SG.DETINT);
      }
     return (int)result; 
   
}        //____ Check_Timers() ____//
/*********************************************************************************\
** VEEPROM GetSettings     Read settings from VEE or use defaults if not available
\*********************************************************************************/
void SGGetSettings(void)
   {
   
   char *p;
   
   p = VEEFetchData(WISPRGAIN_NAME).str;
	SG.WGAIN = atof(p ? p : WISPRGAIN_DEFAULT);
	DBG( uprintf("WISPRGAIN=%u (%s)\n", SG.WGAIN, p ? "vee" : "def");  cdrain();)

   p = VEEFetchData(DETECTINT_NAME).str;
	SG.DETINT = atof(p ? p : DETECTINT_DEFAULT);
	DBG( uprintf("DETECTINT=%u (%s)\n", SG.DETINT, p ? "vee" : "def");  cdrain();)

   p = VEEFetchData(DETECTIONMAX_NAME).str;
	SG.DETMAX = atof(p ? p : DETECTIONMAX_DEFAULT);
	DBG( uprintf("DETECTIONMAX=%u (%s)\n", SG.DETMAX, p ? "vee" : "def");  cdrain();)
	
   p = VEEFetchData(PWRONDEPTH_NAME).str;
	SG.PWRONDEPTH = atof(p ? p : PWRONDEPTH_DEFAULT);
	DBG( uprintf("PWRONDEPTH=%u (%s)\n", SG.PWRONDEPTH, p ? "vee" : "def");  cdrain();)
	
	p = VEEFetchData(PWROFFDEP_NAME).str;
	SG.PWROFFDEP = atof(p ? p : PWROFFDEP_DEFAULT);
	DBG( uprintf("PWROFFDEP=%u (%s)\n", SG.PWROFFDEP, p ? "vee" : "def");  cdrain();)	
   
   p = VEEFetchData(DIVENUM_NAME).str;
	SG.DIVENUM = atoi(p ? p : DIVENUM_DEFAULT);
	DBG( uprintf("DIVENUM=%u (%s)\n", SG.DIVENUM, p ? "vee" : "def");  cdrain();)
   
	p = VEEFetchData(PROJID_NAME).str;
	strncpy(SG.PROJID, p ? p : PROJID_DEFAULT, sizeof(SG.PROJID));
	DBG( uprintf("PROJID=%s (%s)\n", SG.PROJID, p ? "vee" : "def");  cdrain();)
	
	p = VEEFetchData(PLTFRMID_NAME).str;
	strncpy(SG.PLTFRMID, p ? p : PLTFRMID_DEFAULT, sizeof(SG.PLTFRMID));
	DBG( uprintf("PLTFRMID=%s (%s)\n", SG.PLTFRMID, p ? "vee" : "def");  cdrain();)
	
	p = VEEFetchData(STARTUPS_NAME).str;
	SG.STARTUPS = atoi(p ? p : STARTUPS_DEFAULT);
	DBG( uprintf("STARTUPS=%d (%s)\n", SG.STARTUPS, p ? "vee" : "def");  cdrain();)
   
   p = VEEFetchData(MINVOLT_NAME).str;
	SG.MINVOLT = atof(p ? p : MINVOLT_DEFAULT);
	DBG( uprintf("MINVOLT=%4.1 (%s)\n", SG.STARTUPS, p ? "vee" : "def");  cdrain();)
   
   p = VEEFetchData(VERSION_NAME).str;
	SG.VERSION = atoi(p ? p : VERSION_DEFAULT);
	DBG( uprintf("VERSION=%d (%s)\n", SG.VERSION, p ? "vee" : "def");  cdrain();)
   
   }  //____ GetSettings() ____//
/******************************************************************************\
**	Time & Date String
** Get the RTC time seconds since 1970 and convert it to an
** understandable format
\******************************************************************************/
char* Time(ulong *seconds)
{
	
	RTCtm	*rtc_time;				
	ulong	secs = NULL;

   
	ushort ticks;

	RTCGetTime(&secs, &ticks);
	rtc_time = RTClocaltime (&secs);
	*seconds=secs;
	sprintf(time_chr,"%.2d/%.2d/%.4d %.2d:%.2d:%.2d",
	rtc_time->tm_mon + 1, rtc_time->tm_mday, rtc_time->tm_year + 1900,\
	rtc_time->tm_hour,rtc_time->tm_min,rtc_time->tm_sec);
	return time_chr;
	
}  	   //____ TimeDate() ____//
/******************************************************************************\
**	SleepUntilWoken		Finish up
**	
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void Sleep(void){


	IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector);     //Console Interrupt
	IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
   IEVInsertAsmFunct(IRQ2_ISR, level2InterruptAutovector);
   IEVInsertAsmFunct(IRQ2_ISR, spuriousInterrupt);
   IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector);     //PAM Interrupt
	IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);
	
	CTMRun(false);
	SCITxWaitCompletion();
	EIAForceOff(true);         // turn off RS232 Transmitters
	QSMStop();
	CFEnable(false);

   PinBus(IRQ2);                    // Console Interrupt
   while((PinTestIsItBus(IRQ2))==0)
      PinBus(IRQ2); 
   PinBus(IRQ4RXD);                    // Console Interrupt
   while((PinTestIsItBus(IRQ4RXD))==0)
      PinBus(IRQ4RXD); 
   PinBus(IRQ5);                       // PAM Interrupt
   while((PinTestIsItBus(IRQ5))==0)
      PinBus(IRQ5);   


	while (PinRead(IRQ5)&&PinRead(IRQ4RXD)&&PinRead(IRQ2)&&((data==false)&&(data2==false)))  
		LPStopCSE(LPMODE);		
   
   QSMRun();
	EIAForceOff(false);			// turn on the RS232 Transmitters
	CFEnable(true);				// turn on the CompactFlash card
   CTMRun(true);

   PIORead(IRQ2);
   while((PinTestIsItBus(IRQ2))!=0)
      PIORead(IRQ2);
   PIORead(IRQ4RXD);                      //Console
   while((PinTestIsItBus(IRQ4RXD))!=0)
      PIORead(IRQ4RXD);
   PIORead(IRQ5);                         //PAM
   while((PinTestIsItBus(IRQ5))!=0)
      PIORead(IRQ5);
      
	
	PutInSleepMode=false;
   
	RTCDelayMicroSeconds(1000L);
					
}     	//____ Sleep() ____//
/******************************************************************************\
**	ExtFinishPulseRuptHandler		IRQ5 logging stop request interrupt
**	
\******************************************************************************/
IEV_C_FUNCT(ExtFinishPulseRuptHandler)
{
	#pragma unused(ievstack)	// implied (IEVStack *ievstack:__a0) parameter

	PinIO(IRQ5);
	SystemOn = false;    //Hmmmm? Interrupt button?? or Pam?
	
	PinRead(IRQ5);
	
}	      //____ ExtFinishPulseRuptHandler() ____//
/*************************************************************************\
**  static void Irq2ISR(void)
\*************************************************************************/	
static void IRQ2_ISR(void)
{
PinIO(IRQ2);
RTE();
}	      //____ Irq2ISR ____//
/*************************************************************************\
**  static void IRQ4_ISR(void)
\*************************************************************************/	
static void IRQ4_ISR(void)
{
PinIO(IRQ4RXD);
RTE();
}	      //____ Irq4ISR() ____//
/*************************************************************************\
**  static void IRQ5_ISR(void)
\*************************************************************************/	
static void IRQ5_ISR(void)
{
PinIO(IRQ5);
RTE();
}	      //____ Irq5ISR() ____//
/******************************************************************************\
**	WriteFile      //The Engineering file to be compiled every #minutes
** 1)Detections: Average/Median
** 2)Sample detections
** 3)Power data
\******************************************************************************/
int WriteFile(float Ah, float voltage, float max_dep, char *fname, ulong TotalSeconds, ulong uploadseconds)
{
   float energy=0.0;
   char dtxfname[sizeof "DTX0000.dat"];
   uchar *dtxstring;
   struct stat fileinfo;
   long BlkLength=1024;
   long LastBlkLength;
   long NumBlks;
   int i;
   FILE* uploadfile;
   FILE* dtxfile;
  
   dtxfname[0]='\0';
   dtxstring=(uchar*) malloc(BlkLength);
   memset(dtxstring, 0, BlkLength*(sizeof dtxstring   [0]));
   sprintf(dtxfname, "DTX%04d.dat", SG.DIVENUM);
   
     
   //Open Upload File
	if((uploadfile = fopen(fname, "w")) == NULL){  // Open the file
	   flogf("\nCannot open upload file");
	   return false;
   }
	//Print to Upload file of System parameters
   energy=voltage*Ah*TotalSeconds/1000;
	fprintf(uploadfile, "\nMax Depth: %.2f Meters\nCurrent: %.3fA\nVoltage: %.2fV\nEnergy: %.1fkJ\nTime: %lu Seconds\nUpload Time: %lu Seconds\nWISPR FS:%.2f%%\nDetections: %d\n", max_dep, Ah, voltage, energy, (ulong)TotalSeconds, (ulong)uploadseconds, WISPRFreeSpace, TotalDetections);
   flogf("\nMax Depth: %.2f Meters\nCurrent: %.3fA\nVoltage: %.2fV\nEnergy: %.1fkJ\nTotal Time: %lu Seconds\nUpload Time: %lu Seconds\nWISPR FS:%.2f%%\nDetections: %d\n", max_dep, Ah, voltage, energy, (ulong)TotalSeconds, (ulong)uploadseconds, WISPRFreeSpace, TotalDetections);
 
   //Print SG System Parameters to File
   DisplayParameters(uploadfile);
   
   
   //Open DTX file for appending to Upload file
   if((dtxfile = fopen(dtxfname, "r")) == NULL)  
      flogf("\n%s|Cannot open DTX file for appending upload file", Time(NULL));
      
   else{
      stat(dtxfname, &fileinfo);
      DBG(cprintf("\nDTXFile Size: %ld\n", fileinfo.st_size);)
      NumBlks=fileinfo.st_size/BlkLength;
      LastBlkLength=fileinfo.st_size%BlkLength;
      DBG(cprintf("\nDTX blocks: %ld, last block: %ld\n", NumBlks, LastBlkLength);)
      memset(dtxstring, 0, BlkLength*(sizeof dtxstring   [0]));

      for(i=0;i<NumBlks;i++){
         fread(dtxstring, sizeof(uchar), BlkLength, dtxfile);
         flogf("%s", dtxstring);
         fprintf(uploadfile, "%s", dtxstring);
         }
      dtxstring = (uchar*)realloc(dtxstring, LastBlkLength);
      //memset(dtxstring, 0, LastBlkLength*(sizeof dtxstring  [0]));
      fread(dtxstring, sizeof(uchar), LastBlkLength, dtxfile);
      flogf("%s", dtxstring);
      fprintf(uploadfile, "%s", dtxstring);

      fclose(dtxfile);
      }
      
	fclose(uploadfile);
	RTCDelayMicroSeconds(100000L);
	
   return 0;

}        //____ WriteFile() ____//
/******************************************************************************\
**	UploadFile      Start checking for upload command and send the file off
\******************************************************************************/
int UploadFile(char *fname, TUPort* Port)
{

   uchar *buffer;
   TUChParams PortParams;
   struct stat info;
   short BlkLength, BufferSize, NumBlks, LastBlkLength, BlkNum;
   
   flogf("\n%s|UPLOAD FILE: %s", Time(NULL), fname);

   TUTxFlush(Port);
   TURxFlush(Port);
   
   TUGetCurrentParams(Port, &PortParams);
   BufferSize=PortParams.txqsz;
   BlkLength=BufferSize;
   
   stat(fname, &info);   
   NumBlks=info.st_size/BlkLength;
   LastBlkLength=info.st_size%BlkLength;
   

   //cprintf("\nFile Size: %ld, BSZ: %d",info.st_size, BufferSize, BlkLength);
   //cprintf("\nNumber of Blocks: %d, Last Blk Length: %d", NumBlks+1, LastBlkLength);
   
   if((file = fopen(fname, "r")) == NULL){  
      flogf("\n%s|UPLOAD FILE ERROR: Cannot open file", Time(NULL));
      return -1;
      }
   
   for(BlkNum=0;BlkNum<=NumBlks;BlkNum++){
      
      if(BlkNum==NumBlks) BlkLength=LastBlkLength;
      
      
      buffer=(uchar *) malloc(BlkLength);
      memset(buffer, 0, BlkLength * (sizeof buffer[0])); //Flush the buffer
      RTCDelayMicroSeconds(20000L);
      fread(buffer, sizeof(uchar), BlkLength, file);

      cdrain();
      coflush();
      
      flogf("\n%s|Block %d, Bytes: %d", Time(NULL), BlkNum+1, BlkLength);
      
      TUTxFlush(Port);
      TURxFlush(Port);
      TUTxPutBlock(Port, buffer, BlkLength, 5000);   //Dump the contents of the buffer through the serial line
      TUTxWaitCompletion(Port);
      RTCDelayMicroSeconds(100000L);
      }
   
   fclose(file);

   RTCDelayMicroSeconds(10000L);

   
   return 1;



}        //____ UploadFile() ____//
/******************************************************************************\
**	Write Dive Number
\******************************************************************************/
void WriteDivenum(short newdivenum){

   char strbuf[4];
	SG.DIVENUM=newdivenum;
	sprintf(strbuf, "%u", SG.DIVENUM);
	VEEStoreStr(DIVENUM_NAME, strbuf);
	

}        //____ WriteStatus() ____//
/******************************************************************************\
**	Write Gain
\******************************************************************************/
void WriteGain(){

   char strbuf[4];
	
	sprintf(strbuf, "%u", SG.WGAIN);
	VEEStoreStr(WISPRGAIN_NAME, strbuf);

}        //____ WriteStatus() ____//
/**********************************************************************************************\
**	void DisplayParameters()
\**********************************************************************************************/
void DisplayParameters(FILE *stream){
   
   VEEData vdp;
	SGSettings	*setp = VEESettings;
   

   fprintf(stream, "SETTING NAME:       SETTING VALUE:\n");
   
   while(setp->optName){
   
      vdp = VEEFetchData(setp->optName);
      if(vdp.str==0){
         fprintf(stream, "Didn't find %s, do not use this system\n", setp->optName);
         setp++;
         continue;
         }
      setp->optCurrent = vdp.str;   
      
      fprintf(stream, "%-20s%-20s\n", setp->optName, setp->optCurrent);
      cprintf("%-20s%-20s\n", setp->optName, setp->optCurrent);
		setp++;
		}
		

	
}
/***********************************************************************************************\
** void parseparam(char*);
\***********************************************************************************************/
void ParseParam(char* param){
   
   char* string;
   char* string2={""};
   short value;
   float floating;

   string2=(char*)malloc(4);
   
   flogf("\n%s|ParseParam%s", Time(NULL), param);

   //WISPR Gain
   if((string=strchr(param, 'G'))!=NULL){
      strncpy(string2, string+1, 1);
      value=atoi(string2);
      if(value<0||value>3){
         DBG(flogf("\nNot Updating WGain..%d.", value);)
         }
      else if(value!=SG.WGAIN){
         SG.WGAIN=value;
         flogf("\n%s|Writing new WGAIN value: %d", Time(NULL), value);
         sprintf(string2, "%d", value);
         VEEStoreStr(WISPRGAIN_NAME, string2);
         }
      }

   //Maximum Upload in KB (1000byte blocks) 2 digit 
   if((string=strchr(param, 'U'))!=NULL){
      strncpy(string2, string+1, 2);
      value = atoi(string2);
      if(value<=0){
         DBG(flogf("Not updating max upload..."); )
         value=1;
         }
      else if(value>MAX_UPLOAD){
         DBG(flogf("\nUpload at Maximum ");)
         value=MAX_UPLOAD;
         }
      if(value!=SG.MAXUPL){
         SG.MAXUPL=value;
         flogf("\n%s|Writing new MAXUPL value: %d", Time(NULL), value);
         sprintf(string2, "%d", value);
         VEEStoreStr(MAXUPL_NAME, string2);
         }           
      }
     
   //Maximum number of detections 2 digit  
   if((string=strchr(param, 'd'))!=NULL){
      memset(string2, 0, sizeof(string2));
      strncpy(string2, string+1, 2);
      value=atoi(string2);
      if(value<0){
         DBG(flogf("\nNot updating max det..");)
         value=0;
         }
      else if(value>MAX_DETECTIONS){
         DBG(flogf("\nMax Detections now being returned");)
         value=MAX_DETECTIONS;
         }
      if(value!=SG.DETMAX){   
         SG.DETMAX=value;
         flogf("\n%s|Writing new DETMAX value: %d", Time(NULL), value);
         sprintf(string2, "%d", SG.DETMAX);
         VEEStoreStr(DETECTIONMAX_NAME, string2);
         
         }
      }     
       
   //Detection Interval     
   if((string=strchr(param, 'D'))!=NULL){
      memset(string2, 0, sizeof(string2));
      strncpy(string2, string+1, 2);
      value=atoi(string2);
      value =Check_Timers(value);
      if(value!=SG.DETINT){
         SG.DETINT=value;
         Check_Timers(SG.DETINT);
         flogf("\n%s|Writing new DETINT value: %d", Time(NULL), SG.DETINT);
            }
      }
      
   //Power on Depth   
   if((string=strchr(param, 'I'))!=NULL){
      memset(string2, 0, sizeof(string2));
      strncpy(string2, string+1, 3);
      DBG(flogf("\nPower on statement: string2: %s", string2);)
      value = atoi(string2);
      if(value<MIN_WISPR_DEPTH)
         value=MIN_WISPR_DEPTH;
      else if(value>=999)
         value=999;
      if(value!=SG.PWRONDEPTH){
         flogf("\n%s|Writing new PWRONDEPTH value: %d", Time(NULL), value);
         SG.PWRONDEPTH=value;
         sprintf(string2, "%d", value);
         VEEStoreStr(PWRONDEPTH_NAME, string2);
         }
      }   
         
   //Power off Depth   
   if((string=strchr(param, 'E'))!=NULL){
      memset(string2, 0, sizeof(string2));
      strncpy(string2, string+1, 3);
      DBG(flogf("\nPower off statement: string2: %s", string2);)
      value = atoi(string2);
      if(MIN_WISPR_DEPTH>value)
         value=MIN_WISPR_DEPTH;
      else if(value>=999)
         value=999;
      if(value!=SG.PWROFFDEP){
         flogf("\n%s|Writing new PWROFFDEPTH value: %d", Time(NULL), value);
         SG.PWROFFDEP=value;
         sprintf(string2, "%d", value);
         VEEStoreStr(PWROFFDEP_NAME, string2);
         }
      }   
         
   //Maximum Startups 4 digit 
   if((string=strchr(param, 'S'))!=NULL){
      memset(string2, 0, sizeof(string2));
      strncpy(string2, string+1, 4);
      DBG(flogf("\nMaximum startups: string2: %s", string2);)
      value = atoi(string2);
      if(value<=0||value>9999){
         DBG(flogf("\nNot updating startupsmax..%s.", string2); )
         }
      else{
         SG.MAXSTARTS=value;
         flogf("\n%s|Writing new MAXSTARTS value: %d", Time(NULL), value);
         sprintf(string2, "%d", value);
         VEEStoreStr(MAXSTARTS_NAME, string2);   
         }         
      }

   //Minimum System Voltage   
   if((string=strchr(param, 'v'))!=NULL){  
      memset(string2, 0, sizeof(string2));
      strncpy(string2, string+1, 4);  
      //cprintf("\nvolt string2: %s\n", string2);
      floating=atof(string2);
      if(floating<MIN_BATTERY_VOLTAGE){
         DBG(flogf("\nnot updating..voltage.");)
         }
      else if(floating!=SG.MINVOLT){
         SG.MINVOLT=value;
         flogf("\n%s|Writing new MINVOLT value: %3.1f", Time(NULL), floating);
         sprintf(string2, "%3.1f", floating);
         VEEStoreStr(MINVOLT_NAME, string2);
         }
      }
      
   if((string=strchr(param, 'R'))!=NULL){
         flogf("\n%s|Program Reset Parsed...", Time(NULL));
      Shutdown=true;
      }
      
 
   
}       //____ parseparam() ____//
/*******************************************************************************
** Open_File
** Open a POSIX file to write data
****************************************************************************/	
void Open_File(long *fcounter)
{
long  counter;	
static char	*fnew="C:00000000.LOG";				   //first file

	fnew=GetNextRECFileName(true, true, &counter); //Find the next file number 0000xxxx.dat. Don't increment the file number yet.
	//printf("COunter %ld\n", counter);
	*fcounter=counter;
	/*
	*fcounter=counter;
	fp = (FILE *) &AcqFileHandle;
	if ((AcqFileHandle = open(fnew, O_RDWR | O_CREAT | O_TRUNC)) <= 0)
		{
		fp = 0;
		printf("Couldn't open data file!\n");putflush();CIOdrain();
		return fp;
		}
	DBG(printf("%s is created\n",fnew);putflush();CIOdrain();)
	strcpy(logfile,fnew);
	return fp;
	*/
	strcpy(logfile,fnew);

	//Initflog(RAOT.LOGFILE, LOGTOCON ? true : false);
	logfile[strlen(logfile)]=0;
	Initflog(logfile, true);
	RTCDelayMicroSeconds(200000L);
	DBG(printf("%s is created\n",logfile+2);putflush();CIOdrain();)
   //return fnew;
}
/******************************************************************************\
**	GetNextRECFileName
** Search the file with *.REC extension created latest and get the next file 
** name with file number as the counter. File counter is the first 8 numerals, 
** e.g., ########.REC. The numeral is increated by 1 to create a latest file.
** If it sees 00000100.REC in the directory, it creates 00000101.REC next.
** H. Matsumoto		
\******************************************************************************/
static char *GetNextRECFileName(bool hunt, bool incIndex, long *fcounter)
	{
	static long		counter = -1;		// 2003-08-21
	long			   val, maxval = -1;	// 2003-08-21
	static char		dfname[] = "x:00000000.LOG";
	static char		path[] = "x:";
	DIRENT			de;
	short			   err;
	short			   i;

	if (hunt)		// only do this when requested (usually just at first mount)
		{
		//path[0] = dfname[0] = BIRS.HDDOSDRV[0];//C
		path[0] = dfname[0] = 'C'; //C: drive
		
		if ((err = DIRFindFirst(path, &de)) != dsdEndOfDir)
			{
			do	{
				if (err != 0)
					break;
				if (de.d_name[9] == 'L' && de.d_name[10] == 'O' && de.d_name[11] == 'G')
					{
					for (i = val = 0; i < 8; i++)
						if (de.d_name[i] >= '0' && de.d_name[i] <= '9')
							val = (val * 10) + de.d_name[i] - '0';
						else
							break;
					if (i == 8) 	// all digits
						if (val > maxval)
							maxval = val;
					}
				} while (DIRFindNext(&de) != dsdEndOfDir);
			}
		if (maxval < 0 && counter < 0)	// 2003-08-27
			counter = 0;	// 2003-08-21
		//if (maxval > counter)	// 99/07/22
		else if (maxval > counter)	// 2003-08-21
			counter = maxval + 1;
		}
   //A new file name
	sprintf(&dfname[2], "%08lu.LOG", counter);
	DBG(printf("File created %s\n", dfname);putflush();CIOdrain();)
	*fcounter=counter;
	
	if (incIndex)
		counter++;
	
	//nfile=(ushort) counter;
	FileNumber=(ushort) counter;
	
	return dfname;

	}	//____ GetNextRECFileName() ____//
/******************************************************************************\
**	BIRNextFileName		
** Also use renam(const char* old, const char* newn); to rename files once done 
** uploading 
\******************************************************************************/
static char *BIRNextFileName(bool hunt, bool incIndex)
	{
//	ulong			val, maxval = 0;
//	static ulong	counter = 0;
	static long		counter = -1;		// 2003-08-21
	long			val, maxval = -1;	// 2003-08-21
	static char		dfname[] = "x:00000000.DAT";
	static char		path[] = "x:";
	DIRENT			de;
	short			err;
	short			i;

	if (hunt)		// only do this when requested (usually just at first mount)
		{
	//	path[0] = dfname[0] = BIRS.HDDOSDRV[0];
		if ((err = DIRFindFirst(path, &de)) != dsdEndOfDir)
			{
			do	{
				if (err != 0)
					break;
				if (de.d_name[9] == 'D' && de.d_name[10] == 'A' && de.d_name[11] == 'T')
					{
					for (i = val = 0; i < 8; i++)
						if (de.d_name[i] >= '0' && de.d_name[i] <= '9')
							val = (val * 10) + de.d_name[i] - '0';
						else
							break;
					if (i == 8) 	// all digits
						if (val > maxval)
							maxval = val;
					}
				} while (DIRFindNext(&de) != dsdEndOfDir);
			}
		if (maxval < 0 && counter < 0)	// 2003-08-27
			counter = 0;	// 2003-08-21
		//if (maxval > counter)	// 99/07/22
		else if (maxval > counter)	// 2003-08-21
			counter = maxval + 1;
		}

	sprintf(&dfname[2], "%08lu.DAT", counter);
	
	if (incIndex)
		counter++;
	
	return dfname;

	}	//____ BIRNextFileName() ____//
