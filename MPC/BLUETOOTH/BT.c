#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include	<dosdrive.h>	// PicoDOS DOS Drive and Directory Definitions
#include <errno.h>
#include	<dirent.h>		// PicoDOS POSIX-like Directory Access Defines
#include	<dosdrive.h>	// PicoDOS DOS Drive and Directory Definitions
#include	<fcntl.h>		// PicoDOS POSIX-like File Access Definitions
#include	<stat.h>		   // PicoDOS POSIX-like File Status Definitions
#include	<termios.h>		// PicoDOS POSIX-like Terminal I/O Definitions
#include	<unistd.h>		// PicoDOS POSIX-like UNIX Function Definitions

#include	<cfxbios.h>		// Persistor BIOS and I/O Definitions
#include	<cfxpico.h>		// Persistor PicoDOS Definitions
#include <ADS.h>
#include <MPC_Global.h>
#include <Settings.h>
#include <PLATFORM.h>
#include <BT.h>
#ifdef WISPR
#include <WISPR.h>
#endif
#ifdef IRIDIUM
#include <GPSIRID.h>
#endif



extern Settings SYSSettings[];
#ifdef WISPR
extern Settings WISPSettings[];
#endif
#ifdef IRIDIUM
extern Settings IRIDSettings[];
#endif
#ifdef ACOUSTICMODEM
extern Settings AMDMSettings[];
#endif

extern Settings BTSettings[];

BluetoothParameters BT;

static const struct bt_com
	{
	char*	command;
	ushort	retval;
	char* execute;
	} bt_com[]  = {
	   "run", 1,   "Starting Program",
	   //"RUN", 1,   "Starting Program",
	   "set", 2,   "List Settings",
	   //"SET", 2,   "List Settings",
	   "btoff", 3, "BlueTooth Power Off",
	   
	/*   "cap", 4,   "Capture File",
	   "CAP", 4,   "Capture File",*/
	   
	   "res", 6,   "Restart Program",
	   //"RES", 6,   "Restart Program",
/*	   "type", 7,   "Type File",
	   "TYPE", 7,   "Type File",
	   "ren", 8,   "Rename File",
	   "REN", 8,   "Rename File",*/
	   "help", 9,  "Help",
	   //"Help", 9,  "Help",
	   
	//   "dir", 10,  "Directory",
	//   "DIR", 10,  "Directory",
	   "esc", 11,  "Escape",
	  // "ESC", 11,  "Escape",
	   "hibernate", 12, "Hibernation Mode",
	//   "HIBERNATE", 12, "Hibernation Mode",
	   0,  0,   0};

static struct varnames
{  
   char* name;
   } varnames[] = {
      "a","b", "c", "d","e","f","g","h","i","j","k","l","m",
      "n","o","p","q","r","s","t","u","v","w","x","y","z","\0"
      };
	   
	   

int Bluetooth_Execute(ushort);
void Bluetooth_PrintSettings(Settings*);
void Bluetooth_Settings();
int Bluetooth_Console(char);
static void IRQ2_ISR();
static void IRQ4_ISR();
static void IRQ5_ISR();
int memcasecmp(const char*, const char*, size_t);
void BTPrint(char*, ...);
//IRIDUM TUPORT Setup
TUPort *BTPort;

short BT_RX, BT_TX;
bool BT_On;
static int variablenumber;
bool SettingsMode;

bool FoundSetting;




/**********************************************************************\
** Bluetooth_Power()
\**********************************************************************/
void Bluetooth_Power(bool on){


   SettingsMode=false;

   FoundSetting=false;
   if(on){
      BT_On=true;
      #ifdef RAOT
         PIOSet(BTPWR2);
         RTCDelayMicroSeconds(250000L);
      #endif
   
   	BT_RX = TPUChanFromPin(BTRX);
   	BT_TX = TPUChanFromPin(BTTX);
   	//Power ON
   	PIOSet(BTCOM);
   	PIOSet(BTPWR);

   	BTPort = TUOpen(BT_RX, BT_TX, BTBAUD, 0);
   	if(BTPort == 0) 
   		flogf("\nERROR|BT TUPort failed to open\n");

      }
   //Power OFF   
   else if(!on){
      BT_On=false;
   	PIOClear(BTCOM);
   	PIOClear(BTPWR);
   	TUClose(BTPort);
   	}
   	
   flogf("\n\t|Bluetooth_Power: %s\n", on ? "on" : "off");	
	


}        //____ Bluetooth_Power() ____//
/**********************************************************************\
** Bluetooth_Interface()
\**********************************************************************/
void Bluetooth_Interface(){

   int val=0;
   int idle=0;
   BT_On=true;
   SettingsMode=false;

   FoundSetting=false;

        
   Bluetooth_Sleep(); 
   
   BTPrint("\nWelcome to RAOS Bluetooth Interface");

   
   while(BT_On&&idle<2){

      Bluetooth_Sleep();
      
      if(tgetq(BTPort)){
         val = Bluetooth_Data();
         if(val>=-1) val =Bluetooth_Execute(val); 
         if(val==1) break;
         if(val==12&&!SystemOn) flogf("\nEntering Hibernation State");
         idle=0;
         }
         
      else if(cgetq())
         if(cgetc() == '1') 
            break;
     
      else if(AD_Check()){
         idle++;  
         }
         
      System_Timer();   
      }
      
      
      
      BTPrint("\nBT Inteface exit");
      
      RTCDelayMicroSeconds(3000000L);
      
      
      if(!BT_On&&BT.ON==0)
         Bluetooth_Power(false);
         

}        //____ Bluetooth_Interface() ____//
/**********************************************************************\
** Bluetooth_Test()
\**********************************************************************/
void Bluetooth_Test(){
   
   Bluetooth_Power(true);
   
   Bluetooth_Interface();
   
   Bluetooth_Power(false);
   
}        //____ Bluetooth_Test() ____//
/**********************************************************************\
** Bluetooth_Data()
\**********************************************************************/
int Bluetooth_Data(){

	short	i;
   char *instring;
   char inchar;
   short index;
   int retval=0;
   int x;
   static int SettingIndex;

   instring=(char*)calloc(51,1);

   if(tgetq(BTPort)<1) return -1;
   

	for(i=0;i<50;i++){  
	   inchar = TURxGetByteWithTimeout(BTPort, 50);
		if(inchar==-1||inchar==10) 
		   break;
		else
		   instring[i]=inchar;
		}
		
		i++;
		instring[i]='\0';
		
		
		flogf("\nInstring: %s", instring);
   
   if(FoundSetting){
      BTPrint("\nIs %s the new value you want to use? \'y\' or \'n\'", instring);
      
      Bluetooth_Sleep();
      
      inchar = tgetc(BTPort);
      TUTxFlush(BTPort);
      TURxFlush(BTPort);
      if(inchar=='y'){
         BTPrint("\nSaving %s to %s", instring, varnames[SettingIndex].name);
         execstr("set %s=%s", varnames[SettingIndex].name, instring);
         FoundSetting=false;
         }
      else if(inchar=='n'){
         BTPrint("\nNot Saving %s to %s", instring, varnames[SettingIndex].name);
         FoundSetting=false;
         }
      else{
         BTPrint("\nBad Entry. Choose parameter again");
         FoundSetting=false;
         }
      return -2;  
      }
   
   else if(SettingsMode){
      if(memcasecmp("esc", instring, 3)==0)
         return 11;
   
      x=atoi(instring);     
      for(index=0; index<variablenumber; index++){
         
         
         if(memcasecmp(varnames[index].name,instring, strlen(varnames[index].name))==0){
            flogf("\nMatch! %s and  %s", varnames[index].name, instring);
            SettingIndex=index;
            FoundSetting=true;
            }
            
         else if(x-1==index&&x!=0){
            SettingIndex=index;
            
            flogf("x=%d, index=%d", x, index);
            FoundSetting=true;
            }
            
         if(FoundSetting){
            BTPrint("\nWhat new value would you like for %s?", varnames[index].name);
            
            Bluetooth_Sleep();
            
            Bluetooth_Data();
            return 0;
            }

         }
         if(!FoundSetting)
            BTPrint("\n%s is not a valid setting to modify.\nPlease try again, or type \"esc\"", instring);
         return -2;
      }
   SettingIndex=-1;
   for(index=0; bt_com[index].command; index++){
      
      if(memcasecmp(bt_com[index].command, instring, strlen(bt_com[index].command))==0){
         retval = index;
         SettingIndex=retval;
         break;
         
         }
      else retval= -1;
      }
   if(SettingIndex==-1){
      BTPrint("\n%s is not a valid command.\n", instring);
      for(index=0; bt_com[index].retval; index++)
         if(bt_com[index].retval==9){
            retval = index;
            break;
            }
      }
   
   
   return retval;


}        //____ Bluetooth_Data() ____//
/**********************************************************************\
** Bluetooth_Execute
\**********************************************************************/
int Bluetooth_Execute(ushort value){
   int retval=0;
   int index=0;
   char inchar;
   
   if(value>=0){
      BTPrint("\nBluetooth_Execute: %s", bt_com[value].execute);
   
      switch(bt_com[value].retval){
         case 1:
            retval = 1; 
            break;
            
         //Print Settings, then modify as needed. Escape (11) and Set (2) to reprint any changes.   
         case 2:
            Bluetooth_Settings(); 
            break;
         
         //btoff   
         case 3:
            retval=3;
            BT_On=false;
            BT.ON=0;
            break;
         
         case 6:
            RTCDelayMicroSeconds(200000L); 
            BIOSReset();
            break;
         
         //Print Help Commands
         case 9:
            BTPrint("\nBluetooth Help Commands:");

            while(bt_com[index].command){

               BTPrint("\n%-05s: %s", bt_com[index].command, bt_com[index].execute);
               
               RTCDelayMicroSeconds(25000L);
               index++;
               }
                 
         
         //Backup out of Settings Mode
         case 11:
            retval =11; 
            break;
         
         //Hibernation Mode
         case 12:
            BTPrint("\nAre you sure you want to hibernate?");

            Bluetooth_Sleep();
            inchar = tgetc(BTPort);
            
            if(inchar=='y'){
               BTPrint("\nHibernation activated");
               InduceHibernation();
               //SystemOn=false;
               //DataAlarm=DataAlarm ? false : true;
               }
            else if(inchar=='n'){
               BTPrint("\nHibernation dettered");
               return 1;
               }
               
            return 12;  
         }
      }
   else
      BTPrint("Invalid Command. Try \"help\" for list of commands");
            

   
   return retval;

}        //____ Bluetooth_Execute() ____//
/**********************************************************************\
** Bluetooth_Settings()
\**********************************************************************/
void Bluetooth_Settings(char*){
   int retval=0;
   int i=0;
   
   variablenumber=0;
   SettingsMode=true;
   BTPrint("\nVariable=Value");
   RTCDelayMicroSeconds(100000L);
   
   Bluetooth_PrintSettings(SYSSettings);
   #ifdef IRIDIUM
      Bluetooth_PrintSettings(IRIDSettings);
   #endif
   #ifdef WISPR
      Bluetooth_PrintSettings(WISPSettings);
   #endif
   #ifdef SEAGLIDER
      Bluetooth_PrintSettings(SEAGSettings);
   #endif
   #ifdef ACOUSTICMODEM
      Bluetooth_PrintSettings(AMDMSettings);
   #endif
   

   BTPrint("\nWhich Variable would you like to modify?");
   
   
   while(retval!=11){

      Bluetooth_Sleep();
      
      AD_Check();
      
      retval= Bluetooth_Data();
   }
   SettingsMode=false;

   BTPrint("\nExited Settings Mode");



}        //____ Bluetooth_Settings() ____//
/**********************************************************************\
** Bluetooth_PrintSettings()
\**********************************************************************/
void Bluetooth_PrintSettings(Settings *VEESettings){

   Settings *setp = VEESettings;
   VEEData vdp;
   
   while(setp->optName){
      vdp=VEEFetchData(setp->optName);

		setp->optCurrent = vdp.str;
		BTPrint("\n%d)%s=%s", variablenumber+1, setp->optName, setp->optCurrent);
				
		RTCDelayMicroSeconds(1000L);


      varnames[variablenumber].name=setp->optName;
      variablenumber++;		
      setp++;
      if(setp->optName==NULL) break;
		}
      

   

}        //____ Vluetooth_PrintSettings() ____//
/**********************************************************************\
** Bluetooth_Sleep()
\**********************************************************************/
void Bluetooth_Sleep(){

	
	RTCDelayMicroSeconds(5000L);
	#ifdef RAOB
	IEVInsertAsmFunct(IRQ2_ISR, level2InterruptAutovector);     
	IEVInsertAsmFunct(IRQ2_ISR, spuriousInterrupt);
	#endif
	#ifdef RAOT
	IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector);
	IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);	
	#endif
   IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector);     //Console Interrupt
   IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);

	
	//CTMRun(false);
	SCITxWaitCompletion();
	EIAForceOff(true);
	//QSMStop();
	CFEnable(false);
		  
	TickleSWSR();	// another reprieve
#ifdef RAOB
   PinBus(IRQ2);                    //AModem Interrupt
   while((PinTestIsItBus(IRQ2))==0)
      PinBus(IRQ2);
#endif
#ifdef RAOT
   PinBus(IRQ5);
   while((PinTestIsItBus(IRQ5))==0)
      PinBus(IRQ5);
#endif      


   PinBus(IRQ4RXD);                    // Console Interrupt
   while((PinTestIsItBus(IRQ4RXD))==0)
      PinBus(IRQ4RXD);

#ifdef RAOB
   //Watch Amodem Tuport, Console and AD Logging   
	while(PinRead(IRQ2)&&PinRead(IRQ4RXD)&&!data)  
		LPStopCSE(FastStop);		
#endif		
#ifdef RAOT
   while(PinRead(IRQ5)&&PinRead(IRQ4RXD)&&!data)
      LPStopCSE(FastStop);
#endif      
      
   
	EIAForceOff(false);			// turn on the RS232 driver
	CFEnable(true);				// turn on the CompactFlash card
   
#ifdef RAOB
   PIORead(IRQ2);
   while((PinTestIsItBus(IRQ2))!=0)
      PIORead(IRQ2);
#endif
#ifdef RAOT
   PIORead(IRQ5);
   while((PinTestIsItBus(IRQ5))!=0)
      PIORead(IRQ5);
#endif      

   PIORead(IRQ4RXD);
   while((PinTestIsItBus(IRQ4RXD))!=0)
      PIORead(IRQ4RXD);

	RTCDelayMicroSeconds(5000L);
	flogf(".");


}        //____ Bluetooth_Sleep() ____//
/*************************************************************************\
**  static void Irq3ISR(void)
\*************************************************************************/	
static void IRQ2_ISR(void)
{
PinIO(IRQ2);
RTE();
}	      //____ Irq3ISR ____//
/*************************************************************************\
**  static void IRQ4_ISR(void)
\*************************************************************************/	
static void IRQ4_ISR(void)
{
PinIO(IRQ4RXD);
RTE();
}	      //____ Irq4ISR() ____//
/*************************************************************************\
** static void IRQ5ISR()
\*************************************************************************/
static void IRQ5_ISR(void){
PinIO(IRQ5);
RTE();
}        //____ IRQ5ISR() ____//
/*************************************************************************\
** BTPrint()
\*************************************************************************/
void BTPrint(char *format, ...){

   va_list arg;
   char string[65];
   int i=0;
  
  
   va_start (arg, format);
   vsprintf(string, format, arg);
   
   flogf("%s", string);
   TUTxPrintf(BTPort, "%s", string);
   va_end (arg);
   
   TUTxWaitCompletion(BTPort);
   TUTxFlush(BTPort);
   TURxFlush(BTPort);
   
   
}        //____ BTPrint() ____//
/*************************************************************************\
** Bluetooth_LowPower()
**
** Return value:
** 1: Continue program
** 2: Reboot Program
** 3: Hibernate.. Can't wait up from BT
** 0: NULL
\*************************************************************************/
int Bluetooth_LowPower(){


   int val, ret=0;
/*
#ifdef WISPR
   //Shutdown all other devices (WISPR)
   if(WISPR_Status())
      WISPRSafeShutdown();
#endif
*/
   BTPrint("\nWelcome to RAOS Bluetooth Low Power");

   if(!BT_On) Bluetooth_Power(true);
   while(BT_LP){

      AD_Check();

      if(tgetq(BTPort)){
         val = Bluetooth_Data();
         if(val>=0){ Bluetooth_Execute(val); val=0;}         
         }
      else if(cgetq())
         val = Bluetooth_Console(cgetc());
         
      else{
         Bluetooth_Sleep();
         }
      
      if(val==1){
         ret= val;
         break;
         }
      else if(val==6){
         ret= 2;
         BIOSReset();
         }
      else if(val==12){
         ret= 3;
      //   SleepUntilWoken();
         
         }
      
   }

   return ret;   

}        //____ Bluetooth_LowPower() ____//
/*************************************************************************\
** Bluetooth_Console();
\*************************************************************************/
int Bluetooth_Console(char c){

   switch(c){
      case '1':
         return 12;

      case 'H': case 'h':
         cprintf("Bluetooth_Console Help:\n1: Hibernate\nR: Restart\nC: Continue\n");
         break;
      case 'R': case 'r':
         return 6;

      case 'C': case 'c':
         return 1;
                  
      }
   return 0;
}        //____ Bluetooth_Console() ____//
/*************************************************************************\
**
\*************************************************************************/
int memcasecmp(const char* str1, const char* str2, size_t len){

   int i;
   int ret=0;
   int length;
  // DBG(flogf("\nmemcasecmp: len: %d, intlen: %d", len, (int)len);)
   length= (int)len;
   
   for(i=0; i<length; i++){
      if(str2[i]<'a'){
         
         if((str2[i]+32)!=str1[i]) ret++;
         }
      else if(str2[i]>='a'){
        
         if((str2[i]!=str1[i])) ret++;
         }
   
   }
 //  flogf("ret: %d", ret);
   return ret;

}        //____ memcasecmp() ____//
/*************************************************************************\
** GetBTSettings
\*************************************************************************/
void GetBTSettings(){

   char* p;
   

   //"B" 
   p = VEEFetchData(BLUETOOTH_NAME).str;
   BT.ON = atoi( p ? p : BLUETOOTH_DEFAULT);
   DBG( uprintf("BT.ON=%u (%s)\n", BT.ON, p ? "vee" : "def"); cdrain();)
   if(BT.ON>=1) BT.ON=1;
   else BT.ON=0;
	
}        //____ GetBTSettings() ____//
/**
**
*/
void Bluetooth_LPWait(){

   BT_LP=true;

}