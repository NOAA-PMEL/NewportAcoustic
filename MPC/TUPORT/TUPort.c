#include	<cfxbios.h>		// Persistor BIOS and I/O Definitions
#include	<string.h>
#include <TUPort.h>
#include <MPC_Global.h>
#ifdef WISPR
#include <WISPR.h>
#endif
#include <PLATFORM.h>



//AMODEM TUPORT Setup
TUPort *AModemPort;    
short AModem_RX, AModem_TX;

//CTD TUPORT Setup
TUPort *CTDPort; 
short CTD_RX, CTD_TX;
 
//IRIDUM TUPORT Setup
TUPort *GPSIRIDPort;
short GPSIRID_RX, GPSIRID_TX;

//NIGK TUPORT Setup
TUPort *NIGKPort;
//Uses the same RX & TX as AModem

//PAM TUPORT Setup
TUPort *PAMPort;
short PAM_RX, PAM_TX;

//Seaglider TUPORT Setup
TUPort *SGPort;
//Uses same RX & Tx as CTD

/**************************************************************\
** void TUPortSettings()
\**************************************************************/
void OpenTUPort(char* Port, bool on){

   int WisprNum=1;
   
   flogf("\n%s|%s TUPort %s",Time(NULL), Port, on ? "Open" : "Close");
   RTCDelayMicroSeconds(20000L);
   
   
   //Acoustic Modem TUPort Settings
   if(strncmp("AMODEM", Port, 6)==0){
   	
   	#ifdef ACOUSTICMODEM
   	if(on){
      	AModem_RX = TPUChanFromPin(AMODEMRX);
      	AModem_TX = TPUChanFromPin(AMODEMTX);

   	   PIOClear(AMODEMPWR);
   	   RTCDelayMicroSeconds(250000L);
   	   PIORead(48);
      	PIOSet(AMODEMPWR);
         AModemPort = TUOpen(AModem_RX, AModem_TX, AMODEMBAUD, 0);
      	if(AModemPort == 0) 
            flogf("\n\t|Bad %s port\n", Port);
      	else{          
      	   TUTxFlush(AModemPort);
            TURxFlush(AModemPort); 
            Delay_AD_Log(5); //Wait 5 seconds for AModemPort to power up
            
            }
   	   }
   	if(!on){
   	   Delay_AD_Log(5);
      	PIOClear(AMODEMPWR);
      	TUClose(AModemPort);
   	   }
   	return;
   	
   	#else
   	   flogf("\n\t|DEVICE NOT DEFINED: %s", Port);
   	#endif
      }
   
   //CTDPort TUPort Settings
   if((strncmp("CTD", Port, 3)==0)||(strncmp("SG", Port, 2)==0)){

      #ifdef CTDSENSOR      
   	if(on){
      	CTD_RX = TPUChanFromPin(32);
      	CTD_TX = TPUChanFromPin(31);
   	   //CTDreceive = calloc(200,sizeof(char));
         //CTDdata = calloc(200, sizeof(char));
         PIOSet(22);     
      	PIOClear(23);   
         CTDPort = TUOpen(CTD_RX, CTD_TX, BAUD, 0);
      	if(CTDPort == 0)  flogf("\nBad TU Channel: CTDPort...");
   	   }
   	if(!on){
   	   PIOClear(22);
   	   TUClose(CTDPort);
   	   }
   	return;
      #endif
      #ifdef SEAGLIDER
   	if(on){
      	CTD_RX = TPUChanFromPin(32);
      	CTD_TX = TPUChanFromPin(31);
   	   PIOSet(22);
   	   PIOClear(23);
   	   SGPort = TUOpen(CTD_RX, CTD_TX, SGBAUD, 0);
   	   if(SGPort == 0) flogf("\nBad TU Channel: SGPort...");
   	   }
   	if(!on){
   	   PIOClear(22);
   	   TUClose(SGPort);
   	   }
   	return;
   	
   	#endif
   	   flogf("\n\t|DEVICE NOT DEFINED: %s", Port);
   	
	}	
	
#ifdef WISPR
   //PAM TUPort Settings:
   else if(strncmp("PAM", Port, 3)==0){
   
   //  WisprNum=1;  //MULTIWISPR is not yet implemented. 
      WisprNum = GetWISPRNumber();
      flogf(": WISPR%d", WisprNum);
      if(on){
         PAM_RX= TPUChanFromPin(28);
         PAM_TX= TPUChanFromPin(27);
         PAMPort = TUOpen(PAM_RX, PAM_TX, BAUD, 0);
         }
      else if(!on){
         TUTxFlush(PAMPort);
         TURxFlush(PAMPort);
         TUClose(PAMPort);
         }
         PIOClear(WISPRONE);
         PIOClear(WISPRTWO);
         PIOClear(WISPRTHREE);
         PIOClear(WISPRFOUR);
    //  PIOClear(WISPR_PWR_ON); PIOClear(WISPR_PWR_OFF);       
     /* 
      //PAM 1 
      if(WisprNum==1){
         if(on){
            PIOSet(WISPRONE);
            PIOClear(WISPRTWO);      
            }
         else{
            PIOClear(WISPRONE);
            PIOClear(WISPRTWO);
            }
         }
      //PAM 2
      else if(WisprNum==2){
         if(on){
            PIOSet(WISPRONE);
            PIOSet(WISPRTWO);
            }
         else{
            PIOClear(WISPRONE);
            PIOClear(WISPRTWO);
            }
         }
      //PAM 3
      else if(WisprNum==3){
         if(on){
            PIOSet(WISPRTHREE);
            PIOClear(WISPRFOUR);
            }
         else{
            PIOClear(WISPRTHREE);
            PIOClear(WISPRFOUR);
            }
            
         }
      //PAM 4
      else if(WisprNum==4){
         if(on){
            PIOSet(WISPRTHREE);
            PIOSet(WISPRFOUR);
            } 
         else{
            PIOClear(WISPRTHREE);
            PIOClear(WISPRFOUR);
            }
         }
      else if(WisprNum==0){
         flogf("\n\t|WISPR Zero. Run out of space?");
         }
         
      //Bad PAM
      else{
         if(PAMPort == 0) printf("\nBad TU Channel: PAM...");
         flogf("\n\t|Wrong PAM Port..."); 
         TUClose(PAMPort);
         }*/
      RTCDelayMicroSeconds(100000L);
      }
      
#endif
      
	//GPS/IRID TUPort Settings
	else if(strncmp("IRIDGPS", Port, 7)==0){
	   #ifdef IRIDIUM
	   if(on){
			GPSIRID_RX = TPUChanFromPin(IRDGPSRX);
			GPSIRID_TX = TPUChanFromPin(IRDGPSTX);
			//Power ON
			PIOSet(IRDGPSCOM);
			PIOSet(IRDGPSPWR);
			GPSIRIDPort = TUOpen(GPSIRID_RX, GPSIRID_TX, IRIDBAUD, 0);
			if(GPSIRIDPort == 0) 
				flogf("\n\t|Bad %s port\n", Port);
         }
      //Power OFF   
	   if(!on){
			PIOClear(IRDGPSCOM);
			PIOClear(IRDGPSPWR);
			TUClose(GPSIRIDPort);
			}
			
	   return;
	   #else
	      flogf("\n\t|DEVICE NOT DEFINED: %s", Port);
	   #endif
		}	  
		
   else if(strncmp("NIGK", Port, 4)==0){
      #ifdef WINCH
		if(on){
			AModem_RX = TPUChanFromPin(AMODEMRX);
			AModem_TX = TPUChanFromPin(AMODEMTX);

 		    PIOClear(AMODEMPWR);
		    RTCDelayMicroSeconds(250000L);
		    PIORead(48);	
			PIOSet(AMODEMPWR);	//Powers up the DC-DC for the Acoustic Modem Port
			 NIGKPort = TUOpen(AModem_RX, AModem_TX, AMODEMBAUD, 0);
			if(NIGKPort == 0) 
				flogf("\n\t|Bad %s port\n", Port);
			else{          
			   TUTxFlush(NIGKPort);
				TURxFlush(NIGKPort); 
				RTCDelayMicroSeconds(5000000L); //Wait 5 seconds for AModemPort to power up
				}
			}
		else{
			RTCDelayMicroSeconds(5000000L);
			PIOClear(AMODEMPWR);
			TUClose(NIGKPort);
		   }
		return;
		
		#endif
		}

}        //_____ OpenTUPort() ______//


