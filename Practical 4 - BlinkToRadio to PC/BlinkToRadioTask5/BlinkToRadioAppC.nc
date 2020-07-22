#include <Timer.h> 
#include "BlinkToRadio.h" 

configuration BlinkToRadioAppC { 
} 
implementation { 
	components MainC; 
	components LedsC; 
	components BlinkToRadioC as App; 
	components new TimerMilliC() as Timer0;
	components ActiveMessageC; 
	components new AMSenderC(AM_BLINKTORADIO);
	components new AMReceiverC(AM_BLINKTORADIO);
	components SerialActiveMessageC; //new
	components new SerialAMSenderC(AM_BLINKTORADIO); //new
	
	App.SerialControl -> SerialActiveMessageC; //new
	App.SerialSend -> SerialAMSenderC; //new
	App.Boot -> MainC; 
	App.Leds -> LedsC; 
	App.Timer0 -> Timer0; 
     	App.Packet -> AMSenderC; 
     	App.AMPacket -> AMSenderC; 
     	App.AMSend -> AMSenderC; 
     	App.AMControl -> ActiveMessageC;
	App.Receive -> AMReceiverC; 
} 