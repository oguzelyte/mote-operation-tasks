 
 /*
	@edited by Olivija Guzelyte (160421859)
	@version 11/05/2018
	
 */
 
 #include <Timer.h>
 #include "BlinkToRadio.h"
 
configuration BlinkToRadioAppC {}

implementation {
  components BlinkToRadioC;

  components MainC;
  components LedsC;
  components AMSendReceiveC as Radio;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer_ack; // step 7. introduce acknowledgement timer to resend data if acknowledgement isn't received

  BlinkToRadioC.Boot -> MainC;
  BlinkToRadioC.RadioControl -> Radio;

  BlinkToRadioC.Leds -> LedsC;
  BlinkToRadioC.Timer0 -> Timer0;
  BlinkToRadioC.Timer_ack -> Timer_ack; // step 7. wire the interface to the component that provides it

  BlinkToRadioC.Packet -> Radio;
  BlinkToRadioC.AMPacket -> Radio;
  BlinkToRadioC.AMSendReceiveI -> Radio;
}
