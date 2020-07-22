#include <Timer.h>
#include "BlinkToRadio.h"

module BlinkToRadioC {
 uses interface Boot;
 uses interface Leds;
 uses interface Timer<TMilli> as Timer0;
 uses interface Packet; 
 uses interface AMPacket; 
 uses interface AMSend; 
 uses interface SplitControl as AMControl;
 uses interface Receive;
 uses interface SplitControl as SerialControl; //new
 uses interface AMSend as SerialSend; //new
}

implementation {
 bool busy = FALSE; 
 message_t pkt; 
 uint16_t counter = 0;

event void Boot.booted() {
 call AMControl.start();
 call SerialControl.start(); //new
}

event void SerialControl.startDone(error_t err) {
 if (err == SUCCESS) {
 } else {
  call SerialControl.start();
 }
}

event void SerialControl.stopDone(error_t err) {}

event void SerialSend.sendDone(message_t* msg, error_t error) {}

event void AMSend.sendDone(message_t* msg, error_t error) { 
 if (&pkt == msg) { 
 busy = FALSE; 
 } 
} 

event void AMControl.startDone(error_t err) { 
 if (err == SUCCESS) { 
  call Timer0.startPeriodic(TIMER_PERIOD_MILLI); 
  } 
 else { 
  call AMControl.start(); 
 } 
}
 
event void AMControl.stopDone(error_t err) { 
}

event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) { 
 if ((len == sizeof(BlinkToRadioMsg)) && (call       	AMPacket.destination(msg) == TOS_NODE_ID)) { 
	BlinkToRadioMsg* btrpkt = (BlinkToRadioMsg*)payload;
	call SerialSend.send(TOS_NODE_ID, msg, sizeof(BlinkToRadioMsg)); // new 
	call Leds.set(btrpkt->counter*2); 
 } 
return msg; 
} 

event void Timer0.fired() {
 counter++;

 if (!busy) { 
  BlinkToRadioMsg* btrpkt = (BlinkToRadioMsg*)(call      
  Packet.getPayload(&pkt, sizeof (BlinkToRadioMsg))); 
  btrpkt -> nodeid = TOS_NODE_ID; 
  btrpkt -> counter = counter; 
  if (call AMSend.send(2, &pkt, sizeof(BlinkToRadioMsg)) == SUCCESS){ 
   busy = TRUE; 
  } 
 } 
}
}