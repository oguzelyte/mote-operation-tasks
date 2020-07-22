 /*
	@edited by Olivija Guzelyte (160421859)
	@version 11/05/2018
	
 */
 
 #include <Timer.h>
 #include "BlinkToRadio.h"
 
module BlinkToRadioC {
  uses {
    interface Boot;
    interface SplitControl as RadioControl;

    interface Leds;
    interface Timer<TMilli> as Timer0;
	interface Timer<TMilli> as Timer_ack; // step 7. extra timer to resend message if ack is not received

    interface Packet;
    interface AMPacket;
    interface AMSendReceiveI;
  }
}

implementation {
  bool busy = FALSE; // Part 1. Protocol PAR in the absence of errors. step 4, 5. busy flag to prevent sending messages when waiting for Echo to respond
  uint32_t i = 0; // Part 2. Step 1, 2, 3, 4. global variable to send 60 messages with a send60msgs() task
  uint16_t counter = 0; 
  uint32_t t = 2000; // step 7.
  message_t sendMsgBuf;
  message_t ackMsgBuf; // Part 1. Protocol PAR in the absence of errors. step 3. created to send ack message
  message_t sendMsgBufCopy; // Part 1 Protocol PAR in the presence of errors. step 6.  created for sendMsg copy
  message_t* sendMsg = &sendMsgBuf; // initially points to sendMsgBuf 
  message_t* ackMsg = &ackMsgBuf; // Part 1. Protocol PAR in the absence of errors. step 3. initially points to ackMsgBuf
  message_t* sendMsg_copy = &sendMsgBufCopy; // Part 1 Protocol PAR in the presence of errors. step 6. 
  void deepCopySendMsg(); // create deep copy of sendMsg 
  
  
  // Part 2. Step 1, 2, 3, 4. task to send 60 messages
  task void send60msgs() {
	// this code mimics Timer0.fired() code
    BlinkToRadioMsg* btrpkt;
	call AMPacket.setType(sendMsg, AM_BLINKTORADIO);
	call AMPacket.setDestination(sendMsg, DEST_ECHO);
	call AMPacket.setSource(sendMsg, TOS_NODE_ID);
	call Packet.setPayloadLength(sendMsg, sizeof(BlinkToRadioMsg));

	btrpkt = (BlinkToRadioMsg*)(call Packet.getPayload(sendMsg, sizeof (BlinkToRadioMsg)));
	counter++;
	btrpkt->type = TYPE_DATA; //00 55 IS DATA
	btrpkt->seq = counter%2;
	btrpkt->nodeid = TOS_NODE_ID;
	btrpkt->counter = counter;
	call Leds.set(btrpkt->counter);
	
	busy = TRUE; // set busy to true so sending doesn't happen while it's waiting for the Echo mote to echo data message back
	deepCopySendMsg(); //Part 2. Step 1, 2, 3, 4. do a deep copy of send message
	i++; // increment counter when 1 message is sent
	sendMsg = call AMSendReceiveI.send(sendMsg); // send the message		
	call Timer_ack.startOneShot(t); // step 7. start oneshot timer
   }
 
  
  event void Boot.booted() {
    call RadioControl.start();
  };

  event void RadioControl.startDone(error_t error) {
	if (error == SUCCESS) {		
		post send60msgs(); // Part 2. Step 1, 2, 3, 4. post the task 
    }
  };

  event void RadioControl.stopDone(error_t error){};


  event void Timer0.fired() {
	   
    if(!busy){ // Part 1. Protocol PAR in the absence of errors. step 4, 5. make sure this doesn't fire while it's waiting for response from Echo
		BlinkToRadioMsg* btrpkt;
		call AMPacket.setType(sendMsg, AM_BLINKTORADIO);
		call AMPacket.setDestination(sendMsg, DEST_ECHO);
		call AMPacket.setSource(sendMsg, TOS_NODE_ID);
		call Packet.setPayloadLength(sendMsg, sizeof(BlinkToRadioMsg));

		btrpkt = (BlinkToRadioMsg*)(call Packet.getPayload(sendMsg, sizeof (BlinkToRadioMsg)));
		counter++;
		btrpkt->type = TYPE_DATA; //00 55 IS DATA
		btrpkt->seq = counter%2; // Part 1. Protocol PAR in the absence of errors. step 9, 10, 11, 12.
		btrpkt->nodeid = TOS_NODE_ID;
		btrpkt->counter = counter;
		call Leds.set(btrpkt->counter);
	
		busy = TRUE; // Part 1. Protocol PAR in the absence of errors. step 4, 5. set busy flag to true so fired wouldn't fire
		deepCopySendMsg(); // Part 1 Protocol PAR in the presence of errors. step 6. deep copy of sendMsg	
		sendMsg = call AMSendReceiveI.send(sendMsg); //send the data msg
		call Timer_ack.startOneShot(t); // start oneShot timer
		
	}
  }
  
  // Part 2. Step 1, 2, 3, 4. does a deep copy of sendMsg
  void deepCopySendMsg(){
	  
	// copy all the fields of sendMsg and create an identical copy, because memcpy doesn't do a proper copy
	BlinkToRadioMsg* btrpkt;
	call AMPacket.setType(sendMsg_copy, AM_BLINKTORADIO);
	call AMPacket.setDestination(sendMsg_copy, DEST_ECHO);
	call AMPacket.setSource(sendMsg_copy, TOS_NODE_ID);
	call Packet.setPayloadLength(sendMsg_copy, sizeof(BlinkToRadioMsg));

	btrpkt = (BlinkToRadioMsg*)(call Packet.getPayload(sendMsg_copy, sizeof (BlinkToRadioMsg)));

	btrpkt->type = TYPE_DATA; //00 55 IS DATA
	btrpkt->seq = counter%2;
	btrpkt->nodeid = TOS_NODE_ID;
	btrpkt->counter = counter;
	  
  }

  // step 7. when the acknowledgement timer fires
  event void Timer_ack.fired() {
	 call AMSendReceiveI.send(sendMsg_copy); // resend the copied data message
	 call Timer_ack.startOneShot(t); // start the acknowledgement timer again
  }
 

  event message_t* AMSendReceiveI.receive(message_t* msg) {
    
	uint8_t len = call Packet.payloadLength(msg);

	BlinkToRadioMsg* btrpkt = (BlinkToRadioMsg*)(call Packet.getPayload(msg, len));
	BlinkToRadioMsg* ack_btrpkt; // Part 1. Protocol PAR in the absence of errors. step 3. create an acknowledgement message
	if(btrpkt->type == TYPE_DATA) { // Part 1. Protocol PAR in the absence of errors. step 3. is the message received type data
		
		// Part 1. Protocol PAR in the absence of errors. step 3. create an acknowledgement message
		call AMPacket.setType(ackMsg, AM_BLINKTORADIO);
		call AMPacket.setDestination(ackMsg, DEST_ECHO);
		call AMPacket.setSource(ackMsg, TOS_NODE_ID);
		call Packet.setPayloadLength(ackMsg, sizeof(BlinkToRadioMsg));
		ack_btrpkt = (BlinkToRadioMsg*)(call Packet.getPayload(ackMsg, sizeof (BlinkToRadioMsg)));

		ack_btrpkt->type = TYPE_ACK; // 00 CC is ack
		ack_btrpkt->seq = btrpkt->seq; // Part 1. Protocol PAR in the absence of errors. step 9, 10, 11, 12. sequence number must be the same of the received message's
		ack_btrpkt->nodeid = TOS_NODE_ID; //5D is my dest id
		ack_btrpkt->counter = btrpkt->counter; // Part 1. Protocol PAR in the absence of errors. step 9, 10, 11, 12. counter must be the same as of the received message's
		call Leds.set(ack_btrpkt->counter); // call leds to check whether the data msg is received
        // Part 1. Protocol PAR in the absence of errors. step 3. end of creation of acknowledgement message
		
		call AMSendReceiveI.send(ackMsg); // Part 1. Protocol PAR in the absence of errors. step 3. send ack message

	} else if ((btrpkt->type == TYPE_ACK) && (btrpkt->seq == counter%2)) { // Part 1. Protocol PAR in the absence of errors. step 4, 5. and step 9, 10, 11, 12. is it an ack message and is its sequence number the same as current data message's sequence number
		call Timer_ack.stop(); // step 7. stop the acknowledgement timer
		//free(sendMsg_copy);
		if(i<60) { // Part 2. Step 1, 2, 3, 4. if message sent less than 60 times, post the task again
			post send60msgs(); // Part 2. Step 1, 2, 3, 4.
		} else if (i == 60) { // Part 2. Step 1, 2, 3, 4.
			call Timer0.startPeriodic(TIMER_PERIOD_MILLI); // Part 2. Step 1, 2, 3, 4.
			i++; // Part 2. Step 1, 2, 3, 4. increment the i so the Timer0 wouldn't get started many times
		}
		busy = FALSE; //Part 1. Protocol PAR in the absence of errors. step 4, 5. set the busy flag to false so Timer0.fired can send a data message again		
	}

	return msg; // no need to make msg point to new buffer as msg is no longer needed
  }

}

 

