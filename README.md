# Mote Operation Tasks
 Mote operating tasks using TinyOS - operating system specifically designed for embedded networks.

# Stop and Wait Protocol PAR (Positive Acknowledgement Retransmission) - Final Coursework

This gave me the opportunity to develop my skills in developing programs for wireless
network communication using the nesC programming language. Specifically, I was required to demonstrate
my understanding and implement the stop and wait PAR protocol.

I have been provided with a new set of files for the BlinkToRadio application. The files include a configuration called AMSendReceiveC, which implements a new interface called AMSendReceiveI using implementation AMSendReceiveP. This interface provides a send command and receive event to allow radio communication. The semantics of send and receive are described in AMSendReceiveP. The BlinkToRadioC implementation gives an example of how the send command and receive event are used to
duplicate the functionality of the last tutorial on Mote to PC communication.

# Part 1 - Final Coursework


**Protocol PAR in the absence of errors.**


1. In the BlinkToRadio.h, make sure you set AM_BLINKTORADIO to 6 so that Echo does not
drop messages.
2. Ignore the message sequence number field for now.
3. Modify event AMSendReceiveI.receive (as Receiver host) to send an acknowledge
message each time it receives a data message.
4. Modify receive (as Sender host) to inform fired when an acknowledge message arrives.
5. Modify event Timer0.fired to wait for the acknowledge message before sending the next data
message. (hint: use a boolean variable to communicate between receive and fired)
6. Test your application using the java Listen program to verify that data and
acknowledge messages are sent as expected.
7. Temporarily comment out the sending of acknowledge messages and ensure that
Timer0.fired is blocked.
8. Restore the sending of acknowledge messages.
9. Modify fired to set the sequence number field alternately to 0/1 for each sent data message
(see lecture notes on the PAR protocol).
10. Modify receive (as Receiver host) to set the sequence number field of the ack message
appropriately.
11. Modify receive (as Sender host) to use that sequence number when informing fired.
12. Test your application. Listen should show correct field values. Take a screenshot as you will need it
when writing your report


**Protocol PAR in the presence of errors.**


1. Make sure you set AM_BLINKTORADIO to 99 so that Echo drops messages.
2. Test your application by using java Listen.
3. It should deadlock after the first dropped message.
4. If it does not, then it is not correct.
5. DO NOT proceed any further until you have fixed the problem.
6. Modify fired to save each data message.
7. Modify fired to resend the saved message if an acknowledge message is not
received within t milliseconds of being sent. (Hint: you already know how to
start Timer0 in periodic mode, you may create another timer and start in singleshot mode)
8. Experiment with the value of t.
9. Test your application. Listen should show the resending of data after a timeout.
Take a screenshot as you will need it when writing your report


# Part 2


**PAR at speed in the absence of errors.**


1. Make sure you set AM_BLINKTORADIO to 6 so that Echo does not drop
messages.
2. To date, your application has sent 1 data message per second so it is unlikely that it
will still be waiting for an acknowledge message when it wishes to send the next
data message.
3. You will need to modify your application to send the first 60 data messages as fast
as it can. However, you cannot use the event fired to do this as events all run at the
same priority level and your application will never receive any messages (Hint: you
may want to create a task instead).
4. Post your newly created task in the right places inside the code to make sure that
the program works as expected.
5. Test your application by using java Listen. Listen should show that your mote
sends the first 60 sequences of 4 messages very fast (data to echo, data from echo,
ack to echo, ack from echo).


**Protocol PAR at speed in the presence of errors.**


1. Make sure you set AM_BLINKTORADIO to 99 so that Echo drops messages.
2. Test your application by using java Listen and inspect the output.
