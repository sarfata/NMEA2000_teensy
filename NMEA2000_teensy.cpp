/*
  The MIT License

  Copyright (c) 2015-2017 Thomas Sarlandie thomas@sarlandie.net

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

/*
  Inherited NMEA2000 object for Teensy internal CAN
  based setup. See also NMEA2000 library.
  Requires FlexCAN library.
*/


#include "NMEA2000_teensy.h"
#include <FlexCAN.h>
#include <kinetis_flexcan.h>

//*****************************************************************************
tNMEA2000_teensy::tNMEA2000_teensy(uint16_t _DefTimeOut, uint8_t CANBusIndex) : tNMEA2000() {
#if defined(FlexCAN_MAILBOX_TX_BUFFER_SUPPORT)
  NumTxMailBoxes=3; // Use different buffers for high, mid and low priorities
#else
  NumTxMailBoxes=1; // Force all to be sent through one mail box.
#endif

#if NMEA2000_TEENSY_VER == 1
  CANbus = new FlexCAN(250000);
#else

#if NMEA2000_TEENSY_MAX_CAN_BUSSES == 2 && NMEA2000_TEENSY_VER>1
  if ( CANBusIndex>1 ) CANBusIndex=1;
  if ( CANBusIndex==0 ) {
    CANbus = &Can0;
  } else {
    CANbus = &Can1;
  }
#else
  (void)CANBusIndex; // just avoid warning
  CANbus = &Can0;
#endif

#endif
  DefTimeOut=_DefTimeOut;
}

extern void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);


//*****************************************************************************
bool tNMEA2000_teensy::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool fastpacket) {
  CAN_message_t out;
  out.id = id;
  out.len = len;
  out.ext = 1;
#if NMEA2000_TEENSY_VER == 1
  out.timeout = (wait_sent?DefTimeOut:0);
#endif
#if NMEA2000_TEENSY_VER == 2
  out.rtr=0;
#endif

  // Fill the frame buffer
  for (int i=0; i<len && i<8; i++) out.buf[i] = buf[i];

#if defined(FlexCAN_MAILBOX_TX_BUFFER_SUPPORT)
  uint8_t prio = (uint8_t) ((id >> 26) & 0x7);
  if ( !fastpacket && prio<=2 ) { // For single frame high priority messages, use first box
    return CANbus->write(out,CANbus->getFirstTxBox()) == 1;
  } else {
    // Send other in 2 priority boxes. pri 5-7 on last box
    return (prio<5?CANbus->write(out,CANbus->getFirstTxBox()+1):CANbus->write(out,CANbus->getFirstTxBox()+2)) == 1;
  }
#else
  return (CANbus->write(out) == 1);
#endif
}

//*****************************************************************************
bool tNMEA2000_teensy::CANOpen() {
#if NMEA2000_TEENSY_VER == 1
  CANbus->begin();
#else
  CANbus->begin(250000);
  CANbus->setNumTXBoxes(NumTxMailBoxes);

  CAN_filter_t filter;
  filter.rtr=0;
  filter.ext=1;
  filter.id=0;
  // Leave one box for other than extended messages
  for (int box=1; box<NUM_MAILBOXES; box++) CANbus->setFilter(filter,box);
  // Set mask to all rx mailboxes. Just fix error on FlexCAN constructor, where numTxMailboxes where not initialized.
  for (int box=0; box<NUM_MAILBOXES; box++) CANbus->setMask(0,box);
#endif
  return true;
}

//*****************************************************************************
bool tNMEA2000_teensy::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  CAN_message_t incoming;

  if (CANbus->available() > 0) {
    CANbus->read(incoming);
    id = incoming.id;
    len = incoming.len;
    for (int i = 0; i < len && i < 8; i++) {
      buf[i] = incoming.buf[i];
    }
    return true;
  }
  else {
    return false;
  }
}

//*****************************************************************************
void tNMEA2000_teensy::InitCANFrameBuffers() {
#if defined(FlexCAN_DYNAMIC_BUFFER_SUPPORT)
  if ( MaxCANReceiveFrames==0 ) MaxCANReceiveFrames=32; // Use default, if not set
  if ( MaxCANReceiveFrames<10 ) MaxCANReceiveFrames=10; // Do not allow less than 10 - Teensy should have enough memory.
  CANbus->setRxBufferSize(MaxCANReceiveFrames);

  if (MaxCANSendFrames==0 ) MaxCANSendFrames=50;  // Use big enough default buffer
  if (MaxCANSendFrames<30 ) MaxCANSendFrames=30; // Do not allow less than 30 - Teensy should have enough memory.

  uint16_t TotalFrames=MaxCANSendFrames;
  MaxCANSendFrames=4; // we do not need much libary internal buffer since driver has them.

#if defined(FlexCAN_MAILBOX_TX_BUFFER_SUPPORT)
  uint16_t MidPriorityBufferSize=TotalFrames-MaxCANSendFrames;
  CANbus->setNumTXBoxes(NumTxMailBoxes);

// With this we can have different buffers for high and mid priority and low priority messages.
// After message has been sent to driver, it buffers it automatically and sends it by interrupt.
// We may need to make these possible to set.
// High priority buffer: Pri 2, Single frame
// Mid priority buffer: Pri 2-4 fast packet, 2-4 single frame
// Low priority buffer: Pri 5-7
  uint16_t HighPriorityBufferSize=MidPriorityBufferSize / 20;
  HighPriorityBufferSize=(HighPriorityBufferSize<5?HighPriorityBufferSize:5); // just guessing
  MidPriorityBufferSize-=HighPriorityBufferSize;
  uint16_t LowestPriorityBufferSize= (MidPriorityBufferSize * 5 / 10);
  if ( LowestPriorityBufferSize<19 ) LowestPriorityBufferSize=19; // We need min this much to get e.g. product information out
  MidPriorityBufferSize-=LowestPriorityBufferSize;

  CANbus->setMailBoxTxBufferSize(CANbus->getFirstTxBox(),HighPriorityBufferSize); // Highest priority buffer
  CANbus->setMailBoxTxBufferSize(CANbus->getFirstTxBox()+1,MidPriorityBufferSize); // Mid priority buffer
  CANbus->setMailBoxTxBufferSize(CANbus->getLastTxBox(),LowestPriorityBufferSize); // Lowest priority buffer
  CANbus->setTxBufferSize(2); // We do not need global buffer, so reserve min.
#else
  uint16_t GlobalBufferSize=TotalFrames-MaxCANSendFrames;
  CANbus->setTxBufferSize(GlobalBufferSize);
#endif

#endif

  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}
