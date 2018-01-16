/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file RangingTag.ino
 * Use this to test two-way ranging functionality with two DW1000. This is
 * the tag component's code which polls for range computation. Addressing and
 * frame filtering is currently done in a custom way, as no MAC features are
 * implemented yet.
 *
 * Complements the "RangingAnchor" example sketch.
 *
 * @todo
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

#include <SPI.h>
#include <DW1000.h>
#include <DWMessages.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define BLINK 4
#define RANGE_FAILED 255

#define ADDRESS 0x03      //Tag 3, placed on right of hopper, Tag 4 on the left
//#define NETWORK_ID 0xAB   //Network ID of Tag
#define DEBUG true        //print out network traffic

// message variables
DWMessages networkMsgs;

// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
boolean protocolFailed = false;

// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;

// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];

// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 100;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    networkMsgs.printFullSet();
    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println("DW1000 initialized ...");
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(ADDRESS);
    DW1000.setNetworkId(ADDRESS + 10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000.getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    // tag starts by transmitting a POLL message

    pinMode(5,OUTPUT); //transmit GPIO indicator.
    pinMode(4,OUTPUT); //recieve GPIO indicator.
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
    
    networkMsgs.setMsgByIndex(0);                //initialize our expected message
    receiver();
    transmitPoll(0x01);//initialize expected message variables, send poll
    
    
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    // add delay to ensure all other antennas time out?

    //delay(resetPeriod/2); //ensure all other antennas reset.
    protocolFailed = false;

    networkMsgs.setMsgByIndex(0);                //initialize our expected message
    transmitPoll(0x01);

    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPoll(byte destination) {
  
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;//hide address in first nybble
    //Serial.println(data[0]);
    data[0] |= (ADDRESS-1) <<4;
    data[0] |= (destination-1) <<6;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();

    
}

void transmitRange(byte destination) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;//

    // delay sending the message and remember expected future sent timestamp
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    data[0] |= (ADDRESS-1) <<4;
    data[0] |= (destination-1) <<6;
    //printData(data, LEN_DATA);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void loop() {

    if (!sentAck && !receivedAck) {
        // check if inactive
        if (millis() - lastActivity > resetPeriod) {
            Serial.print("Timed out waiting for ");
            networkMsgs.printExpected();
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        digitalWrite(5, HIGH);
        sentAck = false;
        byte msgId = data[0]&0x0F;
        if (msgId == POLL) {
            DW1000.getTransmitTimestamp(timePollSent);  //log timestamp if we sent a poll
            if(networkMsgs.getIndex() == 0x01)                       //sent a poll to Anchor 1 means we've reset
              protocolFailed = false;
            //Serial.println("Sent POLL.");
        } else if (msgId == RANGE) {
            DW1000.getTransmitTimestamp(timeRangeSent);
        }

        networkMsgs.setToNextExpected();
        
        noteActivity();
        digitalWrite(5, LOW);
    }
    if (receivedAck) 
    {
        digitalWrite(4, HIGH);
        receivedAck = false;
        // get message and parse
        DW1000.getData(data, LEN_DATA);

         //storing the message we just recieved
        byte rec_from = ((data[0] >> 4) & 0x03) + 1;    //grab bits 4 and 5 and add one
        byte rec_to = ((data[0] >> 6) & 0x03) + 1;     //grab bits 6 and 7 and add one
        
        if(data[0] != RANGE_FAILED)
          data[0] &= 0x0F;//filter out address stamps if not Range failure
        
        byte rec_type = data[0]; 

        //store recieved message
        networkMsgs.storeRecieved(rec_type, rec_from, rec_to);
        
        //#ifdef DEBUG
          //Serial.print("Recieved ");networkMsgs.printRecieved();
        //#endif
        
        //is it the correct message to the correct reciever from the correct sender
        if(!networkMsgs.eqDWMsg())
        {
          protocolFailed = true;  //messages are out of sync, wait until next poll
          #ifdef DEBUG
            Serial.println("Messages are out of sync!");
            Serial.print("Expected: ");networkMsgs.printExpected();
            Serial.print("Recieved: ");networkMsgs.printRecieved();
          #endif
          //re-initialize message expectations
          networkMsgs.setMsgByIndex(0);  //Ranging tag 3 should be initialized at 1 
        }
        
        if (networkMsgs.recType() == POLL_ACK && !protocolFailed) 
        {
          if(networkMsgs.recTo() == ADDRESS)//poll ack was meant for this tag
          {
            DW1000.getReceiveTimestamp(timePollAckReceived);
            transmitRange(networkMsgs.recFrom());         //send range to same device
            networkMsgs.setToNextExpected();
            //Serial.print("Sent range to ");Serial.println(recievedMessage.from_address);
            
          }
          else  //poll ack was meant for other tag
          {
            networkMsgs.setToNextExpected();
          }
          
        } 
        else if(networkMsgs.recType() == POLL && !protocolFailed)//we're hearing the poll from Tag 4
        { 
          networkMsgs.setToNextExpected();
        }
        else if(networkMsgs.recType() == RANGE && !protocolFailed)//recieved range from other tag
        {
          networkMsgs.setToNextExpected();
        }
        else if (networkMsgs.recType() == RANGE_REPORT && !protocolFailed) //if we recieve a range report 
        {
          if(networkMsgs.recTo() == ADDRESS)//range report is meant for us
          {
            float curRange;
            memcpy(&curRange, data + 1, 4);
            networkMsgs.setToNextExpected();
            //Serial.print("Report recieved, expecting: ");printDWMessage(expectedMessage); 
            
          }
          else  //An anchor just reported to tag 4
          {
            networkMsgs.setToNextExpected();         

            //tag 3 polls the opposite anchor to the one that just reported
            if(networkMsgs.recFrom() == ANCHOR_1_ADDRESS)
            {  
              transmitPoll(ANCHOR_2_ADDRESS);
            }
            else if(networkMsgs.recFrom() == ANCHOR_2_ADDRESS)
            {  
              transmitPoll(ANCHOR_1_ADDRESS);
            }

          }
            
        } 
        else if (networkMsgs.recType() == RANGE_FAILED || protocolFailed) 
        {
            resetInactive();  //reset states if failure occurs.
            
            
        }
        noteActivity();
        digitalWrite(4, LOW);
    }
}

void printData(byte data[], int lenData)
{
  for(int i = 0; i < lenData; i++)
  {
    if(data[i] < 0x10)
      Serial.print(" 0x0");
    else
      Serial.print(" 0x");
      
    Serial.print(data[i], HEX);   
  }
  Serial.println("");//new line
}


