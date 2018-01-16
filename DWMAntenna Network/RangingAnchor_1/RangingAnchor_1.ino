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
 * @file RangingAnchor.ino
 * Use this to test two-way ranging functionality with two
 * DW1000. This is the anchor component's code which computes range after
 * exchanging some messages. Addressing and frame filtering is currently done
 * in a custom way, as no MAC features are implemented yet.
 *
 * Complements the "RangingTag" example sketch.
 *
 * @todo
 *  - weighted average of ranging results based on signal quality
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

//used for memory management
#define USE_I2C 1

#include <SPI.h>
#include <DW1000.h>
#include <DWMessages.h>

#ifdef USE_I2C
  #include <Wire.h>
#endif

#define MOVING_FILTER true  //uncomment to filter incoming data in real time
#define ADDRESS 0x01        //the I2C and RF address of the antenna controller


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
//#define DEBUG true                            //print out network traffic


// message variables
DWMessages networkMsgs;

// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
// last computed range/time
DW1000Time timeComputedRange;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 100;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

float distance; //measured distance to tag
const float bias = 0.0; //bias added by antenna delays. Calculated via moving kalman filter at velocity 0
float rateRecieved = 0.0; //rate at which we recieve data

long long lastMsg = 0; //milliseconds since last message
long count = 0; //Number of readings recieved
const long goal = 3000; //Number of readings required (10,000 is usually sufficient to see gaussian distribution.)

//Moving KF Variables
float movingEst[2] =      {0.0,   0.0 }; //current estimate
float variance[2] =       {-1.0, -1.0 }; //variance in estimate
float accuracy[2] =       {0.05,  0.05}; //accuracy in meters
float movementSpeed =  0.5; //Maximum estimated movement speed in m/s (Safest to over-estimate) Set to 0 to calibrate delay
float Kg[2]; //Kalman Gain

//TO DO:
/*
 * 
 * Measure Bias values, ensure they're constant and different ranges
 * if so, theyre simply delays introduced by PCB processing overhead etc.
 * use static Kalman filter to solve for Bias
 * 
 * 1. (DONE)Write Code for Static Kalman filter (i.e. assume steady state truth value)
 * 2. Write code for Dynamic Kalman Filter
 * 3. Incorporate Filter in header/class
 * 4. Wrte structures for timestamped distance readings
 * 5. Write I2C interface
 * 6. Write ROS node using I2C on Odroid C2
 * 
 */

void setup() {
    // DEBUG monitoring
    
    
    Serial.begin(115200);
    networkMsgs.printFullSet();
    delay(1000);
    Serial.println(F("### DW1000-arduino-ranging-anchor ###"));
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println(F("DW1000 initialized ..."));
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(ADDRESS);
    DW1000.setNetworkId(10 + ADDRESS);
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
    // anchor starts in receiving mode, awaiting a ranging poll message
    networkMsgs.setMsgByIndex(0);                //initialize our expected message

    #ifdef USE_I2C
      Wire.begin(ADDRESS);                  // join i2c bus with address ADDRESS
      Wire.onReceive(updateVelocity); // register event
      Wire.onRequest(sendDistance);   // register other event
    #endif

    pinMode(5,OUTPUT); //transmit GPIO indicator.
    pinMode(4,OUTPUT); //recieve GPIO indicator.
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
    
    receiver();
    noteActivity();
    
    // for first time ranging frequency computation
    rangingCountPeriod = millis();

   
   
    
}

const int distanceScale = 8192; // (2^16)/8m

#ifdef USE_I2C
// recieving a new velocity measurement
void updateVelocity() {
  //should only recieve single byte
  uint16_t temp = 0;
  temp |= Wire.read() << 8; //speed sent over in (8m/s)/(2^16)
  temp |= Wire.read();
  movementSpeed = ((float)temp) / distanceScale;
  //Serial.println("Velocity Updated!");
}

//recieving a request for distance data
void sendDistance()
{
  uint16_t temp;
  for(int i = 0; i < 2; i++)
  {
    if(movingEst[i] > 8.0)
      temp = 8.0 * distanceScale;
    else
      temp = movingEst[i] * distanceScale;
    
    // 8m is the max distance we should ever measure.
    // distance sent in 2 bytes multiples of  8m/(2^16)
    Wire.write((char)(temp>>8));
    Wire.write((char)(temp & 0x00FF));
  }
}
#endif

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
    protocolFailed = false;
    networkMsgs.setMsgByIndex(0);
    receiver();
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

void transmitPollAck(byte destination) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL_ACK;
    // delay the same amount as ranging tag
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    data[0] |= (ADDRESS-1) <<4;         //bits 4 and 5 are the FROM bits
    data[0] |= (destination - 1) << 6;    //bits 6 and 7 are the TO bits
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeReport(float curRange, byte destination) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    data[0] |= (ADDRESS-1) << 4;        //bits 4 and 5 are the FROM bits
    data[0] |= (destination - 1) << 6;    //bits 6 and 7 are the TO bits
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

void computeRangeAsymmetric() {
    // asymmetric two-way ranging (more computation intense, less error prone)
    DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
    DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
    DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}


/*
 * END RANGING ALGORITHMS
 * ----------------------
 */

 //Filter algorithm

//Arguments are newest reading and time of previous reading in milliseconds
 void movingFilter(float estimate, uint32_t currTime, uint32_t prevTime, uint8_t tagNum)
 {
  uint8_t index = tagNum - 3;
  long timeInc = 0; //milliseconds between readings
  if(variance < 0)//initialize global filter variables
  {
    variance[index] = accuracy[index]; //initial variance is accuracy^2
    movingEst[index] = estimate;        //initial distance estimate is just first reading
    Serial.print("Initialized at: ");Serial.println(estimate);
  }
  else
  {
    timeInc = currTime - prevTime;
    
    if(timeInc > 0)
    {
      //this estimate assumes max movement speed. Including velocity info via I2C bus could improve this. 
      variance[index] += (timeInc * movementSpeed * movementSpeed) / 1000.0; //variance increases by max potential movement speed
    }
    Kg[index] = variance[index] / (variance[index] + accuracy[index]);
    movingEst[index] += Kg[index] * (estimate - movingEst[index]);
    variance[index] = (1.0-Kg[index])*variance[index];
  }
    
 }

uint16_t sent_count = 0;
void loop() {
    int32_t curMillis = millis();

    if (!sentAck && !receivedAck)//if we're waiting for poll, dont time out. 
    {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            Serial.print("Timed out waiting for: ");
            networkMsgs.printExpected();
            resetInactive();
        }
        return;
    }
    else if(!sentAck && !receivedAck)
    {
      if (curMillis - lastActivity > 10000) 
      {
            //if we enter here then the system has failed and requires rebooting
      }
    }
    // continue on any success confirmation
    if (sentAck) {
        digitalWrite(5, HIGH);
        sentAck = false;
        byte msgId = data[0]&0x0F;
        if (msgId == POLL_ACK)        //this anchor sent a poll ack 
        {
          DW1000.getTransmitTimestamp(timePollAckSent);
        }

        //important that we increment message after Tx to expect response
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
        //printData(data, LEN_DATA);//uncomment to read all incoming data

        //storing the message we just recieved
        byte rec_from = ((data[0] >> 4) & 0x03) + 1;    //grab bits 4 and 5 and add one
        byte rec_to = ((data[0] >> 6) & 0x03) + 1;     //grab bits 6 and 7 and add one
        
        if(data[0] != RANGE_FAILED)
          data[0] &= 0x0F;//filter out address stamps if not Range failure
        
        byte rec_type = data[0];  

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
          networkMsgs.setMsgByIndex(0); 
        }
        
        //if we recieved a poll message
        if (networkMsgs.recType() == POLL) {
          if(networkMsgs.recTo() == ADDRESS)
          {       
            // if poll is to our Address
            //get the timestamp and send our ack back to sending tag
            if(networkMsgs.recFrom() == 0x03)
            {        
              protocolFailed = false;                     //reset failure flag, as this is where the protocol begins
              networkMsgs.setMsgByIndex(0);
            }
            DW1000.getReceiveTimestamp(timePollReceived);
            transmitPollAck(networkMsgs.recFrom());       //send a poll ack
            //increment expected message
            sent_count++;//
            networkMsgs.setToNextExpected();
          }
          else
          {
            //poll is to tag anchor 2, increment expeted message
            networkMsgs.setToNextExpected();
          }
            
        }
        else if (networkMsgs.recType() == RANGE && !protocolFailed) //if we recieved a range message and we aren't out of sync
        {

          if(networkMsgs.recTo() == ADDRESS)
          {   
              //if range is sent to our Address, compute and process measurement
              DW1000.getReceiveTimestamp(timeRangeReceived);
              timePollSent.setTimestamp(data + 1);
              timePollAckReceived.setTimestamp(data + 6);
              timeRangeSent.setTimestamp(data + 11);
              // (re-)compute range as two-way ranging is done
              computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
              transmitRangeReport(timeComputedRange.getAsMicroSeconds(), networkMsgs.recFrom());
              distance = timeComputedRange.getAsMeters();
              
              networkMsgs.setToNextExpected();
            
              #ifdef MOVING_FILTER
                  if(abs(distance) < 8.0) //throw out outliers
                    movingFilter(distance, curMillis, lastMsg, networkMsgs.recFrom());
              #endif

              //distance and power output for debugging/accuracy testing.
              rateRecieved = 1.0/((millis() - lastMsg)/1000.0);
              lastMsg = curMillis;
              count++;
              //Serial.print(distance - bias, 4); Serial.print("\t"); Serial.print(movingEst, 4); Serial.print("\t"); Serial.print(DW1000.getReceivePower()); Serial.print("\t");Serial.println(rateRecieved);
              
              
              //Serial.print("Measurement: ");Serial.print(distance, 4);Serial.print("\t Moving Estimate: ");Serial.println(movingEst);
              //Serial.print(distance, 4);Serial.print("\t");Serial.print(movingEst, 4);Serial.print("\t");Serial.println(networkMsgs.recFrom(), HEX);
              Serial.print("Tag 3 distance: ");Serial.print(movingEst[0], 4);Serial.print("m\t");Serial.print("Tag 4 distance: ");Serial.print(movingEst[1], 4);Serial.println("m");
              
              //use this for taking a finite amount of readings
              /*
              if(false)//count > goal)
              {
                Serial.println("Test Complete!");
                Serial.print("Time to completion: ");Serial.print(millis()/(1000.0*60)); Serial.println(" minutes.");
                while(true)
                {
                  //Spin lock
                }
              }
              */
              //Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
              //Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
              //Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
              //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
              //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
              //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
              // update sampling rate (each second)
              successRangingCount++;
              if (curMillis - rangingCountPeriod > 1000) {
                  samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                  rangingCountPeriod = curMillis;
                  successRangingCount = 0;
              }
            }
          else
          {
            //range message is to other anchor
            networkMsgs.setToNextExpected();
            
          }
        }
        else if(networkMsgs.recType() == POLL_ACK && !protocolFailed) //heard ACK from anchor 2
        {
          networkMsgs.setToNextExpected();
        }
        else if(networkMsgs.recType() == RANGE_REPORT && !protocolFailed) //other anchor is sending range report
        {
          networkMsgs.setToNextExpected(); //heard report to anchor 2
          
        }
        else if (protocolFailed)
        {
            resetInactive();
        }
        digitalWrite(4, LOW);

        noteActivity();
        
    }
}

#ifdef DEBUG
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

#endif



