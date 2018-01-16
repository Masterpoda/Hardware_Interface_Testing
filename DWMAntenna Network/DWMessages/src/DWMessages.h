#ifndef _DWMESSAGES_H_INCLUDED
#define _DWMESSAGES_H_INCLUDED

//Class used for managing DWM1000 messages

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define BLINK 4
#define RANGE_FAILED 255

#define ANCHOR_1_ADDRESS  0x01
#define ANCHOR_2_ADDRESS  0x02
#define TAG_3_ADDRESS     0x03
#define TAG_4_ADDRESS     0x04

#define NUM_SETS          0x04    //how many sets of messages per cycle
#define SET_SIZE          0x04    //how many individual messages in a set

#include <inttypes.h>
#include <Arduino.h>


//structure of a single message
struct DWMessage{
  uint8_t type;
  uint8_t from_address;
  uint8_t to_address;
  uint8_t msgInd;
};


class DWMessages{
public:

  DWMessages(){
    initMessageSet(); //initialize message set data
  };

  ~DWMessages(){};

  //returns the index of the next expected message
  static uint8_t getIndex();

  //initializes the set that describes the expected order of messages
  static void initMessageSet();

  //prints a expected DWMessage over serial
  static void printExpected();

  //prints a recieved DWMessage over serial
  static void printRecieved();

  //print the full set of expected messages
  static void printFullSet();

  //returns true if recieved and expected messages are equal
  static bool eqDWMsg();

  //sets expected message by index
  static void setMsgByIndex(uint8_t index);

  //sets the expected message to the next message not sent by the address
  static void setToNextExpected();

  //stores the recieved message
  static void storeRecieved(uint8_t rec_type, uint8_t rec_from, uint8_t rec_to);

  //returns the type, from address and to address of the last recieved message
  static uint8_t recType();
  static uint8_t recFrom();
  static uint8_t recTo();

private:

  //private member function for printing arbitrary DWMessage
  static void printDWMessage(DWMessage msgToPrint);

  //data desctibing the messages we expect to recieve
  static DWMessage messageSets[NUM_SETS][SET_SIZE];

  //data describing the next message we expect to recieve
  static DWMessage expectedMessage;

  //data describing the message we just recieved
  static DWMessage recievedMessage;


};

#endif
