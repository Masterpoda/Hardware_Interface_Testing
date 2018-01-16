#include "DWMessages.h"


//member variables
static DWMessage DWMessages::messageSets[NUM_SETS][SET_SIZE];
static DWMessage DWMessages::expectedMessage;
static DWMessage DWMessages::recievedMessage;

//initialize whole message order
static void DWMessages::initMessageSet()
{
  for(int i = 0; i < NUM_SETS; i++)
  {

    //define types in message order
    messageSets[i][0].type = POLL;
    messageSets[i][1].type = POLL_ACK;
    messageSets[i][2].type = RANGE;
    messageSets[i][3].type = RANGE_REPORT;

    for(int j = 0; j < SET_SIZE; j++)
    {
      messageSets[i][j].msgInd = i*4+j; //setting indicies of a the messages
        if(j%2 == 0)          //even valued messages are sent by anchors
        {

          if(i%2 == 0)        //even message set anchors address tag 3
            messageSets[i][j].from_address = TAG_3_ADDRESS;
          else                //odd message set anchors address tag 4
            messageSets[i][j].from_address = TAG_4_ADDRESS;

          if(i < 2)           //anchor 1
            messageSets[i][j].to_address = ANCHOR_1_ADDRESS;
          else                //anchor 2
            messageSets[i][j].to_address = ANCHOR_2_ADDRESS;

        }
        else                  //odd values are sent by tags
        {
          if(i%2 == 0)        //tag 3 sends on even message sets
            messageSets[i][j].to_address = TAG_3_ADDRESS;
          else                //tag 4 sends on odd message sets
            messageSets[i][j].to_address = TAG_4_ADDRESS;

          if(i < 2)           //anchor 1
            messageSets[i][j].from_address = ANCHOR_1_ADDRESS;
          else                //anchor 2
            messageSets[i][j].from_address = ANCHOR_2_ADDRESS;

        }
    }
  }
}

static uint8_t DWMessages::getIndex()
{
  return expectedMessage.msgInd;
}
//print a sinlge DWMessage
static void DWMessages::printDWMessage(DWMessage msgToPrint)
{
  switch(msgToPrint.type)
  {
    case POLL:
      Serial.print("POLL");
      break;
    case POLL_ACK:
      Serial.print("ACK");
      break;
    case RANGE:
      Serial.print("RANGE");
      break;
    case BLINK:
      Serial.print("BLINK");
      break;
    case RANGE_REPORT:
      Serial.print("REPORT");
      break;
    default:
      Serial.print("UNKNOWN");
      break;
  }
  Serial.print("\tfrom ");Serial.print(msgToPrint.from_address);
  Serial.print(" to ");Serial.print(msgToPrint.to_address);
  Serial.print(" msg Index: ");Serial.println(msgToPrint.msgInd);
}

//print the message we expect to hear next
static void DWMessages::printExpected()
{
  printDWMessage(expectedMessage);
}

//print the message we just recieved
static void DWMessages::printRecieved()
{
  printDWMessage(recievedMessage);
}

//print the full message set
static void DWMessages::printFullSet()
{
  Serial.println("Full Message Set:\n-----------------\n");
  for(int i = 0; i < NUM_SETS; i++)
  {
    for(int j = 0; j < SET_SIZE; j++)
    {
        printDWMessage(messageSets[i][j]);
    }
  }
}

//returns true if expected message matches recieved message
static bool DWMessages::eqDWMsg()
{
  return(expectedMessage.type == recievedMessage.type &&
         expectedMessage.from_address == recievedMessage.from_address &&
         expectedMessage.to_address == recievedMessage.to_address);
}

//sets the message by index
static void DWMessages::setMsgByIndex(uint8_t index)
{
  if(index < NUM_SETS*SET_SIZE && index >= 0)
  {
    expectedMessage.type = messageSets[index/NUM_SETS][index%SET_SIZE].type;
    expectedMessage.from_address = messageSets[index/NUM_SETS][index%SET_SIZE].from_address;
    expectedMessage.to_address = messageSets[index/NUM_SETS][index%SET_SIZE].to_address;
    expectedMessage.msgInd = messageSets[index/NUM_SETS][index%SET_SIZE].msgInd;
  }
  else
  {
    Serial.print("Invalid Index: ");
    Serial.println(index);
  }
}

//sets recieved message to the next message in list
static void DWMessages::setToNextExpected()
{
  expectedMessage.msgInd++;
  if(expectedMessage.msgInd > NUM_SETS*SET_SIZE-1)
  {
    expectedMessage.msgInd = 0;
  }
  
  setMsgByIndex(expectedMessage.msgInd);
}

static void DWMessages::storeRecieved(uint8_t rec_type, uint8_t rec_from, uint8_t rec_to)
{
  recievedMessage.type = rec_type;
  recievedMessage.from_address = rec_from;
  recievedMessage.to_address = rec_to;
  recievedMessage.msgInd = expectedMessage.msgInd;
}

static uint8_t DWMessages::recType()
{
  return recievedMessage.type;
}

static uint8_t DWMessages::recFrom()
{
  return recievedMessage.from_address;
}

static uint8_t DWMessages::recTo()
{
  return recievedMessage.to_address;
}
