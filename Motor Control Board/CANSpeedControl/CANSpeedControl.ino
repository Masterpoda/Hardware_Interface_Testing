// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

#include <mcp_can.h>
#include <SPI.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}



unsigned char id = 0x3; 
int16_t MotorSpeed = 900; 

void loop()
{
    /*
    //CANReport();
    for(MotorSpeed = 100; MotorSpeed < 950; MotorSpeed +=5)
    {
      CANMonitor();
  
      delay(100);                       // send data per 100ms
      SetSpeed(id, MotorSpeed);
      GetCurrentConsumption(id);
    }

    */

    SetSpeed(id, MotorSpeed*(-1));
    RelativeEncoder(id);
    //CANMonitor();
    CANReport();

}

void SetSpeed(uint8_t id, int16_t motorSpeed)
{
  unsigned char stmp[8] = {id, 0, 0, 1, 0,  0, 0, 0}; // set speed of motor 1 (index 3)

  /*
   * The last 4 bytes of stmp indicate the signed motor speed. 
   * Speeds only go from -1000 to 1000, so only first 2 bytes are needed.
   * however the final 2 bits must be set to 0xFF for negative values.
   */
  
  if(motorSpeed >= 0 && motorSpeed <= 1000)
  {
    stmp[5] = motorSpeed>>8;        //Low nybble
    stmp[4] = motorSpeed & 0xFF;    //High nybble
  }
  else if(motorSpeed < 0 && motorSpeed >= -1000)
  {
    stmp[7] = 0xFF;     
    stmp[6] = 0xFF;    
    stmp[5] = motorSpeed>>8;        //Low nybble
    stmp[4] = motorSpeed & 0xFF;    //High nybble
  }

  id = (id<<4) | 0x06;    //not sure why the ID has to be formatted like this. probably has to do with roboCAN
                          //in truth, the ID is set as 3 on the MCB. my guess is that the high nybble is the
                          //address while the low is an opcode
  CAN.sendMsgBuf(id, id, 8, stmp);
}

void GetCurrentConsumption(uint8_t id)
{
  //"Who's asking?"         (index 0)
  //"Whatcha want?"         (index 1-2)
  //get data from battery 1 (index 3)
  unsigned char stmp[4] = {1, 0x0C, 0, 1}; 
  
  id = (id<<4) | 0x05; //opcode of 5

  CAN.sendMsgBuf(id, id, 4, stmp);
  
}

void CANReport()
{
  unsigned char len = 0;
  unsigned char buf[8];
  
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
  
    unsigned int canId = CAN.getCanId();
    
    Serial.println("-----------------------------");
    Serial.print("Get data from ID: ");
    Serial.println(canId, HEX);
  
    for(int i = 0; i<len; i++)    // print the data
    {
        Serial.print(buf[i], HEX);
        Serial.print("\t");
    }
    Serial.println();
  }
}

//This function will be used to update stored info on motors (which will likely be a class I'll write later)
void CANMonitor()
{
  unsigned char len = 0;
  unsigned char buf[8];
  
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
  
    unsigned int canId = CAN.getCanId();

    for(int i = 0; i<len; i++)
    {
      //parsing the buffer will be much more sophisticated once we better understand the Data labels.
      if(buf[0] == 3)
      {
        if(buf[1]==0x0C)
        {
          int8_t current = buf[4];
          current += 2;
          Serial.print("Speed: ");Serial.print(MotorSpeed/10.0);Serial.print("% ");
          Serial.print("Current: ");Serial.print(current*100);Serial.println(" mA");
        }
        if(buf[1]==0x58)
        {
          int32_t encoder = buf[4] | buf[5]<<8 | buf[6]<<16 | buf[7]<<24;
          Serial.print("Speed: ");Serial.print(MotorSpeed/10.0);Serial.print("% ");
          Serial.print("Encoder: ");Serial.print(encoder);Serial.println(" ticks");
        }
      }
      
    }

  }
  
}

//Sends out a request for absolute encoder position from MCB with given ID
void AbsoluteEncoder(uint8_t id)
{
  //"Who's asking?"         (index 0)
  //"Whatcha want?"         (index 1-2)
  //get data from motor 1   (index 3)
  unsigned char stmp[4] = {1, 0x05, 0, 1}; 
  
  id = (id<<4) | 0x05; //opcode of 5

  CAN.sendMsgBuf(id, id, 4, stmp);
}

//Sends out a request for relative encoder position from MCB with given ID
void RelativeEncoder(uint8_t id)
{
  //"Who's asking?"         (index 0)
  //"Whatcha want?"         (index 1-2)
  //get data from motor 1   (index 3)
  unsigned char stmp[4] = {1, 0x58, 0, 1}; 
  
  id = (id<<4) | 0x05; //opcode of 5

  CAN.sendMsgBuf(id, id, 4, stmp);
}
// END FILE
