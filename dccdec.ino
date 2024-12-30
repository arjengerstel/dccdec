#include <Arduino.h>
#include <NmraDcc.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Modified from example in NMRA DCC library
//
// Decoder for 16 servo-type turnout motors
// Controlled by I2C Servo driver board
//

#define DEBUG

NmraDcc  Dcc ;
DCC_MSG  Packet ;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625     

// Define the Arduino input Pin number for the DCC Signal 
#define DCC_PIN     2

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, DEFAULT_ACCESSORY_DECODER_ADDRESS & 0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, DEFAULT_ACCESSORY_DECODER_ADDRESS >> 8},
};

struct TurnoutSetting
{
  uint8_t Address;
  uint16_t PosMin;
  uint16_t PosMax;
};

#define NUMTURNOUTS 6

// Turnout numbers and servo motor settings for each turnout motor
TurnoutSetting Turnout[] = {
  {1, 60, 120},
  {2, 60, 120},
  {3, 60, 120},
  {4, 60, 120},
  {5, 60, 120},
  {6, 60, 120}
};

uint16_t PrevAddr = 0;
uint8_t PrevDirection = 0;

uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }

const int DccAckPin = 3 ;

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void)
{
  Serial.println("notifyCVAck") ;
  
  digitalWrite( DccAckPin, HIGH );
  delay( 6 );  
  digitalWrite( DccAckPin, LOW );
}

// Uncomment to print all DCC Packets
//#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This function is called whenever a normal DCC Turnout Packet is received and we're in Board Addressing Mode
/*
void notifyDccAccTurnoutBoard( uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower )
{
  Serial.print("notifyDccAccTurnoutBoard: ") ;
  Serial.print(BoardAddr,DEC) ;
  Serial.print(',');
  Serial.print(OutputPair,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;
}
*/

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  // Filter multiple identical DCC turnout commands
  if ((Addr == PrevAddr) && (Direction == PrevDirection)) {
    // Serial.println("Duplicate command");
    return;
  }
  
 #ifdef DEBUG
  Serial.print("notifyDccAccTurnoutOutput: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;
#endif

  for(int i=0; i<NUMTURNOUTS; i++) {
    if(Turnout[i].Address == Addr) {
      Serial.print("Setting Turnout#");
      Serial.print(Addr);
      Serial.print(" to position ");
      if(Direction == 1) 
      {
        Serial.println(Turnout[i].PosMax);
        pwm.setPWM(Turnout[i].Address - 1, 0, angleToPulse(Turnout[i].PosMax));
      } 
      else 
      {
        Serial.println(Turnout[i].PosMin);
        pwm.setPWM(Turnout[i].Address - 1, 0, angleToPulse(Turnout[i].PosMin));
      }
    } 
  }
  PrevAddr = Addr;
  PrevDirection = Direction;
}

// This function is called whenever a DCC Signal Aspect Packet is received
/*
void notifyDccSigOutputState( uint16_t Addr, uint8_t State)
{
  Serial.print("notifyDccSigOutputState: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.println(State, HEX) ;
}
*/


void setup()
{
  Serial.begin(115200);
  uint8_t maxWaitLoops = 255;
  while(!Serial && maxWaitLoops--)
    delay(20);
      
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );

  Serial.println("16-servo wisseldecoder");
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  Serial.println("DCC Init Done");

  // Initialize PWM I2C servo driver
  pwm.begin();
  pwm.setPWMFreq(60); 

  Serial.println("Servo Driver Initalized");
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}
