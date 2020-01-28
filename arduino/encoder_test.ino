/* Program to test encoders. (Connect both encoders)
 *  Centauri Robotics, All rights reserved
 *  Date: December 13, 2019
 */
//////////////////////////////////////////////////////////////////////////////////////////// 
//Encoder pins definition

// Left encoder
//Channel A to Pin 2
//Channel B to Pin 9
int Left_Encoder_PinA = 2;
int Left_Encoder_PinB = 9;

//Encoder value counter (Part of program; Not a hardware pin)
volatile long Left_Encoder_Ticks = 0;

//Variable to read current state of left encoder pin
//This pins along with Channel A which is set as interrupt, allows to detect the direction
volatile bool LeftEncoderBSet;

/////////////////////////////////////////////////////////////////////////////////////////////

//Right Encoder

int Right_Encoder_PinA = 3;
int Right_Encoder_PinB = 10;

//Encoder value counter (Part of program; Not a hardware pin)
volatile long Right_Encoder_Ticks = 0;

//Variable to read current state of right encoder pin
//This pins along with Channel A which is set as interrupt, allows to detect the direction
volatile bool RightEncoderBSet;

////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  //Init Serial port with 115200 buad rate
  Serial.begin(115200);  
  SetupEncoders();
}


void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT);      // sets pin A pullup
  pinMode(Left_Encoder_PinB, INPUT);      // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);
  

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT);      // sets pin A pullup
  pinMode(Right_Encoder_PinB, INPUT);      // sets pin B pullup

  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING); 
}

void loop()
{
  Update_Encoders();
}


void Update_Encoders()
{
   Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
 }

 void do_Left_Encoder()
{
   LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
   
}
void do_Right_Encoder()
{
   RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
 }
