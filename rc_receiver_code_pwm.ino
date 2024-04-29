#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

//Define widths
int pwm_width_4 = 0;
int pwm_width_5 = 0;
int pwm_width_6 = 0;
int pwm_width_7 = 0;
int pwm_width_8 = 0;
int pwm_width_9 = 0;

Servo PWM4;
Servo PWM5;
Servo PWM6;
Servo PWM7;
Servo PWM8;
Servo PWM9;

                    //We could use up to 32 channels
struct Data_Package {
  byte yaw;
  byte throttle;
  byte j1Button;
  
  byte roll;
  byte pitch;
  byte j2Button;
  
  byte pot1;
  byte pot2;
  byte AUX1;
  byte AUX2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data;


const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL;
RF24 radio(3, 2); 



void resetData()
{
//We define the inicial value of each data input
//3 potenciometers will be in the middle position so 127 is the middle from 254
data.throttle = 0;
data.yaw = 127;
data.pitch = 127;
data.roll = 127;
data.AUX1 = 0;
data.AUX2 = 0;
}

/**************************************************/

void setup()
{
  Wire.begin();
  //Set the pins for each PWM signal
  PWM4.attach(4);
  PWM5.attach(5);
  PWM6.attach(6);
  PWM7.attach(7);
  PWM8.attach(8);
  PWM9.attach(9);

  //Configure the NRF24 module
  resetData();
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,my_radio_pipe);
  Serial.begin(9600);
  //we start the radio comunication
  radio.startListening();

}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(Data_Package));
lastRecvTime = millis(); //here we receive the data
}
}

/**************************************************/

void loop()
{
recvData();
unsigned long now = millis();
//Here we check if we've lost signal, if we did we reset the values 
if ( now - lastRecvTime > 1000 ) {
// Signal lost?
resetData();
}

pwm_width_4 = map(data.pot1, 0, 255, 1000, 2000);     //PWM value on digital pin D2
pwm_width_5 = map(data.yaw,      0, 255, 1000, 2000);     //PWM value on digital pin D3
pwm_width_6 = map(data.pitch,    0, 255, 1000, 2000);     //PWM value on digital pin D4
pwm_width_7 = map(data.roll,     0, 255, 1000, 2000);     //PWM value on digital pin D5
pwm_width_8 = map(data.AUX2,     0, 255, 1000, 2000);     //PWM value on digital pin D6
pwm_width_9 = map(data.AUX1,     0, 255, 1000, 2000);     //PWM value on digital pin D7


//Now we write the PWM signal using the servo function
PWM4.writeMicroseconds(pwm_width_4);
PWM5.writeMicroseconds(pwm_width_5);
PWM6.writeMicroseconds(pwm_width_6);
PWM7.writeMicroseconds(pwm_width_7);
PWM8.writeMicroseconds(pwm_width_8);
PWM9.writeMicroseconds(pwm_width_9);

}//Loop end
