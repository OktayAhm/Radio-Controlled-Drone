#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;
Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;
struct Data_Package {
byte throttle;      
byte pitch;
byte roll;
byte yaw;
byte aux1;
byte aux2;
};

Data_Package data;
const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL;
RF24 radio(3, 2); 

void ResetData()
{
data.roll = 127;   
data.pitch = 127;  
data.throttle = 12; 
data.yaw = 127;  
data.aux1 = 127;   
data.aux2 = 127;  
}
void setup()
{

  ch1.attach(4);
  ch2.attach(5);
  ch3.attach(6);
  ch4.attach(7);
  ch5.attach(8);
  ch6.attach(9);

  ResetData();
  radio.begin();
  radio.openReadingPipe(0,my_radio_pipe);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening(); 
  pinMode(6,OUTPUT);
}
unsigned long lastRecvTime = 0;
void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(Data_Package));
lastRecvTime = millis();  
}
}
void loop()
{
recvData();
unsigned long now = millis();
if ( now - lastRecvTime > 1000 ) {
ResetData();
}
ch_width_4 = map(data.yaw,      0, 255, 1000, 2000);    
ch_width_2 = map(data.pitch,    0, 255, 1000, 2000);     
ch_width_3 = map(data.throttle, 0, 255, 1000, 2000);     
ch_width_1 = map(data.roll,     0, 255, 1000, 2000);    
ch_width_5 = map(data.aux1,     0, 255, 1000, 2000);     
ch_width_6 = map(data.aux2,     0, 255, 1000, 2000);    

ch1.writeMicroseconds(ch_width_1);
ch2.writeMicroseconds(ch_width_2);
ch3.writeMicroseconds(ch_width_3);
ch4.writeMicroseconds(ch_width_4);
ch5.writeMicroseconds(ch_width_5);
ch6.writeMicroseconds(ch_width_6);
}
