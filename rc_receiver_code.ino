#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(3, 2);  
const byte address[6] = "00001"; 
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

Servo throttle; 
Servo rudder; 
Servo elevator;
Servo aileron; 

Servo aux1; 
Servo aux2; 

int throttleValue;
int rudderValue;
int elevatorValue;
int aileronValue;

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


void setup() {
  
  throttle.attach(10); 
  rudder.attach(5);   
  elevator.attach(6);  
  aileron.attach(9);  
  
  aux1.attach(4);     
  aux2.attach(7);     
  
  
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  resetData();
 
}
void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); 
    lastReceiveTime = millis(); 
  }

  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { 
    resetData(); 
  }

  Serial.print("j1PotX: ");
  Serial.print(data.yaw);
  Serial.print("; j1PotY: ");
  Serial.print(data.throttle);

  Serial.print("j2PotX: ");
  Serial.print(data.roll);
  Serial.print("; j2PotY: ");
  Serial.print(data.pitch);

  throttleValue = constrain(data.throttle, 80, 255); 
  throttleValue = map(throttleValue, 127, 255, 1000, 2000);
  throttle.writeMicroseconds(throttleValue);

}

void resetData() {

  data.yaw = 127;
  data.throttle = 127;
  data.roll = 127;
  data.pitch = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.AUX1 = 1;
  data.AUX2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}
