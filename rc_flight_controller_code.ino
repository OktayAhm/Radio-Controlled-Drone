#include <Wire.h>
#include <Servo.h>

Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;

byte i2c_rcv;          
unsigned long time_start;  

int pwm_width_4 = 0;
int pwm_width_5 = 0;
int pwm_width_6 = 0;
int pwm_width_7 = 0;
int pwm_width_8 = 0;
int pwm_width_9 = 0;

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

unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

int input_YAW = 6; 
int input_PITCH = 5;   
int input_ROLL = 4;   
int input_THROTTLE = 2; 

float elapsedTime, time, timePrev;     
int gyro_error=0;                        
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;      
float Gyro_angle_x, Gyro_angle_y;       
float Gyro_raw_error_x, Gyro_raw_error_y; 

int acc_error=0;                           
float rad_to_deg = 180/3.141592654;         
float Acc_rawX, Acc_rawY, Acc_rawZ;         
float Acc_angle_x, Acc_angle_y;             
float Acc_angle_error_x, Acc_angle_error_y; 

float Total_angle_x, Total_angle_y;

int i;
int mot_activated=0;
long activate_count=0;
long des_activate_count=0;

float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;

double roll_kp=0.7;//3.55
double roll_ki=0.006;//0.003
double roll_kd=1.2;//2.05
float roll_desired_angle = 0;     

float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;

double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05
float pitch_desired_angle = 0;    

                              

void setup() {

  Wire.begin(9); 
  Wire.onReceive(receiveEvent);

  L_F_prop.attach(3); 
  L_B_prop.attach(10); 
  R_F_prop.attach(11); 
  R_B_prop.attach(9); 

  L_F_prop.writeMicroseconds(1000); 
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000); 
  R_B_prop.writeMicroseconds(1000);
  
  
  Wire.begin();                          
  Wire.beginTransmission(0x68);          
  Wire.write(0x6B);                      
  Wire.write(0x00);
  Wire.endTransmission(true);            
  
  Wire.beginTransmission(0x68);          
  Wire.write(0x1B);                      
  Wire.write(0x10);                     
  Wire.endTransmission(true);           

  Wire.beginTransmission(0x68);        
  Wire.write(0x1C);                       
  Wire.write(0x10);                 
  Wire.endTransmission(true);  


  Serial.begin(9600);
  delay(1000);
  time = millis();                        

  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);          
      Wire.write(0x43);                       
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,4,true);          
         
      Gyr_rawX=Wire.read()<<8|Wire.read();    
      Gyr_rawY=Wire.read()<<8|Wire.read();
   
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
      if(i==199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x/200;
        Gyro_raw_error_y = Gyro_raw_error_y/200;
        gyro_error=1;
      }
    }
  }  

  if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                       
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; 
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;

      
      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg)); 
      
      if(a==199)
      {
        Acc_angle_error_x = Acc_angle_error_x/200;
        Acc_angle_error_y = Acc_angle_error_y/200;
        acc_error=1;
      }
    }
  }
}

void receiveEvent() {
  pwm_width_4 = Wire.read();    
  pwm_width_5 = Wire.read();
  pwm_width_6 = Wire.read();
  pwm_width_7 = Wire.read();
}

void loop() {
  
  timePrev = time; 
  time = millis();  
  elapsedTime = (time - timePrev) / 1000;     

  Wire.beginTransmission(0x68);            
  Wire.write(0x43);                      
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true);                  
  Gyr_rawX=Wire.read()<<8|Wire.read();     
  Gyr_rawY=Wire.read()<<8|Wire.read();
  Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
  Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;  

  Gyro_angle_x = Gyr_rawX*elapsedTime;
  Gyro_angle_y = Gyr_rawY*elapsedTime;

  Wire.beginTransmission(0x68);   
  Wire.write(0x3B);                
  Wire.endTransmission(false);      
  Wire.requestFrom(0x68,6,true);    
    
  Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;  

 Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
 Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;   

 Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
 Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;

roll_desired_angle = map(pwm_width_7,1000,2000,-10,10);
pitch_desired_angle = map(pwm_width_6,1000,2000,-10,10);

roll_error = Total_angle_y - roll_desired_angle;
pitch_error = Total_angle_x - pitch_desired_angle;    

roll_pid_p = roll_kp*roll_error;
pitch_pid_p = pitch_kp*pitch_error;

if(-3 < roll_error <3)
{
  roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
}
if(-3 < pitch_error <3)
{
  pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
}

roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
/*The final PID values is the sum of each of this 3 parts*/
roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

if(roll_PID < -400){roll_PID=-400;}
if(roll_PID > 400) {roll_PID=400; }
if(pitch_PID < -400){pitch_PID=-400;}
if(pitch_PID > 400) {pitch_PID=400;}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwm_R_F  = 115 + pwm_width_4 - roll_PID - pitch_PID;
pwm_R_B  = 115 + pwm_width_4 - roll_PID + pitch_PID;
pwm_L_B  = 115 + pwm_width_4 + roll_PID + pitch_PID;
pwm_L_F  = 115 + pwm_width_4 + roll_PID - pitch_PID;



/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right front
if(pwm_R_F < 1100)
{
  pwm_R_F= 1100;
}
if(pwm_R_F > 2000)
{
  pwm_R_F=2000;
}

//Left front
if(pwm_L_F < 1100)
{
  pwm_L_F= 1100;
}
if(pwm_L_F > 2000)
{
  pwm_L_F=2000;
}

//Right back
if(pwm_R_B < 1100)
{
  pwm_R_B= 1100;
}
if(pwm_R_B > 2000)
{
  pwm_R_B=2000;
}

//Left back
if(pwm_L_B < 1100)
{
  pwm_L_B= 1100;
}
if(pwm_L_B > 2000)
{
  pwm_L_B=2000;
}

roll_previous_error = roll_error; //Remember to store the previous error.
pitch_previous_error = pitch_error; //Remember to store the previous error.



/*now we can write the values PWM to the ESCs only if the motor is activated
*/

  if(mot_activated)
  {
  L_F_prop.writeMicroseconds(pwm_L_F); 
  L_B_prop.writeMicroseconds(pwm_L_B);
  R_F_prop.writeMicroseconds(pwm_R_F); 
  R_B_prop.writeMicroseconds(pwm_R_B);
  }
  if(!mot_activated)
  {
    L_F_prop.writeMicroseconds(1000); 
    L_B_prop.writeMicroseconds(1000);
    R_F_prop.writeMicroseconds(1000); 
    R_B_prop.writeMicroseconds(1000);
  }

  if(pwm_width_4 < 1100 && pwm_width_5 > 1800 && !mot_activated)
  {
    if(activate_count==200)
    {
      mot_activated=1;   
      PORTB |= B00100000; //D13 LOW   
      
    }
    activate_count=activate_count+1;
  }
  if(!(pwm_width_4 < 1100 && pwm_width_5 > 1800) && !mot_activated)
  {
    activate_count=0;    
  }

   if(pwm_width_4 < 1100 && pwm_width_5 < 1100 && mot_activated)
  {
    if(des_activate_count==300)
    {
      mot_activated=0;       
      PORTB &= B11011111; //D13 LOW   
    }
    des_activate_count=des_activate_count+1;
  }
  if(!(pwm_width_4 < 1100 && pwm_width_5 < 1100) && mot_activated)
  {
    des_activate_count=0;
  }
 
}














/*
ISR(PCINT0_vect){
//First we take the current count value in micro seconds using the micros() function
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_ROLL = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }



  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }


  
  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }


  
  ///////////////////////////////////////Channel 4
  if(PINB & B00010000 ){                             //pin D12  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }


 
}
 */
