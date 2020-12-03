#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>
int encoder_pin = 2; // pulse output from the module
unsigned int rpm=0; // rpm reading
volatile byte pulses=0; // number of pulses
unsigned long timeold=0;
// number of pulses per revolution
// based on your encoder disc
unsigned int pulsesperturn = 12;

void counter()
{
   //Update count
   pulses++;
}
int pos =0;
Servo myservo;
MPU6050 mpu6050(Wire);
void setup() 
{
  Serial.begin(9600);
  pinMode(encoder_pin, INPUT);
   //Interrupt 0 is digital pin 2
   //Triggers on Falling Edge (change from HIGH to LOW)
   attachInterrupt(0, counter, FALLING);
   // Initialize
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  myservo.attach(11);         // servo motor output pin
  pinMode(5,INPUT);          // clutch button input pin
  pinMode(3,INPUT);          // first gear switch
  pinMode(4,INPUT);          // reverse gear switch
 pinMode(6,INPUT);          // Ignition switch
 pinMode(9,OUTPUT);         // DC motor and 100 ohm resistor
 pinMode(10,OUTPUT);         // with resistor and  220 ohm resistor
 pinMode(13,OUTPUT);
}

void loop() {

  if(digitalRead(6)==1)          //if ignition is ON
  {
    {
      digitalWrite(13,HIGH);    // LED will glow when ignited
     mpu6050.update();
  Serial.print("\nangleY = ");
  Serial.print(mpu6050.getAngleY());
   if (millis() - timeold >= 100) {
      //Don't process interrupts during calculations
      detachInterrupt(0);
      rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
      timeold = millis();
      pulses = 0;
      Serial.print("\nRPM = ");
      Serial.println(rpm,DEC);
      //Restart the interrupt processing
      attachInterrupt(0, counter, FALLING);
      // put your main code here, to run repeatedly:
  }
    digitalWrite(10,HIGH); 
    digitalWrite(9,HIGH); 
  }
  if(digitalRead(5)==0)          // if clutch is not engaged
  {
  if((mpu6050.getAngleY()>=(-20))&&(digitalRead(3)==1))    // Inclined gyroscope angle=-15 and declined gyroscope angle is +5
  {
    myservo.write(pos=40);  
   digitalWrite(10,HIGH); 
   digitalWrite(9,LOW);
    
  } 
     
  if((mpu6050.getAngleY()<(-20))&&(mpu6050.getAngleY()<5))  
  {
   digitalWrite(8,HIGH); 
    digitalWrite(9,HIGH); 
    myservo.write(pos=0);
  }    

 if((mpu6050.getAngleY()>=5)&&(digitalRead(4)==1))     //decline angle
   {
    myservo.write(pos=-60);
    digitalWrite(10,HIGH); 
    digitalWrite(9,LOW); 
  }
  }
   if(digitalRead(6)==0)     // else function for ignition and engine=0
  {
    digitalWrite(13,LOW);
     digitalWrite(10,LOW); 
  digitalWrite(9,LOW);
    Serial.println("\nThe engine is OFF");
  }
  }
}
