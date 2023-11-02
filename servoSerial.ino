#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
int pinServo1 = 2;
int pinServo2 = 3;
int pinServo3 = 4;
int pinServo4 = 5;
int pinServo5 = 6;
int pulsoMin = 544;
int pulsoMax = 2444;

char option = 0;
void setup() {
  Serial.begin(9600);
  servo1.attach(pinServo1, pulsoMin, pulsoMax);
  servo2.attach(pinServo2, pulsoMin, pulsoMax);
  servo3.attach(pinServo3, pulsoMin, pulsoMax);
  servo4.attach(pinServo4, pulsoMin, pulsoMax);
  servo5.attach(pinServo5, pulsoMin, pulsoMax);
}

void loop() {

  int t=25;
  
  if(Serial.available()>0){
    option = Serial.read();
    Serial.println(option);
    if(option == 'a'){
        servo1.write(180);
        
    }
    if(option == 'b'){
        servo1.write(0);
        
    }

    if(option == 'c'){
        servo2.write(180);
        
    }
    if(option == 'd'){
        servo2.write(0);
        
    }

    if(option == 'e'){
        servo3.write(180);
        
    }
    if(option == 'f'){
        servo3.write(0);
        
    }

    if(option == 'g'){
        servo4.write(180);
        
    }
    if(option == 'h'){
        servo4.write(0);
        
    }

    if(option == 'i'){
        servo5.write(180);
        
    }
    if(option == 'j'){
        servo5.write(0);
        
    }
    
  }
  
}

