#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

Servo esc;
Servo ser;
int led = 13;

int command;

bool start = true;

const int BUFFER_SIZE = 5;
char buf[BUFFER_SIZE];

void setup(){

  pinMode(led, OUTPUT);

  Serial.begin(115200);
  ser.attach(9); //blue wire is servo
  ser.write(98);
  esc.attach(10); //white wire is motor

  
  digitalWrite(led, HIGH);

  /*
  esc.writeMicroseconds(1000);
  delay(4000);//mandatory delay waiting for motor to be ready'
  esc.writeMicroseconds(2000);
  delay(4000);
  esc.writeMicroseconds(1500);
  delay(5000);
  */
}

void loop() {

  while(Serial.available() > 0) {

    Serial.readBytes(buf, BUFFER_SIZE); //reading command from rasppi
    buf[4] = '\0';
    command = atoi(buf);

    if(command == 1500 && start){
      delay(3000);
      digitalWrite(led, LOW);
      start = false;
    }
    //Serial.println(command);
    
    if(!start){
      if(command <= 2000)
      {
        esc.writeMicroseconds(command);
      }
      else
      {
        ser.write(command - 2000);
      }
    }
  }
}

