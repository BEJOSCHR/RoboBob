
#include <Math.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ------------------------------------------------------------------------------------------------------

#define M_FL 1 //FrontLeft
#define M_FR 2 //FrontRight
#define M_BL 3 //BackLeft
#define M_BR 4 //BackRight

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

//Motors
// > FL
#define M_FL_IN1_D 28 //Motor - FrontLeft - Richtung_1 - Digital
#define M_FL_IN2_D 26 //Motor - FrontLeft - Richtung_2 - Digital
#define M_FL_SPEED_A 4 //Motor - FrontLeft - Speed - Analog
int m_fl_speed = 0; //Motor - FrontLeft - speed - int
int m_fl_direction = BRAKE; //Motor - FrontLeft - direction - int 
// > FR
#define M_FR_IN1_D 30 //Motor - FrontRight - Richtung_1 - Digital
#define M_FR_IN2_D 32 //Motor - FrontRight - Richtung_2 - Digital
#define M_FR_SPEED_A 6 //Motor - FrontRight - Speed - Analog
int m_fr_speed = 0; //Motor - FrontRight - speed - int
int m_fr_direction = BRAKE; //Motor - FrontRight - direction - int 
// > BL
#define M_BL_IN1_D 24 //Motor - BackLeft - Richtung_1 - Digital
#define M_BL_IN2_D 22 //Motor - BackLeft - Richtung_2 - Digital
#define M_BL_SPEED_A 5 //Motor - BackLeft - Speed - Analog
int m_bl_speed = 0; //Motor - BackLeft - speed - int
int m_bl_direction = BRAKE; //Motor - BackLeft - direction - int 
// > BR
#define M_BR_IN1_D 34 //Motor - BackRight - Richtung_1 - Digital
#define M_BR_IN2_D 36 //Motor - BackRight - Richtung_2 - Digital
#define M_BR_SPEED_A 7 //Motor - BackRight - Speed - Analog
int m_br_speed = 0; //Motor - BackRight - speed - int
int m_br_direction = BRAKE; //Motor - BackRight - direction - int 

//Speed
//#define MIN_SPEED 74 // So speeds given from 1 = 128 to 181 = 255 (Range: 1 to 181 aka 75 to 255)
#define MIN_SPEED 90 // So speeds given from 1 = 128 to 165 = 255 (Range: 1 to 165 aka 91 to 255)
int strength_level = 1; //1, 2 or 3 - low, medium and high
int MAX_POWER = 130; //init on low strength level
int MAX_SPEED = MAX_POWER-MIN_SPEED;
const int MAX_SPEED_UP = 503;
const int MAX_SPEED_DOWN = 517;
int MAX_TURN = MAX_POWER-MIN_SPEED;
const int MAX_TURN_LEFT = 505;
const int MAX_TURN_RIGHT = 511;

//Break
#define BRAKE_REVERSE_DELAY 20 //When breaking give a short impulse in the opposite direction to stop the motor
int brake_delay = 0;

//Radio
RF24 radio(2, 3); // CE, CSN
const byte address[6] = "00001";
#define MAX_NONE_RECEIVES 5 
int noneReceives = MAX_NONE_RECEIVES; //Set to max, so it gives a sound on connect

//Sound
#define SOUND_PIN 10

#define UPDATE_INTERVALL 70 //The intervall in ms in which we scan and send the new data
#define DEBUG false 

// ------------------------------------------------------------------------------------------------------

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
//Needs to be synchronised so the same as in the receiver side code
struct Transmission_Data {
  bool t = false;   //Button toggle
  int s = 1;        //Strength level
  int x = 0;        //x-axis
  int y = 0;        //y-axis
};

Transmission_Data last_data;

// ------------------------------------------------------------------------------------------------------

void setup() {

  Serial.begin(9600);
  Serial.println("Starting ROBO_BOB - Jippi");

  // M_FL
  pinMode(M_FL_IN1_D, OUTPUT);
  pinMode(M_FL_IN2_D, OUTPUT);
  pinMode(M_FL_SPEED_A, OUTPUT);
  digitalWrite(M_FL_IN1_D, LOW);
  digitalWrite(M_FL_IN2_D, LOW);
  // M_FR
  pinMode(M_FR_IN1_D, OUTPUT);
  pinMode(M_FR_IN2_D, OUTPUT);
  pinMode(M_FR_SPEED_A, OUTPUT);
  digitalWrite(M_FR_IN1_D, LOW);
  digitalWrite(M_FR_IN2_D, LOW);
  // M_BL
  pinMode(M_BL_IN1_D, OUTPUT);
  pinMode(M_BL_IN2_D, OUTPUT);
  pinMode(M_BL_SPEED_A, OUTPUT);
  digitalWrite(M_BL_IN1_D, LOW);
  digitalWrite(M_BL_IN2_D, LOW);
  // M_BR
  pinMode(M_BR_IN1_D, OUTPUT);
  pinMode(M_BR_IN2_D, OUTPUT);
  pinMode(M_BR_SPEED_A, OUTPUT);
  digitalWrite(M_BR_IN1_D, LOW);
  digitalWrite(M_BR_IN2_D, LOW);

  //Radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  //Sound
  pinMode(SOUND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, LOW);

  //Internal LED
  pinMode(LED_BUILTIN, OUTPUT);

  bob_brake(false);
  Serial.println("Startup complete!");
  
}

// ------------------------------------------------------------------------------------------------------
//https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/

void loop() {

  // Check whether there is data to be received
  if(radio.available()) {

    if(noneReceives >= MAX_NONE_RECEIVES) {
      //First connect - 1x SOUND
      digitalWrite(SOUND_PIN, HIGH);
      delay(30);
      digitalWrite(SOUND_PIN, LOW);
    }
    noneReceives = 0;
        
    Transmission_Data data = receiveData();

    //Do stuff
    int strength_level = data.s;
    updateMaxLevel(strength_level);
    int speed = data.x;
    int realMotorSpeed = adjustSpeed(speed); //Always a positiv speed
    int turn = data.y; // + = Links | - = Rechts
    int realMotorTurn = adjustTurn(turn); //Always a positiv turnSpeed

    if(DEBUG == true) { 
      Serial.print(">> RealSpeed: ");
      Serial.print(realMotorSpeed);
      Serial.print(" | RealTurn: ");
      Serial.print(realMotorTurn); 
      Serial.println(""); 
    }

    if(speed == 0) {
      //Turn on the spot
      if(turn > 0) {
        //To the left
        bob_turnLeft(realMotorTurn, realMotorTurn*1, true);
        if(DEBUG == true) { Serial.println(">>> Turn on the spot to the left"); }
      }else if(turn < 0) {
        //To the right
        bob_turnRight(realMotorTurn, realMotorTurn*1, true);
        if(DEBUG == true) { Serial.println(">>> Turn on the spot to the right"); }
      }else {
        //Just stay still
        bob_brake(true);
        //bob_brake(false);
        if(DEBUG == true) { Serial.println(">>> Braking!!!"); }
      }
    }else {
      //Move
      if(turn == 0) {
        //Move straight
        if(speed > 0) {
          //Forward
          bob_forward(realMotorSpeed, data.t);
            if(DEBUG == true) { Serial.println(">>> Just Forward"); }
        }else {
          //Backwards
          bob_backward(realMotorSpeed, data.t);
            if(DEBUG == true) { Serial.println(">>> Just Backward"); }
        }
      }else {
        //Move with turn
        if(speed > 0) {
          //Forward
          if(turn > 0) {
            //To the left
            bob_turnLeft(realMotorSpeed+realMotorTurn, realMotorTurn, true);
            if(DEBUG == true) { Serial.println(">>> Forward to the left"); }
          }else if(turn < 0) {
            //To the right
            bob_turnRight(realMotorSpeed+realMotorTurn, realMotorTurn, true);
            if(DEBUG == true) { Serial.println(">>> Forward to the right"); }
          }
        }else {
          //Backwards
          if(turn > 0) {
            //To the left
            bob_turnLeft(realMotorSpeed+realMotorTurn, MAX_TURN_LEFT+realMotorTurn, true); //Increase low wheels speed so they go "backwards", but keep driving "forwards"
            if(DEBUG == true) { Serial.println(">>> Backwards to the left"); }
          }else if(turn < 0) {
            //To the right
            bob_turnRight(realMotorSpeed+realMotorTurn, MAX_TURN_RIGHT+realMotorTurn, true); //Increase low wheels speed so they go "backwards", but keep driving "forwards"
            if(DEBUG == true) { Serial.println(">>> Backwards to the right"); }
          }
        }
      }
    }

    last_data = data;    
    if(DEBUG == true) {
      Serial.print("> Received: x=");
      Serial.print(data.x);
      Serial.print(" | y=");
      Serial.print(data.y);
      Serial.print(" | toggle=");
      Serial.print(data.t);
      Serial.println(""); 
    }
  }else {
    if(noneReceives > MAX_NONE_RECEIVES) {
      //Blink
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
    }else{
      noneReceives += 1;
      if(noneReceives == MAX_NONE_RECEIVES) {
        //No connection - BRAKE!!!
        bob_brake(true);
        //First disconnect - 3x SOUND
        for(int i = 3 ; i > 0 ; i--) {
          digitalWrite(SOUND_PIN, HIGH);
          delay(30);
          digitalWrite(SOUND_PIN, LOW);
          delay(60);
        }
      }
    }
    if(DEBUG == true) {
      Serial.println("Nothing received");
    }
  }

  delay(UPDATE_INTERVALL);
  
}

// ------------------------------------------------------------------------------------------------------

//Update the speed values by a given strength level
void updateMaxLevel(int new_strength_level) {
  strength_level = new_strength_level;
  if(strength_level == 3) {
    //FAST
    MAX_POWER = 255;
  }else if(strength_level == 2) {
    //MEDIUM
    MAX_POWER = 200;
  }else {
    //SLOW
    MAX_POWER = 130;
  }
  MAX_SPEED = MAX_POWER-MIN_SPEED;
  MAX_TURN = MAX_POWER-MIN_SPEED;
}

//Returns the real motorSpeed (always positiv) from the given transmitted speed (from - over 0 to +)
int adjustSpeed(int speed) {
  int absSpeed = abs(speed);
  if(speed > 0) {
    //Forward
    double percentage = ((double) absSpeed) / ((double) MAX_SPEED_UP);
    double flattened = flatSpeedFunction(percentage);
    int realSpeed = (int) (MAX_SPEED*flattened);
    return realSpeed;
  }else if(speed < 0) {
    //Backward
    double percentage = ((double) absSpeed) / ((double) MAX_SPEED_DOWN);
    double flattened = flatSpeedFunction(percentage);
    int realSpeed = (int) (MAX_SPEED*flattened);
    return realSpeed;
  }else {
    return 0;
  }
}

double flatSpeedFunction(double x) {
  return 1.0*pow(x, 9);
}

//Returns the real motorTurnSpeed (always positiv) from the given transmitted speed (from - over 0 to +)
int adjustTurn(int speed) {
  int absTurn = abs(speed);
  if(speed > 0) { 
    //Left
    double percentage = ((double) absTurn) / ((double) MAX_TURN_LEFT);
    double flattened = flatTurnFunction(percentage);
    int realTurn = (int) (MAX_TURN*flattened);
    return realTurn;
  }else if(speed < 0) {
    //Right
    double percentage = ((double) absTurn) / ((double) MAX_TURN_RIGHT);
    double flattened = flatTurnFunction(percentage);
    int realTurn = (int) (MAX_TURN*flattened);
    return realTurn;
  }else {
    return 0;
  }
}

double flatTurnFunction(double x) {
  return 1.0*pow(x, 9);
}

// ------------------------------------------------------------------------------------------------------

//Change the speed and the direction of all motors to BRAKE
void bob_brake(bool breakImpulse) {
  if(breakImpulse == true) {
    brake_delay = BRAKE_REVERSE_DELAY;
  }
  motorAction(M_FL, BRAKE, 0);
  motorAction(M_FR, BRAKE, 0);
  motorAction(M_BL, BRAKE, 0);
  motorAction(M_BR, BRAKE, 0);
  brake_delay = 0;
}

//Change the speed and the direction of all motors to drive FORWARD
//If all=false we only use the front wheels
void bob_forward(int speed, bool all) {
  motorAction(M_FL, FORWARD, speed);
  motorAction(M_FR, FORWARD, speed);
  if(all == true) {
    motorAction(M_BL, FORWARD, speed);
    motorAction(M_BR, FORWARD, speed); 
  }else {
    motorAction(M_BL, BRAKE, 0);
    motorAction(M_BR, BRAKE, 0);
  }
}

//Change the speed and the direction of all motors to drive BACKWARD
//If all=false we only use the front wheels
void bob_backward(int speed, bool all) {
  motorAction(M_FL, BACKWARD, speed);
  motorAction(M_FR, BACKWARD, speed);
  if(all == true) {
    motorAction(M_BL, BACKWARD, speed);
    motorAction(M_BR, BACKWARD, speed);
  }else {
    motorAction(M_BL, BRAKE, 0);
    motorAction(M_BR, BRAKE, 0);
  }
}

//Change the speed and the direction of all motors to turn LEFT
void bob_turnLeft(int speed, int turnSpeed, bool forward) {
  int lowSpeed = speed-turnSpeed;
  if(turnSpeed >= 2*speed) { //Dont go faster in the other direction then the main speed
    lowSpeed = speed*(-1);
  }
  if(forward == true) {
    //Forward turning
    if(lowSpeed > 0) {
      motorAction(M_FL, FORWARD, lowSpeed);
      motorAction(M_FR, FORWARD, speed);
      motorAction(M_BL, FORWARD, lowSpeed);
      motorAction(M_BR, FORWARD, speed);
    }else if(lowSpeed < 0) {
      motorAction(M_FL, BACKWARD, (lowSpeed*-1) );
      motorAction(M_FR, FORWARD, speed);
      motorAction(M_BL, BACKWARD, (lowSpeed*-1) );
      motorAction(M_BR, FORWARD, speed);
    }else {
      motorAction(M_FL, BRAKE, 0);
      motorAction(M_FR, FORWARD, speed);
      motorAction(M_BL, BRAKE, 0);
      motorAction(M_BR, FORWARD, speed); 
    }
  }else {
    //Backward turning
    if(lowSpeed > 0) {
      motorAction(M_FL, BACKWARD, lowSpeed);
      motorAction(M_FR, BACKWARD, speed);
      motorAction(M_BL, BACKWARD, lowSpeed);
      motorAction(M_BR, BACKWARD, speed);
    }else if(lowSpeed < 0) {
      motorAction(M_FL, FORWARD, (lowSpeed*-1) );
      motorAction(M_FR, BACKWARD, speed);
      motorAction(M_BL, FORWARD, (lowSpeed*-1) );
      motorAction(M_BR, BACKWARD, speed);
    }else {
      motorAction(M_FL, BRAKE, 0);
      motorAction(M_FR, BACKWARD, speed);
      motorAction(M_BL, BRAKE, 0);
      motorAction(M_BR, BACKWARD, speed); 
    }
  }
}

//Change the speed and the direction of all motors to turn RIGHT
void bob_turnRight(int speed, int turnSpeed, bool forward) {
  int lowSpeed = speed-turnSpeed;
  if(turnSpeed >= 2*speed) { //Dont go faster in the other direction then the main speed
    lowSpeed = speed*(-1);
  }
  if(forward == true) {
    //Forward turning
    if(lowSpeed > 0) {
      motorAction(M_FL, FORWARD, speed);
      motorAction(M_FR, FORWARD, lowSpeed);
      motorAction(M_BL, FORWARD, speed);
      motorAction(M_BR, FORWARD, lowSpeed);
    }else if(lowSpeed < 0) {
      motorAction(M_FL, FORWARD, speed);
      motorAction(M_FR, BACKWARD, (lowSpeed*-1) );
      motorAction(M_BL, FORWARD, speed);
      motorAction(M_BR, BACKWARD, (lowSpeed*-1) );
    }else {
      motorAction(M_FL, FORWARD, speed);
      motorAction(M_FR, BRAKE, 0);
      motorAction(M_BL, FORWARD, speed);
      motorAction(M_BR, BRAKE, 0); 
    }
  }else {
    //Backward turning
    if(lowSpeed > 0) {
      motorAction(M_FL, BACKWARD, speed);
      motorAction(M_FR, BACKWARD, lowSpeed);
      motorAction(M_BL, BACKWARD, speed);
      motorAction(M_BR, BACKWARD, lowSpeed);
    }else if(lowSpeed < 0) {
      motorAction(M_FL, BACKWARD, speed);
      motorAction(M_FR, FORWARD, (lowSpeed*-1) );
      motorAction(M_BL, BACKWARD, speed);
      motorAction(M_BR, FORWARD, (lowSpeed*-1) );
    }else {
      motorAction(M_FL, BACKWARD, speed);
      motorAction(M_FR, BRAKE, 0);
      motorAction(M_BL, BACKWARD, speed);
      motorAction(M_BR, BRAKE, 0); 
    }
  }
}

//Change the speed and the direction of the given motor 
//Uses enmus motor=M_FL,M_FR,M_BL,M_BR and command=FORWARD,BACKWARD,BRAKE
void motorAction(int motor, int command, int speed) {
  int realSpeed = 0;
  if(speed > (255-MIN_SPEED) ) {
    speed = (255-MIN_SPEED);
  }
  if(speed > 0) {
    realSpeed = MIN_SPEED+speed;
  }
  switch(motor) {
    case M_FL:
      motorDirection(M_FL_IN1_D, M_FL_IN2_D, command, m_fl_direction);
      analogWrite(M_FL_SPEED_A, realSpeed);
      m_fl_direction = command;
      m_fl_speed = speed;
      break;
    case M_FR:
      motorDirection(M_FR_IN1_D, M_FR_IN2_D, command, m_fr_direction);
      analogWrite(M_FR_SPEED_A, realSpeed);
      m_fr_direction = command;
      m_fr_speed = speed;
      break;
    case M_BL:
      motorDirection(M_BL_IN1_D, M_BL_IN2_D, command, m_bl_direction);
      analogWrite(M_BL_SPEED_A, realSpeed);
      m_bl_direction = command;
      m_bl_speed = speed;
      break;
    case M_BR:
      motorDirection(M_BR_IN1_D, M_BR_IN2_D, command, m_br_direction);
      analogWrite(M_BR_SPEED_A, realSpeed);
      m_br_direction = command;
      m_br_speed = speed;
      break;
  }
}

//Change the direction on the given motor pins 
//Uses enmus command=FORWARD,BACKWARD,BRAKE
void motorDirection(int pin_in1, int pin_in2, int command, int last_command) {
  switch(command) {
    case FORWARD:
      digitalWrite(pin_in1, LOW);
      digitalWrite(pin_in2, HIGH);
      break;
    case BACKWARD:
      digitalWrite(pin_in1, HIGH);
      digitalWrite(pin_in2, LOW);
      break;
    case BRAKE:
      //If a delay is set, Give a short imulse with previous set speed int the opposite direction to enhance breaking power for the given delay time
      if(brake_delay > 0) {
        if(last_command == FORWARD) {
          digitalWrite(pin_in1, HIGH);
          digitalWrite(pin_in2, LOW);
          delay(brake_delay);
        }else if(last_command == BACKWARD) {
          digitalWrite(pin_in1, LOW);
          digitalWrite(pin_in2, HIGH);
          delay(brake_delay);
        }
      }
      digitalWrite(pin_in1, LOW);
      digitalWrite(pin_in2, LOW);
      break;
  }
}

// ------------------------------------------------------------------------------------------------------

//Receive a given Transmission_Data struct over the radio connection, size limit: 32 bytes!
Transmission_Data receiveData() {
  Transmission_Data data;
  radio.read(&data, sizeof(Transmission_Data)); // Read the whole data and store it into the 'data' structure
  return data;
}

