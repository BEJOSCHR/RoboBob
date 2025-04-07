
#include <Math.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ------------------------------------------------------------------------------------------------------

//Disable flag
bool disable_all = false;

//Joystick
#define JS_BUTTON 6 //Joystick - button - digital
#define JS_X A0 //Joystick - x-axis - analog
#define JS_Y A1 //Joystick - y-axis - analog
#define JS_X_CENTER 503
#define JS_Y_CENTER 513
int js_x = 0; //The offset from the center pos
int js_y = 0; //The offset from the center pos
bool js_button = false;
bool js_button_toggle = false;

//Radio
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

//LEDs
#define L_MODE 9
#define L_STRENGTH_1 10
#define L_STRENGTH_2 11
#define L_STRENGTH_3 12

//Button
#define B_PIN 22
bool b_state = false;
bool b_state_toggle = false;
int b_strength_counter = 1; //1, 2 or 3

//General
#define UPDATE_INTERVALL 50 //The intervall in ms in which we scan and send the new data
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

// ------------------------------------------------------------------------------------------------------
//https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/

void setup() {

  disable_all = false;

  Serial.begin(9600);
  Serial.println("Starting ROBO_BOB_CONTROLLER - Juhu");
  Serial.print("Update intervall: "); Serial.println(UPDATE_INTERVALL);

  //Init joystick
  pinMode(JS_BUTTON, INPUT);
  digitalWrite(JS_BUTTON, HIGH);

  //Init radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX); //Set power usage, min or max or something between
  radio.stopListening(); //Sender - not receiving

  //LEDs
  pinMode(L_MODE, OUTPUT);
  digitalWrite(L_MODE, LOW);
  pinMode(L_STRENGTH_1, OUTPUT);
  digitalWrite(L_STRENGTH_1, LOW);
  pinMode(L_STRENGTH_2, OUTPUT);
  digitalWrite(L_STRENGTH_2, LOW);
  pinMode(L_STRENGTH_3, OUTPUT);
  digitalWrite(L_STRENGTH_3, LOW);

  //Button
  pinMode(B_PIN, INPUT);
  
  //Internal LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("Startup complete!");
  
}

// ------------------------------------------------------------------------------------------------------

void loop() {

  updateButtonState(); //Enables / Disables everything so keep checking
  if(disable_all == false) {
    updateJoystickData();
    Transmission_Data data = createTransmittionData(js_button_toggle, b_strength_counter, js_x, js_y);
    sendData(data); 
    updateVisuals();
    delay(UPDATE_INTERVALL);
  }else {
    updateVisuals(); //Keep updating visuals
    delay(UPDATE_INTERVALL); //Always on a delay
  }
  
}

// ------------------------------------------------------------------------------------------------------

//Reads the values from the joystick and updates the representing vars js_x, js_y, js_button
void updateJoystickData() {
  bool tmp_js_button = (digitalRead(JS_BUTTON)-1)*(-1); //Invert the LOW (0) signal on press to a true (1) signal in variable
  if(tmp_js_button != js_button) {
    //Button state change
    js_button_action(tmp_js_button);
  }
  js_button = tmp_js_button;
  js_x = removeZeroJiggle(analogRead(JS_X)-JS_X_CENTER);
  js_y = removeZeroJiggle(analogRead(JS_Y)-JS_Y_CENTER);
  if(DEBUG == true) {
    Serial.print("> Joystick read: x=");
    Serial.print(js_x);
    Serial.print(" | y=");
    Serial.print(js_y);
    Serial.print(" | strength=");
    Serial.print(b_strength_counter);
    Serial.print(" | toggle=");
    Serial.print(js_button_toggle);
    Serial.println(""); 
  }
}

//Executes the given button action depending on the new button state
void js_button_action(bool newState) {
  if(newState == 1) {
    //Now pressed
  }else {
    //Now released
    js_button_toggle = !js_button_toggle;
  }
}

//Remove jiggle from value around 0
int removeZeroJiggle(int value) {
  if(value >= -2 && value <= 2) {
    return 0;
  }
  return value;
}

// ------------------------------------------------------------------------------------------------------

long time_counter = 0;
//Reads the values from the external button and updates the representing vars
void updateButtonState() {

  bool newState = digitalRead(B_PIN);
  
  if(newState != b_state) {
    //Some change of state
    time_counter = 0; //Reset on change
    if(disable_all == false) {
      //If not disabled do normal action
      if(newState == true) {
        //Now pressed
        b_state_toggle = true;
      }else {
        //Now released
        b_state_toggle = false;

        //Do toggle action here
        b_strength_counter = b_strength_counter+1;
        if(b_strength_counter > 3) {
          b_strength_counter = 1;
        }
        
      } 
    }
  }else {
    //Same state
    if(newState == true && time_counter >= 50) { //50 is a try and error delay for a long button press in correlation to the delay during disable time
      //Button is pressed for some while
      if(disable_all == true) {
        //Only reduce once to counter button update on release
        b_strength_counter = b_strength_counter-1;
      }
      disable_all = !disable_all;
      time_counter = 0; //Reset after update
      Serial.print("New active state: "); Serial.println(disable_all);
    }else {
      time_counter++;
    }
  }
  b_state = newState;
  
}

// ------------------------------------------------------------------------------------------------------

//Packs the given data into the data struct, size limit: 32 bytes!
Transmission_Data createTransmittionData(boolean button_toggle, int strength_level, int x, int y) {
  Transmission_Data data; //New struct
  data.t = button_toggle;
  data.s = strength_level;
  data.x = x*(-1); //x is inverted from the joystick
  data.y = y;
  return data;
}

//Send the given Transmission_Data struct over the radio connection, size limit: 32 bytes!
void sendData(Transmission_Data data) {
  digitalWrite(LED_BUILTIN, HIGH);
  radio.write(&data, sizeof(Transmission_Data));
  delay(20);
  digitalWrite(LED_BUILTIN, LOW);
}

// ------------------------------------------------------------------------------------------------------

//Debug in console and set display led states
void updateVisuals() {
  if(DEBUG == true) {
    if(disable_all == false) {
      Serial.print("Data send [Size = "); Serial.print(sizeof(Transmission_Data)); Serial.println(" bytes]"); 
    }else {
      Serial.println("Actions disabled!"); 
    }
  }
  //MODE
  if(disable_all == false) {
    if(js_button_toggle == true) {
      digitalWrite(L_MODE, HIGH);
    }else {
      digitalWrite(L_MODE, LOW);
    }
    //STRENGTH
    if(b_strength_counter == 1) {
      digitalWrite(L_STRENGTH_1, HIGH);
      digitalWrite(L_STRENGTH_2, LOW);
      digitalWrite(L_STRENGTH_3, LOW);
    }else if(b_strength_counter == 2) {
      digitalWrite(L_STRENGTH_1, HIGH);
      digitalWrite(L_STRENGTH_2, HIGH);
      digitalWrite(L_STRENGTH_3, LOW);
    }else if(b_strength_counter == 3) {
      digitalWrite(L_STRENGTH_1, HIGH);
      digitalWrite(L_STRENGTH_2, HIGH);
      digitalWrite(L_STRENGTH_3, HIGH);
    }else {
      digitalWrite(L_STRENGTH_1, LOW);
      digitalWrite(L_STRENGTH_2, LOW);
      digitalWrite(L_STRENGTH_3, LOW);
    }
  }else {
    //Show deactive state
    digitalWrite(L_MODE, LOW);
    digitalWrite(L_STRENGTH_1, LOW);
    digitalWrite(L_STRENGTH_2, LOW);
    digitalWrite(L_STRENGTH_3, LOW);
  }
}


