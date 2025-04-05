
#include <Math.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ------------------------------------------------------------------------------------------------------

#define JS_BUTTON 6 //Joystick - button - digital
#define JS_X A0 //Joystick - x-axis - analog
#define JS_Y A1 //Joystick - y-axis - analog
#define JS_X_CENTER 503
#define JS_Y_CENTER 513
int js_x = 0; //The offset from the center pos
int js_y = 0; //The offset from the center pos
bool js_button = false;
bool js_button_toggle = false;


RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

#define UPDATE_INTERVALL 50 //The intervall in ms in which we scan and send the new data
#define DEBUG true 

// ------------------------------------------------------------------------------------------------------

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
//Needs to be synchronised so the same as in the receiver side code
struct Transmission_Data {
  bool t = false;   //Button toggle
  int x = 0;        //x-axis
  int y = 0;        //y-axis
};

// ------------------------------------------------------------------------------------------------------
//https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/

void setup() {

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

  //Internal LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Startup complete!");
  
}

// ------------------------------------------------------------------------------------------------------

void loop() {

  updateJoystickData();
  Transmission_Data data = createTransmittionData(js_button_toggle, js_x, js_y);
  sendData(data);
  updateDebug();
  delay(UPDATE_INTERVALL);
  
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
    Serial.print(" | button=");
    Serial.print(js_button);
    Serial.print(" | toggle=");
    Serial.print(js_button_toggle);
    Serial.println(""); 
  }
}

//Executes the given button action depending on the new button state
void js_button_action(bool newState) {
  if(newState == 1) {
    //Now pressed
    js_button_toggle = !js_button_toggle;
    if(DEBUG == true) {
      Serial.println("Pressed");
    }
  }else {
    //Now released

    if(DEBUG == true) {
      Serial.println("Released");
    }
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

//Packs the given data into the data struct, size limit: 32 bytes!
Transmission_Data createTransmittionData(boolean button_toggle, int x, int y) {
  Transmission_Data data; //New struct
  data.t = button_toggle;
  data.x = x*(-1); //x is inverted from the joystick
  data.y = y;
  return data;
}

//Send the given Transmission_Data struct over the radio connection, size limit: 32 bytes!
void sendData(Transmission_Data data) {
  radio.write(&data, sizeof(Transmission_Data));
  //delay(500);
}

// ------------------------------------------------------------------------------------------------------

void updateDebug() {
  Serial.print("Data send [Size = "); Serial.print(sizeof(Transmission_Data)); Serial.println(" bytes]");
  if(js_button_toggle == true) {
    digitalWrite(LED_BUILTIN, HIGH);
  }else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}


