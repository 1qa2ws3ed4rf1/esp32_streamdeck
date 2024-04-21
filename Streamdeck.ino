#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <Wire.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h>  //include

//lib:Fastled,ArduinoJson Board:esp32s2

#define TC_W_ADDR 0x70
#define TC_R_ADDR 0x71
#define buttonpin 6  //pins
#define a1 36        //pins
#define a2 37        //pins
#define a3 38        //pins
#define sound 35     //pins
#define x 33         //pins
#define y 34         //pins
#define EEPROM_SIZE 1024
#define NUM_LEDS 16  //define

int addr = 0;  //EEPROM addr
uint16_t sum = 0;   // checksum
StaticJsonDocument<500> jsonBuffer;
typedef struct quetran {
  int buttonpress[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // torch 1 - 16,i6 dev by user
  int adc[6] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };                      //  a1,a2,a3,sound,a1|x,a2|y
  
} quetran;                                                                 //queuetranfrom
typedef struct jsonio
{
    /* data */
    int rgb[48] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} //r,g,b,r,g,b
    int c[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

}jsonio;

bool EEPROM_check = false;                                                // EEPROM useable
QueueHandle_t queue;
QueueHandle_t uartin;
CRGB leds[NUM_LEDS];                                                    //var

void i2c_init_1(void *p) {
  Wire.beginTransmission(TC_W_ADDR);
  for (int i = 0; i <= 7; i++) {
    Wire.write(EEPROM.read(i));
    sum = sum + EEPROM.read(i);
  }
  Wire.write(sum);
  Wire.endTransmission(true);
  uint16_t ack = Wire.requestFrom(TC_R_ADDR, 1);
  if (ack == 0x53) {
    Serial.println("I2C1 Data transfor complete!\n");
    for (int i = 0; i <= 7; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
  } else {
    Serial.println("I2C1 Data transfor failed!");
  }
  vTaskDelete(NULL);
}

void i2c_init_2(void *p) {
  Wire1.beginTransmission(TC_W_ADDR);
  for (int i = 7; i <= 15; i++) {
    Wire1.write(EEPROM.read(i));
    sum = sum + EEPROM.read(i);
  }
  Wire1.write(sum);
  Wire1.endTransmission(true);
  uint16_t ack = Wire1.requestFrom(TC_R_ADDR, 1);
  if (ack == 0x53) {
    Serial.println("I2C2 Data transfor complete!\n");
    for (int i = 7; i <= 15; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
  } else {
    Serial.println("I2C2 Data transfor failed!\n");
  }
  vTaskDelete(NULL);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.printf("Starting system init,EEPROM init \n");
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("failed to initialise EEPROM! Config will not can be write or read!\n");
  } else {
    EEPROM_check = true;
    Serial.printf("EEPROM inited!\n");
  }
  Serial.printf("I2C initing...\n");
  if (Wire.begin()) {  // I2C1 init
    if (!(Wire.setPins(14, 15) && Wire.setClock(9600))) {
      Serial.printf("I2C 1 Init Failed! Reseting....");
      ESP.restart();
    }
  } else {
    Serial.printf("I2C 1 Init Failed! Reseting....");
    ESP.restart();
  }
  if (Wire1.begin()) {  // I2C2 init
    if (!(Wire1.setPins(14, 15) && Wire1.setClock(9600))) {
      Serial.printf("I2C 2 Init Failed! Reseting....");
      ESP.restart();
    }
  } else {
    Serial.printf("I2C 2 Init Failed! Reseting....");
    ESP.restart();
  }
  if (EEPROM_check && EEPROM.read(0) != 0) {
    xTaskCreate(i2c_init_1,
                "i2c1",
                2048,
                NULL,
                5,
                NULL);
    xTaskCreate(i2c_init_2,
                "i2c2",
                2048,
                NULL,
                5,
                NULL);
  }
  Serial.printf("I2Cs Inited! Initing RGB....\n");
  FastLED.addLeds<WS2812, 7, GRB>(leds, NUM_LEDS);  // leds init
  FastLED.setBrightness(128);
  fill_gradient_RGB(leds, 0, CRGB(68, 32, 121), 15, CRGB(44, 121, 32));  //leds rgb show
  FastLED.show();
  delay(1000);
  for (int i = 0; !(i == 15); i++) {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(100);
  }  // clear leds
  Serial.printf("RGB Inited! Initing Others....\n");
  uartin = xQueueCreate(1, sizeof(String));
  queue = xQueueCreate(1, sizeof(quetran));
  xTaskCreate(main_uart_hand,
              "uart",
              2048,
              NULL,
              5,
              NULL);
  
}

jsonio json(String json){ //TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

}

void main_input_hand(void *p){
  while(1){
    quetran thisout;
    uint8_t torch = Wire.requestFrom(TC_R_ADDR, 1,false);
    uint8_t torch_r = Wire.requestFrom(TC_R_ADDR, 1,true); //unuse
    uint8_t torch1 = Wire1.requestFrom(TC_R_ADDR, 1,false);
    torch_r = Wire.requestFrom(TC_R_ADDR, 1,true); // unuse
    for (int i = 0;i<=7;i++){
        thisout.buttonpress[i] = ((torch) >> (i)) & 0x01
    }
    for (int i = 8;i<=15;i++){
        thisout.buttonpress[i] = ((torch1) >> (i)) & 0x01
    }
    thisout.adc[0] = analogRead(a1);
    thisout.adc[1] = analogRead(a2);
    thisout.adc[2] = analogRead(a3);
    thisout.adc[3] = analogRead(sound);
    thisout.adc[4] = analogRead(x);
    thisout.adc[5] = analogRead(y);
    xQueueSend(queue, (void *)&thisout, 0);
  }
}
void main_rgb_hand(void *p){
  while(1){

  }
}
void scan_i2c_devices(void *p){ // detect another i2c devices

  vTaskDelete(NULL);
}
void main_uart_hand(void *p) {

  while (1) {
    String in;
    if (Serial.available() > 0) {
      in = Serial.readStringUntil("\n")
      xQueueSend(uartin, (void *)&in, 0);
    }
    quetran beout;
    if (xQueueReceive(queue, (void *)&beout, 0) == errQUEUE_EMPTY) {
    } else {
      JsonArray button = jsonBuffer.createNestedArray("button");
      for (int i = 0; i <= 15; i++) {
        button.add(beout.buttonpress[i]);
      }
      JsonArray adc = jsonBuffer.createNestedArray("adc");
      for (int i = 0; i <= 5; i++) {
        adc.add(beout.adc[i]);
      }
      String uartoutput;
      serializeJson(jsonBuffer, uartoutput);
      Serial.println(uartoutput);
    }
  }
}

void loop() {}  // Use freertos....
