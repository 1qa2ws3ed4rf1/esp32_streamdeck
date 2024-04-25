#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "i2c.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h> //include

// lib:Fastled,ArduinoJson Board:esp32s2

#define TC_W_ADDR 0x70
#define TC_R_ADDR 0x71
#define buttonpin 6 // pins
#define a1 36       // pins
#define a2 37       // pins
#define a3 38       // pins
#define sound 35    // pins
#define x 33        // pins
#define y 34        // pins
#define EEPROM_SIZE 1024
#define NUM_LEDS 16 // define

int addr = 0;     // EEPROM addr
uint16_t sum = 0; // checksum
StaticJsonDocument<500> jsonBuffer;
typedef struct quetran
{
    int buttonpress[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // torch 1 - 16,i6 dev by user
    int adc[6] = {0, 0, 0, 0, 0, 0};                                        //  a1,a2,a3,sound,a1|x,a2|y

} quetran; // queuetranfrom
typedef struct jsonio
{
    /* data */
    int rgb[48] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // r,g,b,r,g,b
    int c[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

} jsonio;

bool EEPROM_check = false; // EEPROM useable
QueueHandle_t queue;
QueueHandle_t uartin;
CRGB leds[NUM_LEDS]; // var
i2c_cmd_handle_t cmd;
i2c_cmd_handle_t cmd1;

void i2c_init_1(void *p)
{
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC_W_ADDR, false);
    for (int i = 0; i <= 7; i++)
    {
        i2c_master_write_byte(cmd, EEPROM.read(i),  I2C_MASTER_ACK);
        sum = sum + EEPROM.read(i);
    }
    i2c_master_write_byte(cmd, sum,  I2C_MASTER_ACK);
    uint8_t re;
    i2c_master_read_byte(cmd, &re,  I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    if (re == 0x53)
    {
        Serial.println("I2C1 Data transfor complete!\n");
        for (int i = 0; i <= 7; i++)
        {
            EEPROM.write(i, 255);
        }
        EEPROM.commit();
    }
    else
    {
        Serial.printf("I2C1 Data transfor failed!");
        Serial.println(re);
    }
    vTaskDelete(NULL);
}

void i2c_init_2(void *p)
{
    cmd1 = i2c_cmd_link_create();
    i2c_master_start(cmd1);
    i2c_master_write_byte(cmd1, TC_W_ADDR, false);
    for (int i = 8; i <= 15; i++)
    {
        i2c_master_write_byte(cmd1, 0x01,  I2C_MASTER_ACK);
        sum = sum + EEPROM.read(i);
    }
    i2c_master_write_byte(cmd1, sum,  I2C_MASTER_ACK);
    uint8_t re;
    i2c_master_read_byte(cmd1, &re,  I2C_MASTER_ACK);
    i2c_master_stop(cmd1);
    i2c_master_cmd_begin(1, cmd1, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd1);
    if (re == 0x53)
    {
        Serial.println("I2C2 Data transfor complete!\n");
        for (int i = 8; i <= 15; i++)
        {
            EEPROM.write(i, 255);
        }
        EEPROM.commit();
    }
    else
    {
        Serial.printf("I2C2 Data transfor failed!");
        Serial.println(re);
    }
    vTaskDelete(NULL);
}

void i2c_init(int sda, int clk, int i2cwho)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = clk,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE};
        i2c_config.master.clk_speed = 9600;
    i2c_param_config(i2cwho, &i2c_config);
    i2c_driver_install(i2cwho, I2C_MODE_MASTER, 0, 0, 0);
}
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial1.begin(115200); // garbage throw at here... (commit error)
    Serial.printf("Starting system init,EEPROM init \n");
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("failed to initialise EEPROM! Config will not can be write or read!\n");
    }
    else
    {
        EEPROM_check = true;
        Serial.printf("EEPROM inited!\n");
    }
    Serial.printf("I2C initing...\n");
    i2c_init(13, 12, 0);
    i2c_init(14, 15, 1);
    if (EEPROM_check && (EEPROM.read(0) != 255))
    {
        Serial.println(EEPROM.read(0));
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
    xTaskCreate(i2c_init_2,
                    "i2c2",
                    2048,
                    NULL,
                    5,
                    NULL); // debug
    Serial.printf("I2Cs Inited! Initing RGB....\n");
    FastLED.addLeds<WS2812, 7, GRB>(leds, NUM_LEDS); // leds init
    FastLED.setBrightness(128);
    fill_gradient_RGB(leds, 0, CRGB(68, 32, 121), 15, CRGB(44, 121, 32)); // leds rgb show
    FastLED.show();
    delay(1000);
    for (int i = 0; !(i == 16); i++)
    {
        leds[i] = CRGB::Black;
        FastLED.show();
        delay(100);
    } // clear leds
    Serial.printf("RGB Inited! Initing Others....\n");
    uartin = xQueueCreate(1, sizeof(jsonio));
    queue = xQueueCreate(1, sizeof(quetran));
    xTaskCreate(main_input_hand,
                "input",
                2048,
                NULL,
                5,
                NULL);
    xTaskCreate(main_uart_hand,
                "uart",
                2048,
                NULL,
                5,
                NULL);
    xTaskCreate(main_c_rgb_hand,
                "rgb",
                1024,
                NULL,
                5,
                NULL);
    Serial.printf("FreeRTOS inited! System init complete!\n");
    Serial.printf("ST\n"); // start transfrom
}

jsonio json(String json)
{
    StaticJsonDocument<500> jsonBuffer;
    DeserializationError error = deserializeJson(jsonBuffer, const_cast<char *>(json.c_str()));
    if (error)
    {
        Serial.print(F("DeserializeJson failed: "));
        Serial.println(error.f_str());
    }
    jsonio thisoutput;
    for (int i = 0; i <= 47; i++)
    {
        thisoutput.rgb[i] = jsonBuffer["rgb"][i];
    }
    for (int i = 0; i <= 15; i++)
    {
        thisoutput.c[i] = jsonBuffer["c"][i];
    }
    return thisoutput;
}

void main_input_hand(void *p)
{
    while (1)
    {
        cmd = i2c_cmd_link_create();
        cmd1 = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_start(cmd1);
        i2c_master_write_byte(cmd, TC_R_ADDR,  I2C_MASTER_ACK);
        i2c_master_write_byte(cmd1, TC_R_ADDR,  I2C_MASTER_ACK);
        quetran thisout;
        uint8_t torch;
        uint8_t torch_r;
        uint8_t torch1;
        i2c_master_read_byte(cmd, &torch,  I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &torch_r,  I2C_MASTER_NACK);
        i2c_master_read_byte(cmd1, &torch1,  I2C_MASTER_ACK);
        i2c_master_read_byte(cmd1, &torch_r,  I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_stop(cmd1);
        i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        i2c_master_cmd_begin(1, cmd1, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd1);
        Serial.println(torch);
        for (int i = 0; i <= 7; i++)
        {
            thisout.buttonpress[i] = ((torch) >> (i)) & 0x01;
        }
        for (int i = 8; i <= 15; i++)
        {
            thisout.buttonpress[i] = ((torch1) >> (i)) & 0x01;
        }
        thisout.adc[0] = analogRead(35); // a1
        Serial.printf("%x",analogRead(35));
        thisout.adc[1] = analogRead(a2);
        thisout.adc[2] = analogRead(a3);
        thisout.adc[3] = analogRead(sound);
        thisout.adc[4] = analogRead(x);
        thisout.adc[5] = analogRead(y);
        xQueueSend(queue, (void *)&thisout, 0);
        vTaskDelay(30);
    }
}
void main_c_rgb_hand(void *p)
{
    while (1)
    {
        jsonio bein;
        if (xQueueReceive(uartin, (void *)&bein, 0) == errQUEUE_EMPTY)
        {
        }
        else
        {
            for (int i = 0; i <= 47; i = i + 3)
            {
                int nowled = 0;
                leds[nowled] = CRGB(bein.rgb[i], bein.rgb[i + 1], bein.rgb[i + 2]);
                nowled++;
            }
            int f = 0;
            for (int i = 0; i <= 15; i++)
            {
                if (bein.c[i] == 0)
                {
                    f++;
                }
            }
            if (f == 16)
            {
                if (EEPROM_check)
                {
                    for (int i = 0; i <= 15; i++)
                    {
                        EEPROM.write(i, bein.c[i]);
                    }
                }
            }
        }
    }
}

void main_uart_hand(void *p)
{

    while (1)
    {
        String in;
        if (Serial.available() > 0)
        {
            in = Serial.readStringUntil('\n');
            jsonio inj = json(in);
            xQueueSend(uartin, &inj, 0);
        }
        quetran beout;
        if (xQueueReceive(queue, (void *)&beout, 0) == errQUEUE_EMPTY)
        {
        }
        else
        {
            JsonArray button = jsonBuffer.createNestedArray("button");
            for (int i = 0; i <= 15; i++)
            {
                button.add(beout.buttonpress[i]);
            }
            JsonArray adc = jsonBuffer.createNestedArray("adc");
            for (int i = 0; i <= 5; i++)
            {
                adc.add(beout.adc[i]);
            }
            String uartoutput;
            serializeJson(jsonBuffer, uartoutput);
            Serial.println(uartoutput);
        }
    }
}

void loop() {} // Use freertos....
