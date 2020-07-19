/**************************************************************************
 *   LedCube 3x3 - mit MQTT control                                       *
 * ************************************************************************/
#include <Arduino.h>

#include <LittleFS.h>              //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

// needed for OTA & MQTT
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include "init.h"
#include <SPI.h>

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40]  = "192.168.178.21";
char mqtt_port[6]     = "1883";

const char* ssid =        "yourSSID";
const char* password =    "youPassword";


//Project name
char WifiApName[40]     = "Wifi_LedCube";
char MQTTClientName[40] = "LedCube";
const char OtaName[40]  = "LedCube_OTA";

WiFiClient espClient;
PubSubClient client(espClient);

// MQTT 
const char* pup_ldrlight                = "/hhome/ledCube/ldrlight";
const char* pup_active                  = "/hhome/ledCube/active";

const char* sub_summer                  = "/hhome/ledCube/summer";

const char* sub_cube_brightness         = "/hhome/ledCube/brightness";
const char* sub_cube_base_time          = "/hhome/ledCube/base_time";
const char* sub_cube_invers             = "/hhome/ledCube/cube_invers";

const char* sub_cube_snake_activ        = "/hhome/ledCube/snake_active";
const char* sub_cube_movie_number       = "/hhome/ledCube/movie_number";
const char* sub_cube_auto_update        = "/hhome/ledCube/auto_update";
const char* sub_cube_on                 = "/hhome/ledCube/cube_on";
const char* sub_cube_brightness_control = "/hhome/ledCube/brightness_control";

byte brightness_update = 0;
byte brightness_control = 0;

long randNumber;

byte snake_active = 1;
byte movie_number = 0;
byte auto_update = 0;
boolean cube_on = true;

#define CUBE_BASETIME          5 // ms
#define CUBE_UPDATE_TIMEBASE  50 // ms

int cube_change_time = CUBE_BASETIME * CUBE_UPDATE_TIMEBASE;

int beeb = 0;
int beeb_state = 0;

const int LEDPin    =  2; //
const int OEPin     = 16; //
const int CSPin     = 15; //
const int clockPin  = 14; // 
const int dataPin   = 13; // 
const int summerPin =  4; //

/************************************************************************/
/*                                                                      */
/************************************************************************/
const int sensorPin = A0;    // select the input pin for the potentiometer
int LDRValue = 0;            // variable to store the value coming from the sensor
int LDRValue_mean = 0;
int brightness_LDR = 32;


/************************************************************************/
/*                                                                      */
/************************************************************************/
uint8_t CubePulse (void);

#define UPDATE_TIMEBASE  5 // ms
#define SNAKE_TIMEBASE  50 // ms

#define NUMBER_OF_LEVELS    3
#define NUMBER_OF_ROWS      3

#define CUBE_DURATION_TIME    50  // ms
#define CUBE_SPEED_VALUE    CUBE_UPDATE_TIMEBASE / CUBE_DURATION_TIME

#define DEFAULT_DIM       12
#define MAX_DIM_VALUE     15

#define MAX_PULSE_AT_INI    3

/************************************************************************/
/* Variablen                                                            */
/************************************************************************/

int cube_array_hsw_base [3] = {0, 0, 0};

uint8_t cube   [3][3] = {0b111, 0b111, 0b111,
                         0b111, 0b101, 0b111,
                         0b111, 0b111, 0b111
                        }; // Cube_ARRAY

uint8_t cube_invers = 0;

uint8_t snake_status = 1;

struct s
{
  uint8_t head [3];
  uint8_t mid [3];
  uint8_t tail [3];
  uint8_t tail2 [3];
  uint8_t last_head [3];
} snake = {};


int main_speed = UPDATE_TIMEBASE;

/************************************************************************/
/* WIFI Manager                                                         */
/************************************************************************/

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// OTA setup function:
void OTA_setup (void)
{
  // Ergänzung OTA
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OtaName);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"admin");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/************************************************************************/
/* CUBE                                                                 */
/************************************************************************/
void setCUBE_Brightness (byte value){
  if (value > 63) value = 63;
  analogWrite (OEPin, 1023 - pwmtable_10[value]);
}

void set_SPI_value (byte high, byte low, byte level)
{

  high &= 0b00011111;

  if (level == 0 )  {
    high |= 0b01000000;
  }
  else if (level == 1 ) {
    high |= 0b00100000;
  }
  else if (level == 2 )  {
    high |= 0b10000000;
  }

  // take the latchPin low so the LEDs don't change while you're sending in bits:
  digitalWrite(CSPin, LOW);
  //delay (1);
  // shift out the bits:
  SPI.transfer(low);
  SPI.transfer(high);
  //take the latch pin high so the LEDs will light up:
  //delay(2);
  digitalWrite(CSPin, HIGH);
}

void setCube (int value_0, int value_1, int value_2)
{
  // Level 0
  set_SPI_value(1, 1, 0);
  // Level 1
  set_SPI_value(1, 1, 1);
  // Level 2
  set_SPI_value(1, 1, 2);
}

void setup_HW ()
{
  //set pins to output so you can control the shift register
  pinMode(CSPin, OUTPUT);
  pinMode(OEPin, OUTPUT);
  digitalWrite(OEPin, LOW);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(summerPin, OUTPUT);
  //analogWrite (summerPin, 511);
  SPI.begin();
}

void summerOn (void){
  analogWrite (summerPin, 511);
}

void summerOff (void){
  analogWrite (summerPin, 0);
}
int state_debug_led = 0;

void toggel_led (void){
  if (state_debug_led)
  {
    digitalWrite(LEDPin, HIGH);
    state_debug_led = 0;
  }
  else
  {
    digitalWrite(LEDPin, LOW);
    state_debug_led = 1;
  }
}


void summer (void){
  if (beeb == 0)
    analogWrite(summerPin, 0);
  if (beeb == 1)
  {
    if (beeb_state == 1) {
      analogWrite(summerPin, 512);
      beeb_state = 0;
    }
    else {
      analogWrite(summerPin, 0);
      beeb_state = 1;
    }
  }
  if (beeb == 2)
    analogWrite(summerPin, 512);

  if (beeb == 3)  {
    analogWrite(summerPin, 512);
    beeb = 0;
  }
}


/************************************************************************/
/* Umrechung: uint8_t array[3][3] --> uint16 [3]                  */
/************************************************************************/
void SetCube (void)
{
  for (uint8_t i = 0; i < NUMBER_OF_LEVELS; i ++)
  {
    cube_array_hsw_base[i] = 0;
    cube_array_hsw_base[i] |= (cube [i][0] & 0b111);
    cube_array_hsw_base[i] |= (cube [i][1] & 0b111) << 3;
    cube_array_hsw_base[i] |= (cube [i][2] & 0b111) << 6;
  }
}

/************************************************************************/
/*                 */
/************************************************************************/
void updateCube (void)
{
  static int level = 0;
  int help = 0;
  uint8_t lowbyte = 0;
  uint8_t highbyte = 0;

  if (!cube_invers)
  {
    if (cube_array_hsw_base[level] & 0x1)   help |= 0b00000000001;
    if (cube_array_hsw_base[level] & 0x2)   help |= 0b00000000100;
    if (cube_array_hsw_base[level] & 0x4)   help |= 0b00000001000;
    if (cube_array_hsw_base[level] & 0x8)   help |= 0b00000010000;
    if (cube_array_hsw_base[level] & 0x10)  help |= 0b00000100000;
    if (cube_array_hsw_base[level] & 0x20)  help |= 0b00001000000;
    if (cube_array_hsw_base[level] & 0x40)  help |= 0b00010000000;
    if (cube_array_hsw_base[level] & 0x80)  help |= 0b00100000000;
    if (cube_array_hsw_base[level] & 0x100) help |= 0b10000000000;
  }
  else
  {
    help = 0x07ff;
    if (cube_array_hsw_base[level] & 0x1)   help &= ~(0b00000000001);
    if (cube_array_hsw_base[level] & 0x2)   help &= ~(0b00000000100);
    if (cube_array_hsw_base[level] & 0x4)   help &= ~(0b00000001000);
    if (cube_array_hsw_base[level] & 0x8)   help &= ~(0b00000010000);
    if (cube_array_hsw_base[level] & 0x10)  help &= ~(0b00000100000);
    if (cube_array_hsw_base[level] & 0x20)  help &= ~(0b00001000000);
    if (cube_array_hsw_base[level] & 0x40)  help &= ~(0b00010000000);
    if (cube_array_hsw_base[level] & 0x80)  help &= ~(0b00100000000);
    if (cube_array_hsw_base[level] & 0x100) help &= ~(0b10000000000);
  }
  lowbyte   = lowByte(help);
  highbyte  = highByte (help);
  set_SPI_value(highbyte, lowbyte, level);
  level ++;
  if (level > 2) level = 0;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void CubeOn (void)
{
  for (uint8_t level = 0; level < NUMBER_OF_LEVELS; level ++)
    for (uint8_t reihe = 0; reihe < NUMBER_OF_ROWS; reihe ++)
      cube [level][reihe] = 0b111;
  SetCube();
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void CubeOff (void)
{
  for (uint8_t level = 0; level < NUMBER_OF_LEVELS; level ++)
    for (uint8_t reihe = 0; reihe < NUMBER_OF_ROWS; reihe ++)
      cube [level][reihe] = 0b000;
  SetCube();
}

/************************************************************************/
/* INVERS                                                               */
/************************************************************************/
void SetCubeInvers (uint8_t value)
{
  cube_invers = value;
}

/************************************************************************/
/* Punkte - voxel                                                       */
/************************************************************************/
void SetVoxel (uint8_t x, uint8_t y, uint8_t z)
{
  cube [z][y] |= (1 << x);
}

/************************************************************************/
/* SNAKE                                                             */
/************************************************************************/
void InitSnake (void)
{
  snake.head [0] = 2;   // x
  snake.head [1] = 0;   // y
  snake.head [2] = 0;   // z

  snake.mid [0] = 1;    // x
  snake.mid [1] = 0;    // y
  snake.mid [2] = 0;    // z

  snake.tail [0] = 0;   // x
  snake.tail [1] = 0;   // y
  snake.tail [2] = 0;   // z

  snake.tail2 [0] = 0;  // x
  snake.tail2 [1] = 1;  // y
  snake.tail2 [2] = 0;  // z
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void SnakeMove (void)
{
  uint8_t zufall = 0;
  uint8_t finished = 0;
  uint8_t last_head_pos [3];
  uint8_t snake_move [3];

  // letzet Position merken
  last_head_pos [0] = snake.head [0];
  last_head_pos [1] = snake.head [1];
  last_head_pos [2] = snake.head [2];

  // für die Änderung... (Shadow)
  snake_move [0] = snake.head [0];
  snake_move [1] = snake.head [1];
  snake_move [2] = snake.head [2];

  do
  {
    zufall = rand() % 6; // Zufallswert berechnen (0-5)...

    if (zufall % 2)
    {
      if ((!(snake.head [0] - snake.last_head[0])) && (!(snake.head [1] - snake.last_head[1])) && (snake.head [2] == 1))
      {
        if (snake.head [2] > snake.last_head[2])
          snake_move [2]++;
        else
          snake_move [2]--;
        finished = 1;
      }
      else if ((!(snake.head [2] - snake.last_head[2])) && (!(snake.head [1] - snake.last_head[1])) && (snake.head [0] == 1))
      {
        if (snake.head [0] > snake.last_head[0])
          snake_move [0]++;
        else
          snake_move [0]--;
        finished = 1;
      }
      else if ((!(snake.head [2] - snake.last_head[2])) && (!(snake.head [0] - snake.last_head[0])) && (snake.head [1] == 1))
      {
        if (snake.head [1] > snake.last_head[1])
          snake_move [1]++;
        else
          snake_move [1]--;
        finished = 1;
      }
    }
    if (!finished)
    {
      switch (zufall)
      {
        case 0: // x erhöhen
          //if ((snake.head [0] < 2) && (snake.head [0] >= snake.mid [0]))
          if (snake.head [0] < 2)
          {
            snake_move [0]++;
            finished = 1;
          }
          else
            finished = 0;
          break;
        case 1: // x verringern
          if (snake.head [0] > 0)
          {
            snake_move [0]--;
            finished = 1;
          }
          else
            finished = 0;
          break;
        case 2: // y erhöhen
          if (snake.head [1] < 2)
          {
            snake_move [1]++;
            finished = 1;
          }
          else
            finished = 0;
          break;
        case 3: // y verringern
          if (snake.head [1] > 0)
          {
            snake_move [1]--;
            finished = 1;
          }
          else
            finished = 0;
          break;
        case 4: // z erhöhen
          if (snake.head [2] < 2)
          {
            snake_move [2]++;
            finished = 1;
          }
          else
            finished = 0;
          break;
        case 5: // z verringern
          if (snake.head [2] > 0)
          {
            snake_move [2]--;
            finished = 1;
          }
          else
            finished = 0;
          break;
      }
    }

    if ((snake_move [0] == snake.mid[0]) && (snake_move [1] == snake.mid[1]) && (snake_move [2] == snake.mid[2]))
    {
      finished = 0;
      snake_move [0] = snake.head [0];
      snake_move [1] = snake.head [1];
      snake_move [2] = snake.head [2];
    }
    if ((snake_move [0] == snake.tail2[0]) && (snake_move [1] == snake.tail2[1]) && (snake_move [2] == snake.tail2[2]))
    {
      finished = 0;
      snake_move [0] = snake.head [0];
      snake_move [1] = snake.head [1];
      snake_move [2] = snake.head [2];
    }
  } while (!finished);

  snake.last_head [0] = snake.head [0];
  snake.last_head [1] = snake.head [1];
  snake.last_head [2] = snake.head [2];

  snake.head [0] = snake_move [0];
  snake.head [1] = snake_move [1];
  snake.head [2] = snake_move [2];

  // mid und tail anpassen

  snake.tail2 [0] = snake.tail [0];
  snake.tail2 [1] = snake.tail [1];
  snake.tail2 [2] = snake.tail [2];


  snake.tail [0] = snake.mid [0];
  snake.tail [1] = snake.mid [1];
  snake.tail [2] = snake.mid [2];


  snake.mid [0] = last_head_pos [0];
  snake.mid [1] = last_head_pos [1];
  snake.mid [2] = last_head_pos [2];

  CubeOff();

  SetVoxel (snake.head [0], snake.head [1], snake.head [2]);
  SetVoxel (snake.mid  [0], snake.mid  [1], snake.mid  [2]);
  SetVoxel (snake.tail [0], snake.tail [1], snake.tail [2]);
  //SetVoxel (snake.tail2 [0], snake.tail2 [1],snake.tail2 [2]);

  SetCube();
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void runMovie (byte number){
  static byte movie_position = 0;

  byte number_of_elements =  movie_number_of_elements_table [number];

  byte adr = movie_position * 3;

  cube_array_hsw_base [0] = moviePrt_table[number] [adr++];
  cube_array_hsw_base [1] = moviePrt_table[number] [adr++];
  cube_array_hsw_base [2] = moviePrt_table[number] [adr++];

  movie_position ++;
  if (movie_position > (number_of_elements - 1)) movie_position = 0;
}


/************************************************************************/
/*                                                                      */
/************************************************************************/
void setSnakeSpeed (int *time)
{
  *time = SNAKE_TIMEBASE * main_speed;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void setCubeSpeed (int *time, byte number)
{
  *time = movie_speed_table[number] * main_speed;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void get_LDR (void){
  LDRValue = analogRead(sensorPin);
  LDRValue_mean = (LDRValue_mean + LDRValue) / 2;
}

void brightness (void)
{
  get_LDR();
  if (brightness_control == 1) //
  {
    // range LDR 0 ... 1024     --> je größer desto dunkler!
    if (LDRValue > 3)
      brightness_LDR = (1023 - LDRValue) + 1 ;
    else
      brightness_LDR = 3;

    if (brightness_LDR > 63) brightness_LDR = 63;

    setCUBE_Brightness(brightness_LDR);
  }
}


/**************************************************************************
 *   Das ist ein Test2
 **************************************************************************/
void check_wifi() {
  Serial.println();   Serial.print("Connecting to ");   Serial.println(ssid);
  WiFi.begin(ssid, password);
  Serial.print(".");
}


/**************************************************************************
 *   Das ist ein Test2
 **************************************************************************/
void callback (char* topic, byte* payload, unsigned int length) {

  String help_str_1 = "";
  String help_str_2 = "";
  int help_int = 0;

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  char* charhelp;

  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
    charhelp += (char)payload[i];
  }
  Serial.println();

  // SUMMER
  if (strcmp(topic, sub_summer) == 0) {
    if ((char)payload[0] == '0')
      beeb = 0;
    if ((char)payload[0] == '1')
      beeb = 1;
    if ((char)payload[0] == '2')
      beeb = 2;
    if ((char)payload[0] == '3')
      beeb = 3;
  }

  // CUBE inverse
  if (strcmp(topic, sub_cube_invers) == 0) {
    if ((char)payload[0] == '0')
      cube_invers = 0;
    if ((char)payload[0] == '1')
      cube_invers = 1;
  }

  // Snake aktiv?
  if (strcmp(topic, sub_cube_snake_activ) == 0) {
    if ((char)payload[0] == '1') {
      snake_active = 1;
      cube_on = true;
    }
    else if ((char)payload[0] == '0') {
      snake_active = 0;
    }
  }

  // CUBE ON / OFF
  if (strcmp(topic, sub_cube_on) == 0) {
    if ((char)payload[0] == '1') {
      cube_on = true;
      snake_active = 0;
      movie_number = 15;
    }
    else if ((char)payload[0] == '0') {
      cube_on = false;
    }
  }

  // movie Number
  if (strcmp(topic, sub_cube_movie_number) == 0) {
    help_int = messageTemp.toInt();
    if (help_int > 14) help_int = 14;

    movie_number = (byte) help_int;
    cube_on = true;
    snake_active = 0;
  }

  // Brightness
  if (strcmp(topic, sub_cube_brightness) == 0) {
    help_int = messageTemp.toInt();
    if (help_int > 63) help_int = 63;

    setCUBE_Brightness((byte) help_int);
  }

  if (strcmp(topic, sub_cube_auto_update) == 0) {
    if ((char)payload[0] == '0')
      auto_update = 0;
    if ((char)payload[0] == '1')
      auto_update = 1;
  }

  // Speed
  if (strcmp(topic, sub_cube_base_time) == 0) {
    help_int = messageTemp.toInt();
    if (help_int < 1) help_int = 1;
    if (help_int > 50) help_int = 50;

    main_speed = help_int;
  }

  if (strcmp(topic, sub_cube_brightness_control) == 0) {
    if ((char)payload[0] == '0')
      brightness_control = 0;
    else if ((char)payload[0] == '1')
      brightness_control = 1;
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTTClientName)) {

      client.subscribe(sub_summer);  client.loop();
      client.subscribe(sub_cube_brightness);  client.loop();
      client.subscribe(sub_cube_base_time);  client.loop();

      client.subscribe(sub_cube_invers);  client.loop();

      client.subscribe(sub_cube_snake_activ);  client.loop();
      client.subscribe(sub_cube_movie_number);  client.loop();
      client.subscribe(sub_cube_auto_update);  client.loop();
      client.subscribe(sub_cube_on);  client.loop();
      client.subscribe(sub_cube_brightness_control);  client.loop();
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      //Wait 2,5 seconds before retrying
      delay(2500);
    }
  }
}

void mqtt_reconnect() {
  if (client.connect(MQTTClientName)) {

    client.subscribe(sub_summer);  client.loop();
    client.subscribe(sub_cube_brightness);  client.loop();
    client.subscribe(sub_cube_base_time);  client.loop();

    client.subscribe(sub_cube_invers);  client.loop();

    client.subscribe(sub_cube_snake_activ);  client.loop();
    client.subscribe(sub_cube_movie_number);  client.loop();
    client.subscribe(sub_cube_auto_update);  client.loop();
    client.subscribe(sub_cube_on);  client.loop();
    client.subscribe(sub_cube_brightness_control);  client.loop();

    Serial.println();
    Serial.println("mqtt reconnected!");
  }
  else
  {
    Serial.println();
    Serial.println(" mqtt disconnected!");
  }
}


void pub_mqtt (void)
{
  client.publish(pup_active, "1");
  client.publish(pup_ldrlight, String(LDRValue_mean).c_str(), 1);
}

/**************************************************************************
 *   Das ist ein Test2
 **************************************************************************/
void WIFI_OTA_MQTT_handler (void) {

  // OTA handler !
  ArduinoOTA.handle();

  // Wifi lost?
  if (WiFi.status() != WL_CONNECTED) {
    check_wifi();
  }

  // MQTT connect
  if (!client.connected()) {
    mqtt_reconnect();
  } client.loop();
}


void setup()
{
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, HIGH);
  randomSeed(analogRead(A0));
  Serial.println("Hello ...");

  Serial.begin(115200);
  setup_HW();
  summerOn();   delay(500);    summerOff();delay(500); 

  //clean FS, for testing
  //LittleFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");
  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);

        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  //if (!wifiManager.autoConnect("WEMOS D1", "password")) {
  if (!wifiManager.autoConnect(WifiApName)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());


  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  // OTA starts here!
  OTA_setup();

  // MQTT - Connection:
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //reconnect();

  digitalWrite(LEDPin, HIGH);
  CubeOn();
  InitSnake();
  setCUBE_Brightness(63);
}



void loop()
{
  static long lastReadingTime1 = 0;
  static long lastReadingTime2 = 0;
  static long lastReadingTime3 = 0;
  static long lastReadingTime4 = 0;

  WIFI_OTA_MQTT_handler ();

  if (millis() - lastReadingTime1 > (100)) {  // 0,2sec
    lastReadingTime1 = millis();
    summer();
  }

  if (millis() - lastReadingTime2 > 4) {  // 5mSec
    lastReadingTime2 = millis();
    updateCube();
    get_LDR();
  }

  if ((millis() - lastReadingTime3) > ((unsigned long)cube_change_time)) {  // tbd sec
    lastReadingTime3 = millis();
    toggel_led();

    if (cube_on)
    {
      if (snake_active)
      {
        SnakeMove();
        setSnakeSpeed(&cube_change_time);
      }
      else
      {
        setCubeSpeed(&cube_change_time, movie_number);
        runMovie(movie_number);
      }
    }
    else
      CubeOff();
  }

  if (millis() - lastReadingTime4 > (10000)) {  // tbd sec
    lastReadingTime4 = millis();
    pub_mqtt();
    if (auto_update && !snake_active)
    {
      movie_number ++;
      if (movie_number > 14) movie_number = 0;
    }
  }
}
