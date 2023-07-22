#define ESP_DRD_USE_SPIFFS true

// ----------------------------
// Standard Libraries - Already Installed if you have ESP32 set up
// ----------------------------

#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>

// ----------------------------
// Additional Libraries - each one of these will need to be installed.
// ----------------------------

#include <WiFiManager.h>
// Captive portal for configuring the WiFi

// Can be installed from the library manager (Search for "WifiManager", install the Alhpa version)
// https://github.com/tzapu/WiFiManager

#include <ESP_DoubleResetDetector.h>
// A library for checking if the reset button has been pressed twice
// Can be used to enable config mode
// Can be installed from the library manager (Search for "ESP_DoubleResetDetector")
//https://github.com/khoih-prog/ESP_DoubleResetDetector

#include <ArduinoJson.h>
// ArduinoJson is used for parsing and creating the config file.
// Search for "Arduino Json" in the Arduino Library manager
// https://github.com/bblanchon/ArduinoJson

// -------------------------------------
// -------   Other Config   ------
// -------------------------------------

#include <Wire.h>
#include <MPU6050.h>
#define BUZZER_PIN 12
#define BUZZER_CHANNEL 0
#define BEEP_DURATION 5000 // 1 second (in milliseconds)


const int PIN_LED = 2;

#define JSON_CONFIG_FILE "/sample_config.json"

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

// -----------------------------

// -----------------------------

DoubleResetDetector *drd;

//flag for saving data
bool shouldSaveConfig = false;

char testString[50] = "deafult value";
int testNumber = 1500;
bool testBool = true;

unsigned long button_time = 0;  
unsigned long last_button_time = 0;
const float fallThreshold = 650000; // Adjust this value to suit your needs (in m/s^3)
const int sampleInterval = 10;   // Interval in milliseconds between readings
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;


struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {15, 0, false};

MPU6050 mpu;

void IRAM_ATTR isr() {
    button_time = millis();
    if (button_time - last_button_time > 250){
        button1.numberKeyPresses++;
        button1.pressed = true;
       last_button_time = button_time;
    }
}


void saveConfigFile()
{
  Serial.println(F("Saving config"));
  StaticJsonDocument<512> json;
  json["testString"] = testString;
  json["testNumber"] = testNumber;
  json["testBool"] = testBool;

  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }

  serializeJsonPretty(json, Serial);
  if (serializeJson(json, configFile) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }
  configFile.close();
}

bool loadConfigFile()
{
  //clean FS, for testing
  // SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  // May need to make it begin(true) first time you are using SPIFFS
  // NOTE: This might not be a good way to do this! begin(true) reformats the spiffs
  // it will only get called if it fails to mount, which probably means it needs to be
  // formatted, but maybe dont use this if you have something important saved on spiffs
  // that can't be replaced.
  if (SPIFFS.begin(false) || SPIFFS.begin(true))
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists(JSON_CONFIG_FILE))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile)
      {
        Serial.println("opened config file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error)
        {
          Serial.println("\nparsed json");

          strcpy(testString, json["testString"]);
          testNumber = json["testNumber"].as<int>();
          testBool = json["testBool"].as<bool>();

          return true;
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  //end read
  return false;
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// This gets called when the config mode is launced, might
// be useful to update a display with this info.
void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered Conf Mode");

  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());

  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void setup()
{

  Wire.begin(22, 21); // SDA to GPIO 22, SCL to GPIO 21

  // Initialize MPU6050
  mpu.initialize();

  // Verify the connection
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);
  ledcSetup(BUZZER_CHANNEL, 2000, 16); // 2000 Hz, 8-bit resolution
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  pinMode(PIN_LED, OUTPUT);

  bool forceConfig = false;

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
  if (drd->detectDoubleReset())
  {
    Serial.println(F("Forcing config mode as there was a Double reset detected"));
    forceConfig = true;
  }

  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup)
  {
    Serial.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }

  //WiFi.disconnect();
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  Serial.begin(115200);
  delay(10);

  // wm.resetSettings(); // wipe settings

  WiFiManager wm;

  //wm.resetSettings(); // wipe settings
  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wm.setAPCallback(configModeCallback);

  //--- additional Configs params ---

  // Text box (String)
  WiFiManagerParameter custom_text_box("key_text", "Enter your string here", testString, 50); // 50 == max length

  // Text box (Number)
  char convertedValue[6];
  sprintf(convertedValue, "%d", testNumber); // Need to convert to string to display a default value.

  WiFiManagerParameter custom_text_box_num("key_num", "Enter your number here", convertedValue, 7); // 7 == max length

  //Check Box
  char *customHtml;
  if (testBool)
  {
    customHtml = "type=\"checkbox\" checked";
  }
  else
  {
    customHtml = "type=\"checkbox\"";
  }
  WiFiManagerParameter custom_checkbox("key_bool", "Checkbox", "T", 2, customHtml); // The "t" isn't really important, but if the
  // box is checked the value for this field will
  // be "t", so we can check for that.

  //add all your parameters here
  wm.addParameter(&custom_text_box);
  wm.addParameter(&custom_text_box_num);
  wm.addParameter(&custom_checkbox);

  Serial.println("hello");

  digitalWrite(PIN_LED, LOW);
  if (forceConfig)
  {
    if (!wm.startConfigPortal("WifiTetris", "clock123"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  else
  {
    if (!wm.autoConnect("WifiTetris", "clock123"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      // if we still have not connected restart and try all over again
      ESP.restart();
      delay(5000);
    }
  }

  // If we get here, we are connected to the WiFi
  digitalWrite(PIN_LED, HIGH);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Lets deal with the user config values

  // Copy the string value
  strncpy(testString, custom_text_box.getValue(), sizeof(testString));
  Serial.print("testString: ");
  Serial.println(testString);

  //Convert the number value
  testNumber = atoi(custom_text_box_num.getValue());
  Serial.print("testNumber: ");
  Serial.println(testNumber);

  //Handle the bool value
  testBool = (strncmp(custom_checkbox.getValue(), "T", 1) == 0);
  Serial.print("testBool: ");
  if (testBool)
  {
    Serial.println("true");
  }
  else
  {
    Serial.println("false");
  }

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveConfigFile();
  }
}

void loop()
{
  drd->loop();
  static unsigned long lastTime = 0;

  // Read accelerometer data
  if (millis() - lastTime >= sampleInterval) {
    lastTime = millis();

    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Convert accelerometer readings from m/s^2 to mg (1 g = 9.8 m/s^2)
    float acceleration_mg_x = ax / 9.8;
    float acceleration_mg_y = ay / 9.8;
    float acceleration_mg_z = az / 9.8;

    // Calculate jerk by taking the derivative of acceleration with respect to time
    float jerkX = (acceleration_mg_x - prevAccX) / (sampleInterval / 1000.0);
    float jerkY = (acceleration_mg_y - prevAccY) / (sampleInterval / 1000.0);
    float jerkZ = (acceleration_mg_z - prevAccZ) / (sampleInterval / 1000.0);

    // Update previous acceleration values for the next iteration
    prevAccX = acceleration_mg_x;
    prevAccY = acceleration_mg_y;
    prevAccZ = acceleration_mg_z;

    // Calculate the magnitude of jerk
    float jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);
    Serial.println(jerkMagnitude);
    // Check for a fall
    if (jerkMagnitude > fallThreshold) {
      // Fall detected
      Serial.println("Fall detected!");
      ledcWriteTone(BUZZER_CHANNEL, 5000); // Play a 1kHz tone on the buzzer pin
      // Add your fall reaction code here (e.g., triggering an alarm, sending an alert, etc.)
    }
    if (button1.pressed) {
      Serial.printf("Button has been pressed %u times\n", button1.numberKeyPresses);
      button1.pressed = false;
      ledcWrite(BUZZER_CHANNEL, 0); // Stop the tone
    }
  }
}
