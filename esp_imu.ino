#include <Wire.h>
#include <MPU6050.h>
#define BUZZER_PIN 14
#define BUZZER_CHANNEL 0
#define BEEP_DURATION 5000 // 1 second (in milliseconds)
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

Button button1 = {12, 0, false};

MPU6050 mpu;

void IRAM_ATTR isr() {
    button_time = millis();
    if (button_time - last_button_time > 250){
        button1.numberKeyPresses++;
        button1.pressed = true;
       last_button_time = button_time;
    }
}

void setup() {
  Wire.begin(2, 15); // SDA to GPIO 22, SCL to GPIO 21
  Serial.begin(9600);

  // Initialize MPU6050
  mpu.initialize();

  // Verify the connection
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);
  ledcSetup(BUZZER_CHANNEL, 2000, 16); // 2000 Hz, 8-bit resolution
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
}

void loop() {
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
