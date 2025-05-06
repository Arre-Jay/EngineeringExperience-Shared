#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>
#include <WiFi.h>

// Digital Outputs
#define ENABLE_5V 0  
#define ENABLE_3V 8 
#define PUMP1 2
#define PUMP2 3
#define LED_GPIO 18  // Define LED pin

// digital inputs
#define POWER_SWITCH 19  // Define LED pin

// Analog Inputs
#define MOISTURE_SENSOR_1 5  // GPIO5
#define MOISTURE_SENSOR_2 4  // GPIO4

// Comms
#define SDA_PIN 6  // Set SDA to GPIO 6
#define SCL_PIN 7  // Set SCL to GPIO 7

// OLED Registers and defines
#define OLED_ADDRESS 0x3C
#define SCREEN_WIDTH 128  // OLED display width
#define SCREEN_HEIGHT 32  // OLED display height
#define OLED_RESET    -1  // Reset pin (not used on some displays)
#define SCREEN_BUFFER_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 8)
uint8_t screenBuffer[SCREEN_BUFFER_SIZE]; // Screen buffe
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BMP388 Registers
#define BMP388_ADDRESS 0x76
#define BMP388_TEMP_DATA 0x04
#define BMP388_PRESSURE_DATA 0x07
#define BMP388_STATUS 0xF3
#define BMP388_CTRL_MEAS 0xF4
Adafruit_BMP3XX bmp;

int minMoisture = 900;  // Minimum reading (wet condition)
int maxMoisture = 2600; // Maximum reading (dry condition)
bool ledState = 0; 
int switchDelay = 0; 
#define SwDelay 250

// 2.7 and 5.1 pot div
void setup() {
  
  // init digital outputs
  pinMode(LED_GPIO, OUTPUT);  // Set GPIO as output
  pinMode(ENABLE_5V, OUTPUT);    // Set GPIO as output
  //pinMode(ENABLE_3V, OUTPUT);    // Set GPIO as output
  pinMode(PUMP1, OUTPUT);     // Set GPIO as output
  pinMode(PUMP2, OUTPUT);     // Set GPIO as output

  // init digital inputs
  pinMode(POWER_SWITCH, INPUT);     // Set GPIO as output
  
  // init analog inputs
  pinMode(MOISTURE_SENSOR_1, INPUT);
  pinMode(MOISTURE_SENSOR_2, INPUT);

  // init peripherals
  Serial.begin(115200);  // Start serial comms
  delay(250);  // Wait for the serial monitor
  Serial.println("I2C Initialized on SDA: 6, SCL: 7");

  switchDelay = 0;

  //digitalWrite(ENABLE_3V, HIGH);  // Enable 5V
  //Serial.println("POWER UP");

  // switch on 5V
  digitalWrite(ENABLE_5V, HIGH);  // Enable 5V
  Serial.println("5V ON");

  // start i2c 
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize BMP388
  if (!bmp.begin_I2C(BMP388_ADDRESS)) {
    Serial.println("Couldn't find BMP388 sensor!");
    while (1);
  }

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC,OLED_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("EngineeringExperience");
  display.display();  // Update display
  delay(2000);

  testdrawline();
  
}

void loop() {

  delay(1000);  // Wait 1 second
  ledState = !ledState;  // Toggle LED state
  digitalWrite(LED_GPIO, ledState);  // switch LED state

  int moistureLevel1 = analogRead(MOISTURE_SENSOR_1);
  int moistureLevel2 = analogRead(MOISTURE_SENSOR_2);

  Serial.println(moistureLevel1);
  Serial.println(moistureLevel2);

  // Read and calculate moisture percentage
  int moisturePercent1 = getMoisturePercentage(moistureLevel1);
  int moisturePercent2 = getMoisturePercentage(moistureLevel2);

  Serial.print("Moisture Sensors: ");
  Serial.print(moisturePercent1);
  Serial.print(" ");
  Serial.println(moisturePercent2);

  // Read temperature and pressure data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure()/ 100.0f;

  // Print the results
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C | Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  // Motor control logic
  if (moisturePercent1 < 40) {
    digitalWrite(PUMP2, HIGH); // Turn ON motor
    Serial.println("Motor: ON");
  } 
  else if (moisturePercent1 > 60) {
    digitalWrite(PUMP2, LOW);  // Turn OFF motor
    Serial.println("Motor: OFF");
  }

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temperature, 1);  // Show 1 decimal place
  display.println(" C");

  display.setCursor(0, 16);
  display.print("Press: ");
  display.print(pressure, 1);
  display.println(" hPa");

  display.display();  // Update OLED

}

// Function to convert moisture reading to percentage
int getMoisturePercentage(int sensorValue) {
  int moisturePercent = map(sensorValue, minMoisture, maxMoisture, 100, 0); // Map value to range 0-100
  return moisturePercent;
}

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}