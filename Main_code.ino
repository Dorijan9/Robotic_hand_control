#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SR04.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET 4      // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define TRIG_PIN 12
#define ECHO_PIN 11

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

const int pwm = 3;      // digital pin 3 will be used for PWM (Pulse Width Modulation) output
const int dir = 8;      // digital pin 8 will be used for high/low output
const int ledPin = 13;  // digital pin 13 is also wired to a built-in LED
const int greenLedPin = 6;  // digital pin 6 for the green LED
const int redLedPin = 7;    // digital pin 7 for the red LED
uint8_t motorSpeed = 120; // 8-bit unsigned integer (0-255) defining motor speed
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

int buzzer = 10; // the pin of the active buzzer
bool playBuzzer = true; // flag to control buzzer behavior

void playBuzzerOnce()
{
  unsigned char i;
  // output an frequency
  for (i = 0; i < 80; i++)
  {
    digitalWrite(buzzer, HIGH);
    delay(1); // wait for 1ms
    digitalWrite(buzzer, LOW);
    delay(1); // wait for 1ms
  }
  // output another frequency
  for (i = 0; i < 100; i++)
  {
    digitalWrite(buzzer, HIGH);
    delay(2); // wait for 2ms
    digitalWrite(buzzer, LOW);
    delay(2); // wait for 2ms
  }
  playBuzzer = false; // set the flag to false after playing once
}

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

  uint32_t currentFrequency;

  if (! ina219.begin()) {
  Serial.println("Failed to find INA219 chip");
  while (1) { delay(10); }
  }
  pinMode(pwm, OUTPUT);    // Set PWM pin as output
  pinMode(dir, OUTPUT);    // Direction pin pins are also set as output
  pinMode(ledPin, OUTPUT); // LED pin is an output
  pinMode(greenLedPin, OUTPUT); // Green LED pin is an output
  pinMode(redLedPin, OUTPUT);   // Red LED pin is an output
  pinMode(buzzer, OUTPUT);      // initialize the buzzer pin as an output

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(20, 0);
  display.println("Distance");
  display.setCursor(20, 15);
  display.println("Force");
  display.setTextSize(2);
  display.setCursor(30, 25);
  display.println("0");
  display.setCursor(95, 25);
  display.println("cm");
  display.setCursor(30, 45);
  display.println("0");
  display.setCursor(95, 45);
  display.println("N");
  display.display();
  delay(1200);
  display.clearDisplay();
}

void loop()
{
  if (playBuzzer)
  {
    playBuzzerOnce(); // Play the buzzer sound only once at the beginning
  }

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.print("Bus Voltage:   ");
  Serial.print(busvoltage);
  Serial.println(" V");
  Serial.print("Shunt Voltage: ");
  Serial.print(shuntvoltage);
  Serial.println(" mV");
  Serial.print("Load Voltage:  ");
  Serial.print(loadvoltage);
  Serial.println(" V");
  Serial.print("Current:       ");
  Serial.print(current_mA);
  Serial.println(" mA");
  Serial.print("Power:         ");
  Serial.print(power_mW);
  Serial.println(" mW");
  Serial.println("");

  display.clearDisplay();

  // Read distance from the SR04 sensor
  long distance = sr04.Distance();
  // Read force from the force sensor
  int forceValue = analogRead(A0);

  // Display "Distance", "Force" and "Current" at the top
  display.setTextSize(1);
  display.setCursor(20, 0);
  display.println("Distance");
  display.setCursor(20, 15);
  display.println("Force");
  display.setCursor(20, 30);
  display.println("Current");

  // Display the actual measurement at the bottom
  display.setTextSize(1);
  display.setCursor(80, 0);
  display.println(distance);
  display.setCursor(110, 0);
  display.println("cm");
  display.setCursor(80, 15);
  display.println(forceValue);
  display.setCursor(110, 15);
  display.println("N");
  display.setCursor(80, 30);
  display.println(current_mA);
  display.setCursor(110, 30);
  display.println("mA");
  display.display();

  // Control the gripper based on the distance and force
  if (distance < 10 && forceValue < 10)
  {
    // If distance is less than 10 cm, turn on the red LED and turn off the green LED
    digitalWrite(greenLedPin, LOW);   // Turn off the green LED
    digitalWrite(redLedPin, HIGH);      // Turn on the red LED

    digitalWrite(dir, LOW);      // Tighten the grip
    analogWrite(pwm, motorSpeed);  // Resume motor motion
  }
  else if (distance < 10 && forceValue > 10)
  {
    // If distance is 10 cm or more and force value is above 5, turn on the green LED and turn off the red LED
    digitalWrite(greenLedPin, HIGH);    // Turn on the green LED
    digitalWrite(redLedPin, LOW);     // Turn off the red LED

    digitalWrite(dir, HIGH);       // Loosen the grip
    analogWrite(pwm, motorSpeed);  // Resume motor motion 
  }
  else if (distance > 10 && forceValue < 10)
  {
    // If distance is 10 cm or more, and force value is 5 or above, turn on the green LED and turn off the red LED
    digitalWrite(greenLedPin, HIGH);    // Turn on the green LED
    digitalWrite(redLedPin, LOW);     // Turn off the red LED

    digitalWrite(dir, HIGH);      // Loosen the grip
    analogWrite(pwm, motorSpeed);  // Resume motor motion
  }
  else if (distance > 10 && forceValue > 10)
  {
    // If distance is 10 cm or more, and force value is 5 or above, turn on the green LED and turn off the red LED
    digitalWrite(greenLedPin, HIGH);    // Turn on the green LED
    digitalWrite(redLedPin, LOW);     // Turn off the red LED

    digitalWrite(dir, HIGH);      // Loosen the grip
    analogWrite(pwm, motorSpeed);  // Resume motor motion
  }
  else
  {
    //Any other case
    digitalWrite(greenLedPin, HIGH);   // Turn on the green LED
    digitalWrite(redLedPin, LOW);      // Turn off the red LED

    digitalWrite(dir, HIGH);      // Tighten the grip
    analogWrite(pwm, motorSpeed);  // Resume motor motion
  }
  delay(500); // You can adjust the delay as needed
}
