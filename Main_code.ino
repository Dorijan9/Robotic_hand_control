#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SR04.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Stepper.h>

Adafruit_INA219 ina219;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#define TRIG_PIN 5
#define ECHO_PIN 6

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

const int sw = 2;      // digital pin 2 will be used for the switch that can switch between the two robotic hand states of operation
const int pwm = 3;      // digital pin 3 will be used for PWM (Pulse Width Modulation) output
const int dir = 13;      // digital pin 8 will be used for high/low output
const int greenLedPin = A2;  // analogue pin A2 for the green LED
const int redLedPin = A1;    // analogue pin A1 for the red LED
uint8_t motorSpeed = 200; // 8-bit unsigned integer (0-255) defining motor speed
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
const int buzzer = 4; // the pin of the active buzzer
bool playBuzzer = true; // flag to control buzzer behavior

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 15;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

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
  pinMode(greenLedPin, OUTPUT); // Green LED pin is an output
  pinMode(redLedPin, OUTPUT);   // Red LED pin is an output
  pinMode(buzzer, OUTPUT);      // initialize the buzzer pin as an output
  pinMode(sw, INPUT);          // Pin will be used for input from the switch

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }

  myStepper.setSpeed(rolePerMinute);
  // initialize the serial port:
}

void loop()
{
  int pushButtonValue = digitalRead(sw);
  if (pushButtonValue == HIGH)
  {
    if (playBuzzer)
    {
      playBuzzerOnce(); // Play the buzzer sound only once at the beginning
    }

    motorSpeed = 200;
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
  
    // Read distance from the SR04 sensor
    long distance = sr04.Distance();
    // Read force from the force sensor
    int forceValue = analogRead(A0);
  
    // Display "Distance", "Force" and "Current" at the top
    display.clearDisplay();
    display.setTextColor(WHITE);
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
    delay(100);
  
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
  
      digitalWrite(dir, HIGH);      // Loosen the grip
      analogWrite(pwm, motorSpeed);  // Resume motor motion
    }
  }
  else
  {
    motorSpeed = 0;
    digitalWrite(greenLedPin, LOW);   // Turn off the green LED
    digitalWrite(redLedPin, LOW);      // Turn on the red LED
    analogWrite(pwm, motorSpeed);  // Resume motor motion
    int distance2 = 0;
    int forceValue2 = 0;
    int current_mA2 = 0;
    // Display the actual measurement at the bottom
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(80, 0);
    display.println(distance2);
    display.setCursor(110, 0);
    display.println("cm");
    display.setCursor(80, 15);
    display.println(forceValue2);
    display.setCursor(110, 15);
    display.println("N");
    display.setCursor(80, 30);
    display.println(current_mA2);
    display.setCursor(110, 30);
    display.println("mA");
    display.display();

    // step one revolution  in one direction:
    Serial.println("clockwise");
    myStepper.step(stepsPerRevolution);
    delay(500);

    // step one revolution in the other direction:
    Serial.println("counterclockwise");
    myStepper.step(-stepsPerRevolution);
    delay(500);
    digitalWrite(8, LOW);  
    digitalWrite(9, LOW);   
    digitalWrite(10, LOW);   
    digitalWrite(11, LOW);      
  }
}
