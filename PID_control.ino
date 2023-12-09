#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SR04.h>
#include <Adafruit_INA219.h>
#include <Stepper.h>
#include <IRremote.h>

Adafruit_INA219 ina219;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#define kp 1
#define ki 0.002
#define kd 5
double priError = 0;
double toError = 0;

#define TRIG_PIN 34
#define ECHO_PIN 26

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

const int numReadings = 10;
int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int avgCurrent = 0;            // the average

const int sw = 24;      // digital pin 24 will be used for the switch that can switch between the two robotic hand states of operation
const int pwm = 2;      // digital pin 2 will be used for PWM (Pulse Width Modulation) output
const int dir = 22;      // digital pin 22 will be used for high/low output
const int greenLedPin = 36;  // digital pin 36 for the green LED
const int redLedPin = 28;    // digital pin 28 for the red LED
uint8_t motorSpeed = 200; // 8-bit unsigned integer (0-255) defining motor speed
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
int current_new = 0;
int current_display = 0;
const int buzzer = 30; // analogue pin 30 for the active buzzer
bool playBuzzer = true; // flag to control buzzer behavior

#define STEPS  32   // Number of steps per revolution of Internal shaft
int  Steps2Take;  // 2048 = 1 Revolution
const int receiver = 12; // Signal Pin of IR receiver to Arduino Digital Pin 12
const int N4 = 8;
const int N3 = 9;
const int N2 = 10;
const int N1 = 11;

// Setup of proper sequencing for Motor Driver Pins
// In1, In2, In3, In4 in the sequence 1-3-2-4
Stepper small_stepper(STEPS, N4, N2, N3, N1);
IRrecv irrecv(receiver);    // create instance of 'irrecv'
decode_results results;     // create instance of 'decode_results'

void translateIR() // takes action based on IR code received
// describing Remote IR codes 
{
  switch(results.value)
  {
  case 0xFFA25D: Serial.println("POWER"); 
      break; 

  case 0xFFE21D: Serial.println("FUNC/STOP"); 
      break;

  case 0xFF629D: Serial.println("VOL+"); 
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  512;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;

  case 0xFF22DD: Serial.println("FAST BACK");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -512;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;

  case 0xFF02FD: Serial.println("PAUSE"); 
      break;

  case 0xFFC23D: Serial.println("FAST FORWARD");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  512;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;

  case 0xFFE01F: Serial.println("DOWN"); 
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  -1024;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;

  case 0xFFA857: Serial.println("VOL-");
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  -512;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;

  case 0xFF906F: Serial.println("UP");
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  1024;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;

  case 0xFF9867: Serial.println("EQ"); 
      break;

  case 0xFFB04F: Serial.println("ST/REPT"); 
      break;

  case 0xFF6897: Serial.println("0"); 
      break;

  case 0xFF30CF: Serial.println("1");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  32;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF18E7: Serial.println("2");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  64;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF7A85: Serial.println("3");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  128;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF10EF: Serial.println("4");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  256;  // Rotate CW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF38C7: Serial.println("5");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -64;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF5AA5: Serial.println("6");    
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -128;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF42BD: Serial.println("7");    
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -256;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF4AB5: Serial.println("8");    
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -512;  // Rotate CCW
      small_stepper.step(Steps2Take);
      break;
  case 0xFF52AD: Serial.println("9"); 
      break;

  case 0xFFFFFFFF: Serial.println(" REPEAT"); 
      break;  

  default: 
    Serial.println(" other button   ");

  }// End Case
}

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

void PID() 
{
  int setP = 300;
  double error = setP - avgCurrent;
  double Pvalue = error * kp;
    Serial.println("P");
    Serial.println(Pvalue);
  double Ivalue = toError * ki;
    Serial.println("I");
    Serial.println(Ivalue);
  double Dvalue = (error - priError) * kd;
    Serial.println("D");
    Serial.println(Dvalue); 
 

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  priError = error;
  toError += error;
  current_new = (int)PIDvalue;
  Serial.println("PID");
  Serial.println(current_new);
  current_display=map(current_new,-1000,1000,200,400);
  motorSpeed=map(current_new,-1000,1000,150,200);
  Serial.println("Speed");
  Serial.println(motorSpeed);
  if (current_new < 0) {
    motorSpeed = 0;
  }
  if (current_new > 400) {
    motorSpeed=200;
  }
}

void smoothing()
{
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = current_mA;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  avgCurrent = total / numReadings;
  //Serial.println(avgCurrent);
  // send it to the computer as ASCII digits
  delay(1);  // delay in between reads for stability
}

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

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

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }

  irrecv.enableIRIn(); // Start the receiver
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

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
  
    // Read distance from the SR04 sensor
    long distance = sr04.Distance();
    // Read force from the force sensor
    int forceValue = analogRead(A0);
  
    smoothing();
    PID();
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
    display.println(current_display);
    display.setCursor(110, 30);
    display.println("mA");
    display.display();
    delay(10);
    Serial.println(motorSpeed);

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
    const int distance2 = 0;
    const int forceValue2 = 0;
    const int current_mA2 = 0;
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

    if (irrecv.decode(&results)) // have we received an IR signal?
    {
      translateIR(); 
      irrecv.resume(); // receive the next value
      digitalWrite(N4, LOW);  
      digitalWrite(N3, LOW);   
      digitalWrite(N2, LOW);   
      digitalWrite(N1, LOW); 
      delay(100); // Do not get immediate repeat
    }     
  }
}