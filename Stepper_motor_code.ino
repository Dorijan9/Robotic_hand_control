#include "IRremote.h"
#include "Stepper.h"

/*----- Variables, Pins -----*/
#define STEPS  32   // Number of steps per revolution of Internal shaft
int  Steps2Take;  // 2048 = 1 Revolution
int receiver = 12; // Signal Pin of IR receiver to Arduino Digital Pin 6

/*-----( Declare objects )-----*/
// Setup of proper sequencing for Motor Driver Pins
// In1, In2, In3, In4 in the sequence 1-3-2-4

Stepper small_stepper(STEPS, 8, 10, 9, 11);
IRrecv irrecv(receiver);    // create instance of 'irrecv'
decode_results results;     // create instance of 'decode_results'

/*-----( Function )-----*/
void translateIR() // takes action based on IR code received
// describing Remote IR codes 
{
  switch(results.value)
  {
  case 0xFFA25D: Serial.println("POWER"); 

  case 0xFFE21D: Serial.println("FUNC/STOP"); break;

  case 0xFF629D: Serial.println("VOL+"); 
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  512;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;

  case 0xFF22DD: Serial.println("FAST BACK");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -512;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;

  case 0xFF02FD: Serial.println("PAUSE"); break;

  case 0xFFC23D: Serial.println("FAST FORWARD");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  512;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;

  case 0xFFE01F: Serial.println("DOWN"); 
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  -1024;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;

  case 0xFFA857: Serial.println("VOL-");
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  -512;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;

  case 0xFF906F: Serial.println("UP");
      small_stepper.setSpeed(500); //Max seems to be 500     
      Steps2Take  =  1024;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;

  case 0xFF9867: Serial.println("EQ"); break;

  case 0xFFB04F: Serial.println("ST/REPT"); break;

  case 0xFF6897: Serial.println("0");

  case 0xFF30CF: Serial.println("1");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  32;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF18E7: Serial.println("2");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  64;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF7A85: Serial.println("3");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  128;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF10EF: Serial.println("4");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  256;  // Rotate CW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF38C7: Serial.println("5");
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -64;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF5AA5: Serial.println("6");    
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -128;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF42BD: Serial.println("7");    
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -256;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF4AB5: Serial.println("8");    
      small_stepper.setSpeed(1000); //Max seems to be 500     
      Steps2Take  =  -512;  // Rotate CCW
      small_stepper.step(Steps2Take);
      delay(100);
      break;
  case 0xFF52AD: Serial.println("9"); break;

  case 0xFFFFFFFF: Serial.println(" REPEAT"); break;  

  default: 
    Serial.println(" other button   ");

  }// End Case

  delay(100); // Do not get immediate repeat


} //END translateIR
void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);
  Serial.println("IR Receiver Button Decode"); 
  irrecv.enableIRIn(); // Start the receiver
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results)) // have we received an IR signal?

  {
    translateIR(); 
    irrecv.resume(); // receive the next value
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }  
}/* --(end main loop )-- */
