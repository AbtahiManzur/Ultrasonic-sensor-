/*
 * 2 HC - SR04 ultrasonic modules connected to a singe Arduino UNO through GPIO.
 * This code calculates distance to a surface and triggers its corresponding buzzer if a surface is within a 
 * certain range using interrupts.
 * 
 */

#define TRIG PD2  /*Trig pin takes a 10 microsecond TTL pulse and sends out an ultrasonic burst and the echo pin goes high*/
#define ECHO PD3  /*Once the reflected sound is received, the echo pin goes back low*/
#define OUT PB0   /*The output will be used to trigger a buzzer*/

#define TRIG2 PD4  /*Same macros but for the other ultrasonic sensor*/
#define ECHO2 PB2   
#define OUT2 PB1    


volatile unsigned long riseTime;  
volatile unsigned long fallTime;  
unsigned long pulseTime;         
unsigned long distance;           
float getDistance();

volatile unsigned long riseTime2;  
volatile unsigned long fallTime2;  
unsigned long pulseTime2;         
unsigned long distance2;           
float getDistance2();

void setup() 
{
  DDRB |= (1<<OUT);     /*set up TRIG and OUT pins as outputs and ECHO pin as an input*/
  DDRD |= (1<<TRIG);
  DDRD &= ~(1<<ECHO);
  PCICR |= (1<<PCIE2);  /*Configure rising and falling edge interrupts on the ECHO pin*/
  PCMSK2 |= (1<<PCINT19); 

  DDRB |= (1<<OUT2);  /*Same thing for other ultrasonic sensor*/
  DDRD |= (1<<TRIG2);
  DDRB &= ~(1<<ECHO2);
  PCICR |= (1<<PCIE0); 
  PCMSK0 |= (1<<PCINT2); 
  Serial.begin(9600);
}

void loop() 
{
  float range = getDistance();
  float range2 = getDistance2();
  Serial.println(range);
  if (range <= 30)
  {
    PORTB |= (1<<OUT); 
  }
  else
  {
    PORTB &= ~(1<<OUT);
  }
  if (range2 <= 30)
  {
    PORTB |= (1<<OUT2);
  }
  else
  {
    PORTB &= ~(1<<OUT2);
  }
}

float getDistance()     
{
  PORTD &= ~(1<<TRIG);          /*First clear the TRIG pin then raise it high for 10 microseconds. An interrupt will calculate the pulse time and the distance to */
  delayMicroseconds(2);         /*a surface from the first ultrasonic sensor is returned*/
  PORTD = (1<<TRIG);
  delayMicroseconds(10);
  PORTD &= ~(1<<TRIG);
  distance = pulseTime*0.0343/2; // result in cm 
  return distance;
}

float getDistance2()          
{
  PORTD &= ~(1<<TRIG2);         /*Same thing for the second ultrasonic sensor*/
  delayMicroseconds(2);
  PORTD = (1<<TRIG2);
  delayMicroseconds(10);
  PORTD &= ~(1<<TRIG2);
  distance2 = pulseTime2*0.0343/2; // result in cm 
  return distance2;
}


ISR(PCINT2_vect)            
{  
  if ((PIND & (1<<ECHO)) == (1<<ECHO))  /*ECHO pin interrupt of first ultrasonic sensor checks to see if echo pin is high*/
    {                                   /*If yes, that means a burst was sent so start the micros() for timing*/
      riseTime = micros();              /*If not, that means a burst was sent and now echo going low, this is the total pulse time*/
    }    
  else
    {
      fallTime = micros();
      pulseTime = fallTime - riseTime; 
    }x

}

ISR(PCINT0_vect)
{
   if ((PINB & (1<<ECHO2)) == (1<<ECHO2)) /*Same thing for second ultrasonic sensor*/
    { 
      riseTime2 = micros(); 
    }    
   else 
    {
      fallTime2 = micros();
      pulseTime2 = fallTime2 - riseTime2; 
    }
}
