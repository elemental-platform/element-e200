// Elemental E200 v1.0.4 firmware

// Developed by AKstudios
// Updated: 06/13/2019

#include <RFM69.h>  //  https://github.com/LowPowerLab/RFM69
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h> 
#include <avr/sleep.h>
#include <avr/wdt.h>

// define node parameters
//#define NODEID              212
uint16_t NODEID =             212; // same as above, but supports 10bit addresses (up to 1023 node IDs)
#define NETWORKID           210
#define ROOM_GATEWAYID      210
#define GATEWAYID           1
#define GATEWAY_NETWORKID   1
#define FREQUENCY           RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY          "Tt-Mh=SQ#dn#JY3_" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW          //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED                 9 // led pin
#define POWER               4
//#define targetRSSI    -80

// define objects
RFM69 radio;

// define other global variables
char dataPacket[150], _dataPacket[150];
int state, wake_interval = 0;

// ISR 
ISR(PCINT0_vect)  // Interrupt Service Routine for PCINT0 vector (pin 8)
{
  PCMSK0 = 0x00;   // Disable all PCInt Interrupts
  //asm("nop");  // do nothing
}

ISR(WDT_vect)  // Interrupt Service Routine for WatchDog Timer
{
  wdt_disable();  // disable watchdog
}

void setup()
{
  Serial.begin(115200);
  
  pinMode(POWER, OUTPUT);
  pinMode(LED, OUTPUT);  // pin 9 controls LED
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  //radio.enableAutoPower(targetRSSI);

  fadeLED();
}


void PCINT_sleep()
{
  Serial.flush(); // empty the send buffer, before continue with; going to sleep
  radio.sleep();
  digitalWrite(POWER, LOW);
  delay(1);
  
  cli();          // stop interrupts
  //DDRB |= ~(1<<DDB0); // set pin D8 as INPUT
  //PORTD |= (1<<PORTD7); //Activate pullup on pin D7
  PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts
  PCICR |= (1<<PCIE0); // enable interrupts on PCINT[0:7]
  PCMSK0 |= (1<<PCINT0); // pin change mask register for pin D8
  sei();  // enable global interrupts

  byte _ADCSRA = ADCSRA;  // save ADC state
  ADCSRA &= ~(1 << ADEN);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();       

  sleep_enable();  
  sleep_bod_disable();
  sei();       
  sleep_cpu();   
    
  sleep_disable();   
  sei();  // enable global interrupts  

  ADCSRA = _ADCSRA; // restore ADC state (enable ADC)
  delay(1);
}

void WDT_sleep()
{
  Serial.flush(); // empty the send buffer, before continue with; going to sleep
  radio.sleep();
  digitalWrite(POWER, LOW);
  delay(1);
  
  cli();          // stop interrupts
  MCUSR = 0;
  WDTCSR  = (1<<WDCE | 1<<WDE);     // watchdog change enable
  WDTCSR  = 1<<WDIE | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0); // set  prescaler to 8 second
  sei();  // enable global interrupts

  byte _ADCSRA = ADCSRA;  // save ADC state
  ADCSRA &= ~(1 << ADEN);

  asm("wdr");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();       

  sleep_enable();  
  sleep_bod_disable();
  sei();       
  sleep_cpu();   
    
  sleep_disable();   
  sei();  

  ADCSRA = _ADCSRA; // restore ADC state (enable ADC)
  delay(1);
}


void loop() 
{
  if(state==0)
  {
    sendData();
    PCINT_sleep();
    checkPin();
    sendData();
  }

  while(state==1)
  {
    checkPin(); // read current state of PIR. 1 = movement detected, 0 = no movement.
    WDT_sleep();
  }
}

void sendData()
{
  //checkPin(); // read current state of PIR. 1 = movement detected, 0 = no movement.
  readSensors();
// send datapacket
  radio.sendWithRetry(ROOM_GATEWAYID, dataPacket, strlen(dataPacket));  // send data
  WDT_sleep();   // sleep 8 seconds before sending data to main gateway
  radio.setNetwork(GATEWAY_NETWORKID);
  radio.sendWithRetry(GATEWAYID, dataPacket, strlen(dataPacket));
  radio.setNetwork(NETWORKID);

  memset(dataPacket, 0, sizeof dataPacket);   // clear array
  //_dataPacket[0] = (char)0;

  digitalWrite(LED, HIGH);
  delay(5);
  digitalWrite(LED, LOW);
  //fadeLED();
  
}


void readSensors()
{  
  digitalWrite(POWER, HIGH);
  delay(1);
  
  // read battery level
  float avg=0.0;
  for(int i=0; i<5; i++)
  {
    avg = avg + analogRead(A7);
  }
  float adc_a7 = avg / 5.0;
  float batt = (adc_a7/1023) * 2 * 3.3;
  

  // define character arrays for all variables
  char _i[3];
  char _m[3];
  char _b[5];
  
  // convert all flaoting point and integer variables into character arrays
  dtostrf(NODEID, 1, 0, _i);
  dtostrf(state, 1, 0, _m);  // this function converts float into char array. 3 is minimum width, 2 is decimal precision
  dtostrf(batt, 4, 2, _b);
  delay(10);
  
  dataPacket[0] = 0;  // first value of dataPacket should be a 0
  
  // create datapacket by combining all character arrays into a large character array
  strcat(dataPacket, "i:");
  strcat(dataPacket, _i);
  strcat(dataPacket, ",m:");
  strcat(dataPacket, _m);
  strcat(dataPacket, ",b:");
  strcat(dataPacket, _b);
  delay(10);
}



void checkPin()
{
  state = digitalRead(8); // read current state of PIR. 1 = movement detected, 0 = no movement.
}



// Averaging ADC values to counter noise in readings  *********************************************
float averageADC(int pin)
{
  float sum=0.0;
  for(int i=0;i<5;i++)
  {
     sum = sum + analogRead(pin);
  }
  float average = sum/5.0;
  return average;
}


void fadeLED()
{
  int brightness = 0;
  int fadeAmount = 5;
  for(int i=0; i<510; i=i+5)  // 255 is max analog value, 255 * 2 = 510
  {
    analogWrite(LED, brightness);  // pin 9 is LED
  
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;  // increment brightness level by 5 each time (0 is lowest, 255 is highest)
  
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
    // wait for 20-30 milliseconds to see the dimming effect
    delay(10);
  }
  digitalWrite(LED, LOW); // switch LED off at the end of fade
}

// bruh
