/*
 *  * This is a Teensy 3.2 LIN-Bus demo using this board:
 * http://skpang.co.uk/catalog/teensy-canbus-and-linbus-breakout-board-include-teensy-32-p-1566.html
 *     
 * Reads the LIN and display the data raw. Includes sync, ID, data and CS
 * 11 bytes mean: sync(0x55) + ID + Data(8 bytes) + CS = 11 
 * 
 * skpang.co.uk 2019
 *  
 */
 
#include "lin-bus.h"

int led = 13;
int lin_cs = 23;
 int lin_tx_pin = 1;
 int lin_rx_pin = 0;

int test_pin = 11;
lin_bus lin(BAUD_19200, lin_tx_pin, lin_rx_pin);
//lin_bus lin(BAUD_9600, lin_tx_pin, lin_rx_pin);

uint8_t lin_data[10]; 
uint8_t lin_length;

void setup() 
{
  pinMode(led, OUTPUT);    
  pinMode(lin_cs, OUTPUT); 
  pinMode(test_pin, OUTPUT); 
  
  digitalWrite(led, HIGH);   
  digitalWrite(lin_cs, HIGH);    
  digitalWrite(test_pin, HIGH); 
   
  delay(1000); 
   
  Serial.begin(11520);            // Configure serial port for debug messages
  Serial.println("LIN-bus Slave read RAW. www.skpang 2019");
  Serial.println("Reads the LIN and display the data raw. Includes sync, ID, data and CS");
  Serial.println("11 bytes mean: sync(0x55) + ID + Data(8 bytes) + CS = 11 ");
  Serial.println(" ");
  digitalWrite(led, LOW); 
 
  lin.slave_init();
}

  
void loop() 
{
   if(lin.get_slave_state() == GOT_DATA)
   {
    
      lin.slave_read(lin_data,&lin_length);
     
      Serial.print("Data received. Length: ");
      Serial.print(lin_length,DEC);
      Serial.print("  Data: ");
      
      for(uint8_t i=0;i<lin_length;i++)
      {
         Serial.print(lin_data[i],HEX);
         Serial.print(" ");
      }
      Serial.println("  ");
    }         
}
