
#include "lin-bus.h"

uint8_t rx_state;
uint8_t rx_pin;
uint32_t kbps;

elapsedMillis timeout;
elapsedMicros break_length;

lin_bus::lin_bus(uint16_t baudrate, uint8_t Tx_pin, uint8_t Rx_pin)
{
  tx_pin = Tx_pin;
  rx_pin = Rx_pin;
  kbps = baudrate;

  if (kbps == BAUD_19200)
  {
    Serial1.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
  } else Serial1.begin(BAUD_9600_CONST);

}

int lin_bus::write(uint8_t ident, uint8_t data[], uint8_t data_size)
{
  uint8_t addrbyte = (ident & 0x3f) | addrParity(ident);
  uint8_t cksum = dataChecksum(data, data_size, 0);

  if (kbps == BAUD_19200)
  {
    Serial1.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
  } else Serial1.begin(BAUD_9600_CONST);

  Serial1.write(0x0);       // Write break
  Serial1.flush();

  Serial1.begin(kbps);      // Configure baudrate

  Serial1.write(0x55);      // write Synch Byte to serial
  Serial1.write(addrbyte);  // write Identification Byte to serial

  for (uint8_t i = 0; i < data_size; i++) Serial1.write(data[i]); // write data to serial
  Serial1.write(cksum);

  Serial1.end();
  return 1;
}


/* ISR for measuring break
 *  
 *  Line break for 19200 is 675us
 *                 9600 is 1354us
 * 
 */
void lin_bus::rxISR(void)
{

  if (rx_state == WAIT_BREAK)
  {
    if (digitalReadFast(rx_pin) == 0)
    {
      digitalWrite(11, LOW); 
      break_length = 0;
      rx_state = WAIT_HIGH;
     
    }
  }
  
  if (rx_state == WAIT_HIGH)
  {
    if (digitalReadFast(rx_pin) == 1)
    {
      if (kbps == BAUD_19200)
      { 
          if((break_length > 600) && (break_length < 700))
          {
              Serial1.begin(BAUD_19200);        // Configure serial baudrate 
          } else
          {
             //Serial.println("Wrong 19200 break ");
             rx_state = WAIT_BREAK;
          }
      }else 
      {

        if((break_length > 1250) && (break_length < 1400))
          {
              Serial1.begin(BAUD_9600);;        // Configure serial baudrate 
          }else
          {
            // Serial.println("Wrong 9600 break ");
             rx_state = WAIT_BREAK;
          }
      
      }
      Serial1.clear();
      rx_state = GOT_DATA;
    }
  }
}

void lin_bus::slave_init(void)
{

  pinMode(rx_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rx_pin), rxISR, CHANGE);

  rx_state = WAIT_BREAK;
}

uint8_t lin_bus::get_slave_state(void)
{
  return rx_state;
}


void lin_bus::set_slave_state(uint8_t state)
{
  rx_state = state;
  pinMode(rx_pin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(rx_pin),rxISR,CHANGE);
}

int lin_bus::slave_rx(uint8_t data[], uint8_t data_size)
{
  return 1;
}


uint8_t lin_bus::slave_read(uint8_t data[], uint8_t *lin_length)
{
  uint8_t i = 0;
  uint8_t rx;
  uint8_t rtn = 0;
  

  timeout = 0;
  while (i < 11)  // 11 characters to receive. Sync(0x55) + ID + Data(8 bytes) + CS = 11
  {
    if (Serial1.available())
    {
      rx = Serial1.read();
      data[i] = rx;
      i++;
      *lin_length = i;
      rtn = 1;
    }

    if (kbps == BAUD_19200)
    { 
      if (timeout > 8) // 8ms timeout for 19200 baud
      {
        //Serial.println("Rx timeout");
        break;
        rtn = 2;
      }
    }else
    {
      if (timeout > 14) // 14ms timeout for 9600 baud
      {
        //Serial.println("Rx timeout");
        break;
        rtn = 2;
      }
      
      
    }
  }
  
  set_slave_state(WAIT_BREAK);
  return rtn;
}



int lin_bus::write_request(uint8_t ident)
{
  uint8_t addrbyte = (ident & 0x3f) | addrParity(ident);

  Serial1.begin(695);       // Configure serial baudrate so that writing a 0x00 is the correct break length
  Serial1.write(0x0);       // Write break
  Serial1.flush();

  Serial1.begin(kbps);      // Configure baudrate
  Serial1.write(0x55);      // write Synch Byte to serial
  Serial1.write(addrbyte);  // write Identification Byte to serial

  Serial1.flush();          // Wait untill all data has transmitted
  Serial1.clear();          // Clear rx buffer
  return 1;
}

int lin_bus::read_request(uint8_t data[], uint8_t data_size)
{
  uint8_t i = 0;
  uint8_t rx;

  elapsedMillis waiting;

  while (i < data_size)
  {
    if (Serial1.available())
    {
      rx = Serial1.read();
      data[i] = rx;
      Serial.println(rx, HEX);
      i++;
    }

    if (waiting > 100) // 100ms timeout
    {
      Serial.println("Rx timeout");
      break;
    }
  }

  return 1;
}

#define BIT(data,shift) ((addr&(1<<shift))>>shift)
uint8_t lin_bus::addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr, 0) ^ BIT(addr, 1) ^ BIT(addr, 2) ^ BIT(addr, 4);
  uint8_t p1 = ~(BIT(addr, 1) ^ BIT(addr, 3) ^ BIT(addr, 4) ^ BIT(addr, 5));
  return (p0 | (p1 << 1)) << 6;

}

uint8_t lin_bus::dataChecksum(const uint8_t* message, uint8_t nBytes, uint16_t sum)
{

  while (nBytes-- > 0) sum += *(message++);
  // Add the carry
  while (sum >> 8) // In case adding the carry causes another carry
    sum = (sum & 255) + (sum >> 8);
  return (~sum);

}
