// Exhilaration Data Logger
//     Date:  
//        August 2010
//     Authors:
//        F. Graffagnino
//        J. Moreau
//        J. Borland

#include "Wire.h"
#include "hrmi_funcs.h"
#include <Ports.h>
#include "PortsBMP085.h"
#include <RF12.h> // needed to avoid a linker error :(

/*
 * HRMI_HOST_BAUDRATE should be set to the baudrate the host will use
 *   to communicate with the Arduino over the serial interface.
 *
 * HRMI_I2C_ADDR should be set to the I2C address the HRMI is configured
 *   with.
 */
#define HRMI_HOST_BAUDRATE 57600
#define HRMI_I2C_ADDR      63
#define HRMI_NO_DATA       0x2

// modes
#define MODE_WAITING_FOR_SER_CMD 0
#define MODE_RECORDING           1
#define MODE_WAITING_FOR_RESET   2
#define MODE_MEMORY_FULL         3

// constants
#define SERIAL_CMD_TIMEOUT       60000
#define MEM_SIZE_IN_BYTES        262144
//#define MEM_SIZE_IN_BYTES        256
#define SENSOR_VALS_RECORD_SIZE  10

PortI2C one (1);
PortI2C two (2);
PortI2C three (3);

//BMP085 psensor (one, 0); // ultra low power
BMP085 psensor (one, 1); // standard
//BMP085 psensor (one, 3); // ultra high resolution
DeviceI2C rtc (two, 0x68);
MemoryPlug mem (three);
MemoryStream stream (mem);
BlinkPlug buttons (4);

MilliTimer timer;
int mode;
int numEntries = 0;               // Number of HR values to request
int numRspBytes;                  // Number of Response bytes to read
byte i2cRspArray[34];	          // I2C response array, sized to read 32 HR values
byte hrmi_addr = HRMI_I2C_ADDR;   // I2C address to use
byte sensor_vals[SENSOR_VALS_RECORD_SIZE];
int green_led_state;
char input_cmd;

static byte bin2bcd (byte val) {
    return val + 6 * (val / 10);
}

static byte bcd2bin (byte val) {
    return val - 6 * (val >> 4);
}

static void getDate (byte* buf) {
    rtc.send();
    rtc.write(0);
    rtc.stop();

    rtc.receive();
    buf[5] = bcd2bin(rtc.read(0));
    buf[4] = bcd2bin(rtc.read(0));
    buf[3] = bcd2bin(rtc.read(0));
    rtc.read(0);
    buf[2] = bcd2bin(rtc.read(0));
    buf[1] = bcd2bin(rtc.read(0));
    buf[0] = bcd2bin(rtc.read(1));
    rtc.stop();
}

/*
 * Reads the sensor values and stores them in buffer
 *
 * buffer[0] = hours
 * buffer[1] = minutes
 * buffer[2] = seconds
 * buffer[3] = heart rate
 * buffer[4-5] = raw temperature
 * buffer[6-9] = pressure
 */
void read_sensor_values(byte * buffer)
{
  // Get the response from the RTC
  byte now[6];
  getDate(now);
  buffer[0] = now[3];  // hours
  buffer[1] = now[4];  // minutes
  buffer[2] = now[5];  // seconds
  
  // Get the response from the HRMI
  // Request a set of heart rate values
  numEntries = 1;
  hrmiCmdArg(hrmi_addr, 'G', numEntries);
  numRspBytes = numEntries + 2;
  if (hrmiGetData(hrmi_addr, numRspBytes, i2cRspArray) != -1)
  {
      // First see, if we have valid data.  If so, print it out
      if ( i2cRspArray[0] & HRMI_NO_DATA ) 
      {
          buffer[3] = 0x00;  // zero for invalid data
      } 
      else 
      {
          buffer[3] = i2cRspArray[2];  // valid heart rate
      }
  }

  // Get the response from the pressure/temp sensor
  int32_t traw = psensor.measure(BMP085::TEMP);
  int32_t praw = psensor.measure(BMP085::PRES);
  // omit following code to avoid linking in some complex calculation code
  struct { int16_t temp; int32_t pres; } payload;
  psensor.calculate(payload.temp, payload.pres);
  memcpy(&buffer[4], &payload.temp, 2);
  memcpy(&buffer[6], &payload.pres, 4);
}

void print_sensor_record(byte *buffer) {
  
    struct { int16_t temp; int32_t pres; } payload;  
    float tempF;

    Serial.print((int) buffer[0]);
    Serial.print(":");
    Serial.print((int) buffer[1]);
    Serial.print(":");
    Serial.print((int) buffer[2]);
    Serial.print(", ");
    Serial.print((int) buffer[3]);
    Serial.print(", ");
    memcpy(&payload.temp, &buffer[4], 2);
    memcpy(&payload.pres, &buffer[6], 4);
    tempF = (payload.temp / 10.0) * 1.8 + 32.0;
    Serial.print(tempF);
    Serial.print(", ");
    Serial.print(payload.pres);
    Serial.println();
}

/*
 * Reads all values stored in memory and prints them out to the serial port
 * 
 * format:
 * hh:mm:ss, heart rate, temperature F, pressure
 */
void print_all_data()
{
  int i;
  byte buffer[SENSOR_VALS_RECORD_SIZE];
  
  stream.reset();
  uint32_t mem_ptr = 0;
  uint32_t num_records_returned = 0;
  for (mem_ptr = 0; mem_ptr < (MEM_SIZE_IN_BYTES - SENSOR_VALS_RECORD_SIZE); mem_ptr += SENSOR_VALS_RECORD_SIZE)
  {
      for (i = 0; i < SENSOR_VALS_RECORD_SIZE; i++) {
          buffer[i] = stream.get();
      }

      // check for end of valid data      
      if (buffer[0] == 238 && buffer[1] == 238 && buffer[2] == 238) {
          Serial.println();
          Serial.print(num_records_returned);
          Serial.println(" records returned.");
          break;       
      } else {
          // Check to see if we need to discard the corrupted data from a flush
          if (buffer[0] == 255 && buffer[1] == 255 && buffer[2] == 255) {
              //Corrupted record.  Do nothing for now
          } else {
              print_sensor_record( buffer );
              num_records_returned++;
          }
      }
   }
}

/*
 * Dumps all memory without any regard to end of data or validity
 * 
 * format:
 * hh:mm:ss, heart rate, temperature F, pressure
 */
void dump_all_data()
{
  int i;
  byte buffer[SENSOR_VALS_RECORD_SIZE];
  
  stream.reset();
  uint32_t mem_ptr = 0;
  uint32_t num_records_returned = 0;
  for (mem_ptr = 0; mem_ptr < (MEM_SIZE_IN_BYTES - SENSOR_VALS_RECORD_SIZE); mem_ptr += SENSOR_VALS_RECORD_SIZE)
  {
      for (i = 0; i < SENSOR_VALS_RECORD_SIZE; i++) {
          buffer[i] = stream.get();
      }

      print_sensor_record( buffer );
      num_records_returned++;
  }
}

//static void reset_header_byte () {
//}

void setup () {
    Serial.begin(115200);
    Serial.println("exhilaration logger");
    
    // initialize into "WAITING_FOR_SERIAL_COMMAND" mode
    mode = MODE_WAITING_FOR_SER_CMD;

    // Initialize the I2C communication for HRMI
    hrmi_open();
    hrmiCmdArg(hrmi_addr, 'S', 1);
    
    // init pressure/temp sensor    
    // may omit following call if calculate() won't be called later on
    psensor.getCalibData();
    
    // set up timer to wait for an input command
    timer.set(SERIAL_CMD_TIMEOUT);
    Serial.print("waiting for command for ");
    Serial.print(SERIAL_CMD_TIMEOUT);
    Serial.println(" seconds ['D' = Dump Memory, 'G' = Get Valid Data, 'R' = Reset memory]");

    // turn on green led solid and red off
    green_led_state = 1;
    buttons.ledOn(1);
    buttons.ledOff(2);
}

void loop () {
  uint32_t millis_start, loop_duration;
  float percentage_complete;
   
  millis_start = millis();
  
  // logic depends on the current mode
  switch (mode) {
    
    case MODE_WAITING_FOR_SER_CMD:
    case MODE_WAITING_FOR_RESET:
    
        // check to see if timer has not expired
        // or we are waiting indefinitely
        if ( (mode==MODE_WAITING_FOR_RESET) || !timer.poll() ) {

            // timer is still alive.  check for commands
            if (Serial.available()) {
              
                // we received a command - process it
                input_cmd = Serial.read();
                switch (input_cmd) {

                    case 'D':
                        Serial.println("Dumping all memory...");
                        dump_all_data();
                        mode = MODE_WAITING_FOR_RESET;
                        break;

                    case 'G':
                        Serial.println("Returning all logged data...");
                        print_all_data();
                        mode = MODE_WAITING_FOR_RESET;
                        break;
                        
                    case 'R':
                        Serial.print("Resetting all memory... ");
                        stream.reset();
                        for (uint32_t ii = 0; ii < MEM_SIZE_IN_BYTES; ii++) {
                            if (ii % 1024 == 0) {
                                percentage_complete = ((float)ii / (float)MEM_SIZE_IN_BYTES) * 100.0;
                                Serial.print(ii);
                                Serial.print(" ");
                                Serial.print(percentage_complete);
                                Serial.println("% complete");
                            }
                            stream.put(0xEE);
                        }
                        stream.flush();
                        mode = MODE_WAITING_FOR_RESET;
                        Serial.println("done.");
                        break;
                    
                    default:
                        Serial.println("Invalid command.  Waiting indefinitely");
                        mode = MODE_WAITING_FOR_RESET;                        
                        break;
                }
            }
        } else {
            // timer has expired.  transition to recording mode
            
            if (mode != MODE_WAITING_FOR_RESET) {
                //reset memory stream to beginning
                stream.reset();
                Serial.println("entering recording mode...");
                mode = MODE_RECORDING;
            }
        }
        break;
        
    case MODE_RECORDING:
        // time to do some work
        read_sensor_values( sensor_vals );
        
        // print out for debugging
        print_sensor_record( sensor_vals );
        
        //walk through each byte and put into the mem stream
        for (int xx = 0; xx < SENSOR_VALS_RECORD_SIZE; xx++) {
            stream.put(sensor_vals[xx]);
        }
        stream.flush();
        
        // check to see if we are too close to the end
        if ((MEM_SIZE_IN_BYTES - stream.position(1)) < 60) {
            stream.flush();
            mode = MODE_MEMORY_FULL;
        }
        
        // toggle green led state
        if (green_led_state) {
            buttons.ledOff(1);
            green_led_state = 0;
        } else {
            buttons.ledOn(1);
            green_led_state = 1;            
        }
        break;

    case MODE_MEMORY_FULL:
        buttons.ledOn(2);
                
        break;
        
    default:
        // todo - print out error
        break;
        
  }

  loop_duration = millis() - millis_start;
  //Serial.print(loop_duration);
  //Serial.println(" millisecond loop duration");
  delay(1000 - loop_duration);

}

