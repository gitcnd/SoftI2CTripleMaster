/*
  Arduino_Pro_Mini33_HMC5883L_Magnetometer_Triple_BitBang_Play.ino v1.0 

  This triggers a data read operation, then afterwartds, reads the data.
  With multiple magnetometers on the wire, it means a simultaneous read can be done by them all, then afterwards, the data read from each individually.

  (C) Copyright 2018 (NOT: GNU GENERAL PUBLIC LICENSE, Version 3) - Chris Drake <cdrake@cpan.org> * All Rights Reserved *.
*/

#include <SerialID.h>	// So we know what code and version is running inside our MCUs
SerialIDset("\n#\tv1.01 " __FILE__ "\t" __DATE__ " " __TIME__);


#include <NodeID.h>	// So every device knows who itself is
NodeID NodeID(2); unsigned int myNodeID=0; // Unique ID for each board.  We use the 3rd and 4th bytes (zero-based offset 2 and 3) inside EEPROM to store our ID.


#include <MorseLED.h>	// So we can communicated codes with the LED
MorseLED morse(13);


// Must define the bitbang pin numbers here, before including the header
#define SDA1_PORT PORTC
#define SDA1_PIN 0
#define SDA2_PORT PORTC
#define SDA2_PIN 4
#define SDA3_PORT PORTC
#define SDA3_PIN 1
#define SDA4_PORT PORTC
#define SDA4_PIN 2
// #define SDA5_PORT PORTC
// #define SDA5_PIN 3
#define SCL_PORT PORTC
#define SCL_PIN 5 // = A5
#include <SoftI2CTripleMaster.h>


unsigned long timer=millis();


//triple axis HMC5883L data
int16_t x=1;
int16_t y=2;
int16_t z=3;


#define MAG_HMC5883L_ADDRESS ((char) 0x1E) //I2C Address for The HMC5883 - nb - 400kbps max I2C data rate




void setup() {
  pinMode(13,OUTPUT);
  morse.asay(F("R"));
  SerialIDshow(115200); // starts Serial.
  unsigned int myNodeID=NodeID.get();
  Serial.print("# My NodeID=");Serial.println(myNodeID); // NodeID.set(9);

  if (!i2c_init()) // Initialize i2c bitbanger and check for bus lockup
        Serial.println("I2C init failed");

  Setup_HMC5883L();
} // setup

void loop() { 


  Read_HMC5883L(); 	// Populates x,y,z

  float bearing=((atan2(y,x))*180)/PI;//values will range from +180 to -180 degrees
  if (bearing < 0)  { bearing = 360 + bearing; }

  // Show Values
  Serial.print(x); Serial.print(F("\t"));
  Serial.print(y); Serial.print(F("\t"));
  Serial.print(z); Serial.print(F("\t"));
  Serial.println(bearing);

  morse.delayled(2000);
  Serial.println("Triggered...");
  Trigger_Read();
  morse.delayled(2000);

} // loop



void Setup_HMC5883L() {

  //Wire.begin();
  //Wire.beginTransmission(MAG_HMC5883L_ADDRESS); //start talking
  //Wire.write(0x02); // Set the Register
  //Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  //Wire.endTransmission();

  while (!i2c_start((MAG_HMC5883L_ADDRESS<<1)|I2C_WRITE)); // wait for it not to be busy
  // i2c_write(0x02); i2c_write(0x00); // Tell the HMC5883 to Continuously Measure
  // i2c_write(0x00); i2c_write(0x00); i2c_write(0xFF); // Setup: CRA=no-averaging, no-bias + CRB=max-gain + Max-speed, MODE=idle-mode
  i2ca_write(0x00); i2ca_write(0x00); i2ca_write(0x00); i2ca_write(0xFF); // Setup: CRA=no-averaging, no-bias + CRB=max-gain + Max-speed, MODE=idle-mode
  i2c_stop(); // send stop condition

} // Setup_HMC5883L

void Trigger_Read() {
  while (!i2c_start((MAG_HMC5883L_ADDRESS<<1)|I2C_WRITE)); // wait for it not to be busy
  i2ca_write(0x02); i2ca_write(0b11111101);
  i2c_stop(); // send stop condition

}


void Read_HMC5883L() {
  //Tell the HMC what regist to begin writing data into
//  Wire.beginTransmission(MAG_HMC5883L_ADDRESS);
//  Wire.write(0x03); //start with register 3.
//  Wire.endTransmission();
 
 //Read the data.. 2 bytes for each axis.. 6 total bytes
//  Wire.requestFrom(MAG_HMC5883L_ADDRESS, 6);
//  if(6<=Wire.available()){
//    x =  Wire.read()<<8;//MSB x 
//    x |= Wire.read();   //LSB x
//    z =  Wire.read()<<8;//MSB z
//    z |= Wire.read();   //LSB z
//    y =  Wire.read()<<8;//MSB y
//    y |= Wire.read();   //LSB y
//  }


  while (!i2c_start((MAG_HMC5883L_ADDRESS<<1)|I2C_WRITE)); // wait for it not to be busy
  i2ca_write(0x03); // start with register 3.
  i2c_rep_start((MAG_HMC5883L_ADDRESS<<1)|I2C_READ); // restart for reading
  x =  i2c2_read(false)<<8;
  x |= i2c2_read(false);
  z =  i2c2_read(false)<<8;
  z |= i2c2_read(false);
  y =  i2c2_read(false)<<8;
  y |= i2c2_read(true); // read one byte and send NAK to terminate
  i2c_stop(); // send stop condition

}


/* 
// Simple sketch to read out one register of an I2C device

#define I2C_7BITADDR 0x68 // DS1307
#define MEMLOC 0x0A 

void setup(void) {
    Serial.begin(57600);
    if (!i2c_init()) // Initialize everything and check for bus lockup
        Serial.println("I2C init failed");
}

void loop(void){
    if (!i2c_start((I2C_7BITADDR<<1)|I2C_WRITE)) { // start transfer
        Serial.println("I2C device busy");
        return;
    }
    i2c_write(MEMLOC); // send memory address
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ); // restart for reading
    byte val = i2c_read(true); // read one byte and send NAK to terminate
    i2c_stop(); // send stop condition
    Serial.println(val);
    delay(1000);
}

*/
