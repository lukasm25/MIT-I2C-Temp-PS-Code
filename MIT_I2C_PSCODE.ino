#include <Wire.h>
// Addresses to probe. Your sensor will be at one of these addresses.
// We'll scan all three, initialize anything we find, then read temps.
const uint8_t ADDRS[] = {0x4D, 0x48, 0x4F};
// Low-level I2C 16-bit helpers
//    Many temperature ICs (e.g., MAX31875) use 16-bit registers.
//    These helpers read/write a 16-bit register at <reg> from/to device <a>.
//    The device returns MSB then LSB, so we assemble in that order.

// Read a 16-bit register (returns MSB:LSB in 'v'). 'false' means I2C error.
bool i2cRead16(uint8_t a, uint8_t r, uint16_t &v){
  Wire.beginTransmission(a);
  Wire.write(r);                         // set register pointer
  if (Wire.endTransmission(false) != 0)  // repeated start, don't release bus
    return false;
  if (Wire.requestFrom((int)a, 2) != 2)  // request 2 bytes
    return false;
  v = (Wire.read() << 8) | Wire.read();  // combine MSB, LSB
  return true;
}

// Write a 16-bit register (send MSB then LSB). 'false' means I2C error.
bool i2cWrite16(uint8_t a, uint8_t r, uint16_t v){
  Wire.beginTransmission(a);
  Wire.write(r);                         // set register pointer
  Wire.write((uint8_t)(v >> 8));         // MSB first
  Wire.write((uint8_t)(v & 0xFF));       // LSB
  return Wire.endTransmission() == 0;
}

// Configuration step: force 12-bit & ensure conversions run
  //MAX31875 config register @ 0x01 is 16 bits. Key fields used here:
    // D8   : SHDN (1 = shutdown, 0 = continuous)
    // D6:D5: RES (00=9-bit, 01=10-bit, 10=11-bit, 11=12-bit)
    // D2:D1: Conversion rate (00,01,10,11 for different SPS)
  //We:
    // 1) Read current config (for visibility).
    // 2) Set RES=11 (12-bit).
    // 3) Clear SHDN (continuous conversions).
    // 4) Set conv rate to 1 sps (D2:D1=01) so 12-bit has time to finish.
    // 5) Write it back and print before/after so you can verify it stuck.
    // 6) Wait ~150 ms for the first 12-bit conversion to complete.

// Bit masks for clarity
static const uint16_t RES_MASK   = 0x0060; // bits 6:5
static const uint16_t SHDN_MASK  = 0x0100; // bit 8
static const uint16_t RATE_MASK  = 0x0006; // bits 2:1
static const uint16_t RATE_1SPS  = 0x0002; // bits 2:1 = 01

// Attempt to set 12-bit, continuous, 1 sps. Returns true if config reads back with RES=11.
bool force12_and_run(uint8_t a){
  uint16_t cfg_before;
  if(!i2cRead16(a, 0x01, cfg_before))    // read config register
    return false;

  uint16_t cfg = cfg_before;

  // Set resolution to 12-bit: D6:D5 = 11
  cfg &= ~RES_MASK;                       // clear RES bits
  cfg |=  RES_MASK;                       // set to 11

  // Ensure continuous conversion: clear shutdown bit (D8 = 0)
  cfg &= ~SHDN_MASK;

  // Use 1 sample/second conversion rate (D2:D1 = 01)
  cfg = (cfg & ~RATE_MASK) | RATE_1SPS;

  // Write config back to the device
  if(!i2cWrite16(a, 0x01, cfg))
    return false;

  // Read back to confirm what actually latched
  uint16_t cfg_after;
  if(!i2cRead16(a, 0x01, cfg_after))
    return false;

  // Print both for debugging/verification
  Serial.print("0x"); if(a<0x10) Serial.print('0'); Serial.print(a, HEX);
  Serial.print(" cfg before=0x"); Serial.print(cfg_before, HEX);
  Serial.print(" after=0x");      Serial.print(cfg_after,  HEX);
  Serial.print("  RES=");         Serial.print((cfg_after >> 5) & 0x3, BIN); // expect 11
  Serial.print("  SHDN=");        Serial.print((cfg_after >> 8) & 0x1);      // expect 0
  Serial.print("  RATE=");        Serial.println((cfg_after >> 1) & 0x3);    // expect 1

  // First 12-bit result needs ~140 ms; wait before the initial read loop
  delay(150);

  // Return true only if resolution reads back as 12-bit (11)
  return (((cfg_after >> 5) & 0x3) == 0b11);
}

// Temperature read (TMP102/MAX31875 12-bit format)
//    Register 0x00 returns a 16-bit word where the temperature is a
//    12-bit two's-complement value left-justified in the 16-bit field:
//    [sign][11:0 data]xxxx  (low 4 bits are don't-care)
//    Steps:
//    Read 16-bit value. Shift right 4 to align 12-bit field. Sign-extend if negative. Multiply by 0.0625 °C/LSB (2^-4) to get Celsius
bool read12(uint8_t a, float &tC){
  // Point to temperature register (0x00), then read 2 bytes
  Wire.beginTransmission(a);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0)   // repeated start
    return false;
  if (Wire.requestFrom((int)a, 2) != 2)
    return false;

  // Combine bytes and convert to signed 12-bit value
  uint16_t w  = (Wire.read() << 8) | Wire.read();
  int16_t raw = ((int16_t)w) >> 4;        // drop low 4 "don't care" bits
  if (raw & 0x0800) raw |= 0xF000;        // sign-extend negative temps

  tC = raw * 0.0625f;                     // 1 LSB = 0.0625 °C (12-bit)
  return true;
}

// Setup
//    Initialize Serial and I2C. For each candidate address that ACKs:
//    Try to force 12-bit + continuous + 1 sps. Warn if the config step didn't verify.
//    Note: We don't block if a device isn't present; we just skip it.
void setup(){
  Serial.begin(115200);
  while(!Serial) {}                       // wait for Serial on boards that need it
  Wire.begin();

  Serial.println("\nI2C Temp: scan 0x4D/0x48/0x4F, force 12-bit, then read");

  // Probe each address and configure if present
  for (uint8_t a : ADDRS){
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {    // device ACKed
      if (!force12_and_run(a)) {
        Serial.print("WARN: could not verify 12-bit at 0x");
        if(a<0x10) Serial.print('0'); Serial.println(a, HEX);
      }
    }
  }
}

// Loop
//    Each second, try to read the temperature from any device that ACKs. 
//    Print a timestamp (ms), address, and temperature in °C.
void loop(){
  for (uint8_t a : ADDRS){
    // Skip addresses that aren't responding at this moment
    Wire.beginTransmission(a);
    if (Wire.endTransmission() != 0)
      continue;

    float tC;
    if (read12(a, tC)){
      Serial.print(millis());
      Serial.print(", addr 0x");
      if(a<0x10) Serial.print('0');
      Serial.print(a, HEX);
      Serial.print(", ");
      Serial.print(tC, 4);                // show 12-bit precision (0.0625 °C)
      Serial.println(" C");
    } else {
      Serial.print("read failed @ 0x");
      if(a<0x10) Serial.print('0');
      Serial.println(a, HEX);
    }
  }

  delay(1000);                            // 1 Hz logging
}

