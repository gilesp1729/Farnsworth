#include <ArduinoBLE.h>
#include <RunningAverage.h>
//#include <NanoBLEFlashPrefs.h>
#include <VescUart.h>

// Farnsworth

// Open source cycle display and pedelec speed controller.
// Controls VESC and reports via BLE using CP and Babelfish protocol.

#include "constants.h"
#include "poller.h"

/** Initiate VescUart class */
VescUart VU;

// Service and characteristics
BLEService CyclePowerService("1818");
BLECharacteristic CyclePowerFeature("2A65", BLERead, 4);
BLECharacteristic CyclePowerMeasurement("2A63", BLERead | BLENotify, 14);
BLECharacteristic CyclePowerSensorLocation("2A5D", BLERead, 1);

// Battery service, in case it is needed. Even if not, it keeps
// the display tidy on apps that display it
BLEService batteryService("180F");

// Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);
int BatteryPercent = 100;  // start with 100% full

// Babelfish motor service and characteristics
BLEService motorService("FFF0");
BLECharacteristic motorMeasurement("FFF1", BLERead | BLENotify, 14);
BLECharacteristic motorSettings("FFF2", BLERead | BLENotify, 7);
BLECharacteristic motorNewSettings("FFF3", BLEWrite, 7);

int sensor_pos = 11;      // sensor position magic number.
// No idea what they mean (shitty specs) but it's mandatory to supply one.

unsigned char bleBuffer[14];
unsigned char slBuffer[1];
unsigned char fBuffer[4];
int connected = 0;
int vesc_connected = 0;

// Feature bits: bit 2 - wheel pair present, bit 3 - crank pair present
unsigned short feature_bits = 0x0C;

// Flags bits: bit 4 - wheel pair present, bit 5 - crank pair present
unsigned short flags = 0x30;

// Torque sensor reading with no force on pedal
float static_torque;

// Moving average filter for torque readings
RunningAverage torqueFilter(FILTER_SIZE);

// Poller for the wheel and crank pins
Poller Wheel(WHEEL_PIN);
Poller Crank(CRANK_PIN);

// Access to the flash (non-volatile) memory
//NanoBLEFlashPrefs flash;


// Timing and counters
volatile unsigned long previousMillis = 0;
volatile unsigned long previousPowerCalc = 0;
volatile unsigned long currentMillis = 0;

// Debouncing counters for wheel and crank interrupt routines
volatile unsigned long time_prev_wheel = 0, time_now_wheel;
volatile unsigned long time_prev_crank = 0, time_now_crank;
volatile unsigned long time_chat_wheel = 50;  // dead zone for bounces
volatile unsigned long time_chat_crank = 10;  // dead zone for bounces (crank)

// Counters for updating power and speed services
volatile unsigned long wheelRev = 0;
volatile unsigned long oldWheelRev = 0;
volatile unsigned long oldWheelMillis = 0;  // last time sent to BLE
volatile unsigned long lastWheeltime = 0;   // last time measurement taken
// Note: the wheel time is in half-ms (1/2048 sec), unlike CSC where it is in ms
volatile float raw_torque = 0;              // torque in Nm read from sensor
volatile float torque = 0;                  // averaged torque in Nm
volatile float speed = 0;                   // calculated speed in km/h
volatile float crpm = 0;                    // calculated crank rpm
short power = 0;                            // human power in watts
float req_amps;                             // requested amps sent to VESC

// Wheel circumference and initial speed limit for motor
unsigned long circ = 2300;            // mm
unsigned long speed_limit = 2500;     // km/h*100

// Ramp down speed from speed_limit to speed_lnmit + speed_ramp
float speed_ramp = 4.0;

// PAS level for motor (1 = eco, 2 = tour, 3 = sport, etc.)
int pas = 2;

// PAS level multipliers for 5 PAS levels. 
float pas_mult[6] = { 0, 1.0, 1.8, 2.6, 3.4, 4.2};

// Counters for cadence reporting
volatile unsigned long time_prev_crank_avg = 0;
volatile unsigned int crankPulsesRev = 0;   // pulse counter for whole revolutions
volatile unsigned int crankPulsesCad = 0;   // pulse counter for averaging out cadence measurement
volatile unsigned int crankRev = 0;         // rev counter for CSC/CP reporting
volatile unsigned int oldCrankRev = 0;
volatile unsigned long lastCranktime = 0;   // time counter for CSC/CP reporting
volatile unsigned long oldCrankMillis = 0;


// Fill the CP measurement array and send it
void fillCP()
{
  // Fill the CP measurement characteristic.
  int n = 0;  // to facilitate adding and removing stuff
  bleBuffer[n++] = flags & 0xff;
  bleBuffer[n++] = (flags >> 8) & 0xff;
  bleBuffer[n++] = power & 0xff;
  bleBuffer[n++] = (power >> 8) & 0xff;
  bleBuffer[n++] = wheelRev & 0xff;
  bleBuffer[n++] = (wheelRev >> 8) & 0xff;  // UInt32
  bleBuffer[n++] = (wheelRev >> 16) & 0xff;
  bleBuffer[n++] = (wheelRev >> 24) & 0xff;
  bleBuffer[n++] = lastWheeltime & 0xff;
  bleBuffer[n++] = (lastWheeltime >> 8) & 0xff;
  bleBuffer[n++] = crankRev & 0xff;
  bleBuffer[n++] = (crankRev >> 8) & 0xff;
  bleBuffer[n++] = lastCranktime & 0xff;
  bleBuffer[n++] = (lastCranktime >> 8) & 0xff;

  CyclePowerMeasurement.writeValue(bleBuffer, n);

  // Fill the battery % and some motor characteristics.
  if (vesc_connected)
  {
    // Assume 100% SOC is 4V per cell, 0% is 3V.
    BatteryPercent = 100 * ((VU.data.inpVoltage / NUM_SERIES_CELLS) - 3);
    if (BatteryPercent < 0)
      BatteryPercent = 0;
    else if (BatteryPercent > 100)
      BatteryPercent = 100;
    batteryLevelChar.writeValue(BatteryPercent);
  }
}

// Fill the motor service characteristics and send them
void fillMS()
{
  // Some integer values needed for the Babelfish protocol
  uint16_t volts100 = VU.data.inpVoltage * 100;
  uint16_t amps100 = VU.data.avgInputCurrent * 100;
  uint16_t req_amps100 = req_amps * 100;
  uint16_t kmh100 = speed * 100;

  int n = 0;  // to facilitate adding and removing stuff
  bleBuffer[n++] = kmh100 & 0xff;		                // speed in km/h*100
  bleBuffer[n++] = (kmh100 >> 8) & 0xff;	   
  bleBuffer[n++] = crpm;			                      // cadence in rpm
  bleBuffer[n++] = (int)(power * pas_mult[pas]) & 0xff;		        // motor power in watts = human power * PAS multiplier
  bleBuffer[n++] = ((int)(power * pas_mult[pas]) >> 8) & 0xff;	   
  bleBuffer[n++] = volts100 & 0xff;		              // battery volts*100
  bleBuffer[n++] = (volts100 >> 8) & 0xff;	   
  bleBuffer[n++] = amps100 & 0xff;		              // motor current in amps*100
  bleBuffer[n++] = (amps100 >> 8) & 0xff;	   
  bleBuffer[n++] = req_amps100 & 0xff;		          // requested motor current in amps*100
  bleBuffer[n++] = (req_amps100 >> 8) & 0xff;				      
  bleBuffer[n++] = pas;			                          // Unused (PAS left here for backward compatibility)
  bleBuffer[n++] = (uint8_t)(VU.data.tempMotor + 40);		// Motor temp in degC + 40			      
  bleBuffer[n++] = (uint8_t)(VU.data.tempMosfet + 40);	// Controller temp in degC + 40		      
  motorMeasurement.writeValue(bleBuffer, n);				      

  n = 0;								      
  bleBuffer[n++] = speed_limit & 0xff;		        // speed limit in km/h*100		      
  bleBuffer[n++] = (speed_limit >> 8) & 0xff;
  bleBuffer[n++] = circ & 0xff;		                  // wheel circumference in mm
  bleBuffer[n++] = (circ >> 8) & 0xff;
  bleBuffer[n++] = pas;                             // PAS level (0-5)
  bleBuffer[n++] = 0;
  bleBuffer[n++] = 0;
  motorSettings.writeValue(bleBuffer, n);
}

// Check if writable characteristics have changed from the central.
void check_writable_chars()
{
  motorNewSettings.readValue(bleBuffer, 7);
  speed_limit = bleBuffer[0] + ((uint16_t)bleBuffer[1] << 8);
  circ = bleBuffer[2] + ((uint16_t)bleBuffer[3] << 8);
  pas = bleBuffer[4];
}

// Get VESC stats
// This is very slow (100ms!) esp when no VESC is connected, so don't do it 
// very often.
void queryVESC()
{
  if ( VU.getVescValues() ) 
  {
    vesc_connected = true;
  #if 0
    Serial.print("RPM ");
    Serial.print(VU.data.rpm / NUM_POLE_PAIRS);
    Serial.print(" Volts ");
    Serial.print(VU.data.inpVoltage);
    Serial.print(" Amps ");
    Serial.print(VU.data.avgInputCurrent);
    Serial.print(" Duty ");
    Serial.print(VU.data.dutyCycleNow);
    Serial.print(" Ctrl temp ");
    Serial.print(VU.data.tempMosfet);
    Serial.print(" Motor temp ");
    Serial.print(VU.data.tempMotor);
    Serial.println();
#endif
  }
  else
  {
    vesc_connected = false;
  }
}

// Update old values and send CP and CSC to BLE client
void update_chars(bool calc_power, String sType)
{
  queryVESC();

  // Update old values and send CP
  oldWheelRev = wheelRev;
  oldCrankRev = crankRev;
  oldWheelMillis = currentMillis;
  oldCrankMillis = currentMillis;
  previousMillis = currentMillis;
  fillCP();
  fillMS();

  // Some debug output to indicate what triggered the update
  Serial.print("Wheel Rev.: ");
  Serial.print(wheelRev);
  Serial.print(" WheelTime : ");
  Serial.print(lastWheeltime);
  Serial.print(" Crank Rev.: ");
  Serial.print(crankRev);
  Serial.print(" CrankTime : ");
  Serial.print(lastCranktime);
  Serial.print(" Speed : ");
  Serial.print(speed);
  Serial.print(" Cadence : ");
  Serial.print(crpm);
  Serial.print("  ");
  Serial.println(sType);

  Serial.print("Torque raw ");
  Serial.print(raw_torque);
  Serial.print("Torque avg ");
  Serial.print(torque);
  Serial.print(" (Human)Power : ");
  Serial.println(power);
}

// Interrupt routines trigger when a pulse is received (falling edge on pin)
void wheelAdd()
{
  time_now_wheel = millis();
  if (time_now_wheel > time_prev_wheel + time_chat_wheel)
  {
    // Calculate the speed in km/h based on wheel circumference
    speed = (circ * 3.6) / (time_now_wheel - time_prev_wheel);

    // Update the wheel counter and remember the time of last update
    wheelRev = wheelRev + 1;
    time_prev_wheel = time_now_wheel;
    lastWheeltime = millis() << 1;
  }
}

// Take a torque sensor reading and convert it to an instananeous torque in Nm 
float readTorque()
{
  float read_torque = analogRead(TORQUE_PIN);   

  read_torque = (read_torque - static_torque) / TORQUE_SLOPE;
  if (read_torque < 0)
    read_torque = 0;
  return read_torque;
}

// Cadence sensor triggers 32 times per crank revolution. Maintain a separate counter.
void crankAdd()
{
  time_now_crank = millis();
  if (time_now_crank > time_prev_crank + time_chat_crank) 
  {
    // Read the torque every cadence pulse and add it in to the running average buffer.
    raw_torque = readTorque();
    torqueFilter.addValue(raw_torque);

    // Calculate the cadence rpm
    // The short inter-pulse intervals may need to be added over
    // a longer time, comparable to the torque filter
    if (++crankPulsesCad > PULSES_AVG_CAD)
    {
      crankPulsesCad = 0;
      crpm = 60000.0 / (AVG_PER_REV * (time_now_crank - time_prev_crank_avg));
      time_prev_crank_avg = time_now_crank;
    }

    // If we have completed a full revolution
    if (++crankPulsesRev > PULSES_PER_REV)
    {
      crankPulsesRev = 0;
      crankRev = crankRev + 1;
      lastCranktime = millis();
    }

    time_prev_crank = time_now_crank;
  }
}


// Calculate power from peak torque and cadence. Multiply by the PAS multiplier
// and send it down to the VESC as a motor current setting.
void updateVESC()
{
  torque = torqueFilter.getMaxInBuffer();
  power = torque * crpm * (TWO_PI / 60);   // the human average power in watts

  if (vesc_connected)
  {
    // Set motor current based on power and PAS multiplier.
    
    // Sanity check the current to be between 0 and MAX_AMPS.
    req_amps = pas_mult[pas] * power / VU.data.inpVoltage;
    if (req_amps < 0)
      req_amps = 0;
    else if (req_amps > MAX_AMPS)
      req_amps = MAX_AMPS;

    // If speed limit has been exceeded, ramp down the current
    // and finally cut it off altogether.
    float lim = speed_limit / 100.0;
    if (speed > lim + speed_ramp)
      req_amps = 0;
    else if (speed > lim)
      req_amps *= 1.0 - ((speed - lim) / speed_ramp);

    VU.setCurrent(req_amps);
  }
}

  // if the wheel timer has not been updated for 4 seconds, zero out the
  // internal speed abd cadence values, so the power resets to zero.
void zero_on_inactivity()
{
  if (currentMillis > time_prev_wheel + WHEEL_INACTIVITY_INTERVAL)
  {
    speed = 0;
    power = 0;
    //Wheel.clear();
  }

  // Similarly for the crank. Zero the filter out to provide a starting ramp
  // when next started up.
  if (currentMillis > time_prev_crank + CRANK_INACTIVITY_INTERVAL)
  {
    crpm = 0;
    power = 0;
    torqueFilter.fillValue(0, FILTER_SIZE);  
    //Crank.clear();
  }
}

void setup()
{
  int count = 0;

  Serial.begin(9600);  // initialize serial communication
  Serial1.begin(115200); // Initialise serial comms with VESC

  while (!Serial)
  {
    // Be sure to break out so we don't wait forever if no serial is connected
    if (count++ > 20)
      break;
    delay(100);
  }

  // Setup VESC UART on Serial1
  VU.setSerialPort(&Serial1);

  // Initialize the LED on pin 13 to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialise BLE and establish cycling power characteristics
  BLE.begin();
  BLE.setLocalName("Farnsworth");
  BLE.setAdvertisedService(CyclePowerService);
  CyclePowerService.addCharacteristic(CyclePowerFeature);
  CyclePowerService.addCharacteristic(CyclePowerMeasurement);
  CyclePowerService.addCharacteristic(CyclePowerSensorLocation);
  BLE.addService(CyclePowerService);

  // Establish Babelfish motor measurement service and its characteristics
  motorService.addCharacteristic(motorMeasurement);
  motorService.addCharacteristic(motorSettings);
  motorService.addCharacteristic(motorNewSettings);
  BLE.addService(motorService);

  // Don't advertise the battery service; it will be found when the app connects,
  // if the app is looking for it
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);
  batteryLevelChar.writeValue(BatteryPercent);

  unsigned long t = millis();
  lastWheeltime = t << 1;       // this is in half-ms
  lastCranktime = t;
  torqueFilter.fillValue(0, FILTER_SIZE);   // fill torque running average buffer with zeroes
  Wheel.clear();
  Crank.clear();              // initialise running sum buffers

  // Write the initial values of the CP (power) characteristics
  slBuffer[0] = sensor_pos & 0xff;
  fBuffer[0] = feature_bits & 0xff;   // little endian
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x00;
  CyclePowerFeature.writeValue(fBuffer, 4);
  CyclePowerSensorLocation.writeValue(slBuffer, 1);
  fillCP();
  fillMS();

  // Attach the wheel and crank interrupt routines to their input pins
  // DONT DO THIS as the interrupts are really dodgy on the Nano 33BLE.
  // We will poll them instead.
  //attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelAdd, FALLING);
  //attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankAdd, FALLING);

  // Read the torque sensor ADC with no force on pedal.
  static_torque = analogRead(TORQUE_PIN);   
  Serial.print("Initial static torque ADC: ");
  Serial.println(static_torque);

  // Advertise that we are ready to go
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop()
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected())
    {
      connected = 1;
      currentMillis = millis();
      Wheel.poll_pin(wheelAdd);
      Crank.poll_pin(crankAdd);

      // Check if connected central has updated the writable settings
      // (PAS, speed limit, wheel circ)
      check_writable_chars();

      // Check and report the wheel and crank measurements every REPORTING_INTERVAL ms
      if (oldWheelRev < wheelRev && currentMillis - oldWheelMillis >= REPORTING_INTERVAL)
      {
        update_chars(1, "wheel");
      }
      else if (oldCrankRev < crankRev && currentMillis - oldCrankMillis >= REPORTING_INTERVAL)
      {
        update_chars(1, "crank");
      }
      else if (currentMillis - previousMillis >= REPORTING_INTERVAL)
      {
        // simulate some speed on the wheel, 500ms per rev ~16km/h, 800ms ~10km/h
        //wheelAdd();
        //crankAdd();  // don't do this, it will look very slow (only 1/32 of a rev)
        update_chars(0, "timer");
      }
      
      if (currentMillis - previousPowerCalc >= POWER_CALC_INTERVAL)
      {
        // Calculate human power from average torque and cadence. Multiply by the PAS level
        // and send it down to the VESC as motor power
        updateVESC();
        previousPowerCalc = currentMillis;
      }

      // Zero things if inactive
      zero_on_inactivity();
    }

    // when the central disconnects, turn off the LED:
    connected = 0;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");

    // Reinstate defaults in case we ride away without powering off and on
    speed_limit = 2500;
    pas = 2;
  }
  else
  {
    // No central is connected. We run with default parameters
    // (PAS level 2, 25km/h speed limit)
    currentMillis = millis();
    Wheel.poll_pin(wheelAdd);
    Crank.poll_pin(crankAdd);

    if (currentMillis - previousMillis >= REPORTING_INTERVAL)
    {
#if 0
      // simulate some speed on the wheel, 500ms per rev ~16km/h, 800ms ~10km/h
      //wheelAdd();
      //crankAdd();  // don't do this, it will look very slow (only 1/32 of a rev)

      update_chars(0, "timer");
#else
      queryVESC();      
#endif
    }

    if (currentMillis - previousPowerCalc >= POWER_CALC_INTERVAL)
    {
      // Calculate power from average torque and cadence. Multiply by the PAS multiplier
      // and send it down to the VESC
      updateVESC();
      previousPowerCalc = currentMillis;
    }

    // Zero things if inactive
    zero_on_inactivity();
  }
}
