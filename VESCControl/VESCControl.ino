#include <VescUart.h>

/** Initiate VescUart class */
VescUart VU;

// Number of pole-pairs in the motor 
#define NUM_POLE_PAIRS  8

int duty = 0;
void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup UART port (Serial1). Baud rate matches VESC UART setup */
  Serial1.begin(115200);

  while (!Serial) {;}

  /** Define which ports to use as UART */
  VU.setSerialPort(&Serial1);
}

void loop()
{
  // Set VESC up for current, rpm or duty.
  if (Serial.available())
  {
    char buf[16];
    int len;
    char *p;

    // Accept and act on commands from the Serial monitor.
    // Anything that is not a command just gets echoed to the serial monitor.
    while (Serial.available())
    {
      len = Serial.readBytesUntil('\n', buf, 16);
      buf[len] = '\0';
      p = buf;
      Serial.println(buf);

      switch (*p++)
      {
        case 's':
        case 'S':
          // Stop the motor.
          VU.setCurrent(0);
          duty = 0;
          break;

        case 'd':
        case 'D':
          // Set duty cycle in percent.
          if (len <= 1)
            break;
          while (isspace(*p))
            p++;
          VU.setDuty(atoi(p) / 100.0f);
          break;

        case 'i':
        case 'I':
          // Set current in amps.
          if (len <= 1)
            break;
          while (isspace(*p))
            p++;
          VU.setCurrent(atoi(p));
          break;

        case 'r':
        case 'R':
          // Set speed in RPM.
          if (len <= 1)
            break;
          while (isspace(*p))
            p++;
          VU.setRPM(atoi(p) * NUM_POLE_PAIRS);
          break;
      }
    }
  }

  /** Call the function getVescValues() to acquire data from VESC */
  if ( VU.getVescValues() )
  {
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
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  delay(100);
}
