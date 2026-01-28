// Constants used by Farnsworth to interface to the VESC.

// Pins for wheel and cadence sensors
#define WHEEL_PIN 10

// SD pin from toque/cadence sensor (PAS1 and PAS2 are not used)
#define CRANK_PIN 2

// Torque sensor (analog) pin.
#define TORQUE_PIN A0

// Torque slope (ADC counts per Nm) Determined by experiment
// to reliably produce progressive torque values in Nm.
#define TORQUE_SLOPE  6.8

// Maxiumum amps sent to VESC. Should be <= the max current set in the VESC.
#define MAX_AMPS      12

// Cadence pulses per revolution of crank at the SD (speed/direction) line.
// The quadrature outputs (not used here) may be at this rate or half this rate.
// Some analog sensors gives 32 pulses/rev.
#define PULSES_PER_REV  46

// Cadence pulses per cadence averaging interval.
// Does not have to be a divisor of PULSES_PER_REV, but should be between 1/4 and 1/2.
// The number of averaging intervals per rev is precalculated here.
#define PULSES_AVG_CAD  12
#define AVG_PER_REV     ((float)PULSES_PER_REV / PULSES_AVG_CAD)

// Number of pole-pairs in the motor (only used for debugging prints)
#define NUM_POLE_PAIRS  5

// Number of nom. 3.7V Li cells in series in the battery pack
#define NUM_SERIES_CELLS  13

// Reporting interval in ms
#define REPORTING_INTERVAL    500

// Power calculation intrval in ms. 
#define POWER_CALC_INTERVAL   100

// Intervals (in ms) after which sensors are considered inactive
#define WHEEL_INACTIVITY_INTERVAL   2000
#define CRANK_INACTIVITY_INTERVAL   500

// Size of moving average filter for torque readings (they are read 
// once per cadence pulse, ~30-180 ms) They need to be collected
// over some part of a crank revolution, to smooth out dead spots and
// to provide a starting ramp. The maximum torque is taken for power
// calculations.
#define FILTER_SIZE 23


