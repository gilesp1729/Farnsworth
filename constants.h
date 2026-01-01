// Constants used by bicycle computer


// Pins for wheel and quadrature cadence sensors
#define WHEEL_PIN 10

// SD pin from toque/cadence sensor (PAS1 and PAS2 are not used)
#define CRANK_PIN 2

// Torque sensor (analog)
#define TORQUE_PIN A0

// Cadence pulses per revolution of crank
#define PULSES_PER_REV 32

// Number of pole-pairs in the motor (only used for debugging prints)
#define NUM_POLE_PAIRS  8

// Number of nom. 3.7V Li cells in series in the battery pack
#define NUM_SERIES_CELLS  13

// Reporting interval in ms
#define REPORTING_INTERVAL    500

// Power calculation intrval in ms
#define POWER_CALC_INTERVAL   50

// Intervals (in ms) after which sensors are considered inactive
#define WHEEL_INACTIVITY_INTERVAL   2000
#define CRANK_INACTIVITY_INTERVAL   500

// Size of moving average filter for torque readings (they are read 
// and averaged once per cadence pulse, ~30-180 ms) They need to be averaged
// over some part of a crank revolution, to smooth out dead spots and
// to provide a starting ramp.
// 16 = half a crank revolution
// 32 = a full revolution
#define FILTER_SIZE 16


