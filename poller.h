// Class for polling pins with noise smoothing.
#define BUF_SIZE  8
#define BUF_THRESHOLD 4

class Poller
{
public:  
  Poller(int p) { pin = p; clear(); };
  ~Poller() {};

private:
  int pin;
  int buf[BUF_SIZE];
  int _index = 0;
  int state_prev = BUF_SIZE;
  int state_now;

public:
  void clear()
  {
    _index = 0;
    for (int i = 0; i < BUF_SIZE; i++)
      buf[i] = 1;
    state_prev = BUF_SIZE;
  };

  void poll_pin(void addFn(void))
  {
    // Do a running sum. 
    // Like a running average, but no divides and with integers for speed.
    buf[_index++] = digitalRead(pin);
    if (_index == BUF_SIZE)
      _index = 0;

    state_now = 0;
    for (int i = 0; i < BUF_SIZE; i++)
      state_now += buf[i];

    if (state_now < BUF_THRESHOLD && state_prev >= BUF_THRESHOLD)
      addFn();
    
    state_prev = state_now;
  };
};
























#if 0
// Poll the wheel and crank pins. As fast as possible as it is called every loop.
void poll_wheel_crank_pins()
{
  int state_now_wheel, state_now_crank;

  // Detect a falling edge of the pin
  state_now_wheel = digitalRead(WHEEL_PIN);
  if (state_now_wheel < state_prev_wheel)
    wheelAdd();
  state_prev_wheel = state_now_wheel;

#if 0 //Simple poll as for the wheel
  state_now_crank = digitalRead(CRANK_PIN);
  if (state_now_crank < state_prev_crank)
    crankAdd();
  state_prev_crank = state_now_crank;
#else
  // Crank is a lot noisier, so do a running sum. 
  // Like a running average, but no divides and with integers for speed.
  crank_buf[crank_index++] = digitalRead(CRANK_PIN);
  if (crank_index == CRANK_BUF_SIZE)
    crank_index = 0;
  for (int i = 0; i < CRANK_BUF_SIZE; i++)
    state_now_crank += crank_buf[i];
  if (state_now_crank < CRANK_BUF_THRESHOLD && state_prev_crank >= CRANK_BUF_THRESHOLD)
    crankAdd();
  state_prev_crank = state_now_crank;
#endif
}

// Initialise the crank running sum buffer to a known state on startup.
void clear_crank_buf()
{
  crank_index = 0;
  for (int i = 0; i < CRANK_BUF_SIZE; i++)
    crank_buf[i] = 1;
  state_prev_crank = CRANK_BUF_SIZE;
}

#endif