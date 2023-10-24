/******************************************************
 * ****************************************************
 * ****  PID Discrete                             *****
 * ****  POChE                                    *****
 * ****                                           *****
 * ****  By: SERGIO ANDRES CASTAÑO GIRALDO        *****
 * ****  https://controlautomaticoeducacion.com/  *****
 * ****                                           *****
 * ****************************************************
 */
// This simulates a 20W heater block
//#include <TimerOne.h>

bool automatic = true;  // true = closed-loop, false = open-loop

const int PWM_PIN = 3;  // UNO PWM pin for Output
const int SETPOINT_PIN = A1;  // Analog pin for Setpoint Potentiometer

// Global variables for PID
double setpoint, temperature, Output = 0;
double T0 = 25.0;
float heaterWatts = 0;

float e[3] = {0, 0, 0};  // Error vector
float u[2] = {0, 0};  // Control Law vector
int kU = sizeof(u) / sizeof(float) - 1;
int kE = sizeof(e) / sizeof(float) - 1;
float kp, ti, td, q0, q1, q2;  // PID Parameters

// System model parameters
float K = 1.7, tau = 150, theta = 16;
int Ts = 1;  // Sampling period in seconds
float L = theta + Ts / 2;

// Moving average filter for ADC reading
float TemperatureFilter(float T)
{
  float S1, aux;
  int i;
  aux = 0;
  for (i = 0; i < 10; i++) {
    aux = aux + T;  // sensor
  }
  S1 = aux / 10.0;
  return (S1);
}

// Function to update past vectors
void update_past(float v[], int kT) {
  int i;
  for (i = 1; i <= kT; i++) {
    v[i - 1] = v[i];
  }
}

// PID Controller function
float PID_Controller(float u[], float e[3], float q0, float q1, float q2)
{
  float lu;
  // PID Control
  lu = u[0] + q0 * e[2] + q1 * e[1] + q2 * e[0];  // Discrete PID controller law

  // Anti-Windup
  if (lu >= 255)
    lu = 255;
  if (lu <= 0)
    lu = 0;

  return (lu);
}

// SampleTime function
void SampleTime(void)
{
  // Update vectors u and e
  update_past(u, kU);
  update_past(e, kE);

  // Compute current error
  e[kE] = setpoint - temperature;
  
  // Compute PID Control Action
  u[kU] = PID_Controller(u, e, q0, q1, q2);
  Output = u[kU];
}

// Initialize PID parameters (this may vary depending on the tuning)
void initializePID() {
  // Ziegler–Nichols Tuning
  kp = (1.2 * tau) / (K * L);
  ti = 2 * L;
  td = 0.5 * L;

  // Discrete PID Controller
  q0 = kp * (1 + Ts / (2 * ti) + td / Ts);
  q1 = -kp * (1 - Ts / (2 * ti) + (2 * td) / Ts);
  q2 = (kp * td) / Ts;
}

void setup() {
  Serial.begin(9600);
  initializePID();  // Initialize PID parameters
  /*
  if (automatic) {
    Timer1.initialize(Ts * 1000000);  // Set TIMER to 8 seconds
    Timer1.attachInterrupt(SampleTime);  // Set up interrupt to call SampleTime
  }*/
}

void loop() {
  static uint32_t last = 0;
  uint32_t interval = 1000;

  setpoint = analogRead(SETPOINT_PIN) / 4;  // Read setpoint from potentiometer

  if (automatic) {
    heaterWatts = Output * 20.0 / 255;  // 20W heater
  } else {
    heaterWatts = setpoint * 20.0 / 255;  // 20W heater
  }

  temperature = TemperatureFilter(simPlant(heaterWatts, T0));  // Sensor reading

  if (millis() - last >= interval) {
    last += interval;
    // Send data over serial port, if needed
    Serial.print(setpoint);
    Serial.print(",");
    //Serial.print(Output);
    //Serial.print(",");
    Serial.println(temperature);
    //Serial.print(",");
    if (automatic) {
      SampleTime();  // Control Law
    }
  }
}

// Simulation function for plant model
float simPlant(float Q, float T0) {
  // Simulate a 1x1x2cm aluminum block with a heater and passive ambient cooling
  float h = 5;  // W/m2K, thermal convection coefficient for Al passive
  float Cps = 0.89;  // J/g°C, specific heat capacity of Al
  float area = 1e-4;  // m2, area for convection
  float mass = 10;  // g, mass of Al block
  float Tamb = 25;  // °C, ambient temperature
  static float T = T0;  // °C, initializing temperature of Al block
  static uint32_t last = 0;  // Time variable to keep track of last temperature calculation
  uint32_t interval = 100;  // ms, time interval for temperature calculation

  // Update temperature if enough time has passed
  if (millis() - last >= interval) {
    last += interval;
    // Update temperature
    T = T + (Q * interval / 1000) / (mass * Cps) - (T - Tamb) * area * h;
  }
  return T;  // Return updated temperature
}