#include <Adafruit_LSM9DS0.h>
unsigned long tic;
double roll_est = 0;
double pitch_est = 0;
double roll_meas = 0;
double pitch_meas = 0;
double bias_p = 0;
double bias_q = 0;

#define sampleTime 0.004
#define PIN_CLK 6
#define PIN_MOSI 7
#define PIN_CS_G 8
#define PIN_CS_XM 9
#define PIN_MISO 10

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(PIN_CLK, PIN_MISO, PIN_MOSI, PIN_CS_XM, PIN_CS_G);


void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}


void setup() {
  Serial.begin(115200);

  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }

  //set timer2 interrupt at 100 Hz
  noInterrupts();
  TCCR2A = 0;                           // Set entire TCCR0A register to 0
  TCCR2B = 0;                           // Same for TCCR0B
  TCNT2  = 0;                           // Initialize counter value to 0
  OCR2A  = 115;                         // 16e06/preScaler/desRate) -1
  TCCR2A |= (1 << WGM21);               // Turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // Set CS22, CS21 and CS20 for 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);              // Enable timer compare interrupt
  interrupts();
}



// Define control loop as interrupt service routine
ISR(TIMER2_COMPA_vect){

  // Get measurement
  lsm.readAccel();
  lsm.readGyro();

  double p = lsm.gyroData.x*0.00875 * PI/180;
  double q = lsm.gyroData.y*0.00875 * PI/180;

  double udot = -lsm.accelData.x*0.061/1000*9.81;
  double vdot = -lsm.accelData.y*0.061/1000*9.81;
  double wdot = -lsm.accelData.z*0.061/1000*9.81;


  // Inclinometer
  roll_meas = atan2(-vdot, -wdot);
  pitch_meas = atan2(udot, sqrt( sq(vdot) + sq(wdot) ));

  // Kalman filter phi
  double delta_roll = roll_meas - roll_est;
  roll_est += sampleTime*(p-bias_p) + 0.0258*delta_roll;
  bias_p += -0.0221*delta_roll;

  // Kalman filter theta
  double delta_pitch = pitch_meas - pitch_est;
  pitch_est += sampleTime*(q-bias_q) + 0.0258*delta_pitch;
  bias_q += -0.0221*delta_pitch;


  //Serial.println(millis());
  Serial.print(F("Orientation: "));
  Serial.print(roll_est*180/PI);
  Serial.print(F(" "));
  Serial.print(pitch_est*180/PI);
  Serial.print(F(" "));
  Serial.print(0);
  Serial.println(F(""));

  //Serial.println(String(micros()) + ";" + String(lsm.accelData.x*0.061/1000*9.81) + ";" + String(lsm.accelData.y*0.061/1000*9.81) + ";" + String(lsm.accelData.z*0.061/1000*9.81) + ";" + String(lsm.gyroData.x*0.00875) + ";" + String(lsm.gyroData.y*0.00875) + ";" + String(lsm.gyroData.z*0.00875));
}


void loop() {
  /*
  zdot  = lsm.accelData.z*0.061/1000*9.81;
  p     = lsm.gyroData.x*0.00875;
  */
}
