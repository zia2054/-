#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10 
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 245 
#define _DIST_MAX 265 

// Distance sensor
#define EMA_ALPHA 0.35 

// Servo range
#define _DUTY_MIN 1310 
#define _DUTY_NEU 1380  
#define _DUTY_MAX 2070 

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 800

// Event periods
#define _INTERVAL_DIST 10   
#define _INTERVAL_SERVO 10  
#define _INTERVAL_SERIAL 100

#define DELAY_MICROS  1200

// PID parameters
#define _KP 1.6
#define _KI 0.02
#define _KD 100

float dist_min, dist_max, dist_cali;
float samples_num = 3;
float ema_dist = 0;

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; 
float dist_ema;
float dist_raw;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist;
bool event_servo, event_serial;

// Servo speed control
float duty_chg_per_interval; 
float duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

const float coE[] = {-0.0000415, 0.0262743, -3.5600456, 233.2976919};

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 0);
 myservo.attach(PIN_SERVO);
 iterm = 0;


// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX; 
  duty_curr = _DUTY_NEU;

// move servo to neutral position
 myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); 
}
  
void loop() {
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;
  if(event_dist) {
      event_dist = false;
  // get a distance reading from the distance sensor
  
   dist_cali = ir_distance_filtered();

    
  // PID control logic
    error_curr = _DIST_TARGET - dist_cali;  
    pterm = _KP * error_curr; 
    dterm = _KD *(error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;


  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 

    error_prev = error_curr;
  
    last_sampling_time_dist = millis();
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
  }
//     update servo position
      myservo.writeMicroseconds(duty_curr);
      last_sampling_time_servo = millis();

  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_cali);
    Serial.print(",T:");
    Serial.print(_DIST_TARGET);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    last_sampling_time_serial = millis();

  }
}

float ir_distance(void){  
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}


float ir_distance_filtered(void) {
  float x = filtered_ir_distance();
  dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return dist_raw;

}
