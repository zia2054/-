#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define INTERVAL 25
#define _DIST_MIN 100
#define _DIST_MAX 300

float timeout;
float dist_min, dist_max, dist_raw;
unsigned long last_sampling_time;
float scale;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  dist_min - _DIST_MIN;
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0;
  dist_raw = 0.0;
  scale = 0.001 * 0.5 * SND_VEL;

  Serial. begin(57600);
  last_sampling_time = 0;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  Serial.print("Min:0");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("Max:400");
  float cheak = 255.0/200.0;
  if(dist_raw == 0) {
    analogWrite(PIN_LED, 255);
  }
  else if(dist_raw < 200){
    analogWrite(PIN_LED, 255-cheak*(dist_raw-100));
  }
  else{
    analogWrite(PIN_LED, cheak*(dist_raw-100)-255);
  }

  last_sampling_time += INTERVAL;  
  }
  float USS_measure(int TRIG, int ECHO)
  {
    float reading;
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    reading = pulseIn(ECHO, HIGH, timeout) * scale;
    if(reading < dist_min || reading > dist_max) reading = 0.0;
    return reading;
}
