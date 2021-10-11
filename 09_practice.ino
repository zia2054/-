#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define INTERVAL 25
#define _DIST_MIN 100
#define _DIST_MAX 300
#define _DIST_ALPHA 0.1

float timeout;
float dist_min, dist_max, dist_raw, dist_ema, alpha;
unsigned long last_sampling_time;
float scale;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO,INPUT);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0;
  dist_raw = dist_ema = 0.0;
  scale = 0.001 * 0.5 * SND_VEL;

  Serial.begin(57600);

  last_sampling_time = 0;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;

  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = (dist_raw*0.1)+0.9*(dist_ema);

  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("ema:");
  Serial.print(dist_ema);
  Serial.print(",");
  Serial.println("Max:500");

  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
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
