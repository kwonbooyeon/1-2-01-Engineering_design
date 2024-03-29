#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13
int analogPin =0;

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 400 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.1 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

#define _DUTY_MIN 770 // servo full clockwise position (0 degree)
#define _DUTY_NEU  1400// servo neutral position (90 degree)
#define _DUTY_MAX 830 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw,dist_ema, dist_prev ,alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
Servo myservo;

float reading;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = alpha*dist_raw+(1-alpha)*dist_ema;
  
// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());
  Serial.print(",val:");
  Serial.print(ir_distance());
  Serial.println(",Max:400");

// adjust servo position according to the USS read value

 
//  Serial.println((dist_raw-180)/(360-180)*((_DUTY_MAX-_DUTY_MIN)/100));

 if (dist_ema<200){
  myservo.writeMicroseconds(_DUTY_MAX);
  }
  else{
  myservo.writeMicroseconds(_DUTY_MIN);
  }// add your code here!  
// update last sampling time
  last_sampling_time += INTERVAL;

//LED
  if (360>=dist_ema and dist_ema>=180) digitalWrite(PIN_LED, LOW);
  else digitalWrite(PIN_LED, HIGH);

}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float readingTest;
/*  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  readingTest = pulseIn(ECHO, HIGH, timeout) * scale;// unit: mm
*/
  readingTest = ir_distance(); 
  if(readingTest!=0.0){
    if(readingTest < dist_min || readingTest > dist_max) reading = 0.0;// return 0 when out of range.
    else reading = readingTest;
  }  
  return reading;
}

//적외선센서
float ir_distance(void){// return value unit: mm
  float val;
  float volt =analogRead(analogPin);
  val = ((6762.0/(volt-9.0))-4.0) *10.0;
  return val;
  }
