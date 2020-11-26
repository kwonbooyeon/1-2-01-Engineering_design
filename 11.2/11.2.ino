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

#define _DUTY_MIN 1300// servo full clockwise position (0 degree)
#define _DUTY_NEU 1470 // servo neutral position (90 degree)
#define _DUTY_MAX 1700 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw,dist_ema, dist_prev ,alpha; // unit: mm
int point;
float lastEMA=0.0;
bool ballclosing;
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
//------------
int duty_chg_per_interval; // maximum duty difference per interval
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;
//-----------
float reading;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _DUTY_NEU;
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

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
 
  
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
  Serial.print("dist_ir:");
  Serial.print(dist_ema);
  Serial.print(",duty_target:");
  Serial.print(map(duty_target,1000,2000,410,510));
  Serial.print(",duty_curr:");
  Serial.print(map(duty_curr,1000,2000,410,510));
  Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

  

//target 설정
// adjust servo position according to the USS read value
/*//공이 굴러가는 방향따라 각도바뀌는 지점을 바꾸려고 했으나 결과가 좋지 못함.
  if(lastEMA - dist_ema >= 0)ballclosing = true;
  else ballclosing = false;
  lastEMA= dist_ema;
// if (ballclosing) point = 250;*/
  point = 180;
  int pointTerm = 10;
//  Serial.println((dist_raw-180)/(360-180)*((_DUTY_MAX-_DUTY_MIN)/100));
  if (point+pointTerm<=dist_ema){
    duty_target = _DUTY_NEU-(dist_ema-(point+pointTerm))*(_DUTY_NEU-_DUTY_MIN)/(300-(point+pointTerm));
    //myservo.writeMicroseconds(_DUTY_MIN);
  }else if(point-pointTerm>dist_ema){
    duty_target = _DUTY_NEU-(dist_ema-(point-pointTerm))*(_DUTY_MAX-_DUTY_NEU)/(point-pointTerm);
    //myservo.writeMicroseconds(_DUTY_MAX);
  }// add your code here!  

  
//현재 움직임
// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }

// update servo position
  myservo.writeMicroseconds(duty_curr);


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
