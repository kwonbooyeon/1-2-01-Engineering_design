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

#define _DIST_TARGET 200 
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 400 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.1 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

#define _DUTY_MIN 1370//1440// servo full clockwise position (0 degree)
#define _DUTY_NEU 1470 // servo neutral position (90 degree)
#define _DUTY_MAX 1640//1600 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 500 // servo speed limit (unit: degree/second)

#define _KD 50.0
#define _KP 0.5
// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw,dist_ema, dist_prev ,alpha ,dist_target; // unit: mm
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


// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] interval간격으로 동작 시행을 위한 정수 값
bool event_dist, event_servo, event_serial; //[3023] 적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;


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
  dist_target = _DIST_TARGET ;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

  error_prev=0;

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
  Serial.print(",pterm:");
  Serial.print(map(pterm,-1000,1000,510,610));
  Serial.print(",dterm:");
  Serial.print(map(dterm,-1000,1000,510,610));
  Serial.print(",duty_target:");
  Serial.print(map(duty_target,1000,2000,410,510));
  Serial.print(",duty_curr:");
  Serial.print(map(duty_curr,1000,2000,410,510));
  Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  

//target 설정
/*error_curr = dist_target - dist_ema; 
  pterm = _KP * error_curr;
  dterm = _KD * (error_curr - error_prev);
  point = dist_target;
//  Serial.println((dist_raw-180)/(360-180)*((_DUTY_MAX-_DUTY_MIN)/100));
  if (error_curr < 0) duty_target = _DUTY_NEU-(_DUTY_NEU-_DUTY_MIN);//(350-dist_target);
  else duty_target = _DUTY_NEU-(_DUTY_MAX-_DUTY_NEU);//(dist_target-50);
*/     
 point = dist_target;
  int pointTerm = 0;
//  Serial.println((dist_raw-180)/(360-180)*((_DUTY_MAX-_DUTY_MIN)/100));
  if (point+pointTerm<=dist_ema){
    duty_target = _DUTY_NEU-(dist_ema-(point+pointTerm))*(_DUTY_NEU-_DUTY_MIN)/(300-(point+pointTerm));
    //myservo.writeMicroseconds(_DUTY_MIN);
  }else if(point-pointTerm>dist_ema){
    duty_target = _DUTY_NEU-(dist_ema-(point-pointTerm))*(_DUTY_MAX-_DUTY_NEU)/(point-pointTerm);
    //myservo.writeMicroseconds(_DUTY_MAX);
  }// add your code here!  
//------------------


//if(event_dist) {
error_curr = dist_target - dist_ema; 
pterm = _KP * error_curr;
dterm = _KD * (error_curr - error_prev);
control = (dterm + pterm);

if (control > 0) control = control * ( _DUTY_MAX-_DUTY_NEU)/(dist_target);//(400-dist_target);
else control = control * (_DUTY_NEU - _DUTY_MIN)/(300-dist_target);//dist_target;
duty_target = duty_target + control;

// Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
// update error_prev
error_prev = error_curr;
//}


//------------------
  
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
