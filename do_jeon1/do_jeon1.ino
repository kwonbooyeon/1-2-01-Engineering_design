unsigned int PIN_LED;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial){
      ;
      }//초기화
  PIN_LED=7;
  pinMode(PIN_LED,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Trangle(100);
  Serial.println("paaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
}//issue 트라이앵글 속도조절

int Trangle(float period){
   int duty=0;
   for (float count=0;count<2000;count+=period){
      if (count<1000)
        duty = count/10;
      else
        duty = (2000-count)/10;
      LedPWM(duty,period);
      Serial.println(duty);
    }
  }

int LedPWM(int duty,float period){
   for (int count=0;count<100;count++){
      if (count<duty)
        digitalWrite(PIN_LED,0);
      else 
        digitalWrite(PIN_LED,1);
        
      delayMicroseconds(period/100);
   }
}

/*
// period: 100 to 10000 (unit: us movement
int set_period(int period){ 
   for (int i=0;i<1000;i=+period){
      set_duty
      delayMicroseconds(period)
    }
  }


// duty: 0 to 100 (unit: %) lighting level
int set_duty(int duty,int period) {
  digitalWrite(PIN_LED,0)
  digitalWrite(PIN_LED,1)
  }
  //내가 이해한게 맞다면 duty는 주기동안 켜져있는 시간 퍼센트 
  //period는 1초를 몇으로 나눌것인가

*/
