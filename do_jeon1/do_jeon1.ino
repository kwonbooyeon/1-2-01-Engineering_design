unsigned int PIN_LED;
void setup() {
  PIN_LED=7;
  pinMode(PIN_LED,OUTPUT);
}

void loop() {
  Triangle(100);//주기는 Triangle()광호안쪽에 쓰시면 됩니다.
}//issue 트라이앵글 속도이상해지는거 고치기

int Triangle(float period){//밝아졌다 어두워졌다
   int duty=0;
   for (float count=0;count<2000;count+=period){
      if (count<1000)
        duty = count/10;
      else
        duty = (2000-count)/10;
      LedPWM(duty,period);
    };
  }

int LedPWM(int duty,float period){//빛 세기 조절
   for (int count=0;count<100;count++){
      if (count<duty)
        digitalWrite(PIN_LED,0);
      else 
        digitalWrite(PIN_LED,1);
        
      delayMicroseconds(period*10);
   }
}
