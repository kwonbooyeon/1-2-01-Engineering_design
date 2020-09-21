unsigned int count, toggle, PIN_LED;

void setup() {
  // put your setup code here, to run once:
    PIN_LED=7;
    count=toggle=0;
    pinMode(PIN_LED,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    ++count; // 초 세기
    toggle = toggle_state(toggle); // toggle LED value
    digitalWrite(PIN_LED, toggle); // update LED status
    delay(100); // wait for 100 millisecond (깜빡임 때문에)
}

int toggle_state(int toggle) {
    if (count>20) { //2초후 빛이 꺼짐.
      toggle = 1 ;
    }
    else if (count<10) {
      toggle = 0 ; //카운트의 초반 10회(1초)간  불을 켜둠
      }
    else{
      toggle = count%2;//1초 이후에 count의 홀 짝 기준으로 0,1 반복 출력으로 깜빡임
    }
    
    return toggle ;
  }
