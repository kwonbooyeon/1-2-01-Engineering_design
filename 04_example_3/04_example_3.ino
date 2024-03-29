#define PIN_LED 13
unsigned int count, toggle;
void setup() {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);//Initialize serial port
    while(!Serial){
      ;//wait for serial port to connect.
      }//초기화
    Serial.println("HelloWorld!");
    count=toggle=0;
    digitalWrite(PIN_LED, toggle);// turn off LED
}

void loop() {
    Serial.println(++count);
    toggle = toggle_state(toggle); // toggle LED value
    digitalWrite(PIN_LED, toggle); // update LED status
    delay(1000); // wait for 1,000 milliseconds
}

int toggle_state(int toggle) {
    toggle = count%2; //count의 홀 짝 기준으로 0,1 반복 출력
    return toggle ;
  }
