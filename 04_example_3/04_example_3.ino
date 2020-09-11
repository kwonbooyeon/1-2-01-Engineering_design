#define PIN_LED 13
unsigned int count, toggle;
void setup() {
  // put your setup code here, to run once:
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);
    while(!Serial){
      ;
      }//초기화
    Serial.println("HelloWorld!");
    count=toggle=0;
    digitalWrite(PIN_LED, toggle);
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println(++count);
    toggle = toggle_state(toggle); 
    digitalWrite(PIN_LED, toggle);
    delay(1000);
}

int toggle_state(int toggle) {
    toggle = count%2;
    return toggle;
  }
