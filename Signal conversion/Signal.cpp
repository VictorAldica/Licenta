// code for arduino
int buzzer = 9; // buzzer
int motorPin = 10; // motor

void setup() {
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  if(Serial.available() > 0) {
    int distance = Serial.parseInt();
    int intensity = map(distance, 0, 2000, 0, 255); 
    analogWrite(motorPin, intensity);
    tone(buzzer, intensity);
  }
}
