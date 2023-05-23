// Arduino code
#define BUZZER_COUNT 2 
#define MOTOR_COUNT 4 

int buzzers[BUZZER_COUNT] = {9, 10}; 
int motors[MOTOR_COUNT] = {5, 6, 7, 8};

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < BUZZER_COUNT; i++) {
    pinMode(buzzers[i], OUTPUT);
  }
  for(int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motors[i], OUTPUT);
  }
}


float filter_factor = 0.1; 
float smoothed_intensity = 0.0;

void loop() {
  if(Serial.available() > 0) {
    int distance = Serial.parseInt();
    int intensity = map(distance, 0, 2000, 0, 255);

    
    smoothed_intensity = (1.0 - filter_factor) * smoothed_intensity + filter_factor * intensity;

    for(int i = 0; i < BUZZER_COUNT; i++) {
      tone(buzzers[i], smoothed_intensity);
    }
    for(int i = 0; i < MOTOR_COUNT; i++) {
      analogWrite(motors[i], smoothed_intensity);
    }
  }
}
