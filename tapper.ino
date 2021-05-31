int speedRotary;
int durationRotary;
int mySpeed;
int myDuration;


enum State_enum {OFF, LEFT_PAUSE, RIGHT_PAUSE, LEFT_RUN, RIGHT_RUN};
uint8_t state;
unsigned long last_transition;

void setup() {
  Serial.begin(9600);
  Serial.print("Tapper Program is running");
  //pins 10 and 112 are LEDs to monitor the speed and duration of the motors
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
  //pin 3 is a switch to turn it off
  pinMode(3, INPUT_PULLUP);

  state = OFF;
}

void loop() {
  bool pow_sw = digitalRead(3);
  if (pow_sw) {
    Serial.println("pow_sw is TRUE");
  }
  else {
    Serial.println("pow_sw is FALSE");
  }

  //Speed - how fast the tappers alternate
  speedRotary = analogRead(4);

  //maps the speed from zero to 2 seconds
  mySpeed = map(speedRotary, 0, 1023, 0, 2000);
  Serial.print("Current Speed: ");
  Serial.println(mySpeed);

  //Duration - how long the tappers are active for
  durationRotary = analogRead(5);

  //maps the duration from zero to 2 seconds
  myDuration = map(durationRotary, 0, 1023, 0, 2000);
  Serial.print("Current Duration: ");
  Serial.println(myDuration);

  Serial.print("State: ");
  Serial.println(state);

  switch (state) {
    case OFF:
      if (pow_sw) {
        state = LEFT_RUN;
        last_transition = millis();
      }
      else
      {
        digitalWrite(10, LOW);
        digitalWrite(12, LOW);
      }
      break;

    case LEFT_RUN:
      if (!pow_sw) {
        state = OFF;
        break;
      }
      if ((millis() - last_transition) > mySpeed)
      {
        state = LEFT_PAUSE;
        last_transition = millis();
      }
      else
      {
        digitalWrite(10, LOW);
        digitalWrite(12, HIGH);
      }
      break;

    case RIGHT_RUN:
      if (!pow_sw) {
        state = OFF;
        break;
      }
      if ((millis() - last_transition) > mySpeed)
      {
        state = RIGHT_PAUSE;
        last_transition = millis();
      }
      else
      {
        digitalWrite(10, HIGH);
        digitalWrite(12, LOW);
      }
      break;

    case RIGHT_PAUSE:
      if (!pow_sw) {
        state = OFF;
        break;
      }
      if ((millis() - last_transition) > myDuration)
      {
        state = LEFT_RUN;
        last_transition = millis();
      }
      else
      {
        digitalWrite(10, LOW);
        digitalWrite(12, LOW);
      }
      break;

    case LEFT_PAUSE:
      if (!pow_sw) {
        state = OFF;
        break;
      }
      if ((millis() - last_transition) > myDuration)
      {
        state = RIGHT_RUN;
        last_transition = millis();
      }
      else
      {
        digitalWrite(10, LOW);
        digitalWrite(12, LOW);
      }
      break;
  }
}
