int speedRotary;
int durationRotary;
int mySpeed;
int myDuration;

/* Based on the Monster Moto Shield example from Sparkfun. If you see them, buy them a drink.

  Use the motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
  function to control the motors. Available options are CW, CCW,
  BRAKEVCC, or BRAKEGND. Use motorOff(uint8_t motor) to turn a specific motor off.

  The motor variable in each function should be a Motor enum, alternatively 0 or a 1.
  pwm in the motorGo function should be a value between 0 and 255. */

#define BRAKEVCC 0
#define CW  1
#define CCW 2
#define BRAKEGND 3

#define MOTOR_A 0
#define MOTOR_B 1

const uint8_t PWM_MAX = 255;
const uint8_t PWM_HALF = PWM_MAX / 2;

const int currentSensingThreshhold = 100;

/* Voltage controlled input pins with hysteresis, CMOS compatible. These two pins
  control the state of the bridge in normal operation according to the truth table (brake
  to VCC , brake to GND, clockwise and counterclockwise).
  Table 1.    Truth table in normal operating conditions
  +------+-----+------+-----+-------+-------+---------------------+------------------------+
  | INA  | INB | ENA  | ENB | OUTA  | OUTB  |         CS          |          MODE          |
  +------+-----+------+-----+-------+-------+---------------------+------------------------+
  |    1 |   1 |    1 |   1 | H     | H     | High Imp.           | Brake to VCC           |
  |    1 |   0 |    1 |   1 | H     | L     | I_Sense = I_OUT / K | Clockwise (CW)         |
  |    0 |   1 |    1 |   1 | L     | H     | I_SENSE = I_OUT / K | Counterclockwise (CCW) |
  |    0 |   0 |    1 |   1 | L     | L     | High Imp.           | Brake to GND           |
  +------+-----+------+-----+-------+-------+---------------------+------------------------+
  For more information, see the VNH2SP30-E motor driver datasheet */
const int inAPin[2] = {7, 4};
const int inBPin[2] = {8, 9};

/* Voltage controlled input pins with hysteresis, CMOS compatible. Gates of low side
  FETs are modulated by the PWM signal during their ON phase allowing speed
  control of the motors. */
const int pwmPin[2] = {5, 6};

/* When pulled low, disables the half-bridge of the VNH2SP30-E of the motor. In case of
  fault detection (thermal shutdown of a high side FET or excessive ON state voltage drop
  across a low side FET), this pin is pulled low by the device.
  Table 2.    Truth table in fault conditions (detected on OUTA)
  +------+-----+------+-----+-------+-------+------------+
  | INA  | INB | ENA  | ENB | OUTA  | OUTB  |     CS     |
  +------+-----+------+-----+-------+-------+------------+
  | 1    | 1   |    1 |   1 | OPEN  | H     | High Imp.  |
  | 1    | 0   |    1 |   1 | OPEN  | L     | High Imp.  |
  | 0    | 1   |    1 |   1 | OPEN  | H     | I_OUTB / K |
  | 0    | 0   |    1 |   1 | OPEN  | L     | High Imp.  |
  | X    | X   |    0 |   0 | OPEN  | OPEN  | High Imp.  |
  | X    | 1   |    0 |   1 | OPEN  | H     | I_OUTB / K |
  | X    | 0   |    0 |   1 | OPEN  | L     | High Imp.  |
  +------+-----+------+-----+-------+-------+------------+
  For more information, see the VNH2SP30-E motor driver datasheet */
const int enPin[2] = {0, 1};

/* Analog current sense input. This input senses a current proportional to the motor
  current. The information can be read back as an analog voltage. */
const int csPin[2] = {2, 3};

// On-Board LED used as status indication
const int statPin = 13;

enum State_enum {OFF, LEFT_PAUSE, RIGHT_PAUSE, LEFT_RUN, RIGHT_RUN};
uint8_t state;
unsigned long last_transition;

void motorSetup()
{
  pinMode(statPin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inAPin[i], OUTPUT);
    pinMode(inBPin[i], OUTPUT);
    pinMode(pwmPin[i], OUTPUT);
  }
  // Initialize with brake applied
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inAPin[i], LOW);
    digitalWrite(inBPin[i], LOW);
  }
}

void motorOff(uint8_t motor)
{
  // Initialize brake to Vcc
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inAPin[i], LOW);
    digitalWrite(inBPin[i], LOW);
  }
  analogWrite(pwmPin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either MOTOR_A (0) or MOTOR_B (1), will select
  which of the two motors to be controlled. Invalid values are ignored.

  mode: Should be one of the following values
  BRAKEVCC (0): Brake to VCC
  CW (1): Turn Clockwise
  CCW (2): Turn Counter-Clockwise
  BRAKEGND (3): Brake to GND

  speed: should be a value between 0 and PWM_MAX (255), higher the number, the faster
*/
void motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
{

  if (motor == MOTOR_A || motor == MOTOR_B)
  {
    switch (mode)
    {
      case BRAKEVCC: // Brake to VCC
        digitalWrite(inAPin[motor], HIGH);
        digitalWrite(inBPin[motor], HIGH);
        break;
      case CW: // Turn Clockwise
        digitalWrite(inAPin[motor], HIGH);
        digitalWrite(inBPin[motor], LOW);
        break;
      case CCW: // Turn Counter-Clockwise
        digitalWrite(inAPin[motor], LOW);
        digitalWrite(inBPin[motor], HIGH);
        break;
      case BRAKEGND: // Brake to GND
        digitalWrite(inAPin[motor], LOW);
        digitalWrite(inBPin[motor], LOW);
        break;

      default:
        // Invalid mode does not change the PWM signal
        return;
    }
    analogWrite(pwmPin[motor], speed);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.print("Tapper Program is running");
  //pins 10 and 112 are LEDs to monitor the speed and duration of the motors
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
  //pin 3 is a switch to turn it off
  pinMode(3, INPUT_PULLUP);

  state = OFF;
  motorSetup();
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
        motorOff(MOTOR_A);
        motorOff(MOTOR_B);
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
        motorOff(MOTOR_A);
        motorGo(MOTOR_B, CW, PWM_MAX);
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
        motorGo(MOTOR_A, CW, PWM_MAX);
        motorOff(MOTOR_B);
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
        motorOff(MOTOR_A);
        motorOff(MOTOR_B);
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
        motorOff(MOTOR_A);
        motorOff(MOTOR_B);
      }
      break;
  }
}
