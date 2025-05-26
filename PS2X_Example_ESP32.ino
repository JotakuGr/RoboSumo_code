#include <Arduino.h>
#include <PS2X_lib.h>  //for v1.6
/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/

//  ESP32 pin
// https://github.com/espressif/arduino-esp32/blob/master/docs/esp32_pinmap.png

#define PS2_DAT 5   //MISO  5
#define PS2_CMD 18  //MOSI  18
#define PS2_SEL 19  //SS    19
#define PS2_CLK 17  //SLK   17

#define leftm1 27
#define leftm2 26
#define rightm1 25
#define rightm2 23
#define left_EN 32
#define right_EN 16

#define pwm_Freq 500
#define pwm_Res 8

int LX = 0;
int LY = 0;
int RX_VAL = 0;
int RY = 0;
/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures true
#define rumble true

//#define pressures   false
//#define rumble      false

PS2X ps2x;  // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;

void setup() {

  // 9600
  Serial.begin(9600);

  pinMode(leftm1, OUTPUT);
  pinMode(leftm2, OUTPUT);
  pinMode(rightm1, OUTPUT);
  pinMode(rightm2, OUTPUT);

  ledcAttach(left_EN, pwm_Freq, pwm_Res);
  ledcAttach(right_EN, pwm_Freq, pwm_Res);

  //added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  ledcWrite(left_EN, 0);
  ledcWrite(right_EN, 0);

  while (error != 0) {
    delay(1000);  // 1 second wait
    //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum++;
  }

  Serial.println(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println(" Unknown Controller type found ");
      break;
    case 1:
      Serial.println(" DualShock Controller found ");
      break;
    case 2:
      Serial.println(" GuitarHero Controller found ");
      break;
    case 3:
      Serial.println(" Wireless Sony DualShock Controller found ");
      break;
  }
}
// Fona neutra
bool inNeutralZone(int val) {
  return val > 120 && val < 135;
}

// if zona neutra
bool allInNeutral() { 
  return inNeutralZone(LX) && inNeutralZone(LY) && inNeutralZone(RX_VAL) && inNeutralZone(RY);
}

void loop() {

  if (type == 1) {                      //DualShock Controller
    ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed

    //will be TRUE as long as button is pressed
    if (ps2x.Button(PSB_START))
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");

    //will be TRUE as long as button is pressed
    if (ps2x.Button(PSB_PAD_UP)) {
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if (ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_TRIANGLE))
        Serial.println("△ pressed");
    }
    //△□○×
    if (ps2x.ButtonPressed(PSB_CIRCLE))  //will be TRUE if button was JUST pressed
      Serial.println("○ just pressed");
    if (ps2x.NewButtonState(PSB_CROSS))  //will be TRUE if button was JUST pressed OR released
      Serial.println("× just changed");
    if (ps2x.ButtonReleased(PSB_SQUARE))  //will be TRUE if button was JUST released
      Serial.println("□ just released");

    if (ps2x.Button(PSB_L1)) {
      LY = ps2x.Analog(PSS_LY);  //receive values from p22 joystick
      LX = ps2x.Analog(PSS_LX);
      RY = ps2x.Analog(PSS_RY);
    }
    //if ternario
  /*(allInNeutral()) ? waithere():
      (LY < 100 && RY > 200) ? right(255) :
        (LY < 100 && RY < 100) ? left(255) :
          (LY > 200) ? REV(255) :
            (LY < 100) ? forward(255) :
              waithere();*/
    if (allInNeutral()) {
      waithere();
    }
    if (LY < 100 && RY > 200) {
      right(255);
    } else if (LY < 100 && RY < 100) {
      left(255);
    } else if (LY > 200)  //check if the joystick pushed up side
    {
      REV(255);
    } else if (LY < 100) {
      forward(255);
    }

    LY = LX = RX_VAL = RY = 128;  //return to default values
  }
  delay(50);
}

void setMotorDirection(bool l1, bool l2, bool r1, bool r2) {  //Metodo da direção dos motores(Pino, HIGH/LOW)
  digitalWrite(leftm1, l1);
  digitalWrite(leftm2, l2);
  digitalWrite(rightm1, r1);
  digitalWrite(rightm2, r2);
}


void forward(int vel) {
  Serial.println("forward");
  setMotorDirection(HIGH, LOW, HIGH, LOW);

  ledcWrite(left_EN, vel);
  ledcWrite(right_EN, vel);
}
void REV(int vel) {
  Serial.println("rev");
  setMotorDirection(LOW, HIGH, LOW, HIGH);

  ledcWrite(left_EN, vel);
  ledcWrite(right_EN, vel);
}
void left(int vel) {
  Serial.println("left");
  setMotorDirection(LOW, HIGH, HIGH, LOW);

  ledcWrite(left_EN, vel);
  ledcWrite(right_EN, vel);
}
void right(int vel) {
  Serial.println("right");
  setMotorDirection(HIGH, LOW, LOW, HIGH);

  ledcWrite(left_EN, vel);
  ledcWrite(right_EN, vel);
}
void waithere() {
  //Serial.println("stop");
  ledcWrite(left_EN, 0);
  ledcWrite(right_EN, 0);
}