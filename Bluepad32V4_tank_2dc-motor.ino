#include <Bluepad32.h>
// Related PCB: TANK_ESP32_D1_MINI_LM317
// Motor driver: Pololu DRV8835 dual motor driver
//   mode=1 (phase/enable)
//   phase enable out1 out2  operating mode
//     0    pwm   pwm  L      forward/brake at speed pwm %
//     1    pwm   L    pwm    reverse/brake at speed pwm %
//     x    0     L    L      brake low

// This code is optimized for Mini4WD Wild LunchBox with 2 DC motor

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Assigning ports
const int pinMotorPhaseL = 22;  // Motor phase M1 (FWD = 0, BWD = 1) (digitial output)
const int pinMotorPWML = 25;    // Motor speed M1 control (PWM) (analog output)
const int pinMotorPhaseR = 32;  // Motor phase M2 (FWD = 0, BWD = 1) (digitial output)
const int pinMotorPWMR = 27;    // Motor speed M2 control (PWM) (analog output)

const int pinLEDhead = 0;    // Headlight (digitial output) (Need to rework for V1.0)
const int pinLEDbrake = 18;  // Brake light (digitial output)
const int pinLEDleft = 10;   // Left turn light (digitial output)
const int pinLEDright = 21;  // Right turn light (digitial output)
const int pinLEDback = 23;   // Backward light (digitial output)
const int pinLEDaux1 = 5;    // Aux. light #1 (digitial output)
const int pinLEDaux2 = 17;   // Aux. light #2 (digitial output)
const int pinLEDaux3 = 16;   // Aux. light #3 (digitial output)
const int pinLEDconn = 2;    // LED onboard (digital output)

const int pinVsense = 13;  // Input voltage sensor (anaolog input)

const float refVoltIN = 5.0;  // input battery voltage reference


// define LED mode
int EMERGENCY = LOW;       // Emergency mode
int LT_TURN = LOW;         // Left turn enable/disable
int RT_TURN = LOW;         // Right turn enable/disable
int stateLED_brake = LOW;  // Brake state
int stateLED_LT = LOW;     // Left turn signal state
int stateLED_RT = LOW;     // Right turn signal state
int stateLED_head = LOW;   // Headlight state
int stateLED_back = LOW;   // Backward light state
int stateLED_AUX1 = LOW;  // Aux. light #1 state  license plate light (works with headlight)
int stateLED_AUX2 = LOW;  // Aux. light #2 state  fog light
int stateLED_AUX3 = LOW;  // Aux. light #3 state  roof top light

// define time variables
unsigned long timePrev = 0;
unsigned long timePrev2 = 0;

unsigned long timePrevLoop = 0;  // loop 실행 간격을 조절하기 위한 타이머
const int LOOP_INTERVAL = 10;    // loop 주기를 10ms로 설정

// 진동을 지속적으로 유지하기 위한 변수
unsigned long timePrevRumble = 0;
const int INTERVAL_RUMBLE = 100;  // 진동 업데이트 간격 (50ms)

uint8_t weakRumble;

int maxSpeed;               // 255 is the max.
const long INTERVAL = 500;  // for LED blinking
const int delayBtn = 500;

// stop all DC motors
void stopAll() {
  setMotorL(0, 0);
  setMotorR(0, 0);
  digitalWrite(pinLEDbrake, LOW);
}

// -------------- start of function definition --------------
// speed control DC motor
void controlMotor(int Accel, int Y, int pinMotorPhase, int pinMotorPWM) {
  int speed;
  int mPhase;
  if (Y > 10) {  // stick lower (go backward)
    mPhase = HIGH;
    //speed = abs(Y / 2);
    speed = Accel / 4;
  } else if (Y < -10) {  // stick upper (go forward)
    mPhase = LOW;
    //speed = abs(Y / 2);
    speed = Accel / 4;
  } else {
    mPhase = LOW;
    speed = 0;
  }
  if (speed == 256) speed = 255;
  digitalWrite(pinMotorPhase, mPhase);
  analogWrite(pinMotorPWM, speed);
  Serial.printf("Phase %d/%3d speed %d/%3d Rumble %d", pinMotorPhase, mPhase, pinMotorPWM, speed, weakRumble);
}

// control DC motor - left
void setMotorL(int Accel, int Y) {
  Serial.print("L: ");
  controlMotor(Accel, Y, pinMotorPhaseL, pinMotorPWML);
}

// control DC motor - right
void setMotorR(int Accel, int Y) {
  Serial.print("R: ");
  controlMotor(Accel, Y, pinMotorPhaseR, pinMotorPWMR);
}

// control LED
void updateLEDState(int &stateLED, unsigned long &prevTime, int pin, int interval) {
  if (millis() - prevTime >= interval) {
    //    prevTime = timeNow;
    stateLED = !stateLED;
  }
  digitalWrite(pin, stateLED);
}

void turnSignalControl() {
  if (EMERGENCY == HIGH) {  // Emergency mode
    if (millis() - timePrev >= INTERVAL) {
      timePrev = millis();
      stateLED_LT = !stateLED_LT;
      stateLED_RT = !stateLED_RT;
    }
  } else {  // Normal mode
    if (LT_TURN == HIGH) {
      if (millis() - timePrev >= INTERVAL) {
        timePrev = millis();
        stateLED_LT = !stateLED_LT;
        stateLED_RT = LOW;
      }
    } else stateLED_LT = LOW;
    if (RT_TURN == HIGH) {
      if (millis() - timePrev >= INTERVAL) {
        timePrev = millis();
        stateLED_LT = LOW;
        stateLED_RT = !stateLED_RT;
      }
    } else stateLED_RT = LOW;
  }
  digitalWrite(pinLEDleft, stateLED_LT);
  digitalWrite(pinLEDright, stateLED_RT);
  // Serial.printf(" Light_L: %d, Light_R: %d", stateLED_LT, stateLED_RT);
}

void toggleLight(int pinLED, int &stateLED) {
  stateLED = !stateLED;
  digitalWrite(pinLED, stateLED);
}

void backwardLightControl(int LY, int RY) {
  if (LY > 10 && RY > 10) {
    stateLED_back = HIGH;
  } else stateLED_back = LOW;
  digitalWrite(pinLEDback, stateLED_back);
  ;
}

void voltageSense(int voltageIN) {
  int R1 = 1000;
  int R2 = 2000;
  float refVolt = 3.3;
  float Vsense = refVolt * R1 / (R1 + R2);
}

void donothing() {
  // do nothing
}


// -------------- end of function definition --------------

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      // 🔹 연결 시 짧게 진동 (약한 모터 사용, 300ms 동안)
      ctl->playDualRumble(0, 300, 80, 0);  // 약한 모터 80 강도로 300ms 동안 진동

      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }

  digitalWrite(pinLEDconn, HIGH);
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
  digitalWrite(pinLEDconn, LOW);
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

/*
void dumpMouse(ControllerPtr ctl) {
    Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    // TODO: Print pressed keys
    Serial.printf("idx=%d\n", ctl->index());
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}
*/

void processGamepad(ControllerPtr ctl) {
  int LX = ctl->axisX();
  int LY = ctl->axisY();
  int RX = ctl->axisRX();
  int RY = ctl->axisRY();
  int Acc = ctl->throttle();

  bool btn_x = ctl->a();   // button 'X (down)' on PS controller
  bool btn_o = ctl->b();   // button 'O (right)' on PS controller
  bool btn_sq = ctl->x();  // button '□ (left)' on PS controller
  bool btn_tr = ctl->y();  // button '△ (up)' on PS controller
  bool btn_l1 = ctl->l1();
  bool btn_r1 = ctl->r1();
  bool btn_thumbL = ctl->thumbL();
  bool btn_thumbR = ctl->thumbR();

  if ((btn_x) && (millis() - timePrev2 > delayBtn)) {  // toggle fog light
    toggleLight(pinLEDaux2, stateLED_AUX2);
    timePrev2 = millis();
  }
  if ((btn_o) && (millis() - timePrev2 > delayBtn)) {  // toggle headlight
    stateLED_AUX1 = stateLED_head;
    toggleLight(pinLEDhead, stateLED_head);  // toggle headlight
    toggleLight(pinLEDaux1, stateLED_AUX1);  // toggle license plate light
    timePrev2 = millis();
  }
  if ((btn_sq) && (millis() - timePrev2 > delayBtn)) {  // toggle rooftop light
    toggleLight(pinLEDaux3, stateLED_AUX3);
    timePrev2 = millis();
  }
  if ((btn_tr) && (millis() - timePrev2 > delayBtn)) {  // toggle emergency mode
    EMERGENCY = !EMERGENCY;
    timePrev2 = millis();
  }

  if (Acc < 10) stateLED_brake = HIGH;
  else stateLED_brake = LOW;
  digitalWrite(pinLEDbrake, stateLED_brake);

  setMotorL(Acc, LY);
  setMotorR(Acc, RY);
  backwardLightControl(LY, RY);

  // LY upper, RY 0 or lower --> Right turn
  if (LY < -10 && RY >= 0) {
    RT_TURN = HIGH;
  } else RT_TURN = LOW;
  // LY 0 or lower, RY upper --> Left turn
  if (LY >= 0 && RY < -10) {
    LT_TURN = HIGH;
  } else LT_TURN = LOW;
  turnSignalControl();

  // 🔹 Acc 값에 따라 진동 조절 🔹
  if (abs(LY) > 10 && abs(RY) > 10 & Acc > 10) {
    weakRumble = map(Acc, 0, 1023, 0, 16);  // 약한 모터: 0~128

    // 진동이 끊기지 않도록 주기적으로 업데이트
    if (millis() - timePrevRumble >= INTERVAL_RUMBLE) {
      timePrevRumble = millis();

      if (Acc < 10) {
        weakRumble = 0;  // 정지 시 진동 없음
      } else {
        ctl->playDualRumble(0, INTERVAL_RUMBLE, weakRumble, 0);  // 약한 모터만 지속적으로 동작
      }
    }
  }

  Serial.printf(" HL:%d\t BL:%d\t BR:%d\t LL:%d\t RL:%d\t A1:%d\t A2:%d\t A3:%d\n", stateLED_head, stateLED_back, stateLED_brake, stateLED_LT, stateLED_RT, stateLED_AUX1, stateLED_AUX2, stateLED_AUX3);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
        /*
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
    */
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

  // Defining pin mode for I/O
  pinMode(pinLEDhead, OUTPUT);
  pinMode(pinLEDbrake, OUTPUT);
  pinMode(pinLEDleft, OUTPUT);
  pinMode(pinLEDright, OUTPUT);
  pinMode(pinLEDback, OUTPUT);
  pinMode(pinLEDaux1, OUTPUT);
  pinMode(pinLEDaux2, OUTPUT);
  pinMode(pinLEDaux3, OUTPUT);
  pinMode(pinLEDconn, OUTPUT);

  pinMode(pinMotorPWML, OUTPUT);
  pinMode(pinMotorPhaseL, OUTPUT);
  pinMode(pinMotorPWMR, OUTPUT);
  pinMode(pinMotorPhaseR, OUTPUT);

  pinMode(pinVsense, INPUT);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop. unsigned long timeNow = millis();
  if (millis() - timePrevLoop >= LOOP_INTERVAL) {
    timePrevLoop = millis();

    // 블루투스 컨트롤러 업데이트
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
      processControllers();
    }
  }

  // 블루투스 작업이 끊기지 않도록 적절한 딜레이 유지
  vTaskDelay(1);  // 1ms 정도만 CPU 양보
}
