#include <Arduino.h>
#include <DDT_Motor_M15M06.h>
#include <BluetoothSerial.h>

static const int Empty = 0;
static const int Start = 1;
static const int ArrowPressUp = 2;
static const int ArrowPressDown = 3;
static const int ArrowPressLeft = 4;
static const int ArrowPressRight = 5;
static const int ArrowPressCenter = 6;
static const int ArrowOut = 7;
static const int ButtonPressA = 8;
static const int ButtonPressB = 9;
static const int ButtonPressC = 10;
static const int ButtonPressD = 11;
static const int ButtonPressE = 12;
static const int ButtonPressF = 13;
static const int ButtonPressG = 14;
static const int ButtonPressH = 15;
static const int ButtonPressI = 16;
static const int ButtonPressJ = 17;
static const int ButtonPressK = 18;
static const int ButtonPressL = 19;
static const int ButtonPressM = 20;
static const int ButtonPressN = 21;
static const int ButtonPressO = 22;
static const int ButtonPressP = 23;
static const int ButtonPressQ = 24;
static const int ButtonPressR = 25;
static const int ButtonPressS = 26;
static const int ButtonPressT = 27;
static const int ButtonPressU = 28;
static const int ButtonPressV = 29;
static const int ButtonPressW = 30;
static const int ButtonPressX = 31;
static const int ButtonPressY = 32;
static const int ButtonPressZ = 33;
static const int ButtonOut = 34;
struct Action {
    int id;
    String command;
};
Action ACTIONS[] = {
    { Empty, "__" },
    { Start, "START" },
    { ArrowPressUp, "ARROW_PRESS_UP" },
    { ArrowPressDown, "ARROW_PRESS_DOWN" },
    { ArrowPressLeft, "ARROW_PRESS_LEFT" },
    { ArrowPressRight, "ARROW_PRESS_RIGHT" },
    { ArrowPressCenter, "ARROW_PRESS_CENTER" },
    { ArrowOut, "ARROW_OUT" },
    { ButtonPressA, "BUTTON_PRESS_A" },
    { ButtonPressB, "BUTTON_PRESS_B" },
    { ButtonPressC, "BUTTON_PRESS_C" },
    { ButtonPressD, "BUTTON_PRESS_D" },
    { ButtonPressE, "BUTTON_PRESS_E" },
    { ButtonPressF, "BUTTON_PRESS_F" },
    { ButtonPressG, "BUTTON_PRESS_G" },
    { ButtonPressH, "BUTTON_PRESS_H" },
    { ButtonPressI, "BUTTON_PRESS_I" },
    { ButtonPressJ, "BUTTON_PRESS_J" },
    { ButtonPressK, "BUTTON_PRESS_K" },
    { ButtonPressL, "BUTTON_PRESS_L" },
    { ButtonPressM, "BUTTON_PRESS_M" },
    { ButtonPressN, "BUTTON_PRESS_N" },
    { ButtonPressO, "BUTTON_PRESS_O" },
    { ButtonPressP, "BUTTON_PRESS_P" },
    { ButtonPressQ, "BUTTON_PRESS_Q" },
    { ButtonPressR, "BUTTON_PRESS_R" },
    { ButtonPressS, "BUTTON_PRESS_S" },
    { ButtonPressT, "BUTTON_PRESS_T" },
    { ButtonPressU, "BUTTON_PRESS_U" },
    { ButtonPressV, "BUTTON_PRESS_V" },
    { ButtonPressW, "BUTTON_PRESS_W" },
    { ButtonPressX, "BUTTON_PRESS_X" },
    { ButtonPressY, "BUTTON_PRESS_Y" },
    { ButtonPressZ, "BUTTON_PRESS_Z" },
    { ButtonOut, "BUTTON_OUT" },
};


// ---- S/W Version ------------------
#define VERSION_NUMBER  "  Ver. 1.3.1"
// -----------------------------------

////////PIN番号////////

const int TX_PIN = 27;
const int RX_PIN = 26;

//////////////////////


// int16_t forwardMaxSpeed = 100; //前進時の最大速度
int16_t forwardMaxSpeed = 60; //前進時の最大速度 //OriHime-TX仕様
int16_t backwardMaxSpeed = 50;  //後退時の最大速度
int16_t rotationMaxSpeed = 20;  //回転時の最大速度
int16_t currentForwardSpeed = 80;
int16_t currentTurnSpeed = 80;
int16_t currentSpeed = 0;   // currentSpeed of motor
uint8_t Acce = 0;    // Acceleration of motor
uint8_t Brake_P = 0; //0xffを入れるとブレーキが入る その場でブレーキが入る// Brake position of motor
uint8_t leftMotorID = 1;      // ID of Motor (default:1)
uint8_t rightMotorID = 2;

const int fwRunAdd = 5; // 平地走行モードでの回転速度の上昇数 前進
const int runAdd = 5; //平地走行モードでの回転速度の上昇数 バック
const int fwTurnAdd = 3;
const int dashDeceleration = 2;    //isQuickDashActiveからの減速値  MAX:100の時は4 MAX:80の時は2a
const int turnDeceleration = 5;
const int turnAdd = 3; // 平地走行モードでの回転速度の上昇数 左右回転
const int fwBrakeAdd = 4; //前進での回転速度の減少数
const int brakeAdd = 3; //平地走行モードでの回転速度の減少数 30

int modeDDT = 0; //0:停止 1:前進 2:後進 3:右回転 4:左回転 5:右斜前 6:左斜前 7:右斜後 8:左斜後
int lastModeDDT = 0;

int spendTime = 0;
const int changeModeTime = 20;

int operationMode = 2;  //1:直進モードあり  2:直進モードなし

bool rightMotorActive = false;
bool leftMotorActive = false;


bool isForwardButtonPressed = false;  //true:前進ボタンを押した false:前進ボタンを離した

bool isQuickDashActive = false; //前進しながらQキーを押すと加速する
bool wasQuickDashActive = false;

bool isForwardSpeedLimited = false;


BluetoothSerial SerialBT;

Receiver Receiv;

// M5Stackのモジュールによって対応するRX,TXのピン番号が違うためM5製品とRS485モジュールに対応させてください
auto motor_handler = MotorHandler(RX_PIN, TX_PIN); // RX,TX

const int16_t currentSpeed_MAX = 330;
const int16_t currentSpeed_MIN = -330;


void moveDDT(int id, int velocity){
    motor_handler.Control_Motor(velocity, id, Acce, Brake_P, &Receiv);
  delay(18);    //←いじらない
}




void straightenFace(){  //OriHimeの顔を正面に向ける
  Serial.println("SET_ORIHIME_HEAD_0.5,0.5"); // 頭
}

void currentSpeedUp(){
    isQuickDashActive = true;
    Serial.println("Speed UP!!");

    // forwardMaxSpeed = 160;    //OriHime-T仕様
    forwardMaxSpeed = 120;   //OriHime-TX仕様
    backwardMaxSpeed = 80;
    rotationMaxSpeed = 40;
    currentForwardSpeed = forwardMaxSpeed;
}

void currentSpeedDown(){
    isQuickDashActive = false;
    wasQuickDashActive = true;
    Serial.println("Speed Down!!");    

    // forwardMaxSpeed = 80;   //OriHime-T仕様
    forwardMaxSpeed = 60;    //OriHime-TX仕様
    backwardMaxSpeed = 50;
    rotationMaxSpeed = 20;
}



void setup() {
    // delay(3000);
    Serial.begin(19200);
    Serial2.begin(115200);
    SerialBT.begin("OryArm", true); 
    Serial.println("The device started in master mode, make sure remote BT device is on!");
    
    // Attempt to connect to the specified device
    if(SerialBT.connect("OryArm")) { // ここで直接デバイス名を指定
        Serial.println("Connected Successfully!");
    } else {
        Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
      // No need to disconnect here; just attempt to reconnect or prompt for manual restart
    }

    
    motor_handler.Control_Motor(0, leftMotorID, Acce, Brake_P, &Receiv);
    motor_handler.Control_Motor(0, rightMotorID, Acce, Brake_P, &Receiv);
    
    Serial.println(VERSION_NUMBER);
}



Action checkAction(String command) {
    command.trim();
    for (int i = 0; i < sizeof(ACTIONS); i += 1) {
        if (command == ACTIONS[i].command) {
        return ACTIONS[i];
        }
    }
    return ACTIONS[0];
}



void loop() {
    // Serial.println(spendTime);

    delay(5);

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        Action action = checkAction(command);
        if (action.id == 0) return;


        if (action.id == ArrowPressUp) {
            isForwardButtonPressed = true;
            if (modeDDT == 3){
                Serial.println("右斜前");
                modeDDT = 5;
                delay(10);
            }
            else if (modeDDT == 4){
                Serial.println("左斜前");
                modeDDT = 6;
                delay(10);
            }
            else {
                Serial.println("前進");
                modeDDT = 1;
                lastModeDDT = 1;
                straightenFace();
                delay(10);
            }
            
        }


        if (action.id == ArrowPressDown) {
            if (modeDDT == 3){
                Serial.println("右斜後");
                modeDDT = 7;
                delay(10);
            }
            else if (modeDDT == 4){
                Serial.println("左斜後");
                modeDDT = 8;
                delay(10);
            }
            else {
                Serial.println("後退");
                modeDDT = 2;
                delay(10);
            }
        }


        if (action.id == ArrowPressRight) {
            if(modeDDT == 1){
                Serial.println("右斜前");
                modeDDT = 5;
                // lastModeDDT = 5;
                delay(10);
            }
            else if (modeDDT == 2){
                Serial.println("右斜後");
                modeDDT = 7;
                delay(10);
            }
            else{
                Serial.println("右回転");
                modeDDT = 3;
                lastModeDDT = 3;
                delay(10);
            }
        }


        if (action.id == ArrowPressLeft) {
            if(modeDDT == 1){
                Serial.println("左斜前");
                modeDDT = 6;
                // lastModeDDT = 6;
                delay(10);
            }
            else if (modeDDT == 2){
                Serial.println("左斜後");
                modeDDT = 8;
                delay(10);
            }
            else{
                Serial.println("左回転");
                modeDDT = 4;
                lastModeDDT = 4;
                delay(10);
            }
        }


        if (action.id == ArrowOut) {
            isForwardButtonPressed = false;
            spendTime = 0;
            if (isQuickDashActive == true){
                currentSpeedDown();
            } else {
                wasQuickDashActive = false;
                isForwardSpeedLimited = false;
                if (modeDDT == 1){
                    SerialBT.write(10);
                }

                if (operationMode == 1){
                    if(modeDDT == 0 || modeDDT == 2 || modeDDT == 3 || modeDDT == 4){
                        modeDDT = 0;
                        delay(10);
                    }
                    else if (modeDDT == 5 || modeDDT == 6){
                        modeDDT = 1;
                        lastModeDDT = 1;
                        delay(10);
                    }
                    else if (modeDDT == 7 || modeDDT == 8){
                        modeDDT = 2;
                        delay(10);
                    }
                }
                else if (operationMode == 2){
                    if(modeDDT == 0 || modeDDT == 1 || modeDDT == 2 || modeDDT == 3 || modeDDT == 4){
                        modeDDT = 0;
                        lastModeDDT = 0;
                        delay(10);
                    }
                    else if (modeDDT == 5 || modeDDT == 6){
                        modeDDT = 1;
                        delay(10);
                    }
                    else if (modeDDT == 7 || modeDDT == 8){
                        modeDDT = 2;
                        delay(10);
                    }
                }
                SerialBT.write(11);
            }
        }


        if (action.id == ArrowPressCenter) {

            if (modeDDT == 1 ||modeDDT == 3 || modeDDT == 4 || modeDDT == 5 || modeDDT == 6){
                currentSpeedUp();
            }
            else {
                modeDDT = 0;
                delay(20);
            }
        }





        if (action.id == ButtonPressA) { //
            // Serial.println("A-button");
            SerialBT.write(49);
            Serial.println("motion-1");
        }


        if (action.id == ButtonPressB) { //
        // Serial.println("B-button");
            SerialBT.write(50);
            Serial.println("motion-2");
        }


        if (action.id == ButtonPressC) { //
        // Serial.println("C-button");
            SerialBT.write(51);
            Serial.println("motion-3");
        }


        if (action.id == ButtonPressD) { //
        // Serial.println("D-button");
            SerialBT.write(52);
            Serial.println("motion-4");
        }

        if (action.id == ButtonPressE) { //
        // Serial.println("E-button");
            SerialBT.write(53);
            Serial.println("motion-5");
        }




        if (action.id == ButtonOut) {
            SerialBT.write(0xff);
        }
    }



    if (modeDDT == 0) {
        if (rightMotorActive == true && leftMotorActive == false) {  //前進

            if (currentSpeed > 75){
                while (currentSpeed > 75){
                    currentSpeed -= fwBrakeAdd;
                    moveDDT(leftMotorID, currentSpeed);
                    moveDDT(rightMotorID, -currentSpeed);
                    // moveDDT(Speed, -Speed);
                }
            }
            if (75 > currentSpeed > 0){
                while (currentSpeed > 0){
                    currentSpeed -= 3;
                    moveDDT(leftMotorID, currentSpeed);
                    moveDDT(rightMotorID, -currentSpeed);
                    // moveDDT(Speed, -Speed);
                }
            }

            currentSpeed = 0;
            moveDDT(leftMotorID, currentSpeed);
            moveDDT(rightMotorID, currentSpeed);
            // moveDDT(Speed, currentSpeed);
        }

        if (rightMotorActive == false && leftMotorActive == true) { //後退

            while (currentSpeed > 0) {
                currentSpeed -= brakeAdd;
                moveDDT(leftMotorID, -currentSpeed);
                moveDDT(rightMotorID, currentSpeed);
                // moveDDT(-Speed, currentSpeed);
            }

            currentSpeed = 0;
            moveDDT(leftMotorID, currentSpeed);
            moveDDT(rightMotorID, currentSpeed);
            // moveDDT(Speed, currentSpeed);
        }

        if (rightMotorActive == true && leftMotorActive == true) {  //右回転

            while (currentSpeed > 0) {
                currentSpeed -= brakeAdd;
                moveDDT(leftMotorID, -currentSpeed);
                moveDDT(rightMotorID, currentSpeed);
                // moveDDT(-Speed, currentSpeed);
            }

            currentSpeed = 0;
            moveDDT(leftMotorID, currentSpeed);
            moveDDT(rightMotorID, currentSpeed);
            // moveDDT(Speed, currentSpeed);
        }

        if (rightMotorActive == false && leftMotorActive == false) {  //左回転

            while (currentSpeed > 0) {
                currentSpeed -= brakeAdd;
                moveDDT(leftMotorID, -currentSpeed);
                moveDDT(rightMotorID, -currentSpeed);
                // moveDDT(-Speed, -Speed);
            }

            currentSpeed = 0;
            moveDDT(leftMotorID, currentSpeed);
            moveDDT(rightMotorID, currentSpeed);
            // moveDDT(Speed, currentSpeed);
        }

    }




    if (modeDDT == 1) {  //前進
        rightMotorActive = true;
        leftMotorActive = false;

        if (wasQuickDashActive == true){
            currentForwardSpeed -= dashDeceleration;
            moveDDT(leftMotorID, currentForwardSpeed * 1.02);  //←いじらない
            moveDDT(rightMotorID, -currentForwardSpeed);
            // moveDDT(currentForwardSpeed * 1.02, -currentForwardSpeed);
            if (currentForwardSpeed < forwardMaxSpeed){
                currentForwardSpeed = forwardMaxSpeed;
                isForwardSpeedLimited = true;
                delay(20);
            }
        }
        else {
            currentSpeed += fwRunAdd;

            moveDDT(leftMotorID, currentSpeed * 1.02);  //←いじらない
            moveDDT(rightMotorID, -currentSpeed);
            // moveDDT(Speed * 1.02, -Speed);
            

            if (currentSpeed > forwardMaxSpeed) {
                currentSpeed = forwardMaxSpeed;
                isForwardSpeedLimited = true;
                delay(20);
            }
        }
    }


    if (modeDDT == 2) {  //後退
        if (lastModeDDT == 1){
            modeDDT = 0;
            lastModeDDT = 2;
        }
        else {
            rightMotorActive = false;
            leftMotorActive = true;
            currentSpeed += runAdd;

            moveDDT(leftMotorID, -currentSpeed);
            moveDDT(rightMotorID, currentSpeed);
            // moveDDT(-Speed, currentSpeed);

            if (currentSpeed > backwardMaxSpeed) {
                currentSpeed = backwardMaxSpeed;
                delay(20);
            }
        }
    }


    if (modeDDT == 3) {  //右回転
        rightMotorActive = true;
        leftMotorActive = true;
        currentSpeed += turnAdd;

        moveDDT(leftMotorID, currentSpeed);
        moveDDT(rightMotorID, currentSpeed);
        // moveDDT(Speed, currentSpeed);

        if (currentSpeed > rotationMaxSpeed) {
            currentSpeed = rotationMaxSpeed;
            delay(20);
        }
    }


    if (modeDDT == 4) {  //左回転
        rightMotorActive = false;
        leftMotorActive = false;
        currentSpeed += turnAdd;

        moveDDT(leftMotorID, -currentSpeed);
        moveDDT(rightMotorID, -currentSpeed);
        // moveDDT(-Speed, -Speed);

        if (currentSpeed > rotationMaxSpeed) {
            currentSpeed = rotationMaxSpeed;
            delay(20);
        }
    }


    if (modeDDT == 5) {  //右斜前
        rightMotorActive = true;
        leftMotorActive = true;

        
        currentSpeed += fwTurnAdd;

        moveDDT(leftMotorID, currentSpeed);
        moveDDT(rightMotorID, -currentSpeed*2/3);
        // moveDDT(Speed, -Speed*2/3);

        if (currentSpeed > forwardMaxSpeed) {
            currentSpeed = forwardMaxSpeed;
            delay(20);
        }
        
    }


    if (modeDDT == 6) {  //左斜前
        rightMotorActive = false;
        leftMotorActive = false;

        currentSpeed += fwTurnAdd;

        moveDDT(leftMotorID, currentSpeed*2/3);
        moveDDT(rightMotorID, -currentSpeed);
        // moveDDT(Speed*2/3, -Speed);

        if (currentSpeed > forwardMaxSpeed) {
            currentSpeed = forwardMaxSpeed;
            delay(20);
        
        }
    }


    if (modeDDT == 7) {  //右斜後
        rightMotorActive = true;
        leftMotorActive = true;
        currentSpeed += runAdd;

        moveDDT(leftMotorID, -currentSpeed);
        moveDDT(rightMotorID, currentSpeed/2);
        // moveDDT(-Speed, currentSpeed/2);

        if (currentSpeed > forwardMaxSpeed) {
            currentSpeed = forwardMaxSpeed;
            delay(20);
        }
    }


    if (modeDDT == 8) {  //左斜後
        rightMotorActive = false;
        leftMotorActive = false;
        currentSpeed += runAdd;

        moveDDT(leftMotorID, -currentSpeed/2);
        moveDDT(rightMotorID, currentSpeed);
        // moveDDT(-Speed/2, currentSpeed);

        if (currentSpeed > forwardMaxSpeed) {
            currentSpeed = forwardMaxSpeed;
            delay(20);
        }
    }

    if (isForwardButtonPressed == true){
        spendTime++;
        if (spendTime > changeModeTime){
            operationMode = 2;
        }
    }

}