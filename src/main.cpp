#include <Arduino.h>
#include <DDT_Motor_M15M06.h>

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
struct Action
{
    int id;
    String command;
};
Action ACTIONS[] = {
    {Empty, "__"},
    {Start, "START"},
    {ArrowPressUp, "ARROW_PRESS_UP"},
    {ArrowPressDown, "ARROW_PRESS_DOWN"},
    {ArrowPressLeft, "ARROW_PRESS_LEFT"},
    {ArrowPressRight, "ARROW_PRESS_RIGHT"},
    {ArrowPressCenter, "ARROW_PRESS_CENTER"},
    {ArrowOut, "ARROW_OUT"},
    {ButtonPressA, "BUTTON_PRESS_A"},
    {ButtonPressB, "BUTTON_PRESS_B"},
    {ButtonPressC, "BUTTON_PRESS_C"},
    {ButtonPressD, "BUTTON_PRESS_D"},
    {ButtonPressE, "BUTTON_PRESS_E"},
    {ButtonPressF, "BUTTON_PRESS_F"},
    {ButtonPressG, "BUTTON_PRESS_G"},
    {ButtonPressH, "BUTTON_PRESS_H"},
    {ButtonPressI, "BUTTON_PRESS_I"},
    {ButtonPressJ, "BUTTON_PRESS_J"},
    {ButtonPressK, "BUTTON_PRESS_K"},
    {ButtonPressL, "BUTTON_PRESS_L"},
    {ButtonPressM, "BUTTON_PRESS_M"},
    {ButtonPressN, "BUTTON_PRESS_N"},
    {ButtonPressO, "BUTTON_PRESS_O"},
    {ButtonPressP, "BUTTON_PRESS_P"},
    {ButtonPressQ, "BUTTON_PRESS_Q"},
    {ButtonPressR, "BUTTON_PRESS_R"},
    {ButtonPressS, "BUTTON_PRESS_S"},
    {ButtonPressT, "BUTTON_PRESS_T"},
    {ButtonPressU, "BUTTON_PRESS_U"},
    {ButtonPressV, "BUTTON_PRESS_V"},
    {ButtonPressW, "BUTTON_PRESS_W"},
    {ButtonPressX, "BUTTON_PRESS_X"},
    {ButtonPressY, "BUTTON_PRESS_Y"},
    {ButtonPressZ, "BUTTON_PRESS_Z"},
    {ButtonOut, "BUTTON_OUT"},
};

// ---- S/W Version ------------------
#define VERSION_NUMBER "  Ver. 1.4.0"
// -----------------------------------

////////PIN番号////////

const int TX_PIN = 26;
const int RX_PIN = 27;

//////////////////////

int16_t fwMaxSpeed = 100;  // 前進時の最大速度
int16_t bkMaxSpeed = 50;   // 後退時の最大速度
int16_t turnMaxSpeed = 20; // 回転時の最大速度
int16_t fwSpeed = 80;
int16_t fwTurnSpeed = 80;
int16_t Speed = 0;   // Speed of motor
uint8_t Acce = 0;    // Acceleration of motor
uint8_t Brake_P = 0; // 0xffを入れるとブレーキが入る その場でブレーキが入る// Brake position of motor
uint8_t leftID = 1;  // ID of Motor (default:1)
uint8_t rightID = 2;

const int fwRunAdd = 5; // 平地走行モードでの回転速度の上昇数 前進
const int runAdd = 5;   // 平地走行モードでの回転速度の上昇数 バック
const int fwTurnAdd = 3;
const int dashDeceleration = 2; // QDashからの減速値  MAX:100の時は4 MAX:80の時は2a
const int turnDeceleration = 5;
const int turnAdd = 3;    // 平地走行モードでの回転速度の上昇数 左右回転
const int fwBrakeAdd = 4; // 前進での回転速度の減少数
const int brakeAdd = 3;   // 平地走行モードでの回転速度の減少数 30

int modeDDT = 0; // 0:停止 1:前進 2:後進 3:右回転 4:左回転 5:右斜前 6:左斜前 7:右斜後 8:左斜後
int lastModeDDT = 0;

int spendTime = 0;
const int changeModeTime = 20;

int operationMode = 2; // 1:直進モードあり  2:直進モードなし

bool flagR = false;
bool flagL = false;

bool pushUpButton = false; // true:前進ボタンを押した false:前進ボタンを離した

bool QDash = false; // 前進しながらQキーを押すと加速する
bool lastQDash = false;

bool fwLimitSpeed = false;

Receiver Receiv;
// M5Stackのモジュールによって対応するRX,TXのピン番号が違うためM5製品とRS485モジュールに対応させてください
auto motor_handler = MotorHandler(RX_PIN, TX_PIN); // RX,TX

const int16_t SPEED_MAX = 330;
const int16_t SPEED_MIN = -330;

void moveDDT(int id, int velocity)
{
    motor_handler.Control_Motor(velocity, id, Acce, Brake_P, &Receiv);
    delay(18); // ←いじらない
}

// void moveDDT(int l, int r){
//     motor_handler.Control_Motor(l, leftID, Acce, Brake_P, &Receiv);
//     delay(20);
//     motor_handler.Control_Motor(r, rightID, Acce, Brake_P, &Receiv);
//     delay(50);
// }

void straightenFace()
{                                               // OriHimeの顔を正面に向ける
    Serial.println("SET_ORIHIME_HEAD_0.5,0.5"); // 頭
}

void speedUp()
{
    QDash = true;
    Serial.println("Speed UP!!");

    fwMaxSpeed = 160;
    bkMaxSpeed = 80;
    turnMaxSpeed = 40;
    fwSpeed = fwMaxSpeed;
}

void speedDown()
{
    QDash = false;
    lastQDash = true;
    Serial.println("Speed Down!!");

    fwMaxSpeed = 80;
    bkMaxSpeed = 50;
    turnMaxSpeed = 20;
}

void setup()
{
    delay(3000);
    Serial.begin(19200);
    Serial2.begin(115200);
    Serial.println("DDT-Motor RS485");

    motor_handler.Control_Motor(0, leftID, Acce, Brake_P, &Receiv);
    motor_handler.Control_Motor(0, rightID, Acce, Brake_P, &Receiv);

    Serial.println(VERSION_NUMBER);
}

Action checkAction(String command)
{
    command.trim();
    for (int i = 0; i < sizeof(ACTIONS); i += 1)
    {
        if (command == ACTIONS[i].command)
        {
            return ACTIONS[i];
        }
    }
    return ACTIONS[0];
}

void loop()
{
    // Serial.println(spendTime);

    delay(5);

    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        Action action = checkAction(command);
        if (action.id == 0)
            return;

        if (action.id == ArrowPressUp)
        {
            pushUpButton = true;
            if (modeDDT == 3)
            {
                Serial.println("右斜前");
                modeDDT = 5;
                delay(10);
            }
            else if (modeDDT == 4)
            {
                Serial.println("左斜前");
                modeDDT = 6;
                delay(10);
            }
            else
            {
                Serial.println("前進");
                modeDDT = 1;
                lastModeDDT = 1;
                straightenFace();
                delay(10);
            }
        }

        if (action.id == ArrowPressDown)
        {
            if (modeDDT == 3)
            {
                Serial.println("右斜後");
                modeDDT = 7;
                delay(10);
            }
            else if (modeDDT == 4)
            {
                Serial.println("左斜後");
                modeDDT = 8;
                delay(10);
            }
            else
            {
                Serial.println("後退");
                modeDDT = 2;
                delay(10);
            }
        }

        if (action.id == ArrowPressRight)
        {
            if (modeDDT == 1)
            {
                Serial.println("右斜前");
                modeDDT = 5;
                // lastModeDDT = 5;
                delay(10);
            }
            else if (modeDDT == 2)
            {
                Serial.println("右斜後");
                modeDDT = 7;
                delay(10);
            }
            else
            {
                Serial.println("右回転");
                modeDDT = 3;
                lastModeDDT = 3;
                delay(10);
            }
        }

        if (action.id == ArrowPressLeft)
        {
            if (modeDDT == 1)
            {
                Serial.println("左斜前");
                modeDDT = 6;
                // lastModeDDT = 6;
                delay(10);
            }
            else if (modeDDT == 2)
            {
                Serial.println("左斜後");
                modeDDT = 8;
                delay(10);
            }
            else
            {
                Serial.println("左回転");
                modeDDT = 4;
                lastModeDDT = 4;
                delay(10);
            }
        }

        if (action.id == ButtonPressA)
        { //
            // Serial.println("A-button");
            Serial.println("Check version");
            Serial.println(VERSION_NUMBER);
            Serial.println("Check mode");
            if (operationMode == 1)
            {
                Serial.println("直進モード有効");
            }
            else if (operationMode == 2)
            {
                Serial.println("直進モード無効");
            }
        }

        if (action.id == ButtonPressB)
        {   //
            // Serial.println("B-button");
            operationMode = 1;
            Serial.println("直進モード有効");
        }

        if (action.id == ButtonPressC)
        {   //
            // Serial.println("C-button");
            operationMode = 2;
            Serial.println("直進モード無効");
        }

        if (action.id == ButtonOut)
        {
        }

        if (action.id == ArrowOut)
        {
            pushUpButton = false;
            spendTime = 0;
            if (QDash == true)
            {
                speedDown();
            }
            else
            {
                lastQDash = false;
                fwLimitSpeed = false;
                if (operationMode == 1)
                {
                    if (modeDDT == 0 || modeDDT == 2 || modeDDT == 3 || modeDDT == 4)
                    {
                        modeDDT = 0;
                        delay(10);
                    }
                    else if (modeDDT == 5 || modeDDT == 6)
                    {
                        modeDDT = 1;
                        lastModeDDT = 1;
                        delay(10);
                    }
                    else if (modeDDT == 7 || modeDDT == 8)
                    {
                        modeDDT = 2;
                        delay(10);
                    }
                }
                else if (operationMode == 2)
                {
                    if (modeDDT == 0 || modeDDT == 1 || modeDDT == 2 || modeDDT == 3 || modeDDT == 4)
                    {
                        modeDDT = 0;
                        lastModeDDT = 0;
                        delay(10);
                    }
                    else if (modeDDT == 5 || modeDDT == 6)
                    {
                        modeDDT = 1;
                        delay(10);
                    }
                    else if (modeDDT == 7 || modeDDT == 8)
                    {
                        modeDDT = 2;
                        delay(10);
                    }
                }
            }
        }

        if (action.id == ArrowPressCenter)
        {

            if (modeDDT == 1 || modeDDT == 3 || modeDDT == 4 || modeDDT == 5 || modeDDT == 6)
            {
                speedUp();
            }
            else
            {
                modeDDT = 0;
                delay(20);
            }
        }
    }

    if (modeDDT == 0)
    {
        if (flagR == true && flagL == false)
        { // 前進

            if (Speed > 75)
            {
                while (Speed > 75)
                {
                    Speed -= fwBrakeAdd;
                    moveDDT(leftID, Speed);
                    moveDDT(rightID, -Speed);
                    // moveDDT(Speed, -Speed);
                }
            }
            if (75 > Speed > 0)
            {
                while (Speed > 0)
                {
                    Speed -= 3;
                    moveDDT(leftID, Speed);
                    moveDDT(rightID, -Speed);
                    // moveDDT(Speed, -Speed);
                }
            }

            Speed = 0;
            moveDDT(leftID, Speed);
            moveDDT(rightID, Speed);
            // moveDDT(Speed, Speed);
        }

        if (flagR == false && flagL == true)
        { // 後退

            while (Speed > 0)
            {
                Speed -= brakeAdd;
                moveDDT(leftID, -Speed);
                moveDDT(rightID, Speed);
                // moveDDT(-Speed, Speed);
            }

            Speed = 0;
            moveDDT(leftID, Speed);
            moveDDT(rightID, Speed);
            // moveDDT(Speed, Speed);
        }

        if (flagR == true && flagL == true)
        { // 右回転

            while (Speed > 0)
            {
                Speed -= brakeAdd;
                moveDDT(leftID, -Speed);
                moveDDT(rightID, Speed);
                // moveDDT(-Speed, Speed);
            }

            Speed = 0;
            moveDDT(leftID, Speed);
            moveDDT(rightID, Speed);
            // moveDDT(Speed, Speed);
        }

        if (flagR == false && flagL == false)
        { // 左回転

            while (Speed > 0)
            {
                Speed -= brakeAdd;
                moveDDT(leftID, -Speed);
                moveDDT(rightID, -Speed);
                // moveDDT(-Speed, -Speed);
            }

            Speed = 0;
            moveDDT(leftID, Speed);
            moveDDT(rightID, Speed);
            // moveDDT(Speed, Speed);
        }
    }

    if (modeDDT == 1)
    { // 前進
        flagR = true;
        flagL = false;

        if (lastQDash == true)
        {
            fwSpeed -= dashDeceleration;
            moveDDT(leftID, fwSpeed * 1.02); // ←いじらない
            moveDDT(rightID, -fwSpeed);
            // moveDDT(fwSpeed * 1.02, -fwSpeed);
            if (fwSpeed < fwMaxSpeed)
            {
                fwSpeed = fwMaxSpeed;
                fwLimitSpeed = true;
                delay(20);
            }
        }
        else
        {
            Speed += fwRunAdd;

            moveDDT(leftID, Speed * 1.02); // ←いじらない
            moveDDT(rightID, -Speed);
            // moveDDT(Speed * 1.02, -Speed);

            if (Speed > fwMaxSpeed)
            {
                Speed = fwMaxSpeed;
                fwLimitSpeed = true;
                delay(20);
            }
        }
    }

    if (modeDDT == 2)
    { // 後退
        if (lastModeDDT == 1)
        {
            modeDDT = 0;
            lastModeDDT = 2;
        }
        else
        {
            flagR = false;
            flagL = true;
            Speed += runAdd;

            moveDDT(leftID, -Speed);
            moveDDT(rightID, Speed);
            // moveDDT(-Speed, Speed);

            if (Speed > bkMaxSpeed)
            {
                Speed = bkMaxSpeed;
                delay(20);
            }
        }
    }

    if (modeDDT == 3)
    { // 右回転
        flagR = true;
        flagL = true;
        Speed += turnAdd;

        moveDDT(leftID, Speed);
        moveDDT(rightID, Speed);
        // moveDDT(Speed, Speed);

        if (Speed > turnMaxSpeed)
        {
            Speed = turnMaxSpeed;
            delay(20);
        }
    }

    if (modeDDT == 4)
    { // 左回転
        flagR = false;
        flagL = false;
        Speed += turnAdd;

        moveDDT(leftID, -Speed);
        moveDDT(rightID, -Speed);
        // moveDDT(-Speed, -Speed);

        if (Speed > turnMaxSpeed)
        {
            Speed = turnMaxSpeed;
            delay(20);
        }
    }

    if (modeDDT == 5)
    { // 右斜前
        flagR = true;
        flagL = true;

        Speed += fwTurnAdd;

        moveDDT(leftID, Speed);
        moveDDT(rightID, -Speed * 2 / 3);
        // moveDDT(Speed, -Speed*2/3);

        if (Speed > fwMaxSpeed)
        {
            Speed = fwMaxSpeed;
            delay(20);
        }
    }

    if (modeDDT == 6)
    { // 左斜前
        flagR = false;
        flagL = false;

        Speed += fwTurnAdd;

        moveDDT(leftID, Speed * 2 / 3);
        moveDDT(rightID, -Speed);
        // moveDDT(Speed*2/3, -Speed);

        if (Speed > fwMaxSpeed)
        {
            Speed = fwMaxSpeed;
            delay(20);
        }
    }

    if (modeDDT == 7)
    { // 右斜後
        flagR = true;
        flagL = true;
        Speed += runAdd;

        moveDDT(leftID, -Speed);
        moveDDT(rightID, Speed / 2);
        // moveDDT(-Speed, Speed/2);

        if (Speed > fwMaxSpeed)
        {
            Speed = fwMaxSpeed;
            delay(20);
        }
    }

    if (modeDDT == 8)
    { // 左斜後
        flagR = false;
        flagL = false;
        Speed += runAdd;

        moveDDT(leftID, -Speed / 2);
        moveDDT(rightID, Speed);
        // moveDDT(-Speed/2, Speed);

        if (Speed > fwMaxSpeed)
        {
            Speed = fwMaxSpeed;
            delay(20);
        }
    }

    if (pushUpButton == true)
    {
        spendTime++;
        if (spendTime > changeModeTime)
        {
            operationMode = 2;
        }
    }
}