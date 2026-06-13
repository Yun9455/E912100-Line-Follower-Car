
// 1. 硬體腳位定義 (Hardware Pinout)
// 感測器 (由左至右)
const int PIN_SNS_LL = 36; // 最左
const int PIN_SNS_L  = 39; // 左中
const int PIN_SNS_C  = 34; // 正中
const int PIN_SNS_R  = 35; // 右中
const int PIN_SNS_RR = 4;  // 最右
// 馬達控制 (L298N)
const int PIN_MOT_L_PWM = 25;
const int PIN_MOT_L_DIR1 = 26;
const int PIN_MOT_L_DIR2 = 27;
const int PIN_MOT_R_PWM = 33;
const int PIN_MOT_R_DIR1 = 14;
const int PIN_MOT_R_DIR2 = 13;
// 安全與系統操作介面
const int PIN_BTN_TSMS  = 32;
const int PIN_BTN_VLS   = 22;
const int PIN_BTN_ESTOP = 23;
const int PIN_BTN_RESET = 15;
// ASL 狀態指示燈
const int PIN_LED_G = 18;
const int PIN_LED_R = 19;
const int PIN_LED_B = 21;
// 2. 系統參數設定 (System Parameters)
// 幾何控制參數 (為 18 公分前懸最佳化)
const float Ld = 140.0;          // 前視距離 Lookahead Distance
const float L  = 120.0;          // 輪距 Wheelbase
const float SNS_POS[5] = {24.0, 12.0, 0.0, -12.0, -24.0};
// 動力參數
const int SPEED_BASE = 130;
const int SPEED_MAX  = 210;
const int SPEED_MIN  = 80;
const int SPEED_SEARCH = 95;     // 尋線打轉速度
// PWM 設定
const int PWM_FREQ = 1000;
const int PWM_RES = 8;
const int CH_L = 0;
const int CH_R = 1;
// 歷史趨勢紀錄 (用於過直角彎)
const int T_SIZE = 5;
float errorBuffer[T_SIZE] = {0};
int bufferIdx = 0;
float currentTrend = 0;
// 失聯計時器
unsigned long timeLost = 0;
const unsigned long TIMEOUT_LIMIT = 5000; // 5秒 DNF 判定

// 狀態機定義
enum class SystemState {
    INIT_SAFE,      // 初始/安全鎖定 (綠燈)
    STANDBY,        // 待機/準備起跑 (藍燈)
    TRACKING,       // 自主循跡中 (紅燈)
    EMERGENCY       // EBS 觸發狀態 (綠燈/馬達斷電)
};
SystemState currentState = SystemState::INIT_SAFE;
// 3. 基礎硬體控制函式 (Hardware Abstraction)
void initHardware() {
    Serial.begin(115200);   
    // 輸入腳位
    int inputs[] = {PIN_SNS_LL, PIN_SNS_L, PIN_SNS_C, PIN_SNS_R, PIN_SNS_RR};
    for(int pin : inputs) pinMode(pin, INPUT);
    pinMode(PIN_BTN_TSMS, INPUT_PULLUP);
    pinMode(PIN_BTN_VLS, INPUT_PULLUP);
    pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_BTN_RESET, INPUT_PULLUP);
    // 輸出腳位
    int outputs[] = {PIN_MOT_L_DIR1, PIN_MOT_L_DIR2, PIN_MOT_R_DIR1, PIN_MOT_R_DIR2, PIN_LED_G, PIN_LED_R, PIN_LED_B};
    for(int pin : outputs) pinMode(pin, OUTPUT);
    // PWM 初始化相容性處理
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcAttach(PIN_MOT_L_PWM, PWM_FREQ, PWM_RES);
        ledcAttach(PIN_MOT_R_PWM, PWM_FREQ, PWM_RES);
    #else
        ledcSetup(CH_L, PWM_FREQ, PWM_RES);
        ledcSetup(CH_R, PWM_FREQ, PWM_RES);
        ledcAttachPin(PIN_MOT_L_PWM, CH_L);
        ledcAttachPin(PIN_MOT_R_PWM, CH_R);
    #endif
}
void setASL(bool r, bool g, bool b) {
    // 假設共陰極 LED (HIGH 亮)
    digitalWrite(PIN_LED_R, r ? HIGH : LOW);
    digitalWrite(PIN_LED_G, g ? HIGH : LOW);
    digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}
void applyMotorPower(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);
    // 左輪方向與動力
    digitalWrite(PIN_MOT_L_DIR1, leftPWM > 0);
    digitalWrite(PIN_MOT_L_DIR2, leftPWM < 0);
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(PIN_MOT_L_PWM, abs(leftPWM));
    #else
        ledcWrite(CH_L, abs(leftPWM));
    #endif
    // 右輪方向與動力
    digitalWrite(PIN_MOT_R_DIR1, rightPWM > 0);
    digitalWrite(PIN_MOT_R_DIR2, rightPWM < 0);
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(PIN_MOT_R_PWM, abs(rightPWM));
    #else
        ledcWrite(CH_R, abs(rightPWM));
    #endif
}
void haltVehicle() {
    applyMotorPower(0, 0);
}
// 4. 感測與演算法模組 (Sensing & Algorithm)
// 讀取黑線誤差 (回傳 target Y)
float getLineError(int &activeCount) {
    int readings[5] = {
        digitalRead(PIN_SNS_LL), digitalRead(PIN_SNS_L), 
        digitalRead(PIN_SNS_C), 
        digitalRead(PIN_SNS_R), digitalRead(PIN_SNS_RR)
    };
    float sum = 0;
    activeCount = 0;
    for(int i = 0; i < 5; i++) {
        if(readings[i] == LOW) { // 偵測到黑線
            sum += SNS_POS[i];
            activeCount++;
        }
    }
    return (activeCount > 0) ? (sum / activeCount) : 0;
}
// 更新歷史趨勢 (加權平均)
void updateTrend(float newError) {
    errorBuffer[bufferIdx] = newError;
    bufferIdx = (bufferIdx + 1) % T_SIZE;
    currentTrend = 0;
    // 近期資料權重較高，增強對直角彎的反應
    for(int i = 0; i < T_SIZE; i++) {
        currentTrend += errorBuffer[i]; 
    }
}
// 盲區尋線邏輯
void executeSearch() {
    if(currentTrend >= 0) {
        applyMotorPower(-SPEED_SEARCH, SPEED_SEARCH); // 原地左轉
    } else {
        applyMotorPower(SPEED_SEARCH, -SPEED_SEARCH); // 原地右轉
    }
}
// Pure Pursuit 核心演算法
void executePurePursuit() {
    int activeSensors = 0;
    float errY = getLineError(activeSensors);
    // 處理出界狀況
    if(activeSensors == 0) {
        if(timeLost == 0) timeLost = millis();
        
        if(millis() - timeLost <= TIMEOUT_LIMIT) {
            executeSearch();
        } else {
            currentState = SystemState::EMERGENCY;
            Serial.println("EBS: Line Lost Timeout");
        }
        return;
    }
    // 正常循跡
    timeLost = 0;
    updateTrend(errY);

    // 計算曲率與差速
    float curvature = (2.0 * errY) / ((Ld * Ld) + (errY * errY));
    float vL = SPEED_BASE * (1.0 - curvature * L / 2.0);
    float vR = SPEED_BASE * (1.0 + curvature * L / 2.0);

    applyMotorPower((int)vL, (int)vR);
    delay(10); // 維持取樣率
}
// 5. 系統主流程 (Main State Machine)
void setup() {
    initHardware();
    haltVehicle();
    currentState = SystemState::INIT_SAFE;
}
void loop() {
    // 絕對優先權：硬體 E-STOP 或 TSMS 關閉
    if(digitalRead(PIN_BTN_ESTOP) == LOW || digitalRead(PIN_BTN_TSMS) == HIGH) {
        currentState = SystemState::EMERGENCY;
    }
    // 狀態機控制
    switch(currentState) {
        case SystemState::INIT_SAFE:
            haltVehicle();
            setASL(false, true, false); // 綠燈
            timeLost = 0;
            // TSMS開啟 且 沒按E-STOP 即可進入待機
            if(digitalRead(PIN_BTN_TSMS) == LOW && digitalRead(PIN_BTN_ESTOP) == HIGH) {
                currentState = SystemState::STANDBY;
            }
            break;
        case SystemState::STANDBY:
            haltVehicle();
            setASL(false, false, true); // 藍燈
            // 收到 VLS 起跑訊號
            if(digitalRead(PIN_BTN_VLS) == LOW) {
                currentState = SystemState::TRACKING;
                Serial.println("VLS Triggered. Autonomous Run Start.");
                delay(300); // 防彈跳
            }
            break;

        case SystemState::TRACKING:
            setASL(true, false, false); // 紅燈
            executePurePursuit();
            break;

        case SystemState::EMERGENCY:
            haltVehicle();
            setASL(false, true, false); // 綠燈 (安全停機)
            // EBS 解鎖條件：TSMS 開著 + 放開 ESTOP + 按下 RESET
            if(digitalRead(PIN_BTN_TSMS) == LOW && 
               digitalRead(PIN_BTN_ESTOP) == HIGH && 
               digitalRead(PIN_BTN_RESET) == LOW) {
                currentState = SystemState::STANDBY;
                Serial.println("EBS Reset.");
                delay(300);
            }
            break;
    }
}
