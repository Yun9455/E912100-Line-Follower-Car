// 1. 硬體腳位定義 (Hardware Pinout)
// 感測器陣列 (由左至右，對應實體位置)
const int PIN_SNS_LL = 36; // 最左側感測器
const int PIN_SNS_L  = 39; // 左側感測器
const int PIN_SNS_C  = 34; // 正中央感測器
const int PIN_SNS_R  = 35; // 右側感測器
const int PIN_SNS_RR = 4;  // 最右側感測器

// L298N 馬達驅動板控制腳位 (支援 PWM 調速與前後向控制)
const int PIN_MOT_L_PWM = 25;  // 左輪速度 (ENA)
const int PIN_MOT_L_DIR1 = 26; // 左輪方向1 (IN1)
const int PIN_MOT_L_DIR2 = 27; // 左輪方向2 (IN2)
const int PIN_MOT_R_PWM = 33;  // 右輪速度 (ENB)
const int PIN_MOT_R_DIR1 = 14; // 右輪方向1 (IN3)
const int PIN_MOT_R_DIR2 = 13; // 右輪方向2 (IN4)

// 安全與系統操作介面 (利用內建上拉電阻)
const int PIN_BTN_TSMS  = 32;  // 總電源安全開關 (Tractive System Master Switch)
const int PIN_BTN_VLS   = 22;  // 車輛啟動發車鈕 (Vehicle Launch Switch)
const int PIN_BTN_ESTOP = 23;  // 緊急停止按鈕 (Emergency Stop)
const int PIN_BTN_RESET = 15;  // 系統重置/解除鎖定鈕

// ASL 狀態指示燈 (RGB LED，共陰極設計)
const int PIN_LED_G = 18; // 綠燈 (代表安全或鎖定狀態)
const int PIN_LED_R = 19; // 紅燈 (代表循跡危險/作動狀態)
const int PIN_LED_B = 21; // 藍燈 (代表待機準備狀態)

// 2. 系統參數設定 (System Parameters)
// 幾何控制參數：針對 21 公分前懸(感測器到輪軸距離)進行最佳化 (已配合報告修正)
const float Ld = 220.0;          // 前視距離 (Lookahead Distance)，作為 Pure Pursuit 的假想目標點距離
const float L  = 120.0;          // 輪距 (Wheelbase)，左右輪中心點的距離
const float SNS_POS[5] = {24.0, 12.0, 0.0, -12.0, -24.0}; // 5顆感測器相對於車體中軸線的物理橫向偏移量(mm)

// 動力參數 (PWM 範圍 0-255)
const int SPEED_BASE = 130;      // 基礎直行巡航速度
const int SPEED_MAX  = 210;      // 系統容許最大馬達輸出限制
const int SPEED_MIN  = 80;       // 系統容許最小馬達輸出(避免死區)
const int SPEED_SEARCH = 75;     // 盲區尋線時，執行原地樞軸轉向(Pivot Turn)的速度 (已配合報告降速)

// ESP32 PWM 硬體定時器設定
const int PWM_FREQ = 1000;       // PWM 頻率 1kHz，適合驅動直流有刷馬達
const int PWM_RES = 8;           // 8-bit 解析度 (數值範圍 0-255)
const int CH_L = 0;              // 舊版 ESP32 核心使用的左輪 PWM 通道
const int CH_R = 1;              // 舊版 ESP32 核心使用的右輪 PWM 通道

// 歷史趨勢紀錄緩衝區 (針對直角彎防呆設計)
const int T_SIZE = 5;            // 記錄過去 5 次的有效誤差值
float errorBuffer[T_SIZE] = {0}; // 儲存誤差的環狀陣列(Circular Buffer)
int bufferIdx = 0;               // 陣列寫入指標
float currentTrend = 0;          // 計算出的近期軌跡偏移動態總和

// 失聯計時器 (安全規範：車輛衝出賽道判定)
unsigned long timeLost = 0;      // 記錄開始脫離黑線的系統時間(ms)
const unsigned long TIMEOUT_LIMIT = 5000; // 5秒 DNF(未完賽)強制斷電判定門檻

// 定義系統的四階段有限狀態機 (Finite State Machine)
enum class SystemState {
    INIT_SAFE,      // 初始安全鎖定：馬達強制停止，亮綠燈
    STANDBY,        // 待機準備起跑：系統通電就緒，亮藍燈等待 VLS 觸發
    TRACKING,       // 自主循跡中：演算法接管車輛控制，亮紅燈
    EMERGENCY       // EBS 觸發狀態：發生危險或逾時，馬達強制斷電，亮綠燈
};
SystemState currentState = SystemState::INIT_SAFE; // 預設開機進入最高安全層級

// 3. 基礎硬體控制函式 (Hardware Abstraction)
void initHardware() {
    Serial.begin(115200);   
    // 初始化感測器為輸入模式
    int inputs[] = {PIN_SNS_LL, PIN_SNS_L, PIN_SNS_C, PIN_SNS_R, PIN_SNS_RR};
    for(int pin : inputs) pinMode(pin, INPUT);

    // 初始化安全開關為上拉輸入 (常態為 HIGH，按下接 GND 變 LOW)
    pinMode(PIN_BTN_TSMS, INPUT_PULLUP);
    pinMode(PIN_BTN_VLS, INPUT_PULLUP);
    pinMode(PIN_BTN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_BTN_RESET, INPUT_PULLUP);

    // 初始化 L298N 控制腳與指示燈為輸出模式
    int outputs[] = {PIN_MOT_L_DIR1, PIN_MOT_L_DIR2, PIN_MOT_R_DIR1, PIN_MOT_R_DIR2, PIN_LED_G, PIN_LED_R, PIN_LED_B};
    for(int pin : outputs) pinMode(pin, OUTPUT);

    // 跨版本 ESP32 Arduino 核心的 PWM 初始化防呆處理
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        // 適用於 ESP32 Core 3.x 以上的新版 API
        ledcAttach(PIN_MOT_L_PWM, PWM_FREQ, PWM_RES);
        ledcAttach(PIN_MOT_R_PWM, PWM_FREQ, PWM_RES);
    #else
        // 適用於 ESP32 Core 2.x 以下的舊版 API
        ledcSetup(CH_L, PWM_FREQ, PWM_RES);
        ledcSetup(CH_R, PWM_FREQ, PWM_RES);
        ledcAttachPin(PIN_MOT_L_PWM, CH_L);
        ledcAttachPin(PIN_MOT_R_PWM, CH_R);
    #endif
}

void setASL(bool r, bool g, bool b) {
    // 控制狀態指示燈 (基於共陰極硬體架構，輸出 HIGH 導通點亮)
    digitalWrite(PIN_LED_R, r ? HIGH : LOW);
    digitalWrite(PIN_LED_G, g ? HIGH : LOW);
    digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

void applyMotorPower(int leftPWM, int rightPWM) {
    // 限制傳入的 PWM 數值在合理範圍內，避免溢位崩潰
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    // 根據正負值決定左輪旋轉方向 (H橋邏輯)
    digitalWrite(PIN_MOT_L_DIR1, leftPWM > 0);
    digitalWrite(PIN_MOT_L_DIR2, leftPWM < 0);
    
    // 輸出左輪絕對值動力
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(PIN_MOT_L_PWM, abs(leftPWM));
    #else
        ledcWrite(CH_L, abs(leftPWM));
    #endif

    // 根據正負值決定右輪旋轉方向 (H橋邏輯)
    digitalWrite(PIN_MOT_R_DIR1, rightPWM > 0);
    digitalWrite(PIN_MOT_R_DIR2, rightPWM < 0);
    
    // 輸出右輪絕對值動力
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(PIN_MOT_R_PWM, abs(rightPWM));
    #else
        ledcWrite(CH_R, abs(rightPWM));
    #endif
}

void haltVehicle() {
    // 軟體煞車封裝：直接將雙輪動力歸零
    applyMotorPower(0, 0);
}

// 4. 感測與演算法模組 (Sensing & Algorithm)
float getLineError(int &activeCount) {
    // 一次性讀取 5 路 TCRT5000 數位訊號
    int readings[5] = {
        digitalRead(PIN_SNS_LL), digitalRead(PIN_SNS_L), 
        digitalRead(PIN_SNS_C), 
        digitalRead(PIN_SNS_R), digitalRead(PIN_SNS_RR)
    };

    float sum = 0;
    activeCount = 0;

    // 使用質心法 (Center of Mass) 計算黑線的實際偏移位置
    for(int i = 0; i < 5; i++) {
        if(readings[i] == LOW) { // LOW 代表紅外線被黑線吸收(未反射)
            sum += SNS_POS[i];   // 將有觸發的感測器物理座標加總
            activeCount++;       // 紀錄有幾顆感測器壓在線上
        }
    }

    // 回傳平均誤差值 (若無感測器觸發則回傳 0，交由後續失聯邏輯處理)
    return (activeCount > 0) ? (sum / activeCount) : 0;
}

void updateTrend(float newError) {
    // 將最新誤差值寫入環狀陣列，覆蓋最舊的資料
    errorBuffer[bufferIdx] = newError;
    bufferIdx = (bufferIdx + 1) % T_SIZE;
    currentTrend = 0;

    // 加總陣列內所有誤差，形成趨勢向量。若偏左總和為正，偏右總和為負。
    // 此機制可確保即便感測器瞬間掃出線外，系統仍「記得」出界前的轉彎方向。
    for(int i = 0; i < T_SIZE; i++) {
        currentTrend += errorBuffer[i]; 
    }
}

void executeSearch() {
    // 盲區尋線防呆：當車輛因過彎太快或遇到直角彎導致感測器全數失聯時觸發
    if(currentTrend >= 0) {
        // 趨勢大於等於0代表出界前軌跡在左側，強制左後方樞軸轉向(Pivot Turn)尋線
        applyMotorPower(-SPEED_SEARCH, SPEED_SEARCH); 
    } else {
        // 趨勢小於0代表出界前軌跡在右側，強制右後方樞軸轉向(Pivot Turn)尋線
        applyMotorPower(SPEED_SEARCH, -SPEED_SEARCH); 
    }
}

void executePurePursuit() {
    int activeSensors = 0;
    // 取得當前橫向誤差 (Target Y) 與壓線的感測器數量
    float errY = getLineError(activeSensors);

    // 第一重防護：處理完全出界狀況
    if(activeSensors == 0) {
        if(timeLost == 0) timeLost = millis(); // 啟動失聯計時器
        
        if(millis() - timeLost <= TIMEOUT_LIMIT) {
            // 失聯 5 秒內，依據歷史趨勢執行原地打轉尋線
            executeSearch();
        } else {
            // 失聯超過 5 秒，判定無法救回，觸發 EBS 緊急制動切斷動力
            currentState = SystemState::EMERGENCY;
            Serial.println("EBS: Line Lost Timeout");
        }
        return; // 中斷本次演算，不輸出動力
    }

    // 第二重防護：正常循跡時，重置失聯計時並更新歷史記憶
    timeLost = 0;
    updateTrend(errY);

    // Pure Pursuit (純粹追跡) 核心數學模型
    // 依據幾何關係計算曲率 (Curvature, k = 2y / (Ld^2 + y^2))
    float curvature = (2.0 * errY) / ((Ld * Ld) + (errY * errY));

    // 將曲率映射為差速輪的左右動力分配 (加入輪距 L 作為力臂補償)
    float vL = SPEED_BASE * (1.0 - curvature * L / 2.0);
    float vR = SPEED_BASE * (1.0 + curvature * L / 2.0);

    // 輸出計算結果至馬達驅動層
    applyMotorPower((int)vL, (int)vR);

    delay(10); // 維持系統 100Hz 取樣率，避免迴圈空轉與 PWM 雜訊
}

// 5. 系統主流程 (Main State Machine)
void setup() {
    // 初始化所有腳位與暫存器
    initHardware();
    haltVehicle(); // 確保開機瞬間馬達無動力輸出
    currentState = SystemState::INIT_SAFE; // 強制進入最高安全層級
}

void loop() {
    // 絕對硬體優先權：不論系統處於何種狀態，只要安全開關觸發即強制切斷
    // TSMS(總電源)斷開 或 ESTOP(急停)被按下
    if(digitalRead(PIN_BTN_ESTOP) == LOW || digitalRead(PIN_BTN_TSMS) == HIGH) {
        currentState = SystemState::EMERGENCY;
    }

    // 狀態機控制核心 (執行對應狀態的行為與條件轉移)
    switch(currentState) {
        case SystemState::INIT_SAFE:
            haltVehicle(); // 物理鎖定馬達
            setASL(false, true, false); // 亮綠燈代表安全
            timeLost = 0; // 重置計時器
            // 轉移條件：解開 TSMS 且 確認未按下 ESTOP，才允許進入待機
            if(digitalRead(PIN_BTN_TSMS) == LOW && digitalRead(PIN_BTN_ESTOP) == HIGH) {
                currentState = SystemState::STANDBY;
            }
            break;

        case SystemState::STANDBY:
            haltVehicle(); // 維持馬達鎖定
            setASL(false, false, true); // 亮藍燈代表通電準備就緒
            // 轉移條件：接收到 VLS (Vehicle Launch Switch) 起跑訊號
            if(digitalRead(PIN_BTN_VLS) == LOW) {
                currentState = SystemState::TRACKING;
                Serial.println("VLS Triggered. Autonomous Run Start.");
                delay(300); // 簡單防彈跳延遲，避免開關雜訊造成狀態誤判
            }
            break;

        case SystemState::TRACKING:
            setASL(true, false, false); // 亮紅燈代表車輛具備動能，警告人員遠離
            executePurePursuit(); // 將控制權移交給循跡演算法
            break;

        case SystemState::EMERGENCY:
            haltVehicle(); // EBS 觸發，立刻切斷輸出並物理煞停
            setASL(false, true, false); // 亮綠燈代表系統已被安全限制
            // 轉移條件 (解除 EBS 鎖定)：確認總電源開啟 + 放開急停鈕 + 人工按下 RESET 鈕確認
            if(digitalRead(PIN_BTN_TSMS) == LOW && 
               digitalRead(PIN_BTN_ESTOP) == HIGH && 
               digitalRead(PIN_BTN_RESET) == LOW) {
                currentState = SystemState::STANDBY; // 退回待機狀態等待重新起跑
                Serial.println("EBS Reset.");
                delay(300); // 按鈕防彈跳
            }
            break;
    }
}
