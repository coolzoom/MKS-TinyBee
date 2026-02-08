#include "TimerOne.h"                     // TimerOne 库：提供 16 位硬件定时器（Timer1）功能
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>              // EnableInterrupt 库：简化外部中断的注册

// ---------- 步进电机 IO 定义 ----------
#define EN  8     // 步进电机使能端，低电平有效（EN=LOW → 驱动器工作）
#define X_DIR  5  // X 轴方向控制
#define Y_DIR  6  // Y 轴方向控制
#define Z_DIR  7  // Z 轴方向控制
#define A_DIR  13 // A 轴方向控制 //CNC s hield 13 ； 一体 10

#define X_STP  2  // X 轴步进脉冲输出
#define Y_STP  3  // Y 轴步进脉冲输出
#define Z_STP  4  // Z 轴步进脉冲输出
#define A_STP  12  // A 轴步进脉冲输出 //cnc shield 12 ； 一体 9

// ---------- 电机编号（仅作标识） ----------
int MotorX  = 1;  // MotorX 电机 X 右上
int MotorY  = 2;  // MotorY 电机 Y 左上
int MotorZ  = 3;  // MotorZ 电机 Z 右下
int MotorA  = 4;  // MotorA 电机 A 左下

// ---------- 接收机 PWM 输入 ----------
byte receiver_pins[4] = {14, 15, 16, 17}; // 通道 1~4 对应 Arduino A0~A3（数字 14~17）
//A0 / 14 / cncshield abort 左右
//A1 / 15 / cncshield hold 前后
//A2 / 16 / cncshield resume 
//A3 / 17 / cncshield coolen 旋转

volatile int receiver_input[4];          // 存放每个通道的 PWM 脉宽（单位 µs），volatile 防止中断优化

// ---------- 用于测量 PWM 脉宽的计时变量 ----------
unsigned long timer_1, timer_2, timer_3, timer_4; // 每个通道的上升沿时间戳

// ---------- 运行控制变量 ----------
int stepflage = 0;          // 是否进入步进模式（1=步进，0=停止）
bool isRaytracingEnabled = true; // 光线追踪功能开关，默认开启

// ---------- 光线追踪对中控制 ----------
bool isCentered = false;     // 机器人是否已对中
unsigned long raytracingStartTime = 0; // 光线追踪开始时间
const unsigned long RAYTRACING_TIMEOUT = 10000; // 光线追踪超时时间（10秒）
const int CENTER_THRESHOLD_LOW = 1400; // 对中阈值下限
const int CENTER_THRESHOLD_HIGH = 1550; // 对中阈值上限
const int RAY_CENTER_PULSE = 1470;     // 光线传感器“正中”对应的PWM脉宽
const int RAY_DEADBAND = 40;           // 允许的中心死区（±40us左右约等于1个传感器偏差）

// ---------- 光线循迹 PID 控制 ----------
// 这里的PID用于根据“离中心的偏差”动态调节横移速度：偏差越大 → 纠偏越快
// 注意：由于当前硬件是四个电机共享同一个Timer1步频，无法实现真正的矢量合成，
// 因此我们在自动循迹模式下仍然是“前进 / 左移 / 右移”三选一，只是用PID调节横移速度。
float rayKp = 0.8f;
float rayKi = 0.0f;
float rayKd = 0.1f;
float rayErrorIntegral = 0.0f;
float rayLastErrorNorm = 0.0f;
unsigned long rayLastPidTime = 0;
const int   RAY_MAX_ERROR = 500;             // 误差归一化上限（大于该值按最大偏差处理）
const unsigned long RAY_PID_INTERVAL_MIN = 600;   // 横移最快速度（步进间隔最小，单位us）
const unsigned long RAY_PID_INTERVAL_MAX = 20000; // 横移最慢速度（步进间隔最大，单位us）

unsigned long computeRayStepInterval(int error);

// 标记当前是否在“前进/后退”模式（由遥控或串口命令设置）
bool isForwardOrBackward = false;

// 光线纠偏状态：避免左右来回抖动
bool isRayCorrecting = false;              // 当前是否处于一次横向纠偏周期中
int  rayCorrectDir = 0;                    // 纠偏方向：-1=向左，+1=向右
unsigned long rayCorrectStartMs = 0;       // 本次纠偏开始时间
const unsigned long RAY_CORRECT_MIN_MS = 60;   // 至少纠偏这么久（避免太短看不到效果）
const unsigned long RAY_CORRECT_MAX_MS = 250;  // 最多纠偏这么久（避免严重过冲）

// ---------- 串口命令相关变量 ----------
String serialCommand = "";  // 存储接收到的串口命令
bool commandReady = false;   // 命令是否准备好处理
bool isSerialControlled = false; // 标记是否正在通过串口命令控制机器人运动

// ---------- 速度计算相关变量 ----------
#define WHEEL_DIAMETER 100.0  // 轮子直径，单位：mm
#define GEAR_RATIO 14.0       // 减速比
#define STEP_ANGLE 1.8        // 步进电机步距角，单位：度
#define MECANUM_TRANSFORM_FACTOR 0.7071  // 麦克纳姆轮横向移动转换因子（√2/2）
float stepDistance;          // 每一步的移动距离，单位：mm
float currentSpeed = 0.0;    // 当前速度，单位：mm/s

// ---------- 距离控制相关变量 ----------
bool isDistanceControlled = false; // 标记是否正在进行距离控制
volatile unsigned long targetSteps = 0; // 目标步数（volatile 确保在中断中正确访问）
volatile unsigned long currentSteps = 0; // 当前已走步数（volatile 确保在中断中正确访问）

// ---------- 底板识别串口输出（供上位机自动连接识别） ----------
unsigned long lastIdentifySent = 0;      // 上次发送识别串的时间
const unsigned long IDENTIFY_INTERVAL_MS = 2000;  // 每 2 秒发送一次，避免上位机在 setup() 之后才打开串口时收不到

// ---------- 函数声明 ----------
void pwmReceive();          // 中断回调：捕获 PWM 脉宽
void one_stepx();           // 定时器中断回调：产生四电机同步步进脉冲
void processSerialCommand(); // 串口命令处理函数

// ==================== setup() ====================
void setup(){
  Serial.begin(115200);               // 调试串口，波特率 115200

  // 将所有步进电机相关的 IO 设为输出
  pinMode(X_DIR,OUTPUT); pinMode(X_STP,OUTPUT);
  pinMode(Y_DIR,OUTPUT); pinMode(Y_STP,OUTPUT);
  pinMode(Z_DIR,OUTPUT); pinMode(Z_STP,OUTPUT);
  pinMode(A_DIR,OUTPUT); pinMode(A_STP,OUTPUT);
  pinMode(EN,OUTPUT);
  digitalWrite(EN,LOW);               // 使能驱动器（低电平有效）

  // ---------- Timer1 配置 ----------
  // Timer1 初始化为每 1 ms（1000 µs）触发一次中断，调用 one_stepx()
  Timer1.initialize(1000);            
  // Timer1.pwm(9, 512); // （保留）示例：在 D9 上产生 50% 占空比 PWM
  Timer1.attachInterrupt(one_stepx);  // 注册中断回调函数

  // ---------- 为四个 PWM 接收引脚注册 CHANGE 中断 ----------
  for (int i = 0; i < 4; i++) {
       pinMode(receiver_pins[i], INPUT_PULLUP);               // 使用内部上拉
       enableInterrupt(receiver_pins[i], pwmReceive, CHANGE); // 任意电平变化触发
   }

  // 开启全局中断（EnableInterrupt 库内部已调用 sei()，此处保留以示意）
  sei();
  
  // 计算步进电机每一步的实际移动距离
  // stepDistance = (π * 轮子直径) / (360/步距角 * 减速比)
  stepDistance = (PI * WHEEL_DIAMETER) / ((360.0 / STEP_ANGLE) * GEAR_RATIO);
  Serial.print("每一步移动距离: ");
  Serial.print(stepDistance);
  Serial.println(" mm");
  Serial.println("robotbase");  // 供上位机自动连接识别（含 "robotbase" 即可）
}

// ==================== loop() ====================
void loop(){
//   // 周期发送识别串，便于上位机在 setup() 之后才打开串口时也能识别本设备
//   if (millis() - lastIdentifySent >= IDENTIFY_INTERVAL_MS) {
//     Serial.println("robotbase");
//     lastIdentifySent = millis();
//   }

  // 接收串口命令
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == '\n' || incomingChar == '\r') {
      // 命令结束符，设置命令准备好标志
      if (serialCommand.length() > 0) {
        commandReady = true;
      }
    } else {
      // 添加接收到的字符到命令字符串
      serialCommand += incomingChar;
    }
  }
  
  // 处理接收到的串口命令
  if (commandReady) {
    processSerialCommand();
    serialCommand = "";
    commandReady = false;
  }
  

  
  delay(50);   // 主循环每 50 ms 读取一次最新的 PWM 脉宽
  //Serial.print("舵机引脚: pwm ");
  //Serial.println(receiver_input[0]);
  
  // ---------- 读取遥控通道并决定运动方向 ----------
  // 通道 2（receiver_input[1]）控制前后 A1 / 15 / cncshield hold
  if(receiver_input[1] > 1900 && receiver_input[1] < 2100){ // 前进
    digitalWrite(X_DIR,false);
    digitalWrite(Y_DIR,true);
    digitalWrite(Z_DIR,false);
    digitalWrite(A_DIR,true);
    Serial.println("1");
    stepflage = 1;               // 进入步进模式
    isCentered = false; // 手动控制时重置对中状态
    raytracingStartTime = 0; // 重置计时器
    isForwardOrBackward = true;
  } 
  else if(receiver_input[1] < 1100 && receiver_input[1] > 900){ // 后退
    digitalWrite(X_DIR,true);
    digitalWrite(Y_DIR,false);
    digitalWrite(Z_DIR,true);
    digitalWrite(A_DIR,false);
    Serial.println("2");
    stepflage = 1;
    isCentered = false; // 手动控制时重置对中状态
    raytracingStartTime = 0; // 重置计时器
    isForwardOrBackward = true;
  }

  //raytacking 0-14 个输出分别为540 - 2400， 第七个：1340， 中点第8个光感 1470， 第九个：1600
  //读值 548 672 804 
  //
  // 通道 1（receiver_input[2]）控制左右平移 A2 / 16 / cncshield resume 左右
  else if(isRaytracingEnabled && receiver_input[2] < CENTER_THRESHOLD_LOW && receiver_input[2] > 500){ // 左移
    digitalWrite(X_DIR,false);
    digitalWrite(Y_DIR,false);
    digitalWrite(Z_DIR,true);
    digitalWrite(A_DIR,true);
    Serial.println("7");
    stepflage = 1;
    isForwardOrBackward = false;
    // 光线追踪时，左右平移速度调整到约10 mm/s（间隔约11220微秒）
    //Timer1.initialize(5610);
  } 
  else if(isRaytracingEnabled && receiver_input[2] > CENTER_THRESHOLD_HIGH && receiver_input[2] < 2500){ // 右移
    digitalWrite(X_DIR,true);
    digitalWrite(Y_DIR,true);
    digitalWrite(Z_DIR,false);
    digitalWrite(A_DIR,false);
    Serial.println("8");
    stepflage = 1;
    isForwardOrBackward = false;
    // 光线追踪时，左右平移速度调整到约10 mm/s（间隔约11220微秒）
    //Timer1.initialize(5610);
  }

    // 通道 1（receiver_input[0]）控制左右平移 A0 / 14 / cncshield abort 左右
  else if(receiver_input[0] < 1100 && receiver_input[0] > 900){ // 左移
    digitalWrite(X_DIR,false);
    digitalWrite(Y_DIR,false);
    digitalWrite(Z_DIR,true);
    digitalWrite(A_DIR,true);
    Serial.println("3");
    stepflage = 1;
    isForwardOrBackward = false;
  } 
  else if(receiver_input[0] > 1900 && receiver_input[0] < 2100){ // 右移
    digitalWrite(X_DIR,true);
    digitalWrite(Y_DIR,true);
    digitalWrite(Z_DIR,false);
    digitalWrite(A_DIR,false);
    Serial.println("4");
    stepflage = 1;
    isForwardOrBackward = false;
  }

  // 通道 4（receiver_input[3]）控制旋转 A3 / 17 / cncshield coolen 旋转
  else if(receiver_input[3] < 1400 && receiver_input[3] > 500){ // 逆时针
    digitalWrite(X_DIR,true);
    digitalWrite(Y_DIR,true);
    digitalWrite(Z_DIR,true);
    digitalWrite(A_DIR,true);
    Serial.println("5");
    stepflage = 1;
    isForwardOrBackward = false;
  } 
  else if(receiver_input[3] > 1550 && receiver_input[3] < 2500){ // 顺时针
    digitalWrite(X_DIR,false);
    digitalWrite(Y_DIR,false);
    digitalWrite(Z_DIR,false);
    digitalWrite(A_DIR,false);
    Serial.println("6");
    stepflage = 1;
    isForwardOrBackward = false;
  }
  else if(!isSerialControlled){
    stepflage = 0;   // 没有有效指令且没有收到串口命令，停止步进
    isForwardOrBackward = false;
  }     
  
  // ---------- 光线辅助对中（前进/后退时自动横向微调，带防抖/防过冲） ----------
  // 条件：启用光线追踪 + 当前在运动 + 方向为前进/后退
  if (isRaytracingEnabled && stepflage == 1 && isForwardOrBackward) {
    int rayPulse = receiver_input[2];  // 来自 raytracing 模块的PWM
    if (rayPulse > 500 && rayPulse < 2500) {
      int error = rayPulse - RAY_CENTER_PULSE; // 误差>0：光斑在右侧；<0：光斑在左侧
      int errAbs = abs(error);

      // 如果当前脉宽已经回到中心死区以内，立即停止任何纠偏动作，
      // 不再执行本次循环中的左右移动/旋转，保持当前前进/后退状态。
      if (errAbs <= RAY_DEADBAND) {
        isRayCorrecting = false;
        return;
      }

      // 如果当前不在纠偏周期内，且偏差明显，则启动一次纠偏
      if (!isRayCorrecting && errAbs > (RAY_DEADBAND + 20)) { // 启动阈值略大于死区，形成滞回
        isRayCorrecting = true;
        rayCorrectDir = (error < 0) ? -1 : 1;  // 记录本次纠偏方向
        rayCorrectStartMs = millis();
      }

      // 如果正在纠偏，则按记录的方向持续横移一小段时间
      if (isRayCorrecting) {
        unsigned long now = millis();
        unsigned long elapsed = now - rayCorrectStartMs;

        // 到达最短纠偏时间后，如果误差已经进入死区，或者方向反转，结束本次纠偏
        if (elapsed >= RAY_CORRECT_MIN_MS) {
          if (errAbs <= RAY_DEADBAND || (rayCorrectDir * error) <= 0) {
            isRayCorrecting = false;
          }
        }

        // 超过最大纠偏时间也强制结束，避免严重过冲
        if (elapsed >= RAY_CORRECT_MAX_MS) {
          isRayCorrecting = false;
        }

        // 仍在纠偏中：根据记录的方向输出横移，并用PID调节速度
        if (isRayCorrecting) {
          if (rayCorrectDir < 0) {
            // 记录方向为左 → 左平移纠偏
            digitalWrite(X_DIR,false);
            digitalWrite(Y_DIR,false);
            digitalWrite(Z_DIR,true);
            digitalWrite(A_DIR,true);
          } else if (rayCorrectDir > 0) {
            // 记录方向为右 → 右平移纠偏
            digitalWrite(X_DIR,true);
            digitalWrite(Y_DIR,true);
            digitalWrite(Z_DIR,false);
            digitalWrite(A_DIR,false);
          }
          unsigned long interval = computeRayStepInterval(error);
          Timer1.initialize(interval);
          Timer1.attachInterrupt(one_stepx);
          Timer1.start();
        }
      }
    }
  } else {
    // 非前进/后退运动或未启用光线追踪时，不做纠偏
    isRayCorrecting = false;
  }
  
  // 不再强制重置isSerialControlled，让串口命令控制状态保持到收到停止命令为止
}

// ==================== 串口命令处理函数 ====================
// 支持的命令格式：
// F[速度] - 前进 (Forward)
// B[速度] - 后退 (Backward)
// L[速度] - 左转 (Left)
// R[速度] - 右转 (Right)
// SL[速度] - 左平移 (Slide Left)
// SR[速度] - 右平移 (Slide Right)
// S - 停止 (Stop)
// 速度参数可选，单位：mm/s，默认速度：50 mm/s
void processSerialCommand() {
  //Serial.print("收到命令: ");
  //Serial.println(serialCommand);

  // 上位机自动连接探测：立即回复识别串，无需 ACK，缩短扫描时间
  if (serialCommand.equalsIgnoreCase("ID") || serialCommand.equalsIgnoreCase("?")) {
    Serial.println("robotbase");
    return;
  }

  // 立即发送命令确认
  Serial.println("ACK");
  
  // 优先处理停止命令
  if (serialCommand.equalsIgnoreCase("S")) {
    stepflage = 0;
    isSerialControlled = false; // 停止时重置串口控制标志
    isDistanceControlled = false; // 停止时重置距离控制模式
    isForwardOrBackward = false;
    Serial.println("ok");
    return;
  }
  
  // 处理状态查询命令
  if (serialCommand.equalsIgnoreCase("STATUS")) {
    //Serial.print("STATUS:");
    //Serial.print(",isSerialControlled=");
    //Serial.print(isSerialControlled);
    Serial.print("move_state=");
    Serial.println(stepflage);
    //Serial.print(",isDistanceControlled=");
    //Serial.print(isDistanceControlled);
    //Serial.print(",currentSpeed=");
    //Serial.print(currentSpeed);
    //Serial.print(",isRaytracingEnabled=");
    //Serial.print(isRaytracingEnabled);
    //Serial.print(",isCentered=");
    //Serial.println(isCentered);
    return;
  }
  
  // 处理光线追踪启停命令（仅控制是否使用光线辅助，不再进入单独FOLLOW模式）
  if (serialCommand.equalsIgnoreCase("FINDRAY")) {
    isRaytracingEnabled = true;
    isCentered = false; // 开启光线追踪时重置对中状态
    raytracingStartTime = millis(); // 开始计时
    Serial.println("raytracking enabled");
    return;
  } else if (serialCommand.equalsIgnoreCase("DISABLERAY")) {
    isRaytracingEnabled = false;
    isCentered = false; // 关闭光线追踪时重置对中状态
    raytracingStartTime = 0; // 重置计时器
    Serial.println("raytracking disabled");
    return;
  }
  
  // 设置串口命令控制标志
  isSerialControlled = true;
  
  char command = serialCommand[0];
  float speed = 50.0; // 默认速度 50 mm/s
  
  // 提取参数（距离和速度）
  float distance = 0.0; // 默认距离0，即速度控制模式
  if (serialCommand.length() > 1) {
    // 检查是否为距离控制命令（SDL, SDR, FD, BD, LD, RD）
    bool isDistanceCmd = false;
    if ((command == 'S' && serialCommand.length() > 3 && serialCommand[1] == 'D' && (serialCommand[2] == 'L' || serialCommand[2] == 'R')) ||
        (command == 'F' && serialCommand[1] == 'D') ||
        (command == 'B' && serialCommand[1] == 'D') ||
        (command == 'L' && serialCommand[1] == 'D') ||
        (command == 'R' && serialCommand[1] == 'D')) {
      isDistanceCmd = true;
    }
    
    if (isDistanceCmd) {
      // 距离控制命令格式：SDL[距离]或SDR[距离]或FD[距离]或BD[距离]或带速度参数
      int paramStart = 0;
      
      if (command == 'S') {
        paramStart = 3; // SDL或SDR从索引3开始
      } else {
        paramStart = 2; // FD或BD从索引2开始
      }
      
      int speedStart = serialCommand.indexOf(':', paramStart); // 寻找速度分隔符
      if (speedStart > 0) {
        // 有速度参数，格式：[命令][距离]:[速度]
        String distanceStr = serialCommand.substring(paramStart, speedStart);
        distance = distanceStr.toFloat();
        String speedStr = serialCommand.substring(speedStart + 1);
        speed = speedStr.toFloat();
      } else {
        // 无速度参数，格式：[命令][距离]
        String distanceStr = serialCommand.substring(paramStart);
        distance = distanceStr.toFloat();
      }
    } else {
      // 普通命令，提取速度参数
      if (command == 'S' && (serialCommand[1] == 'L' || serialCommand[1] == 'R')) {
        // 双字符命令（SL, SR）
        String speedStr = serialCommand.substring(2);
        speed = speedStr.toFloat();
      } else {
        // 单字符命令（F, B, L, R）
        String speedStr = serialCommand.substring(1);
        speed = speedStr.toFloat();
      }
    }
    
    // 验证并限制速度范围 (0-200 mm/s)
    if (isnan(speed) || speed < 0) {
        speed = 50.0; // 使用默认速度
        Serial.println("警告：速度参数无效，使用默认速度50 mm/s");
    } else if (speed > 200) {
        speed = 200;
        Serial.println("警告：速度超过最大值200 mm/s，已限制");
    }
    
    // 验证并限制距离范围 (0-10000 mm)
    if (isnan(distance) || distance < 0) {
        distance = 0;
        Serial.println("警告：距离参数无效，设置为0");
    } else if (distance > 10000) {
        distance = 10000;
        Serial.println("警告：距离超过最大值10000 mm，已限制");
    }
    
    // 验证命令格式
    if (command == 'F' || command == 'B' || command == 'L' || command == 'R' || command == 'S') {
        // 命令类型有效
    } else {
        Serial.print("错误：未知命令类型: ");
        Serial.println(command);
        return;
    }
  }
  
  // 根据速度计算步进间隔时间
  if (speed > 0) {
    // 计算每秒步数: speed / stepDistance
    float stepsPerSecond = speed / stepDistance;
    
    // 检查是否为横向移动命令（SDL/SDR/SL/SR），如果是，应用麦克纳姆轮转换因子
    bool isTransverseMove = false;
    if ((command == 'S' && serialCommand.length() > 1) && 
        (serialCommand[1] == 'L' || serialCommand[1] == 'R')) {
      isTransverseMove = true;
    }
    
    // 横向移动时需要增加轮子转速以补偿转换因子
    if (isTransverseMove) {
      stepsPerSecond /= MECANUM_TRANSFORM_FACTOR;
    }
    
    // 计算每步间隔时间 (微秒): 1,000,000 / stepsPerSecond
    unsigned long interval = 1000000.0 / stepsPerSecond;
    
    // 限制最小间隔时间（防止超过定时器中断频率限制）
    // Timer1的最大频率约为20kHz，所以最小间隔约为50微秒
    if (interval < 50) interval = 50;
    // 限制最大间隔时间（防止速度过慢）
    if (interval > 100000) interval = 100000; // 0.1秒/步，约0.011 mm/s
    
    // 重新初始化Timer1以调整中断频率
    Timer1.initialize(interval);
    Timer1.attachInterrupt(one_stepx);
    Timer1.start(); // 确保Timer1启动
    
    currentSpeed = speed;
    Serial.print("速度设置为: ");
    Serial.print(speed);
    Serial.print(" mm/s，间隔: ");
    Serial.print(interval);
    Serial.println(" us");
  }
  
  // 执行相应的运动命令
  switch (command) {
    case 'F': // 前进
      if (serialCommand.length() > 1 && serialCommand[1] == 'D') { // 距离控制前进 FD
        digitalWrite(X_DIR, false);
        digitalWrite(Y_DIR, true);
        digitalWrite(Z_DIR, false);
        digitalWrite(A_DIR, true);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("距离控制前进，距离: ");
        Serial.print(distance);
        Serial.print(" mm，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        
        // 计算目标步数 - 横向移动需要应用麦克纳姆轮转换因子
        if (distance > 0) {
          targetSteps = distance / (stepDistance * MECANUM_TRANSFORM_FACTOR);
          currentSteps = 0;
          isDistanceControlled = true;
        }
      } else { // 普通前进
        digitalWrite(X_DIR, false);
        digitalWrite(Y_DIR, true);
        digitalWrite(Z_DIR, false);
        digitalWrite(A_DIR, true);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("前进，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        // 关闭距离控制模式
        isDistanceControlled = false;
      }
      isForwardOrBackward = (stepflage == 1);
      break;
      
    case 'B': // 后退
      if (serialCommand.length() > 1 && serialCommand[1] == 'D') { // 距离控制后退 BD
        digitalWrite(X_DIR, true);
        digitalWrite(Y_DIR, false);
        digitalWrite(Z_DIR, true);
        digitalWrite(A_DIR, false);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("距离控制后退，距离: ");
        Serial.print(distance);
        Serial.print(" mm，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        
        // 计算目标步数 - 横向移动需要应用麦克纳姆轮转换因子
        if (distance > 0) {
          targetSteps = distance / (stepDistance * MECANUM_TRANSFORM_FACTOR);
          currentSteps = 0;
          isDistanceControlled = true;
        }
      } else { // 普通后退
        digitalWrite(X_DIR, true);
        digitalWrite(Y_DIR, false);
        digitalWrite(Z_DIR, true);
        digitalWrite(A_DIR, false);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("后退，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        // 关闭距离控制模式
        isDistanceControlled = false;
      }
      isForwardOrBackward = (stepflage == 1);
      break;
      
    case 'L': // 左转
      if (serialCommand.length() > 1 && serialCommand[1] == 'D') { // 距离控制左转 LD
        digitalWrite(X_DIR, true);
        digitalWrite(Y_DIR, true);
        digitalWrite(Z_DIR, true);
        digitalWrite(A_DIR, true);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("距离控制左转，角度: ");
        Serial.print(distance);
        Serial.print(" 度，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        
        // 计算目标步数 - 旋转角度需要转换为轮边移动距离
        // 这里需要知道机器人的轮距或旋转半径才能精确计算
        // 假设旋转半径为机器人中心到轮子的距离（近似值）
        float rotationRadius = 150.0; // 假设旋转半径为150mm
        float wheelTravelDistance = (PI * 2 * rotationRadius) * (distance / 360.0);
        if (wheelTravelDistance > 0) {
          targetSteps = wheelTravelDistance / stepDistance;
          currentSteps = 0;
          isDistanceControlled = true;
        }
      } else { // 普通左转
        digitalWrite(X_DIR, true);
        digitalWrite(Y_DIR, true);
        digitalWrite(Z_DIR, true);
        digitalWrite(A_DIR, true);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("左转，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        // 关闭距离控制模式
        isDistanceControlled = false;
      }
      isForwardOrBackward = false;
      break;
      
    case 'R': // 右转
      if (serialCommand.length() > 1 && serialCommand[1] == 'D') { // 距离控制右转 RD
        digitalWrite(X_DIR, false);
        digitalWrite(Y_DIR, false);
        digitalWrite(Z_DIR, false);
        digitalWrite(A_DIR, false);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("距离控制右转，角度: ");
        Serial.print(distance);
        Serial.print(" 度，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        
        // 计算目标步数 - 旋转角度需要转换为轮边移动距离
        float rotationRadius = 150.0; // 假设旋转半径为150mm
        float wheelTravelDistance = (PI * 2 * rotationRadius) * (distance / 360.0);
        if (wheelTravelDistance > 0) {
          targetSteps = wheelTravelDistance / stepDistance;
          currentSteps = 0;
          isDistanceControlled = true;
        }
      } else { // 普通右转
        digitalWrite(X_DIR, false);
        digitalWrite(Y_DIR, false);
        digitalWrite(Z_DIR, false);
        digitalWrite(A_DIR, false);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("右转，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        // 关闭距离控制模式
        isDistanceControlled = false;
      }
      isForwardOrBackward = false;
      break;
      
    case 'S':
      if (serialCommand.length() > 3 && serialCommand[1] == 'D' && serialCommand[2] == 'L') { // 距离控制左平移 SDL
        digitalWrite(X_DIR, false);
        digitalWrite(Y_DIR, false);
        digitalWrite(Z_DIR, true);
        digitalWrite(A_DIR, true);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("距离控制左平移，距离: ");
        Serial.print(distance);
        Serial.print(" mm，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        
        // 计算目标步数 - 横向移动需要应用麦克纳姆轮转换因子
        if (distance > 0) {
          targetSteps = distance / (stepDistance * MECANUM_TRANSFORM_FACTOR);
          currentSteps = 0;
          isDistanceControlled = true;
        }
      } else if (serialCommand.length() > 3 && serialCommand[1] == 'D' && serialCommand[2] == 'R') { // 距离控制右平移 SDR
        digitalWrite(X_DIR, true);
        digitalWrite(Y_DIR, true);
        digitalWrite(Z_DIR, false);
        digitalWrite(A_DIR, false);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("距离控制右平移，距离: ");
        Serial.print(distance);
        Serial.print(" mm，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        
        // 计算目标步数 - 横向移动需要应用麦克纳姆轮转换因子
        if (distance > 0) {
          targetSteps = distance / (stepDistance * MECANUM_TRANSFORM_FACTOR);
          currentSteps = 0;
          isDistanceControlled = true;
        }
      } else if (serialCommand.length() > 1 && serialCommand[1] == 'L') { // 左平移
        digitalWrite(X_DIR, false);
        digitalWrite(Y_DIR, false);
        digitalWrite(Z_DIR, true);
        digitalWrite(A_DIR, true);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("左平移，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        // 关闭距离控制模式
        isDistanceControlled = false;
      } else if (serialCommand.length() > 1 && serialCommand[1] == 'R') { // 右平移
        digitalWrite(X_DIR, true);
        digitalWrite(Y_DIR, true);
        digitalWrite(Z_DIR, false);
        digitalWrite(A_DIR, false);
        stepflage = speed > 0 ? 1 : 0;
        Serial.print("右平移，速度: ");
        Serial.print(speed);
        Serial.println(" mm/s");
        // 关闭距离控制模式
        isDistanceControlled = false;
      }
      isForwardOrBackward = false;
      break;
    
    default:
      Serial.print("未知命令: ");
      Serial.println(command);
      break;
  }
}

// ==================== 中断：捕获 PWM 脉宽 ====================
// 每个遥控通道的 PWM 信号为 1 ms~2 ms 高电平宽度（对应舵机/RC 信号）
// 通过在上升沿记录时间，在下降沿计算宽度，即可得到遥控杆位置信息
void pwmReceive() {
    // arduinoInterruptedPin：当前触发中断的引脚编号（EnableInterrupt 库提供）
    int currPin = arduinoInterruptedPin;
    // micros()：当前微秒计数
    unsigned long currTime = micros();

    // arduinoPinState：当前引脚电平（1=HIGH, 0=LOW）
    int pinLevel = arduinoPinState;

    // ---------- 通道 1 ----------
    if (currPin == 14 && pinLevel > 0) {          // 上升沿 → 记录起始时间
        timer_1 = currTime;
    } else if (currPin == 14 && pinLevel == 0) { // 下降沿 → 计算脉宽
        receiver_input[0] = currTime - timer_1;  // 单位 µs
    }

    // ---------- 通道 2 ----------
    if (currPin == 15 && pinLevel > 0) {
        timer_2 = currTime;
    } else if (currPin == 15 && pinLevel == 0) {
        receiver_input[1] = currTime - timer_2;
    }

    // ---------- 通道 3 ----------
    if (currPin == 16 && pinLevel > 0) {
        timer_3 = currTime;
    } else if (currPin == 16 && pinLevel == 0) {
        receiver_input[2] = currTime - timer_3;
    }

    // ---------- 通道 4 ----------
    if (currPin == 17 && pinLevel > 0) {
        timer_4 = currTime;
    } else if (currPin == 17 && pinLevel == 0) {
        receiver_input[3] = currTime - timer_4;
    }
}

// ==================== Timer1 中断：四电机同步步进 ====================
// 该函数被硬件定时器调用。只要 stepflage 为 1，
// 就会在 **同一时刻** 同时产生 X、Y、Z、A 四路 STEP 脉冲，
// 从而实现四个步进电机的同步运动（方向已在 loop() 中预先设置）。
void one_stepx() {                        
  if(stepflage == 1){
    // 产生高电平脉冲
    digitalWrite(X_STP, HIGH);
    digitalWrite(Y_STP, HIGH);
    digitalWrite(Z_STP, HIGH);
    digitalWrite(A_STP, HIGH);
    // 脉冲宽度固定为 1 微秒（步进驱动器只需要极短的脉冲即可识别）
    asm("nop"); // 大约 1 微秒的延迟
    
    // 产生低电平脉冲（步进完成一次）
    digitalWrite(X_STP, LOW);
    digitalWrite(Y_STP, LOW);
    digitalWrite(Z_STP, LOW);
    digitalWrite(A_STP, LOW);
    
    // 步数计数和距离控制检查
    if (isDistanceControlled) {
      currentSteps++;
      if (currentSteps >= targetSteps) {
        // 达到目标步数，停止运动
        stepflage = 0;
        isDistanceControlled = false;
        isSerialControlled = false;
        // 输出完成信息，方便上位机知道已经走到位
        //Serial.println("距离控制运动完成");
        Serial.println("ok");
      }
    }
  }   
}

// ==================== 光线循迹 PID：根据误差动态调节横移速度 ====================
// 输入：error（当前PWM与中心PWM的差值，单位us，>0 为偏右，<0 为偏左）
// 输出：Timer1步进间隔（微秒）。间隔越小，车横移越快。
unsigned long computeRayStepInterval(int error) {
  // 只关心偏差的绝对值大小
  float e = (float)abs(error);
  if (e > RAY_MAX_ERROR) {
    e = (float)RAY_MAX_ERROR;
  }

  // 误差归一化到 0~1
  float errNorm = e / (float)RAY_MAX_ERROR;

  // 计算 dt（秒），带有简单防抖
  unsigned long now = millis();
  float dt = 0.02f; // 默认20ms
  if (rayLastPidTime != 0 && now > rayLastPidTime) {
    dt = (now - rayLastPidTime) / 1000.0f;
  }
  rayLastPidTime = now;

  // 积分与微分
  rayErrorIntegral += errNorm * dt;
  float dErr = (errNorm - rayLastErrorNorm) / dt;
  rayLastErrorNorm = errNorm;

  // 计算PID输出（0~约>1，后面再做限幅）
  float u = rayKp * errNorm + rayKi * rayErrorIntegral + rayKd * dErr;

  // 安全限幅：小于0按0处理，大于1按1处理
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;

  // 将控制量映射到步进间隔：u=0 → 最慢，u=1 → 最快
  unsigned long interval = RAY_PID_INTERVAL_MAX -
                           (unsigned long)((RAY_PID_INTERVAL_MAX - RAY_PID_INTERVAL_MIN) * u);

  // 保险再次限幅
  if (interval < RAY_PID_INTERVAL_MIN) interval = RAY_PID_INTERVAL_MIN;
  if (interval > RAY_PID_INTERVAL_MAX) interval = RAY_PID_INTERVAL_MAX;

  return interval;
}
