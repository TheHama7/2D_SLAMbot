// === === === MOTOR PIN DEFINITION 电机相关的引脚定义 === === ===

// 本例程用于读取两路编码器所连接的电机车轮转速，后续例程中会介绍如何闭环控制电机速度

// 定义A/B两路编码器的引脚，每路编码器由两根信号线来获取反馈信息，分别与对应的霍尔元件相连接
// 对于单独的某一路编码器：通过检测其中一个霍尔元件的高低电平变化频率来计算电机转速，再通过判断另一个
// 霍尔传感器的高低电平状态来判断旋转方向。

// A路编码器的引脚定义
const uint16_t AENCA = 35;        // Encoder A input A_C2(B)
const uint16_t AENCB = 34;        // Encoder A input A_C1(A)

// B路编码器的引脚定义
const uint16_t BENCB = 16;        // Encoder B input B_C2(B)
const uint16_t BENCA = 27;        // Encoder B input B_C1(A)

// 用于机算在“interval”时间（单位ms）内的编码器的某一个霍尔传感器的高低电平变化次数
// 因为后面初始化中断时使用的是RISING，所以具体来说是低电平变高电平的次数
volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count  = 0;

// 机算速度的周期时间，每隔这么多ms机算一次速度
int interval = 100;

// 当前的时间
long currentMillis = 0;

// 用来存储左右两侧的转速
double rpm_B = 0;
double rpm_A = 0;

// 电机的减速比，减速电机的电机转速和输出轴的转速是不一样的
// 例如DCGM3865这款电机，减速比是1：42，意味着电机转动42圈，输出轴转动1圈
// 对应输出轴的一圈，电机需要转的圈数越多，减速比越大，通常扭矩越大
// 一下以DCGM3865电机（减速比1：42）为例
double reduction_ratio = 42;

// 编码器线数，电机转动一圈，编码器的一个霍尔传感器的高低电平变化次数
int ppr_num = 11;

// 输出轴转动一圈，编码器的一个霍尔传感器的高低电平变化次数
double shaft_ppr = reduction_ratio * ppr_num;


// IRAM_ATTR是一个用于修饰函数和变量的宏定义。
// 表示将函数或变量放置在IRAM（Instruction RAM）中，这是ESP32芯片的指令内存。
// 使用IRAM_ATTR修饰的函数和变量将被编译器放置在IRAM中，
// 这可以提高这些函数和变量的执行速度，特别是对于需要频繁执行的代码段。
// 这在一些实时性要求高的应用中非常有用

// 中断函数的回调函数，参考后面的attachInterrupt()函数，当某个霍尔编码器的电平
// 由低到高变化时，会调用这个函数。
// 在这个函数中，会判断这一路的另一个霍尔元件的高低电平来判断旋转方向。
void IRAM_ATTR B_wheel_pulse() {
  if(digitalRead(BENCA)){
    B_wheel_pulse_count++;
  }
  else{
    B_wheel_pulse_count--;
  }
}


void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){
    A_wheel_pulse_count++;
  }
  else{
    A_wheel_pulse_count--;
  }
}


void setup(){
  // 设置编码器相关引脚的工作模式
  // 在使用编码器时，常常需要在其引脚上启用上拉电阻。这是因为编码器通常使用开漏输出或输出为无源（open collector）的方式。

  // 当编码器的引脚配置为输入模式时，如果没有外部上拉电阻或内部上拉电阻（使用INPUT_PULLUP选项）连接到引脚上，
  // 引脚的电平可能会漂移或处于不确定的状态。这可能导致读取编码器信号时出现错误或不稳定。

  // 通过使用INPUT_PULLUP选项，你可以在引脚上启用内部上拉电阻，将引脚的默认电平设置为高电平（逻辑1）。
  // 这样做可以有效地防止引脚电平漂移，并确保在没有外部信号时，引脚保持在已定义的状态。当编码器产生信号时，
  // 引脚的电平将改变，可以通过中断或轮询的方式检测到状态变化。

  // 因此，在配置编码器引脚时，使用INPUT_PULLUP选项是一种常见的做法，可以提高编码器信号的可靠性和稳定性。
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);

  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);
 
  // 设置中断和对应的回调函数，当BEBCB由低电平变高电平时（RISING），调用 B_wheel_pulse 函数。
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  // 设置中断和对应的回调函数，当AEBCB由低电平变高电平时（RISING），调用 A_wheel_pulse 函数。
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  // 初始化串口，波特率115200
  Serial.begin(115200);
  // 等待串口初始化完成
  while(!Serial){}
}


void loop(){
  // 计算B通道电机输出轴的转速，单位为 转/分钟
  rpm_B = (float)((B_wheel_pulse_count / shaft_ppr) * 60 * (1000 / interval));
  B_wheel_pulse_count = 0;

  // 计算A通道电机输出轴的转速，单位为 转/分钟
  rpm_A = (float)((A_wheel_pulse_count / shaft_ppr) * 60 * (1000 / interval));
  A_wheel_pulse_count = 0;

  Serial.print("RPM_A: ");Serial.print(rpm_A);Serial.print("   RPM_B: ");Serial.println(rpm_B);
  Serial.println("--- --- ---");

  delay(interval);
}