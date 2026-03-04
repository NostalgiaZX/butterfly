/**
 * @file     : YDIFly.ino
 * @brief    : YDIFLY蝴蝶扑翼机开源代码，其中功能包括实现蝴蝶的基本遥控飞行，翅膀竖立功能等等，且可通过修改宏定义轻松修改代码参数，无须看懂代码
 * @author   : 一点创绘
 * @date     : 2025-11-1
 * @version  : v1.7.A  适用以LM-D02灵眸舵机为代表的 脉冲宽度825-2175 对应角度180度
 * 
 * @license  : GPL 3.0 License
 * @changelog:
 * - v1.7 (2026-1-14): 升级拐弯、俯仰的控制逻辑。SWD摇杆上拨时，翅膀的第一次扑翼方向反向。
 * - v1.6 (2025-10-16): 提高遥控操作手感，方便对扑翼机的操控。
 * - v1.5 (2025-9-16): 加入遥控连接检测功能，调整摇杆逻辑使其更符合使用习惯。
 * - v1.4 (2025-9-6): 加入舵机方向可调的功能，可通过宏定义轻松修改舵机方向。
 * - v1.3 (2025-9-3): 修改一些代码BUG，并加入遥控显示电池电压功能。
 * - v1.2 (2025-9-2): 初始版本，具有基本的扑翼飞行、翅膀并拢竖立等功能，可用ELRS遥控进行远程控制。
 */
#include <AlfredoCRSF.h>
#include <Servo.h>

/******************** 基本参数 ******************* */
#define YDIFLY_SERVO_L_PIN                  9       // 引脚设置
#define YDIFLY_SERVO_R_PIN                  10      // 引脚设置
#define YDIFLY_LED_PIN                      16      // 引脚设置

#define YDIFLY_REMOTE_JOY_MID               1500    // 遥控摇杆的中间值

/******************** 舵机参数设置 ******************* */
#define YDIFLY_SERVO_ANGLE_L_INIT           90
#define YDIFLY_SERVO_ANGLE_R_INIT           90
#define YDIFLY_SERVO_ANGLE_L_MAX            180
#define YDIFLY_SERVO_ANGLE_L_MIN            0
#define YDIFLY_SERVO_ANGLE_R_MAX            180
#define YDIFLY_SERVO_ANGLE_R_MIN            0

/******************** 舵机方向设置 ******************* */
#define YDIFLY_SERVO_L_DIR                  0       // 左舵机摆动方向，0表示正向，1表示反向
#define YDIFLY_SERVO_R_DIR                  1       // 右舵机摆动方向，0表示正向，1表示反向

/******************** 遥控控制系数设置 ******************* */
#define YDIFLY_YAW_REMOTE_ANGLE_MAX         20.0f   // 正负20°
#define YDIFLY_FACTOR_FREQ                  0.05f
#define YDIFLY_FACTOR_AMP                   0.05f
#define YDIFLY_FACTOR_OFFSET                0.02f

#define YDIFLY_FACTOR_FILTER                0.2f

/******************** 翅膀扑翼周期设置 ******************* */
#define YDIFLY_CYCLE_MIN                    235
#define YDIFLY_CYCLE_MAX                    500

/******************** 翅膀扑翼幅度设置 ******************* */
// 通过遥控SWB拨杆进行调整
#define YDIFLY_AMP0                         30      // 扑翼幅度为 ±60°
#define YDIFLY_AMP1                         40      // 扑翼幅度为 ±70°
#define YDIFLY_AMP2                         50      // 扑翼幅度为 ±75°

/******************** 翅膀上拍下拍速度差 ******************* */
#define YDIFLY_SPEED_DIFF                   0       // 速度差需要在 -YDIFLY_CONTROL_CYCLE~YDIFLY_CONTROL_CYCLE 之间

/******************** 任务控制周期参数 ******************* */
#define YDIFLY_CONTROL_CYCLE                25     // 舵机的控制周期，ms
#define YDIFLY_LED_CYCLE                    200    // LED闪缩周期，ms


typedef enum
{
    SERVO_L,    // 左翅膀舵机
    SERVO_R,    // 右翅膀舵机
}ydifly_servo_name_e;

typedef struct
{
    float amp;                      // 扑翼幅值
    float freq;                     // 扑翼频率
    float offset;                   // 舵机中间值偏移
    float yaw;                      // 偏航角度控制
    float pitch;                    // 俯仰角度控制
    uint8_t swa;                    // SWA 信号，0和2
    uint8_t swb;                    // SWB 信号，0、1、2
    uint8_t swc;                    // SWC 信号，0、1、2
    uint8_t swd;                    // SWD 信号，0、1、2
}ydifly_remote_cmd_t;

typedef struct
{
    uint32_t sys_time_ms;           // 系统时间，单位ms
    uint32_t sys_time_ms_last;      // 以往时间，单位ms
    uint32_t led_time_ms_last;      // 以往时间，单位ms
    ydifly_remote_cmd_t remote;     // 遥控相关参数
    ydifly_remote_cmd_t remote_last;// 上一次遥控参数
}ydifly_control_t;

ydifly_control_t ydifly;
AlfredoCRSF crsf;
Servo servo_l, servo_r;
float time_now = 0;
float time_init = 0;

static void ReceiverData( ydifly_remote_cmd_t* remote );
static void YDIFlyServoSinControl( float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff );
static void YDIServoAngleControl( ydifly_servo_name_e name, float angle );
static void YDISendBattery(float voltage, float current, float capacity, float remaining);

void setup() {
  // put your setup code here, to run once:   
  pinMode(YDIFLY_LED_PIN,OUTPUT);     // 初始化LED灯控制引脚
  digitalWrite(YDIFLY_LED_PIN, HIGH); // 打开LED灯

  Serial.begin(CRSF_BAUDRATE, SERIAL_8N1, SERIAL_FULL); // 初始化串口
  crsf.begin(Serial);                                   // 初始化接收机

  servo_l.attach(YDIFLY_SERVO_L_PIN, 500, 2500); // 初始化 L 舵机引脚
  servo_r.attach(YDIFLY_SERVO_R_PIN, 500, 2500); // 初始化 R 舵机引脚
  YDIServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_INIT );
  YDIServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_INIT );

  while( crsf.getChannel(3) < 800 )   // 检测是否连接到遥控
  {
    static uint8_t led_sta = 0;

    crsf.update();

    /* LED灯闪烁 */
    if( led_sta == 0 )
    {
      digitalWrite(YDIFLY_LED_PIN, LOW); // 关闭LED灯
      led_sta = 1;
    }
    else
    {
      digitalWrite(YDIFLY_LED_PIN, HIGH); // 打开LED灯
      led_sta = 0;
    }
    delay(YDIFLY_LED_CYCLE);
  }

  digitalWrite(YDIFLY_LED_PIN, HIGH); // 连接上遥控，LED常亮
  ydifly.sys_time_ms = millis();      // 更新系统时间
  ydifly.sys_time_ms_last = millis(); // 初始化以往时间
}

void loop() {
  // put your main code here, to run repeatedly:
  float angle_l_add, angle_l_mid, angle_r_add, angle_r_mid, control_T;

  ydifly.sys_time_ms = millis();      // 更新系统时间
  
  if( ydifly.sys_time_ms - ydifly.sys_time_ms_last >= YDIFLY_CONTROL_CYCLE )
  {
    float temp = 0;

    float batt_volt = (float)analogRead(A0)*11/1070;

    ydifly.sys_time_ms_last += YDIFLY_CONTROL_CYCLE;      // 时间补全

    crsf.update();                      // 更新接收机数据
    ReceiverData(&ydifly.remote);       // 遥控数据进行解析，获取最新数据

    YDISendBattery(batt_volt, 1.0, 30, 50);

    /* 翅膀扑翼幅度解算 */
    if     ( ydifly.remote.swb == 0 )        ydifly.remote.amp = YDIFLY_AMP0;   // 不同档位幅值不同
    else if( ydifly.remote.swb == 1 )        ydifly.remote.amp = YDIFLY_AMP1;   // 不同档位幅值不同
    else                                     ydifly.remote.amp = YDIFLY_AMP2;   // 不同档位幅值不同

    /* 通过SWD拨杆，设置翅膀第一次是向上扑还是向下扑 */
    if     ( ydifly.remote.swd == 2 )        time_init = 3.141592653f;  // SWD上拨，扑翼初始方向反向
    else                                     time_init = 0;             // SWD下拨和中档，扑翼初始方向正向

    /* 一阶滤波 */
    ydifly.remote.yaw   = YDIFLY_FACTOR_FILTER*ydifly.remote.yaw    + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.yaw;
    ydifly.remote.pitch = YDIFLY_FACTOR_FILTER*ydifly.remote.pitch  + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.pitch;
    ydifly.remote.freq  = YDIFLY_FACTOR_FILTER*ydifly.remote.freq   + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.freq;
    ydifly.remote.amp   = YDIFLY_FACTOR_FILTER*ydifly.remote.amp    + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.amp;
    ydifly.remote.offset= YDIFLY_FACTOR_FILTER*ydifly.remote.offset + (1-YDIFLY_FACTOR_FILTER)*ydifly.remote_last.offset;

    ydifly.remote_last.yaw   = ydifly.remote.yaw;
    ydifly.remote_last.pitch = ydifly.remote.pitch;
    ydifly.remote_last.freq  = ydifly.remote.freq;
    ydifly.remote_last.amp   = ydifly.remote.amp;
    ydifly.remote_last.offset= ydifly.remote.offset;

    /* 舵机角度控制 */
    if( ydifly.remote.pitch > 0 )
    {
        angle_l_mid = (YDIFLY_SERVO_ANGLE_L_INIT-YDIFLY_SERVO_ANGLE_L_MIN-ydifly.remote.amp)*ydifly.remote.pitch/1500;
        angle_r_mid = (YDIFLY_SERVO_ANGLE_R_INIT-YDIFLY_SERVO_ANGLE_R_MIN-ydifly.remote.amp)*ydifly.remote.pitch/1500;
        temp = (angle_l_mid>angle_r_mid) ? angle_r_mid : angle_l_mid;
    }
    else
    {
        angle_l_mid = (YDIFLY_SERVO_ANGLE_L_MAX-YDIFLY_SERVO_ANGLE_L_INIT-ydifly.remote.amp)*ydifly.remote.pitch/1500;
        angle_r_mid = (YDIFLY_SERVO_ANGLE_R_MAX-YDIFLY_SERVO_ANGLE_R_INIT-ydifly.remote.amp)*ydifly.remote.pitch/1500;
        temp = (angle_l_mid>angle_r_mid) ? angle_l_mid : angle_r_mid;
    }

    /* 舵机扑翼中位值计算 */
    angle_l_mid = YDIFLY_SERVO_ANGLE_L_INIT - temp + ydifly.remote.offset*YDIFLY_FACTOR_OFFSET;
    angle_r_mid = YDIFLY_SERVO_ANGLE_R_INIT - temp - ydifly.remote.offset*YDIFLY_FACTOR_OFFSET;
    
    if( ydifly.remote.freq > 10 )       // 如果有给油门，则进入起飞程序
    {
      float angle_rate = 1, angle_rate_min = 1;

      temp = (ydifly.remote.yaw>0) ? (ydifly.remote.yaw*YDIFLY_YAW_REMOTE_ANGLE_MAX/1500) : (ydifly.remote.yaw*YDIFLY_YAW_REMOTE_ANGLE_MAX/1500);
      angle_l_add = - temp + ydifly.remote.amp;
      angle_r_add = + temp + ydifly.remote.amp;

      /* 限幅 */
      angle_l_add = (angle_l_add>ydifly.remote.amp) ? ydifly.remote.amp : angle_l_add;
      angle_r_add = (angle_r_add>ydifly.remote.amp) ? ydifly.remote.amp : angle_r_add;

      /* 限幅 */
      if( (angle_l_mid+angle_l_add) > YDIFLY_SERVO_ANGLE_L_MAX )                
      {
          angle_rate = (YDIFLY_SERVO_ANGLE_L_MAX-angle_l_mid)/angle_l_add;
          angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
      }
      if( (angle_l_mid-angle_l_add) < YDIFLY_SERVO_ANGLE_L_MIN )
      {
          angle_rate = (angle_l_mid-YDIFLY_SERVO_ANGLE_L_MIN)/angle_l_add;
          angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
      }
      if( (angle_r_mid+angle_r_add) > YDIFLY_SERVO_ANGLE_R_MAX )
      {
          angle_rate = (YDIFLY_SERVO_ANGLE_R_MAX-angle_r_mid)/angle_r_add;
          angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
      }
      if( (angle_r_mid-angle_r_add) < YDIFLY_SERVO_ANGLE_R_MIN )
      {
          angle_rate = (angle_r_mid-YDIFLY_SERVO_ANGLE_R_MIN)/angle_r_add;
          angle_rate_min = ( angle_rate_min > angle_rate ) ? angle_rate : angle_rate_min;
      }
      angle_l_add *= angle_rate_min;
      angle_r_add *= angle_rate_min;

      /* 舵机控制正弦周期 */
      control_T = YDIFLY_CYCLE_MAX + ydifly.remote.freq*(YDIFLY_CYCLE_MIN - YDIFLY_CYCLE_MAX)/1500;

      /* 舵机角度控制 */
      YDIFlyServoSinControl( (angle_l_mid+angle_l_add), (angle_l_mid-angle_l_add), (angle_r_mid+angle_r_add), (angle_r_mid-angle_r_add), control_T, YDIFLY_SPEED_DIFF );
    }
    else if( ydifly.remote.swc == 2 )     // 如果拨动开关，翅膀摆动。拨杆到上档位
    {
      YDIServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_MIN );
      YDIServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_MIN );
      time_now = 0;
    }
    else if( ydifly.remote.swc == 1 )     // 舵机处于初始位置。拨杆到中档位
    {
      YDIServoAngleControl( SERVO_L, angle_l_mid );
      YDIServoAngleControl( SERVO_R, angle_r_mid );
      time_now = 0;
    }
    else    // 如果拨动开关，翅膀摆动。拨杆到下档位
    {
      YDIServoAngleControl( SERVO_L, YDIFLY_SERVO_ANGLE_L_MAX );
      YDIServoAngleControl( SERVO_R, YDIFLY_SERVO_ANGLE_R_MAX );
      time_now = 0;
    }
  }
  delay(1);
}

static void ReceiverData( ydifly_remote_cmd_t* remote )
{
  remote->freq  = constrain( crsf.getChannel(3), 1000, 2000 ) - 1000;
  remote->yaw   = (float)crsf.getChannel(4) - YDIFLY_REMOTE_JOY_MID;
  remote->pitch = (float)crsf.getChannel(2) - YDIFLY_REMOTE_JOY_MID;
  remote->offset= (float)crsf.getChannel(1) - YDIFLY_REMOTE_JOY_MID;

  remote->freq  *= 1.5f;
  remote->yaw   *= 1.5f;
  remote->pitch *= 1.5f;
  remote->offset*= 1.5f;

  if      ( crsf.getChannel(5) < 1100 )       remote->swa = 0;
  else if ( crsf.getChannel(5) > 1900 )       remote->swa = 2;
  if      ( crsf.getChannel(6) < 1100 )       remote->swb = 0;
  else if ( crsf.getChannel(6) > 1900 )       remote->swb = 2;
  else if ( crsf.getChannel(6) > 1300 )       remote->swb = 1;
  if      ( crsf.getChannel(7) < 1100 )       remote->swc = 0;
  else if ( crsf.getChannel(7) > 1900 )       remote->swc = 2;
  else if ( crsf.getChannel(7) > 1300 )       remote->swc = 1;
  if      ( crsf.getChannel(8) < 1100 )       remote->swd = 0;
  else if ( crsf.getChannel(8) > 1900 )       remote->swd = 2;
  else if ( crsf.getChannel(8) > 1300 )       remote->swd = 1;
}

static void YDIFlyServoSinControl( float l_angle_max, float l_angle_min, float r_angle_max, float r_angle_min, float T, float speed_diff )
{
    float angle_set = 0;

    /* 输入参数保护 */
    if( speed_diff > YDIFLY_CONTROL_CYCLE )         speed_diff = YDIFLY_CONTROL_CYCLE-1;
    else if( speed_diff < -YDIFLY_CONTROL_CYCLE )   speed_diff =-YDIFLY_CONTROL_CYCLE+1;

    angle_set = ((l_angle_max-l_angle_min)/2)*sin( time_now*6.283185307179586/T + time_init ) + (l_angle_max+l_angle_min)/2;
    YDIServoAngleControl( SERVO_L, angle_set );
  
    angle_set = ((r_angle_max-r_angle_min)/2)*sin( time_now*6.283185307179586/T + time_init ) + (r_angle_max+r_angle_min)/2;
    YDIServoAngleControl( SERVO_R, angle_set );

    time_now += YDIFLY_CONTROL_CYCLE;
    if( (time_now > T*0.25f) && (time_now < T*0.75f) )  time_now -= speed_diff;
    else                                                time_now += speed_diff;

    if( time_now > T )
    {
      time_now -= T;
    }    
}

static void YDISendBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   // 电压
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   // 电流
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   // 容量
  crsfBatt.remaining = (uint8_t)(remaining);                // SOC
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

static void YDIServoAngleControl( ydifly_servo_name_e name, float angle )
{
  if( angle < 0 )       angle = 0;
  else if( angle > 180) angle = 180;

  if( name == SERVO_L )
  {
    #if YDIFLY_SERVO_L_DIR
    servo_l.write( angle );
    #else
    servo_l.write( 180 - angle );
    #endif
  }
  else if( name == SERVO_R )
  {
    #if YDIFLY_SERVO_R_DIR
    servo_r.write( angle );
    #else
    servo_r.write( 180 - angle );
    #endif
  }
  else
  {
    return ;
  }
}
