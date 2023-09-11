#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

#define RC_MAX 660
#define RC_MIN -660
#define motor_max 900
#define motor_min -900

pid_struct_t motor_pid_chassis[4];
pid_struct_t supercap_pid;
motor_info_t motor_info_chassis[8];        // 电机信息结构体
fp32 chassis_motor_pid[3] = {30, 0.5, 10}; // 用的原来的pid
fp32 superpid[3] = {120, 0.1, 0};
volatile int16_t Vx = 0, Vy = 0, Wz = 0;
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
volatile int16_t motor_speed_target[4];
extern RC_ctrl_t rc_ctrl;
extern ins_data_t ins_data;
extern float powerdata[4];
extern uint16_t shift_flag;

uint8_t rc[18];
uint8_t motor_flag[4] = {0, 0, 0, 0}; // LF RF RB LB
int16_t avg_speed = 0;
// Save imu data

int8_t chassis_mode = 1; // 判断底盘状态，用于UI编写

// 获取imu——Yaw角度差值参数
static void Get_Err();

// 参数重置
static void Chassis_loop_Init();

// super_cap
void power_limit(int *speed);

int chassis_mode_flag = 0;

void qe();

#define angle_valve 5
#define angle_weight 55

void Chassis_task(void const *pvParameters)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&motor_pid_chassis[i], chassis_motor_pid, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
  }
  pid_init(&supercap_pid, superpid, 3000, 3000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

  for (;;) // 底盘运动任务
  {
    // 遥控器控制
    // chanel 0 left max==-660,right max==660
    // chanel 1 up max==660,down max==-660
    // chanel 2 left max==-660,right max==660
    // chanel 3 up max==660,down max==-660
    // chanel 4 The remote control does not have this channel

    if (rc_ctrl.rc.s[0] == 1)
    {
      LEDB_ON(); // BLUE LED
      LEDR_OFF();
      LEDG_OFF();
    }
    else if (rc_ctrl.rc.s[0] == 2)
    {
      LEDG_ON(); // GREEN LED
      LEDR_OFF();
      LEDB_OFF();
      RC_to_motor();
    }
    else if (rc_ctrl.rc.s[0] == 3)
    {
      LEDR_ON(); // RED LED
      LEDB_OFF();
      LEDG_OFF();
      RC_Move();
    }
    else
    {
      LEDR_OFF();
      LEDB_OFF();
      LEDG_OFF();
    }
    osDelay(1);
  }
}

static void Chassis_loop_Init()
{
  Vx = 0;
  Vy = 0;
  Wz = 0;
}

// 运动解算
void chassis_motol_speed_calculate()
{

  motor_speed_target[CHAS_LF] = 0;
  motor_speed_target[CHAS_RF] = 0;
  motor_speed_target[CHAS_RB] = 0;
  motor_speed_target[CHAS_LB] = 0;
}
// 运动解算
// 速度限制函数
void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed)
{
  uint8_t i = 0;
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;
  for (i = 0; i < 4; i++)
  {
    temp = (motor_speed[i] > 0) ? (motor_speed[i]) : (-motor_speed[i]); // 求绝对值

    if (temp > max)
    {
      max = temp;
    }
  }

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    for (i = 0; i < 4; i++)
    {
      motor_speed[i] *= rate;
    }
  }
}
// 电机电流控制
void chassis_current_give()
{

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed, motor_speed_target[i]);
  }
  set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
}

// 线性映射函数
static int16_t map_range(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值映射到[0, 1]的范围
  double normalized_value = (value * 1.0 - from_min * 1.0) / (from_max * 1.0 - from_min * 1.0);

  // 然后将[0, 1]的范围映射到[to_min, to_max]的范围
  int16_t mapped_value = (int16_t)(normalized_value * (to_max - to_min) + to_min);

  return mapped_value;
}

void RC_to_motor(void)
{
  // 电机速度与遥控器通道的对应关系
  avg_speed = map_range(rc_ctrl.rc.ch[3], RC_MIN, RC_MAX, motor_min, motor_max);
  motor_speed_target[CHAS_LF] = avg_speed;
  motor_speed_target[CHAS_RF] = -avg_speed;
  motor_speed_target[CHAS_RB] = -avg_speed;
  motor_speed_target[CHAS_LB] = avg_speed;

  // 判断需要旋转的电机
  for (uint8_t i = 0; i < 4; i++)
  {
    /* code */
    if (motor_flag[i] == 0)
    {
      motor_speed_target[i] = 0;
    }
  }
  // 电机电流控制
  chassis_current_give();
}


void RC_Move(void)
{
  // 从遥控器获取控制输入
  // int16_t forward_backward_input = rc_ctrl.rc.ch[1]; // 前后输入
  // int16_t left_right_input = rc_ctrl.rc.ch[0];       // 左右输入
  // int16_t rotation_input = rc_ctrl.rc.ch[2];         // 旋转输入
  Vx = rc_ctrl.rc.ch[3]; // 前后输入
  Vy = rc_ctrl.rc.ch[2];       // 左右输入
  Wz = rc_ctrl.rc.ch[0];         // 旋转输入

  /*************记得加上线性映射***************/
  Vx = map_range(Vx, RC_MIN, RC_MAX, motor_min, motor_max);
  Vy = map_range(Vy, RC_MIN, RC_MAX, motor_min, motor_max);
  Wz = map_range(Wz, RC_MIN, RC_MAX, motor_min, motor_max);

  // 根据分解的速度调整电机速度目标
  // motor_speed_target[CHAS_LF] = Vx - Vy - Wz;
  // motor_speed_target[CHAS_RF] = Vx + Vy + Wz;
  // motor_speed_target[CHAS_RB] = Vx - Vy + Wz;
  // motor_speed_target[CHAS_LB] = Vx + Vy - Wz;
  motor_speed_target[CHAS_LF] = Wz + Vx + Vy;
  motor_speed_target[CHAS_RF] = Wz - Vx + Vy;
  motor_speed_target[CHAS_RB] = Wz - Vx - Vy;
  motor_speed_target[CHAS_LB] = Wz + Vx - Vy;

  // 电机电流控制
  chassis_current_give();
}