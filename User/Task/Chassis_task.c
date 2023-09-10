#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
pid_struct_t motor_pid_chassis[4];
pid_struct_t supercap_pid;
motor_info_t  motor_info_chassis[8];       //电机信息结构体
 fp32 chassis_motor_pid [3]={30,0.5,10};   //用的原来的pid
 fp32 superpid[3] = {120,0.1,0};
volatile int16_t Vx=0,Vy=0,Wz=0;
int16_t Temp_Vx;
int16_t Temp_Vy;
int fllowflag = 0;
volatile int16_t motor_speed_target[4];
 extern RC_ctrl_t rc_ctrl;
 extern ins_data_t ins_data;
 extern float powerdata[4];
 extern uint16_t shift_flag;
// Save imu data

int8_t chassis_mode = 1;//判断底盘状态，用于UI编写

//获取imu——Yaw角度差值参数
static void Get_Err(); 

//参数重置
static void Chassis_loop_Init(); 

//super_cap
void power_limit(int *speed);

int chassis_mode_flag =0;

void qe();
	
#define angle_valve 5
#define angle_weight 55
 
   void Chassis_task(void const *pvParameters)
{
 			       for (uint8_t i = 0; i < 4; i++)
			{
        pid_init(&motor_pid_chassis[i], chassis_motor_pid, 6000, 6000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
				
			} 
				pid_init(&supercap_pid, superpid, 3000, 3000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384			

  
    for(;;)				//底盘运动任务
    {     
 					
            osDelay(1);

    }



}





static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
}

//运动解算
void chassis_motol_speed_calculate()
{
	
	  motor_speed_target[CHAS_LF] =  0;
    motor_speed_target[CHAS_RF] =  0;
    motor_speed_target[CHAS_RB] =  0; 
    motor_speed_target[CHAS_LB] =  0;
}
//运动解算
//速度限制函数
  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}
//电机电流控制
void chassis_current_give() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {
        motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }
    	set_motor_current_can2(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
 

}






