#include <Arduino.h>
//电机引脚定义
#define leftA_PIN1 11
#define leftB_PIN2 10
#define righA_PIN3 3
#define righB_PIN4 9
//红外感应引脚定义
#define leftA_track_PIN1 2
#define leftB_track_PIN2 4
#define middle_track_PIN3 7
#define righA_track_PIN4 8
#define righB_track_PIN5 12
//红外感应引脚初始化
void track_pinint()
{
  pinMode(leftA_track_PIN1, INPUT);  // 设置引脚为输入引脚
  pinMode(leftB_track_PIN2, INPUT);  // 设置引脚为输入引脚
  pinMode(middle_track_PIN3, INPUT); // 设置引脚为输入引脚
  pinMode(righA_track_PIN4, INPUT);  // 设置引脚为输入引脚
  pinMode(righB_track_PIN5, INPUT);  // 设置引脚为输入引脚
}
//电机引脚初始化
void motor_pinint()
{
  pinMode(leftA_PIN1, OUTPUT); // 设置引脚为输出引脚
  pinMode(leftB_PIN2, OUTPUT); // 设置引脚为输出引脚
  pinMode(righA_PIN3, OUTPUT); // 设置引脚为输出引脚
  pinMode(righB_PIN4, OUTPUT); // 设置引脚为输出引脚
}
//PID控制
float Kp =20, Ki = 0.00015, Kd =6.5;    //2.5                //pid弯道参数参数 
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//pid直道参数 
float decide = 0,Y=0;                                   //元素判断
float previous_error = 0, previous_I = 0;           //误差值 


void setup()
{
  Serial.begin(115200); // 串口波特率115200（PC端使用）
  track_pinint();       // 循迹引脚初始化
  motor_pinint();       // 电机引脚初始化
}

void Motor_Speed(int Left1_Speed, int Left2_Speed, int Right1_Speed, int Right2_Speed)//速度模块
{
  analogWrite(leftA_PIN1, Left1_Speed* 2.55 - Y); //控制电机的速度，不同的车模，代码执行效果不同！
  analogWrite(leftB_PIN2, Left2_Speed* 2.55 - Y);
  analogWrite(righA_PIN3, Right1_Speed* 2.55 + Y);
  analogWrite(righB_PIN4, Right2_Speed* 2.55 + Y);
}
//红外感应数据读取
uint8_t user_scan()
{
  uint8_t rev=0;
  rev |= digitalRead(leftA_track_PIN1);
  rev = rev<<1;
  rev |= digitalRead(leftB_track_PIN2);
 rev = rev<<1;
  rev |= digitalRead(middle_track_PIN3);
 rev = rev<<1;
  rev |= digitalRead(righA_track_PIN4);
rev = rev<<1;
  rev |= digitalRead(righB_track_PIN5);
  return rev;
}

int calc_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;
 
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
 
  previous_error = error;

  return PID_value;
}

void loop()                                  //遇黑线灯亮0，1时灯灭
{
  int error=0;
  uint8_t def =0;
  def = user_scan();
  switch (def)
  {
     case 0:                                  //直行：00000
      error=1;
      Y=calc_pid();
      Motor_Speed(45,0,45,0);
      break; 

     case 17:                                   //直行：10001
      error=1;
      Y=calc_pid();
      Motor_Speed(40,0,40,0);
      break; 
      
    case 27:                                  //直行：11011
      error=0;
      Y=calc_pid();
      Motor_Speed(40,0,40,0);
      break; 

    case 23:                                 //10111，正常左转1
       error=-1;
        Y=calc_pid();
        Motor_Speed(40,0,50,0);
       break;

    case 15:                               //01111，正常左转2
        error=-2;
        Y=calc_pid();
       Motor_Speed(0,30,50,0); 
       break;
       

    case 29:                                //11101，正常右转1
        error=1;
        Y=calc_pid();
      Motor_Speed(35,0,0,35);
      break; 

    case 30:                                //11110，正常右转2
        error=2;
       Y=calc_pid();
      Motor_Speed(40,0,0,35);
      break;
      
    case 7:                                 //00111，左急转弯
        error=-3;
        Y=calc_pid();
      Motor_Speed(0,40,40,0);                //70，70
      break; 
      
     case 28:                                 //11100，右急转弯
       error=3;
       Y=calc_pid();
      Motor_Speed(50,0,0,40);
      break; 
      
      case 24:                                 //11000，右急转弯
       error=2;
       Y=calc_pid();
      Motor_Speed(30,0,0,30);
      break;

      case 25:                                 //11001，右急转弯
       error=2;
       Y=calc_pid();
      Motor_Speed(40,0,0,30);
      break;

      case 19:                                 //10011，左急转弯
       error=3;
       Y=calc_pid();
      Motor_Speed(0,35,35,0);
      break;


      case 11:                                 //01011，急转弯
       error=1;
       Y=calc_pid();
      Motor_Speed(35,0,35,0);
      break;
       
      default:
        error=2;
        Y=calc_pid();
       Motor_Speed(20,0,35,0);
       
  }
}