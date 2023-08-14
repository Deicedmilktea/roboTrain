# PID
* 定义PID结构体
```C++
typedef struct
{
    float target_val;   //目标值
    float err;          //偏差值
    float err_last;     //上一个偏差值
    float Kp,Ki,Kd;     //比例、积分、微分系数
    float integral;     //积分值
    float output_val;   //输出值
}PID;
```
* PID实现
```C++
float PID_realize(float actual_val)
{
    //计算目标值与实际值的误差
    pid.err = pid.target_val - actual_val;
    
    //积分项
    pid.integral += pid.err;
​
    //PID算法实现
    pid.output_val = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err- pid.err_last);
​
    //误差传递
    pid.err_last = pid.err;
​
    //返回当前实际值
    return pid.output_val;
}
```