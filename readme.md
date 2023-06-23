# PID算法
## 1、PID算法的基本原理
PID算法是一种用于控制系统的通用算法，它通过对被控对象的控制量与给定量之间的偏差进行比较，  
根据比较结果计算出控制量的修正量，从而实现对被控对象的控制。
## 2、定义结构体
```c
typedef struct
{
	float target_val;
	float actual_val;
	float err;
	float err_last;
	float err_sum;
	float Kp, Ki, Kd;
	float err_prev;
}tPid;
```
## 3、PID算法实现
```c
float PID_realize(tPid * pid, float actual_val)
{
	if (pid->target_val == 0)
	{
		return 0;
	}
	
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	
	
	pid->err_sum += pid->err;
	if (pid->err_sum > 1000) pid->err_sum = 1000;
	if (pid->err_sum < -1000) pid->err_sum = -1000;
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	
	//pid->actual_val += pid->Kp * (pid->err - pid->err_last) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_last + pid->err_prev);
	pid->err_prev = pid->err_last;
	pid->err_last = pid->err;

	return pid->actual_val;
}
```