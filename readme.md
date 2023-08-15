# PID�㷨
## 1��PID�㷨�Ļ���ԭ��
PID�㷨��һ�����ڿ���ϵͳ��ͨ���㷨����ͨ���Ա��ض���Ŀ������������֮���ƫ����бȽϣ�  
���ݱȽϽ������������������������Ӷ�ʵ�ֶԱ��ض���Ŀ��ơ�
## 2������ṹ��
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
## 3��PID�㷨ʵ��
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