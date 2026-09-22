#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "RC.h"
#include "imu.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			if (num1 < CHASSIS_MOTOR_NUM) chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			if (num2 < PANTILE_MOTOR_NUM) pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			if (num3 < SHOOTER_MOTOR_NUM) shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			if (num4 < SUPPLY_MOTOR_NUM)
			{
				motor[i]->spinning = false;
				motor[i]->need_curcircle = false;
				supply_motor[num4++] = motor[i];
			}
			break;
		default:
			break;
		}
	}
	if (pantile_motor[PANTILE::TYPE::PITCH]) pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	if (pantile_motor[PANTILE::TYPE::YAW]) pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	if (ctrl.pantile_motor[PANTILE::TYPE::PITCH]){
		const int32_t input = Setrange(ch_pitch, 660);
		if (input > 0) {
			if(imu_pantile.GetAnglePitch() < para.imu_pitch_max)
			ctrl.pantile_motor[PANTILE::TYPE::PITCH]->setangle += para.pitch_speed;
		}
		else if (input < 0) {
			if (imu_pantile.GetAnglePitch() > para.imu_pitch_min)
			ctrl.pantile_motor[PANTILE::TYPE::PITCH]->setangle -= para.pitch_speed;
		}
	}

	if (ctrl.pantile_motor[PANTILE::TYPE::YAW]) {
		Motor* const yaw_motor = ctrl.pantile_motor[PANTILE::TYPE::YAW];
		const int32_t input = Setrange(static_cast<int16_t>(ch_yaw), 660);

		// Yaw uses cascaded position-speed control. The stick changes the angle target.
		yaw_motor->setangle += static_cast<float>(input) * para.yaw_speed / 660.f;
	}





}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	
}

void CONTROL::CHASSIS::Keep_Direction()
{


}

void CONTROL::CHASSIS::Update()
{
	// Wheel order: 左前(5), 右前(6), 右后(7), 左后(8).
	// Positive wheel speed is assumed to move the car forward.
	if (!ctrl.chassis_motor[0] || !ctrl.chassis_motor[1] ||
		!ctrl.chassis_motor[2] || !ctrl.chassis_motor[3]) return;//检测四个底盘电机是否全存在
	const int32_t wheel[CHASSIS_MOTOR_NUM] = {
		speedx + speedy + speedz, -speedx + speedy + speedz,
		-speedx - speedy + speedz, speedx - speedy + speedz
	};//计算四个麦轮速度,麦轮解算必然导致四个轮子转速不同
	int32_t peak = 0;
	for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
		peak = std::max(peak, std::abs(wheel[i]));//peak为最快轮子的速度
	const int32_t limit = std::min<int32_t>(para.max_speed, ctrl.chassis_motor[0]->maxspeed);//取小保证两个都不上限
	for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
		ctrl.chassis_motor[i]->setspeed = peak > limit && limit > 0//速度在超限的时候转化为上限,在未超过时就是正常传输目标速度
			? wheel[i] * limit / peak : wheel[i];
}

void CONTROL::PANTILE::Update()
{
	
}

void CONTROL::SHOOTER::Update()
{
	
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

