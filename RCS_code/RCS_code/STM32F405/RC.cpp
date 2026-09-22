#include "label.h"
#include "RC.h"
#include "control.h"

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	// DR16 DBUS: 100 kbit/s, 8 data bits + even parity, 1 stop bit.
	huart->Init(Instance, BaudRate, UART_WORDLENGTH_9B, UART_PARITY_EVEN)
		.DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	RC_CheckState();
	RC_Control();

	if (Shift_mode())
	{

	}

}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}

void RC::RC_CheckState() {

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::SEPARATE;//进入底盘云台分离
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::ROTATION;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::SPINNING;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::FOLLOW;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(DOWN, UP):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(DOWN, MID):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(DOWN, DOWN):
		ctrl.mode = CONTROL::STOP;
		break;

	default:
		ctrl.mode = CONTROL::STOP;
		break;
	}

}

void RC::RC_Control()//在此函数里面实现换算摇杆编码和底盘速度
{
	ctrl.chassis.speedx = 0;
	ctrl.chassis.speedy = 0;
	ctrl.chassis.speedz = 0;
	const bool online = received_frame &&//启动通信检测是因为last_tick和tick_now初始均为0
		(xTaskGetTickCount() - last_frame_tick <= pdMS_TO_TICKS(100));//拿计数器读到的现在的时刻和上一帧的读到的时刻相减,与100ms相比较
	// Left switch middle, right switch up: left stick translates; right ch[0] rotates.
	if (online && ctrl.mode == CONTROL::FOLLOW)
	{
		ctrl.chassis.speedx = CONTROL::Setrange(rc.ch[3], 660) * para.max_speed / 660;
		ctrl.chassis.speedy = CONTROL::Setrange(rc.ch[2], 660) * para.max_speed / 660;
		ctrl.chassis.speedz = CONTROL::Setrange(rc.ch[0], 660) * para.rota_speed / 660;
	}
	if (online && ctrl.mode == CONTROL::RESET)
	{
		;
	}
	if (online && ctrl.mode == CONTROL::SEPARATE)
	{
		ctrl.CONTROL::Control_Pantile(rc.ch[0], rc.ch[1]);
	}
}

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}
	if (pd_Rx != pdTRUE) return;

	if (m_uart->dataDmaNum != 18) { ++invalid_frame_count; return; }
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)
	{ ++invalid_frame_count; return; }

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 25 && rc.ch[0] >= -25)rc.ch[0] = 0;
	if (rc.ch[1] <= 25 && rc.ch[1] >= -25)rc.ch[1] = 0;
	if (rc.ch[2] <= 25 && rc.ch[2] >= -25)rc.ch[2] = 0;
	if (rc.ch[3] <= 25 && rc.ch[3] >= -25)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);
	if (rc.s[0] < UP || rc.s[0] > MID || rc.s[1] < UP || rc.s[1] > MID)
	{ ++invalid_frame_count; return; }
	++valid_frame_count;
	received_frame = true;//遥控器对频成功自动接受数据帧后置1
	last_frame_tick = xTaskGetTickCount();

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}