#pragma once
#include "usart.h"
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>

#define RC_STATE(s0, s1) ( ((s0) << 8) | (s1) )

/*
Switches: s[0] left, s[1] right; up=1, down=2, middle=3.
DBUS channels: ch[0] right stick horizontal, ch[1] right stick vertical,
ch[2] left stick horizontal, ch[3] left stick vertical.
*/

class RC
{
public:
	int gear;
	bool top_mode = true;
	bool fix = false;

	struct
	{
		int16_t ch[4];
		uint8_t s[2];
	}rc, pre_rc;

	enum POSITION { UP = 1, DOWN, MID };
	struct PC
	{
		int16_t x, y, z;
		uint8_t press_l, press_r;

		uint8_t key_h, key_l;
		const float spdratio = 1.f;
	}pc;

	uint8_t* GetDMARx(void) { return m_frame; }

	bool judement_start = false;
	uint32_t valid_frame_count{}; // Watch in debugger
	uint32_t invalid_frame_count{};
	void Decode();
	void OnRC();
	void OnPC();
	void Update();
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	bool Shift_mode();
	void RC_CheckState();
	void RC_Control();

private:
	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN]{};
	bool received_frame = false;
	TickType_t last_frame_tick = 0;
};

extern RC rc;
