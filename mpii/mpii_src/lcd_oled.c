/*
 * lcd_oled.c
 *
 *  Created on: 15 ���. 2018 �.
 *      Author: ������
 */


#include "lcd_oled.h"
#include "stm32f1xx_hal_conf.h"
#include "time.h"
static void oled_cs_set()
{
	HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
}

static void oled_cs_reset()
{
	HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
}

static void oled_scl_set()
{
	HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_SET);
}

static void oled_scl_reset()
{
	HAL_GPIO_WritePin(OLED_SCK_GPIO_Port, OLED_SCK_Pin, GPIO_PIN_RESET);
}

static void oled_mosi_set()
{
	HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_SET);
}

static void oled_mosi_reset()
{
	HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_RESET);
}

static GPIO_PinState oled_miso_read()
{
	return HAL_GPIO_ReadPin(OLED_MISO_GPIO_Port, OLED_MISO_Pin);
}

static uint8_t oled_recv_data (){		//������ �� �������
	uint8_t retv;			//��� ����� ������

	uint16_t data=0x100;		//��� ����� ���� ���������� 10 ���
	//� ��� ������� RS RW � 8��� ������ ������
	//RS=0 RW=1 (������) 8��� ������ 0x00
	uint16_t bites = 0x200;		//��� ������� ��� �������� ����� �� �����
	oled_scl_set();				//SCL=1 ���������� � ��������
	oled_mosi_set();		//SDI=1
	oled_cs_reset();				//CS=0 ����� ���� (������ ��������)
	for (uint8_t i = 0; i < 18; i++){		//���� ������� �� 18��� ������ 10 ��� ������ ������� �� ������
		//��������� 8��� ��� ������
		oled_scl_reset();		//������� ����
		if (data & bites)
			oled_mosi_set();//���� �� ���� ���������� ��� SDI=1
		else
			oled_mosi_reset();		//���� ���� �������������� SDI=0

		if (bites)
			bites >>= 1;	//���� �� ����������� ��������� ������
		else
			bites = 0x200;	//���� ����������� �� ���������� �� 10���

		retv <<= 1;			//������� ���� ������������ ��� ���������� ������ ����
		if (oled_miso_read() == GPIO_PIN_SET)
			retv |= 1;	//����������� �������� ���
		oled_scl_set();		//����������� ����
		HAL_Delay(1);
	}
	oled_cs_set();			//CS=1 ���������� ����� ���� ������� � ������� ������� �� ������
	return retv;			//���������� �������� ����
}


static uint8_t oled_wait()
{
	uint8_t d;

	for (uint8_t i = 0; i < 255; i++){		//��� ����� ���� ������� �����. �������� 255 ��� ��������� ������
		d = oled_recv_data();		//������ ������
		if ((d & 0x80) == 0)
			break;//���� BF=0 ������ ���������� ����� ������� �� �����
	}
	return (d & 0x80);		//������� ������ ����� �� ����

}

static void oled_send_cmd (uint8_t dat){ //����� ������� � �������
	//��� ��������
	uint16_t data=dat;      //RS=0 RW=0 �� ����� ������ ������������ ���� ����������� ���������������

	//oled_wait();

	oled_scl_set();          //SCL=1 ���������� � ��������
	oled_mosi_reset();          //SDI=1
	oled_cs_reset();           //CS=0 ����� ���� (������ ��������)

	for (uint8_t i = 0; i < 10; i++){      //������� �� 10 (����� 10��� ����������)
		oled_scl_reset();      //SCL=1 ��� ������� ������ ��� ������� ������ ���� ���

		if (data & 0x200)
			oled_mosi_set();//���� �� ���� ���������� ��� SDI=1
		else
			oled_mosi_reset();      //���� ���� �������������� SDI=0
		//		HAL_Delay(1);

		data<<=1;     //�������� ����
		oled_scl_set();      //SCL=0 ��� �������.
		//		HAL_Delay(1);
	}
	oled_cs_set();           //CS=1 ���������� ����� ���� ������� � ������� ������� �� ������
	HAL_Delay(1);
}

static void oled_send_data (uint8_t ch)
{

	uint16_t data=0x200 | ch;	//��������� ��� ������� RS=1
	//oled_wait();
	oled_scl_set();			//SCL=1 ���������� � ��������
	oled_mosi_reset();			//SDI=1
	oled_cs_reset();			//CS=0 ����� ���� (������ ��������)
	for (int i = 0; i < 10; i++){		//������� �� 10 (����� 10��� ����������)
		oled_scl_reset();		//SCL=0 ��� �������.
		if (data & 0x200)
			oled_mosi_set();//���� �� ���� ���������� ��� SDI=1
		else
			oled_mosi_reset();		//���� ���� �������������� SDI=0

		data<<=1;		//�������� ����
		//		HAL_Delay(1);
		oled_scl_set();		//SCL=1 ��� ������� ������ ��� ������� ������ ���� ���
		//	HAL_Delay(1);



	}
	oled_cs_set();    			//CS=1 ���������� ����� ���� ������� � ������� ������� �� ������
	HAL_Delay(1);
}


void oled_lcd_init()
{

	oled_cs_set();
	oled_scl_set();
	oled_mosi_set();

	oled_send_cmd(0x01);
	oled_send_cmd(0x3A);
	oled_send_cmd(0x01);
	oled_send_cmd(0x0C);
	oled_send_cmd(0x06);
	oled_send_cmd(0x17);
	oled_send_cmd(0x14);


}

void oled_print_upper(const char * buff, uint8_t len)
{

	oled_send_cmd(0x80);
	if (len > 16)
		len = 16;

	for(uint8_t i = 0; i < len; i++)
	{

		oled_send_data(*buff++);
	}
	if (len < 16)
	{
		for (uint8_t i = 0; i < 16 - len; i++)
			oled_send_data(' ');
	}
}

void oled_print_lower_and_time(char * buff, uint8_t len)
{
	RTC_TimeTypeDef rtc_td;
	char buffer[5];
	HAL_RTC_GetTime(&hrtc, &rtc_td, RTC_FORMAT_BIN);

	oled_send_cmd(0xC0);

	if (len > 11)
		len = 11;
	for(uint8_t i = 0; i < len; i++)
	{
		oled_send_data(*buff++);
	}

	if (len < 11)
	{
		for (uint8_t i = 0; i < 11 - len; i++)
			oled_send_data(' ');
	}
	oled_send_cmd(0xC0 + 11);
	sprintf(&buffer,"%02u:%02u", rtc_td.Hours, rtc_td.Minutes);
	for(uint8_t i = 0; i < sizeof(buffer); i++)
	{
		oled_send_data(buffer[i]);
	}

}

void oled_print_lower(char * buff, uint8_t len)
{
	oled_send_cmd(0xC0);
	//oled_send_cmd(0x41);
	if (len > 16)
		len = 16;
	for(uint8_t i = 0; i < len; i++)
	{
		oled_send_data(*buff++);
	}

}

void oled_clr_upper()
{
	oled_send_cmd(0x80);
	for(uint8_t i = 0; i < 16; i++)
	{
		oled_send_data(' ');
	}
}

void oled_clr_lower()
{
	oled_send_cmd(0xC0);
	for(uint8_t i = 0; i < 11; i++)
	{
		oled_send_data(' ');
	}
}

void oled_clr_all()
{
	oled_send_cmd(0x01);
}
