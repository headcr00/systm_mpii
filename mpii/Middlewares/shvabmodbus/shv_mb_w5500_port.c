/**
 * @file shv_mb_w5500_port.c
 *
 * @date 12 февр. 2018 г.
 * @author Shvabenland
 * @brief
 */


#include "shv_modbus.h"
#include "spi_w5500.h"
#include "socket.h"
#include "w5500.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wizchip_conf.h"

#include "stdlib.h"

modbus_t * mb_pointer;
char * output_buffer;
uint8_t recv_buff[255] = {0};
void vEthTask(void *pvParameters);
TaskHandle_t ethTaskHandler;
/**
 * Sends data through Ethernet.
 * @param buff Input buffer
 * @param len Length of buffer
 * @return returns sent numbers or socket error
 */
int32_t eth_send_tcp_data(uint8_t * buff, uint16_t len)
{
	return send(0, buff, len);
}

void modbus_init_w5500(modbus_t * mb)
{
	mb_pointer = mb;
	wiz_NetInfo wnet = {\
			.mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
			.ip = {mb->ip_addr.ip1, mb->ip_addr.ip2, mb->ip_addr.ip3, mb->ip_addr.ip4},//todo ethernet setting in modbus registers
			.sn = {mb->netmask_addr.ip1, mb->netmask_addr.ip2, mb->netmask_addr.ip3, mb->netmask_addr.ip4},
			.gw = {mb->gw_ip_addr.ip1, mb->gw_ip_addr.ip2, mb->gw_ip_addr.ip3, mb->gw_ip_addr.ip4},
			.dns = {mb->dns_ip_addr.ip1, mb->dns_ip_addr.ip2, mb->dns_ip_addr.ip3, mb->dns_ip_addr.ip4},
			.dhcp = NETINFO_STATIC };

	w5500_init(wnet);
	xTaskCreate(vEthTask, "eth", configMINIMAL_STACK_SIZE * 5, NULL, 2, &ethTaskHandler );
}

void modbus_reinit_w5500(modbus_t * mb)
{
	if (ethTaskHandler != 0)
	{
		vTaskDelete(ethTaskHandler);
		modbus_init_w5500(mb);
	}
	else
	{
		modbus_init_w5500(mb);
	}

}

void vEthTask(void *pvParameters)
{

	int8_t sock_result;


	sock_result = socket(0, Sn_MR_TCP, mb_pointer->port, SF_TCP_NODELAY);

	for(;;)
	{
		sock_result = listen(0);
		if (sock_result == SOCK_OK)
		{//while socket in listen mode we wait for connection
			while(getSn_SR(0) == SOCK_LISTEN)
				vTaskDelay(100);
			//If we got a connection, read IP address and port
			if (getSn_SR(0) == SOCK_ESTABLISHED)
			{

				for(;;)
				{
					int32_t sock_status; //Don't mess with sock_result, this var shows errors
					sock_status = recv(0, recv_buff, sizeof(recv_buff));

					if (sock_status > 0){
						//led_pc_conn_ctl(Bit_SET);
						//hr_bn_selfcheckout |= 1 <<TCP_IP_CONNSTATE;
						//If we receive valid data (sock_status > 0), send event to modbus
						uint16_t len = (uint16_t) sock_status;
						mb_error_code err = modbus_tcp_process_request(mb_pointer, recv_buff, &output_buffer, &len);
						if (err == MB_ENOERR || err == MB_ENOREG)
						{
							eth_send_tcp_data(output_buffer, len);

						}
						if (output_buffer != 0)
							vPortFree(output_buffer);
					}
					else
					{
						//Port seems to be closed
						//hr_bn_selfcheckout &= ~(1 <<TCP_IP_CONNSTATE);
						//led_pc_conn_ctl(Bit_RESET);
						break;
					}

					vTaskDelay(100);
				}
			}

		}
		else //If socket closed reopen
		{
			close(0);
			socket(0, Sn_MR_TCP, mb_pointer->port, SF_TCP_NODELAY);
		}

	}
}
