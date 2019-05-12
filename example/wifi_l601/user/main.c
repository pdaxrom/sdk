/***************************************************************************** 
* 
* File Name : main.c
* 
* Description: main 
* 
* Copyright (c) 2014 Winner Micro Electronic Design Co., Ltd. 
* All rights reserved. 
* 
* Author : dave
* 
* Date : 2014-6-14
*****************************************************************************/ 
#include "wm_include.h"
#include "wm_gpio_afsel.h"
#include "wm_uart.h"

#define USER_TASK_STK_SIZE	1024
#define UART_RX_BUF_SIZE	1024
#define USER_TASK_PRIO          32

#define QUEUE_SIZE	32

#define MSG_UART0_RECEIVE_DATA	1
#define MSG_UART1_RECEIVE_DATA	2

static u32 user_task_stk[USER_TASK_STK_SIZE];

static tls_os_queue_t *task_q = NULL;

static struct {
    int msg_num;
    int	rx_len[2];
    unsigned char *rx_buf[2];
} task_data;

static s16 uart0_cb_rx(u16 len)
{
    task_data.rx_len[0] = len;

    if (task_data.msg_num < 3)
    {
        task_data.msg_num++;
        tls_os_queue_send(task_q, (void *) MSG_UART0_RECEIVE_DATA, 0);
    }

    return WM_SUCCESS;
}

static s16 uart1_cb_rx(u16 len)
{
    task_data.rx_len[1] = len;

    if (task_data.msg_num < 3)
    {
        task_data.msg_num++;
        tls_os_queue_send(task_q, (void *) MSG_UART1_RECEIVE_DATA, 0);
    }

    return WM_SUCCESS;
}

void uart2uart_task(void *data)
{
    tls_uart_options_t opt;
    void *msg;
    int ret;

    task_data.rx_buf[0] = tls_mem_alloc(UART_RX_BUF_SIZE);
    task_data.rx_buf[1] = tls_mem_alloc(UART_RX_BUF_SIZE);
    task_data.rx_len[0] = 0;
    task_data.rx_len[1] = 0;
    task_data.msg_num = 0;

    tls_uart_set_baud_rate(TLS_UART_0, 115200);
    tls_uart_rx_callback_register(TLS_UART_0, uart0_cb_rx);

    opt.baudrate = 115200;
    opt.paritytype = 0;
    opt.stopbits = 0;
    opt.charlength = TLS_UART_CHSIZE_8BIT;
    opt.flow_ctrl = TLS_UART_FLOW_CTRL_NONE;

    wm_uart1_rx_config(WM_IO_PB_11);
    wm_uart1_tx_config(WM_IO_PB_12);

    if (WM_SUCCESS != tls_uart_port_init(TLS_UART_1, &opt, 0)) {
	printf("uart1 init error\n");
    }

    //tls_uart_set_baud_rate(TLS_UART_1, 115200);
    tls_uart_rx_callback_register(TLS_UART_1, uart1_cb_rx);

    for (;;)
    {
        tls_os_queue_receive(task_q, (void **) &msg, 0, 0);

        switch ((u32) msg)
        {
	    case MSG_UART0_RECEIVE_DATA: {
		ret = tls_uart_read(TLS_UART_0, task_data.rx_buf[0], task_data.rx_len[0]);
		if (task_data.msg_num) {
		    task_data.msg_num--;
		}
		if(ret <= 0) {
		    break;
		}
//		printf("uart0 rx %d of %d\r\n", ret, task_data.rx_len[0]);
		tls_uart_write(TLS_UART_1, task_data.rx_buf[0], ret);
	    }
	    case MSG_UART1_RECEIVE_DATA: {
		ret = tls_uart_read(TLS_UART_1, task_data.rx_buf[1], task_data.rx_len[1]);
		if (task_data.msg_num) {
		    task_data.msg_num--;
		}
		if(ret <= 0) {
		    break;
		}
//		printf("uart1 rx %d of %d\r\n", ret, task_data.rx_len[1]);
		tls_uart_write(TLS_UART_0, task_data.rx_buf[1], ret);
	    }
	}
    }
}

void UserMain(void)
{
    printf("\r\nw600 hello world example, compile @%s %s\r\n", __DATE__, __TIME__);

    tls_os_queue_create(&task_q, QUEUE_SIZE);

    /* create task */
    tls_os_task_create(NULL,
            "helloworld",
            uart2uart_task,
            (void*) 0,
            (void*) &user_task_stk,  /* 任务栈的起始地址 */
            USER_TASK_STK_SIZE *sizeof(u32),  /* 任务栈的大小     */
            USER_TASK_PRIO,
            0);

//    uart_demo(115200, 0, 0);
}
