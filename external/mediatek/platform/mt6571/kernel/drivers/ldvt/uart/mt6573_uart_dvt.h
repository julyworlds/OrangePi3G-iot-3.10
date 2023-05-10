/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MT6573_UART_DVT_H__
#define	__MT6573_UART_DVT_H__

#define MAX_UART_PORT_ID            (2)
#define MIN_UART_PORT_ID            (1)

/* UART GPIO Settings section */
#define UART_GPIO_PIN_UN_SUPPORT    (-1)
/* UART 1 Pin ID*/
#define UART1_TX_GPIO_PIN           (96)
#define UART1_RX_GPIO_PIN           (95)
#define UART1_CTS_GPIO_PIN          (98)
#define UART1_RTS_GPIO_PIN          (101)
/* UART 1 Pin mode */
#define UART1_TX_GPIO_PIN_M         (1)
#define UART1_RX_GPIO_PIN_M         (1)
#define UART1_CTS_GPIO_PIN_M        (4)
#define UART1_RTS_GPIO_PIN_M        (4)

/* UART 2 Pin ID*/
#define UART2_TX_GPIO_PIN           (94)
#define UART2_RX_GPIO_PIN           (93)
#define UART2_CTS_GPIO_PIN          (48)
#define UART2_RTS_GPIO_PIN          (49)
/* UART 2 Pin mode */
#define UART2_TX_GPIO_PIN_M         (1)
#define UART2_RX_GPIO_PIN_M         (1)
#define UART2_CTS_GPIO_PIN_M        (5)
#define UART2_RTS_GPIO_PIN_M        (5)

/* UART IRQ ID Section */
#define UART1_IRQ_ID                (31)
#define UART2_IRQ_ID                (32)

/* UART Registers */
#define UART_RBR(base)                    (base+0x00) /* Read only */
#define UART_THR(base)                    (base+0x00) /* Write only */
#define UART_IER(base)                    (base+0x04)
#define UART_IIR(base)                    (base+0x08) /* Read only */
#define UART_FCR(base)                    (base+0x08) /* Write only */
#define UART_LCR(base)                    (base+0x0c)
#define UART_MCR(base)                    (base+0x10)
#define UART_LSR(base)                    (base+0x14)
#define UART_MSR(base)                    (base+0x18)
#define UART_SCR(base)                    (base+0x1c)
#define UART_DLL(base)                    (base+0x00) /* Only when LCR.DLAB = 1 */
#define UART_DLH(base)                    (base+0x04) /* Only when LCR.DLAB = 1 */
#define UART_EFR(base)                    (base+0x08) /* Only when LCR = 0xbf */
#define UART_XON1(base)                   (base+0x10) /* Only when LCR = 0xbf */
#define UART_XON2(base)                   (base+0x14) /* Only when LCR = 0xbf */
#define UART_XOFF1(base)                  (base+0x18) /* Only when LCR = 0xbf */
#define UART_XOFF2(base)                  (base+0x1c) /* Only when LCR = 0xbf */
#define UART_AUTOBAUD_EN(base)            (base+0x20)
#define UART_HIGHSPEED(base)              (base+0x24)
#define UART_SAMPLE_COUNT(base)           (base+0x28) 
#define UART_SAMPLE_POINT(base)           (base+0x2c) 
#define UART_AUTOBAUD_REG(base)           (base+0x30)
#define UART_RATE_FIX_AD(base)            (base+0x34)
#define UART_AUTOBAUD_SAMPLE(base)        (base+0x38)
#define UART_GUARD(base)                  (base+0x3c)
#define UART_ESCAPE_DAT(base)             (base+0x40)
#define UART_ESCAPE_EN(base)              (base+0x44)
#define UART_SLEEP_EN(base)               (base+0x48)
#define UART_DMA_EN(base)                 (base+0x4c)
#define UART_RXTRI_AD(base)               (base+0x50)
#define UART_FRACDIV_L(base)              (base+0x54)
#define UART_FRACDIV_M(base)              (base+0x58)
#define UART_FCR_RD(base)                 (base+0x5C)
#define UART_ACTIVE_EN(base)              (base+0x60)

typedef struct _UART_REG_LIST{
    unsigned int UART_RBR;
    unsigned int UART_THR;
    unsigned int UART_IER;
    unsigned int UART_IIR;
    unsigned int UART_FCR;
    unsigned int UART_LCR;
    unsigned int UART_MCR;
    unsigned int UART_LSR;
    unsigned int UART_MSR;
    unsigned int UART_SCR;
    unsigned int UART_DLL;
    unsigned int UART_DLH;
    unsigned int UART_EFR;
    unsigned int UART_XON1;
    unsigned int UART_XON2;
    unsigned int UART_XOFF1;
    unsigned int UART_XOFF2;
    unsigned int UART_AUTOBAUD_EN;
    unsigned int UART_HIGHSPEED;
    unsigned int UART_SAMPLE_COUNT;
    unsigned int UART_SAMPLE_POINT;
    unsigned int UART_AUTOBAUD_REG;
    unsigned int UART_RATE_FIX_AD;
    unsigned int UART_AUTOBAUD_SAMPLE;
    unsigned int UART_GUARD;
    unsigned int UART_ESCAPE_DAT;
    unsigned int UART_ESCAPE_EN;
    unsigned int UART_SLEEP_EN;
    unsigned int UART_DMA_EN;
    unsigned int UART_RXTRI_AD;
    unsigned int UART_FRACDIV_L;
    unsigned int UART_FRACDIV_M;
    unsigned int UART_FCR_RD;
    unsigned int UART_ACTIVE_EN;
}UART_REG_LIST;

/* VFF TX Registers */
#define VFF_TX_INT_FLAG(_b)            (_b+0x0000)
#define VFF_TX_INT_EN(_b)              (_b+0x0004)
#define VFF_TX_EN(_b)                  (_b+0x0008)
#define VFF_TX_RST(_b)                 (_b+0x000C)
#define VFF_TX_STOP(_b)                (_b+0x0010)
#define VFF_TX_FLUSH(_b)               (_b+0x0014)
#define VFF_TX_ADDR(_b)                (_b+0x001C)
#define VFF_TX_LEN(_b)                 (_b+0x0024)
#define VFF_TX_THRE(_b)                (_b+0x0028)
#define VFF_TX_WPT(_b)                 (_b+0x002C)
#define VFF_TX_RPT(_b)                 (_b+0x0030)
#define VFF_TX_W_INT_BUF_SIZE(_b)      (_b+0x0034)
#define VFF_TX_INT_BUF_SIZE(_b)        (_b+0x0038)
#define VFF_TX_VALID_SIZE(_b)          (_b+0x003C)
#define VFF_TX_LEFT_SIZE(_b)           (_b+0x0040)
#define VFF_TX_DEBUG_STATUS(_b)        (_b+0x0050)

typedef struct _VFF_TX_REG_LIST{
    unsigned int VFF_TX_INT_FLAG;
    unsigned int VFF_TX_INT_EN;
    unsigned int VFF_TX_EN;
    unsigned int VFF_TX_RST;
    unsigned int VFF_TX_STOP;
    unsigned int VFF_TX_FLUSH;
    unsigned int VFF_TX_ADDR;
    unsigned int VFF_TX_LEN;
    unsigned int VFF_TX_THRE;
    unsigned int VFF_TX_WPT;
    unsigned int VFF_TX_RPT;
    unsigned int VFF_TX_W_INT_BUF_SIZE;
    unsigned int VFF_TX_INT_BUF_SIZE;
    unsigned int VFF_TX_VALID_SIZE;
    unsigned int VFF_TX_LEFT_SIZE;
    unsigned int VFF_TX_DEBUG_STATUS;
}VFF_TX_REG_LIST;

/* VFF RX Registers */
#define VFF_RX_INT_FLAG(_b)            (_b+0x0000)
#define VFF_RX_INT_EN(_b)              (_b+0x0004)
#define VFF_RX_EN(_b)                  (_b+0x0008)
#define VFF_RX_RST(_b)                 (_b+0x000C)
#define VFF_RX_STOP(_b)                (_b+0x0010)
#define VFF_RX_FLUSH(_b)               (_b+0x0014)
#define VFF_RX_ADDR(_b)                (_b+0x001C)
#define VFF_RX_LEN(_b)                 (_b+0x0024)
#define VFF_RX_THRE(_b)                (_b+0x0028)
#define VFF_RX_WPT(_b)                 (_b+0x002C)
#define VFF_RX_RPT(_b)                 (_b+0x0030)
#define VFF_RX_FC_THRSHOLD(_b)         (_b+0x0034)
#define VFF_RX_INT_BUF_SIZE(_b)        (_b+0x0038)
#define VFF_RX_VALID_SIZE(_b)          (_b+0x003C)
#define VFF_RX_LEFT_SIZE(_b)           (_b+0x0040)
#define VFF_RX_DEBUG_STATUS(_b)        (_b+0x0050)

typedef struct _VFF_RX_REG_LIST{
    unsigned int VFF_RX_INT_FLAG;
    unsigned int VFF_RX_INT_EN;
    unsigned int VFF_RX_EN;
    unsigned int VFF_RX_RST;
    unsigned int VFF_RX_STOP;
    unsigned int VFF_RX_FLUSH;
    unsigned int VFF_RX_ADDR;
    unsigned int VFF_RX_LEN;
    unsigned int VFF_RX_THRE;
    unsigned int VFF_RX_WPT;
    unsigned int VFF_RX_RPT;
    unsigned int VFF_RX_FC_THRSHOLD;
    unsigned int VFF_RX_INT_BUF_SIZE;
    unsigned int VFF_RX_VALID_SIZE;
    unsigned int VFF_RX_LEFT_SIZE;
    unsigned int VFF_RX_DEBUG_STATUS;
}VFF_RX_REG_LIST;

/* Base address section */
#define UART1_BASE_ADDR     (0xF1005000)
#define UART2_BASE_ADDR     (0xF1006000)

#define VFF_TX1_BASE_ADDR   (0xF1000000 + 0x480)
#define VFF_TX2_BASE_ADDR   (0xF1000000 + 0x580)

#define VFF_RX1_BASE_ADDR   (0xF1000000 + 0x500)
#define VFF_RX2_BASE_ADDR   (0xF1000000 + 0x600)

/* Error info */
enum{
	UART_DRV_EXT_ERR_SUCCESS = 0,
	UART_DRV_EXT_ERR_INVALID_UART_ID = 1000,
	UART_DRV_EXT_ERR_MAPPING,
	UART_DRV_EXT_ERR_UNDEFINED_OP,
	UART_DRV_EXT_ERR_NULL_POINTER,
	UART_DRV_EXT_ERR_MONITOR_MISS_MATCH,
};

#endif //__MT6573_UART_DVT_H__

