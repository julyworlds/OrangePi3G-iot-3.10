#ifndef __IRQ_H__
#define __IRQ_H__

#include "mt_reg_base.h"

/*
 * Define hadware registers.
 */
#define GIC_DIST_BASE       (GIC_BASE + 0x1000)
#define GIC_CPU_BASE        (GIC_BASE + 0x2000)
#define INT_POL_CTL0        (MCUSYS_CFGREG_BASE + 0x100)

/*
 * Define IRQ number.
 */
#define GIC_PRIVATE_SIGNALS     (32)
#define NR_GIC_SGI              (16)
#define NR_GIC_PPI              (16)
#define GIC_PPI_OFFSET          (27)
#define MT_NR_PPI               (5)
#define MT_NR_SPI               (128)
#define NR_MT_IRQ_LINE          (GIC_PPI_OFFSET + MT_NR_PPI + MT_NR_SPI)    // 27 Private interrupt and 5 PPIs and 128 SPIs

/*
 * Define IRQ code.
 */
#define GIC_PPI_GLOBAL_TIMER                (GIC_PPI_OFFSET + 0)
#define GIC_PPI_LEGACY_FIQ                  (GIC_PPI_OFFSET + 1)
#define GIC_PPI_PRIVATE_TIMER               (GIC_PPI_OFFSET + 2)
#define GIC_PPI_WATCHDOG_TIMER              (GIC_PPI_OFFSET + 3)
#define GIC_PPI_LEGACY_IRQ                  (GIC_PPI_OFFSET + 4)
#define MT_CIRQ_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 200)

#define MT_USB0_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 20)
#define MT_TS_IRQ_ID                        (GIC_PRIVATE_SIGNALS + 21)
#define MT_TSATCH_IRQ_ID                    (GIC_PRIVATE_SIGNALS + 22)
#define MT_LOWBATTERY_IRQ_ID                (GIC_PRIVATE_SIGNALS + 23)
#define MT_PWM_IRQ_ID                       (GIC_PRIVATE_SIGNALS + 24)
#define MT_PTP_THERM_IRQ_ID                 (GIC_PRIVATE_SIGNALS + 25)
#define MT_MSDC0_IRQ_ID                     (GIC_PRIVATE_SIGNALS + 26)
#define MT_MSDC1_IRQ_ID                     (GIC_PRIVATE_SIGNALS + 27)
#define MT_I2C0_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 28)
#define MT_I2C1_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 29)

#define MT_PTP_FSM_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 30)  /* new */
#define MT_UART1_IRQ_ID                     (GIC_PRIVATE_SIGNALS + 31)
#define MT_UART2_IRQ_ID                     (GIC_PRIVATE_SIGNALS + 32)
#define MT_NFIECC_IRQ_ID                    (GIC_PRIVATE_SIGNALS + 33)
#define MT_NFI_IRQ_ID                       (GIC_PRIVATE_SIGNALS + 34)
#define MT_GDMA1_IRQ_ID                     (GIC_PRIVATE_SIGNALS + 35)
#define MT_GDMA2_IRQ_ID                     (GIC_PRIVATE_SIGNALS + 36)
#define MT_DMA_I2C0_TX_IRQ_ID               (GIC_PRIVATE_SIGNALS + 37)  /* new */
#define MT_DMA_I2C0_RX_IRQ_ID               (GIC_PRIVATE_SIGNALS + 38)  /* new */
#define MT_DMA_I2C1_TX_IRQ_ID               (GIC_PRIVATE_SIGNALS + 39)  /* new */

#define MT_DMA_I2C1_RX_IRQ_ID               (GIC_PRIVATE_SIGNALS + 40)  /* new */
#define MT_DMA_BTIF_TX_IRQ_ID               (GIC_PRIVATE_SIGNALS + 41)  /* new */
#define MT_DMA_BTIF_RX_IRQ_ID               (GIC_PRIVATE_SIGNALS + 42)  /* new */
#define MT_DMA_UART0_TX_IRQ_ID              (GIC_PRIVATE_SIGNALS + 43)
#define MT_DMA_UART0_RX_IRQ_ID              (GIC_PRIVATE_SIGNALS + 44)
#define MT_DMA_UART1_TX_IRQ_ID              (GIC_PRIVATE_SIGNALS + 45)
#define MT_DMA_UART1_RX_IRQ_ID              (GIC_PRIVATE_SIGNALS + 46)
#define MT_DMA_IRQ_ALL_IRQ_ID               (GIC_PRIVATE_SIGNALS + 47)  /* new */
#define MT_DMA_FIQ_ALL_IRQ_ID               (GIC_PRIVATE_SIGNALS + 48)  /* new */
#define MT_SPI1_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 49)

#define MT_MSDC0_WAKEUP_PS_ID               (GIC_PRIVATE_SIGNALS + 50)
#define MT_MSDC1_WAKEUP_PS_ID               (GIC_PRIVATE_SIGNALS + 51)
#define MT_BTIF_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 52)  /* new */
#define MT_RESERVED_53_IRQ_ID               (GIC_PRIVATE_SIGNALS + 53)
#define MT_RESERVED_54_IRQ_ID               (GIC_PRIVATE_SIGNALS + 54)
#define MT_RESERVED_55_IRQ_ID               (GIC_PRIVATE_SIGNALS + 55)
#define MT_DCC_APARM_IRQ_ID                 (GIC_PRIVATE_SIGNALS + 56)
#define MT_RESERVED_57_IRQ_ID               (GIC_PRIVATE_SIGNALS + 57)
#define MT_APARM_DOMAIN_IRQ_ID              (GIC_PRIVATE_SIGNALS + 58)
#define MT_APARM_DECERR_IRQ_ID              (GIC_PRIVATE_SIGNALS + 59)

#define MT_DOMAIN_ABORT0_ID                 (GIC_PRIVATE_SIGNALS + 60)
#define MT_DOMAIN_ABORT1_ID                 (GIC_PRIVATE_SIGNALS + 61)
#define MT_DOMAIN_ABORT2_ID                 (GIC_PRIVATE_SIGNALS + 62)
#define MT_DOMAIN_ABORT3_ID                 (GIC_PRIVATE_SIGNALS + 63)
#define MT_AFE_IRQ_MCU_ID                   (GIC_PRIVATE_SIGNALS + 64)
#define MT_M4U0_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 65)
#define MT_M4U_SEC_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 66)
#define MT_EMI_ACC_OOR_IRQ_ID               (GIC_PRIVATE_SIGNALS + 67)  /* new */
#define MT_AHBMON_IRQ_ID                    (GIC_PRIVATE_SIGNALS + 68)  /* new */
#define MT_RESERVED_69_IRQ_ID               (GIC_PRIVATE_SIGNALS + 69)

#define MT_RESERVED_70_IRQ_ID               (GIC_PRIVATE_SIGNALS + 70)
#define MT_RESERVED_71_IRQ_ID               (GIC_PRIVATE_SIGNALS + 71)
#define MT_SPM0_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 72)  /* 1-2 (MT_SPM_IRQ_ID) */
#define MT_SPM1_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 73)  /* 1-2 (MT_SPM_IRQ_ID) */
#define MT_GPT_IRQ_ID                       (GIC_PRIVATE_SIGNALS + 74)
#define MT_CCIF0_AP_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 75)
#define MT_EINT_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 76)
#define MT_RESERVED_77_IRQ_ID               (GIC_PRIVATE_SIGNALS + 77)
#define MT_RESERVED_78_IRQ_ID               (GIC_PRIVATE_SIGNALS + 78)
#define MT_RESERVED_79_IRQ_ID               (GIC_PRIVATE_SIGNALS + 79)

#define MT_RESERVED_80_IRQ_ID               (GIC_PRIVATE_SIGNALS + 80)
#define MT_RESERVED_81_IRQ_ID               (GIC_PRIVATE_SIGNALS + 81)
#define MT_RESERVED_82_IRQ_ID               (GIC_PRIVATE_SIGNALS + 82)
#define MT_RESERVED_83_IRQ_ID               (GIC_PRIVATE_SIGNALS + 83)
#define MT_RESERVED_84_IRQ_ID               (GIC_PRIVATE_SIGNALS + 84)
#define MT_RESERVED_85_IRQ_ID               (GIC_PRIVATE_SIGNALS + 85)
#define MT_RESERVED_86_IRQ_ID               (GIC_PRIVATE_SIGNALS + 86)
#define MT_EINT_EVENT_ID                    (GIC_PRIVATE_SIGNALS + 87)
#define MT_PMIC_WRAP_IRQ_ID                 (GIC_PRIVATE_SIGNALS + 88)
#define MT_KP_IRQ_ID                        (GIC_PRIVATE_SIGNALS + 89)

#define MT_RESERVED_90_IRQ_ID               (GIC_PRIVATE_SIGNALS + 90)
#define MT_RESERVED_91_IRQ_ID               (GIC_PRIVATE_SIGNALS + 91)
#define MT_MM_MUTEX_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 92)
#define MT_MDP_RSZ_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 93)
#define MT_MDP_WROT_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 94)
#define MT_MDP_RDMA_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 95)
#define MT_MDP_SHP_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 96)
#define MT_DISP_WDMA_IRQ_ID                 (GIC_PRIVATE_SIGNALS + 97)
#define MT_DISP_RDMA_IRQ_ID                 (GIC_PRIVATE_SIGNALS + 98)
#define MT_DISP_BLS_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 99)

#define MT_DISP_PQ_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 100)
#define MT_DISP_OVL_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 101)
#define MT_DISP_DSI_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 102) /* DSI_IRQ */
#define MT_DISP_DPI_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 103)
#define MT_DISP_DBI_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 104)
#define MT_SMI_LARB0_IRQ_ID                 (GIC_PRIVATE_SIGNALS + 105)
#define MT_CMDQ_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 106)
#define MT_CMDQ_SECURE_IRQ_ID               (GIC_PRIVATE_SIGNALS + 107)
#define MT_VDEC_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 108)
#define MT_VENC_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 109)

#define MT_CAM0_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 110)
#define MT_CAM1_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 111)
#define MT_CAM2_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 112)
#define MT_SENINF_IRQ_ID                    (GIC_PRIVATE_SIGNALS + 113)
#define MT_MFG0_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 114)
#define MT_MFG1_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 115)
#define MT_MFG2_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 116)
#define MT_MFG3_IRQ_ID                      (GIC_PRIVATE_SIGNALS + 117)
#define MT_RESERVED_118_IRQ_ID              (GIC_PRIVATE_SIGNALS + 118)
#define MT_RESERVED_119_IRQ_ID              (GIC_PRIVATE_SIGNALS + 119)

#define MT_CAMERA_IRQ_ID                    MT_CAM0_IRQ_ID
#define MT_DISP_COLOR_IRQ_ID                MT_DISP_PQ_IRQ_ID

#define MT_MD_WDT1_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 120)
#define MT_BT_CVSD_IRQ_ID                   (GIC_PRIVATE_SIGNALS + 121)  /* new */
#define MT_CONN2AP_BTIF_WAKEUP_IRQ_ID       (GIC_PRIVATE_SIGNALS + 122)  /* new */
#define MT_WF_HIF_IRQ_ID                    (GIC_PRIVATE_SIGNALS + 123)  /* new */
#define MT_CONN_WDT_IRQ_ID                  (GIC_PRIVATE_SIGNALS + 124)  /* new */
#define MT_RESERVED_125_IRQ_ID              (GIC_PRIVATE_SIGNALS + 125)
#define MT_WDT_IRQ_ID                       (GIC_PRIVATE_SIGNALS + 126)
#define MT_SYS_CIRQ_EVENT_IRQ_ID            (GIC_PRIVATE_SIGNALS + 127)  /* new */

#define MT_APARM_GPTTIMER_IRQ_LINE          MT_GPT_IRQ_ID  // alias name for GPT

#endif  /*  !__IRQ_H__ */

