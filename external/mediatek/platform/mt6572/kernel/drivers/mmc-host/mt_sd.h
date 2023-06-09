#ifndef MT6572_SD_H
#define MT6572_SD_H

#include <linux/bitops.h>
#include <linux/mmc/host.h>
#include <mach/sync_write.h>
#include <mach/mt_reg_base.h>
#include <linux/semaphore.h>
#define MAX_GPD_NUM         (1 + 1)  /* one null gpd */
#define MAX_BD_NUM          (1024)
#define MAX_BD_PER_GPD      (MAX_BD_NUM)
#define HOST_MAX_NUM        (2)
#define CLK_SRC_MAX_NUM     (7)
#define PERI_MSDC0_PDN      (17)    //Reserved for used by mt_dump.c. Shall be removed after fixing mt_dump.c
#define PERI_MSDC1_PDN      (18)    //Reserved for used by mt_dump.c. Shall be removed after fixing mt_dump.c

#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1
#define CUST_EINT_DEBOUNCE_DISABLE          0
#define CUST_EINT_DEBOUNCE_ENABLE           1
#define CUST_EINT_EDGE_SENSITIVE            0
#define CUST_EINT_LEVEL_SENSITIVE           1
//////////////////////////////////////////////////////////////////////////////


//#define EINT_MSDC2_INS_NUM              14
//#define EINT_MSDC2_INS_DEBOUNCE_CN      1
//#define EINT_MSDC2_INS_POLARITY         CUST_EINT_POLARITY_LOW
//#define EINT_MSDC2_INS_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
//#define EINT_MSDC2_INS_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_ENABLE

#if 0 //chang by jst for sd card hot-plug debug
#define EINT_MSDC1_INS_NUM              18
#define EINT_MSDC1_INS_DEBOUNCE_CN      1
#define EINT_MSDC1_INS_POLARITY         CUST_EINT_POLARITY_LOW
#define EINT_MSDC1_INS_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
#define EINT_MSDC1_INS_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_ENABLE
#else
#define EINT_MSDC1_INS_NUM              6
#define EINT_MSDC1_INS_DEBOUNCE_CN      1
#define EINT_MSDC1_INS_POLARITY         CUST_EINT_POLARITY_HIGH
#define EINT_MSDC1_INS_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
#define EINT_MSDC1_INS_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_ENABLE
#endif

//#define MSDC_CLKSRC_REG     (0xf100000C)
#define MSDC_DESENSE_REG    (0xf0007070)

#ifdef CONFIG_MT6572_FPGA
#define FPGA_PLATFORM
#endif
#define MSDC_AUTOCMD12          (0x0001)
#define MSDC_AUTOCMD23          (0x0002)
#define MSDC_AUTOCMD19          (0x0003)
/*--------------------------------------------------------------------------*/
/* Common Macro                                                             */
/*--------------------------------------------------------------------------*/
#define REG_ADDR(x)                 ((volatile u32*)(base + OFFSET_##x))

/*--------------------------------------------------------------------------*/
/* Common Definition                                                        */
/*--------------------------------------------------------------------------*/
#define MSDC_FIFO_SZ            (128)
#define MSDC_FIFO_THD           (64)  // (128)
#define MSDC_NUM                (4)

#define MSDC_MS                 (0) //No memory stick mode, 0 use to gate clock
#define MSDC_SDMMC              (1)

#define MSDC_MODE_UNKNOWN       (0)
#define MSDC_MODE_PIO           (1)
#define MSDC_MODE_DMA_BASIC     (2)
#define MSDC_MODE_DMA_DESC      (3)
#define MSDC_MODE_DMA_ENHANCED  (4)
#define MSDC_MODE_MMC_STREAM    (5)

#define MSDC_BUS_1BITS          (0)
#define MSDC_BUS_4BITS          (1)
#define MSDC_BUS_8BITS          (2)

#define MSDC_BRUST_8B           (3)
#define MSDC_BRUST_16B          (4)
#define MSDC_BRUST_32B          (5)
#define MSDC_BRUST_64B          (6)

#define MSDC_PIN_PULL_NONE      (0)
#define MSDC_PIN_PULL_DOWN      (1)
#define MSDC_PIN_PULL_UP        (2)
#define MSDC_PIN_KEEP           (3)

#define MSDC_AUTOCMD12          (0x0001)
#define MSDC_AUTOCMD23          (0x0002)
#define MSDC_AUTOCMD19          (0x0003)

#define MSDC_EMMC_BOOTMODE0     (0)     /* Pull low CMD mode */
#define MSDC_EMMC_BOOTMODE1     (1)     /* Reset CMD mode */

enum {
    RESP_NONE = 0,
    RESP_R1,
    RESP_R2,
    RESP_R3,
    RESP_R4,
    RESP_R5,
    RESP_R6,
    RESP_R7,
    RESP_R1B
};

/*--------------------------------------------------------------------------*/
/* Register Offset                                                          */
/*--------------------------------------------------------------------------*/
#define OFFSET_MSDC_CFG         (0x0)
#define OFFSET_MSDC_IOCON       (0x04)
#define OFFSET_MSDC_PS          (0x08)
#define OFFSET_MSDC_INT         (0x0c)
#define OFFSET_MSDC_INTEN       (0x10)
#define OFFSET_MSDC_FIFOCS      (0x14)
#define OFFSET_MSDC_TXDATA      (0x18)
#define OFFSET_MSDC_RXDATA      (0x1c)
#define OFFSET_SDC_CFG          (0x30)
#define OFFSET_SDC_CMD          (0x34)
#define OFFSET_SDC_ARG          (0x38)
#define OFFSET_SDC_STS          (0x3c)
#define OFFSET_SDC_RESP0        (0x40)
#define OFFSET_SDC_RESP1        (0x44)
#define OFFSET_SDC_RESP2        (0x48)
#define OFFSET_SDC_RESP3        (0x4c)
#define OFFSET_SDC_BLK_NUM      (0x50)
#define OFFSET_SDC_CSTS         (0x58)
#define OFFSET_SDC_CSTS_EN      (0x5c)
#define OFFSET_SDC_DCRC_STS     (0x60)
#define OFFSET_EMMC_CFG0        (0x70)
#define OFFSET_EMMC_CFG1        (0x74)
#define OFFSET_EMMC_STS         (0x78)
#define OFFSET_EMMC_IOCON       (0x7c)
#define OFFSET_SDC_ACMD_RESP    (0x80)
#define OFFSET_SDC_ACMD19_TRG   (0x84)
#define OFFSET_SDC_ACMD19_STS   (0x88)
#define OFFSET_MSDC_DMA_SA      (0x90)
#define OFFSET_MSDC_DMA_CA      (0x94)
#define OFFSET_MSDC_DMA_CTRL    (0x98)
#define OFFSET_MSDC_DMA_CFG     (0x9c)
#define OFFSET_MSDC_DBG_SEL     (0xa0)
#define OFFSET_MSDC_DBG_OUT     (0xa4)
#define OFFSET_MSDC_DMA_LEN     (0xa8)
#define OFFSET_MSDC_PATCH_BIT   (0xb0)
#define OFFSET_MSDC_PATCH_BIT1  (0xb4)
#define OFFSET_MSDC_PAD_CTL0    (0xe0)
#define OFFSET_MSDC_PAD_CTL1    (0xe4)
#define OFFSET_MSDC_PAD_CTL2    (0xe8)
#define OFFSET_MSDC_PAD_TUNE    (0xec)
#define OFFSET_MSDC_DAT_RDDLY0  (0xf0)
#define OFFSET_MSDC_DAT_RDDLY1  (0xf4)
#define OFFSET_MSDC_HW_DBG      (0xf8)
#define OFFSET_MSDC_VERSION     (0x100)
#define OFFSET_MSDC_ECO_VER     (0x104)

/*--------------------------------------------------------------------------*/
/* Register Address                                                         */
/*--------------------------------------------------------------------------*/

/* common register */
#define MSDC_CFG                REG_ADDR(MSDC_CFG)
#define MSDC_IOCON              REG_ADDR(MSDC_IOCON)
#define MSDC_PS                 REG_ADDR(MSDC_PS)
#define MSDC_INT                REG_ADDR(MSDC_INT)
#define MSDC_INTEN              REG_ADDR(MSDC_INTEN)
#define MSDC_FIFOCS             REG_ADDR(MSDC_FIFOCS)
#define MSDC_TXDATA             REG_ADDR(MSDC_TXDATA)
#define MSDC_RXDATA             REG_ADDR(MSDC_RXDATA)
#define MSDC_PATCH_BIT0         REG_ADDR(MSDC_PATCH_BIT)

/* sdmmc register */
#define SDC_CFG                 REG_ADDR(SDC_CFG)
#define SDC_CMD                 REG_ADDR(SDC_CMD)
#define SDC_ARG                 REG_ADDR(SDC_ARG)
#define SDC_STS                 REG_ADDR(SDC_STS)
#define SDC_RESP0               REG_ADDR(SDC_RESP0)
#define SDC_RESP1               REG_ADDR(SDC_RESP1)
#define SDC_RESP2               REG_ADDR(SDC_RESP2)
#define SDC_RESP3               REG_ADDR(SDC_RESP3)
#define SDC_BLK_NUM             REG_ADDR(SDC_BLK_NUM)
#define SDC_CSTS                REG_ADDR(SDC_CSTS)
#define SDC_CSTS_EN             REG_ADDR(SDC_CSTS_EN)
#define SDC_DCRC_STS            REG_ADDR(SDC_DCRC_STS)

/* emmc register*/
#define EMMC_CFG0               REG_ADDR(EMMC_CFG0)
#define EMMC_CFG1               REG_ADDR(EMMC_CFG1)
#define EMMC_STS                REG_ADDR(EMMC_STS)
#define EMMC_IOCON              REG_ADDR(EMMC_IOCON)

/* auto command register */
#define SDC_ACMD_RESP           REG_ADDR(SDC_ACMD_RESP)
#define SDC_ACMD19_TRG          REG_ADDR(SDC_ACMD19_TRG)
#define SDC_ACMD19_STS          REG_ADDR(SDC_ACMD19_STS)

/* dma register */
#define MSDC_DMA_SA             REG_ADDR(MSDC_DMA_SA)
#define MSDC_DMA_CA             REG_ADDR(MSDC_DMA_CA)
#define MSDC_DMA_CTRL           REG_ADDR(MSDC_DMA_CTRL)
#define MSDC_DMA_CFG            REG_ADDR(MSDC_DMA_CFG)
#define MSDC_DMA_LEN            REG_ADDR(MSDC_DMA_LEN)

/* pad ctrl register */
#define MSDC_PAD_CTL0           REG_ADDR(MSDC_PAD_CTL0)
#define MSDC_PAD_CTL1           REG_ADDR(MSDC_PAD_CTL1)
#define MSDC_PAD_CTL2           REG_ADDR(MSDC_PAD_CTL2)

/* data read delay */
#define MSDC_DAT_RDDLY0         REG_ADDR(MSDC_DAT_RDDLY0)
#define MSDC_DAT_RDDLY1         REG_ADDR(MSDC_DAT_RDDLY1)

/* debug register */
#define MSDC_DBG_SEL            REG_ADDR(MSDC_DBG_SEL)
#define MSDC_DBG_OUT            REG_ADDR(MSDC_DBG_OUT)

/* misc register */
#define MSDC_PATCH_BIT          REG_ADDR(MSDC_PATCH_BIT)
#define MSDC_PATCH_BIT1         REG_ADDR(MSDC_PATCH_BIT1)
#define MSDC_PAD_TUNE           REG_ADDR(MSDC_PAD_TUNE)
#define MSDC_HW_DBG             REG_ADDR(MSDC_HW_DBG)
#define MSDC_VERSION            REG_ADDR(MSDC_VERSION)
#define MSDC_ECO_VER            REG_ADDR(MSDC_ECO_VER) /* ECO Version */

/*--------------------------------------------------------------------------*/
/* Register Mask                                                            */
/*--------------------------------------------------------------------------*/

/* MSDC_CFG mask */
#define MSDC_CFG_MODE           (0x1  << 0)     /* RW */
#define MSDC_CFG_CKPDN          (0x1  << 1)     /* RW */
#define MSDC_CFG_RST            (0x1  << 2)     /* RW */
#define MSDC_CFG_PIO            (0x1  << 3)     /* RW */
#define MSDC_CFG_CKDRVEN        (0x1  << 4)     /* RW */
#define MSDC_CFG_BV18SDT        (0x1  << 5)     /* RW */
#define MSDC_CFG_BV18PSS        (0x1  << 6)     /* R  */
#define MSDC_CFG_CKSTB          (0x1  << 7)     /* R  */
#define MSDC_CFG_CKDIV          (0xff << 8)     /* RW */
#define MSDC_CFG_CKMOD          (0x3  << 16)    /* RW */

/* MSDC_IOCON mask */
#define MSDC_IOCON_SDR104CKS    (0x1  << 0)     /* RW */
#define MSDC_IOCON_RSPL         (0x1  << 1)     /* RW */
#define MSDC_IOCON_DSPL         (0x1  << 2)     /* RW */
#define MSDC_IOCON_DDLSEL       (0x1  << 3)     /* RW */
#define MSDC_IOCON_DDR50CKD     (0x1  << 4)     /* RW */
#define MSDC_IOCON_DSPLSEL      (0x1  << 5)     /* RW */
#define MSDC_IOCON_W_DSPL       (0x1  << 8)     /* RW */
#define MSDC_IOCON_D0SPL        (0x1  << 16)    /* RW */
#define MSDC_IOCON_D1SPL        (0x1  << 17)    /* RW */
#define MSDC_IOCON_D2SPL        (0x1  << 18)    /* RW */
#define MSDC_IOCON_D3SPL        (0x1  << 19)    /* RW */
#define MSDC_IOCON_D4SPL        (0x1  << 20)    /* RW */
#define MSDC_IOCON_D5SPL        (0x1  << 21)    /* RW */
#define MSDC_IOCON_D6SPL        (0x1  << 22)    /* RW */
#define MSDC_IOCON_D7SPL        (0x1  << 23)    /* RW */
#define MSDC_IOCON_RISCSZ       (0x3  << 24)    /* RW */

/* MSDC_PS mask */
#define MSDC_PS_CDEN            (0x1  << 0)     /* RW */
#define MSDC_PS_CDSTS           (0x1  << 1)     /* R  */
#define MSDC_PS_CDDEBOUNCE      (0xf  << 12)    /* RW */
#define MSDC_PS_DAT             (0xff << 16)    /* R  */
#define MSDC_PS_CMD             (0x1  << 24)    /* R  */
#define MSDC_PS_WP              (0x1UL<< 31)    /* R  */

/* MSDC_INT mask */
#define MSDC_INT_MMCIRQ         (0x1  << 0)     /* W1C */
#define MSDC_INT_CDSC           (0x1  << 1)     /* W1C */
#define MSDC_INT_ACMDRDY        (0x1  << 3)     /* W1C */
#define MSDC_INT_ACMDTMO        (0x1  << 4)     /* W1C */
#define MSDC_INT_ACMDCRCERR     (0x1  << 5)     /* W1C */
#define MSDC_INT_DMAQ_EMPTY     (0x1  << 6)     /* W1C */
#define MSDC_INT_SDIOIRQ        (0x1  << 7)     /* W1C */
#define MSDC_INT_CMDRDY         (0x1  << 8)     /* W1C */
#define MSDC_INT_CMDTMO         (0x1  << 9)     /* W1C */
#define MSDC_INT_RSPCRCERR      (0x1  << 10)    /* W1C */
#define MSDC_INT_CSTA           (0x1  << 11)    /* R */
#define MSDC_INT_XFER_COMPL     (0x1  << 12)    /* W1C */
#define MSDC_INT_DXFER_DONE     (0x1  << 13)    /* W1C */
#define MSDC_INT_DATTMO         (0x1  << 14)    /* W1C */
#define MSDC_INT_DATCRCERR      (0x1  << 15)    /* W1C */
#define MSDC_INT_ACMD19_DONE    (0x1  << 16)    /* W1C */

/* MSDC_INTEN mask */
#define MSDC_INTEN_MMCIRQ       (0x1  << 0)     /* RW */
#define MSDC_INTEN_CDSC         (0x1  << 1)     /* RW */
#define MSDC_INTEN_ACMDRDY      (0x1  << 3)     /* RW */
#define MSDC_INTEN_ACMDTMO      (0x1  << 4)     /* RW */
#define MSDC_INTEN_ACMDCRCERR   (0x1  << 5)     /* RW */
#define MSDC_INTEN_DMAQ_EMPTY   (0x1  << 6)     /* RW */
#define MSDC_INTEN_SDIOIRQ      (0x1  << 7)     /* RW */
#define MSDC_INTEN_CMDRDY       (0x1  << 8)     /* RW */
#define MSDC_INTEN_CMDTMO       (0x1  << 9)     /* RW */
#define MSDC_INTEN_RSPCRCERR    (0x1  << 10)    /* RW */
#define MSDC_INTEN_CSTA         (0x1  << 11)    /* RW */
#define MSDC_INTEN_XFER_COMPL   (0x1  << 12)    /* RW */
#define MSDC_INTEN_DXFER_DONE   (0x1  << 13)    /* RW */
#define MSDC_INTEN_DATTMO       (0x1  << 14)    /* RW */
#define MSDC_INTEN_DATCRCERR    (0x1  << 15)    /* RW */
#define MSDC_INTEN_ACMD19_DONE  (0x1  << 16)    /* RW */

/* MSDC_FIFOCS mask */
#define MSDC_FIFOCS_RXCNT       (0xff << 0)     /* R */
#define MSDC_FIFOCS_TXCNT       (0xff << 16)    /* R */
#define MSDC_FIFOCS_CLR         (0x1UL<< 31)    /* RW */

/* SDC_CFG mask */
#define SDC_CFG_SDIOINTWKUP     (0x1  << 0)     /* RW */
#define SDC_CFG_INSWKUP         (0x1  << 1)     /* RW */
#define SDC_CFG_BUSWIDTH        (0x3  << 16)    /* RW */
#define SDC_CFG_SDIO            (0x1  << 19)    /* RW */
#define SDC_CFG_SDIOIDE         (0x1  << 20)    /* RW */
#define SDC_CFG_INTATGAP        (0x1  << 21)    /* RW */
#define SDC_CFG_DTOC            (0xffUL << 24)  /* RW */

/* SDC_CMD mask */
#define SDC_CMD_OPC             (0x3f << 0)     /* RW */
#define SDC_CMD_BRK             (0x1  << 6)     /* RW */
#define SDC_CMD_RSPTYP          (0x7  << 7)     /* RW */
#define SDC_CMD_DTYP            (0x3  << 11)    /* RW */
#define SDC_CMD_RW              (0x1  << 13)    /* RW */
#define SDC_CMD_STOP            (0x1  << 14)    /* RW */
#define SDC_CMD_GOIRQ           (0x1  << 15)    /* RW */
#define SDC_CMD_BLKLEN          (0xfff<< 16)    /* RW */
#define SDC_CMD_AUTOCMD         (0x3  << 28)    /* RW */
#define SDC_CMD_VOLSWTH         (0x1  << 30)    /* RW */

/* SDC_STS mask */
#define SDC_STS_SDCBUSY         (0x1  << 0)     /* RW */
#define SDC_STS_CMDBUSY         (0x1  << 1)     /* RW */
#define SDC_STS_SWR_COMPL       (0x1  << 31)    /* RW */

/* SDC_DCRC_STS mask */
#define SDC_DCRC_STS_NEG        (0xff << 8)     /* RO */
#define SDC_DCRC_STS_POS        (0xff << 0)     /* RO */

/* EMMC_CFG0 mask */
#define EMMC_CFG0_BOOTSTART     (0x1  << 0)     /* W */
#define EMMC_CFG0_BOOTSTOP      (0x1  << 1)     /* W */
#define EMMC_CFG0_BOOTMODE      (0x1  << 2)     /* RW */
#define EMMC_CFG0_BOOTACKDIS    (0x1  << 3)     /* RW */
#define EMMC_CFG0_BOOTWDLY      (0x7  << 12)    /* RW */
#define EMMC_CFG0_BOOTSUPP      (0x1  << 15)    /* RW */

/* EMMC_CFG1 mask */
#define EMMC_CFG1_BOOTDATTMC    (0xfffff << 0)  /* RW */
#define EMMC_CFG1_BOOTACKTMC    (0xfffUL << 20) /* RW */

/* EMMC_STS mask */
#define EMMC_STS_BOOTCRCERR     (0x1  << 0)     /* W1C */
#define EMMC_STS_BOOTACKERR     (0x1  << 1)     /* W1C */
#define EMMC_STS_BOOTDATTMO     (0x1  << 2)     /* W1C */
#define EMMC_STS_BOOTACKTMO     (0x1  << 3)     /* W1C */
#define EMMC_STS_BOOTUPSTATE    (0x1  << 4)     /* R */
#define EMMC_STS_BOOTACKRCV     (0x1  << 5)     /* W1C */
#define EMMC_STS_BOOTDATRCV     (0x1  << 6)     /* R */

/* EMMC_IOCON mask */
#define EMMC_IOCON_BOOTRST      (0x1  << 0)     /* RW */

/* SDC_ACMD19_TRG mask */
#define SDC_ACMD19_TRG_TUNESEL  (0xf  << 0)     /* RW */

/* MSDC_DMA_CTRL mask */
#define MSDC_DMA_CTRL_START     (0x1  << 0)     /* W */
#define MSDC_DMA_CTRL_STOP      (0x1  << 1)     /* W */
#define MSDC_DMA_CTRL_RESUME    (0x1  << 2)     /* W */
#define MSDC_DMA_CTRL_MODE      (0x1  << 8)     /* RW */
#define MSDC_DMA_CTRL_LASTBUF   (0x1  << 10)    /* RW */
#define MSDC_DMA_CTRL_BRUSTSZ   (0x7  << 12)    /* RW */
//#define MSDC_DMA_CTRL_XFERSZ    (0xffffUL << 16)/* RW */

/* MSDC_DMA_CFG mask */
#define MSDC_DMA_CFG_STS        (0x1  << 0)     /* R */
#define MSDC_DMA_CFG_DECSEN     (0x1  << 1)     /* RW */
#define MSDC_DMA_CFG_BDCSERR    (0x1  << 4)     /* R */
#define MSDC_DMA_CFG_GPDCSERR   (0x1  << 5)     /* R */
#define MSDC_DMA_AHBHPROT2EN    (0x3  << 8)     /* RW */
#define MSDC_DMA_ACTIVEEN       (0x3  << 12)    /* RW */

/* MSDC_PATCH_BIT mask */
//#define CKGEN_RX_SDCLKO_SEL     (0x1  << 0)     /*This bit removed on MT6589/MT8585*/
#define MSDC_PATCH_BIT_ODDSUPP    (0x1  <<  1)     /* RW */
//#define MSDC_PATCH_BIT_CKGEN_CK (0x1  << 6)     /* This bit removed on MT6589/MT8585 (Only use internel clock) */
#define MSDC_INT_DAT_LATCH_CK_SEL (0x7  <<  7)
#define MSDC_CKGEN_MSDC_DLY_SEL   (0x1F << 10)
#define MSDC_PATCH_BIT_DISRXFIFO  (0x1  << 15)    /* RW */ //Disable RXFIFO read
#define MSDC_PATCH_BIT_IODSSEL    (0x1  << 16)    /* RW */
#define MSDC_PATCH_BIT_IOINTSEL   (0x1  << 17)    /* RW */
#define MSDC_PATCH_BIT_BUSYDLY    (0xf  << 18)    /* RW */
#define MSDC_PATCH_BIT_WDOD       (0xf  << 22)    /* RW */
#define MSDC_PATCH_BIT_IDRTSEL    (0x1  << 26)    /* RW */
#define MSDC_PATCH_BIT_CMDFSEL    (0x1  << 27)    /* RW */
#define MSDC_PATCH_BIT_INTDLSEL   (0x1  << 28)    /* RW */
#define MSDC_PATCH_BIT_SPCPUSH    (0x1  << 29)    /* RW */
#define MSDC_PATCH_BIT_DECRCTMO   (0x1  << 30)    /* RW */
#define MSDC_PATCH_BIT_PTCH31     (0x1  << 31)    /* RW */ //Enable MSDC always drives bus when output wakeup response (BREAK)

/* MSDC_PATCH_BIT1 mask */
#define MSDC_PATCH_BIT1_WRDAT_CRCS  (0x7 << 0)
#define MSDC_PATCH_BIT1_CMD_RSP     (0x7 << 3)

/* MSDC_PAD_CTL0 mask */
#define MSDC_PAD_CTL0_CLKDRVN   (0x7  << 0)     /* RW */
#define MSDC_PAD_CTL0_CLKDRVP   (0x7  << 4)     /* RW */
#define MSDC_PAD_CTL0_CLKSR     (0x1  << 8)     /* RW */
#define MSDC_PAD_CTL0_CLKPD     (0x1  << 16)    /* RW */
#define MSDC_PAD_CTL0_CLKPU     (0x1  << 17)    /* RW */
#define MSDC_PAD_CTL0_CLKSMT    (0x1  << 18)    /* RW */
#define MSDC_PAD_CTL0_CLKIES    (0x1  << 19)    /* RW */
#define MSDC_PAD_CTL0_CLKTDSEL  (0xf  << 20)    /* RW */
#define MSDC_PAD_CTL0_CLKRDSEL  (0xffUL<< 24)   /* RW */

/* MSDC_PAD_CTL1 mask */
#define MSDC_PAD_CTL1_CMDDRVN   (0x7  << 0)     /* RW */
#define MSDC_PAD_CTL1_CMDDRVP   (0x7  << 4)     /* RW */
#define MSDC_PAD_CTL1_CMDSR     (0x1  << 8)     /* RW */
#define MSDC_PAD_CTL1_CMDPD     (0x1  << 16)    /* RW */
#define MSDC_PAD_CTL1_CMDPU     (0x1  << 17)    /* RW */
#define MSDC_PAD_CTL1_CMDSMT    (0x1  << 18)    /* RW */
#define MSDC_PAD_CTL1_CMDIES    (0x1  << 19)    /* RW */
#define MSDC_PAD_CTL1_CMDTDSEL  (0xf  << 20)    /* RW */
#define MSDC_PAD_CTL1_CMDRDSEL  (0xffUL<< 24)   /* RW */

/* MSDC_PAD_CTL2 mask */
#define MSDC_PAD_CTL2_DATDRVN   (0x7  << 0)     /* RW */
#define MSDC_PAD_CTL2_DATDRVP   (0x7  << 4)     /* RW */
#define MSDC_PAD_CTL2_DATSR     (0x1  << 8)     /* RW */
#define MSDC_PAD_CTL2_DATPD     (0x1  << 16)    /* RW */
#define MSDC_PAD_CTL2_DATPU     (0x1  << 17)    /* RW */
#define MSDC_PAD_CTL2_DATIES    (0x1  << 19)    /* RW */
#define MSDC_PAD_CTL2_DATSMT    (0x1  << 18)    /* RW */
#define MSDC_PAD_CTL2_DATTDSEL  (0xf  << 20)    /* RW */
#define MSDC_PAD_CTL2_DATRDSEL  (0xffUL<< 24)   /* RW */

/* MSDC_PAD_TUNE mask */
#define MSDC_PAD_TUNE_DATWRDLY  (0x1F << 0)     /* RW */
#define MSDC_PAD_TUNE_DATRRDLY  (0x1F << 8)     /* RW */
#define MSDC_PAD_TUNE_CMDRDLY   (0x1F << 16)    /* RW */
#define MSDC_PAD_TUNE_CMDRRDLY  (0x1FUL << 22)  /* RW */
#define MSDC_PAD_TUNE_CLKTXDLY  (0x1FUL << 27)  /* RW */

/* MSDC_DAT_RDDLY0/1 mask */
#define MSDC_DAT_RDDLY0_D3      (0x1F << 0)     /* RW */
#define MSDC_DAT_RDDLY0_D2      (0x1F << 8)     /* RW */
#define MSDC_DAT_RDDLY0_D1      (0x1F << 16)    /* RW */
#define MSDC_DAT_RDDLY0_D0      (0x1FUL << 24)    /* RW */

#define MSDC_DAT_RDDLY1_D7      (0x1F << 0)     /* RW */
#define MSDC_DAT_RDDLY1_D6      (0x1F << 8)     /* RW */
#define MSDC_DAT_RDDLY1_D5      (0x1F << 16)    /* RW */
#define MSDC_DAT_RDDLY1_D4      (0x1FUL << 24)    /* RW */

#define CARD_READY_FOR_DATA     (1<<8)
#define CARD_CURRENT_STATE(x)   ((x&0x00001E00)>>9)

#define MSDC0_IO_CONFIG_BASE    (IO_CFG_BOTTOM_BASE) //eMMC booting
#define MSDC1_IO_CONFIG_BASE    (IO_CFG_RIGHT_BASE)

//
// Pad control register definition.
//
#define MSDC0_PAD_CTRL_IES_CFG               (MSDC0_IO_CONFIG_BASE + 0x0000)
#define MSDC0_PAD_CTRL_IES_CFG_SET           (MSDC0_IO_CONFIG_BASE + 0x0004)
#define MSDC0_PAD_CTRL_IES_CFG_CLR           (MSDC0_IO_CONFIG_BASE + 0x0008)
#define MSDC0_PAD_CTRL_SR_CFG                (MSDC0_IO_CONFIG_BASE + 0x0010)
#define MSDC0_PAD_CTRL_SR_CFG_SET            (MSDC0_IO_CONFIG_BASE + 0x0014)
#define MSDC0_PAD_CTRL_SR_CFG_CLR            (MSDC0_IO_CONFIG_BASE + 0x0018)
#define MSDC0_PAD_CTRL_SMT_CFG               (MSDC0_IO_CONFIG_BASE + 0x0020)
#define MSDC0_PAD_CTRL_SMT_CFG_SET           (MSDC0_IO_CONFIG_BASE + 0x0024)
#define MSDC0_PAD_CTRL_SMT_CFG_CLR           (MSDC0_IO_CONFIG_BASE + 0x0028)
#define MSDC0_PAD_CTRL_TDSEL_CFG             (MSDC0_IO_CONFIG_BASE + 0x0030)
#define MSDC0_PAD_CTRL_RDSEL_CFG             (MSDC0_IO_CONFIG_BASE + 0x0034)
#define MSDC0_PAD_CTRL_PULLEN_CFG            (MSDC0_IO_CONFIG_BASE + 0x0040)
#define MSDC0_PAD_CTRL_PULLEN_CFG_SET        (MSDC0_IO_CONFIG_BASE + 0x0044)
#define MSDC0_PAD_CTRL_PULLEN_CFG_CLR        (MSDC0_IO_CONFIG_BASE + 0x0048)
#define MSDC0_PAD_CTRL_PULLSEL_CFG           (MSDC0_IO_CONFIG_BASE + 0x0050)
#define MSDC0_PAD_CTRL_PULLSEL_CFG_SET       (MSDC0_IO_CONFIG_BASE + 0x0054)
#define MSDC0_PAD_CTRL_PULLSEL_CFG_CLR       (MSDC0_IO_CONFIG_BASE + 0x0058)
#define MSDC0_PAD_CTRL_DRV_CFG               (MSDC0_IO_CONFIG_BASE + 0x0060)
#define MSDC0_PAD_CTRL_DINV_CFG              (MSDC0_IO_CONFIG_BASE + 0x0070)
#define MSDC0_PAD_CTRL_DINV_CFG_SET          (MSDC0_IO_CONFIG_BASE + 0x0074)
#define MSDC0_PAD_CTRL_DINV_CFG_CLR          (MSDC0_IO_CONFIG_BASE + 0x0078)

#define MSDC1_PAD_CTRL_IES_CFG               (MSDC1_IO_CONFIG_BASE + 0x0000)
#define MSDC1_PAD_CTRL_IES_CFG_SET           (MSDC1_IO_CONFIG_BASE + 0x0004)
#define MSDC1_PAD_CTRL_IES_CFG_CLR           (MSDC1_IO_CONFIG_BASE + 0x0008)
#define MSDC1_PAD_CTRL_SR_CFG                (MSDC1_IO_CONFIG_BASE + 0x0010)
#define MSDC1_PAD_CTRL_SR_CFG_SET            (MSDC1_IO_CONFIG_BASE + 0x0014)
#define MSDC1_PAD_CTRL_SR_CFG_CLR            (MSDC1_IO_CONFIG_BASE + 0x0018)
#define MSDC1_PAD_CTRL_SMT_CFG               (MSDC1_IO_CONFIG_BASE + 0x0020)
#define MSDC1_PAD_CTRL_SMT_CFG_SET           (MSDC1_IO_CONFIG_BASE + 0x0024)
#define MSDC1_PAD_CTRL_SMT_CFG_CLR           (MSDC1_IO_CONFIG_BASE + 0x0028)
#define MSDC1_PAD_CTRL_TDSEL_CFG             (MSDC1_IO_CONFIG_BASE + 0x0030)
#define MSDC1_PAD_CTRL_RDSEL_CFG             (MSDC1_IO_CONFIG_BASE + 0x0034)
#define MSDC1_PAD_CTRL_PULLEN_CFG            (MSDC1_IO_CONFIG_BASE + 0x0040)
#define MSDC1_PAD_CTRL_PULLEN_CFG_SET        (MSDC1_IO_CONFIG_BASE + 0x0044)
#define MSDC1_PAD_CTRL_PULLEN_CFG_CLR        (MSDC1_IO_CONFIG_BASE + 0x0048)
#define MSDC1_PAD_CTRL_PULLSEL_CFG           (MSDC1_IO_CONFIG_BASE + 0x0050)
#define MSDC1_PAD_CTRL_PULLSEL_CFG_SET       (MSDC1_IO_CONFIG_BASE + 0x0054)
#define MSDC1_PAD_CTRL_PULLSEL_CFG_CLR       (MSDC1_IO_CONFIG_BASE + 0x0058)
#define MSDC1_PAD_CTRL_DRV_CFG               (MSDC1_IO_CONFIG_BASE + 0x0060)
#define MSDC1_PAD_CTRL_DINV_CFG              (MSDC1_IO_CONFIG_BASE + 0x0070)
#define MSDC1_PAD_CTRL_DINV_CFG_SET          (MSDC1_IO_CONFIG_BASE + 0x0074)
#define MSDC1_PAD_CTRL_DINV_CFG_CLR          (MSDC1_IO_CONFIG_BASE + 0x0078)

//
// Pad control pull select bit field definition for MSDC0
//
#define MSDC0_PAD_PULLSEL_CK                 (1 << 0)
#define MSDC0_PAD_PULLSEL_CMD                (1 << 1)
#define MSDC0_PAD_PULLSEL_DAT0               (1 << 2)
#define MSDC0_PAD_PULLSEL_DAT1               (1 << 3)
#define MSDC0_PAD_PULLSEL_DAT2               (1 << 4)
#define MSDC0_PAD_PULLSEL_DAT3               (1 << 5)
#define MSDC0_PAD_PULLSEL_DAT4               (1 << 6)
#define MSDC0_PAD_PULLSEL_DAT5               (1 << 7)
#define MSDC0_PAD_PULLSEL_DAT6               (1 << 8)
#define MSDC0_PAD_PULLSEL_DAT7               (1 << 9)
#define MSDC0_PAD_PULLSEL_RSTB               (1 << 10)

#define MSDC0_PAD_PULLSEL_ALL_MASK          (MSDC0_PAD_PULLSEL_CK   | \
                                             MSDC0_PAD_PULLSEL_CMD  | \
                                             MSDC0_PAD_PULLSEL_DAT0 | \
                                             MSDC0_PAD_PULLSEL_DAT1 | \
                                             MSDC0_PAD_PULLSEL_DAT2 | \
                                             MSDC0_PAD_PULLSEL_DAT3 | \
                                             MSDC0_PAD_PULLSEL_DAT4 | \
                                             MSDC0_PAD_PULLSEL_DAT5 | \
                                             MSDC0_PAD_PULLSEL_DAT6 | \
                                             MSDC0_PAD_PULLSEL_DAT7)

#define MSDC0_CLK_PULL8K_EN     (0x3 << 0)
#define MSDC0_CLK_PULL50K_EN    (0x1 << 0)
#define MSDC0_CLK_PULL10K_EN    (0x1 << 1)
#define MSDC0_CMD_PULL8K_EN     (0x3 << 2)
#define MSDC0_CMD_PULL50K_EN    (0x1 << 2)
#define MSDC0_CMD_PULL10K_EN    (0x1 << 3)
#define MSDC0_DAT0_PULL8K_EN    (0x3 << 4)
#define MSDC0_DAT0_PULL50K_EN   (0x1 << 4)
#define MSDC0_DAT0_PULL10K_EN   (0x1 << 5)
#define MSDC0_DAT1_PULL8K_EN    (0x3 << 6)
#define MSDC0_DAT1_PULL50K_EN   (0x1 << 6)
#define MSDC0_DAT1_PULL10K_EN   (0x1 << 7)
#define MSDC0_DAT2_PULL8K_EN    (0x3 << 8)
#define MSDC0_DAT2_PULL50K_EN   (0x1 << 8)
#define MSDC0_DAT2_PULL10K_EN   (0x1 << 9)
#define MSDC0_DAT3_PULL8K_EN    (0x3 << 10)
#define MSDC0_DAT3_PULL50K_EN   (0x1 << 10)
#define MSDC0_DAT3_PULL10K_EN   (0x1 << 11)
#define MSDC0_DAT4_PULL8K_EN    (0x3 << 12)
#define MSDC0_DAT4_PULL50K_EN   (0x1 << 12)
#define MSDC0_DAT4_PULL10K_EN   (0x1 << 13)
#define MSDC0_DAT5_PULL8K_EN    (0x3 << 14)
#define MSDC0_DAT5_PULL50K_EN   (0x1 << 14)
#define MSDC0_DAT5_PULL10K_EN   (0x1 << 15)
#define MSDC0_DAT6_PULL8K_EN    (0x3 << 16)
#define MSDC0_DAT6_PULL50K_EN   (0x1 << 16)
#define MSDC0_DAT6_PULL10K_EN   (0x1 << 17)
#define MSDC0_DAT7_PULL8K_EN    (0x3 << 18)
#define MSDC0_DAT7_PULL50K_EN   (0x1 << 18)
#define MSDC0_DAT7_PULL10K_EN   (0x1 << 19)
#define MSDC0_RST_PULL_EN       (0x1 << 20)

//
// Pad control pull select bit field definition for MSDC1
//
#define MSDC1_PAD_PULLSEL_CK    (1 << 16)
#define MSDC1_PAD_PULLSEL_CMD   (1 << 17)
#define MSDC1_PAD_PULLSEL_DAT0  (1 << 18)
#define MSDC1_PAD_PULLSEL_DAT1  (1 << 19)
#define MSDC1_PAD_PULLSEL_DAT2  (1 << 20)
#define MSDC1_PAD_PULLSEL_DAT3  (1 << 21)

#define MSDC1_PAD_PULLSEL_ALL_MASK          (MSDC1_PAD_PULLSEL_CK   | \
                                             MSDC1_PAD_PULLSEL_CMD  | \
                                             MSDC1_PAD_PULLSEL_DAT0 | \
                                             MSDC1_PAD_PULLSEL_DAT1 | \
                                             MSDC1_PAD_PULLSEL_DAT2 | \
                                             MSDC1_PAD_PULLSEL_DAT3)

#define MSDC1_CLK_PULL_EN       (0x1 << 16)
#define MSDC1_CMD_PULL_EN       (0x1 << 17)
#define MSDC1_DAT0_PULL_EN      (0x1 << 18)
#define MSDC1_DAT1_PULL_EN      (0x1 << 19)
#define MSDC1_DAT2_PULL_EN      (0x1 << 20)
#define MSDC1_DAT3_PULL_EN      (0x1 << 21)

#define MSDC1_PAD_PULL_EN_ALL_MASK          (MSDC1_CLK_PULL_EN  | \
                                             MSDC1_CMD_PULL_EN  | \
                                             MSDC1_DAT0_PULL_EN | \
                                             MSDC1_DAT1_PULL_EN | \
                                             MSDC1_DAT2_PULL_EN | \
                                             MSDC1_DAT3_PULL_EN)

//
// SR, SMT, TDSEL, RDSEL, Driving strength bit filed definition for MSDC0
//
#define MSDC0_IES_CK_CMD_DAT0_3_MASK    (0x1 << 0)  //For eMMC and SD booting
#define MSDC0_IES_DAT4_7_MASK           (0x1 << 1)  //For eMMC booting

#define MSDC0_SR_CK_CMD_DAT0_3_MASK     (0x1 << 0)  //For eMMC and SD booting
#define MSDC0_SR_DAT4_7_MASK            (0x1 << 1)  //For eMMC booting
#define MSDC0_SR_SET_ALL_MASK           (MSDC0_SR_CK_CMD_DAT0_3_MASK | MSDC0_SR_DAT4_7_MASK)

#define MSDC0_SMT_CK_CMD_DAT0_3_MASK    (0x1 << 0)  //For eMMC and SD booting
#define MSDC0_SMT_DAT4_7_MASK           (0x1 << 1)  //For eMMC booting
#define MSDC0_SMT_SET_ALL_MASK          (MSDC0_SMT_CK_CMD_DAT0_3_MASK | MSDC0_SMT_DAT4_7_MASK)

#define MSDC0_TDSEL_RST0_MASK           (0xF)
#define MSDC0_TDSEL_MASK                (0xF  << 4)
#define MSDC0_RDSEL_RST0_MASK           (0x3)
#define MSDC0_RDSEL_MASK                (0x3F << 2)

#define MSDC0_DRV_CK_CMD_DAT0_3_MASK    (0x7 << 0)  //For eMMC and SD booting
#define MSDC0_DRV_DAT4_7_MASK           (0x7 << 3)  //For eMMC booting
#define MSDC0_DRV_RSTB_MASK             (0x3 << 6)  //For eMMC booting
#define MSDC0_DRV_SET_ALL_MASK          (MSDC0_DRV_CK_CMD_DAT0_3_MASK | MSDC0_DRV_DAT4_7_MASK | MSDC0_DRV_RSTB_MASK)

//
// SR, SMT, TDSEL, RDSEL, Driving strength bit filed definition for MSDC1
//
#define MSDC1_IES_CK_CMD_DAT0_3_MASK    (0x1 << 6)  //For eMMC and SD booting

#define MSDC1_SR_CK_CMD_DAT0_3_MASK     (1 << 2)

#define MSDC1_SMT_CK_CMD_DAT0_3_MASK    (0x1 << 2)

#define MSDC1_TDSEL_MASK                (0xF  << 8)
#define MSDC1_RDSEL_MASK                (0x3F << 10)

#define MSDC1_DRV_CK_CMD_DAT0_3_MASK    (0x7 << 10)


////////////////////////////////////////////////////////////////////////
//
// PINMUX bit field definition
//
////////////////////////////////////////////////////////////////////////
#define MSDC0_DAT1_PINMUX_ADDR  (GPIO_BASE+0x0350)
#define MSDC0_DAT1_PINMUX_BITS  (7<<28)
#define MSDC0_DAT1_PINMUX_VAL   (1<<28)

#define MSDC0_DAT0_PINMUX_ADDR  (GPIO_BASE+0x0350)
#define MSDC0_DAT0_PINMUX_BITS  (7<<24)
#define MSDC0_DAT0_PINMUX_VAL   (1<<24)

#define MSDC0_CMD_PINMUX_ADDR   (GPIO_BASE+0x0350)
#define MSDC0_CMD_PINMUX_BITS   (7<<20)
#define MSDC0_CMD_PINMUX_VAL    (1<<20)

#define MSDC0_CLK_PINMUX_ADDR   (GPIO_BASE+0x0350)
#define MSDC0_CLK_PINMUX_BITS   (7<<16)
#define MSDC0_CLK_PINMUX_VAL    (1<<16)

#define MSDC0_RST_PINMUX_ADDR   (GPIO_BASE+0x0360)
#define MSDC0_RST_PINMUX_BITS   (7<<24)
#define MSDC0_RST_PINMUX_VAL    (1<<24)

#define MSDC0_DAT7_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC0_DAT7_PINMUX_BITS  (7<<20)
#define MSDC0_DAT7_PINMUX_VAL   (1<<20)

#define MSDC0_DAT6_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC0_DAT6_PINMUX_BITS  (7<<16)
#define MSDC0_DAT6_PINMUX_VAL   (1<<16)

#define MSDC0_DAT5_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC0_DAT5_PINMUX_BITS  (7<<12)
#define MSDC0_DAT5_PINMUX_VAL   (1<<12)

#define MSDC0_DAT4_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC0_DAT4_PINMUX_BITS  (7<<8)
#define MSDC0_DAT4_PINMUX_VAL   (1<<8)

#define MSDC0_DAT3_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC0_DAT3_PINMUX_BITS  (7<<4)
#define MSDC0_DAT3_PINMUX_VAL   (1<<4)

#define MSDC0_DAT2_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC0_DAT2_PINMUX_BITS  (7<<0)
#define MSDC0_DAT2_PINMUX_VAL   (1<<0)

#define MSDC1_INS1_PINMUX_ADDR  (GPIO_BASE+0x0330)
#define MSDC1_INS1_PINMUX_BITS  (7<<0)
#define MSDC1_INS1_PINMUX_VAL   (2<<0)

#define MSDC1_INS2_PINMUX_ADDR  (GPIO_BASE+0x0360)
#define MSDC1_INS2_PINMUX_BITS  (7<<20)
#define MSDC1_INS2_PINMUX_VAL   (2<<20)

#define MSDC1_INS3_PINMUX_ADDR  (GPIO_BASE+0x0380)
#define MSDC1_INS3_PINMUX_BITS  (7<<12)
#define MSDC1_INS3_PINMUX_VAL   (3<<12)

#define MSDC1_INS4_PINMUX_ADDR  (GPIO_BASE+0x0390)
#define MSDC1_INS4_PINMUX_BITS  (7<<4)
#define MSDC1_INS4_PINMUX_VAL   (4<<4)

#define MSDC1_INS5_PINMUX_ADDR  (GPIO_BASE+0x0390)
#define MSDC1_INS5_PINMUX_BITS  (7<<20)
#define MSDC1_INS5_PINMUX_VAL   (3<<20)

#define MSDC1_INS6_PINMUX_ADDR  (GPIO_BASE+0x03A0)
#define MSDC1_INS6_PINMUX_BITS  (7<<24)
#define MSDC1_INS6_PINMUX_VAL   (4<<24)

#define MSDC1_INS7_PINMUX_ADDR  (GPIO_BASE+0x03B0)
#define MSDC1_INS7_PINMUX_BITS  (7<<0)
#define MSDC1_INS7_PINMUX_VAL   (2<<0)

#define MSDC1_CLK_PINMUX_ADDR   (GPIO_BASE+0x03B0)
#define MSDC1_CLK_PINMUX_BITS   (7<<12)
#define MSDC1_CLK_PINMUX_VAL    (1<<12)

#define MSDC1_CMD_PINMUX_ADDR   (GPIO_BASE+0x03B0)
#define MSDC1_CMD_PINMUX_BITS   (7<<16)
#define MSDC1_CMD_PINMUX_VAL    (1<<16)

#define MSDC1_DAT0_PINMUX_ADDR  (GPIO_BASE+0x03B0)
#define MSDC1_DAT0_PINMUX_BITS  (7<<20)
#define MSDC1_DAT0_PINMUX_VAL   (1<<20)

#define MSDC1_DAT1_PINMUX_ADDR  (GPIO_BASE+0x03B0)
#define MSDC1_DAT1_PINMUX_BITS  (7<<24)
#define MSDC1_DAT1_PINMUX_VAL   (1<<24)

#define MSDC1_DAT2_PINMUX_ADDR  (GPIO_BASE+0x03B0)
#define MSDC1_DAT2_PINMUX_BITS  (7<<28)
#define MSDC1_DAT2_PINMUX_VAL   (1<<28)

#define MSDC1_DAT3_PINMUX_ADDR  (GPIO_BASE+0x03C0)
#define MSDC1_DAT3_PINMUX_BITS  (7<<0)
#define MSDC1_DAT3_PINMUX_VAL   (1<<0)

#define EN18IOKEY_BASE                  (GPIO_BASE2+0x920)

#define MSDC_EN18IO_CMP_SEL_BASE        (GPIO_BASE2+0x910)
#define MSDC1_EN18IO_CMP_EN             (0x1 << 3)
#define MSDC1_EN18IO_SEL1               (0x7 << 0)
#define MSDC2_EN18IO_CMP_EN             (0x1 << 7)
#define MSDC2_EN18IO_SEL                (0x7 << 4)

#define MSDC_EN18IO_SW_BASE             (GPIO_BASE+0x900)
#define MSDC1_EN18IO_SW                 (0x1 << 19)
#define MSDC2_EN18IO_SW                 (0x1 << 25)
#define MSDC1_TUNE                      (0xF << 20)
#define MSDC2_TUNE                      (0xFUL << 26)

typedef enum MSDC_POWER {

    MSDC_VIO18_MC1 = 0,
    MSDC_VIO18_MC2,
    MSDC_VIO28_MC1,
    MSDC_VIO28_MC2,
    MSDC_VMC,
    MSDC_VGP6,
} MSDC_POWER_DOMAIN;




/*--------------------------------------------------------------------------*/
/* Descriptor Structure                                                     */
/*--------------------------------------------------------------------------*/
typedef struct {
    u32  hwo:1; /* could be changed by hw */
    u32  bdp:1;
    u32  rsv0:6;
    u32  chksum:8;
    u32  intr:1;
    u32  rsv1:15;
    void *next;
    void *ptr;
    u32  buflen:16;
    u32  extlen:8;
    u32  rsv2:8;
    u32  arg;
    u32  blknum;
    u32  cmd;
} gpd_t;

typedef struct {
    u32  eol:1;
    u32  rsv0:7;
    u32  chksum:8;
    u32  rsv1:1;
    u32  blkpad:1;
    u32  dwpad:1;
    u32  rsv2:13;
    void *next;
    void *ptr;
    u32  buflen:16;
    u32  rsv3:16;
} bd_t;

/*--------------------------------------------------------------------------*/
/* Register Debugging Structure                                             */
/*--------------------------------------------------------------------------*/

typedef struct {
    u32 msdc:1;
    u32 ckpwn:1;
    u32 rst:1;
    u32 pio:1;
    u32 ckdrven:1;
    u32 start18v:1;
    u32 pass18v:1;
    u32 ckstb:1;
    u32 ckdiv:8;
    u32 ckmod:2;
    u32 pad:14;
} msdc_cfg_reg;
typedef struct {
    u32 sdr104cksel:1;
    u32 rsmpl:1;
    u32 dsmpl:1;
    u32 ddlysel:1;
    u32 ddr50ckd:1;
    u32 dsplsel:1;
    u32 pad1:10;
    u32 d0spl:1;
    u32 d1spl:1;
    u32 d2spl:1;
    u32 d3spl:1;
    u32 d4spl:1;
    u32 d5spl:1;
    u32 d6spl:1;
    u32 d7spl:1;
    u32 riscsz:1;
    u32 pad2:7;
} msdc_iocon_reg;
typedef struct {
    u32 cden:1;
    u32 cdsts:1;
    u32 pad1:10;
    u32 cddebounce:4;
    u32 dat:8;
    u32 cmd:1;
    u32 pad2:6;
    u32 wp:1;
} msdc_ps_reg;
typedef struct {
    u32 mmcirq:1;
    u32 cdsc:1;
    u32 pad1:1;
    u32 atocmdrdy:1;
    u32 atocmdtmo:1;
    u32 atocmdcrc:1;
    u32 dmaqempty:1;
    u32 sdioirq:1;
    u32 cmdrdy:1;
    u32 cmdtmo:1;
    u32 rspcrc:1;
    u32 csta:1;
    u32 xfercomp:1;
    u32 dxferdone:1;
    u32 dattmo:1;
    u32 datcrc:1;
    u32 atocmd19done:1;
    u32 pad2:15;
} msdc_int_reg;
typedef struct {
    u32 mmcirq:1;
    u32 cdsc:1;
    u32 pad1:1;
    u32 atocmdrdy:1;
    u32 atocmdtmo:1;
    u32 atocmdcrc:1;
    u32 dmaqempty:1;
    u32 sdioirq:1;
    u32 cmdrdy:1;
    u32 cmdtmo:1;
    u32 rspcrc:1;
    u32 csta:1;
    u32 xfercomp:1;
    u32 dxferdone:1;
    u32 dattmo:1;
    u32 datcrc:1;
    u32 atocmd19done:1;
    u32 pad2:15;
} msdc_inten_reg;
typedef struct {
    u32 rxcnt:8;
    u32 pad1:8;
    u32 txcnt:8;
    u32 pad2:7;
    u32 clr:1;
} msdc_fifocs_reg;
typedef struct {
    u32 val;
} msdc_txdat_reg;
typedef struct {
    u32 val;
} msdc_rxdat_reg;
typedef struct {
    u32 sdiowkup:1;
    u32 inswkup:1;
    u32 pad1:14;
    u32 buswidth:2;
    u32 pad2:1;
    u32 sdio:1;
    u32 sdioide:1;
    u32 intblkgap:1;
    u32 pad4:2;
    u32 dtoc:8;
} sdc_cfg_reg;
typedef struct {
    u32 cmd:6;
    u32 brk:1;
    u32 rsptyp:3;
    u32 pad1:1;
    u32 dtype:2;
    u32 rw:1;
    u32 stop:1;
    u32 goirq:1;
    u32 blklen:12;
    u32 atocmd:2;
    u32 volswth:1;
    u32 pad2:1;
} sdc_cmd_reg;
typedef struct {
    u32 arg;
} sdc_arg_reg;
typedef struct {
    u32 sdcbusy:1;
    u32 cmdbusy:1;
    u32 pad:29;
    u32 swrcmpl:1;
} sdc_sts_reg;
typedef struct {
    u32 val;
} sdc_resp0_reg;
typedef struct {
    u32 val;
} sdc_resp1_reg;
typedef struct {
    u32 val;
} sdc_resp2_reg;
typedef struct {
    u32 val;
} sdc_resp3_reg;
typedef struct {
    u32 num;
} sdc_blknum_reg;
typedef struct {
    u32 sts;
} sdc_csts_reg;
typedef struct {
    u32 sts;
} sdc_cstsen_reg;
typedef struct {
    u32 datcrcsts:8;
    u32 ddrcrcsts:4;
    u32 pad:20;
} sdc_datcrcsts_reg;
typedef struct {
    u32 bootstart:1;
    u32 bootstop:1;
    u32 bootmode:1;
    u32 pad1:9;
    u32 bootwaidly:3;
    u32 bootsupp:1;
    u32 pad2:16;
} emmc_cfg0_reg;
typedef struct {
    u32 bootcrctmc:16;
    u32 pad:4;
    u32 bootacktmc:12;
} emmc_cfg1_reg;
typedef struct {
    u32 bootcrcerr:1;
    u32 bootackerr:1;
    u32 bootdattmo:1;
    u32 bootacktmo:1;
    u32 bootupstate:1;
    u32 bootackrcv:1;
    u32 bootdatrcv:1;
    u32 pad:25;
} emmc_sts_reg;
typedef struct {
    u32 bootrst:1;
    u32 pad:31;
} emmc_iocon_reg;
typedef struct {
    u32 val;
} msdc_acmd_resp_reg;
typedef struct {
    u32 tunesel:4;
    u32 pad:28;
} msdc_acmd19_trg_reg;
typedef struct {
    u32 val;
} msdc_acmd19_sts_reg;
typedef struct {
    u32 addr;
} msdc_dma_sa_reg;
typedef struct {
    u32 addr;
} msdc_dma_ca_reg;
typedef struct {
    u32 start:1;
    u32 stop:1;
    u32 resume:1;
    u32 pad1:5;
    u32 mode:1;
    u32 pad2:1;
    u32 lastbuf:1;
    u32 pad3:1;
    u32 brustsz:3;
    u32 pad4:1;
    u32 xfersz:16;
} msdc_dma_ctrl_reg;
typedef struct {
    u32 status:1;
    u32 decsen:1;
    u32 pad1:2;
    u32 bdcsen:1;
    u32 gpdcsen:1;
    u32 pad2:26;
} msdc_dma_cfg_reg;
typedef struct {
    u32 sel:16;
    u32 pad2:16;
} msdc_dbg_sel_reg;
typedef struct {
    u32 val;
} msdc_dbg_out_reg;
typedef struct {
    u32 clkdrvn:3;
    u32 rsv0:1;
    u32 clkdrvp:3;
    u32 rsv1:1;
    u32 clksr:1;
    u32 rsv2:7;
    u32 clkpd:1;
    u32 clkpu:1;
    u32 clksmt:1;
    u32 clkies:1;
    u32 clktdsel:4;
    u32 clkrdsel:8;
} msdc_pad_ctl0_reg;
typedef struct {
    u32 cmddrvn:3;
    u32 rsv0:1;
    u32 cmddrvp:3;
    u32 rsv1:1;
    u32 cmdsr:1;
    u32 rsv2:7;
    u32 cmdpd:1;
    u32 cmdpu:1;
    u32 cmdsmt:1;
    u32 cmdies:1;
    u32 cmdtdsel:4;
    u32 cmdrdsel:8;
} msdc_pad_ctl1_reg;
typedef struct {
    u32 datdrvn:3;
    u32 rsv0:1;
    u32 datdrvp:3;
    u32 rsv1:1;
    u32 datsr:1;
    u32 rsv2:7;
    u32 datpd:1;
    u32 datpu:1;
    u32 datsmt:1;
    u32 daties:1;
    u32 dattdsel:4;
    u32 datrdsel:8;
} msdc_pad_ctl2_reg;
typedef struct {
    u32 wrrxdly:3;
    u32 pad1:5;
    u32 rdrxdly:8;
    u32 pad2:16;
} msdc_pad_tune_reg;
typedef struct {
    u32 dat0:5;
    u32 rsv0:3;
    u32 dat1:5;
    u32 rsv1:3;
    u32 dat2:5;
    u32 rsv2:3;
    u32 dat3:5;
    u32 rsv3:3;
} msdc_dat_rddly0;
typedef struct {
    u32 dat4:5;
    u32 rsv4:3;
    u32 dat5:5;
    u32 rsv5:3;
    u32 dat6:5;
    u32 rsv6:3;
    u32 dat7:5;
    u32 rsv7:3;
} msdc_dat_rddly1;
typedef struct {
    u32 dbg0sel:8;
    u32 dbg1sel:6;
    u32 pad1:2;
    u32 dbg2sel:6;
    u32 pad2:2;
    u32 dbg3sel:6;
    u32 pad3:2;
} msdc_hw_dbg_reg;
typedef struct {
    u32 val;
} msdc_version_reg;
typedef struct {
    u32 val;
} msdc_eco_ver_reg;

struct msdc_regs {
    msdc_cfg_reg        msdc_cfg;      /* base+0x00h */
    msdc_iocon_reg      msdc_iocon;    /* base+0x04h */
    msdc_ps_reg         msdc_ps;       /* base+0x08h */
    msdc_int_reg        msdc_int;      /* base+0x0ch */
    msdc_inten_reg      msdc_inten;    /* base+0x10h */
    msdc_fifocs_reg     msdc_fifocs;   /* base+0x14h */
    msdc_txdat_reg      msdc_txdat;    /* base+0x18h */
    msdc_rxdat_reg      msdc_rxdat;    /* base+0x1ch */
    u32                 rsv1[4];
    sdc_cfg_reg         sdc_cfg;       /* base+0x30h */
    sdc_cmd_reg         sdc_cmd;       /* base+0x34h */
    sdc_arg_reg         sdc_arg;       /* base+0x38h */
    sdc_sts_reg         sdc_sts;       /* base+0x3ch */
    sdc_resp0_reg       sdc_resp0;     /* base+0x40h */
    sdc_resp1_reg       sdc_resp1;     /* base+0x44h */
    sdc_resp2_reg       sdc_resp2;     /* base+0x48h */
    sdc_resp3_reg       sdc_resp3;     /* base+0x4ch */
    sdc_blknum_reg      sdc_blknum;    /* base+0x50h */
    u32                 rsv2[1];
    sdc_csts_reg        sdc_csts;      /* base+0x58h */
    sdc_cstsen_reg      sdc_cstsen;    /* base+0x5ch */
    sdc_datcrcsts_reg   sdc_dcrcsta;   /* base+0x60h */
    u32                 rsv3[3];
    emmc_cfg0_reg       emmc_cfg0;     /* base+0x70h */
    emmc_cfg1_reg       emmc_cfg1;     /* base+0x74h */
    emmc_sts_reg        emmc_sts;      /* base+0x78h */
    emmc_iocon_reg      emmc_iocon;    /* base+0x7ch */
    msdc_acmd_resp_reg  acmd_resp;     /* base+0x80h */
    msdc_acmd19_trg_reg acmd19_trg;    /* base+0x84h */
    msdc_acmd19_sts_reg acmd19_sts;    /* base+0x88h */
    u32                 rsv4[1];
    msdc_dma_sa_reg     dma_sa;        /* base+0x90h */
    msdc_dma_ca_reg     dma_ca;        /* base+0x94h */
    msdc_dma_ctrl_reg   dma_ctrl;      /* base+0x98h */
    msdc_dma_cfg_reg    dma_cfg;       /* base+0x9ch */
    msdc_dbg_sel_reg    dbg_sel;       /* base+0xa0h */
    msdc_dbg_out_reg    dbg_out;       /* base+0xa4h */
    u32                 rsv5[2];
    u32                 patch0;        /* base+0xb0h */
    u32                 patch1;        /* base+0xb4h */
    u32                 rsv6[10];
    msdc_pad_ctl0_reg   pad_ctl0;      /* base+0xe0h */
    msdc_pad_ctl1_reg   pad_ctl1;      /* base+0xe4h */
    msdc_pad_ctl2_reg   pad_ctl2;      /* base+0xe8h */
    msdc_pad_tune_reg   pad_tune;      /* base+0xech */
    msdc_dat_rddly0     dat_rddly0;    /* base+0xf0h */
    msdc_dat_rddly1     dat_rddly1;    /* base+0xf4h */
    msdc_hw_dbg_reg     hw_dbg;        /* base+0xf8h */
    u32                 rsv7[1];
    msdc_version_reg    version;       /* base+0x100h */
    msdc_eco_ver_reg    eco_ver;       /* base+0x104h */
};

struct scatterlist_ex {
    u32 cmd;
    u32 arg;
    u32 sglen;
    struct scatterlist *sg;
};

#define DMA_FLAG_NONE       (0x00000000)
#define DMA_FLAG_EN_CHKSUM  (0x00000001)
#define DMA_FLAG_PAD_BLOCK  (0x00000002)
#define DMA_FLAG_PAD_DWORD  (0x00000004)

struct msdc_dma {
    u32 flags;                   /* flags */
    u32 xfersz;                  /* xfer size in bytes */
    u32 sglen;                   /* size of scatter list */
    u32 blklen;                  /* block size */
    struct scatterlist *sg;      /* I/O scatter list */
    struct scatterlist_ex *esg;  /* extended I/O scatter list */
    u8  mode;                    /* dma mode        */
    u8  burstsz;                 /* burst size      */
    u8  intr;                    /* dma done interrupt */
    u8  padding;                 /* padding */
    u32 cmd;                     /* enhanced mode command */
    u32 arg;                     /* enhanced mode arg */
    u32 rsp;                     /* enhanced mode command response */
    u32 autorsp;                 /* auto command response */

    gpd_t *gpd;                  /* pointer to gpd array */
    bd_t  *bd;                   /* pointer to bd array */
    dma_addr_t gpd_addr;         /* the physical address of gpd array */
    dma_addr_t bd_addr;          /* the physical address of bd array */
    u32 used_gpd;                /* the number of used gpd elements */
    u32 used_bd;                 /* the number of used bd elements */
};

struct tune_counter
{
    u32 time_cmd;
    u32 time_read;
    u32 time_write;
};
struct msdc_saved_para
{
    u32                         pad_tune;
    u32                         ddly0;
    u32                         ddly1;
    u8                          cmd_resp_ta_cntr;
    u8                          wrdat_crc_ta_cntr;
    u8                          suspend_flag;
    u32                         msdc_cfg;
    u32                         mode;
    u32                         div;
    u32                         sdc_cfg;
    u32                         iocon;
    int                         ddr;
    u32                         hz;
    u8                          int_dat_latch_ck_sel;
    u8                          ckgen_msdc_dly_sel;
};
struct msdc_host
{
    struct msdc_hw              *hw;

    struct mmc_host             *mmc;           /* mmc structure */
    struct mmc_command          *cmd;
    struct mmc_data             *data;
    struct mmc_request          *mrq;
    int                         cmd_rsp;
    int                         cmd_rsp_done;
    int                         cmd_r1b_done;

    int                         error;
    spinlock_t                  lock;           /* mutex */
    spinlock_t                  clk_gate_lock;
    spinlock_t                  remove_bad_card;    /*to solve removing bad card race condition with hot-plug enable*/
    int                         clk_gate_count;
    struct semaphore            sem;

    u32                         blksz;          /* host block size */
    u32                         base;           /* host base address */
    int                         id;             /* host id */
    int                         pwr_ref;        /* core power reference count */

    u32                         xfer_size;      /* total transferred size */

    struct msdc_dma             dma;            /* dma channel */
    u32                         dma_addr;       /* dma transfer address */
    u32                         dma_left_size;  /* dma transfer left size */
    u32                         dma_xfer_size;  /* dma transfer size in bytes */
    int                         dma_xfer;       /* dma transfer mode */

    u32                         timeout_ns;     /* data timeout ns */
    u32                         timeout_clks;   /* data timeout clks */

    atomic_t                    abort;          /* abort transfer */

    int                         irq;            /* host interrupt */

    struct tasklet_struct       card_tasklet;
    //struct delayed_work           remove_card;

    struct completion           cmd_done;
    struct completion           xfer_done;
    struct pm_message           pm_state;

    u32                         mclk;           /* mmc subsystem clock */
    u32                         hclk;           /* host clock speed */
    u32                         sclk;           /* SD/MS clock speed */
    u8                          core_clkon;     /* Host core clock on ? */
    u8                          card_clkon;     /* Card clock on ? */
    u8                          core_power;     /* core power */
    u8                          power_mode;     /* host power mode */
    u8                          card_inserted;  /* card inserted ? */
    u8                          suspend;        /* host suspended ? */
    u8                          reserved;
    u8                          app_cmd;        /* for app command */
    u32                         app_cmd_arg;
    u64                         starttime;
    struct timer_list           timer;
    struct tune_counter         t_counter;
    u32                         rwcmd_time_tune;
    int                         read_time_tune;
    int                         write_time_tune;
    u32                         write_timeout_uhs104;
    u32                         read_timeout_uhs104;
    u8                          autocmd;
    u32                         sw_timeout;
    u32                         power_cycle; /* power cycle done in tuning flow*/
    bool                        power_cycle_enable;/*Enable power cycle*/
    u32                         sd_30_busy;
    bool                        tune;
    MSDC_POWER_DOMAIN           power_domain;
    bool                        ddr;
    struct msdc_saved_para      saved_para;
    int                         sd_cd_polarity;
    int                         sd_cd_insert_work; //to make sure insert mmc_rescan this work in start_host when boot up
                                                   //driver will get a EINT(Level sensitive) when boot up phone with card insert
    bool                        block_bad_card;
    void                        (*power_control)(struct msdc_host *host,u32 on);
    void                        (*power_switch)(struct msdc_host *host,u32 on);
};
typedef enum {
   TRAN_MOD_PIO,
   TRAN_MOD_DMA,
   TRAN_MOD_NUM
}transfer_mode;

typedef enum {
   OPER_TYPE_READ,
   OPER_TYPE_WRITE,
   OPER_TYPE_NUM
}operation_type;

struct dma_addr{
   u32 start_address;
   u32 size;
   u8 end;
   struct dma_addr *next;
};

static inline unsigned int uffs(unsigned int x)
{
    unsigned int r = 1;

    if (!x)
        return 0;
    if (!(x & 0xffff)) {
        x >>= 16;
        r += 16;
    }
    if (!(x & 0xff)) {
        x >>= 8;
        r += 8;
    }
    if (!(x & 0xf)) {
        x >>= 4;
        r += 4;
    }
    if (!(x & 3)) {
        x >>= 2;
        r += 2;
    }
    if (!(x & 1)) {
        x >>= 1;
        r += 1;
    }
    return r;
}
#define sdr_read8(reg)           __raw_readb(reg)
#define sdr_read16(reg)          __raw_readw(reg)
#define sdr_read32(reg)          __raw_readl(reg)
#if 0
#define sdr_write8(reg,val)      __raw_writeb(val,reg)
#define sdr_write16(reg,val)     __raw_writew(val,reg)
#define sdr_write32(reg,val)     __raw_writel(val,reg)
#define sdr_set_bits(reg,bs)     ((*(volatile u32*)(reg)) |= (u32)(bs))
#define sdr_clr_bits(reg,bs)     ((*(volatile u32*)(reg)) &= ~((u32)(bs)))
#else
#define sdr_write8(reg,val)      mt65xx_reg_sync_writeb(val,reg)
#define sdr_write16(reg,val)     mt65xx_reg_sync_writew(val,reg)
#define sdr_write32(reg,val)     mt65xx_reg_sync_writel(val,reg)
#define sdr_set_bits(reg,bs) \
    do{\
        volatile unsigned int tv = sdr_read32(reg);\
        tv |= (u32)(bs); \
        sdr_write32(reg,tv); \
    }while(0)
#define sdr_clr_bits(reg,bs) \
do{\
        volatile unsigned int tv = sdr_read32(reg);\
        tv &= ~((u32)(bs)); \
        sdr_write32(reg,tv); \
    }while(0)

#endif

#define sdr_set_field(reg,field,val) \
    do {    \
        volatile unsigned int tv = sdr_read32(reg); \
        tv &= ~(field); \
        tv |= ((val) << (uffs((unsigned int)field) - 1)); \
        sdr_write32(reg,tv); \
    } while(0)
#define sdr_get_field(reg,field,val) \
    do {    \
        volatile unsigned int tv = sdr_read32(reg); \
        val = ((tv & (field)) >> (uffs((unsigned int)field) - 1)); \
    } while(0)
#define sdr_set_field_discrete(reg,field,val) \
            do {    \
                volatile unsigned int tv = sdr_read32(reg); \
                tv = (val == 1) ? (tv|(field)):(tv & ~(field));\
                sdr_write32(reg,tv); \
            } while(0)
#define sdr_get_field_discrete(reg,field,val) \
            do {    \
                volatile unsigned int tv = sdr_read32(reg); \
                val = tv & (field) ; \
                val = (val == field) ? 1 :0;\
            } while(0)
#endif

