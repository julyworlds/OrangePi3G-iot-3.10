/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/******************************************************************************
 * mt_pmic_wrap.c - MT6572 PMIC Wrapper Driver
 *
 *
 * DESCRIPTION:
 *     This file provides wrapper functions for other drivers to access PMIC registers
 *
 ******************************************************************************/

#include "typedefs.h"
#include "mtk_pll.h"
#include "mtk_timer.h"
#include "mt_pmic_wrap_init.h"

//---start ---internal API--------------------------------------------------
static S32 _pwrap_wacs2_nochk( U32 write, U32 adr, U32 wdata, U32 *rdata );
static S32 _pwrap_reset_spislv(void);
static S32 _pwrap_init_dio( U32 dio_en );
static S32 _pwrap_init_cipher( void );
static S32 _pwrap_init_reg_clock( U32 regck_sel );
static BOOL _pwrap_timeout_ns (U64 start_time_ns, U64 timeout_time_ns);
static U64 _pwrap_get_current_time(void);
static U64 _pwrap_time2ns (U64 time_us);
static S32 pwrap_read_nochk( U32  adr, U32 *rdata );
static S32 pwrap_write_nochk( U32  adr, U32  wdata );
static S32 _pwrap_wacs2_nochk( U32 write, U32 adr, U32 wdata, U32 *rdata );
//static void pwrap_trace_wacs2(void);
void pwrap_dump_ap_register(void);
//---end--internal API--------------------------------------------------

/******************************************************************************
 wrapper timeout
******************************************************************************/
#define PWRAP_TIMEOUT
//use the same API name with kernel driver
//however,the timeout API in preloader use tick instead of ns
#ifdef PWRAP_TIMEOUT
static U64 _pwrap_get_current_time(void)
{
  return gpt4_get_current_tick();
}
//_pwrap_timeout_tick,use the same API name with kernel driver
static BOOL _pwrap_timeout_ns (U64 start_time, U64 elapse_time)
{
  return gpt4_timeout_tick(start_time, elapse_time);
}
//_pwrap_time2tick_us
static U64 _pwrap_time2ns (U64 time_us)
{
  return gpt4_time2tick_us(time_us);
}

#else
static U64 _pwrap_get_current_time(void)
{
  return 0;
}
static BOOL _pwrap_timeout_ns (U64 start_time, U64 elapse_time)//,U64 timeout_ns)
{
  return FALSE;
}
static U64 _pwrap_time2ns (U64 time_us)
{
  return 0;
}

#endif
//#####################################################################
//define macro and inline function (for do while loop)
//#####################################################################
typedef U32 (*loop_condition_fp)(U32);//define a function pointer

static inline U32 wait_for_fsm_idle(U32 x)
{
  return (GET_WACS0_FSM( x ) != WACS_FSM_IDLE );
}
static inline U32 wait_for_fsm_vldclr(U32 x)
{
  return (GET_WACS0_FSM( x ) != WACS_FSM_WFVLDCLR);
}
static inline U32 wait_for_sync(U32 x)
{
  return (GET_SYNC_IDLE0(x) != WACS_SYNC_IDLE);
}
static inline U32 wait_for_idle_and_sync(U32 x)
{
  return ((GET_WACS0_FSM(x) != WACS_FSM_IDLE) || (GET_SYNC_IDLE0(x) != WACS_SYNC_IDLE)) ;
}
static inline U32 wait_for_wrap_idle(U32 x)
{
  return ((GET_WRAP_FSM(x) != 0x0) || (GET_WRAP_CH_DLE_RESTCNT(x) != 0x0));
}
static inline U32 wait_for_wrap_state_idle(U32 x)
{
  return ( GET_WRAP_AG_DLE_RESTCNT( x ) != 0 ) ;
}
static inline U32 wait_for_man_idle_and_noreq(U32 x)
{
  return ( (GET_MAN_REQ(x) != MAN_FSM_NO_REQ ) || (GET_MAN_FSM(x) != MAN_FSM_IDLE) );
}
static inline U32 wait_for_man_vldclr(U32 x)
{
  return  (GET_MAN_FSM( x ) != MAN_FSM_WFVLDCLR) ;
}
static inline U32 wait_for_cipher_ready(U32 x)
{
  return (x!=1) ;
}
static inline U32 wait_for_stdupd_idle(U32 x)
{
  return ( GET_STAUPD_FSM(x) != 0x0) ;
}

static inline U32 wait_for_state_ready_init(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 *read_reg)
{

  U64 start_time_ns=0, timeout_ns=0;
  U32 reg_rdata=0x0;
  start_time_ns = _pwrap_get_current_time();
  timeout_ns = _pwrap_time2ns(timeout_us);
  do
  {
    reg_rdata = WRAP_RD32(wacs_register);

    if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
    {
      PWRAPERR("%s timeout when waiting for idle\n", __FUNCTION__);
      return E_PWR_WAIT_IDLE_TIMEOUT;
    }
  } while( fp(reg_rdata)); //IDLE State
  if(read_reg)
   *read_reg=reg_rdata;
  return 0;
}

static inline U32 wait_for_state_idle_init(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 wacs_vldclr_register,U32 *read_reg)
{

  U64 start_time_ns=0, timeout_ns=0;
  U32 reg_rdata;
  start_time_ns = _pwrap_get_current_time();
  timeout_ns = _pwrap_time2ns(timeout_us);
  do
  {
    reg_rdata = WRAP_RD32(wacs_register);
    //if last read command timeout,clear vldclr bit
    //read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle
    switch ( GET_WACS0_FSM( reg_rdata ) )
    {
      case WACS_FSM_WFVLDCLR:
        WRAP_WR32(wacs_vldclr_register , 1);
        PWRAPERR("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
        break;
      case WACS_FSM_WFDLE:
        PWRAPERR("WACS_FSM = WACS_FSM_WFDLE\n");
        break;
      case WACS_FSM_REQ:
        PWRAPERR("WACS_FSM = WACS_FSM_REQ\n");
        break;
      default:
        break;
    }
    if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
    {
      PWRAPERR("%s timeout when waiting for idle\n", __FUNCTION__);
      pwrap_dump_ap_register();
      //pwrap_trace_wacs2();
      //BUG_ON(1);
      return E_PWR_WAIT_IDLE_TIMEOUT;
    }
  }while( fp(reg_rdata)); //IDLE State
  if(read_reg)
   *read_reg=reg_rdata;
  return 0;
}

static inline U32 wait_for_state_idle(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 wacs_vldclr_register,U32 *read_reg)
{

  U64 start_time_ns=0, timeout_ns=0;
  U32 reg_rdata;
  start_time_ns = _pwrap_get_current_time();
  timeout_ns = _pwrap_time2ns(timeout_us);
  do
  {
    reg_rdata = WRAP_RD32(wacs_register);
    if( GET_INIT_DONE0( reg_rdata ) != WACS_INIT_DONE)
    {
      PWRAPERR("initialization isn't finished \n");
      return E_PWR_NOT_INIT_DONE;
    }
    //if last read command timeout,clear vldclr bit
    //read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle
    switch ( GET_WACS0_FSM( reg_rdata ) )
    {
      case WACS_FSM_WFVLDCLR:
        WRAP_WR32(wacs_vldclr_register , 1);
        PWRAPERR("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
        break;
      case WACS_FSM_WFDLE:
        PWRAPERR("WACS_FSM = WACS_FSM_WFDLE\n");
        break;
      case WACS_FSM_REQ:
        PWRAPERR("WACS_FSM = WACS_FSM_REQ\n");
        break;
      default:
        break;
    }
    if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
    {
      PWRAPERR("%s timeout when waiting for idle\n", __FUNCTION__);
      pwrap_dump_ap_register();
//      pwrap_trace_wacs2();
//      BUG_ON(1);
      return E_PWR_WAIT_IDLE_TIMEOUT;
    }
  }while( fp(reg_rdata)); //IDLE State
  if(read_reg)
   *read_reg=reg_rdata;
  return 0;
}

static inline U32 wait_for_state_ready(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 *read_reg)
{

  U64 start_time_ns=0, timeout_ns=0;
  U32 reg_rdata;
  start_time_ns = _pwrap_get_current_time();
  timeout_ns = _pwrap_time2ns(timeout_us);
  do
  {
    reg_rdata = WRAP_RD32(wacs_register);

    if( GET_INIT_DONE0( reg_rdata ) != WACS_INIT_DONE)
    {
      PWRAPERR("initialization isn't finished \n");
      return E_PWR_NOT_INIT_DONE;
    }
    if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
    {
      PWRAPERR("timeout when waiting for idle\n");
      pwrap_dump_ap_register();
      return E_PWR_WAIT_IDLE_TIMEOUT;
    }
  } while( fp(reg_rdata)); //IDLE State
  if(read_reg)
   *read_reg=reg_rdata;
  return 0;
}
//******************************************************************************
//--external API for pmic_wrap user-------------------------------------------------
//******************************************************************************
S32 pwrap_read( U32  adr, U32 *rdata )
{
  return pwrap_wacs2( 0, adr,0,rdata );
}

S32 pwrap_write( U32  adr, U32  wdata )
{
  return pwrap_wacs2( 1, adr,wdata,0 );
}
//--------------------------------------------------------
//    Function : pwrap_wacs2()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 pwrap_wacs2( U32  write, U32  adr, U32  wdata, U32 *rdata )
{
  //U64 wrap_access_time=0x0;
  U32 reg_rdata=0;
  U32 wacs_write=0;
  U32 wacs_adr=0;
  U32 wacs_cmd=0;
  U32 return_value=0;
  //unsigned long flags=0;
  //struct pmic_wrap_obj *pwrap_obj = g_pmic_wrap_obj;
  //if (!pwrap_obj)
  //      PWRAPERR("NULL pointer\n");
  //PWRAPFUC();
  //PWRAPLOG("wrapper access,write=%x,add=%x,wdata=%x,rdata=%x\n",write,adr,wdata,rdata);

  // Check argument validation
  if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
  if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
  if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;

  //spin_lock_irqsave(&pwrap_obj->spin_lock,flags);
  // Check IDLE & INIT_DONE in advance
  return_value=wait_for_state_idle(wait_for_fsm_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,PMIC_WRAP_WACS2_VLDCLR,0);
  if(return_value!=0)
  {
    PWRAPERR("wait_for_fsm_idle fail,return_value=%d\n",return_value);
    goto FAIL;
  }
  wacs_write  = write << 31;
  wacs_adr    = (adr >> 1) << 16;
  wacs_cmd = wacs_write | wacs_adr | wdata;

  WRAP_WR32(PMIC_WRAP_WACS2_CMD,wacs_cmd);
  if( write == 0 )
  {
    return_value=wait_for_state_ready(wait_for_fsm_vldclr,TIMEOUT_READ,PMIC_WRAP_WACS2_RDATA,&reg_rdata);
    if(return_value!=0)
    {
      PWRAPERR("wait_for_fsm_vldclr fail,return_value=%d\n",return_value);
      return_value+=1;//E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ
      goto FAIL;
    }
    if (NULL == rdata)
    {
      PWRAPERR("rdata is a NULL pointer\n");
      return_value= E_PWR_INVALID_ARG;
      WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);
      goto FAIL;
    }

    *rdata = GET_WACS0_RDATA( reg_rdata );
    WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);
  }
FAIL:
  //spin_unlock_irqrestore(&pwrap_obj->spin_lock,flags);
  if(return_value!=0)
  {
    PWRAPERR("%s fail,return_value=%d\n", __FUNCTION__, return_value);
    PWRAPERR("timeout:BUG_ON here\n");
    //BUG_ON(1);
  }
  //wrap_access_time=_pwrap_get_current_time();
  //pwrap_trace(wrap_access_time,return_value,write, adr, wdata,rdata);
  return return_value;
}
//******************************************************************************
//--internal API for pwrap_init-------------------------------------------------
//******************************************************************************

//--------------------------------------------------------
//    Function : _pwrap_wacs2_nochk()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 pwrap_read_nochk( U32  adr, U32 *rdata )
{
  return _pwrap_wacs2_nochk( 0, adr,  0, rdata );
}

static S32 pwrap_write_nochk( U32  adr, U32  wdata )
{
  return _pwrap_wacs2_nochk( 1, adr,wdata,0 );
}

static S32 _pwrap_wacs2_nochk( U32 write, U32 adr, U32 wdata, U32 *rdata )
{
  U32 reg_rdata=0x0;
  U32 wacs_write=0x0;
  U32 wacs_adr=0x0;
  U32 wacs_cmd=0x0;
  U32 return_value=0x0;
  //PWRAPFUC();
  // Check argument validation
  if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
  if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
  if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;

  // Check IDLE
  return_value=wait_for_state_idle_init(wait_for_fsm_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,PMIC_WRAP_WACS2_VLDCLR,0);
  if(return_value!=0)
  {
    PWRAPERR("wait_for_fsm_idle fail,return_value=%d\n",return_value);
    return return_value;
  }

  wacs_write  = write << 31;
  wacs_adr    = (adr >> 1) << 16;
  wacs_cmd = wacs_write | wacs_adr | wdata;
  WRAP_WR32(PMIC_WRAP_WACS2_CMD,wacs_cmd);

  if( write == 0 )
  {
    return_value=wait_for_state_ready_init(wait_for_fsm_vldclr,TIMEOUT_READ,PMIC_WRAP_WACS2_RDATA,&reg_rdata);
    if(return_value!=0)
    {
      PWRAPERR("wait_for_fsm_vldclr fail,return_value=%d\n",return_value);
      return_value+=1;//E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ
      return return_value;
    }
    if (NULL == rdata)
    {
      PWRAPERR("rdata is a NULL pointer\n");
      WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);
      return E_PWR_INVALID_ARG;
    }

    *rdata = GET_WACS0_RDATA( reg_rdata );
    WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);
  }
  return 0;
}

//--------------------------------------------------------
//    Function : _pwrap_init_dio()
// Description :call it in pwrap_init,mustn't check init done
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _pwrap_init_dio( U32 dio_en )
{
  U32 arb_en_backup=0x0;
  U32 rdata=0x0;
  U32 return_value=0;

  //PWRAPFUC();
  arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 0x8); // only WACS2
  pwrap_write_nochk(DEW_DIO_EN, dio_en);

  // Check IDLE & INIT_DONE in advance
  return_value=wait_for_state_ready_init(wait_for_idle_and_sync,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
  if(return_value!=0)
  {
    PWRAPERR("%s fail,return_value=%x\n", __FUNCTION__, return_value);
    return return_value;
  }
  WRAP_WR32(PMIC_WRAP_DIO_EN , dio_en);
  // Read Test
  pwrap_read_nochk(DEW_READ_TEST,&rdata);
  if( rdata != DEFAULT_VALUE_READ_TEST )
  {
    PWRAPERR("[Dio_mode][Read Test] fail,dio_en = %x, READ_TEST rdata=%x, exp=0x5aa5\n", dio_en, rdata);
    return E_PWR_READ_TEST_FAIL;
  }
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);
  return 0;
}

//--------------------------------------------------------
//    Function : _pwrap_init_cipher()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _pwrap_init_cipher( void )
{
  U32 arb_en_backup=0;
  U32 rdata=0;
  U32 return_value=0;
  U64 start_time_ns=0, timeout_ns=0;
  //PWRAPFUC();
  arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);

  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 0x8); // only WACS2

  WRAP_WR32(PMIC_WRAP_CIPHER_SWRST , 1);
  WRAP_WR32(PMIC_WRAP_CIPHER_SWRST , 0);
  WRAP_WR32(PMIC_WRAP_CIPHER_KEY_SEL , 1);
  WRAP_WR32(PMIC_WRAP_CIPHER_IV_SEL  , 2);
  // WRAP_WR32(PMIC_WRAP_CIPHER_LOAD, 1);
  WRAP_WR32(PMIC_WRAP_CIPHER_EN, 1);

  //Config CIPHER @ PMIC
  pwrap_write_nochk(DEW_CIPHER_SWRST,   0x1);
  pwrap_write_nochk(DEW_CIPHER_SWRST,   0x0);
  pwrap_write_nochk(DEW_CIPHER_KEY_SEL, 0x1);
  pwrap_write_nochk(DEW_CIPHER_IV_SEL,  0x2);
#ifdef SLV_6320
  pwrap_write_nochk(DEW_CIPHER_LOAD,    0x1);
  pwrap_write_nochk(DEW_CIPHER_START,   0x1);
#else
  pwrap_write_nochk(DEW_CIPHER_EN, 0x1);
#endif

  //wait for cipher data ready@AP
  return_value=wait_for_state_ready_init(wait_for_cipher_ready,TIMEOUT_WAIT_IDLE,PMIC_WRAP_CIPHER_RDY,0);
  if(return_value!=0)
  {
    PWRAPERR("wait for cipher data ready@AP fail,return_value=%x\n", return_value);
    return return_value;
  }

  //wait for cipher data ready@PMIC
  start_time_ns = _pwrap_get_current_time();
  timeout_ns = _pwrap_time2ns(TIMEOUT_WAIT_IDLE);
  do
  {
    pwrap_read_nochk(DEW_CIPHER_RDY,&rdata);
    if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
    {
      PWRAPERR("wait for cipher data ready@PMIC\n");
      //return E_PWR_WAIT_IDLE_TIMEOUT;
    }
  } while( rdata != 0x1 ); //cipher_ready

  pwrap_write_nochk(DEW_CIPHER_MODE, 0x1);
  //wait for cipher mode idle
  return_value=wait_for_state_ready_init(wait_for_idle_and_sync,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
  if(return_value!=0)
  {
    PWRAPERR("wait for cipher mode idle fail,return_value=%x\n", return_value);
    return return_value;
  }
  WRAP_WR32(PMIC_WRAP_CIPHER_MODE , 1);

  // Read Test
  pwrap_read_nochk(DEW_READ_TEST,&rdata);
  if( rdata != DEFAULT_VALUE_READ_TEST )
  {
    PWRAPERR("%s,read test error,error code=%x, rdata=%x\n", __FUNCTION__, 1, rdata);
    return E_PWR_READ_TEST_FAIL;
  }

  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);
  return 0;
}

//--------------------------------------------------------
//    Function : _pwrap_init_sistrobe()
// Description : Initialize SI_CK_CON and SIDLY
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _pwrap_init_sistrobe( void )
{
  U32 arb_en_backup;
  U32 rdata;
  S32 ind;
  U32 tmp1, tmp2;
  U32 result;
  U32 result_faulty;
  U32 leading_one, tailing_one;

  arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);

  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN ,0x8); // only WACS2

  //---------------------------------------------------------------------
  // Scan all possible input strobe by READ_TEST
  //---------------------------------------------------------------------
  result = 0;
  result_faulty = 0;
  for( ind=0; ind<24; ind++)  // 24 sampling clock edge
  {
    WRAP_WR32(PMIC_WRAP_SI_CK_CON , (ind >> 2) & 0x7);
    WRAP_WR32(PMIC_WRAP_SIDLY ,0x3 - (ind & 0x3));
    _pwrap_wacs2_nochk(0, DEW_READ_TEST, 0, &rdata);

    if( rdata == DEFAULT_VALUE_READ_TEST )
      result |= (0x1 << ind);

    PWRAPLOG("%s [Read Test], index=%d, rdata=0x%x\n", __FUNCTION__, ind, rdata);
  }

  //---------------------------------------------------------------------
  // Locate the leading one and trailing one
  //---------------------------------------------------------------------
  for( ind=23 ; ind>=0 ; ind-- )
  {
    if( result & (0x1 << ind) ) break;
  }
  leading_one = ind;

  for( ind=0 ; ind<24 ; ind++ )
  {
    if( result & (0x1 << ind) ) break;
  }
  tailing_one = ind;

  //---------------------------------------------------------------------
  // Check the continuity of pass range
  //---------------------------------------------------------------------
  tmp1 = (0x1 << (leading_one+1)) - 1;
  tmp2 = (0x1 << tailing_one) - 1;
  if( (tmp1 - tmp2) != result )
  {
    /*TERR = "[DrvPWRAP_InitSiStrobe] Fail, tmp1:%d, tmp2:%d", tmp1, tmp2*/
    PWRAPERR("%s Fail,tmp1=%x,tmp2=%x\n", __FUNCTION__, tmp1, tmp2);
    result_faulty = 0x1;
  }

  //---------------------------------------------------------------------
  // Config SICK and SIDLY to the middle point of pass range
  //---------------------------------------------------------------------
  ind = (leading_one + tailing_one)/2;
  WRAP_WR32(PMIC_WRAP_SI_CK_CON , (ind >> 2) & 0x7);
  WRAP_WR32(PMIC_WRAP_SIDLY , 0x3 - (ind & 0x3));

  //---------------------------------------------------------------------
  // Restore
  //---------------------------------------------------------------------
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);

  if( result_faulty == 0 )
    return 0;
  else
  {
    PWRAPERR("%s Fail,result=%x\n", __FUNCTION__, result);
    return result_faulty;
  }
}

//--------------------------------------------------------
//    Function : _pwrap_reset_spislv()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _pwrap_reset_spislv( void )
{
  U32 ret=0;
  U32 return_value=0;
  //PWRAPFUC();
  // This driver does not using _pwrap_switch_mux
  // because the remaining requests are expected to fail anyway

  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 0);
  WRAP_WR32(PMIC_WRAP_WRAP_EN , 0);
  WRAP_WR32(PMIC_WRAP_MUX_SEL , 1);
  WRAP_WR32(PMIC_WRAP_MAN_EN ,1);
  WRAP_WR32(PMIC_WRAP_DIO_EN , 0);

  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_CSL  << 8));
  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_OUTS << 8)); //to reset counter
  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_CSH  << 8));
  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_OUTS << 8));
  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_OUTS << 8));
  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_OUTS << 8));
  WRAP_WR32(PMIC_WRAP_MAN_CMD , (OP_WR << 13) | (OP_OUTS << 8));

  return_value=wait_for_state_ready_init(wait_for_sync,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
  if(return_value!=0)
  {
    PWRAPERR("%s fail,return_value=%x\n", __FUNCTION__, return_value);
    ret=E_PWR_TIMEOUT;
    goto timeout;
  }

  WRAP_WR32(PMIC_WRAP_MAN_EN , 0);
  WRAP_WR32(PMIC_WRAP_MUX_SEL , 0);

timeout:
  WRAP_WR32(PMIC_WRAP_MAN_EN , 0);
  WRAP_WR32(PMIC_WRAP_MUX_SEL , 0);
  return ret;
}

static S32 _pwrap_init_reg_clock( U32 regck_sel )
{
  U32 wdata = 0;
  U32 rdata = 0;
  PWRAPFUC();

  // Set reg clk freq
#ifdef SLV_6320
  pwrap_read_nochk(PMIC_TOP_CKCON2, &rdata);
#endif

#ifdef SLV_6320
  if(regck_sel == 1)
    wdata = (rdata & (~(0x3 << 10))) | (0x1 << 10);
  else
    wdata = rdata & ~(0x3 << 10);
#else
  if(regck_sel == 1) // not supported in 6323!!
    return E_PWR_INIT_REG_CLOCK;
  else
    wdata = 0x3;
#endif

#ifdef SLV_6320
  pwrap_write_nochk(PMIC_TOP_CKCON2, wdata);
  pwrap_read_nochk(PMIC_TOP_CKCON2, &rdata);
#else
  pwrap_write_nochk(TOP_CKCON1_CLR, wdata);
  pwrap_read_nochk(TOP_CKCON1, &rdata);
#endif

#ifdef SLV_6320
  if(rdata != wdata) {
    PWRAPERR("%s,PMIC_TOP_CKCON2 Write Fail, rdata=%x\n", __FUNCTION__, rdata);
    return E_PWR_WRITE_TEST_FAIL;
  }
#else
  if((rdata & 0x3) != 0) {
    PWRAPERR("%s, TOP_CKCON1 Write Fail, rdata=%x\n", __FUNCTION__, rdata);
    return E_PWR_WRITE_TEST_FAIL;
  }
#endif

// Set Dummy cycle for both 6320(assume 18MHz)/6323 (assume 12MHz)
#ifdef SLV_6320
  WRAP_WR32(PMIC_WRAP_RDDMY, 0x5);
#else
  pwrap_write_nochk(DEW_RDDMY_NO, 0x8);
  WRAP_WR32(PMIC_WRAP_RDDMY, 0x8);
#endif

  // Config SPI Waveform according to reg clk
  if( regck_sel == 1 ) { // 6MHz in 6323 => not support; 18MHz in 6320
    WRAP_WR32(PMIC_WRAP_CSHEXT_WRITE, 0x4); // wait data written into register => 3T_PMIC
    // for 6320, slave need enough time (4T of PMIC reg_ck) to back idle state
    WRAP_WR32(PMIC_WRAP_CSHEXT_READ,  0x5);
    WRAP_WR32(PMIC_WRAP_CSLEXT_START, 0x0);
    WRAP_WR32(PMIC_WRAP_CSLEXT_END,   0x0);
  } else if( regck_sel == 2 ) { //12 MHz in 6323; 36MHz in 6320
#ifdef SLV_6320
    // for 6320, slave need enough time (4T of PMIC reg_ck) to back idle state
    WRAP_WR32(PMIC_WRAP_CSHEXT_READ, 0x2);
    WRAP_WR32(PMIC_WRAP_CSHEXT_WRITE, 0x2);
    WRAP_WR32(PMIC_WRAP_RDDMY, 0x2);
#else
    WRAP_WR32(PMIC_WRAP_CSHEXT_READ, 0x0);
    // wait data written into register => 3T_PMIC: consists of CSLEXT_END(1T) + CSHEXT(6T)
    WRAP_WR32(PMIC_WRAP_CSHEXT_WRITE, 0x5);
#endif
    WRAP_WR32(PMIC_WRAP_CSLEXT_START, 0x0);
    WRAP_WR32(PMIC_WRAP_CSLEXT_END,   0x0);
  } else { //Safe mode
    WRAP_WR32(PMIC_WRAP_CSHEXT_WRITE   , 0xF);
    WRAP_WR32(PMIC_WRAP_CSHEXT_READ    , 0xF);
    WRAP_WR32(PMIC_WRAP_CSLEXT_START   , 0xF);
    WRAP_WR32(PMIC_WRAP_CSLEXT_END     , 0xF);
  }

  return 0;
}

S32 pwrap_init(void)
{
  S32 sub_return=0;
  S32 sub_return1=0;
  U32 rdata=0x0;
  U32 clk_sel = 0;
  U32 cg_mask = 0;
  U32 backup = 0;

  //U32 timeout=0;
  PWRAPFUC();
  //###############################
  //toggle PMIC_WRAP and pwrap_spictl reset
  //###############################
  // Turn off module clock
  cg_mask = ((1 << 20) | (1 << 27) | (1 << 28) | (1 << 29));
  backup = (~WRAP_RD32(CLK_SWCG_1)) & cg_mask; // backup for later turn on after reset
  WRAP_WR32(CLK_SETCG_1, cg_mask);
  // dummy read to add latency (to wait clock turning off)
  rdata = WRAP_RD32(PMIC_WRAP_SWRST);

  // Toggle module reset
  WRAP_WR32(PMIC_WRAP_SWRST, 1);
  rdata = WRAP_RD32(WDT_SWSYSRST);
  WRAP_WR32(WDT_SWSYSRST, (rdata | (0x1 << 11)) | (0x88 << 24));
  WRAP_WR32(WDT_SWSYSRST, (rdata & (~(0x1 << 11))) | (0x88 << 24));
  WRAP_WR32(PMIC_WRAP_SWRST, 0);

  // Turn on module clock
  WRAP_WR32(CLK_CLRCG_1, backup | (1 << 20)); // ensure cg for AP is off;

  // Turn on module clock dcm (in global_con)
  // WHQA_00014186: set PMIC bclk DCM default off due to HW issue
  // WRAP_WR32(CLK_SETCG_3, (1 << 2) | (1 << 1));
  WRAP_WR32(CLK_SETCG_3, (1 << 2));

  //###############################
  // Set SPI_CK freq = 26MHz for both 6320/6323
  //###############################
  clk_sel = WRAP_RD32(CLK_SEL_0);
  WRAP_WR32(CLK_SEL_0, clk_sel | (0x3 << 24));

  //###############################
  //toggle PERI_PWRAP_BRIDGE reset
  //###############################
  //WRAP_SET_BIT(0x04,PERI_GLOBALCON_RST1);
  //WRAP_CLR_BIT(0x04,PERI_GLOBALCON_RST1);

  //###############################
  //Enable DCM
  //###############################
  WRAP_WR32(PMIC_WRAP_DCM_EN , 1);
  WRAP_WR32(PMIC_WRAP_DCM_DBC_PRD ,0);

  //###############################
  //Enable 6320 option
  //###############################
#ifdef SLV_6320
  WRAP_WR32(PMIC_WRAP_OP_TYPE , 1);
  WRAP_WR32(PMIC_WRAP_MSB_FIRST , 0);
#endif

  //###############################
  //Reset SPISLV
  //###############################
  sub_return=_pwrap_reset_spislv();
  if( sub_return != 0 )
  {
    PWRAPERR("error,_pwrap_reset_spislv fail,sub_return=%x\n",sub_return);
    return E_PWR_INIT_RESET_SPI;
  }
  //###############################
  // Enable WACS2
  //###############################
  WRAP_WR32(PMIC_WRAP_WRAP_EN,1);//enable wrap
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,8); //Only WACS2
  WRAP_WR32(PMIC_WRAP_WACS2_EN,1);

  //###############################
  // Set Dummy cycle to make it the same at both AP side and PMIC side
  //###############################
#ifdef SLV_6320
  // default value of 6320 dummy cycle is already 0x8
#else
  WRAP_WR32(PMIC_WRAP_RDDMY, 0xF);
#endif

  //###############################
  // Input data calibration flow
  //###############################
  sub_return = _pwrap_init_sistrobe();
  if( sub_return != 0 )
  {
    PWRAPERR("error,DrvPWRAP_InitSiStrobe fail,sub_return=%x\n",sub_return);
    return E_PWR_INIT_SIDLY;
  }

  //###############################
  // SPI Waveform Configuration
  //###############################
  /* 0:safe mode, 1:6MHz, 2:12MHz
   * no support 6MHz since the clock is too slow to transmit data
   * (due to RDDMY's limit -> only 4'hf)
   */
  sub_return = _pwrap_init_reg_clock(2);
  if( sub_return != 0)  {
    PWRAPERR("error,_pwrap_init_reg_clock fail,sub_return=%x\n",sub_return);
    return E_PWR_INIT_REG_CLOCK;
  }

  //###############################
  // Enable PMIC dewrapper (only for 6320)
  // (May not be necessary, depending on S/W partition)
  //###############################
#ifdef SLV_6320
  sub_return= pwrap_write_nochk(PMIC_WRP_CKPDN,   0);//set dewrap clock bit
  sub_return1=pwrap_write_nochk(PMIC_WRP_RST_CON, 0);//clear dewrap reset bit
  if(( sub_return != 0 )||( sub_return1 != 0 ))
  {
    PWRAPERR("Enable PMIC fail, sub_return=%x sub_return1=%x\n", sub_return,sub_return1);
    return E_PWR_INIT_ENABLE_PMIC;
  }
#endif

  //###############################
  // Enable DIO mode
  //###############################
  sub_return = _pwrap_init_dio(1);
  if( sub_return != 0 )
  {
    PWRAPERR("_pwrap_init_dio test error,error code=%x, sub_return=%x\n", 0x11, sub_return);
    return E_PWR_INIT_DIO;
  }

  //###############################
  // Enable Encryption
  //###############################
  sub_return = _pwrap_init_cipher();
  if( sub_return != 0 )
  {
    PWRAPERR("Enable Encryption fail, return=%x\n", sub_return);
    return E_PWR_INIT_CIPHER;
  }

  //###############################
  // Write test using WACS2
  //###############################
  sub_return = pwrap_write_nochk(DEW_WRITE_TEST, WRITE_TEST_VALUE);
  sub_return1 = pwrap_read_nochk(DEW_WRITE_TEST,&rdata);
  if(( rdata != WRITE_TEST_VALUE )||( sub_return != 0 )||( sub_return1 != 0 ))
  {
    PWRAPERR("write test error,rdata=0x%x,exp=0xa55a,sub_return=0x%x,sub_return1=0x%x\n", rdata,sub_return,sub_return1);
    return E_PWR_INIT_WRITE_TEST;
  }

  //###############################
  // Signature Checking - Using Write Test Register
  // should be the last to modify WRITE_TEST
  //###############################
  //_pwrap_wacs2_nochk(1, DEW_WRITE_TEST, 0x5678, &rdata);
  //WRAP_WR32(PMIC_WRAP_SIG_ADR,DEW_WRITE_TEST);
  //WRAP_WR32(PMIC_WRAP_SIG_VALUE,0x5678);
  //WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x1);

  //###############################
  // Signature Checking - Using CRC
  // should be the last to modify WRITE_TEST
  //###############################
  sub_return=pwrap_write_nochk(DEW_CRC_EN, 0x1);
  if( sub_return != 0 )
  {
    PWRAPERR("enable CRC fail,sub_return=%x\n", sub_return);
    return E_PWR_INIT_ENABLE_CRC;
  }
  WRAP_WR32(PMIC_WRAP_CRC_EN ,0x1);
  WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x0);
  WRAP_WR32(PMIC_WRAP_SIG_ADR , DEW_CRC_VAL);


  //###############################
  // PMIC_WRAP enables
  //###############################
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
  //WRAP_WR32(PMIC_WRAP_RRARB_EN ,0x7);
  WRAP_WR32(PMIC_WRAP_WACS0_EN,0x1);
  WRAP_WR32(PMIC_WRAP_WACS1_EN,0x1);
  //WRAP_WR32(PMIC_WRAP_WACS2_EN,0x1);//already enabled
  // WRAP_WR32(PMIC_WRAP_EVENT_IN_EN,0x1);
  //WRAP_WR32(PMIC_WRAP_EVENT_DST_EN,0xffff);
  WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0x5);  //0x1:20us,for concurrence test,MP:0x5;  //100us
  WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0xff);
  WRAP_WR32(PMIC_WRAP_WDT_UNIT,0xf);
  WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0xffffffff);
  WRAP_WR32(PMIC_WRAP_TIMER_EN,0x1);
  WRAP_WR32(PMIC_WRAP_INT_EN,0x7ffffffd); //except for [31] debug_int

  //###############################
  // GPS_INTF initialization
  //###############################
  WRAP_WR32(PMIC_WRAP_ADC_CMD_ADDR, AUXADC_CON21); // RG_GPS_RQST
  WRAP_WR32(PMIC_WRAP_PWRAP_ADC_CMD, 0x8000); // Enable AuxADC GPS request
  WRAP_WR32(PMIC_WRAP_ADC_RDY_ADDR, AUXADC_ADC12); // RG_ADC_RDY_GPS
  WRAP_WR32(PMIC_WRAP_ADC_RDATA_ADDR1, AUXADC_ADC13); // RG_ADC_OUT_GPS
  WRAP_WR32(PMIC_WRAP_ADC_RDATA_ADDR2, AUXADC_ADC14); // RG_ADC_OUT_GPS_LSB

  //###############################
  // Initialization Done
  //###############################
  WRAP_WR32(PMIC_WRAP_INIT_DONE2, 0x1);

  //###############################
  //TBD: Should be configured by MD MCU
  //###############################
  #if 1  //def CONFIG_MTK_LDVT_PMIC_WRAP
    WRAP_WR32(PMIC_WRAP_INIT_DONE0, 1);
    WRAP_WR32(PMIC_WRAP_INIT_DONE1, 1);
  #endif
  return 0;
}

/*-pwrap debug--------------------------------------------------------------------------*/
void pwrap_dump_all_register(void)
{
  U32 i = 0;
  U32 reg_addr = 0;
  U32 reg_value = 0;

  pwrap_dump_ap_register();

  PWRAPREG("dump dewrap register\n");
  for (i = 0; i < PMIC_SPISLV_REG_RANGE; i++) {
    reg_addr = (DEW_BASE + (i * 2));
    pwrap_read_nochk(reg_addr, &reg_value);
    PWRAPREG("0x%X = 0x%X\n", reg_addr, reg_value);
  }
}

void pwrap_dump_ap_register(void)
{
  U32 i = 0;
  U32 reg_addr = 0;
  U32 reg_value = 0;

  PWRAPREG("dump pwrap register\n");
  for (i = 0; i < PMIC_WRAP_REG_RANGE; i++) {
    reg_addr = (PMIC_WRAP_BASE + i * 4);
    reg_value = WRAP_RD32(reg_addr);
    PWRAPREG("0x%X = 0x%X\n", reg_addr, reg_value);
  }
  PWRAPREG("0x%X = 0x%X\n", PMIC_WRAP_SWRST, WRAP_RD32(PMIC_WRAP_SWRST));
  PWRAPREG("0x%X = 0x%X\n", PMIC_WRAP_DEBUG_SEL, WRAP_RD32(PMIC_WRAP_DEBUG_SEL));
}

S32 pwrap_init_preloader(void)
{
  u32 pwrap_ret=0,i=0;
  PWRAPFUC();

  for(i=0;i<3;i++)
  {
    pwrap_ret = pwrap_init();
    if(pwrap_ret!=0)
    {
      printf("[PMIC_WRAP]wrap_init fail,the return value=%x.\n",pwrap_ret);
    }
    else
    {
      printf("[PMIC_WRAP]wrap_init pass,the return value=%x.\n",pwrap_ret);
      break;//init pass
    }
  }

#ifdef PWRAP_PRELOADER_PORTING
  pwrap_init_for_early_porting();
#endif
  return 0;
}
#ifdef PWRAP_PRELOADER_PORTING

//--------------------------------------------------------
//    Function : _pwrap_status_update_test_porting()
// Description :only for early porting
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _pwrap_status_update_test_porting( void )
{
  U32 rdata;
  volatile U32 delay=1000*1000*1;
  PWRAPFUC();
  //disable signature interrupt
  WRAP_WR32(PMIC_WRAP_INT_EN,0x0);
  pwrap_write(DEW_WRITE_TEST, WRITE_TEST_VALUE);
  WRAP_WR32(PMIC_WRAP_SIG_ADR,DEW_WRITE_TEST);
  WRAP_WR32(PMIC_WRAP_SIG_VALUE,0xAA55);
  WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x1);

  //pwrap_delay_us(5000);//delay 5 seconds

  while(delay--);

  rdata=WRAP_RD32(PMIC_WRAP_SIG_ERRVAL);
  if( rdata != WRITE_TEST_VALUE )
  {
    PWRAPERR("%s error,error code=%x, rdata=%x\n", __FUNCTION__, 1, rdata);
    //return 1;
  }
  WRAP_WR32(PMIC_WRAP_SIG_VALUE,WRITE_TEST_VALUE);//the same as write test
  //clear sig_error interrupt flag bit
  WRAP_WR32(PMIC_WRAP_INT_CLR,1<<1);

  //enable signature interrupt
  WRAP_WR32(PMIC_WRAP_INT_EN,0x7ffffffd);
  WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x0);
  WRAP_WR32(PMIC_WRAP_SIG_ADR , DEW_CRC_VAL);
  return 0;
}

int  pwrap_init_for_early_porting(void)
{
    int ret = 0;
    U32 res=0;
    PWRAPFUC();

    ret=_pwrap_status_update_test_porting();
    if(ret==0)
    {
      PWRAPLOG("wrapper_StatusUpdateTest pass.\n");
    }
    else
    {
      PWRAPLOG("error:wrapper_StatusUpdateTest fail.\n");
      res+=1;
    }

    return ret;
}
#endif //PWRAP_PRELOADER_PORTING
