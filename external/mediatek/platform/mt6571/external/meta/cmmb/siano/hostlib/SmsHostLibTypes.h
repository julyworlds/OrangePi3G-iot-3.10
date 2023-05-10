/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 * 
 * MediaTek Inc. (C) 2010. All rights reserved.
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

/*************************************************************************/
/*                                                                       */
/* Copyright (C) 2005,2006 Siano Mobile Silicon Ltd. All rights reserved */
/*                                                                       */
/* PROPRIETARY RIGHTS of Siano Mobile Silicon are involved in the        */
/* subject matter of this material.  All manufacturing, reproduction,    */
/* use, and sales rights pertaining to this subject matter are governed  */
/* by the license agreement.  The recipient of this software implicitly  */
/* accepts the terms of the license.                                     */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

/*************************************************************************/
/*                                                                       */
/* FILE NAME                                                             */
/*                                                                       */
/*      SmsHostLibTypes.h		                                             */
/*                                                                       */
/* COMPONENT                                                             */
/*                                                                       */
/*      Siano Host Library API types and structures						 */
/*                                                                       */
/* DESCRIPTION                                                           */
/*                                                                       */
/*************************************************************************/

#ifndef _SMS_HOST_LIB_TYPES_H_
#define _SMS_HOST_LIB_TYPES_H_

#include "SmsPlatDefs.h"


#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************
*			 Macros
*************************************************************************/

#define SMSHOSTLIB_CMMB_MAX_STATS_NUM_CHANNELS				(16)
#define	SMSHOSTLIB_CMMB_STATS_ERR_HISTORY_LEN				(8)
#define SMSHOSTLIB_CMMB_MAX_NETWORK_NAME_LEN				(16)
#define SMS_CMMB_INVALID_BER								(0xFFFFFFFF)
#define SMSHOSTLIB_MAX_MODEM_VERSION_STRING_SIZE            ( 250 )	//including ASCIZ
#define SMSHOSTLIB_MAX_NUM_LOG_PARAMETERS                   (   8 )
#define SMS_CMMB_DEMUX_ANY_SUBFRAME_INDEX					(0xFF)
// A reserved network level to tell the device to use the currently tuned network
#define SMS_CMMB_NETWORK_LEVEL_USE_CURRENT_NETWORK			(0xFFFFFFFFU)	

//Control word and salt maximum sizes for F/W descrabmler 
#define SMSHOSTLIB_CMMB_CA_CONTROL_WORD_SIZE				(16)
#define SMSHOSTLIB_CMMB_CA_SALT_SIZE						(8)

//Big-card number maximum length
#define SMSHOSTLIB_CMMB_CA_CARD_NUMBER_MAX_LEN				(8)
#define SMSHOSTLIB_CMMB_CA_CARD_VERSION_MAX_LEN				(2)
#define SMS_CMMB_SMD_SN_LEN									(8)

#define SMSHOSTLIB_CMMB_INVALID_CW_ID						((UINT32)~0U)
// Win32 import/export API macro
#ifndef SMSHOSTLIB_API
	#if defined(WIN32) || defined(OS_WINCE)
		#ifdef SMSHOSTLIB_EXPORTS
			#define SMSHOSTLIB_API __declspec(dllexport)
		#else
			#define SMSHOSTLIB_API __declspec(dllimport)
		#endif
	#else
		#define SMSHOSTLIB_API
	#endif
#endif

/*************************************************************************
*			 Enums
*************************************************************************/

/// Return Codes
typedef enum
{
	SMSHOSTLIB_ERR_OK										= 0x00000000,
	SMSHOSTLIB_ERR_PENDING									= 0x00000001,
	SMSHOSTLIB_ERR_UNDEFINED_ERR							= 0x80000000,
	SMSHOSTLIB_ERR_NOT_IMPLEMENTED							= 0x80000002,
	SMSHOSTLIB_ERR_NOT_SUPPORTED							= 0x80000003,

	SMSHOSTLIB_ERR_LIB_NOT_INITIATED						= 0x80000004,
	SMSHOSTLIB_ERR_LIB_ALREADY_INITIATED					= 0x80000005,
	SMSHOSTLIB_ERR_DEVICE_NOT_INITIATED						= 0x80000006,
	SMSHOSTLIB_ERR_COMM_NOT_VALID							= 0x80000007,
	SMSHOSTLIB_ERR_TASK_CREATION_FAILED						= 0x80000008,

	SMSHOSTLIB_ERR_INVALID_ARG								= 0x80000009,
	SMSHOSTLIB_ERR_LIST_FULL								= 0x8000000A,
	SMSHOSTLIB_ERR_DAB_LIB_INIT_FAILED						= 0x8000000B,
	SMSHOSTLIB_ERR_MEM_ALLOC_FAILED							= 0x8000000C,
	SMSHOSTLIB_ERR_TIMEOUT									= 0x8000000D,
	SMSHOSTLIB_ERR_FS_ERROR									= 0x8000000E,
	SMSHOSTLIB_ERR_HANDINIT_ERROR							= 0x8000000F,
	SMSHOSTLIB_ERR_ALREADY_EXIST							= 0x80000010,
	SMSHOSTLIB_ERR_SRV_IS_PLAYING							= 0x80000011,
	SMSHOSTLIB_ERR_ONLY_ONE_SRV_ALLOWED						= 0x80000012,
	SMSHOSTLIB_ERR_ONLY_DMB_SRV_ALLOWED						= 0x80000013,
	SMSHOSTLIB_ERR_REQ_BEFORE_RESPONSE						= 0x80000014,
	SMSHOSTLIB_ERR_NOT_SUPPORTED_BY_LITE					= 0x80000015,

	SMSHOSTLIB_ERR_ENSEMBLE_NOT_EXIST						= 0x80000020,
	SMSHOSTLIB_ERR_COMPONENT_NOT_EXIST						= 0x80000021,
	SMSHOSTLIB_ERR_ALREADY_PLAYED							= 0x80000022,
	SMSHOSTLIB_ERR_PLAYING_ANOTHER_ENSEMBLE					= 0x80000023,
	SMSHOSTLIB_ERR_EXCEED_MAX_SERVICES						= 0x80000024,
	SMSHOSTLIB_ERR_NOT_SUPPORTED_TRANSMIT_MODE				= 0x80000025,
	SMSHOSTLIB_ERR_HANDLE_NOT_AVAIL							= 0x80000026,
	SMSHOSTLIB_ERR_MODEM_START_FAILED						= 0x80000027,

	SMSHOSTLIB_ERR_EXCEED_MAX_HANDLES						= 0x80000030,
	SMSHOSTLIB_ERR_HANDLE_INVALID							= 0x80000031,
	SMSHOSTLIB_ERR_MODEM_STOP_FAILED						= 0x80000032,
	SMSHOSTLIB_ERR_CA_STOP_SERVICE_FAILED					= 0x80000033,
	SMSHOSTLIB_ERR_CS_STOP_SERVICE_FAILED					= 0x80000034,

    SMSHOSTLIB_ERR_EMM_START_SERVICE_NOT_SUPPORTED			= 0x80000039,
	SMSHOSTLIB_ERR_ALREADY_ACTIVE							= 0x80000040,
	SMSHOSTLIB_ERR_NOT_ACTIVE								= 0x80000041,
	SMSHOSTLIB_ERR_SRVM_RESPONSE_TIMEOUT					= 0x80000042,
	SMSHOSTLIB_TUNE_RESPONSE_TIMEOUT						= 0x80000043,
	SMSHOSTLIB_ERR_MODEM_NOT_LOCKED							= 0x80000044,
	SMSHOSTLIB_ERR_NO_SI_ACQ								= 0x80000045,
	SMSHOSTLIB_ERR_SRVM_UNDEFINED_ERR						= 0x80000046,
	SMSHOSTLIB_ERR_ENSEMBLE_IDX_EXCEED						= 0x80000047,
	SMSHOSTLIB_ERR_SERVICE_ACTIVE							= 0x80000048,
	SMSHOSTLIB_ERR_COMM_ERR									= 0x80000049,
	SMSHOSTLIB_ERR_SCAN_ACTIVE								= 0x8000004A,
	SMSHOSTLIB_ERR_SCAN_NOT_ACTIVE							= 0x8000004B,
	SMSHOSTLIB_ERR_SERVICE_MAPPING_ACTIVE					= 0x8000004C,
	SMSHOSTLIB_ERR_SERVICE_MAPPING_ERROR					= 0x8000004D,
	SMSHOSTLIB_ERR_IP_SERVICE_IS_NOT_MAPPED					= 0x8000004E,
	SMSHOSTLIB_ERR_IP_SERVICE_DOES_NOT_EXIST				= 0x8000004F,
	SMSHOSTLIB_ERR_PID_FILTER_DOES_NOT_EXIST				= 0x80000050,
	SMSHOSTLIB_ERR_MODEM_NO_TIMESLICING_IN_TPS				= 0x80000051,
	SMSHOSTLIB_ERR_SCAN_CELL_ID_FILTER_ERROR				= 0x80000052,
	SMSHOSTLIB_ERR_SCAN_STOPPED_BY_HOST						= 0x80000054,
	SMSHOSTLIB_ERR_SERVICE_EXISTS_ON_OTHER_HANDLE			= 0X80000055,

	SMSHOSTLIB_ERR_DEVICE_DISCONNECTED						= 0x80000060,
	SMSHOSTLIB_ERR_DEVICE_DOES_NOT_EXIST					= 0x80000061,

	SMSHOSTLIB_ERR_CA_FAILURE								= 0x80000062,
	SMSHOSTLIB_ERR_DEVICE_NEED_REINIT   					= 0x80000063,

	SMSHOSTLIB_ERR_NETWORK_DOES_NOT_EXIST					= 0x80000070,
	SMSHOSTLIB_ERR_TIMESLOT_NOT_ACTIVE						= 0x80000073,
	SMSHOSTLIB_ERR_CHANNEL_DOES_NOT_EXIST					= 0x80000074,
	SMSHOSTLIB_ERR_EXCEED_MAX_CHANNELS						= 0x80000075,
	SMSHOSTLIB_ERR_SERVICE_DOES_NOT_EXIST					= 0x80000076,
	SMSHOSTLIB_ERR_FREQ_POINT_DOES_NOT_EXIST				= 0x80000077,
	SMSHOSTLIB_ERR_PLAYING_ANOTHER_NETWORK					= 0x80000078,
	SMSHOSTLIB_ERR_EXCEEDED_ALLOCATED_MEMORY				= 0x80000079,
	SMSHOSTLIB_ERR_DATA_CORRUPTED							= 0x8000007A,
	SMSHOSTLIB_ERR_BOTH_SYNC_ASYNC							= 0x8000007B,
	SMSHOSTLIB_ERR_ILLEGAL_DATA								= 0x8000007C,
	SMSHOSTLIB_ERR_PLAYING_ANOTHER_SERVICE					= 0x8000007d,
	SMSHOSTLIB_ERR_ALREADY_SET								= 0x8000007e,
	SMSHOSTLIB_ERR_INJECT_TABLES_FAILED						= 0x8000007f,
	SMSHOSTLIB_ERR_PLAYING_ANOTHER_FREQUENCY				= 0x80000080,
	SMSHOSTLIB_ERR_SERVICE_NOT_ACTIVE						= 0x80000081,
	SMSHOSTLIB_ERR_EXCEED_MAX_CLIENTS						= 0x80000082,
	SMSHOSTLIB_ERR_OUT_OF_BOUNDS							= 0x80000083,
	SMSHOSTLIB_ERR_ILLEGAL_DATA2							= 0x80000084,
	SMSHOSTLIB_ERR_ILLEGAL_DATA3							= 0x80000085,
	SMSHOSTLIB_ERR_INCOMPLETE_FRAME							= 0x80000086,
	SMSHOSTLIB_ERR_NOT_INITIALIZED							= 0x80000087,
	SMSHOSTLIB_ERR_ALREADY_INITIALIZED						= 0x80000088,
	SMSHOSTLIB_ERR_FAILED_CRC_CHECK							= 0x80000089,
	SMSHOSTLIB_ERR_DATA_SCRAMBLED							= 0x80000090,
	SMSHOSTLIB_ERR_NO_SIGNAL								= 0x80000091,
	SMSHOSTLIB_ERR_DESCRAMBLING_FAILURE						= 0x80000092,
	SMSHOSTLIB_ERR_PARSING_FAILURE							= 0x80000093,

	SMSHOSTLIB_ERR_SRVM_GENERAL								= 0x80000100,

	SMSHOSTLIB_ERR_SRVM_PAT_TABLE_READ_ERROR				= 0x80000120,
	SMSHOSTLIB_ERR_SRVM_PAT_TABLE_READ_TIMEOUT				= 0x80000121,

	SMSHOSTLIB_ERR_SRVM_NIT_TABLE_READ_ERROR				= 0x80000122,
	SMSHOSTLIB_ERR_SRVM_NIT_TABLE_READ_TIMEOUT				= 0x80000123,

	SMSHOSTLIB_ERR_SRVM_SDT_TABLE_READ_ERROR				= 0x80000124,
	SMSHOSTLIB_ERR_SRVM_SDT_TABLE_READ_TIMEOUT				= 0x80000125,

	SMSHOSTLIB_ERR_SRVM_INT_TABLE_READ_ERROR				= 0x80000126,
	SMSHOSTLIB_ERR_SRVM_INT_TABLE_READ_TIMEOUT				= 0x80000127,

	SMSHOSTLIB_ERR_SRVM_INT_PMT_TABLE_READ_ERROR			= 0x80000128,
	SMSHOSTLIB_ERR_SRVM_INT_PMT_TABLE_READ_TIMEOUT			= 0x80000129,

	SMSHOSTLIB_ERR_SRVM_MPE_PMT_TABLE_READ_ERROR			= 0x80000130,
	SMSHOSTLIB_ERR_SRVM_MPE_PMT_TABLE_READ_TIMEOUT			= 0x80000131,

	SMSHOSTLIB_ERR_SRVM_NO_LINKAGE_DESC_IN_NIT				= 0x80000132,
	SMSHOSTLIB_ERR_SRVM_NO_PLATFORMS_IN_NIT					= 0x80000133,
	SMSHOSTLIB_ERR_SRVM_NO_VALID_PLATFORMS_IN_NIT			= 0x80000134,

	SMSHOSTLIB_ERR_SRVM_NO_VALID_INT_PID					= 0x80000135,
	SMSHOSTLIB_ERR_SRVM_NO_VALID_IP_SERVICES				= 0x80000136,

	SMSHOSTLIB_ERR_SRVM_NETWORK_ID_NOT_IN_FILTER			= 0x80000137,
	SMSHOSTLIB_ERR_SRVM_PLATFORM_ID_NOT_IN_FILTER			= 0x80000138,
	SMSHOSTLIB_ERR_SRVM_TS_VALIDATION_ERR					= 0x80000139,

	SMSHOSTLIB_ERR_SRVM_MEMORY_ALLOCATION					= 0x80000140,

	SMSHOSTLIB_ERR_REMOTE_HOST_NOT_FOUND					= 0x80000141,
	SMSHOSTLIB_ERR_REMOTE_SOCKET_NOT_FOUND					= 0x80000142,

	SMSHOSTLIB_ERR_UNSUPPORTED_CRYSTAL						= 0x80000143,

	SMSHOSTLIB_ERR_COMM_NOT_INITIALIZED						= 0x80000144,
	SMSHOSTLIB_ERR_DATA_LOSS_DEVICE							= 0x80000145,

	SMSHOSTLIB_ERR_ESG_BAD_DATA     						= 0x80000200,
	SMSHOSTLIB_ERR_ESG_IN_PROGRESS							= 0x80000201,
	SMSHOSTLIB_ERR_ESG_NOT_EXIST      						= 0x80000202,
	SMSHOSTLIB_ERR_ESG_STOPPED   							= 0x80000203,
	SMSHOSTLIB_ERR_ESG_TABLE_NOT_FOUND						= 0x80000204,
	

	SMSHOSTLIB_ERR_CA_MPX_PARSING_ERROR						= 0x80000300,
	SMSHOSTLIB_ERR_CA_DATA_PARSING_ERROR					= 0x80000301,
	SMSHOSTLIB_ERR_CA_DEVICE_ERROR							= 0x80000302,
	SMSHOSTLIB_ERR_CA_CARD_DATA_ERROR						= 0x80000303,
	SMSHOSTLIB_ERR_CA_CARD_MEM_ERROR						= 0x80000304,
	SMSHOSTLIB_ERR_CA_CARD_NO_AUTH							= 0x80000305,
	SMSHOSTLIB_ERR_CA_CARD_NOMATCH_CAS						= 0x80000306,
	SMSHOSTLIB_ERR_CA_CARD_NO_ECM							= 0x80000307,
	SMSHOSTLIB_ERR_CA_CARD_EMM_PROCESSING_ERROR				= 0x80000308,
	SMSHOSTLIB_ERR_CA_CARD_AUTO_TO_EXPIRE					= 0x80000309,
	SMSHOSTLIB_ERR_CA_CARD_FORBIDDEN						= 0x8000030A,
	SMSHOSTLIB_ERR_CA_NO_AUTH_TABLE				        	= 0x8000030B,
	SMSHOSTLIB_ERR_CA_MAX_CA_SERVICES				       	= 0x8000030C,
	SMSHOSTLIB_ERR_CA_NOT_INITIALIZED			        	= 0x8000030D,
	SMSHOSTLIB_ERR_CA_DEVICE_OPEN_FAILED		        	= 0x8000030E,
	SMSHOSTLIB_ERR_CA_SVC_NOT_OPENED			        	= 0x8000030F,
	SMSHOSTLIB_ERR_CA_NOT_SCRAMBLED			        		= 0x80000310,
	SMSHOSTLIB_ERR_CA_AUTH_TABLE_TOO_BIG		        	= 0x80000311,

	SMSHOSTLIB_ERR_RDS_GROUP_NOT_VALID			        	= 0x80000400,	
	SMSHOSTLIB_ERR_RDS_WRONG_POINTER			        	= 0x80000401,
	SMSHOSTLIB_ERR_TMC_WRONG_EVENT				        	= 0x80000402,
	SMSHOSTLIB_ERR_BUFFER_OVERFLOW				        	= 0x80000403,
	SMSHOSTLIB_ERR_NO_PARSING					        	= 0x80000404,
	SMSHOSTLIB_ERR_WRONG_BLOCK_PARSING			        	= 0x80000405,
	SMSHOSTLIB_ERR_FWDL_TIMEOUT								= 0x80000406

} SMSHOSTLIB_ERR_CODES_E;

typedef enum
{
	SMSHOSTLIB_DEVMD_DVBT,
	SMSHOSTLIB_DEVMD_DVBH,
	SMSHOSTLIB_DEVMD_DAB_TDMB,
	SMSHOSTLIB_DEVMD_DAB_TDMB_DABIP,
	SMSHOSTLIB_DEVMD_DVBT_BDA,
	SMSHOSTLIB_DEVMD_ISDBT,
	SMSHOSTLIB_DEVMD_ISDBT_BDA,
	SMSHOSTLIB_DEVMD_CMMB,
	SMSHOSTLIB_DEVMD_RAW_TUNER,
	SMSHOSTLIB_DEVMD_FM_RADIO,
	SMSHOSTLIB_DEVMD_FM_RADIO_BDA,
	SMSHOSTLIB_DEVMD_NONE = 0xFFFFFFFF

} SMSHOSTLIB_DEVICE_MODES_E;

/// Message type responses to APIs requests. The enum is analog to the APIs names
typedef enum
{
	// Initialization and Termination
	SMSHOSTLIB_MSG_INIT_DEVICE_RES = 0,				//!< Response to #SmsHostApiDeviceInit_Req
	SMSHOSTLIB_MSG_DEVICE_TERMINATE_RES,			//!< Response to #SmsHostApiDeviceTerminate_Req

	// Information and Statistics
	SMSHOSTLIB_MSG_GET_VERSION_RES = 1000,			//!< Response to #SmsHostApiGetVersion_Req
	SMSHOSTLIB_MSG_GET_STATISTICS_RES,				//!< Response to #SmsHostApiGetStatistics_Req
	SMSHOSTLIB_MSG_GET_RECEPTION_RES,				//!< Response to #SmsHostApiGetReceptionQuality_Req
	SMSHOSTLIB_MSG_GET_STATISTICS_EX_RES,			//!< Response to SmsHostApiGetStatisticsEx_Req and SmsLiteMsGetStatistics_Req
													//!< Go to the specific request declaration for format of the response structure
	// Tune and Scan operations
	SMSHOSTLIB_MSG_TUNE_RES        = 2000,			//!< Response to #SmsHostApiTune_Req
	SMSHOSTLIB_MSG_SCAN_START_RES,					//!< Response to #SmsHostApiScanStart_Req
	SMSHOSTLIB_MSG_SCAN_STOP_RES,					//!< Response to #SmsHostApiScanStop_Req
	SMSHOSTLIB_MSG_ISDBT_TUNE_RES,					//!< Response to #SmsHostApiIsdbtTune_Req
	SMSHOSTLIB_MSG_SCAN_FILTER_LIST_ADD_RES,		//!< Response to #SmsHostApiScanFilterListAdd_Req

	// DVB-T
	SMSHOSTLIB_MSG_ADD_PID_FILTER_RES = 3000,		//!< Response to #SmsHostApiAddPidFilter_Req
	SMSHOSTLIB_MSG_REMOVE_PID_FILTER_RES,			//!< Response to #SmsHostApiRemovePidFilter_Req
	SMSHOSTLIB_MSG_RETRIEVE_PID_FILTER_LIST_RES,	//!< Response to #SmsHostApiRetrievePidFilterList_Req
	SMSHOSTLIB_MSG_GET_PID_FILTER_LIST_RES,			//!< Obsolete
	SMSHOSTLIN_MSG_GET_SI_TBL_RES,					//!< Response to #SmsHostApiDvbGetSITable_Req

	// DVB-H
	SMSHOSTLIB_MSG_ADD_IP_FILTER_RES = 4000,		//!< Response to #SmsHostApiAddIpFilter_Req
	SMSHOSTLIB_MSG_REMOVE_IP_FILTER_RES,			//!< Response to #SmsHostApiRemoveIpFilter_Req
	SMSHOSTLIB_MSG_GET_IP_FILTER_LIST_RES,			//!< Response to #SmsHostApiGetIpFilterList_Req
	SMSHOSTLIB_MSG_GET_IPS_INT_INFO_RES,			//!< Placeholder for internal API
	SMSHOSTLIB_MSG_GET_AVAIL_IP_PLATFORMS_RES,		//!< Response to #SmsHostApiGetAvailableIpPlatforms_Req
	SMSHOSTLIB_MSG_UTC_TIME_RES,					//!< Response to #SmsHostApiDvbTDT_Req
	SMSHOSTLIB_MSG_TIME_OFFSET_RES,					//!< Response to #SmsHostApiDvbTOT_Req
	SMSHOSTLIB_MSG_SET_OOC_TIMER_RES,				//!< Response to #SmsHostApiSetOOCTimer_Req
	SMSHOSTLIB_MSG_SET_SLEEP_MODE_RES,				//!< Response to #SmsHostApiSetSleepMode_Req
	SMSHOSTLIB_MSG_PRESET_BW_RES,					//!< Response to #SmsHostApiPresetBandwidth_Req

	// DAB, T-DMB and eDAB
	SMSHOSTLIB_MSG_START_SERVICE_RES = 5000,		//!< Response to #SmsHostApiDabStartService_Req
	SMSHOSTLIB_MSG_STOP_SERVICE_RES,				//!< Response to #SmsHostApiDabStopService_Req
	SMSHOSTLIB_MSG_GET_ENSEMBLE_INFO_RES,			//!< Response to #SmsHostApiDabGetEnsembleInfo_Req
	SMSHOSTLIB_MSG_GET_SERVICES_RES,				//!< Response to #SmsHostApiDabGetServices_Req
	SMSHOSTLIB_MSG_GET_COMPONENTS_INFO_RES,			//!< Response to #SmsHostApiDabGetComponentsInfo_Req
	SMSHOSTLIB_GET_CURR_ENSEMBLE_INDEX_RES,			//!< Response to #SmsHostApiDabGetCurrentEnsembleIndex_Req
	SMSHOSTLIB_MSG_POST_VITERBI_BER_STATE_SET_RES,	//!< Obsolete
	SMSHOSTLIB_MSG_GET_COMBINED_COMPONENTS_INFO_RES,//!< Response to #SmsHostApiDabGetCombinedComponentsInfo_Req
	SMSHOSTLIB_MSG_GET_SUBCHANNEL_INFO_RES,			//!< Placeholder for future API

	// GPIO
	SMSHOSTLIB_MSG_GPIO_CONFIGURE_RES = 6000,		//!< Response to #SmsHostApiGPIOConfigure_Req
	SMSHOSTLIB_MSG_GPIO_SET_LEVEL_RES,				//!< Response to #SmsHostApiGPIOSetLevel_Req
	SMSHOSTLIB_MSG_GPIO_GET_LEVEL_RES,				//!< Response to #SmsHostApiGPIOGetLevel_Req
	SMSHOSTLIB_MSG_PWM_CFG_RES,						//!< Response to #SmsHostApiPwmConfigure_Req

	// Raw tuner mode
	SMSHOSTLIB_MSG_RAW_CAPTURE_RES = 7000,			//!< Response to #SmsHostApiRawCapture_Req
	SMSHOSTLIB_MSG_RAW_CAPTURE_ABORT_RES,			//!< Response to #SmsHostApiRawCaptureAbort_Req

	// Debug and testing
	SMSHOSTLIB_MSG_SMS_LOOPBACK_RES	= 7100,			//!< Response for loopback test event
	SMSHOSTLIB_MSG_SMS_DATA_PUMP_RES,				//!< Response for data pump request
	SMSHOSTLIB_MSG_SMS_EXEC_TEST_1_RES,				//!< Response for USB IN EP set request
	SMSHOSTLIB_MSG_GET_USER_INFO_RES,			    //!< Response to #SmsHostApiCmmbSendUserInfo_Req


	// CMMB
	SMSHOSTLIB_MSG_CMMB_START_SERVICE_RES		= 8000,	//!< Response to #SmsHostApiCmmbStartService_Req
													//   Payload format:
													//		UINT32 Service handle 
													//		UINT32 Sub-frame index 
													//		UINT32 Service ID 
	SMSHOSTLIB_MSG_CMMB_STOP_SERVICE_RES		= 8001,	//!< Response to #SmsHostApiCmmbStopService_Req
	SMSHOSTLIB_MSG_CMMB_START_CONTROL_INFO_RES	= 8002,	//!< Response to #SmsHostApiCmmbStartControlInfo_Req
	SMSHOSTLIB_MSG_CMMB_START_TS0_RES = SMSHOSTLIB_MSG_CMMB_START_CONTROL_INFO_RES,	//!< Response to #SmsLiteCmmbStartTs0_Req
	SMSHOSTLIB_MSG_CMMB_STOP_CONTROL_INFO_RES	= 8003,	//!< Response to #SmsHostApiCmmbStopControlInfo_Req
	SMSHOSTLIB_MSG_CMMB_STOP_TS0_RES = SMSHOSTLIB_MSG_CMMB_STOP_CONTROL_INFO_RES,	//!< Response to #SmsLiteCmmbStopTs0_Req


	SMSHOSTLIB_MSG_CMMB_GET_AVAIL_NETWORKS_RES	= 8004,		//!< Response to #SmsHostApiCmmbGetAvailableNetworks_Req
	SMSHOSTLIB_MSG_CMMB_ADD_CHANNEL_FILTER_RES	= 8005,	//!< Response to SmsHostApiCmmbAddChannelFilter_Req
	SMSHOSTLIB_MSG_CMMB_REMOVE_CHANNEL_FILTER_RES = 8006,	//!< Response to SmsHostApiCmmbRemoveChannelFilter_Req
	SMSHOSTLIB_MSG_CMMB_ESG_START_RES			= 8007,	//!< Response to #SmsHostApiCmmbEsgStart_Req
	SMSHOSTLIB_MSG_CMMB_ESG_STOP_RES			= 8008, //!< Response to #SmsHostApiCmmbEsgStop_Req
	SMSHOSTLIB_MSG_CMMB_ESG_COMPLETED_IND		= 8009, //!< Indication that the ESG parsing has completed
	SMSHOSTLIB_MSG_CMMB_SET_OUTPUT_FORMAT_RES	= 8010, //!< Response to #SmsHostApiCmmbSetOutputFormat_Req
	SMSHOSTLIB_MSG_CMMB_GET_CONTROL_TABLE_OBSOLETE = 8011, //!< Obsolete
	SMSHOSTLIB_MSG_CMMB_HOST_NOTIFICATION_IND	= 8012, //!< CMMB notification to host (e.g. emergency data changed)
	SMSHOSTLIB_MSG_ASYNC_ERR_IND				= 8013, //!< Something is wrong with CA service.
	SMSHOSTLIB_MSG_CMMB_GET_NETWORK_OF_FREQ_RES_OBSOLETE = 8014, //!< Obsolete
	SMSHOSTLIB_MSG_CMMB_MRC_STATISTICS_RES		= 8015, //!< Response to #SmsHostApiCmmbGetMrcStatistics_Req
	SMSHOSTLIB_MSG_SET_PERIODIC_STATISTICS_RES	= 8016, //!< Response to #SmsHostApiCmmbSetPeriodicStatistics_Req
	SMSHOSTLIB_MSG_CMMB_SET_AUTO_OUTPUT_TS0_RES	= 8017, //!< Response to #SmsHostApiCmmbSetAutoOuputTs0_Req
	SMSHOSTLIB_MSG_CMMB_GET_SHORT_STATISTICS_RES= 8018, //!< Response to #SmsHostApiCmmbGetShortStatistics_Req
	SMSHOSTLIB_MSG_CMMB_SMD_SN_RES				= 8019, //!< Response to #SmsLiteCmmbSmdSn_Req
														//!< Payload format:
														//!<	UINT8 pSmdSn[8]
														//!< An array of 8 bytes containing the SMD serial number.
														
	SMSHOSTLIB_MSG_CMMB_SET_CA_CW_RES           = 8020, //!< Response to #SmsLiteCmmbSetCaControlWords_Req
	SMSHOSTLIB_MSG_CMMB_SET_CA_SALT_RES         = 8021, //!< Response to #SmsLiteCmmbSetCaSaltKeys_Req
	SMSHOSTLIB_MSG_CMMB_CA_SERVICE_IND			= 8022, //!< Indication related to a scrambled service which was 
														//   recently opened. 
														//   Payload format:
														//		UINT32 Indication type - according to #SmsCaServiceIndicationTypes_EN
														//		UINT32 Service ID

	// ISDBT
	SMSHOSTLIB_MSG_SET_AES128_KEY_RES,				//!< Response to SmsHostApiDeviceSetAES128Key_Req

	// Indications (Unsolicited events)

	// Scan
	SMSHOSTLIB_SCAN_PROGRESS_IND	    = 10000,	//!< Indicates that a frequency had been scanned
	SMSHOSTLIB_SCAN_COMPLETE_IND,					//!< Indicates that the scanned has been completed

	// Service Information
	SMSHOSTLIB_DVB_SI_CHANGE_IND	    = 11000,		//!< Indicates that current SI has been changed
	SMSHOSTLIB_DAB_RECONFIG_IN_PROG_IND = 11100,	//!< Indicates that DAB re-configuration is taking place
	SMSHOSTLIB_DAB_ENSEMBLE_CHANGED_IND,			//!< Indicates that DAB current ensemble has been changed
	SMSHOSTLIB_MSG_FAST_INFORMATION_BLOCK_IND,		//!< Indication contains raw FIC data

	// Handover
	SMSHOSTLIB_HANDOVER_IND	= 12000,				//!< Indicates that DVB-H Handover has been performed
	SMSHOSTLIB_RECEPTION_STATUS_IND,				//!< Indicates the current reception status
	SMSHOSTLIB_INCONSISTENT_DATABASE_IND,			//!< Indicates inconsistency between the internal database and the received psi/si

	// Log
	SMSHOSTLIB_MSG_LOG_ITEM_IND = 13000,			//!< Forwarding of an asynchronous Log message from SMS1000
	SMSHOSTLIB_MSG_LOG_STRING_IND,					//!< Forwarding of a log string from Siano Host Control Library. SMSHOSTLIB_MSG_LOG_STRING_IND data is a null terminated string. The PayloadLen value is up to SMSHOSTLIB_MAX_LOG_STRING_SIZE
	// Other
	SMSHOSTLIB_MSG_UART_FOR_INFRARED_IND	= 14000,	//!< Notification to host with data received from UART
	SMSHOSTLIB_MSG_COMM_STATE_CHANGED_IND,			//!< Notification to host that the current communication state has changed
	SMSHOSTLIB_MSG_DEVICE_UNPLUGGED_IND,			//!< Notification that a device has been unplugged (PNP)
	SMSHOSTLIB_MSG_DEVICE_NEW_DEVICE_DETECTED_IND,	//!< Notification that a new device was detected (PNP)
	SMSHOSTLIB_MSG_RAW_CAPTURE_COMPLETE_IND,		//!< Notification that a raw capture operation has been completed
	// CAS
	SMSHOSTLIB_DAB_CAS_CHANNEL_EVENT_IND,			//!< Notification of CAS channel event
	SMSHOSTLIB_DAB_CAS_APPLICATION_EVENT_IND,		//!< Notification of CAS application event
	SMSHOSTLIB_DAB_CAS_DESCRAMBLER_ERROR_EVENT_IND,	//!< Notification of CAS De scrambler error

	// Debug and testing
	SMSHOSTLIB_MSG_SMS_DATA_PUMP_IND,				//!< Notification of data pump event

	// Tune Indications
	SMSHOSTLIB_MSG_SMS_NO_SIGNAL_IND,				//!< Notification of No Signal		  - after RFTuneReq in DVBT or after IsdbtTune_Req in ISDBT
	SMSHOSTLIB_MSG_SMS_SIGNAL_DETECTED_IND,			//!< Notification of Signal detection - after RFTuneReq in DVBT or after IsdbtTune_Req in ISDBT

	// Infrared interface
	SMSHOSTLIB_MSG_INFRARED_ACTIVATE_RES,			//!< Response for interface diagnostics request
	SMSHOSTLIB_MSG_INFRARED_SAMPLES_IND,			//!< Indication on interface diagnostics

	// MRC Slave shutdown and bringup
	SMSHOSTLIB_MSG_MRC_SLAVE_SHUTDOWN_RES,			//!< Response for MRC shutdown request
	SMSHOSTLIB_MSG_MRC_SLAVE_BRINGUP_RES,			//!< Response for MRC shutdown request

	SMSHOSTLIB_MSG_DATA_LOSS_IND,					//!< Notification that a data packet has been lost

	// Interface data rate tests
	SMSHOSTLIB_MSG_INTERFACE_DATA_RATE_TEST_START_RES,	//!< Response for interface data rate test start request
	SMSHOSTLIB_MSG_INTERFACE_DATA_RATE_TEST_STOP_RES,	//!< Response for interface data rate test stop request

	// Smart card access
	SMSHOSTLIB_MSG_SMART_CARD_RX_IND		= 16000,	//!< Response from the on-chip UAM. The payload contains the response
														//!< buffer from the UAM.
	SMSHOSTLIB_MSG_MBBMS_RX_IND				= 16001,			//!< Response for an MBBMS protocol message

	SMSHOSTLIB_MSG_NSCD_INIT_RES= 20000,
	SMSHOSTLIB_MSG_NSCD_OPEN_SESSION_RES,
	SMSHOSTLIB_MSG_NSCD_CA_PROCESS_SECTION_RES,
	SMSHOSTLIB_MSG_DBD_CREATE_OBJECT_RES,
	SMSHOSTLIB_MSG_DBD_CONFIGURE_RES,
	SMSHOSTLIB_MSG_DBD_SET_KEYS_RES,
	SMSHOSTLIB_MSG_DBD_PROCESS_RES,

	SMSHOSTLIB_MSG_FM_RADIO_BLOCK_IND		= 22000,



	SMSHOSTLIB_MSG_INVALID_RESPONSE_VAL = ~0U,				//!< A value not for use - indicates a response that
														//!< will never appear
} SMSHOSTLIB_MSG_TYPE_RES_E;

typedef enum SMSHOSTLIB_DVB_MODEM_STATE_E
{
	MS_WAKE_UP					= 1,
	MS_AGC_ACQUISITION			= 2,
	MS_ACQUIRE_LOCK				= 3,
	MS_PRE_SLICE				= 5,
	MS_IN_SLICE					= 6,
	MS_POST_SLICE				= 7,
	MS_IQ_AMBIG					= 8,
	MS_SYR_FIRST_SYNC			= 9,
	MS_CHECK_COARSE_FREQ		= 10,
	MS_HANDLE_IFFT				= 11,
	MS_PREPARE_TO_HANDLE_IFFT	= 12,
	MS_HANDLE_IFFT_IN_SLICE		= 13,
	MS_HO_SCAN_PHASE			= 14,
	MS_DEBUG_STATE				= 15,
	MS_WAIT_AFTER_CHC_SET		= 16,
	MS_SET_ICI_AFTER_IFFT		= 17,
	MS_SET_CHC_AFTER_IFFT		= 18,
	MS_PREPARE_POST_SLICE		= 19,
	MS_HANDLE_IDFT_IN_SLICE		= 20,
	MS_WAIT_AFTER_CHC_IN_SLICE	= 21

} SMSHOSTLIB_DVB_MODEM_STATE_ET;

typedef enum SMSHOSTLIB_CODE_RATE_E
{
	CR_1_2	= 0,
	CR_2_3	= 1,
	CR_3_4	= 2,
	CR_5_6	= 3,
	CR_7_8	= 4

} SMSHOSTLIB_CODE_RATE_ET;

typedef enum SMSHOSTLIB_GUARD_INTERVALS_E
{
	GI_1_32	= 0,
	GI_1_16	= 1,
	GI_1_8	= 2,
	GI_1_4	= 3

} SMSHOSTLIB_GUARD_INTERVALS_ET;

typedef enum SMSHOSTLIB_HIERARCHY_E
{
	HR_NONE		= 0,
	HR_ALPHA_1	= 1,
	HR_ALPHA_2	= 2,
	HR_ALPHA_4	= 3

} SMSHOSTLIB_HIERARCHY_ET;

typedef enum SMSHOSTLIB_CONSTELLATION_E
{
	CN_QPSK		= 0,
	CN_16_QAM	= 1,
	CN_64_QAM	= 2,
	CN_DQPSK	= 3

} SMSHOSTLIB_CONSTELLATION_ET;

typedef enum SMSHOSTLIB_FREQ_BANDWIDTH_E
{
	BW_8_MHZ		= 0,
	BW_7_MHZ		= 1,
	BW_6_MHZ		= 2,
	BW_5_MHZ		= 3,
	BW_ISDBT_1SEG	= 4,
	BW_ISDBT_3SEG	= 5,
	BW_2_MHZ		= 6,
	BW_FM_RADIO		= 7,
	BW_1_5_MHZ		= 15,
	BW_UNKNOWN		= 0xFFFF

} SMSHOSTLIB_FREQ_BANDWIDTH_ET;

//! Indicator for Scan operation
typedef enum
{
	SCAN_PARAM_TABLE,								//!< The scan range is given by the structure #SMSHOSTLIB_SCAN_TABLE_ST
	SCAN_PARAM_RANGE								//!< The scan range is given by the structure #SMSHOSTLIB_SCAN_RANGE_ST
} SMSHOSTLIB_SCAN_PARAM_STD_ET;

typedef enum
/*! Reception Quality result ( Scale range: 5 (best) to 0 (no reception at all) ) */
{
	RECEPTION_QUALITY_UNKNOWN,
	RECEPTION_QUALITY_FAIL,
	RECEPTION_QUALITY_POOR,
	RECEPTION_QUALITY_AVERAGE,
	RECEPTION_QUALITY_GOOD,
	RECEPTION_QUALITY_EXCELLENT

} SMSHOSTLIB_RECEPTION_QUALITY_ET;

typedef enum
{
	SMS_CMMB_HOST_NOTIFICATION_TYPE_NONE						= 0,
	SMS_CMMB_HOST_NOTIFICATION_TYPE_CONTROL_INFO_CHANGED		= 1<<0,
	SMS_CMMB_HOST_NOTIFICATION_TYPE_EMERGENCY_BCAST_CHANGED		= 1<<1,
	SMS_CMMB_HOST_NOTIFICATION_TYPE_ESG_CHANGED					= 1<<2,
} SMSHOSTLIB_CMMB_HOST_NOTIFICATION_TYPE_ET;


// The following indications types arrive with the control message #SMSHOSTLIB_MSG_CMMB_CA_SERVICE_IND
// The indications are given when data of a scrambled service is received. They provide information and problems
// related to the keys needed for descrambling the stream.
typedef enum SmsCaServiceIndicationTypes_E
{
	SMS_CA_IND_INVALID = 0,

	// The following 2 types relate to access to a service.
	// These messages are relevant only to products with direct connectivity to a Nagra SMD.

	// Access denied - The user has to purchase the service
	SMS_CA_IND_ACCESS_DENIED = 1,	

	// The following 3 types are related to the internal firmware descrambler.
	// They are relevant to following systems:
	// * Systems with direct connectivity to Nagra SMD 
	// * MBBMS systems which use an 1186 and use the firmware descrambler
	
	// Exceeded max services - The internal descrambler supports maximum 2 concurrent 
	// scrambled services.
	SMS_CA_IND_EXCEEDED_MAX_SCRAMBLED_SERVICES = 3,	

	// Salt keys not set - The application did not provide the salt keys.
	// The application has to set salt keys once each time it opens a scrambled service.
	SMS_CA_IND_SALT_KEYS_NOT_SET = 4,	

	// CW Not set - The application did not provide any CW for the service. 
	SMS_CA_IND_CW_NOT_SET = 5,			
	// CW Missing - The descrambler did not find the CW needed to descramble
	// the current audio/video samples. It means that either the application neglected to 
	// provide one of the CWs, or that the stream has a problem.
	SMS_CA_IND_CW_MISSING_KEY = 6,		

	SMS_CA_IND_LAST,
} SmsCaServiceIndicationTypes_EN;

/*************************************************************************
*			 Structs
*************************************************************************/

//! CMMB per channel statistics
typedef struct SMSHOSTLIB_CMMB_CHANNEL_STATS_S
{
	UINT32	Id;
	UINT32	RsNumTotalBytes;
	UINT32	PostLdpcBadBytes;
	UINT32	LdpcCycleCountAvg;
	UINT32	LdpcNonConvergedWordsCount;
	UINT32	LdpcWordsCount;
	UINT32	RsNumTotalRows;
	UINT32	RsNumBadRows;
	UINT32	NumGoodFrameHeaders;
	UINT32	NumBadFrameHeaders;
	UINT32	Constellation;
	UINT16	DataServicesNumRsRows;
	UINT16	DataServicesNumBadRows;
	UINT32	Reserved1;
	UINT32	Reserved2;
} SMSHOSTLIB_CMMB_CHANNEL_STATS_ST;

//! CMMB FW statistics 
typedef struct SMSHOSTLIB_STATISTICS_CMMB_S
{
	UINT32	Reserved1;				//!< Reserved field. 
	UINT32	Reserved2;				//!< Reserved field. 

	// Common parameters
	UINT32	IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32	IsDemodLocked;			//!< 0 - not locked, 1 - locked
	UINT16	IsExternalAntennaOn;		//!< 0 - external LNA off, 1 - external LNA on
	UINT16	IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	// Reception quality
	INT32	SNR;					//!< dB
	INT32	RSSI;					//!< dBm
	UINT32	ErrorsCounter;			//!< Firmware errors
	INT32	InBandPwr;				//!< In band power in dBM
	INT32	CarrierOffset;			//!< Carrier Offset in Hz
	UINT32	BER;                     //!< BER * 10^-6

	// Transmission parameters
	UINT32	Frequency;				//!< Frequency in Hz
	UINT32	Reserved3;				//!< Reserved field. 

	UINT8	ModemState;				//!< 0 - Acquisition, 1 - Locked
	UINT8	ReceptionQuality;		//!< //!< Signal quality on a scale 0-5
	UINT16	Reserved6;				//!< Reserved field. 
	

	// Per Channel Information
	UINT32	NumActiveChannels;		//!< Number of channels - including control
	UINT8	ErrorsHistory[SMSHOSTLIB_CMMB_STATS_ERR_HISTORY_LEN];

	SMSHOSTLIB_CMMB_CHANNEL_STATS_ST	ChannelsStatsArr[SMSHOSTLIB_CMMB_MAX_STATS_NUM_CHANNELS];		//!< Channels info
} SMSHOSTLIB_STATISTICS_CMMB_ST;

//! CMMB concise statistics 
typedef struct SMSHOSTLIB_CMMB_SHORT_STATISTICS_S
{
	UINT32 IsRfLocked;
	UINT32 IsDemodLocked;
	INT32  InBandPower;
	UINT32 SNR;
	UINT32 Frequency;
	UINT32 ReceptionQuality;
	UINT32 Ber;
	UINT32 RsTotalRows;
	UINT32 RsBadRows;
	UINT32 LdpcTotalWords;
	UINT32 LdpcBadWords;
} SMSHOSTLIB_CMMB_SHORT_STATISTICS_ST;


//! DVBT Statistics
typedef struct TRANSMISSION_STATISTICS_S
{
	UINT32 Frequency;				//!< Frequency in Hz
	UINT32 Bandwidth;				//!< Bandwidth in MHz
	UINT32 TransmissionMode;		//!< FFT mode carriers in Kilos
	UINT32 GuardInterval;			//!< Guard Interval from SMSHOSTLIB_GUARD_INTERVALS_ET
	UINT32 CodeRate;				//!< Code Rate from SMSHOSTLIB_CODE_RATE_ET
	UINT32 LPCodeRate;				//!< Low Priority Code Rate from SMSHOSTLIB_CODE_RATE_ET
	UINT32 Hierarchy;				//!< Hierarchy from SMSHOSTLIB_HIERARCHY_ET
	UINT32 Constellation;			//!< Constellation from SMSHOSTLIB_CONSTELLATION_ET

	// DVB-H TPS parameters
	UINT32 CellId;					//!< TPS Cell ID in bits 15..0, bits 31..16 zero; if set to 0xFFFFFFFF cell_id not yet recovered
	UINT32 DvbhSrvIndHP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	UINT32 DvbhSrvIndLP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked

}TRANSMISSION_STATISTICS_ST;

typedef struct RECEPTION_STATISTICS_S
{
	UINT32 IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked
	UINT32 IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	UINT32 ModemState;				//!< from SMSHOSTLIB_DVB_MODEM_STATE_ET
	INT32  SNR;						//!< dB
	UINT32 BER;						//!< Post Viterbi BER [1E-5]
	UINT32 BERErrorCount;			//!< Number of erroneous SYNC bits.
	UINT32 BERBitCount;				//!< Total number of SYNC bits.
	UINT32 TS_PER;					//!< Transport stream PER, 0xFFFFFFFF indicate N/A
	UINT32 MFER;					//!< DVB-H frame error rate in percentage, 0xFFFFFFFF indicate N/A, valid only for DVB-H
	INT32  RSSI;					//!< dBm
	INT32  InBandPwr;				//!< In band power in dBM
	INT32  CarrierOffset;			//!< Carrier Offset in bin/1024
	UINT32 ErrorTSPackets;			//!< Number of erroneous transport-stream packets
	UINT32 TotalTSPackets;			//!< Total number of transport-stream packets

	INT32  RefDevPPM;
	INT32  FreqDevHz;

	INT32  MRC_SNR;					//!< dB
	INT32  MRC_RSSI;				//!< dBm
	INT32  MRC_InBandPwr;			//!< In band power in dBM

}RECEPTION_STATISTICS_ST;


// Statistics information returned as response for SmsLiteMsGetStatistics_Req for DVB applications, SMS1100 and up
typedef struct SMSHOSTLIB_STATISTICS_DVBT_S
{
	// Reception
	RECEPTION_STATISTICS_ST ReceptionData;

	// Transmission parameters
	TRANSMISSION_STATISTICS_ST TransmissionData;

	UINT32 ReceptionQuality;
} SMSHOSTLIB_STATISTICS_DVBT_ST;

// Helper struct for ISDB-T statistics
typedef struct SMSHOSTLIB_ISDBT_LAYER_STAT_S
{
	// Per-layer information
	UINT32 CodeRate;			//!< Code Rate from SMSHOSTLIB_CODE_RATE_ET, 255 means layer does not exist
	UINT32 Constellation;		//!< Constellation from SMSHOSTLIB_CONSTELLATION_ET, 255 means layer does not exist
	UINT32 BER;					//!< Post Viterbi BER [1E-5], 0xFFFFFFFF indicate N/A
	UINT32 BERErrorCount;		//!< Post Viterbi Error Bits Count
	UINT32 BERBitCount;			//!< Post Viterbi Total Bits Count
	UINT32 PreBER; 				//!< Pre Viterbi BER [1E-5], 0xFFFFFFFF indicate N/A
	UINT32 TS_PER;				//!< Transport stream PER [%], 0xFFFFFFFF indicate N/A
	UINT32 ErrorTSPackets;		//!< Number of erroneous transport-stream packets
	UINT32 TotalTSPackets;		//!< Total number of transport-stream packets
	UINT32 TILdepthI;			//!< Time interleaver depth I parameter, 255 means layer does not exist
	UINT32 NumberOfSegments;	//!< Number of segments in layer A, 255 means layer does not exist
	UINT32 TMCCErrors;			//!< TMCC errors
} SMSHOSTLIB_ISDBT_LAYER_STAT_ST;

// Statistics information returned as response for SmsHostApiGetStatisticsEx_Req for ISDB-T applications, SMS1100 and up
typedef struct SMSHOSTLIB_STATISTICS_ISDBT_S
{
	UINT32 StatisticsType;			//!< Enumerator identifying the type of the structure.  Values are the same as SMSHOSTLIB_DEVICE_MODES_E
	//!< This field MUST always first in any statistics structure

	UINT32 FullSize;				//!< Total size of the structure returned by the modem.  If the size requested by
	//!< the host is smaller than FullSize, the struct will be truncated

	// Common parameters
	UINT32 IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked
	UINT32 IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	// Reception quality
	INT32  SNR;						//!< dB
	INT32  RSSI;					//!< dBm
	INT32  InBandPwr;				//!< In band power in dBM
	INT32  CarrierOffset;			//!< Carrier Offset in Hz

	// Transmission parameters
	UINT32 Frequency;				//!< Frequency in Hz
	UINT32 Bandwidth;				//!< Bandwidth in MHz
	UINT32 TransmissionMode;		//!< ISDB-T transmission mode
	UINT32 ModemState;				//!< 0 - Acquisition, 1 - Locked
	UINT32 GuardInterval;			//!< Guard Interval, 1 divided by value
	UINT32 SystemType;				//!< ISDB-T system type (ISDB-T / ISDB-Tsb)
	UINT32 PartialReception;		//!< TRUE - partial reception, FALSE otherwise
	UINT32 NumOfLayers;				//!< Number of ISDB-T layers in the network
	UINT32 SegmentNumber;			//!< Segment number for ISDB-Tsb
	UINT32 TuneBW;					//!< Tuned bandwidth - BW_ISDBT_1SEG / BW_ISDBT_3SEG

	// Per-layer information
	// Layers A, B and C
	SMSHOSTLIB_ISDBT_LAYER_STAT_ST	LayerInfo[3];	//!< Per-layer statistics, see SMSHOSTLIB_ISDBT_LAYER_STAT_ST

	// Interface information
	UINT32 SmsToHostTxErrors;		//!< Total number of transmission errors.

	// Proprietary information	
	UINT32 ExtAntenna;

	UINT32 ReceptionQuality;

} SMSHOSTLIB_STATISTICS_ISDBT_ST;

// Statistics information returned as response for SmsHostApiGetStatisticsEx_Req for FM Radio applications, SMS1200 and up
typedef struct SMSHOSTLIB_STATISTICS_FM_S
{
	UINT32 StatisticsType;			//!< Enumerator identifying the type of the structure.  Values are the same as SMSHOSTLIB_DEVICE_MODES_E
	//!< This field MUST always first in any statistics structure

	UINT32 FullSize;				//!< Total size of the structure returned by the modem.  If the size requested by
	//!< the host is smaller than FullSize, the struct will be truncated

	// Common parameters
	UINT32 IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32 IsModemLocked;			//!< 0 - not locked, 1 - locked

	// Reception quality
	INT32  SNR;						//!< dB
	INT32  RSSI;					//!< dBm
	INT32  InBandPwr;				//!< In band power in dBM
	INT32  CarrierOffset;			//!< Carrier Offset in Hz
	
	INT32  PreBer;					//
	INT32  BER;						//
	INT32  BlockErrRate;			//

	// Transmission parameters
	UINT32 Frequency;				//!< Frequency in Hz

	//FM Specific
	UINT32 IsFmStereo;				//
	UINT32 IsRdsExist;				// 
	UINT32 TotalCorrectBlock;		//
	UINT32 TotalBlock;		        //

} SMSHOSTLIB_STATISTICS_FM_ST;

/// Statistics information returned as response for SmsHostApiGetStatistics_Req
typedef struct SMSHOSTLIB_STATISTICS_S
{
	UINT32 Reserved;				//!< Reserved

	// Common parameters
	UINT32 IsRfLocked;				//!< 0 - not locked, 1 - locked
	UINT32 IsDemodLocked;			//!< 0 - not locked, 1 - locked
	UINT32 IsExternalLNAOn;			//!< 0 - external LNA off, 1 - external LNA on

	// Reception quality
	INT32  SNR;						//!< dB
	UINT32 BER;						//!< Post Viterbi BER [1E-5]
	UINT32 FIB_CRC;					//!< CRC errors percentage, valid only for DAB
	UINT32 TS_PER;					//!< Transport stream PER, 0xFFFFFFFF indicate N/A, valid only for DVB-T/H
	UINT32 MFER;					//!< DVB-H frame error rate in percentage, 0xFFFFFFFF indicate N/A, valid only for DVB-H
	INT32  RSSI;					//!< dBm
	INT32  InBandPwr;				//!< In band power in dBM
	INT32  CarrierOffset;			//!< Carrier Offset in bin/1024

	// Transmission parameters
	UINT32 Frequency;				//!< Frequency in Hz
	UINT32 Bandwidth;				//!< Bandwidth in MHz, valid only for DVB-T/H
	UINT32 TransmissionMode;		//!< Transmission Mode, for DAB modes 1-4, for DVB-T/H FFT mode carriers in Kilos
	UINT32 ModemState;				//!< from SMSHOSTLIB_DVB_MODEM_STATE_ET , valid only for DVB-T/H
	UINT32 GuardInterval;			//!< Guard Interval from SMSHOSTLIB_GUARD_INTERVALS_ET, valid only for DVB-T/H
	UINT32 CodeRate;				//!< Code Rate from SMSHOSTLIB_CODE_RATE_ET, valid only for DVB-T/H
	UINT32 LPCodeRate;				//!< Low Priority Code Rate from SMSHOSTLIB_CODE_RATE_ET, valid only for DVB-T/H
	UINT32 Hierarchy;				//!< Hierarchy from SMSHOSTLIB_HIERARCHY_ET, valid only for DVB-T/H
	UINT32 Constellation;			//!< Constellation from SMSHOSTLIB_CONSTELLATION_ET, valid only for DVB-T/H

	// Burst parameters, valid only for DVB-H
	UINT32 BurstSize;				//!< Current burst size in bytes, valid only for DVB-H
	UINT32 BurstDuration;			//!< Current burst duration in mSec, valid only for DVB-H
	UINT32 BurstCycleTime;			//!< Current burst cycle time in mSec, valid only for DVB-H
	UINT32 CalculatedBurstCycleTime;//!< Current burst cycle time in mSec, as calculated by demodulator, valid only for DVB-H
	UINT32 NumOfRows;				//!< Number of rows in MPE table, valid only for DVB-H
	UINT32 NumOfPaddCols;			//!< Number of padding columns in MPE table, valid only for DVB-H
	UINT32 NumOfPunctCols;			//!< Number of puncturing columns in MPE table, valid only for DVB-H
	UINT32 ErrorTSPackets;			//!< Number of erroneous transport-stream packets
	UINT32 TotalTSPackets;			//!< Total number of transport-stream packets
	UINT32 NumOfValidMpeTlbs;		//!< Number of MPE tables which do not include errors after MPE RS decoding
	UINT32 NumOfInvalidMpeTlbs;		//!< Number of MPE tables which include errors after MPE RS decoding
	UINT32 NumOfCorrectedMpeTlbs;	//!< Number of MPE tables which were corrected by MPE RS decoding
	// Common params
	UINT32 BERErrorCount;			//!< Number of errornous SYNC bits.
	UINT32 BERBitCount;				//!< Total number of SYNC bits.

	// Interface information
	UINT32 SmsToHostTxErrors;		//!< Total number of transmission errors.

	// DAB/T-DMB
	UINT32 PreBER; 					//!< DAB/T-DMB only: Pre Viterbi BER [1E-5]

	// DVB-H TPS parameters
	UINT32 CellId;					//!< TPS Cell ID in bits 15..0, bits 31..16 zero; if set to 0xFFFFFFFF cell_id not yet recovered
	UINT32 DvbhSrvIndHP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator
	UINT32 DvbhSrvIndLP;			//!< DVB-H service indication info, bit 1 - Time Slicing indicator, bit 0 - MPE-FEC indicator

	UINT32 NumMPEReceived;			//!< DVB-H, Num MPE section received

	UINT32 ReservedFields[10];		//!< Reserved

} SMSHOSTLIB_STATISTICS_ST;


/// Versioning
typedef struct SMSHOSTLIB_VERSIONING_S
{
	UINT8			Major;
	UINT8			Minor;
	UINT8			Patch;
	UINT8			FieldPatch;
} SMSHOSTLIB_VERSIONING_ST;

/// Version
typedef struct SMSHOSTLIB_VERSION_S
{
	UINT16						ChipModel;				//!< e.g. 0x1102 for SMS-1102 "Nova"
	UINT8						Step;					//!< 0 - Step A
	UINT8						MetalFix;				//!< 0 - Metal 0
	UINT8						FirmwareId;				//!< 0xFF - ROM or see #SMSHOSTLIB_DEVICE_MODES_E
	UINT8						SupportedProtocols;		/*!< Bitwise OR combination of supported
														protocols, see #SMSHOSTLIB_DEVICE_MODES_E */
	SMSHOSTLIB_VERSIONING_ST	FwVer;					//!< Firmware version
	SMSHOSTLIB_VERSIONING_ST	RomVer;					//!< ROM version
	UINT8						TextLabel[34];			//!< Text label
	SMSHOSTLIB_VERSIONING_ST	RFVer;					//!< RF tuner version
	UINT32						PkgVer;                 //!< SMS11xx Package Version
	UINT32						Reserved[9];            //!< Reserved for future use
}SMSHOSTLIB_VERSION_ST;

typedef struct SMSHOSTLIB_LOG_HEADER_S
{
	UINT32 n_filtering_classification;
	UINT32 e_format_string_tag;
	UINT32 n_timestamp_microseconds;
} SMSHOSTLIB_LOG_HEADER_ST;

typedef struct SMSHOSTLIB_LOG_ITEM_S
{
	SMSHOSTLIB_LOG_HEADER_ST x_header;
	UINT32                   a_n_parameters[SMSHOSTLIB_MAX_NUM_LOG_PARAMETERS];
} SMSHOSTLIB_LOG_ITEM_ST;


//! ISMACrypt key (control word)
typedef struct SMSHOSTLIB_CA_CW_S
{
	UINT32 Id; //!< Key ID 
	UINT8 Cw[SMSHOSTLIB_CMMB_CA_CONTROL_WORD_SIZE]; //!< Key
}SMSHOSTLIB_CA_CW_ST;


//! ISMACrypt key (control word) pair from ECM section
typedef struct SMSHOSTLIB_CA_CW_PAIR_S
{
	SMSHOSTLIB_CA_CW_ST Odd;  //!< First key 
	SMSHOSTLIB_CA_CW_ST Even; //!< Second key
}SMSHOSTLIB_CA_CW_PAIR_ST;


//! CMMB Big-card device operations structure
typedef struct SMS_CMMB_CA_DEV_OPS_S
{
	BOOL  ( *Open)(void *pDevData); //!< Open Big-card device
	void  ( *Close)(void *pDevData);//!< Close Big-card device
	INT32 ( *Read)(void *pDevData, UINT8 *pData, UINT32 Len); //!< Read Len bytes from big-card device into pData
	INT32 ( *Write)(void *pDevData, UINT8 *pData, UINT32 Len); //!< Write Len bytes to big-card device from pData
	void *pDevData;
} SMS_CMMB_CA_DEV_OPS_ST;


//! CMMB Big-card Device Information structure
typedef struct SMS_CMMB_CA_CARD_INFO_S
{
	UINT8  CardNumLen; //!< Card serial number length. Generally, it should be 8
	UINT8  CardNum[SMSHOSTLIB_CMMB_CA_CARD_NUMBER_MAX_LEN];  //!< Card serial number  
	UINT8  KDANumLen;  //!< Card version length. Should be 2 bytes
	UINT8  KDANum[SMSHOSTLIB_CMMB_CA_CARD_VERSION_MAX_LEN]; //!< Card version. 0x1820 means N card, 0x1821 means S card 
} SMS_CMMB_CA_CARD_INFO;

#ifdef __cplusplus
}
#endif


#endif //_SMS_HOST_LIB_TYPES_H_


