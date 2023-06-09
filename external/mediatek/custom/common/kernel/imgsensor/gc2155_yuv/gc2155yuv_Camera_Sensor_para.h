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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   camera_sensor_para.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor tuning parameters that should be generated by CCT
 *
 *
 * Author:
 * -------
 *   Leo Leo
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * [GC2155YUV V1.0.0]
 * 8.17.2012 Leo.Lee
 * .First Release
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __CAMERA_SENSOR_PARA_H
#define __CAMERA_SENSOR_PARA_H

#define CAMERA_SENSOR_REG_DEFAULT_VALUE  \
		/* ARRAY: SENSOR.reg[11] */\
		{\
			/* STRUCT: SENSOR.reg[0] */\
			{\
				/* SENSOR.reg[0].addr */ 0xFFFFFFFF, /* SENSOR.reg[0].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[1] */\
			{\
				/* SENSOR.reg[1].addr */ 0xFFFFFFFF, /* SENSOR.reg[1].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[2] */\
			{\
				/* SENSOR.reg[2].addr */ 0xFFFFFFFF, /* SENSOR.reg[2].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[3] */\
			{\
				/* SENSOR.reg[3].addr */ 0xFFFFFFFF, /* SENSOR.reg[3].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[4] */\
			{\
				/* SENSOR.reg[4].addr */ 0xFFFFFFFF, /* SENSOR.reg[4].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[5] */\
			{\
				/* SENSOR.reg[5].addr */ 0xFFFFFFFF, /* SENSOR.reg[5].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[6] */\
			{\
				/* SENSOR.reg[6].addr */ 0xFFFFFFFF, /* SENSOR.reg[6].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[7] */\
			{\
				/* SENSOR.reg[7].addr */ 0xFFFFFFFF, /* SENSOR.reg[7].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[8] */\
			{\
				/* SENSOR.reg[8].addr */ 0xFFFFFFFF, /* SENSOR.reg[8].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[9] */\
			{\
				/* SENSOR.reg[9].addr */ 0xFFFFFFFF, /* SENSOR.reg[9].para */ 0xFFFFFFFF\
			},\
			/* STRUCT: SENSOR.reg[10] */\
			{\
				/* SENSOR.reg[10].addr */ 0xFFFFFFFF, /* SENSOR.reg[10].para */ 0xFFFFFFFF\
			}\
		}

#define CAMERA_SENSOR_CCT_DEFAULT_VALUE {{ 0xFFFFFFFF, 0xFFFFFFFF } ,{ 0xFFFFFFFF, 0xFFFFFFFF } ,{ 0xFFFFFFFF, 0xFFFFFFFF } ,{ 0xFFFFFFFF, 0xFFFFFFFF } ,{ 0xFFFFFFFF, 0xFFFFFFFF }}
#endif /* __CAMERA_SENSOR_PARA_H */
