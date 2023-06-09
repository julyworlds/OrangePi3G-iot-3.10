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

#ifndef _MTK_HAL_CAMADAPTER_MTKZSDCC_INC_CAPTURECMDQUETHREAD_H_
#define _MTK_HAL_CAMADAPTER_MTKZSDCC_INC_CAPTURECMDQUETHREAD_H_
//
#include <utils/threads.h>

#if (PLATFORM_VERSION_MAJOR == 2)
#include <utils/RefBase.h>
#else
#include <utils/StrongPointer.h>
#endif
//


namespace android {
namespace NSMtkZsdCcCamAdapter {


/*******************************************************************************
*   ICaptureCmdQueThreadHandler
*******************************************************************************/
class ICaptureCmdQueThreadHandler : public virtual RefBase
{
public:     ////        Instantiation.
    virtual             ~ICaptureCmdQueThreadHandler() {}

public:     ////        Interfaces

    // Derived class must implement the below function. The thread starts its
    // life here. There are two ways of using the Thread object:
    // 1) loop: if this function returns true, it will be called again if 
    //          requestExit() wasn't called.
    // 2) once: if this function returns false, the thread will exit upon return.
    virtual bool        onCaptureThreadLoop()               = 0;

    virtual bool        onInitCapMemory()                   = 0;
};


/*******************************************************************************
*   ICaptureCmdQueThread
*******************************************************************************/
class ICaptureCmdQueThread : public Thread
{
public:     ////        Instantiation.
    static  ICaptureCmdQueThread*   createInstance(ICaptureCmdQueThreadHandler*const pHandler);
    //
public:     ////        Attributes.
    virtual int32_t     getTid() const                      = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Commands.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:     ////        Interfaces.

    virtual status_t    onCapture()                         = 0;

    virtual status_t    onInitMem()                         = 0;
};


}; // namespace NSMtkZsdCcCamAdapter
}; // namespace android
#endif  //_MTK_HAL_CAMADAPTER_MTKZSDCC_INC_CAPTURECMDQUETHREAD_H_

