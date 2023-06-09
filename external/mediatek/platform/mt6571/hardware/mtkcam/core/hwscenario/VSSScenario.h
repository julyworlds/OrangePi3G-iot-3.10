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

#ifndef VSS_SCENARIO_H
#define VSS_SCENARIO_H

#include <mtkcam/v1/ExtImgProc/IExtImgProc.h>
#include <mtkcam/common/ExtImgProcHw/ExtImgProcHw.h>

using namespace NSImageio;
using namespace NSIspio;

namespace NSImageio{
namespace NSIspio{
    class ICamIOPipe;
    class IPostProcPipe;
};
};

#include <utils/threads.h>
using namespace android;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  setConfig --> start --> Loop {enque, deque} --> stop
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class VSSScenario : public IhwScenario{

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  IhwScenario Interface.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:
    /////
    static VSSScenario*     createInstance(EScenarioFmt rSensorType, halSensorDev_e const &dev, ERawPxlID const &bitorder);
    virtual MVOID           destroyInstance();
    virtual                 ~VSSScenario();
    
protected:
                            VSSScenario(EScenarioFmt rSensorType, halSensorDev_e const &dev, ERawPxlID const &bitorder);     

public: 
    virtual MBOOL           init();
    virtual MBOOL           uninit();

    virtual MBOOL           start();
    virtual MBOOL           stop();

    virtual MVOID           wait(EWaitType rType);
    
    virtual MBOOL           deque(MUINT32 port, vector<PortQTBufInfo> *pBufIn);
    virtual MBOOL           enque(vector<PortBufInfo> *pBufIn = NULL, vector<PortBufInfo> *pBufOut = NULL);
    virtual MBOOL           enque(vector<IhwScenario::PortQTBufInfo> const &in);
    virtual MBOOL           replaceQue(vector<PortBufInfo> *pBufOld, vector<PortBufInfo> *pBufNew);

    virtual MBOOL           setConfig(vector<PortImgInfo> *pImgIn); 

    virtual MVOID           getHwValidSize(MUINT32 id, MUINT32 &width, MUINT32 &height, MUINT32 fps, MBOOL tryResize);
    
    virtual MVOID           setCamMode(MUINT32 u4CamMode);
    virtual MVOID           enableTwoRunPass2(MBOOL en);
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Private Operations.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
private:
            MVOID           defaultSetting();
            //
            MBOOL           enquePass1(vector<PortBufInfo> *pBufOut = NULL);
            MBOOL           enquePass2(
                                vector<PortBufInfo> *pBufIn = NULL,
                                vector<PortBufInfo> *pBufOut = NULL);
            MBOOL           enquePass2TwoRunRot(
                                vector<PortBufInfo> *pBufIn = NULL,
                                vector<PortBufInfo> *pBufOut = NULL);
            MBOOL           enquePass2TwoRunPass2(
                                vector<PortBufInfo> *pBufIn = NULL,
                                vector<PortBufInfo> *pBufOut = NULL);
            MBOOL           allocTwoRunPass2TempBuf(MUINT32 bufSize);
            MBOOL           freeTwoRunPass2TempBuf();
            //
            MBOOL           dequePass1(
                                MUINT32 port,
                                vector<PortQTBufInfo> *pBufIn);
            MBOOL           dequePass2(
                                MUINT32 port,
                                vector<PortQTBufInfo> *pBufIn);
            MBOOL           dequePass2TwoRunRot(
                                MUINT32 port,
                                vector<PortQTBufInfo> *pBufIn);
            MBOOL           dequePass2TwoRunPass2(
                                MUINT32 port,
                                vector<PortQTBufInfo> *pBufIn);
            //
            MVOID           handleRotate(vector<PortQTBufInfo> *pBufOut);
            MVOID           calCrop(
                                MUINT32     srcW,
                                MUINT32     SrcH,
                                MUINT32&    rCropW,
                                MUINT32&    rCropH, 
                                MUINT32&    rCropX,
                                MUINT32&    rCropY);
            EImageRotation  calRotation(EImageRotation rot = eImgRot_0);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Data Members.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
private:
    ICamIOPipe*             mpCamIOPipe;
    IPostProcPipe*          mpPostProcPipe;
    NSCamPipe::IXdpPipe*    mpXdpPipe;
    IMemDrv*                mpIMemDrv;
    EScenarioFmt            mSensorType;
    halSensorDev_e          mSensorDev;
    ERawPxlID               mSensorBitOrder;
    Mutex                   mModuleMtx;
    MINT32                  mSensorId;
    MBOOL                   mbP1Out;
    MBOOL                   mbP1DispOut;
    MBOOL                   mbTwoRunRot;
    MBOOL                   mbTwoRunPass2;
    MBOOL                   mbTwoRunPass2Dispo;
    //
    struct sDefaultSetting_Ports{
        PortInfo tgi;
        PortInfo imgo;
        PortInfo img2o;
        PortInfo imgi;
        PortInfo vido;
        PortInfo dispo;
        //
        MVOID dump();
        //
    };
    sDefaultSetting_Ports   mSettingPorts;
    //
    struct TwoRunRotInfo{
        QBufInfo outBuf;
        QBufInfo inBuf;
        PortInfo outPort;
    };
    TwoRunRotInfo           mTwoRunRotInfo;
    //
    struct TwoRunPass2PortInfo{
        PortID   portId;
        QBufInfo bufInfo;
        PortInfo portInfo;
    };
    struct TwoRunPass2Info{
        IMEM_BUF_INFO       tempBuf;
        TwoRunPass2PortInfo tempPort;
        TwoRunPass2PortInfo dispo;
        TwoRunPass2PortInfo vido;
    };
    TwoRunPass2Info         mTwoRunPass2Info;
    //
    const char*             msPass1OutFmt;
    const char*             msPass2DispoOutFmt;
    const char*             msPass2VidoOutFmt;
    ExtImgProcHw*           mpExtImgProcHw;
};


#endif
