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

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#define LOG_TAG "campipe/Cdp"
//
//#include <inc/common/CamLog.h>
#include <mtkcam/Log.h>
#define MY_LOGV(fmt, arg...)    CAM_LOGV("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)    CAM_LOGD("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)    CAM_LOGI("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)    CAM_LOGW("[%s] "fmt, __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)    CAM_LOGE("[%s] "fmt, __FUNCTION__, ##arg)
#define FUNCTION_LOG_START      MY_LOGD("+");
#define FUNCTION_LOG_END        MY_LOGD("-");

//
#include <mtkcam/common.h>
//#include <common/CamTypes.h>
#include <common/hw/hwstddef.h>
//
#include <inc/imageio/IPipe.h>
#include <inc/imageio/ICdpPipe.h>
#include <inc/imageio/ispio_stddef.h>
#include <inc/imageio/ispio_pipe_ports.h>
#include <inc/imageio/ispio_pipe_scenario.h>

//
#include <inc/drv/isp_drv.h>

//
#include "../inc/PipeImp.h"
#include "../inc/CdpPipe.h"
#include "../inc/CampipeImgioPipeMapper.h"
//


/*******************************************************************************
*
********************************************************************************/
namespace NSCamPipe {
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
* 
********************************************************************************/
CdpPipe::
CdpPipe(
    char const*const szPipeName, 
    EPipeID const ePipeID, 
    ESWScenarioID const eSWScenarioID, 
    EScenarioFmt const eScenarioFmt
)
    : PipeImp(szPipeName, ePipeID, eSWScenarioID, eScenarioFmt)
    , mpCdpPipe(NULL) 
    , mu4OutPortEnableFlag(0)
{
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
init()
{
    FUNCTION_LOG_START;

    //(1) CameraIO pipe TG --> ISP --> Mem 
    mpCdpPipe = NSImageio::NSIspio::ICdpPipe::createInstance(
                                                       mapScenarioID(meSWScenarioID, mePipeID), 
                                                       mapScenarioFmt(meScenarioFmt));
    //
    if (NULL == mpCdpPipe || !mpCdpPipe->init())
    {
        return MFALSE;
    }
        
    FUNCTION_LOG_END;
    //
    return MTRUE;
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
uninit()
{
     FUNCTION_LOG_START;
     MBOOL ret = MFALSE; 
     //
     if (NULL != mpCdpPipe)
     {
         if (MTRUE != mpCdpPipe->uninit())
         {
             ret = MFALSE;
         }
         mpCdpPipe->destroyInstance(); 
         mpCdpPipe = NULL;
     }

     FUNCTION_LOG_END;
     //
     return ret;
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
start()
{
    FUNCTION_LOG_START
    MY_LOGD("+ (tid:%d), enable port:0x%x", ::gettid(), mu4OutPortEnableFlag); 
    //set pass2 IN DMA register before pass2 start
    mpCdpPipe->sendCommand((MINT32)NSImageio::NSIspio::EPIPECmd_SET_CURRENT_BUFFER, 
                          (MINT32)NSImageio::NSIspio::EPortIndex_IMGI,
                          0,
                          0
                         );

    //set pass2 OUT DMA register before pass2 start
    // port1 enable 
    if (mu4OutPortEnableFlag & 0x1) 
    {
        mpCdpPipe->sendCommand((MINT32)NSImageio::NSIspio::EPIPECmd_SET_CURRENT_BUFFER, 
                              (MINT32)NSImageio::NSIspio::EPortIndex_DISPO,
                               0,
                               0
                             );
    }
    // port2 enable 
    if (mu4OutPortEnableFlag & 0x2) 
    {
        mpCdpPipe->sendCommand((MINT32)NSImageio::NSIspio::EPIPECmd_SET_CURRENT_BUFFER, 
                              (MINT32)NSImageio::NSIspio::EPortIndex_VIDO,
                               0,
                               0
                             );
    }
    //
    mpCdpPipe->start();
    //
    mpCdpPipe->irq(NSImageio::NSIspio::EPipePass_PASS2, NSImageio::NSIspio::EPIPEIRQ_PATH_DONE);
    // 
    FUNCTION_LOG_END

    return  MTRUE;
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
stop()
{
    FUNCTION_LOG_START;
    //
    if ( ! mpCdpPipe->stop())
    {
       MY_LOGE("mpCdpPipe->stop() fail");
       return MFALSE;
    }
    //
    FUNCTION_LOG_END;
    //
    return MTRUE;
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
enqueBuf(PortID const ePortID, QBufInfo const& rQBufInfo)
{
    FUNCTION_LOG_START;
    MY_LOGD("+ tid(%d) PortID:(type, index, inout)=(%d, %d, %d)",  gettid(), ePortID.type, ePortID.index, ePortID.inout);
    MY_LOGD("QBufInfo:(user, reserved, num)=(%x, %d, %d)", rQBufInfo.u4User, rQBufInfo.u4Reserved, rQBufInfo.vBufInfo.size());
    // 
    for (MUINT32 i = 0; i < rQBufInfo.vBufInfo.size(); i++) 
    {
        MY_LOGD("QBufInfo(VA, PA, Size, ID) = (%x, %x, %d, %d)", rQBufInfo.vBufInfo.at(i).u4BufVA,
                        rQBufInfo.vBufInfo.at(i).u4BufPA, rQBufInfo.vBufInfo.at(i).u4BufSize, rQBufInfo.vBufInfo.at(i).i4MemID);
    }

    // 
    if (EPortType_MemoryOut != ePortID.type && EPortType_MemoryIn != ePortID.type) 
    {
        MY_LOGE("enqueBuf only support memory in/out port type"); 
        return MFALSE;
    }

    // Note:: can't update config, but address
    //
    NSImageio::NSIspio::QBufInfo rOutBufInfo; 
    NSImageio::NSIspio::PortID rPortID(NSImageio::NSIspio::EPortType_Memory, 
                                       NSImageio::NSIspio::EPortIndex_IMGI, 
                                       0); 
    //
    if (EPortType_MemoryOut == ePortID.type) 
    {
        // 
        rPortID.inout = 1; 
        if (0 == ePortID.index)      //yuv out buf 
        {
            rPortID.type = NSImageio::NSIspio::EPortType_DISP_RDMA; 
            rPortID.index = NSImageio::NSIspio::EPortIndex_DISPO;     
        }
        else 
        {
            rPortID.type = NSImageio::NSIspio::EPortType_VID_RDMA; 
            rPortID.index = NSImageio::NSIspio::EPortIndex_VIDO; 
        }
    }

    //
    for (MUINT32 i = 0; i < rQBufInfo.vBufInfo.size(); i++) 
    {
         NSImageio::NSIspio::BufInfo rBuf(rQBufInfo.vBufInfo.at(i).u4BufSize, 
                                          rQBufInfo.vBufInfo.at(i).u4BufVA, 
                                          rQBufInfo.vBufInfo.at(i).u4BufPA, 
                                          rQBufInfo.vBufInfo.at(i).i4MemID,
                                          rQBufInfo.vBufInfo.at(i).i4BufSecu,
                                          rQBufInfo.vBufInfo.at(i).i4BufCohe
                                         );  
         rOutBufInfo.vBufInfo.push_back(rBuf);                                                
    }
    //
    if (EPortType_MemoryOut == ePortID.type) 
    {
        mpCdpPipe->enqueOutBuf(rPortID, rOutBufInfo);         
    } 
    else 
    {
        MY_LOGD(" In buffer "); 
        mpCdpPipe->enqueInBuf(rPortID, rOutBufInfo); 
    }
    FUNCTION_LOG_END;
    return  MTRUE;
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
dequeBuf(PortID const ePortID, QTimeStampBufInfo& rQBufInfo, MUINT32 const u4TimeoutMs /*= 0xFFFFFFFF*/)
{
    FUNCTION_LOG_START;
    MY_LOGD("+ tid(%d) PortID:(type, index, inout, timeout)=(%d, %d, %d, %d)", gettid(), ePortID.type, ePortID.index, ePortID.inout, u4TimeoutMs);
    MBOOL ret = MTRUE; 
    //
    NSImageio::NSIspio::QTimeStampBufInfo rQTimeOutBufInfo; 
    NSImageio::NSIspio::PortID rPortID(NSImageio::NSIspio::EPortType_Memory, 
                                       NSImageio::NSIspio::EPortIndex_IMGI, 
                                       0); 
    //  dequeue buffer 
    if (EPortType_MemoryOut == ePortID.type)
    {
        rPortID.inout = 1; 
        if (0 == ePortID.index && (mu4OutPortEnableFlag & 0x1))   //disp port 
        {
            rPortID.type = NSImageio::NSIspio::EPortType_DISP_RDMA; 
            rPortID.index = NSImageio::NSIspio::EPortIndex_DISPO;           
        }
        else if(1 == ePortID.index && (mu4OutPortEnableFlag & 0x2)) //vdo port
        {
            rPortID.type = NSImageio::NSIspio::EPortType_VID_RDMA; 
            rPortID.index = NSImageio::NSIspio::EPortIndex_VIDO; 
        }    
        else 
        {
            MY_LOGE("The deque out port is not config");    
            return MFALSE; 
        }
    }    
    //
    if (EPortType_MemoryOut == ePortID.type) 
    {  
        ret = mpCdpPipe->dequeOutBuf(rPortID, rQTimeOutBufInfo);         
    }
    else 
    {
        ret = mpCdpPipe->dequeInBuf(rPortID, rQTimeOutBufInfo);         
    }
      
    // (2.2). put buffer in queue 
    rQBufInfo.u4User = rQTimeOutBufInfo.u4User; 
    rQBufInfo.u4Reserved = rQTimeOutBufInfo.u4Reserved;
    rQBufInfo.i4TimeStamp_sec = rQTimeOutBufInfo.i4TimeStamp_sec;
    rQBufInfo.i4TimeStamp_us = rQTimeOutBufInfo.i4TimeStamp_us; 
    
    for (MUINT32 i = 0; i < rQTimeOutBufInfo.vBufInfo.size(); i++) 
    {
        BufInfo rBufInfo; 
        mapBufInfo(rBufInfo, rQTimeOutBufInfo.vBufInfo.at(i)); 

        rQBufInfo.vBufInfo.push_back(rBufInfo); 
    }

    FUNCTION_LOG_END;
    return  ret;
}


/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
configPipe(vector<PortInfo const*>const& vInPorts, vector<PortInfo const*>const& vOutPorts)
{
    FUNCTION_LOG_START;
    MY_LOGD("+ %d in / %d out", vInPorts.size(), vOutPorts.size());
    MBOOL ret; 
    // 
    if (0 == vInPorts.size() 
        || 0 == vOutPorts.size() 
        || vOutPorts.size() > 2) 
    {
        MY_LOGE("Port config error");
        return false; 
    }
    //
    if (EPortType_MemoryIn != vInPorts.at(0)->type) 
    {
        MY_LOGE("The IN port type should be EPortType_MemoryIn type"); 
        return false; 
    }
    //
    for (MUINT32 i = 0; i < vOutPorts.size(); i++) 
    {
        if (EPortType_MemoryOut != vOutPorts.at(i)->type) 
        {
            MY_LOGE("The OUT port type should be EPortType_MemoryOut");
            return false; 
        }
    }  
    // (1). callbacks 
#warning [TODO] callbacks
    mpCdpPipe->setCallbacks(NULL, NULL, NULL);
    // (2). set CQ first before pipe config  
    ret = mpCdpPipe->sendCommand((MINT32)NSImageio::NSIspio::EPIPECmd_SET_CQ_CHANNEL,
                                 (MINT32)NSImageio::NSIspio::EPIPE_PASS2_CQ1,
                                  0,
                                  0
                                 );

    // (2.1) set plane config 
    mpCdpPipe->sendCommand((MINT32)NSImageio::NSIspio::EPIPECmd_SET_IMG_PLANE_BY_IMGI,(MINT32)1,0,0);


    //
    // (3). In MemoryIn Port 
    vector<NSImageio::NSIspio::PortInfo const*> vCdpInPorts; 
    MemoryInPortInfo const* const pMemoryInPort = reinterpret_cast<MemoryInPortInfo const*> (vInPorts.at(0)); 
    MY_LOGD("MemoryInPortInfo: (fmt, width, height) = (0x%x, %d, %d)", pMemoryInPort->eImgFmt, pMemoryInPort->u4ImgWidth, pMemoryInPort->u4ImgHeight); 
    MY_LOGD("MemoryInPortInfo: stride = (%d, %d, %d)",  pMemoryInPort->u4Stride[0],  pMemoryInPort->u4Stride[1],  pMemoryInPort->u4Stride[2]); 
    MY_LOGD("MemoryInPortInfo: crop:(x, y, w, h) = (%d, %d, %d, %d)", pMemoryInPort->rCrop.x, pMemoryInPort->rCrop.y, pMemoryInPort->rCrop.w, pMemoryInPort->rCrop.h); 

    // 
    NSImageio::NSIspio::PortInfo imgi;
    imgi.eImgFmt = pMemoryInPort->eImgFmt; 
    imgi.u4ImgWidth = pMemoryInPort->u4ImgWidth; 
    imgi.u4ImgHeight = pMemoryInPort->u4ImgHeight; 
    imgi.u4Stride[0] = pMemoryInPort->u4Stride[0];      
    imgi.u4Stride[1] = pMemoryInPort->u4Stride[1];      
    imgi.u4Stride[2] = pMemoryInPort->u4Stride[2];      
    imgi.crop.x = pMemoryInPort->rCrop.x;
    imgi.crop.y = pMemoryInPort->rCrop.y;
    imgi.crop.w = pMemoryInPort->rCrop.w;
    imgi.crop.h = pMemoryInPort->rCrop.h;
    imgi.type = NSImageio::NSIspio::EPortType_Memory;           
    imgi.index = NSImageio::NSIspio::EPortIndex_IMGI;           
    imgi.inout  = NSImageio::NSIspio::EPortDirection_In;        
    imgi.pipePass = NSImageio::NSIspio::EPipePass_PASS2;        
    vCdpInPorts.push_back(&imgi); 
    //
    // (4). Out Port    
    vector<NSImageio::NSIspio::PortInfo const*> vCdpOutPorts; 
    NSImageio::NSIspio::PortInfo dispo;
    NSImageio::NSIspio::PortInfo vido;   
    for (MUINT32 i = 0; i < vOutPorts.size(); i++) 
    {
        MemoryOutPortInfo const* const memOutPort= reinterpret_cast<MemoryOutPortInfo const*> (vOutPorts.at(i)); 
        //    
        if (0 == memOutPort->index) 
        {
            MY_LOGD("MemoryOutPortInfo1: (fmt, width, height) = (0x%x, %d, %d)", memOutPort->eImgFmt, memOutPort->u4ImgWidth, memOutPort->u4ImgHeight); 
            MY_LOGD("MemoryOutPortInfo1: stride = (%d, %d, %d)",  memOutPort->u4Stride[0],  memOutPort->u4Stride[1],  memOutPort->u4Stride[2]); 

            dispo.eImgFmt = memOutPort->eImgFmt;      
            dispo.u4ImgWidth = memOutPort->u4ImgWidth;     
            dispo.u4ImgHeight = memOutPort->u4ImgHeight;
            dispo.eImgRot = NSImageio::NSIspio::eImgRot_0;              //dispo NOT support rotation
            dispo.eImgFlip = NSImageio::NSIspio::eImgFlip_OFF;          //dispo NOT support flip          
            dispo.type = NSImageio::NSIspio::EPortType_DISP_RDMA;
            dispo.index = NSImageio::NSIspio::EPortIndex_DISPO; 
            dispo.inout  = NSImageio::NSIspio::EPortDirection_Out; 
            dispo.u4Stride[0] = memOutPort->u4Stride[0]; 
            dispo.u4Stride[1] = memOutPort->u4Stride[1]; 
            dispo.u4Stride[2] = memOutPort->u4Stride[2];
            vCdpOutPorts.push_back(&dispo);
            mu4OutPortEnableFlag |= 0x1;  
        }
#warning [TODO] Should check the port config by scenario 
        else if (1 == memOutPort->index) 
        {
            MY_LOGD("MemoryOutPortInfo2: (fmt, width, height) = (0x%x, %d, %d)", memOutPort->eImgFmt, memOutPort->u4ImgWidth, memOutPort->u4ImgHeight); 
            MY_LOGD("MemoryOutPortInfo2: stride = (%d, %d, %d)",  memOutPort->u4Stride[0],  memOutPort->u4Stride[1],  memOutPort->u4Stride[2]); 
            MY_LOGD("MemoryOutPortInfo2: (flip, rotation) = (%d, %d)", memOutPort->u4Rotation, memOutPort->u4Flip); 

            vido.eImgFmt = memOutPort->eImgFmt;  
            vido.u4ImgWidth = memOutPort->u4ImgWidth;  
            vido.u4ImgHeight = memOutPort->u4ImgHeight;
            vido.eImgRot = static_cast<NSImageio::NSIspio::EImageRotation>(memOutPort->u4Rotation/90);    
            vido.eImgFlip = static_cast<NSImageio::NSIspio::EImageFlip>(memOutPort->u4Flip);

            vido.type = NSImageio::NSIspio::EPortType_VID_RDMA;    
            vido.index = NSImageio::NSIspio::EPortIndex_VIDO;   
            vido.inout  = NSImageio::NSIspio::EPortDirection_Out;
            vido.u4Stride[0] = memOutPort->u4Stride[0]; 
            vido.u4Stride[1] = memOutPort->u4Stride[1];
            vido.u4Stride[2] = memOutPort->u4Stride[2];
            vCdpOutPorts.push_back(&vido); 
            mu4OutPortEnableFlag |= 0x2;  
        }
    }

    ret = mpCdpPipe->configPipe(vCdpInPorts, vCdpOutPorts);
    
    FUNCTION_LOG_END;
    return  ret;
}

/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
queryPipeProperty(vector<PortProperty > &vInPorts, vector<PortProperty > &vOutPorts)
{
    FUNCTION_LOG_START;
    PortID rMemInPortID(EPortType_MemoryIn, 0, 0); 
    PortID rMemOutDispPortID(EPortType_MemoryOut, 0, 1); 
    PortID rMemOutVdoPortID(EPortType_MemoryOut, 1, 1);   
    //
#warning [TODO]
    PortProperty rMemInPortProperty(rMemInPortID, eImgFmt_UNKNOWN, MFALSE, MFALSE); 
    PortProperty rMemOutDispProperty(rMemOutDispPortID, eImgFmt_BAYER10|eImgFmt_YUY2, MFALSE, MFALSE); 
    PortProperty rMemOutVdoProperty(rMemOutVdoPortID, eImgFmt_YUY2, MFALSE, MFALSE); 

    vInPorts.clear(); 
    vOutPorts.clear(); 

    vInPorts.push_back(rMemInPortProperty);     
    vOutPorts.push_back(rMemOutDispProperty);  
    vOutPorts.push_back(rMemOutVdoProperty); 

    dumpPipeProperty(vInPorts, vOutPorts); 
    FUNCTION_LOG_END;
    return  MTRUE;
}

/*******************************************************************************
* 
********************************************************************************/
MBOOL
CdpPipe::
sendCommand(MINT32 cmd, MINT32 arg1, MINT32 arg2, MINT32 arg3)
{
 
   FUNCTION_LOG_START;
   FUNCTION_LOG_END;
   return MTRUE; 
}

////////////////////////////////////////////////////////////////////////////////
};  //namespace NSCamPipe

