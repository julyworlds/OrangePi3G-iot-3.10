#ifndef I_ZSDCC_PRV_BUF_MGR
#define I_ZSDCC_PRV_BUF_MGR
//
#include <utils/String8.h>
//
//
namespace android {
namespace NSMtkZsdCcCamAdapter {
//
/******************************************************************************
*
*******************************************************************************/
class IPreviewBufMgrHandler : public virtual RefBase
{
public:    
    virtual                ~IPreviewBufMgrHandler() {}

public:
    virtual bool            dequeBuffer(int ePort, ImgBufQueNode &node) = 0;
    virtual bool            enqueBuffer(ImgBufQueNode const& node)= 0;
    virtual bool            registerBuffer(int ePort, int w, int h, const char* format, int cnt) = 0;
    virtual void            allocBuffer(int ePort, int cnt) = 0; 
    virtual void            freeBuffer(int ePort) = 0;
};

/******************************************************************************
 *
 ******************************************************************************/
class IPreviewBufMgr : public IPreviewBufMgrHandler
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Member Enum
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
public:
    enum EBufProvider{
        eBuf_Unknown,
        eBuf_Pass1,
        eBuf_Pass1Disp,
        eBuf_Disp,
        eBuf_AP,
        eBuf_FD,
        eBuf_Rec,
        eBuf_Generic,
    };

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  IPreviewCmdQueThreadHandler Interfaces.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public:    
    
    virtual bool            dequeBuffer(int ePort, ImgBufQueNode &node) = 0;
    virtual bool            enqueBuffer(ImgBufQueNode const& node)= 0;
    virtual bool            registerBuffer(int ePort, int w, int h, const char* format, int cnt) = 0;
    virtual void            allocBuffer(int ePort, int cnt) = 0; 
    virtual void            freeBuffer(int ePort) = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Operations.
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++    
public:

    static IPreviewBufMgr*  createInstance(sp<ImgBufProvidersManager>& rImgBufProvidersMgr);
    virtual void            destroyInstance() = 0;
    virtual                 ~IPreviewBufMgr(){};
};


};  // namespace NSMtkZsdCcCamAdapter
};  // namespace android

#endif
