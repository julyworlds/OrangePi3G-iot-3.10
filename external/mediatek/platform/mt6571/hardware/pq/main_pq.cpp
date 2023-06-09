#define LOG_TAG "PQ"

#include <cutils/xlog.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "ddp_drv.h"
#include "ddp_pq_index.h"
#include "ddp_shp_index.h"

int drvID = -1;
unsigned long lcmindex = 0; 

const DISP_PQ_PARAM pqparam = 
{
    u4SharpnessAdj:2,
    u4ContrastAdj:0,
    u4SaturationAdj:2
};

const DISP_PQ_PARAM pqparam_camera =
{
    u4SharpnessAdj:0,
    u4ContrastAdj:0,
    u4SaturationAdj:0
};



//COLOR_TUNING_INDEX : 10


//DISPLAY_PQ_T pqindex;
int main(int argc, char** argv) 
{
    int actionID=0, RegBase = 0, RegValue = 0, err = 0;
    char fileName[256];

    XLOGD("PQ init start...");
    if(drvID == -1) //initial
        drvID = open("/dev/mtk_disp", O_RDONLY, 0);

    XLOGD("PQ test...");
    ioctl(drvID, DISP_IOCTL_SET_PQINDEX, &pqindex);
    ioctl(drvID, DISP_IOCTL_SET_PQPARAM, &pqparam);
    ioctl(drvID, DISP_IOCTL_SET_GAMMALUT, &gammaindex);
    ioctl(drvID, DISP_IOCTL_SET_SHPINDEX, &shpindex); 
    ioctl(drvID, DISP_IOCTL_SET_PQ_CAM_PARAM, &pqparam_camera);
    ioctl(drvID, DISP_IOCTL_SET_PQ_GAL_PARAM, &pqparam);

    XLOGD("PQ init end !");
    return 0;
}
