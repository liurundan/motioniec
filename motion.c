#include "iec_types_all.h"
#include "POUS.h"

%(headers)s

int __init_motion()
{
    return __MK_Init();
}

void __cleanup_motion()
{
    __MK_Cleanup();
}

void __retrieve_motion()
{
    __MK_Retrieve();
}

void __publish_motion()
{
    __MK_Publish();
}
