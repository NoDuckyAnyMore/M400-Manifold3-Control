/**
 ********************************************************************
 * @file    main.cpp
 * @brief   Manifold 3 entry point for the simplified FC telemetry app.
 *
 * @copyright (c) 2018 DJI. All rights reserved.
 *********************************************************************
 */

#include "application.hpp"
#include <dji_logger.h>
#include <dji_platform.h>

extern "C" {
#include "m400_control/m400_control_service.h"
}

int main(int argc, char **argv)
{
    Application application(argc, argv);
    USER_LOG_INFO("Starting M400 telemetry, widget and control service.");

    T_DjiReturnCode returnCode = M400Control_StartService();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("M400 control service failed with error code: 0x%08llX",
                       (unsigned long long) returnCode);
        return 1;
    }

    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    while (true) {
        osal->TaskSleepMs(1000);
    }
}
