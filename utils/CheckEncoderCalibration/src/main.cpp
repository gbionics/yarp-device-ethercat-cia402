// SPDX-FileCopyrightText: Generative Bionics
// SPDX-License-Identifier: BSD-3-Clause

#include <string>

#include <CiA402/LogComponent.h>
#include <CheckEncoderCalibration/CheckEncoderCalibration.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char** argv)
{
    // prepare and configure the resource finder
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("yarp-cia402-check-encoder-calibration.ini");
    rf.configure(argc, argv);

    CiA402::CheckEncoderCalibration app;
    const bool ok = app.run(rf);
    if (!ok)
    {
        yCError(CIA402, "CheckEncoderCalibration: FAILED");
        return 1;
    }
    yCInfo(CIA402, "CheckEncoderCalibration: DONE");
    return 0;
}
