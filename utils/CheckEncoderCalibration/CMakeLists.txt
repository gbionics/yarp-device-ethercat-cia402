add_application(
    NAME check-encoder-calibration
    SOURCES src/main.cpp src/CheckEncoderCalibration.cpp
    HEADERS include/CheckEncoderCalibration/CheckEncoderCalibration.h
    LINK_LIBRARIES YarpCiA402::EthercatManager YARP::YARP_os tomlplusplus::tomlplusplus)
