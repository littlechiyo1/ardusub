/*
   This is an automatically generated file.
   Do not manually modify this file.
   Any changes made here may be overwritten.
*/

#pragma once
#ifndef MODEL_H
#define MODEL_H

#include <string>

namespace GPIO 
{
    enum class Model 
    {
        CLARA_AGX_XAVIER,
        JETSON_NX,
        JETSON_XAVIER,
        JETSON_TX2,
        JETSON_TX1,
        JETSON_NANO,
        JETSON_TX2_NX,
        JETSON_ORIN,
        JETSON_ORIN_NX,
        JETSON_ORIN_NANO,
    };

    // names
    constexpr const char* MODEL_NAMES[] = {
        "CLARA_AGX_XAVIER",
        "JETSON_NX",
        "JETSON_XAVIER",
        "JETSON_TX2",
        "JETSON_TX1",
        "JETSON_NANO",
        "JETSON_TX2_NX",
        "JETSON_ORIN",
        "JETSON_ORIN_NX",
        "JETSON_ORIN_NANO",
    };

    // alias
    constexpr Model CLARA_AGX_XAVIER = Model::CLARA_AGX_XAVIER;
    constexpr Model JETSON_NX = Model::JETSON_NX;
    constexpr Model JETSON_XAVIER = Model::JETSON_XAVIER;
    constexpr Model JETSON_TX2 = Model::JETSON_TX2;
    constexpr Model JETSON_TX1 = Model::JETSON_TX1;
    constexpr Model JETSON_NANO = Model::JETSON_NANO;
    constexpr Model JETSON_TX2_NX = Model::JETSON_TX2_NX;
    constexpr Model JETSON_ORIN = Model::JETSON_ORIN;
    constexpr Model JETSON_ORIN_NX = Model::JETSON_ORIN_NX;
    constexpr Model JETSON_ORIN_NANO = Model::JETSON_ORIN_NANO;
} // namespace GPIO

#endif
