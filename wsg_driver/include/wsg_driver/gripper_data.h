#pragma once

#include <iostream>
#include <vector>

#include <stdio.h>
#include <string.h>
#include <bitset>


struct GripperFrame {
    unsigned char cmd;
    std::vector<unsigned char> data;
};
std::ostream& operator<<(std::ostream& os, const GripperFrame& c);

struct GripperData{
    std::bitset<8> state{0};

    float pose{0};
    float speed{0};
    float force{0};
    float force_l{0};
    float force_r{0};

    static GripperData fromFrame(GripperFrame);

    static const uint COMMAND_ID = 0xB0;
    static const int  BIT_SF_AXIS_STOPPED= 6;

    bool is_stopped() { return state.test(BIT_SF_AXIS_STOPPED); };
};
std::ostream& operator<<(std::ostream& os, const GripperData& c);

