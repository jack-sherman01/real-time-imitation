#pragma once

#pragma pack(push, 1)
struct TcpRecvData {
    char robot;
    char function;
    double parms[22];
    // double parm1;
    // double parm2;
    // double parm3;
    // double parm4;
    // double parm5;
    // double parm6;
    // double parm7;
    // double parm8;
    // double parm9;
    // double parm10;
    // double parm11;
    // double parm12;
    // double parm13;
    // double parm14;

    TcpRecvData():
        robot(0x00),
        function(0x00)
        // parm1(0), parm2(0), parm3(0), parm4(0), parm5(0), parm6(0), parm7(0), parm8(0)
    {}
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TcpSendData {
    char robot;
    char function;
    char error;
    double parms[22];
    // double parm1;
    // double parm2;
    // double parm3;
    // double parm4;
    // double parm5;
    // double parm6;
    // double parm7;
    // double parm8;
    // double parm9;
    // double parm10;
    // double parm11;
    // double parm12;
    // double parm13;
    // double parm14;

    TcpSendData():
        robot(0x00),
        function(0x00),
        error(0x00)
        // parm1(0), parm2(0), parm3(0), parm4(0), parm5(0), parm6(0), parm7(0), parm8(0)
    {}
};
#pragma pack(pop)