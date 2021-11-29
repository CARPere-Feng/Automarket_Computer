//
// Created by miniload on 2021/11/17.
//

#ifndef MINILOADCORE_DATA_TYPE_H
#define MINILOADCORE_DATA_TYPE_H

#define	PI	 3.1415926535897932

using INT16 = unsigned int;
using BYTE = unsigned char;

union hex2int32{
    int integer32;
    unsigned char hexVal[4];
};

union hex2int
{
    int real_speed;
    unsigned char real_time_speed[4];

};
union hex2int_dis
{
    int dis_moved;
    unsigned char dis[4];
};


#endif //MINILOADCORE_DATA_TYPE_H
