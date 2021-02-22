// Example program
#include <iostream>
#include <string>
#include <functional>
#include <iomanip>

#define M_PI 3.1415926535

int main() {
    //vel is in rad/s, require to convert to (degree/minute) then multiply by 10
    //    rad/s                     dpm          
    //vel ----> [ x180 / PI x 60 ] ----> [ x10 ] ---> cmd
    static uint8_t req_spd_cmd_frame[2][10] = {
            { 0x3E, 0xA2, 0x01, 0x04, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00 },
            { 0x3E, 0xA2, 0x02, 0x04, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00 }
    };
    static int32_t speedControl[2];
    double vel[2] = {1, 1};
    for(int i = 0; i < 2; i++){
        speedControl[i] = vel[i] * 180 * 60 * 10 / M_PI;
        req_spd_cmd_frame[i][5] = *(uint8_t *)(&speedControl[i]);
        req_spd_cmd_frame[i][6] = *((uint8_t *)(&speedControl[i]) + 1);
        req_spd_cmd_frame[i][7] = *((uint8_t *)(&speedControl[i]) + 2);
        req_spd_cmd_frame[i][8] = *((uint8_t *)(&speedControl[i]) + 3);
        req_spd_cmd_frame[i][9] = req_spd_cmd_frame[i][5] + req_spd_cmd_frame[i][6] + req_spd_cmd_frame[i][7] + req_spd_cmd_frame[i][8];
    }
    
    for(int i = 0; i < 10; i++){
        std::cout << std::hex << static_cast<int>(req_spd_cmd_frame[0][i]) << std::endl;
    }
}