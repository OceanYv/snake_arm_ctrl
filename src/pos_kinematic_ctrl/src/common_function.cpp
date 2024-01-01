/**
 * @file common_function.cpp
 * @author OceanYv (Ocean_Yv@zju.edu.cn)
 * @brief 声明了一些基础、通用的函数
 * @version 0.1
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <common_function.h>

bool Confirm_Interaction()
{
    char input;

    while(1){
        printf(" 'y' or 'n': ");
        std::cin >> input;

        if(input == 'y' | input == 'Y' )
            return true;
        else if(input == 'n' | input == 'N')
            return false;
    }   
}

int scanKeyboard(){
    int in;
    struct termios stored_settings, new_settings;

    if(tcgetattr(0, &stored_settings))
        printf("tcgetattr ERROR!!!!");
    memcpy(&new_settings, &stored_settings, sizeof(struct termios));

    // new_settings.c_lflag &=~ (ICANON | ECHO);
    new_settings.c_lflag &=~ (ICANON);
    new_settings.c_cc[VEOL] = 1;
    new_settings.c_cc[VEOF] = 2;
    new_settings.c_cc[VTIME] = 0.1;
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    setbuf(stdin, NULL);
    in = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);
    printf("\n");

    return in;
}