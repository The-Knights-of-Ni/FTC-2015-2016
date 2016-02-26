#ifndef BUTTON
#define BUTTON

#include "maths.h"

#define btn_offset 40 //Modified for simulator

//TODO: BTN_ prefix
enum Buttons
{
    RIGHT_BUMPER,
    LEFT_BUMPER,
    BACK,
    START,
    GUIDE,
    Y,
    X,
    B,
    A,
    DPAD_RIGHT,
    DPAD_LEFT,
    DPAD_DOWN,
    DPAD_UP,
    RIGHT_STICK_BUTTON,
    LEFT_STICK_BUTTON,
    RIGHT_TRIGGER,
    LEFT_TRIGGER,
    SIZEOF_BUTTON
};

#pragma pack(push,1)
struct gamepad
{
    v2f joystick1;
    v2f joystick2;
    
    float left_trigger;
    float right_trigger;
    
    int buttons;
};
#pragma pack(pop)

#define trigger_press_threshold 0.1

struct Button
{
    int currentByte;
    int previousByte;
    //int prevStatus = 0;
    int toggleByte;
    
    static bool getState(int btn, int allBits)
    {
        if((allBits >> (btn)) & 1)
            return 1;
        else
            return 0;
    }
    
    bool press(int btn)
    {
        return getState(btn, currentByte);
    }
    
    bool singlePress(int btn)
    {
        return getState(btn, currentByte) && !getState(btn, previousByte);
    }
    
    bool toggle(int btn)
    {
        return getState(btn, toggleByte);
    }
    
    void updateButtons(gamepad g)
    {
        previousByte = currentByte;
        currentByte = g.buttons;
        currentByte |= (g.right_trigger >= trigger_press_threshold)<<RIGHT_TRIGGER;
        currentByte |= (g.left_trigger >= trigger_press_threshold)<<LEFT_TRIGGER;
        toggleByte = toggleByte^(currentByte & ~previousByte);
    }
};

gamepad * pgamepad1;
#define gamepad1 (*pgamepad1)

gamepad * pgamepad2;
#define gamepad2 (*pgamepad2)

#endif
