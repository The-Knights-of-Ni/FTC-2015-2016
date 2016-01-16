#ifndef BUTTON
#define BUTTON

#define offset 40

enum Buttons
{
    RIGHT_BUMPER,
    LEFT_BUMPER,
    BACK,
    START,
    LOGITECH,
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
    SIZEOF_BUTTON
};

struct Button
{
    int currentByte;
    int previousByte;
    //int prevStatus = 0;
    bool toggles[SIZEOF_BUTTON];//TODO: Make this a bit array
    
    static bool getState(int btn, int allBits)
    {
        if((allBits >> (btn-1)) & 1)
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
        if(getState(btn, currentByte) && getState(btn, previousByte))//If there's a change and it's from low to high, flip it, if not, keep toggle the same.
            toggles[btn] = !toggles[btn];
        return toggles[btn];
    }
    
    void updateButtons()
    {
        previousByte = currentByte;
        //currentByte = stickArray;
    }
};

struct gamepad
{
    v2f joystick1;
    v2f joystick2;

    float left_trigger;
    float right_trigger;
    
    Button buttons;
};

#endif
