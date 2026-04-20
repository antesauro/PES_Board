#pragma once

#include "mbed.h"
#include "DebounceIn.h"

#include "actuators/motor_module_Arm.h"

class UserButtonCraneControl
{
public:
    UserButtonCraneControl(
        PinName button_pin,
        MotorModuleArm& crane_rope_motor,
        Callback<void()> short_press_callback,
        int long_press_ms = 5000,
        float manual_up_velocity_rps = -0.05f);

    void initialize();
    void update();

private:
    void onButtonPressed();
    void onButtonReleased();
    void startManualUp();
    void stopManualUp();

    DebounceIn m_user_button;
    MotorModuleArm& m_crane_rope_motor;
    Callback<void()> m_short_press_callback;

    Timer m_button_press_timer;

    const int m_long_press_ms;
    const float m_manual_up_velocity_rps;

    volatile bool m_button_is_pressed;
    volatile bool m_manual_up_active;
    volatile bool m_request_short_press;
    volatile bool m_request_manual_stop;
    volatile bool m_debug_pressed;
    volatile int  m_debug_released_ms;
};
