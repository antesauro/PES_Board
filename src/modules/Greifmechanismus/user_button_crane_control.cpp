#include "user_button_crane_control.h"

UserButtonCraneControl::UserButtonCraneControl(
    PinName button_pin,
    MotorModuleArm& crane_rope_motor,
    Callback<void()> short_press_callback,
    int long_press_ms,
    float manual_up_velocity_rps) :
    m_user_button(button_pin),
    m_crane_rope_motor(crane_rope_motor),
    m_short_press_callback(short_press_callback),
    m_long_press_ms(long_press_ms),
    m_manual_up_velocity_rps(manual_up_velocity_rps),
    m_button_is_pressed(false),
    m_manual_up_active(false),
    m_request_short_press(false),
    m_request_manual_stop(false),
    m_debug_pressed(false),
    m_debug_released_ms(-1)
{
}

void UserButtonCraneControl::initialize()
{
    m_user_button.fall(callback(this, &UserButtonCraneControl::onButtonPressed));
    m_user_button.rise(callback(this, &UserButtonCraneControl::onButtonReleased));
}

void UserButtonCraneControl::update()
{
    if (m_debug_pressed) {
        m_debug_pressed = false;
        printf("DEBUG: Button pressed\n");
        // ...existing code...
    }

    if (m_debug_released_ms >= 0) {
        printf("DEBUG: Button released (held %d ms)\n", m_debug_released_ms);
        m_debug_released_ms = -1;
    }

    if (m_button_is_pressed && !m_manual_up_active) {
        const int pressed_time_ms = duration_cast<milliseconds>(m_button_press_timer.elapsed_time()).count();
        if (pressed_time_ms >= m_long_press_ms) {
            startManualUp();
        }
    }

    if (m_request_manual_stop) {
        m_request_manual_stop = false;
        stopManualUp();
    }

    if (m_request_short_press) {
        m_request_short_press = false;
        if (m_short_press_callback) {
            m_short_press_callback.call();
        }
    }
}

void UserButtonCraneControl::onButtonPressed()
{
    m_button_is_pressed = true;
    m_button_press_timer.reset();
    m_button_press_timer.start();
    m_debug_pressed = true;
}

void UserButtonCraneControl::onButtonReleased()
{
    const int pressed_time_ms = duration_cast<milliseconds>(m_button_press_timer.elapsed_time()).count();
    m_button_press_timer.stop();
    m_button_is_pressed = false;
    m_debug_released_ms = pressed_time_ms;

    if (pressed_time_ms >= m_long_press_ms) {
        m_request_manual_stop = true;
    } else {
        m_request_short_press = true;
    }
}

void UserButtonCraneControl::startManualUp()
{
    if (m_manual_up_active) {
        return;
    }

    if (!m_button_is_pressed) {
        return;
    }

    m_manual_up_active = true;
    m_crane_rope_motor.enableMotors(); // Motor aktivieren
    m_crane_rope_motor.setVelocity(m_manual_up_velocity_rps);
    printf("Long press erkannt: Seilmotor faehrt sehr langsam nach oben solange Taste gedrueckt ist.\n");
}

void UserButtonCraneControl::stopManualUp()
{
    if (!m_manual_up_active) {
        return;
    }

    m_crane_rope_motor.setVelocity(0.0f);
    m_crane_rope_motor.disableMotors(); // Motor deaktivieren
    m_manual_up_active = false;
    printf("Taste losgelassen: Seilmotor gestoppt.\n");
}
