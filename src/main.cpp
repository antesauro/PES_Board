#include <math.h>

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "FastPWM.h"
#include "PIDCntrl.h"
#include "SensorBar.h"
#include "Servo.h"
#include "UltrasonicSensor.h"

/* LINE FOLLOWER PART*/
// line-following constants
static constexpr float GOOD_POSITION = 0.0f;
static constexpr float STEERING_CENTER = 0.5f;
static constexpr float DRIVE_VOLTAGE_FULL_POWER = 12.0f;
static constexpr float STEERING_MIN = 0.15f;
static constexpr float STEERING_MAX = 0.85f;
static constexpr float PID_KP = -0.0035f;
static constexpr float PID_KI = 0.0f;
static constexpr float PID_KD = 0.0f;
static constexpr float PID_DT_SECONDS = 0.020f;
static constexpr uint8_t SENSOR_MASK_B2_TO_B5 = 0x3C;
static constexpr uint8_t SENSOR_MASK_ALL_BITS = 0xFF;
static constexpr int LINE_EVENT_PICKUP_HOUSE = 1;
static constexpr int LINE_EVENT_DELIVERY_HOUSE = 2;
static constexpr int PICKUP_HOUSE_DISTANCE_MM = 100;
static constexpr int DELIVERY_HOUSE_DISTANCE_MM = 50;
static float steering_command;

// Forward-declaration so the function can be called from within main()
uint8_t run_follow_line_fcn(SensorBar &sensor_bar, PIDCntrl &pid_controller);

/* END LINE FOLLOWER*/

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 50; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    /* INPUT OBJECTS*/

    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // Mechanical Button for emergency reset
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                       // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                       // is a defined potential

    /* OUTPUT OBJECTS*/
    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    /* SENSOR OBJECTS*/
    // sensor bar and PID controller for line following
    SensorBar sensor_bar(PB_IMU_SDA, PB_IMU_SCL, 0.10f, false);
    sensor_bar.clearInvertBits();
    sensor_bar.clearBarStrobe();
    PIDCntrl pid_controller(
        PID_KP, PID_KI, PID_KD, PID_DT_SECONDS, STEERING_MIN - STEERING_CENTER, STEERING_MAX - STEERING_CENTER);

    // Ultrasonic Sensor part
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;
    // min and max ultrasonic sensor reading, (us_distance_min, us_distance_max) -> (servo_min, servo_max)
    float us_distance_min = 6.0f;
    float us_distance_max = 40.0f;

    /* MOTOR OBJECTS*/

    // Servo Objects in order to control the correct pins

    Servo servo_D0(PB_D0); // Steering servo Object
    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // futuba S3003 STEERING SERVO
    float servo_D0_ang_min = 0.035f; // careful, these values might differ from servo to servo
    float servo_D0_ang_max = 0.110f;
    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(0.4f);

    float servo_input = 0.0f;
    int servo_counter = 0; // define servo counter, this is an additional variable variable to command the servo

    // reely S0090 CRANE SERVO
    // float servo_D1_ang_min = 0.0350f;
    // float servo_D1_ang_max = 0.1150f;

    // Servo loop time
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // object to enable power electronics for the DC motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    FastPWM pwm_M1(PB_PWM_M1); // fastPWM obcject for the main drive motor

    /* ROBOT STATES DECLARATION*/
    enum RobotState {
        INITIAL,
        READY,
        DRIVE,
        RETRIEVE,
        PICKUP,
        DELIVER,
        SLEEP,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    // Emergency toggle for the while loop
    // the loop will run as long as no emergeny has been met
    int toggle_emergency = 0;

    // --- code that runs every cycle at the start goes here ---

    // start timer
    main_task_timer.start();

    while (!toggle_emergency) {
        main_task_timer.reset();

        // state machine
        switch (robot_state) {
            case RobotState::INITIAL:
                printf("INITIAL\n");

                // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
                enable_motors = 1;
                if (!servo_D0.isEnabled())
                    servo_D0.enable();

                robot_state = RobotState::READY;
                break;

            case RobotState::READY:
                printf("READY\n");

                if (do_execute_main_task) {
                    robot_state = RobotState::DRIVE;
                    led1 = 1;

                    /* FOR SERVO CALIBRATION PROCESS ONLY*/
                    /*
                    printf("pulse width: %f\n", servo_input);
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    // command the servos
                    servo_D0.setPulseWidth(servo_input);
                    // calculate inputs for the servos for the next cycle
                    if ((servo_input < 1.0f) && // constrain servo_input to be < 1.0f
                        (servo_counter % loops_per_seconds ==
                         0) &&                // true if servo_counter is a multiple of loops_per_second
                        (servo_counter != 0)) // avoid servo_counter = 0
                        servo_input += 0.005f;
                    servo_counter++;*/
                    /* SERVO CALIBRATION PROCESS*/

                } else {
                    // the following code block gets executed only once
                    if (do_reset_all_once) {
                        do_reset_all_once = false;
                        // --- variables and objects that should be reset go here ---
                        // reset variables and objects
                        robot_state = RobotState::INITIAL;
                        servo_D0.disable();
                        servo_input = 0.0f;
                        led1 = 0;
                    }
                }

                break;

            case RobotState::DRIVE: {
                printf("DRIVE\n");
                const int action_code = run_follow_line_fcn(
                    sensor_bar,
                    pid_controller); // run line following function and get action code for state transitions

                pwm_M1.write(0.65f);

                // command the servos
                servo_D0.setPulseWidth(steering_command);
                // calculate inputs for the servos for the next cycle
                if ((servo_input > 0.0f && servo_input < 1.0f) && // constrain servo_input to be < 1.0f
                    (servo_counter % loops_per_seconds ==
                     0) &&                // true if servo_counter is a multiple of loops_per_second
                    (servo_counter != 0)) // avoid servo_counter = 0
                    servo_input += 0.1f;

                if (servo_input == 0.9f)
                    servo_input = 0.1f;

                servo_counter++;

                printf("Pulse width: %f \n", servo_input);

                if (action_code == LINE_EVENT_PICKUP_HOUSE) { // if the robot detects the pickup house, it transitions
                                                              // to the pickup state
                    printf("Querlinie Abhol-Haus: %d mm\n", PICKUP_HOUSE_DISTANCE_MM);
                    // robot_state = RobotState::PICKUP;
                } else if (action_code == LINE_EVENT_DELIVERY_HOUSE) { // if the robot detects the delivery house, it
                                                                       // transitions to the delivery state
                    printf("Querlinie Abliefer-Haus: %d mm\n", DELIVERY_HOUSE_DISTANCE_MM);
                    //// robot_state = RobotState::DELIVER;
                }
                break;
            }

            case RobotState::RETRIEVE:
                printf("RETRIEVE\n");

                break;

            case RobotState::PICKUP:
                printf("PICKUP\n");

                break;

            case RobotState::DELIVER:
                printf("DELIVER\n");

                break;

            case RobotState::SLEEP:
                printf("SLEEP\n");

                break;

            case RobotState::EMERGENCY:
                printf("EMERGENCY\n");
                // the transition to the emergency state causes the execution of the commands contained
                // in the outer else statement scope, and since do_reset_all_once is true the system undergoes a
                // reset
                toggle_emergency = 1;
                break;

            default:
                break; // do nothing
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

uint8_t run_follow_line_fcn(SensorBar &sensor_bar, PIDCntrl &pid_controller)
{
    sensor_bar.update();
    const uint8_t raw = sensor_bar.getRaw();
    const bool line_detected = sensor_bar.isAnyLedActive();
    const int8_t position = line_detected ? sensor_bar.getBinaryPosition() : 0;
    const float error = line_detected ? (static_cast<float>(position) - GOOD_POSITION) : 0.0f;
    // action code: 0 -> no event, 1 -> pickup house detected, 2 -> delivery house detected
    uint8_t action_code = 0;
    if (raw == SENSOR_MASK_ALL_BITS)
        action_code = LINE_EVENT_DELIVERY_HOUSE;
    else if (raw == SENSOR_MASK_B2_TO_B5)
        action_code = LINE_EVENT_PICKUP_HOUSE;
    // calculate steering command with PID controller
    steering_command = STEERING_CENTER + pid_controller.update(error);

    if (!line_detected) {
        pid_controller.reset();
        steering_command = STEERING_CENTER;
    }

    return action_code;
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
