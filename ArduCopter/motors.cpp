#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

void Copter::auto_disarm_check()
{
    if (ap.disarm_delay_blocked_by_pilot) {
        // 检查是否已经真实起飞，若离地则解除阻止
        if (!ap.land_complete) {
            // 一旦离地，说明驾驶员已控制飞机，可安全恢复自动闭锁功能
            ap.disarm_delay_blocked_by_pilot = false;
        } else {
            // 仍在地面，阻止自动闭锁，并重置内部状态
            begin_time = 0;
            was_in_state = false;
            return;
        }
    }
    
    uint32_t tnow = millis();
    
    // 类静态变量：真正的持久状态
    static uint32_t begin_time = 0;
    static bool was_in_state = false;

    bool in_state = (flightmode->mode_number() == Mode::Number::LOITER) 
                    && ap.throttle_zero 
                    && ap.land_complete;

    if (in_state) {
        if (!was_in_state) {
            begin_time = tnow;  // 刚进入状态，开始计时
            was_in_state = true;
        }
        // 已在状态，持续计时
        uint32_t elapsed = tnow - begin_time;
        
        static uint32_t last_print = 0;
        if (tnow - last_print > 400) {
            last_print = tnow;
        }
        
        if (elapsed >= 1000) {
            arming.disarm(AP_Arming::Method::DISARMDELAY);
            was_in_state = false;  // 准备下次
        }
    } else {
        was_in_state = false;  // 离开状态，下次重新计时
    }
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
// full_push is true when slower rate updates (e.g. servo output) need to be performed at the main loop rate.
void Copter::motors_output(bool full_push)
{
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;
        }
        // landing must continue to run the motors output
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();

    // cork now, so that all channel outputs happen at once
    srv.cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // update motors interlock state
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    if (ap.motor_test) {
        // check if we are performing the motor test
        motor_test_output();
    } else {
        // send output signals to motors
        flightmode->output_to_motors();
    }

    // push all channels
    if (full_push) {
        // motor output including servos and other updates that need to run at the main loop rate
        srv.push();
    } else {
        // motor output only at main loop rate or faster
        hal.rcout->push();
    }
}

// motors_output from main thread at main loop rate
void Copter::motors_output_main()
{
    if (!using_rate_thread) {
        motors_output();
    }
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
