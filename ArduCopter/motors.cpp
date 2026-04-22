#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

// auto_disarm_check - disarm after a configurable delay if landed and throttle is low,
// unless takeoff/spool-up is being requested or auto-disarm is disabled.
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, INT8_MAX);

    // ==================== 超级调试打印（每0.5秒一次） ====================
    static uint32_t last_debug_print = 0;
    if (tnow_ms - last_debug_print > 500) {
        bool is_manual = flightmode->has_manual_throttle();
        bool is_loiter = flightmode->mode_number() == Mode::Number::LOITER;
        bool thr_low = false;
        if (is_manual || ! (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK)) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        int32_t timer_ms = (tnow_ms > auto_disarm_begin) ? (tnow_ms - auto_disarm_begin) : 0;

        gcs().send_text(MAV_SEVERITY_INFO,
            "AutoDisarm DEBUG | Mode=%u | Manual=%d | Loiter=%d | thr_low=%d | land_complete=%d | timer=%dms | armed=%d",
            (unsigned)flightmode->mode_number(), is_manual, is_loiter,
            thr_low, ap.land_complete, timer_ms, motors->armed());

        last_debug_print = tnow_ms;
    }
    // ==================== 调试打印结束 ====================

    // Reset timer and exit if disarmed, auto-disarm disabled, or in THROW mode.
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // If takeoff/spool-up is being requested, inhibit auto-disarm
    if (motors->get_desired_spool_state() > AP_Motors::DesiredSpoolState::GROUND_IDLE
        || motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // interlock / e-stop 情况缩短延迟
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
#if FRAME_CONFIG != HELI_FRAME
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        // ==================== 最终修复：把 LOITER 也当成手动模式处理 ====================
        if (flightmode->has_manual_throttle() || flightmode->mode_number() == Mode::Number::LOITER) {
            // 手动模式 或 Loiter：只看油门到底，强制1秒闭锁
            if (thr_low) {
                disarm_delay_ms = 1000;
                static uint32_t last_thr_print = 0;
                if (tnow_ms - last_thr_print > 500) {
                    uint32_t remaining = disarm_delay_ms - (tnow_ms - auto_disarm_begin);
                    gcs().send_text(MAV_SEVERITY_INFO, 
                        "AutoDisarm MANUAL/LOITER: thr_low=1, 剩余 %u ms (当前模式=%u)", 
                        remaining > 0 ? remaining : 0, 
                        (unsigned)flightmode->mode_number());
                    last_thr_print = tnow_ms;
                }
            } else {
                auto_disarm_begin = tnow_ms;
            }
        } else {
            // 其他自动模式（RTL/AUTO等）保留原有保护
            if (!ap.land_complete) {
                auto_disarm_begin = tnow_ms;
            }
        }
        // ==================== 修复结束 ====================
    }

    // disarm once timer expires
    if ((tnow_ms - auto_disarm_begin) >= disarm_delay_ms) {
        if (arming.disarm(AP_Arming::Method::DISARMDELAY)) {
            // 模式切换已在 disarm() 里处理
        }
        auto_disarm_begin = tnow_ms;
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
