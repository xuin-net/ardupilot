#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

void Copter::auto_disarm_check()
{
    uint32_t tnow = millis();

    // 类静态变量
    static uint32_t begin_time = 0;
    static bool was_in_state = false;
    static bool was_armed = false;           // 新增：检测刚解锁

    bool now_armed = motors->armed();

    // ==================== 刚解锁时激活 pilot block ====================
    if (now_armed && !was_armed) {
        disarm_delay_blocked_by_pilot = true;
    }
    was_armed = now_armed;

    if (!now_armed) {
        disarm_delay_blocked_by_pilot = false;
        begin_time = 0;
        was_in_state = false;
        return;
    }

    // 保留部分原版重要检查（强烈建议保留）
    uint32_t disarm_delay_ms = 1000 * constrain_int16(g.disarm_delay, 0, INT8_MAX);
    if (disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        begin_time = 0;
        was_in_state = false;
        return;
    }

    // 电机正在 spool up 或已不在 ground idle 时，阻止自动闭锁
    if (motors->get_desired_spool_state() > AP_Motors::DesiredSpoolState::GROUND_IDLE ||
        motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        begin_time = 0;
        was_in_state = false;
        return;
    }

    // ==================== 你原来的 pilot block 逻辑 ====================
    if (disarm_delay_blocked_by_pilot) {
        if (!ap.land_complete) {
            // 已经真正离地过一次，解除阻止，之后就可以正常自动闭锁
            disarm_delay_blocked_by_pilot = false;
        } else {
            // 仍在地面，持续阻止并重置计时器
            begin_time = 0;
            was_in_state = false;
            return;
        }
    }

    // ==================== 使用原版完整的低油门判断逻辑（关键修复）===================
    bool thr_low;
    bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;

    if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
        thr_low = ap.throttle_zero;
    } else {
        float deadband_top = get_throttle_mid() + g.throttle_deadzone;
        thr_low = channel_throttle->get_control_in() <= deadband_top;
    }

    // 你想要的触发条件：仅在 LOITER + 低油门 + 已着陆
    bool in_state = (flightmode->mode_number() == Mode::Number::LOITER)
                    && thr_low
                    && ap.land_complete;

    // ==================== 计时逻辑 ====================
    if (in_state) {
        if (!was_in_state) {
            begin_time = tnow;      // 刚进入状态，开始计时
            was_in_state = true;
        }

        uint32_t elapsed = tnow - begin_time;

        if (elapsed >= 1000) {      // 你想要的 1 秒
            arming.disarm(AP_Arming::Method::DISARMDELAY);
            was_in_state = false;
            begin_time = tnow;
        }
    } else {
        // 离开状态，重置计时器
        was_in_state = false;
        begin_time = 0;
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
