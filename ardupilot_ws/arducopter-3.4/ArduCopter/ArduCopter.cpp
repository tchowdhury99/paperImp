/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include "Copter.h"

// ── Choi et al. RAID 2020 — recovery module (Algorithm 1) ────────────────────
// §3.4: "The recovery_monitor() function is inserted right after the sensor
// reading code in the main control loop."  All sensor monitors tick at 50 Hz
// here (the one sanctioned deviation from the paper's 400 Hz), after sensor
// acquisition and BEFORE ahrs.update() (Figure 3: substitute before
// convert2angle()).
// RECOVERY_DISABLED builds the unmodified control loop (clean data collection,
// A-side of the §4 A/B evaluation). Comment the define out for the B-side.
// #define RECOVERY_DIAG 1
// #define RECOVERY_DISABLED 1   // recovery ON (B-side)
#ifndef RECOVERY_DISABLED
#include "../libraries/AP_InertialSensor/recovery_monitor.h"
#include "../libraries/AP_InertialSensor/software_sensors.h"

#define REC_INS_MAX 3              // monitored IMU instances (3DR Solo: 3)
#define REC_M_PER_DEG 111194.9f    // geodetic -> local meters (Appendix A)
static RecoveryChannel g_rec_gyro[REC_INS_MAX][3];
static RecoveryChannel g_rec_acc[REC_INS_MAX][3];
static RecoveryChannel g_rec_gps[6];        // pN pE alt vN vE vUp
static RecoveryChannel g_rec_baro;
static RecoveryChannel g_rec_head;
static bool     g_rec_init = false;
static int      g_rec_decim = 0;            // 400 Hz fast_loop -> 50 Hz (Ts=0.02)
static float    g_psi_unwrap, g_psi_prev;   // unwrapped measured yaw
static float    g_head_unwrap, g_head_prev; // unwrapped Eq.6 compass heading
static float    g_vel_hist[3][5];           // model velocity history (Eq. 4)
static float    g_baro_P0, g_baro_T0K;      // Eq. 5 base pressure / temperature
static int32_t  g_home_lat, g_home_lng;     // GPS canonicalization origin
static float    g_home_alt;                 // m (AMSL)
static Vector3f g_field_earth0;             // earth-frame reference mag field
static LPFilter g_supp_lpf[3];              // Appendix B output smoothing

static float baro_inv_convert(float press) {   // inverse Eq. 5: pressure -> z
    return -(8.3143f * g_baro_T0K) / (9.87f * 0.02896f) * logf(press / g_baro_P0);
}

// recovery_action() (Alg.1 l.23): set a flag; the GCS alert is emitted from
// read_AHRS() member context (gcs_send_text_fmt is private to Copter).
static volatile int g_rec_active_ch = -1;
static void recovery_action_flag(int ch) { g_rec_active_ch = ch; }
static uint32_t g_rec_last_msg_ms = 0;

#endif // !RECOVERY_DISABLED
// ─────────────────────────────────────────────────────────────────────────────

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_aux_switches,     10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_proximity,     100,     50),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(update_throttle_hover,100,     90),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(compass_accumulate,   100,    100),
    SCHED_TASK(barometer_accumulate,  50,     90),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75),
#endif
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(landinggear_update,    10,     75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
    SCHED_TASK(update_trigger,        50,     75),
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(perf_update,           0.1,    75),
    SCHED_TASK(read_receiver_rssi,    10,     75),
    SCHED_TASK(rpm_update,            10,    200),
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
#if ADSB_ENABLED == ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100),
#endif
    SCHED_TASK(terrain_update,        10,    100),
#if EPM_ENABLED == ENABLED
    SCHED_TASK(epm_update,            10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
    SCHED_TASK(button_update,          5,    100),
};


void Copter::setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Copter::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
void Copter::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Copter::perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu\n",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void Copter::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);
}


// Main loop - 400hz
void Copter::fast_loop()
{

    // IMU DCM Algorithm — recovery check is now INSIDE read_AHRS(), BEFORE ahrs.update()
    read_AHRS();

    // run low level rate controllers that only require IMU data
    attitude_control.rate_controller_run();
    
#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
}

// update_mount - update camera mount position
// should be run at 50hz
void Copter::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}

// update camera trigger
void Copter::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
    if (camera.check_trigger_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    }    
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
        if (rssi.enabled()) {
            DataFlash.Log_Write_RSSI(rssi);
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
        Log_Write_Proximity();
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif

#if PRECISION_LANDING == ENABLED
    // log output
    Log_Write_Precland();
#endif
}

void Copter::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    update_arming_checks();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        update_using_interlock();

#if FRAME_CONFIG != HELI_FRAME
        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);

        // set all throttle channel settings
        motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif
    }

    // update assigned functions and enable auxiliary servos
    RC_Channel_aux::enable_aux_servos();

    check_usb_mux();

    // update position controller alt limits
    update_poscon_alt_max();

    // enable/disable raw gyro/accel logging
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));

    // log terrain data
    terrain_logging();

    adsb.set_is_flying(!ap.land_complete);
}

// called at 50hz
void Copter::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc, copter.ahrs) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    // ── Choi et al. RAID 2020 — §4.1 attack module ───────────────────────────
    // "We add a piece of malicious code into the sensor interface in the firmware
    //  ... The attacks modify sensor measurements (through attack code) to mimic
    //  the effect of real controlled attacks (e.g. a sinusoidal wave, random or
    //  selected values). We map Mavlink commands to various attack types to
    //  remotely trigger via the ground control."
    // Trigger: spare RC channels (MAVLink RC_CHANNELS_OVERRIDE).
    //   ch7 PWM  -> gyro-X constant injection: bias = (pwm-1500)/500 * 2.0 rad/s
    //   ch8 PWM  -> selects waveform: <1300 constant, 1300-1700 sine, >1700 random
    // Runs in BOTH the recovery-on and recovery-off (A/B) builds, before the
    // recovery monitor and ahrs.update(), so the corrupted reading enters the loop
    // exactly as a physical gyroscope spoof would (§4.3 gyroscope case study).
    {
        uint16_t pwm7 = hal.rcin->read(6);   // channel 7
        if (pwm7 >= 1000 && (pwm7 < 1450 || pwm7 > 1550)) {
            float mag = (pwm7 - 1500) / 500.0f * 2.0f;   // rad/s
            uint16_t pwm8 = hal.rcin->read(7);
            float inj = mag;
            if (pwm8 >= 1300 && pwm8 <= 1700) {
                inj = mag * sinf(AP_HAL::millis() * 0.006283f);  // ~1 Hz sine
            } else if (pwm8 > 1700) {
                inj = mag * (2.0f * ((float)rand() / RAND_MAX) - 1.0f); // random
            }
            uint8_t gi = ins.get_primary_gyro();
            Vector3f g = ins.get_gyro(gi);
            g.x += inj;                              // constant value into roll rate
            ins.set_gyro(gi, g);                     // rate-controller path
            float dt = ins.get_delta_time();
            if (dt > 0.0f) {
                ins.set_delta_angle(gi, g * dt, dt);    // DCM/EKF path
            }
            static uint32_t atk_msg = 0;
            uint32_t now = AP_HAL::millis();
            if (now - atk_msg > 2000) {
                atk_msg = now;
                gcs_send_text_fmt(MAV_SEVERITY_WARNING,
                                  "ATTACK gyroX +%.2f rad/s", inj);
            }
        }
    }

    // ── Choi et al. RAID 2020 — Algorithm 1 monitors, all sensors (§3.4) ─────
    // Substitution happens BEFORE ahrs.update() (Figure 3: before convert2angle).
    // The monitor runs only during OPERATION (armed): the paper assumes the RV
    // "starts with the accurate initial states via software sensors" (§4.2.2).
    // Running it through EKF cold-start/alignment (pre-arm, transient states)
    // would let a channel false-latch on garbage states and then cascade via
    // substitution. We (re)seed the model from the real state at each arming.
#ifndef RECOVERY_DISABLED
    if (!motors.armed()) {
        g_rec_init = false;          // re-seed fresh on next arm
    } else if (++g_rec_decim >= 8) {
        g_rec_decim = 0;
        RecoveryModel& mdl = recovery_shared_model();

        if (!g_rec_init) {
            recovery_model_init(&mdl);
            g_recovery_action = recovery_action_flag;
            for (int i = 0; i < REC_INS_MAX; i++) {
                for (int a = 0; a < 3; a++) {
                    recovery_channel_init(&g_rec_gyro[i][a], CH_P + a);
                    recovery_channel_init(&g_rec_acc[i][a],  CH_VN + a);
                    g_rec_acc[i][a].state_idx = -1;  // Eq.4 conversion: no state sync
                }
            }
            static const int gps_ch[6] = {CH_PN, CH_PE, CH_ALT, CH_VN, CH_VE, CH_VUP};
            for (int k = 0; k < 6; k++) {
                recovery_channel_init(&g_rec_gps[k], gps_ch[k]);
            }
            recovery_channel_init(&g_rec_baro, CH_ALT);
            g_rec_baro.inv_convert = baro_inv_convert;     // §3.3 sync via inverse Eq.5
            recovery_channel_init(&g_rec_head, CH_PSI);

            // seed model state and frame references from the current vehicle state
            const Vector3f &gyr0 = ins.get_gyro();
            mdl.x[CH_PHI]   = ahrs.roll;
            mdl.x[CH_THETA] = ahrs.pitch;
            mdl.x[CH_PSI]   = ahrs.yaw;
            mdl.x[CH_P] = gyr0.x;  mdl.x[CH_Q] = gyr0.y;  mdl.x[CH_R] = gyr0.z;
            g_psi_prev = ahrs.yaw;  g_psi_unwrap = ahrs.yaw;
            g_baro_P0  = barometer.get_ground_pressure();
            g_baro_T0K = barometer.get_ground_temperature() + 273.15f;
            const Location &loc0 = gps.location();
            g_home_lat = loc0.lat;  g_home_lng = loc0.lng;
            g_home_alt = loc0.alt * 0.01f;
            // earth-frame reference field for the heading substitution (Appendix A)
            {
                const Vector3f &f0 = compass.get_field();
                float R0[3][3];
                body_to_inertial_R(ahrs.roll, ahrs.pitch, ahrs.yaw, R0);
                g_field_earth0.x = R0[0][0]*f0.x + R0[0][1]*f0.y + R0[0][2]*f0.z;
                g_field_earth0.y = R0[1][0]*f0.x + R0[1][1]*f0.y + R0[1][2]*f0.z;
                g_field_earth0.z = R0[2][0]*f0.x + R0[2][1]*f0.y + R0[2][2]*f0.z;
                g_head_prev = software_mag_heading(f0.x, f0.y, f0.z,
                                                   ahrs.roll, ahrs.pitch);
                g_head_unwrap = g_head_prev;
            }
            for (int a = 0; a < 3; a++) lpf_init(&g_supp_lpf[a]);
            memset(g_vel_hist, 0, sizeof(g_vel_hist));
            g_rec_init = true;
        }

        // unwrapped measured yaw (model psi state is continuous)
        float dpsi = wrap_pi(ahrs.yaw - g_psi_prev);
        g_psi_prev = ahrs.yaw;
        g_psi_unwrap += dpsi;

        // ── u: target states (Fig. 3 targets = navigation_logic()) ───────────
        // tiltN/tiltE: Appendix A frame canonicalization of the tilt commands.
        Vector3f tgt_cd = attitude_control.get_att_target_euler_cd();
        float phic   = radians(tgt_cd.x * 0.01f);
        float thetac = radians(tgt_cd.y * 0.01f);
        float psic   = g_psi_unwrap + wrap_pi(radians(tgt_cd.z * 0.01f) - ahrs.yaw);
        float cy = cosf(ahrs.yaw), sy = sinf(ahrs.yaw);
        float u[NU] = {
            phic, thetac, psic,
            attitude_control.get_throttle_in(),
            (-thetac) * cy - phic * sy,        // tiltN
            (-thetac) * sy + phic * cy,        // tiltE
            1.0f                                // template equilibrium term
        };

        // ── Algorithm 1 lines 6-7 (shared Eq. 1-3 vehicle model) ─────────────
        recovery_model_update(&mdl, u);

        // model velocity prediction history for the accelerometer Eq. 4
        for (int a = 0; a < 3; a++) {
            for (int k = 4; k > 0; k--) g_vel_hist[a][k] = g_vel_hist[a][k-1];
            g_vel_hist[a][0] = mdl.y[CH_VN + a];
        }

        // ── gyroscopes: one monitor per physical instance (Fig. 3) ───────────
        uint8_t ng = ins.get_gyro_count();
        for (uint8_t i = 0; i < ng && i < REC_INS_MAX; i++) {
            const Vector3f &gv = ins.get_gyro(i);
            float gx = recovery_check(&mdl, &g_rec_gyro[i][0], gv.x);
            float gy = recovery_check(&mdl, &g_rec_gyro[i][1], gv.y);
            float gz = recovery_check(&mdl, &g_rec_gyro[i][2], gv.z);
            if (g_rec_gyro[i][0].recovery_mode || g_rec_gyro[i][1].recovery_mode ||
                g_rec_gyro[i][2].recovery_mode) {
                Vector3f corr(gx, gy, gz);
                ins.set_gyro(i, corr);                      // _gyro[i] path
                float dt = ins.get_delta_time();
                if (dt > 0.0f) {
                    ins.set_delta_angle(i, corr * dt, dt);  // DCM/EKF path
                }
            }
        }

        // ── accelerometers: Eq. 4 conversion from model velocity states ──────
        {
            float aN = software_accel(g_vel_hist[0], TS);
            float aE = software_accel(g_vel_hist[1], TS);
            float aU = software_accel(g_vel_hist[2], TS);
            // specific force in inertial NED (z down): f = a - g
            float fN = aN, fE = aE, fD = -(aU + 9.80665f);
            float R[3][3];
            body_to_inertial_R(mdl.y[CH_PHI], mdl.y[CH_THETA], mdl.y[CH_PSI], R);
            float ms_acc[3] = {                 // body = R^T * inertial (App. A)
                R[0][0]*fN + R[1][0]*fE + R[2][0]*fD,
                R[0][1]*fN + R[1][1]*fE + R[2][1]*fD,
                R[0][2]*fN + R[1][2]*fE + R[2][2]*fD
            };
            uint8_t na = ins.get_accel_count();
            for (uint8_t i = 0; i < na && i < REC_INS_MAX; i++) {
                const Vector3f &av = ins.get_accel(i);
                float ax = recovery_check_ms(&mdl, &g_rec_acc[i][0], ms_acc[0], av.x);
                float ay = recovery_check_ms(&mdl, &g_rec_acc[i][1], ms_acc[1], av.y);
                float az = recovery_check_ms(&mdl, &g_rec_acc[i][2], ms_acc[2], av.z);
                if (g_rec_acc[i][0].recovery_mode || g_rec_acc[i][1].recovery_mode ||
                    g_rec_acc[i][2].recovery_mode) {
                    Vector3f corr(ax, ay, az);
                    ins.set_accel(i, corr);
                    float dt = ins.get_delta_time();
                    if (dt > 0.0f) {
                        ins.set_delta_velocity(i, dt, corr * dt);
                    }
                }
            }
        }

        // ── GPS: position/velocity directly from model states (§3.2) ─────────
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            const Location &loc = gps.location();
            const Vector3f &vel = gps.velocity();
            // geodetic -> local NE meters (frame canonicalization, Appendix A)
            float coslat = cosf(radians(g_home_lat * 1.0e-7f));
            float mN  = (loc.lat - g_home_lat) * 1.0e-7f * REC_M_PER_DEG;
            float mE  = (loc.lng - g_home_lng) * 1.0e-7f * REC_M_PER_DEG * coslat;
            float mAl = loc.alt * 0.01f - g_home_alt;
            float oN  = recovery_check(&mdl, &g_rec_gps[0], mN);
            float oE  = recovery_check(&mdl, &g_rec_gps[1], mE);
            float oAl = recovery_check(&mdl, &g_rec_gps[2], mAl);
            float oVN = recovery_check(&mdl, &g_rec_gps[3], vel.x);
            float oVE = recovery_check(&mdl, &g_rec_gps[4], vel.y);
            float oVU = recovery_check(&mdl, &g_rec_gps[5], -vel.z);
            bool any = false;
            for (int k = 0; k < 6; k++) any |= g_rec_gps[k].recovery_mode;
            if (any) {
                int32_t lat = g_home_lat + (int32_t)(oN / REC_M_PER_DEG * 1.0e7f);
                int32_t lng = g_home_lng + (int32_t)(oE / (REC_M_PER_DEG * coslat) * 1.0e7f);
                int32_t alt_cm = (int32_t)((oAl + g_home_alt) * 100.0f);
                gps.recovery_override(lat, lng, alt_cm, Vector3f(oVN, oVE, -oVU));
            }
        }

        // ── barometer: Eq. 5 on the model altitude state ──────────────────────
        {
            float ms_press = software_baro(mdl.y[CH_ALT], 0.0f, g_baro_P0, g_baro_T0K);
            float out_p = recovery_check_ms(&mdl, &g_rec_baro, ms_press,
                                            barometer.get_pressure());
            if (g_rec_baro.recovery_mode) {
                barometer.recovery_set_pressure(out_p);
            }
        }

        // ── magnetometer / heading channel (§3.2, Eq. 6) ─────────────────────
        // Software sensor = model orientation state psi ("we directly use the
        // orientation states from the system model").  Real measurement = heading
        // derived from the compass field via Eq. 6.  The steady offset between the
        // two (declination) is absorbed by the per-window error compensation e.
        {
            const Vector3f &f = compass.get_field();
            float head = software_mag_heading(f.x, f.y, f.z, ahrs.roll, ahrs.pitch);
            float dh = wrap_pi(head - g_head_prev);
            g_head_prev = head;
            g_head_unwrap += dh;
            recovery_check(&mdl, &g_rec_head, g_head_unwrap);
            if (g_rec_head.recovery_mode) {
                // substitute a field consistent with the software-sensor attitude:
                // body field = R^T(model attitude) * earth reference field (App. A)
                float R[3][3];
                body_to_inertial_R(mdl.y[CH_PHI], mdl.y[CH_THETA], mdl.y[CH_PSI], R);
                Vector3f fb(
                    R[0][0]*g_field_earth0.x + R[1][0]*g_field_earth0.y + R[2][0]*g_field_earth0.z,
                    R[0][1]*g_field_earth0.x + R[1][1]*g_field_earth0.y + R[2][1]*g_field_earth0.z,
                    R[0][2]*g_field_earth0.x + R[1][2]*g_field_earth0.y + R[2][2]*g_field_earth0.z);
                compass.recovery_set_field(fb);
            }
        }

        // ── all gyros compromised: supplementary compensation (Appendix B) ───
        bool all_gyros = (ng > 0);
        for (uint8_t i = 0; i < ng && i < REC_INS_MAX; i++) {
            for (int a = 0; a < 3; a++) {
                if (!g_rec_gyro[i][a].recovery_mode) all_gyros = false;
            }
        }
        if (all_gyros) {
            const Vector3f &acc = ins.get_accel();
            const Vector3f &mf  = compass.get_field();
            float pa, ta, pm;
            supplementary_compensation(acc.x, acc.y, acc.z, mf.x, mf.y, mf.z,
                                       &pa, &ta, &pm);
            pa = lpf_step(&g_supp_lpf[0], pa);   // App. B: low-pass the outputs
            ta = lpf_step(&g_supp_lpf[1], ta);
            pm = lpf_step(&g_supp_lpf[2], pm);
            // weighted sum with the software-sensor attitude (App. B; the paper
            // does not give weights — documented choice W_SUPP = 0.1 per tick)
            const float W_SUPP = 0.1f;
            mdl.x[CH_PHI]   += W_SUPP * (pa - mdl.x[CH_PHI]);
            mdl.x[CH_THETA] += W_SUPP * (ta - mdl.x[CH_THETA]);
            mdl.x[CH_PSI]   += W_SUPP * wrap_pi(pm - mdl.x[CH_PSI]);
        }


        // rate-limited GCS alert for §4 FP/FN observation (member context)
        if (g_rec_active_ch >= 0) {
            uint32_t now = AP_HAL::millis();
            if (now - g_rec_last_msg_ms > 1000) {
                g_rec_last_msg_ms = now;
                gcs_send_text_fmt(MAV_SEVERITY_WARNING,
                                  "RECOVERY active ch=%d", g_rec_active_ch);
            }
            g_rec_active_ch = -1;
        }
    }
#endif // RECOVERY_DISABLED
    // ── ahrs.update() now processes the (possibly substituted) sensors ───────
    ahrs.update();
}

// read baro and rangefinder altitude at 10hz
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN_CALLBACKS(&copter);
