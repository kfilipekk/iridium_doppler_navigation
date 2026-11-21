#include "AP_SoOP.h"
#include "AP_SoOP_MAX2112.h"
#include "AP_SoOP_Capture.h"

extern "C" {
#include "soop_signal.h"
#include "soop_iq4.h"
}

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/AP_SoOPFix.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>             //SYSCFG, for the PC2_C analogue switch
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

//ArduPilot compiles with -fsingle-precision-constant
static const uint64_t UNIX_J2000_US = 946728000ULL * 1000000ULL;   //12:00 UTC
static const uint64_t GPS_UNIX_S = 315964800ULL;                     //1980-01-06 00:00 UTC
//at rest: disarmed, the IMU this quiet and this steady for this long, and the u-blox agreeing
//where it can see. An IMU alone cannot tell rest from steady cruise; steadiness separates a
//table (0.01 m/s/s, 0.08 at most at Castle Mound) from a cabin in flight (0.1-0.5), and the
//u-blox, or its last word before a dropout, separates the rest
static const float STILL_GYRO = 0.03f;      //rad/s
static const float STILL_ACC = 0.3f;        //m/s/s off gravity
static const float STILL_ACC_SD = 0.1f;     //m/s/s, |accel|'s spread over about a second
static const float STILL_GPS_SPD = 1.0f;    //m/s: the u-blox reads up to 0.65 at rest indoors
static const float MOVING_SPD = 3.0f;       //m/s: past this the u-blox has seen it move
static const double STILL_HOLD_S = 2.0;
static const uint32_t BARO_FRESH_MS = 30000;//time only: a cabin's baro, tied to the u-blox
static const uint8_t PPS_PIN = 54;           //TP3, PB9 (PWM5): SERVO5_FUNCTION -1
static const double PPS_HOLD_S = 1.5;        //a pairing serves until the next edge is due
static const uint32_t IQ_SYNC_BLOCKS = 1221; //~10 s of capture blocks between I/Q fsyncs
//a u-blox stamps a fix when its message arrives: the navigation measured 75 +- 3 ms against
//Iridium at Castle Mound (2026-11-16), and a blind start needs it set
static const double GPS_LAG_S = 0.075;
static const uint32_t LEAP_S = 18;                                   //GPS - UTC since 2017
static const char *SOOP_DIR = "APM/SOOP";
//LED0 (blue, D2) and LED1 (green, D3) sink through PE3 and PE4: low is on
#if defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN)
#define AP_SOOP_LEDS 1
static const uint8_t LED_BLUE = HAL_GPIO_B_LED_PIN, LED_GREEN = HAL_GPIO_A_LED_PIN;
#endif
static const uint8_t BLUE_ON_OF = 8;     //1 ms in 8 from the 1 kHz timer: dim, no flicker
static const uint32_t GREEN_MS = 60;     //green after each fused burst
static const uint32_t HEARD_MS = 40;     //blue brighter after each burst the DSP detects
static const uint8_t DIM_ON = 1, FLASH_ON = 3;   //ms on in BLUE_ON_OF: both LEDs stay dim
static const uint32_t STAGE_MS = 4000, STAGE_GAP_MS = 400;   //green's start-up count
//orientation: bursts this strong decode as Ring Alert most of the time when the spectrum is the
//right way up (2026-11-18 flights, swapped offline: 95 of 298 in a minute at cruise)
static const float ORI_CN0 = 50.0f;
static const uint16_t ORI_N = 60, ORI_LOCKED_N = 400;

static AP_SoOP_MAX2112 tuner;
static AP_SoOP_Capture capture;

AP_SoOP *AP_SoOP::get_singleton()
{
    static AP_SoOP singleton;
    return &singleton;
}

namespace AP {
AP_SoOP *soop() { return AP_SoOP::get_singleton(); }
};

//main thread
void AP_SoOP::update()
{
    ant_update(AP_HAL::micros64() / double(1000000));
    if (!_started) {
        _started = true;
#ifdef AP_SOOP_LEDS
        //take the board LEDs only if notify has let them go, or the two would fight
        float ntf = 1;
        if (AP_Param::get("NTF_LED_TYPES", ntf) && !(uint32_t(ntf) & 1U)) {
            _leds = true;
            hal.gpio->pinMode(LED_BLUE, HAL_GPIO_OUTPUT);
            hal.gpio->pinMode(LED_GREEN, HAL_GPIO_OUTPUT);
            hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_SoOP::led_tick, void));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: LEDs left to notify (NTF_LED_TYPES bit 0)");
        }
#endif
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SoOP::thread_main, void),
                                          "SoOP", 8192, AP_HAL::Scheduler::PRIORITY_STORAGE, 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SoOP: no thread");
        }
        return;
    }

    if (_guard_state == SOOP_G_SPOOFED && !_spoof_handled) {
        //the real GPS disagrees with Iridium: stop the autopilot following it
        _spoof_handled = true;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "SoOP: GPS spoofed, %.0f m from Iridium - using SoOP",
                      double(_guard_dist));
        if (!AP_Param::set_by_name("GPS_AUTO_SWITCH", 0) || !AP_Param::set_by_name("GPS_PRIMARY", 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "SoOP: could not switch GPS - take manual control");
        }
    }

    if (_pps && !_pps_armed) {
        _pps_armed = true;
        hal.gpio->pinMode(PPS_PIN, HAL_GPIO_INPUT);
        if (!hal.gpio->attach_interrupt(PPS_PIN,
                FUNCTOR_BIND_MEMBER(&AP_SoOP::pps_isr, void, uint8_t, bool, uint32_t),
                AP_HAL::GPIO::INTERRUPT_RISING)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: no PPS interrupt on TP3 (SERVO5_FUNCTION -1?)");
        }
    }

    veh_t v {};
    v.t = AP_HAL::micros64() / double(1000000);
    v.armed = hal.util->get_soft_armed();

    Vector3f vel;
    const AP_AHRS &ahrs = AP::ahrs();
    v.vel_ok = ahrs.get_velocity_NED(vel);
    v.vn = vel.x;
    v.ve = vel.y;
    v.vd = vel.z;

    //the real GPS, only while it is good enough to calibrate against
    const AP_GPS &gps = AP::gps();
    float hacc = 99.0f, und = 0.0f;
    //a u-blox can report a 3D fix before it knows the week; time_epoch_usec is 0 until it does,
    //and 0 - UNIX_J2000_US wraps, which put UTC millions of years out and every TLE stale
    const uint64_t gps_us = gps.time_epoch_usec(0);
    const bool good = _guard_state != SOOP_G_SPOOFED
                      && gps.status(0) >= AP_GPS::GPS_OK_FIX_3D && gps.num_sats(0) >= 6
                      && gps.horizontal_accuracy(0, hacc) && hacc < 5.0f
                      && gps_us > UNIX_J2000_US;
    _gps_stage = good ? 2 : gps.status(0) >= AP_GPS::GPS_OK_FIX_3D ? 1 : 0;
    if (good) {
        if (gps.get_undulation(0, und)) {
            _undulation = und;
        }
        const Location &loc = gps.location(0);
        const double h = loc.alt / double(100) + _undulation;
        soop_ecef(loc.lat / double(10000000), loc.lng / double(10000000), h, v.gps_ecef);
        v.gps_utc = (gps_us - UNIX_J2000_US) / double(1000000);
        const uint32_t edges = _pps_edges;
        if (_pps && edges != _pps_used) {
            //the edge's board time: its 32-bit stamp, unwrapped against now
            const uint64_t now_us = AP_HAL::micros64();
            const uint64_t edge_us = now_us - uint32_t(uint32_t(now_us) - _pps_edge_us);
            const double tb = edge_us / double(1000000);
            //the message's UTC, lag-corrected, back to the edge, is within ~0.1 s of it:
            //the nearest whole second is the edge's (J2000 is 12:00:00, a whole second)
            const double est = v.gps_utc + _nav->cfg.gps_lag_s - (v.t - tb);
            _pps_tb = tb;
            _pps_utc = floor(est + 0.5);
            _pps_used = edges;
            AP::logger().WriteStreaming("SOP", "TimeUS,Edge,UTC,Off", "QQdf", now_us, edge_us,
                                        _pps_utc, float(est - _pps_utc));
        }
        if (_pps && _pps_tb > 0 && v.t - _pps_tb < PPS_HOLD_S) {
            v.gps_utc = _pps_utc + (v.t - _pps_tb);
            v.pps = true;
        }
        v.gps_ok = true;
        _gps_good_ms = AP_HAL::millis();
        _gps_spd = gps.ground_speed();
        const float b = AP::baro().get_altitude();
        const float off = h - b;                    //baro is relative: tie it to the GPS
        _baro_offset = _have_baro_offset ? 0.98f * _baro_offset + 0.02f * off : off;
        _have_baro_offset = true;
    }
    if (_gps_time_only) {
        //a moving test (an airliner, disarmed): the EKF without GPS drifts on the IMU and then
        //holds still, which at 250 m/s is wrong and would be trusted. Take the u-blox's own
        //velocity while it is good, and none without it, so the navigation carries the last
        v.vel_ok = good;
        if (good) {
            const Vector3f &gv = gps.velocity(0);
            v.vn = gv.x;
            v.ve = gv.y;
            v.vd = gv.z;
        }
    }
    {
        const AP_InertialSensor &ins = AP::ins();
        const float an = ins.get_accel().length();
        _acc_m += 0.05f * (an - _acc_m);
        _acc_m2 += 0.05f * (an * an - _acc_m2);
        const float acc_sd = sqrtf(MAX(0.0f, _acc_m2 - _acc_m * _acc_m));
        bool gps_rest = !_moving_latch;
        if (good) {
            const float spd = gps.ground_speed();
            if (spd > MOVING_SPD) {
                _moving_latch = true;
            } else if (spd < STILL_GPS_SPD) {
                _moving_latch = false;
            }
            gps_rest = spd < STILL_GPS_SPD;
        }
        const bool rest = !v.armed && gps_rest && ins.get_gyro().length() < STILL_GYRO
                          && fabsf(an - GRAVITY_MSS) < STILL_ACC && acc_sd < STILL_ACC_SD;
        if (!rest) {
            _still_since = -1;
        } else if (_still_since < 0) {
            _still_since = v.t;
        }
        v.still = !_no_still && rest && v.t - _still_since >= STILL_HOLD_S;
        _still = v.still;
    }
    v.temp_c = AP::ins().get_temperature(0);
#if AP_OPTICALFLOW_ENABLED && AP_RANGEFINDER_ENABLED
    //a velocity the GPS cannot drag
    const AP_OpticalFlow *of = AP::opticalflow();
    const RangeFinder *rf = AP::rangefinder();
    if (_ind_vel && of != nullptr && of->enabled() && of->healthy() && of->quality() >= AP_SOOP_IND_FLOW_Q
        && AP_HAL::millis() - of->last_update() < 200 && rf != nullptr
        && rf->status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good && v.vel_ok) {
        const float rng = rf->distance_orient(ROTATION_PITCH_270);
        const Vector2f comp = of->flowRate() + of->bodyRate();
        const Matrix3f &m = ahrs.get_rotation_body_to_ned();
        const float r[9] = { m.a.x, m.a.y, m.a.z, m.b.x, m.b.y, m.b.z, m.c.x, m.c.y, m.c.z };
        float vi[3];
        if (soop_flow_velocity(r, comp.x, comp.y, rng, v.vd, vi)) {
            v.ivn = vi[0];
            v.ive = vi[1];
            v.ivd = vi[2];
            v.ind_ok = true;
            AP::logger().WriteStreaming("SOV", "TimeUS,VN,VE,Q,Rng", "QffBf", AP_HAL::micros64(),
                                        v.ivn, v.ive, of->quality(), rng);
        }
    }
#endif
    //time only (a cabin): the baro reads cabin pressure, and its offset is the u-blox's
    //height; once that is stale a climb or descent would move it, so it waits for the u-blox
    if (_have_baro_offset && AP::baro().healthy()
        && (!_gps_time_only || AP_HAL::millis() - _gps_good_ms < BARO_FRESH_MS)) {
        v.baro_h = AP::baro().get_altitude() + _baro_offset;
        v.baro_ok = true;
    }

    WITH_SEMAPHORE(_sem);
    const uint16_t next = (_veh_head + 1) % AP_SOOP_VEH_QUEUE;
    if (next != _veh_tail) {
        _veh[_veh_head] = v;
        _veh_head = next;
    }
}

//the u-blox's timepulse: only the stamp, the pairing is done in update()
void AP_SoOP::pps_isr(uint8_t pin, bool high, uint32_t t_us)
{
    _pps_edge_us = t_us;
    _pps_edges++;
}

//timer thread, 1 kHz
void AP_SoOP::led_tick()
{
#ifdef AP_SOOP_LEDS
    if (++_led_n >= BLUE_ON_OF) {
        _led_n = 0;
    }
    //blue: dim and steady for power, a little brighter for a moment at each burst heard - no
    //GPS or time needed, so it shows reception alone; more satellites, faster blinking.
    //green: as dim a flash for each burst fused to a satellite, and every STAGE_MS a count of
    //how far start-up has got (_stage blinks: 1 u-blox fix not yet good, 2 finding the clock,
    //3 navigating). Low is on
    const uint32_t now = AP_HAL::millis();
    const uint8_t blue_on = now - _heard_ms < HEARD_MS ? FLASH_ON : DIM_ON;
    const uint32_t ph = now % STAGE_MS;
    const bool count = ph / STAGE_GAP_MS < _stage && ph % STAGE_GAP_MS < GREEN_MS;
    hal.gpio->write(LED_BLUE, _led_n >= blue_on);
    hal.gpio->write(LED_GREEN, !((count || now - _fused_ms < GREEN_MS) && _led_n < FLASH_ON));
#endif
}

//antenna feed
void AP_SoOP::ant_update(double t)
{
#if defined(HAL_SOOP_ANT_EN_GPIO) && defined(HAL_SOOP_ANT_FAULT_GPIO) && defined(HAL_SOOP_ANT_ADC_PIN)
    if (!_ant_started) {
        _ant_started = true;
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(STM32H7)
        //PC2_C reaches ADC3_INP0 only with its analogue switch open
        SYSCFG->PMCR |= SYSCFG_PMCR_PC2SO;
#endif
        _ant_adc = hal.analogin->channel(HAL_SOOP_ANT_ADC_PIN);
        hal.gpio->pinMode(HAL_SOOP_ANT_EN_GPIO, HAL_GPIO_OUTPUT);
        hal.gpio->pinMode(HAL_SOOP_ANT_FAULT_GPIO, HAL_GPIO_INPUT);
        soop_ant_cfg_t cfg;
        soop_ant_default_cfg(&cfg);
        soop_ant_init(&_ant, &cfg, t);
        _ant_reported = -1;
    }
    _ant.cfg.feed_enabled = !_ant_feed_off;
    _ant_ma = _ant_adc ? soop_ant_ma(_ant_adc->voltage_average(), HAL_SOOP_ANT_SENSE_OHM,
                                     HAL_SOOP_ANT_GAIN) : 0.0f;
    const bool fault = hal.gpio->read(HAL_SOOP_ANT_FAULT_GPIO) == 0;     //FAULT# is low
    hal.gpio->write(HAL_SOOP_ANT_EN_GPIO, soop_ant_step(&_ant, t, _ant_ma, fault));
    if (_ant.state != _ant_reported) {
        _ant_reported = _ant.state;
        const MAV_SEVERITY sev = _ant.state == SOOP_ANT_OK ? MAV_SEVERITY_INFO
                                 : _ant.state == SOOP_ANT_OFF ? MAV_SEVERITY_NOTICE
                                 : MAV_SEVERITY_WARNING;
        GCS_SEND_TEXT(sev, "SoOP: antenna %s, %.1f mA", soop_ant_name(_ant.state), double(_ant_ma));
    }
    if (t - _ant_log_t >= 1.0) {
        _ant_log_t = t;
        AP::logger().WriteStreaming("SOA", "TimeUS,St,mA,En,Flt,Rt", "QBfBBI",
                                    AP_HAL::micros64(), uint8_t(_ant.state), _ant_ma,
                                    uint8_t(_ant.en), uint8_t(fault), uint32_t(_ant.retries));
#if HAL_GCS_ENABLED
        //live on the GCS too (Mission Planner's Status tab, QGC's MAVLink inspector)
        gcs().send_named_float("ANT_MA", _ant_ma);
#endif
    }
#else
    (void)t;
#endif
}

//set-up
void AP_SoOP::load_config()
{
    //"key value" per line; unknown keys are reported, not ignored silently
    char path[40];
    snprintf(path, sizeof(path), "%s/soop.cfg", SOOP_DIR);
    struct stat st;
    if (AP::FS().stat(path, &st) != 0 || st.st_size <= 0) {
        return;
    }
    if (st.st_size > 4096) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: soop.cfg over 4 KB, not read");
        return;
    }
    //read it whole; comments may fill the first few hundred bytes
    char *buf = (char *)malloc(st.st_size + 1);
    if (buf == nullptr) {
        return;
    }
    const int fd = AP::FS().open(path, O_RDONLY);
    const int32_t n = fd >= 0 ? AP::FS().read(fd, buf, st.st_size) : -1;
    if (fd >= 0) {
        AP::FS().close(fd);
    }
    if (n <= 0) {
        free(buf);
        return;
    }
    buf[n] = 0;
    soop_nav_cfg_t &c = _nav->cfg;
    struct { const char *key; double *val; } dbl[] = {
        { "beta_sigma", &c.beta_sigma }, { "clkd_q", &c.clkd_q }, { "clk_q", &c.clk_q },
        { "vel_sigma", &c.vel_sigma }, { "vel_tau", &c.vel_tau }, { "dut1", &c.dut1 },
        { "mask_deg", &c.mask_deg }, { "max_tle_age_d", &c.max_tle_age_d },
        { "fix_max_hacc", &c.fix_max_hacc },
        { "along_m_per_day", &c.orbit[SOOP_ORBIT_TLE].along_m_per_day },
        { "guard_k", &_gcfg.k }, { "guard_floor_m", &_gcfg.floor_m },
        { "guard_hold_s", &_gcfg.hold_s }, { "guard_free_vel", &_gcfg.free_vel_sigma },
        { "ind_vel_sigma", &_gcfg.ind_vel_sigma },
        { "gps_lag_s", &c.gps_lag_s }, { "still_sigma", &c.still_sigma },
        { "clkd_sigma", &c.clkd_sigma }, { "clkd_tau", &c.clkd_tau },
        { "reacq_frac", &c.reacq_frac }, { "sig_floor", &c.sig_floor },
    };
    //no scanf here: ArduPilot's C library has none with floating point
    char *save = nullptr;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(nullptr, "\n", &save)) {
        char *key = line;
        while (*key == ' ' || *key == '\t') {
            key++;
        }
        char *sp = strpbrk(key, " \t");
        if (key[0] == '#' || key[0] == 0 || sp == nullptr) {
            continue;
        }
        *sp = 0;
        char *end = nullptr;
        const double val = strtod(sp + 1, &end);
        if (end == sp + 1) {
            continue;
        }
        bool known = true;
        if (!strcmp(key, "replay")) {
            _replay = !is_zero(val);
        } else if (!strcmp(key, "swap_iq")) {
            _swap_iq = !is_zero(val);
        } else if (!strcmp(key, "ant_feed")) {
            _ant_feed_off = is_zero(val);
        } else if (!strcmp(key, "ind_vel")) {
            _ind_vel = !is_zero(val);
        } else if (!strcmp(key, "gps_pos")) {
            _gps_time_only = is_zero(val);
        } else if (!strcmp(key, "start_lat")) {
            _start_lla[0] = val;
            _start_keys |= 1;
        } else if (!strcmp(key, "start_lon")) {
            _start_lla[1] = val;
            _start_keys |= 2;
        } else if (!strcmp(key, "start_h")) {
            _start_lla[2] = val;
            _start_keys |= 4;
        } else if (!strcmp(key, "start_sigma")) {
            _start_sigma = val;
        } else if (!strcmp(key, "still")) {
            _no_still = is_zero(val);
        } else if (!strcmp(key, "pps")) {
            _pps = !is_zero(val);
        } else if (!strcmp(key, "iq_rec_s")) {
            _iq_rec_s = val;
        } else if (!strcmp(key, "iq_every_s")) {
            _iq_every_s = val;
        } else if (!strcmp(key, "iq_bits")) {
            _iq_bits = (val >= 16) ? 16 : 4;
        } else if (!strcmp(key, "iq_fast_mps")) {
            _iq_fast_mps = val;
        } else if (!strcmp(key, "bbg")) {
            _bbg = uint8_t(constrain_int16(int16_t(val), 0, 15));
        } else if (!strcmp(key, "lo_offset_hz")) {
            //clear of the dc notch
            if (fabs(val) >= 20000.0 && fabs(val) <= 150000.0) {
                _lo_off = val;
                c.f_lo = SOOP_F_RING_ALERT + val;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: lo_offset_hz %.0f outside 20-150 kHz", val);
            }
        } else if (!strcmp(key, "record_lo_hz")) {
            //record only: the tuner anywhere in the L band around Iridium (its duplex channels are
            //1616-1626 MHz), for I/Q snippets to study offline. The DSP and navigation look for
            //Ring Alert where it is not, so their output means nothing
            if (val >= 1610.0e6 && val <= 1630.0e6) {
                c.f_lo = val;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: record only at %.3f MHz - no navigation",
                              val * 1e-6);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: record_lo_hz %.0f outside 1610-1630 MHz", val);
            }
        } else {
            known = false;
            for (auto &d : dbl) {
                if (!strcmp(key, d.key)) {
                    *d.val = val;
                    known = true;
                }
            }
        }
        if (!known) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: unknown soop.cfg key %s", key);
        }
    }
    free(buf);
}

int AP_SoOP::load_catalogue()
{
    char path[40];
    snprintf(path, sizeof(path), "%s/iridium.tle", SOOP_DIR);
    struct stat st;
    if (AP::FS().stat(path, &st) != 0 || st.st_size <= 0 || st.st_size > 64 * 1024) {
        return 0;
    }
    char *text = (char *)malloc(st.st_size + 1);
    if (text == nullptr) {
        return 0;
    }
    const int fd = AP::FS().open(path, O_RDONLY);
    const int32_t n = fd >= 0 ? AP::FS().read(fd, text, st.st_size) : -1;
    if (fd >= 0) {
        AP::FS().close(fd);
    }
    int count = 0;
    if (n > 0) {
        text[n] = 0;
        for (const char *p = text; *p; p++) {           //size the table by its "1 " lines
            if ((p == text || p[-1] == '\n') && p[0] == '1' && p[1] == ' ') {
                count++;
            }
        }
        _cat = (soop_sat_t *)calloc(count, sizeof(soop_sat_t));
        count = _cat ? soop_ephem_parse(text, _cat, count) : 0;
    }
    free(text);
    return count;
}

bool AP_SoOP::setup()
{
    _dsp = (soop_dsp_t *)hal.util->malloc_type(sizeof(soop_dsp_t), AP_HAL::Util::MEM_FAST);
    _guard = (soop_guard_t *)calloc(1, sizeof(soop_guard_t));
    _nav = _guard ? &_guard->main : nullptr;
    _block = (int16_t *)calloc(2 * SOOP_CAPTURE_BLOCK, sizeof(int16_t));
    if (!_dsp || !_guard || !_block) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SoOP: out of memory");
        return false;
    }
    soop_dsp_init(_dsp);
    _bbg = 2;                        //check_soop.py gain
    soop_nav_cfg_t cfg;
    soop_nav_default_cfg(&cfg);
    _nav->cfg = cfg;
    soop_guard_default_cfg(&_gcfg);
    _lo_off = SOOP_LO_OFFSET_HZ;
    _nav->cfg.gps_lag_s = GPS_LAG_S;
    //real bursts: the DSP's sigma is 5-10x optimistic (flights 2026-11-18/19); the library's
    //1 Hz is for the simulator's ideal signals
    _nav->cfg.sig_floor = 12.0;
    load_config();
    soop_dsp_set_lo(_dsp, _lo_off);
    cfg = _nav->cfg;

    _n_cat = load_catalogue();
    if (_n_cat < 60) {
        //a short catalogue narrows the geometry silently
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SoOP: %d satellites in %s/iridium.tle - need the full catalogue",
                      _n_cat, SOOP_DIR);
        return false;
    }

    if (_replay) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: replay of %s/replay.iq, %d satellites", SOOP_DIR, _n_cat);
    } else {
        // the LO the C side asks for, and the one the synthesiser can actually make
        if (!tuner.init(0, cfg.f_lo, _bbg)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SoOP: MAX2112 not locked (VCO %u, VTUNE ADC %u)",
                          tuner.vco(), tuner.vtune_adc());
            return false;
        }
        cfg.f_lo = tuner.lo_hz();
        if (!AP_SoOP_Capture::supported() || !capture.start(_swap_iq)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SoOP: no I/Q capture on this build");
            return false;
        }
        _capturing = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: tuner locked, VCO %u, %d satellites",
                      tuner.vco(), _n_cat);
        //an ADC that drops triggers raises no error, it just runs slow: every frequency the DSP
        //reads is then scaled by the ratio, so say so rather than search the wrong band
        hal.scheduler->delay(200);
        const double fs_hz = capture.measured_hz();
        if (fabs(fs_hz / SOOP_FS - 1.0) > 0.01) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SoOP: I/Q at %.0f kSPS, the DSP expects %.0f",
                          fs_hz / 1000.0, SOOP_FS / 1000.0);
        }
    }
    soop_guard_init(_guard, &cfg, &_gcfg, _cat, _n_cat, 0.0);
    clock_load(AP::ins().get_temperature(0));
    _fs = SOOP_FS;
    _warned_stale = false;
    return true;
}

void AP_SoOP::thread_main()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(2000);                     //let the SD card and sensors settle
    if (!setup()) {
        return;
    }
    if (!_replay) {
        iq_setup();
    }
    int fd = -1;
    uint64_t replay_sample = 0;
    if (_replay) {
        char path[40];
        snprintf(path, sizeof(path), "%s/replay.iq", SOOP_DIR);
        fd = AP::FS().open(path, O_RDONLY);
        _t0 = AP_HAL::micros64() / double(1000000);
    } else {
        _t0 = capture.start_us() / double(1000000);
    }

    while (true) {
        uint64_t first = 0;
        if (_replay) {
            const int32_t want = 4 * SOOP_CAPTURE_BLOCK;
            if (fd < 0 || AP::FS().read(fd, _block, want) != want) {
                if (fd >= 0) {
                    AP::FS().close(fd);
                    fd = -1;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: replay done");
                }
                hal.scheduler->delay(1000);
                continue;
            }
            first = replay_sample;
            replay_sample += SOOP_CAPTURE_BLOCK;
            //hold real time, so the CPU figure means what it will mean in flight
            const double due = _t0 + replay_sample / _fs;
            const double now = AP_HAL::micros64() / double(1000000);
            if (due > now) {
                hal.scheduler->delay_microseconds(uint16_t(MIN((due - now) * 1e6, 60000.0f)));
            }
        } else if (!capture.next_block(_block, first, 50)) {
            continue;
        }
        run_block(_block, first);
        if (!_replay) {
            iq_record(_block, first);
        }
    }
}

//raw I/Q snippets, as the DSP was handed them (interleaved int16 I, Q at SOOP_FS): what it
//keeps of a burst is a few numbers, and the capture also holds the other simplex channels,
//so a better DSP can run over these later (dsp_host, or soop.cfg replay). SOI logs each
//file's first sample and its board time
//a slot in the ring: what the writer is to do; a DATA slot's record is in _iq_data
struct AP_SoOP::iq_slot_t {
    uint8_t  kind;           //IQ_OPEN, IQ_DATA, IQ_CLOSE
    uint8_t  bits;
    uint64_t first_sample;   //OPEN: the recording's first sample, and its board time
    double   t;
};
enum { IQ_OPEN = 1, IQ_DATA, IQ_CLOSE };
static const uint32_t IQ_RING_KB[] = { 256, 192, 160, 128, 96, 64 };   //the largest that fits
static const uint32_t IQ_RUN_MAX = 8;    //DATA slots merged into one write: ~32 KB, a cluster

//capture thread: start the writer and its ring, if soop.cfg records at all
bool AP_SoOP::iq_setup()
{
    if (_iq_rec_s <= 0 && _iq_fast_mps <= 0) {
        return false;
    }
    //each record starts with its capture block number (uint32), so a file says its own
    //timeline: the SOD records of dropped blocks are droppable log messages, and 2026-11-18 lost
    //a fifth of them (110 s of misplaced I/Q on the return flight)
    _iq_rec_bytes = 4 + (_iq_bits == 16 ? 4 * SOOP_CAPTURE_BLOCK : 4 + SOOP_CAPTURE_BLOCK);
    for (uint32_t kb : IQ_RING_KB) {
        const uint32_t n = kb * 1024 / (_iq_rec_bytes + sizeof(iq_slot_t));
        if (n < 8) {
            break;
        }
        _iq_data = (uint8_t *)malloc(size_t(n) * _iq_rec_bytes);
        _iq_slots = (iq_slot_t *)calloc(n, sizeof(iq_slot_t));
        if (_iq_data && _iq_slots) {
            _iq_nslots = n;
            break;
        }
        free(_iq_data);
        free(_iq_slots);
        _iq_data = nullptr;
        _iq_slots = nullptr;
    }
    if (!_iq_slots || !hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SoOP::iq_writer, void),
                                                    "SoOPiq", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: no I/Q recording (no memory or thread)");
        free(_iq_data);
        free(_iq_slots);
        _iq_data = nullptr;
        _iq_slots = nullptr;
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: I/Q %u-bit, ring %u blocks (%.0f ms)", unsigned(_iq_bits),
                  unsigned(_iq_nslots), _iq_nslots * SOOP_CAPTURE_BLOCK * 1000.0 / SOOP_FS);
    return true;
}

//capture thread: the next free slot and its record space, or null when the writer is that
//far behind
AP_SoOP::iq_slot_t *AP_SoOP::iq_claim(uint8_t **data)
{
    const uint32_t next = (_iq_head + 1) % _iq_nslots;
    if (next == _iq_tail) {
        return nullptr;
    }
    if (data) {
        *data = _iq_data + size_t(_iq_head) * _iq_rec_bytes;
    }
    return &_iq_slots[_iq_head];
}

void AP_SoOP::iq_commit()
{
    __sync_synchronize();                 //the slot's contents before the writer can see it
    _iq_head = (_iq_head + 1) % _iq_nslots;
}

//capture thread: decide what to record and queue it; never touches the card
void AP_SoOP::iq_record(const int16_t *iq, uint64_t first_sample)
{
    if (!_iq_slots) {
        return;
    }
    const double t = _t0 + first_sample / _fs;
    //continuous while fast (an airliner from its take-off roll): latched on the u-blox's speed
    //and held through a window dropout until it reads under half of it again
    if (_iq_fast_mps > 0 && AP_HAL::millis() - _gps_good_ms < 2000) {
        if (_gps_spd > _iq_fast_mps) {
            _iq_fast = true;
        } else if (_gps_spd < 0.5 * _iq_fast_mps) {
            _iq_fast = false;
        }
    }
    iq_slot_t *s;
    if (_iq_close_pending) {
        if (!(s = iq_claim(nullptr))) {
            return;
        }
        s->kind = IQ_CLOSE;
        iq_commit();
        _iq_close_pending = false;
    }
    if (!_iq_open) {
        if (!_iq_fast && (_iq_rec_s <= 0 || t < _iq_next_t)) {
            return;
        }
        if (!(s = iq_claim(nullptr))) {
            return;                       //the writer is behind: start on a later block
        }
        if (!_iq_fast) {
            _iq_next_t = t + (_iq_every_s > 0 ? _iq_every_s : 300.0);
        }
        s->kind = IQ_OPEN;
        s->bits = _iq_bits;
        s->first_sample = first_sample;
        s->t = t;
        iq_commit();
        _iq_open = true;
        //a continuous recording is cut into 10 minute files
        _iq_left = uint32_t((_iq_fast ? 600.0 : _iq_rec_s) * _fs / SOOP_CAPTURE_BLOCK) + 1;
    }
    uint8_t *rec;
    if ((s = iq_claim(&rec))) {
        s->kind = IQ_DATA;
        const uint32_t blk = uint32_t(first_sample / SOOP_CAPTURE_BLOCK);
        memcpy(rec, &blk, sizeof(blk));
        if (_iq_bits == 16) {
            memcpy(rec + 4, iq, 4 * SOOP_CAPTURE_BLOCK);
        } else {
            soop_iq4_pack(iq, SOOP_CAPTURE_BLOCK, rec + 4);
        }
        iq_commit();
    } else {
        //the card is that far behind: this block is not recorded, and the log says which
        _iq_dropped++;
        AP::logger().WriteStreaming("SOD", "TimeUS,S", "QQ", AP_HAL::micros64(), first_sample);
    }
    if (--_iq_left == 0) {
        _iq_open = false;
        if ((s = iq_claim(nullptr))) {
            s->kind = IQ_CLOSE;
            iq_commit();
        } else {
            _iq_close_pending = true;
        }
    }
}

//its own thread: the only code that touches the card for I/Q. A card stall waits here
void AP_SoOP::iq_writer()
{
    char path[40];
    int16_t n = -1;                       //file number, from the first free one
    while (true) {
        if (_iq_tail == _iq_head) {
            hal.scheduler->delay(2);
            continue;
        }
        __sync_synchronize();             //the slot's contents after seeing it committed
        const iq_slot_t *s = &_iq_slots[_iq_tail];
        uint32_t used = 1;
        const char *ext = s->bits == 16 ? "i16b" : "iq4b";   //b: block-numbered records
        switch (s->kind) {
        case IQ_OPEN:
            if (_iq_fd >= 0) {
                AP::FS().close(_iq_fd);
                _iq_fd = -1;
            }
            //leave the card room for the flight log: nothing new with under 3 GB free. Near
            //full, ArduPilot deletes the OLDEST logs (LOG_FILE_MB_FREE), a trip's start
            _iq_skip = AP::FS().disk_free(SOOP_DIR) < int64_t(3) * 1024 * 1024 * 1024;
            if (_iq_skip) {
                break;
            }
            if (n < 0) {                  //so a reboot adds files rather than overwriting
                struct stat st;
                for (n = 0; n < 9999; n++) {
                    snprintf(path, sizeof(path), "%s/iq%04d.%s", SOOP_DIR, int(n), ext);
                    if (AP::FS().stat(path, &st) != 0) {
                        break;
                    }
                }
            }
            snprintf(path, sizeof(path), "%s/iq%04d.%s", SOOP_DIR, int(n), ext);
            _iq_fd = AP::FS().open(path, O_WRONLY | O_CREAT | O_TRUNC);
            if (_iq_fd >= 0) {
                AP::logger().Write("SOI", "TimeUS,N,S,T,Bits", "QHQdB", AP_HAL::micros64(),
                                            uint16_t(n), s->first_sample, s->t, s->bits);
                n++;
                _iq_since_sync = 0;
            }
            break;
        case IQ_DATA: {
            //a run of DATA slots up to the head or the end of the array is one write: the
            //card is far faster at a cluster than at a block at a time
            const uint32_t head = _iq_head;
            while (used < IQ_RUN_MAX && _iq_tail + used < _iq_nslots && _iq_tail + used != head
                   && _iq_slots[_iq_tail + used].kind == IQ_DATA) {
                used++;
            }
            if (_iq_fd >= 0 && !_iq_skip) {
                const int32_t len = int32_t(used * _iq_rec_bytes);
                if (AP::FS().write(_iq_fd, _iq_data + size_t(_iq_tail) * _iq_rec_bytes, len) != len) {
                    AP::FS().close(_iq_fd);
                    _iq_fd = -1;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: I/Q write failed, file closed");
                } else {
                    _iq_written += used;
                    //FAT writes a file's size only when it is closed or synced: without this a
                    //power cut lost the whole open file (up to 10 minutes)
                    _iq_since_sync += used;
                    if (_iq_since_sync >= IQ_SYNC_BLOCKS) {
                        AP::FS().fsync(_iq_fd);
                        _iq_since_sync = 0;
                    }
                }
            }
            break;
        }
        case IQ_CLOSE:
            if (_iq_fd >= 0) {
                AP::FS().close(_iq_fd);
                _iq_fd = -1;
            }
            _iq_skip = false;
            break;
        }
        __sync_synchronize();
        _iq_tail = (_iq_tail + used) % _iq_nslots;
    }
}

void AP_SoOP::run_block(const int16_t *iq, uint64_t first_sample)
{
    soop_burst_t out[8];
    if (!_have_first || first_sample != _next_first) {
        if (_have_first) {
            //blocks the thread fell behind on: the DSP restarts across them, and an I/Q file
            //simply carries on without them, so log where the samples jump to line it up later
            AP::logger().WriteStreaming("SOG", "TimeUS,From,To", "QQQ", AP_HAL::micros64(),
                                        _next_first, first_sample);
            soop_dsp_init(_dsp);          //a gap: the filters and history would straddle it
            soop_dsp_set_lo(_dsp, _lo_off);
        }
        _dsp_base = first_sample;         //the dsp counts from here
        _have_first = true;
    }
    _next_first = first_sample + SOOP_CAPTURE_BLOCK;
    //where the tracked satellites should
    if ((first_sample / SOOP_CAPTURE_BLOCK) % 4 == 0) {
        float pf[SOOP_DSP_MAX_AID], ps[SOOP_DSP_MAX_AID];
        soop_dsp_aid(_dsp, pf, ps, soop_nav_predict(_nav, _t0 + first_sample / _fs, pf, ps,
                                                    SOOP_DSP_MAX_AID));
    }
    const uint32_t t_start = AP_HAL::micros();
    const size_t n = soop_dsp_push(_dsp, iq, SOOP_CAPTURE_BLOCK, out, 8);
    _cpu_us += AP_HAL::micros() - t_start;
    _samples_s += SOOP_CAPTURE_BLOCK;
    for (size_t i = 0; i < n; i++) {
        if (_bq_n == AP_SOOP_BURST_QUEUE) {
            break;
        }
        _heard_ms = AP_HAL::millis();
        burst_t &b = _bq[_bq_n++];
        b.t = _t0 + (_dsp_base + out[i].t_sample) / _fs;
        b.f = out[i].f_hz;
        b.sigma = out[i].f_sigma_hz;
        b.cn0 = out[i].cn0_dbhz;
        b.sv = out[i].ira ? out[i].ra_sv : -1;
        if (out[i].cn0_dbhz >= ORI_CN0) {
            _ori_strong++;
            _ori_ira += out[i].ira;
        }
        if (out[i].ira) {
            AP::logger().WriteStreaming("SOR", "TimeUS,SV,Beam,X,Y,Z,Fix", "QBBhhhB", AP_HAL::micros64(),
                                        out[i].ra_sv, out[i].ra_beam, out[i].ra_pos[0],
                                        out[i].ra_pos[1], out[i].ra_pos[2], out[i].ra_fixed);
        }
        _bursts_s++;
    }
    //the wrong way up, Ring Alert sits outside the band searched and only its image (~25 dB
    //down) and other channels' images come through, Doppler reversed: strong bursts and not one
    //decoded is that, so turn the spectrum over. Once decodes have come, only a long silence moves it
    if (!_replay && _ori_strong >= (_ori_locked ? ORI_LOCKED_N : ORI_N)) {
        if (_ori_ira == 0) {
            capture.set_swap(!capture.swapped());
            _ori_locked = false;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SoOP: no Ring Alert decoded in %u strong bursts - I/Q %s",
                          unsigned(_ori_strong), capture.swapped() ? "swapped" : "as wired");
        } else {
            _ori_locked = true;
        }
        AP::logger().WriteStreaming("SOW", "TimeUS,Swap,Strong,Ira,Lock", "QBHHB", AP_HAL::micros64(),
                                    uint8_t(capture.swapped()), _ori_strong, _ori_ira, uint8_t(_ori_locked));
        _ori_strong = _ori_ira = 0;
    }
    //bursts come out up to one simplex slot after their preamble
    const double newest = _t0 + (first_sample + SOOP_CAPTURE_BLOCK) / _fs;
    feed_until(newest - 0.05);
    if (newest - _last_fix_t >= 0.2) {
        publish(newest - 0.05);
        _last_fix_t = newest;
    }
    if (newest - _last_status_t >= 1.0) {
        log_status(newest);
        _last_status_t = newest;
    }
}

//vehicle samples and bursts, merged by time into the navigation
void AP_SoOP::feed_until(double horizon)
{
    while (true) {
        veh_t v;
        bool have_v = false;
        {
            WITH_SEMAPHORE(_sem);
            if (_veh_tail != _veh_head && _veh[_veh_tail].t <= horizon) {
                v = _veh[_veh_tail];
                have_v = true;
            }
        }
        int bi = -1;
        for (uint16_t i = 0; i < _bq_n; i++) {
            if (_bq[i].t <= horizon && (bi < 0 || _bq[i].t < _bq[bi].t)) {
                bi = i;
            }
        }
        if (bi >= 0 && (!have_v || _bq[bi].t < v.t)) {
            const burst_t b = _bq[bi];
            _bq[bi] = _bq[--_bq_n];
            const int res = soop_guard_burst_sv(_guard, b.t, b.f, b.sigma, b.sv);
            if (res == SOOP_B_FUSED) {
                _fused_ms = AP_HAL::millis();
            }
            AP::logger().WriteStreaming("SOB", "TimeUS,T,F,Sig,CN0,Res,Sat,Ch,Inn",
                                        "QdfffBIbf", AP_HAL::micros64(), b.t, b.f, b.sigma, b.cn0,
                                        uint8_t(res), uint32_t(res == SOOP_B_FUSED ? _nav->last_sat : 0),
                                        int8_t(res == SOOP_B_FUSED ? _nav->last_ch : -1),
                                        float(res == SOOP_B_FUSED ? _nav->last_y : 0.0f));
            continue;
        }
        if (!have_v) {
            return;
        }
        {
            WITH_SEMAPHORE(_sem);
            _veh_tail = (_veh_tail + 1) % AP_SOOP_VEH_QUEUE;
        }
        const bool seed = _gps_time_only && _start_keys == 0 && v.gps_ok;
        if ((_start_keys == 7 || seed) && !_start_done) {
            //the surveyed (or deliberately wrong) start, once, before anything else moves it.
            //Time only with none given (a moving test): the first good u-blox fix, at start_sigma
            double ecef[3];
            if (seed) {
                memcpy(ecef, v.gps_ecef, sizeof ecef);
                soop_geodetic(ecef, &_start_lla[0], &_start_lla[1], &_start_lla[2]);
            } else {
                soop_ecef(_start_lla[0], _start_lla[1], _start_lla[2], ecef);
            }
            const double sigma = _start_sigma > 0 ? _start_sigma : 5000.0;
            soop_guard_set_position(_guard, v.t, ecef, sigma);
            _start_done = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: start %.5f %.5f, %.0f m", _start_lla[0],
                          _start_lla[1], sigma);
        }
        if (v.armed && !_was_armed) {
            soop_guard_arm(_guard, v.t);    //from here the GPS is checked against Iridium
        }
        _was_armed = v.armed;
        if (v.vel_ok) {
            soop_guard_velocity(_guard, v.t, v.vn, v.ve, v.vd);
            if (v.still) {
                soop_guard_still(_guard, v.t);
            }
        }
        if (v.ind_ok) {
            soop_guard_velocity_ind(_guard, v.t, v.ivn, v.ive, v.ivd);
        }
        if (v.gps_ok && _gps_time_only) {
            //time only: the u-blox's position stays out, so what the log shows is Iridium alone
            soop_guard_set_time(_guard, v.t, v.gps_utc + (v.pps ? 0.0 : _nav->cfg.gps_lag_s));
        } else if (v.gps_ok) {
            //soop_nav_gps adds gps_lag_s; a PPS time already is the epoch's
            soop_guard_gps(_guard, v.t, v.gps_utc - (v.pps ? _nav->cfg.gps_lag_s : 0.0), v.gps_ecef);
            _guard_dist = float(_guard->dist_m);
            _guard_state = uint8_t(_guard->state);
            clock_learn(v.temp_c);
        }
        _temp_c = v.temp_c;
        if (v.baro_ok) {
            soop_guard_baro(_guard, v.t, v.baro_h);
        }
    }
}

void AP_SoOP::publish(double t)
{
    //UTC is only known once a time tag has arrived
    if (!_warned_stale && _nav->have_time && soop_nav_cat_stale(_nav)) {
        _warned_stale = true;
        const double utc = _nav->tu0 + _nav->tc_a + (1 + _nav->tc_b) * (t - _nav->tb0);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "SoOP: TLE catalogue stale - newest epoch %.1f d old (max %.0f d), %d satellites unusable",
                      (utc - soop_nav_cat_newest_epoch(_nav)) / 86400.0, _nav->cfg.max_tle_age_d,
                      _nav->n_cat);
    }
    //for the green blinks: once the clock is found it navigates, u-blox or not
    _stage = _nav->have_clock ? 3 : _gps_stage >= 2 && _nav->have_time ? 2 : MIN(_gps_stage, 1);
    soop_fix_t f;
    bool valid = soop_guard_fix(_guard, t, &f);
    if (_gps_time_only) {
        //a blind start is never anchored; it is a test, so offer the fix as GPS2 anyway, but only
        //while the u-blox is good: then the EKF stays on it (more satellites), and cannot switch
        //onto Iridium and feed its own velocity back in. SOF.V keeps the flight rule
        valid = _nav->have_clock && _nav->have_time && f.n_track > 0
                && f.hacc_m < _nav->cfg.fix_max_hacc && AP_HAL::millis() - _gps_good_ms < 2000;
    }
    //a log format holds at most 16 fields, and SOS has them: the still flag lives here
    AP::logger().WriteStreaming("SOF", "TimeUS,Lat,Lng,Alt,HAcc,VAcc,SAcc,NIS,NT,V,St",
                                "QLLfffffBBB", AP_HAL::micros64(),       //NIS is a float
                                int32_t(f.lat_deg * 10000000), int32_t(f.lon_deg * 10000000),
                                float(f.h_m - _undulation), f.hacc_m, f.vacc_m, f.sacc_ms, f.nis,
                                f.n_track, f.valid, uint8_t(_still));
    if (!valid || !_nav->have_time) {
        return;
    }
    AP_SoOPFix::fix_t out {};
    out.lat_deg = f.lat_deg;
    out.lon_deg = f.lon_deg;
    out.alt_m = f.h_m - _undulation;
    out.vn = f.vn;
    out.ve = f.ve;
    out.vd = f.vd;
    out.hacc_m = f.hacc_m;
    out.vacc_m = f.vacc_m;
    out.sacc_ms = f.sacc_ms;
    out.num_sats = f.n_track;
    out.fix_type = 3;
    //GPS time of the fix, from the navigation's own board-to-UTC mapping
    const double utc = _nav->tu0 + _nav->tc_a + (1 + _nav->tc_b) * (t - _nav->tb0);
    const uint64_t gps_ms = uint64_t((utc + 946728000.0L - GPS_UNIX_S + LEAP_S) * 1000);
    out.time_week = gps_ms / AP_MSEC_PER_WEEK;
    out.time_week_ms = gps_ms % AP_MSEC_PER_WEEK;
    out.epoch_ms = uint32_t(t * 1000);    //board time of the capture epoch the fix is for
    AP_SoOPFix::get_singleton()->set(out);
}

//the clock table: "temp_c offset_hz" per line
void AP_SoOP::clock_load(float temp_c)
{
    char path[40];
    snprintf(path, sizeof(path), "%s/clock.dat", SOOP_DIR);
    const int fd = AP::FS().open(path, O_RDONLY);
    if (fd < 0) {
        return;
    }
    char buf[2048];
    const int32_t n = AP::FS().read(fd, buf, sizeof(buf) - 1);
    AP::FS().close(fd);
    buf[n > 0 ? n : 0] = 0;
    char *save = nullptr;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(nullptr, "\n", &save)) {
        char *end = nullptr;
        const long k = strtol(line, &end, 10) - CLK_T0;
        const double hz = strtod(end, nullptr);
        if (end != line && k >= 0 && k < CLK_N) {
            _clk_tab[k] = hz;
            _clk_have[k] = 1;
        }
    }
    //the nearest learnt degree within 10, trusted less the further it is
    const int k0 = int(lroundf(temp_c)) - CLK_T0;
    for (int d = 0; d <= 10; d++) {
        for (int k : { k0 - d, k0 + d }) {
            if (k >= 0 && k < CLK_N && _clk_have[k]) {
                const double sd = 200.0 + 20.0 * d;
                soop_nav_clock_prior(&_guard->main, _clk_tab[k], sd);
                soop_nav_clock_prior(&_guard->free, _clk_tab[k], sd);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SoOP: clock %.0f Hz from %d C", double(_clk_tab[k]), k + CLK_T0);
                return;
            }
        }
    }
}

void AP_SoOP::clock_learn(float temp_c)
{
    //only a clock the sky has settled, not one this table seeded
    if (!_nav->have_clock || _nav->clk_from_prior || _nav->P[6][6] > 30.0 * 30.0) {
        return;
    }
    const int k = int(lroundf(temp_c)) - CLK_T0;
    if (k < 0 || k >= CLK_N) {
        return;
    }
    const float x = float(_nav->x[6]);
    _clk_tab[k] = _clk_have[k] ? 0.99f * _clk_tab[k] + 0.01f * x : x;
    _clk_have[k] = 1;
    _clk_dirty = true;
}

void AP_SoOP::clock_save(double t)
{
    if (!_clk_dirty || t - _clk_save_t < 60.0) {
        return;
    }
    char path[40];
    snprintf(path, sizeof(path), "%s/clock.dat", SOOP_DIR);
    const int fd = AP::FS().open(path, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd < 0) {
        return;
    }
    char line[32];
    for (int k = 0; k < CLK_N; k++) {
        if (_clk_have[k]) {
            const int n = snprintf(line, sizeof(line), "%d %.1f\n", k + CLK_T0, double(_clk_tab[k]));
            AP::FS().write(fd, line, n);
        }
    }
    AP::FS().close(fd);
    _clk_dirty = false;
    _clk_save_t = t;
}

void AP_SoOP::log_status(double t)
{
    clock_save(t);
    if (_iq_slots) {
        //the I/Q ring: blocks waiting for the card, never recorded, and written
        const uint32_t queued = (_iq_head + _iq_nslots - _iq_tail) % _iq_nslots;
        AP::logger().WriteStreaming("SOQ", "TimeUS,Q,D,W", "QIII", AP_HAL::micros64(),
                                    queued, _iq_dropped, _iq_written);
    }
    const float cpu = _samples_s ? 100.0f * _cpu_us / (1e6f * _samples_s / float(SOOP_FS)) : 0;
    if (_capturing) {
        tuner.check_lock();
    }
    //the noise floor the DSP has learnt, dB
    const float nse = _dsp->noise_init && _dsp->noise > 0 ? 10 * log10f(_dsp->noise) : 0;
    AP::logger().WriteStreaming("SOS", "TimeUS,CPU,Bps,Ovr,AErr,Lock,VT,NT,Clk,Fused,Amb,Unm,G,GD,GL,Nse",
                                "QfHIIBBBfIIIBfff", AP_HAL::micros64(), cpu, uint16_t(_bursts_s),
                                capture.overruns(), capture.adc_errors(), uint8_t(tuner.locked()), tuner.vtune_adc(),
                                uint8_t(_nav->n_track), float(_nav->x[6]),
                                _nav->count[SOOP_B_FUSED], _nav->count[SOOP_B_AMBIGUOUS],
                                _nav->count[SOOP_B_UNMATCHED], uint8_t(_guard->state),
                                float(_guard->dist_m), float(_guard->limit_m), nse);
    _cpu_us = 0;
    _samples_s = 0;
    _bursts_s = 0;
    (void)t;
}
