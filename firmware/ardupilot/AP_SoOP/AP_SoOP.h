//AP_SoOP - position from Iridium, on the aircraft
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_SoOP_Capture.h"      //SOOP_CAPTURE_BLOCK

extern "C" {
#include "soop_ant.h"
#include "soop_dsp.h"
#include "soop_guard.h"
}

#define AP_SOOP_VEH_QUEUE 64
#define AP_SOOP_IND_FLOW_Q 100           //flow quality below which its velocity is not used
#define AP_SOOP_BURST_QUEUE 32

class AP_SoOP
{
public:
    static AP_SoOP *get_singleton();

    //main thread, from AP_GPS_SoOP::read()
    void update();

private:
    struct veh_t {
        double t;                //board s
        float  vn, ve, vd;       //flight-EKF velocity, NED
        bool   vel_ok;
        float  baro_h;           //ellipsoid-referenced height from the baro
        bool   baro_ok;
        bool   gps_ok;           //the real GPS, trusted this instant
        bool   armed;
        double gps_utc;          //UTC s since J2000
        double gps_ecef[3];
        float  temp_c;           //IMU die temperature, the TCXO's nearest thermometer
        float  ivn, ive, ivd;    //over the ground from flow and rangefinder: GPS can't drag it
        bool   ind_ok;
        bool   still;            //disarmed with the IMU at rest: the velocity is zero
        bool   pps;              //gps_utc is from the PPS, exact: no lag to add
    };
    struct burst_t {
        double t;
        float  f, sigma, cn0;
        int16_t sv;              //the satellite a decoded Ring Alert names (-1: not decoded)
    };

    void thread_main();
    bool setup();
    void load_config();
    int  load_catalogue();
    void run_block(const int16_t *iq, uint64_t first_sample);
    void feed_until(double horizon);
    void publish(double t);
    void log_status(double t);
    void ant_update(double t);
    void led_tick();
    void pps_isr(uint8_t pin, bool high, uint32_t t_us);
    void iq_record(const int16_t *iq, uint64_t first_sample);
    void iq_writer();
    bool iq_setup();
    struct iq_slot_t;
    iq_slot_t *iq_claim(uint8_t **data);
    void iq_commit();
    void clock_load(float temp_c);
    void clock_learn(float temp_c);
    void clock_save(double t);

    bool     _started;
    //the board LEDs, once NTF_LED_TYPES leaves them: blue dim and steady for power, green a
    //flash for each burst fused to a satellite
    bool     _leds;
    uint8_t  _led_n;
    volatile uint32_t _fused_ms;
    volatile uint32_t _heard_ms;  //the DSP's last burst, matched or not
    volatile uint8_t _gps_stage;  //0 no u-blox fix, 1 a fix not yet good enough, 2 good
    volatile uint8_t _stage;      //green's start-up count, 0-3
    HAL_Semaphore _sem;
    veh_t    _veh[AP_SOOP_VEH_QUEUE];
    uint16_t _veh_head, _veh_tail;
    burst_t  _bq[AP_SOOP_BURST_QUEUE];
    uint16_t _bq_n;

    //set up in the thread
    soop_dsp_t *_dsp;
    soop_guard_t *_guard;
    soop_nav_t *_nav;            //the guard's main filter
    soop_guard_cfg_t _gcfg;
    bool       _was_armed;
    volatile uint8_t _guard_state;   //SOOP_G_*, written by the thread
    volatile float _guard_dist;      //m, the GPS from the GPS-free filter
    bool       _spoof_handled;
    bool       _warned_stale;        //the all-stale TLE warning has been sent once
    soop_sat_t *_cat;
    int16_t    *_block;
    int        _n_cat;
    bool       _replay, _swap_iq, _capturing;
    //the spectrum's orientation, checked by Ring Alert decodes: they only succeed the right way up
    uint16_t   _ori_strong, _ori_ira;
    bool       _ori_locked;
    uint8_t    _bbg;
    float      _lo_off;          //Hz, the LO above Ring Alert
    double     _t0;              //board s of capture sample 0
    uint64_t   _dsp_base;        //capture sample where the dsp last started counting
    uint64_t   _next_first;      //the block that follows without a gap
    bool       _have_first;
    double     _fs;
    double     _undulation;      //m, geoid above ellipsoid, from the GPS when it says
    float      _baro_offset;     //ellipsoidal height minus baro altitude, learnt on GPS
    double     _still_since = -1;//board s the IMU came to rest, -1 while it moves
    float      _acc_m, _acc_m2;  //running mean and mean square of |accel|: its steadiness
    bool       _moving_latch;    //the u-blox saw it move; only the u-blox can say it stopped
    bool       _still;
    bool       _have_baro_offset;
    double     _last_fix_t, _last_status_t;
    uint32_t   _cpu_us, _bursts_s;
    //antenna feed
    soop_ant_t _ant;
    bool       _ant_started;
    int        _ant_reported;
    double     _ant_log_t;
    float      _ant_ma;
    AP_HAL::AnalogSource *_ant_adc;
    volatile bool _ant_feed_off;
    volatile bool _ind_vel;          //feed the guard flow-derived velocity (soop.cfg ind_vel)
    //a blind start, for measuring Iridium alone: the u-blox gives time only (soop.cfg gps_pos 0)
    //and the navigation starts from start_lat/lon/h, start_sigma m
    bool       _gps_time_only;
    double     _start_lla[3], _start_sigma;
    uint8_t    _start_keys;      //which of start_lat, start_lon, start_h were given
    bool       _start_done;
    bool       _no_still;        //soop.cfg still 0: a vehicle that cruises disarmed (an airliner)
    //the u-blox's PPS on TP3 (PB9, GPIO 54; SERVO5_FUNCTION -1), soop.cfg pps 1: each rising
    //edge, stamped in its interrupt, is a whole UTC second
    bool       _pps, _pps_armed;
    volatile uint32_t _pps_edge_us, _pps_edges;
    uint32_t   _pps_used;        //edges already paired with a second
    double     _pps_tb, _pps_utc;//the last paired edge: board s, UTC s since J2000
    //raw I/Q snippets: iq_rec_s seconds every iq_every_s (soop.cfg; 0 = off)
    double     _iq_rec_s, _iq_every_s, _iq_next_t;
    uint32_t   _iq_left;         //blocks still to queue in this recording
    bool       _iq_open, _iq_close_pending;
    //the ring between the capture thread, which fills it, and iq_writer, which alone touches
    //the card: a card stall waits there, never in the capture loop
    iq_slot_t *_iq_slots;         //what the writer is to do, one a slot
    uint8_t   *_iq_data;          //each slot's record, back to back, so a run of DATA slots is
                                  //one contiguous write
    uint32_t   _iq_rec_bytes, _iq_nslots;
    volatile uint32_t _iq_head, _iq_tail;
    uint32_t   _iq_dropped, _iq_written; //blocks the ring had no room for; blocks written
    int        _iq_fd = -1;      //the writer's
    bool       _iq_skip;         //the writer's: this recording found the card too full
    uint32_t   _iq_since_sync;
    int16_t    _iq_n = -1;       //file number, -1 until the first free one is found
    uint8_t    _iq_bits = 4;     //4: soop_iq4 records (.iq4), 16: raw int16 (.bin)
    double     _iq_fast_mps;     //continuous while the u-blox has seen this speed (0: off)
    bool       _iq_fast;         //latched: seen over iq_fast_mps, not yet under half of it
    volatile float _gps_spd;     //the u-blox's ground speed when last good, m/s
    volatile uint32_t _gps_good_ms; //when the u-blox last passed the 'good' test
    uint64_t   _samples_s;
    //the TCXO's offset per degree, remembered across flights in clock.dat
    static const int CLK_T0 = -20, CLK_N = 100;
    float      _clk_tab[CLK_N];
    uint8_t    _clk_have[CLK_N];
    bool       _clk_dirty;
    double     _clk_save_t;
    float      _temp_c;
};

namespace AP {
    AP_SoOP *soop();
};
