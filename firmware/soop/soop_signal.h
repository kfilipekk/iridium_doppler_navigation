#ifndef SOOP_SIGNAL_H
#define SOOP_SIGNAL_H

#define SOOP_SYMBOL_RATE     25000.0      //symbols/s
#define SOOP_CH_SPACING      41666.667    //Hz between simplex channels
#define SOOP_F_RING_ALERT    1626270833.0 //Hz, Ring Alert (IRA)
#define SOOP_N_SIMPLEX       3            //Ring Alert and the channel either side: what the
#define SOOP_RA_CH           1            //+-52 kHz the DSP searches holds (2026-11-20: bursts on
                                          //RA+41.7 kHz and RA-41.7 kHz; the messaging channels are
                                          //above Ring Alert, not below)
#define SOOP_PREAMBLE_SYMS   64           //downlink carrier preamble
#define SOOP_MAX_BURST_S     0.02032      //the simplex slot

#define SOOP_FS              500000.0     //complex samples/s from the ADC pair
#define SOOP_LO_OFFSET_HZ    62500.0      //LO above Ring Alert by default; soop.cfg lo_offset_hz moves it

#endif //SOOP_SIGNAL_H
