#!/usr/bin/env python3
"""Record a field session over MAVLink: both receivers once a second, and GPS denial on a plan.

Usage:
  soop_session.py OUT.csv PLAN

PLAN is comma-separated steps: on:S and off:S hold GPS on or denied for S seconds;
until:HH:MM holds the current state until that local time; off_until:HH:MM denies until then.
Denial is ArduPilot's GPS-disable aux function (MAV_CMD_DO_AUX_FUNCTION, 65), which the next
reboot clears. Example, two denials with GPS back between them:
  soop_session.py s.csv on:120,off:1800,on:300,off:1500,on:240
Watch it with soop_watch.py; reduce it with soop_fieldlog.py.
"""
import sys, time, datetime, csv
from pymavlink import mavutil
import glob
dev=(glob.glob("/dev/serial/by-id/*NAVCORE_SoOP_*") or ["/dev/ttyACM0"])[0]
m=mavutil.mavlink_connection(dev); m.wait_heartbeat(timeout=20)
m.mav.request_data_stream_send(m.target_system,m.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,2,1)
out=open(sys.argv[1],"a",newline=""); w=csv.writer(out)
def now(): return datetime.datetime.now(datetime.timezone.utc).strftime("%H:%M:%S.%f")[:-3]
def mark(s): print(now(), s, flush=True); w.writerow([now(),"MARK",s]); out.flush()
def gps(disable):
    m.mav.command_long_send(m.target_system,m.target_component,218,0,65,2 if disable else 0,0,0,0,0,0)
    mark("GPS DISABLED" if disable else "GPS ENABLED")
def pump(seconds):
    end=time.time()+seconds
    while time.time()<end:
        x=m.recv_match(blocking=True,timeout=1)
        if not x: continue
        t=x.get_type()
        if t=="GPS_RAW_INT": w.writerow([now(),"GPS1",x.fix_type,x.lat/1e7,x.lon/1e7,x.h_acc/1000,x.satellites_visible])
        elif t=="GPS2_RAW": w.writerow([now(),"GPS2",x.fix_type,x.lat/1e7,x.lon/1e7,x.h_acc/1000,x.satellites_visible])
        elif t=="GLOBAL_POSITION_INT": w.writerow([now(),"EKF",0,x.lat/1e7,x.lon/1e7,0,0])
        elif t=="COMMAND_ACK" and x.command==218: mark(f"ack {x.result}")
        elif t=="STATUSTEXT" and "PreArm" not in x.text: mark("TEXT "+x.text)
        out.flush()
for step in sys.argv[2].split(","):
    k,v=step.split(":",1)
    if k=="on": gps(False); pump(float(v))
    elif k=="off": gps(True); pump(float(v))
    elif k in ("until","off_until"):
        if k=="off_until": gps(True)
        hh,mm=map(int,v.split(":")); t=datetime.datetime.now().replace(hour=hh,minute=mm,second=0,microsecond=0)
        pump(max(0,(t-datetime.datetime.now()).total_seconds()))
gps(False); mark("DONE")
