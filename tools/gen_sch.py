#!/usr/bin/env python3
#generate the schematic from tools/design.py: a top sheet and one sheet per block.
#
#Each IC sits in a box with the passives that serve it. Rails are power symbols; signals are
#global labels, so every net keeps the name design.py gives it on every sheet.
import os, sys, re, uuid
from collections import Counter, defaultdict
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design
from kicadtools import project, symlib

ROOT  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JLC   = project.SYMBOLS
STOCK = "/usr/share/kicad/symbols"
NAME  = "NAVCORE-SoOP"
REV, DATE = "E", "2026-11-01"
U = lambda: str(uuid.uuid4())
GRID = 1.27
STUB = 2.54     #wire from a pin end to its rail symbol or label
snap = lambda v: round(v / GRID) * GRID

#rails drawn as power symbols
POWER = ["GND", "VBAT_IN", "VBAT", "+5V_BUCK", "+5V", "+5V_PAYLOAD", "VBUS", "+3V3", "+3V3A",
         "+3V3_CAN", "VDDA", "VCC_RF"]
#rails KiCad's own power library has; the rest go in libraries/symbols/navcore_power.kicad_sym
STOCK_RAILS = {"GND", "+3V3", "+5V", "VBUS", "VDDA"}
POWER_LIB = os.path.join(ROOT, "libraries", "symbols", "navcore_power.kicad_sym")
rail_id = lambda rail: ("power:" if rail in STOCK_RAILS else "navcore_power:") + rail

#(file, sheet name, notes, [(host part, box title)])
SHEETS = [
    ("power", "Power", [
        "3S-5S in on J2. TVS, then Q4 blocks a reversed pack.",
        "U8 makes +5V for the board, U20 a separate +5V for the payload, so servos and LEDs",
        "can't pull the core rail down. U31 stops USB back-feeding the battery lead.",
        "+3V3A is its own LDO for the IMUs, baro and tuner."],
     [("D1", "Input TVS"), ("Q4", "Reverse-battery FET"), ("DZ1", None), ("U8", "+5V buck"),
      ("U31", "Ideal diode, buck to +5V"), ("D_USB", "USB power"), ("U20", "+5V payload buck"),
      ("Q3", "Payload enable"), ("U9", "+3V3 LDO"), ("U10", "+3V3A LDO (analogue)")]
     + [(f"PF{i}", "Power flags" if i == 1 else None) for i in range(1, 12) if i not in (6, 8)]),
    ("mcu", "MCU", [
        "STM32H743, 8 MHz crystal, one 100n per VDD pin.",
        "VDDA and VREF+ are filtered from +3V3 through L1."],
     [("U1", "STM32H743VIT6"), ("Y1", "8 MHz crystal"), ("SW1", "Boot"), ("SW2", "Reset"),
      ("J19", "SWD (TC2030)"), ("D2", "Status LEDs"), ("D3", None), ("Q1", "Buzzer driver"),
      ("D4", None), ("P161", "Buzzer pads"), ("P162", None), ("P163", None),
      ("U17", "LED strip buffer, 3.3 to 5 V"), ("P151", "LED strip pads"), ("P152", None),
      ("P153", None), ("TP3", "Spare PWM"), ("TP4", None), ("FID1", "Fiducials")]),
    ("receiver", "L-band receiver", [
        "Iridium Ring Alert at 1626.27 MHz. The MAX2112 mixes straight to baseband;",
        "the OPA2374 difference amps filter I and Q into the ADCs (PC4, PA4).",
        "Antenna bias: +5V through U24's current limit, U25 reads the current."],
     [("U13", "Tuner, MAX2112"), ("Y2", "25 MHz TCXO"), ("U14", "Baseband amp, OPA2374"),
      ("J12", "Antenna (MMCX)"), ("D7", None), ("U24", "Antenna bias switch"),
      ("U25", "Bias current sense"), ("TP9", "I/Q test points"), ("TP10", None)]),
    ("sensors", "Sensors and storage", [
        "Both IMUs and the baro run from +3V3A.",
        "IMU1 on SPI1, IMU2 on SPI4, baro and tuner share I2C2."],
     [("U2", "IMU1, ICM-45686"), ("U3", "IMU2, ICM-42605"), ("U4", "Barometer, MS5611"),
      ("U19", "Board temperature, TMP119"), ("U5", "Flash, W25Q128"), ("J8", "microSD")]),
    ("connectors", "Connectors and protection", [
        "Every lead that leaves the board has an ESD clamp, and the motor,",
        "telemetry, current, lidar and servo lines a series resistor."],
     [("J1", "USB-C"), ("U12", None), ("J2", "ESC"), ("U26", None), ("U27", None),
      ("J3", "GPS and I2C"), ("U22", None), ("J9", "I2C"), ("J5", "RC in"), ("U23", None),
      ("J6", "CAN"), ("U11", "CAN transceiver"), ("U21", "CAN 3.3 V LDO"), ("D6", None),
      ("J11", "Serial 2"), ("U30", None), ("J14", "Optical flow"), ("U29", None),
      ("J21", "Companion computer"), ("U28", None), ("J22", "Payload 5 V"),
      ("J17", "Servos"), ("J23", None), ("D8", None), ("D9", None), ("D10", None)]),
]
#parts the nets can't place: the tuner rail's filter, and the fiducials
HOST_FIX = {"C49": "U13", "C50": "U13", "C51": "U13", "C84": "U13", "C86": "U13",
            "R28": "U13", "FID2": "FID1", "FID3": "FID1",
            "PF6": "U1", "PF8": "U13"}     #the VDDA and VCC_RF flags sit where those rails are made
#a box that collects other hosts' parts: (box host) <- hosts merged into it
MERGE = {"Q4": ["DZ1"], "D2": ["D3"], "Q1": ["D4"], "P161": ["P162", "P163"],
         "P151": ["P152", "P153"], "TP3": ["TP4"], "D7": [], "J12": ["D7"],
         "TP9": ["TP10"], "J1": ["U12"], "J2": ["U26", "U27"],
         "J3": ["U22"], "J5": ["U23"], "J6": ["D6"], "J11": ["U30"], "J14": ["U29"],
         "J21": ["U28"], "J17": ["J23", "D8", "D9", "D10"], "PF1": [f"PF{i}" for i in range(2, 12) if i not in (6, 8)]}


#return the verbatim (symbol "name" ...) block from a .kicad_sym file.
def raw_symbol(libpath, name):
    s = open(libpath).read()
    m = re.search(r'\(symbol "%s"' % re.escape(name), s)
    if not m: return None
    d, i = 0, m.start()
    while i < len(s):
        if s[i] == '(': d += 1
        elif s[i] == ')':
            d -= 1
            if d == 0: break
        i += 1
    return s[m.start():i+1]

#the top-level (...) elements of a symbol block, in order.
def _parts(blk):
    out, d, start = [], 0, None
    for i, c in enumerate(blk):
        if c == '(':
            d += 1
            if d == 2: start = i
        elif c == ')':
            if d == 2: out.append(blk[start:i+1])
            d -= 1
    return out

#the symbol block with any (extends ...) resolved.
def flat_symbol(libpath, name):
    blk = raw_symbol(libpath, name)
    m = blk and re.search(r'\(extends "([^"]+)"\)', blk[:400])
    if not m: return blk
    parent = flat_symbol(libpath, m.group(1))
    prop = lambda e: re.match(r'\(property "([^"]+)"', e)
    mine = {prop(e).group(1): e for e in _parts(blk) if prop(e)}
    body = []
    for e in _parts(parent):
        if prop(e):
            body.append(mine.pop(prop(e).group(1), e))
        else:
            body.append(e.replace('(symbol "%s_' % m.group(1), '(symbol "%s_' % name))
    body[0:0] = list(mine.values())
    return '(symbol "%s"\n\t' % name + "\n\t".join(body) + "\n)"

#lib_id -> (raw block, pin list).
def collect():
    out, syms_jlc = {}, symlib.load()
    for ref, (lib_id, fp, val, lcsc, dnp) in design.COMPONENTS.items():
        if lib_id in out: continue
        nick, name = lib_id.split(":", 1)
        path = JLC if nick == "jlc_parts" else f"{STOCK}/{nick}.kicad_sym"
        blk = flat_symbol(path, name)
        if blk is None:
            raise SystemExit(f"symbol {lib_id} not found in {path}")
        blk = blk.replace('(symbol "%s"' % name, '(symbol "%s"' % lib_id, 1)
        pins = (syms_jlc.get(name) if nick == "jlc_parts" else symlib.load(path).get(name)) or []
        out[lib_id] = (blk, pins)
    return out

#{lib_id: block} for every rail. KiCad's own symbol where it has one; the others are its
#+3V3 arrow renamed, since a power symbol's value is its net, and are written out as a
#project library so the schematic's copies match a library.
def power_symbols():
    path = f"{STOCK}/power.kicad_sym"
    up = raw_symbol(path, "+3V3")
    out, custom = {}, []
    for rail in POWER:
        if rail in STOCK_RAILS:
            blk = raw_symbol(path, rail)
        else:
            blk = (up.replace('(symbol "+3V3_', '(symbol "%s_' % rail)
                     .replace('(symbol "+3V3"', '(symbol "%s"' % rail, 1)
                     .replace('(property "Value" "+3V3"', '(property "Value" "%s"' % rail)
                     .replace('name \\"+3V3\\"', 'name \\"%s\\"' % rail))
            custom.append(blk)
        out[rail_id(rail)] = blk.replace('(symbol "%s"' % rail, '(symbol "%s"' % rail_id(rail), 1)
    open(POWER_LIB, "w").write('(kicad_symbol_lib\n\t(version 20231120)\n\t(generator "navcore-gen")\n\t'
                               + "\n\t".join(b.replace("\n", "\n\t") for b in custom) + "\n)\n")
    return out

def resolve_pin(lib_id, nums, spec, lib):
    if spec in nums: return spec
    return symlib.resolve(lib[lib_id][1], spec)


#A library vector (y up) turned r degrees, as a sheet offset (y down).
def rotate(x, y, r):
    r %= 360
    qx, qy = {0: (x, y), 90: (-y, x), 180: (-x, -y), 270: (y, -x)}[r]
    return qx, -qy

#the way a pin's free end faces on the sheet, from its library angle and the symbol's rotation:
#a pin's angle points from its end into the body, so the end faces the other way
OUT = {0: "L", 90: "D", 180: "R", 270: "U"}
def facing(pin_rot, sym_rot):
    return OUT[int(round(pin_rot + sym_rot)) % 360]


class Sheet:
    npwr = 0
    def __init__(self, fname, title, lib, pidx, pin_net, powers):
        self.fname, self.title = fname, title
        self.lib, self.pidx, self.pin_net, self.powers = lib, pidx, pin_net, powers
        self.items, self.used = [], set()

    def text(self, s, x, y, size=1.27, bold=False):
        s = s.replace('"', "'")
        self.items.append(f'\t(text "{s}" (exclude_from_sim no) (at {x:.2f} {y:.2f} 0)'
                          f' (effects (font (size {size} {size}){" (bold yes)" if bold else ""})'
                          f' (justify left bottom)) (uuid "{U()}"))')

    def box(self, x0, y0, x1, y1):
        self.items.append(f'\t(rectangle (start {x0:.2f} {y0:.2f}) (end {x1:.2f} {y1:.2f})'
                          ' (stroke (width 0.15) (type dash) (color 120 120 120 1))'
                          f' (fill (type none)) (uuid "{U()}"))')

    #A rail's symbol on a pin end, pointing away from the part.
    def power(self, rail, x, y, face):
        self.used.add(rail_id(rail))
        down = rail == "GND"
        rot = ({"D": 0, "R": 90, "U": 180, "L": 270} if down else
               {"U": 0, "L": 90, "D": 180, "R": 270})[face]
        #the name at the arrow's tip, whichever way it points
        tx, ty, tj = {"U": (x, y - 3.6, ""), "D": (x, y + 4.4, ""),
                      "L": (x - 5.3, y + 0.4, " (justify right)"),
                      "R": (x + 5.3, y + 0.4, " (justify left)")}[face]
        Sheet.npwr += 1
        ref = f"#PWR{Sheet.npwr:04d}"
        self.items += [f'\t(symbol (lib_id "{rail_id(rail)}") (at {x:.2f} {y:.2f} {rot}) (unit 1)',
                       '\t\t(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)',
                       f'\t\t(uuid "{U()}")',
                       f'\t\t(property "Reference" "{ref}" (at {x:.2f} {y:.2f} 0)'
                       ' (effects (font (size 1.27 1.27)) hide))',
                       f'\t\t(property "Value" "{rail}" (at {tx:.2f} {ty:.2f} {90 if rot in (90, 270) else 0})'
                       f' (effects (font (size 1.0 1.0)){tj}))',
                       f'\t\t(pin "1" (uuid "{U()}"))',
                       f'\t\t(instances (project "{NAME}" (path "{self.path}" (reference "{ref}") (unit 1))))',
                       '\t)']

    def label(self, net, x, y, face):
        rot = {"R": 0, "U": 90, "L": 180, "D": 270}[face]
        just = "left" if face in ("R", "U") else "right"
        self.items += [f'\t(global_label "{net}" (shape passive) (at {x:.2f} {y:.2f} {rot})',
                       f'\t\t(effects (font (size 1.0 1.0)) (justify {just}))',
                       f'\t\t(uuid "{U()}")', '\t)']

    def wire(self, x0, y0, x1, y1):
        self.items.append(f'\t(wire (pts (xy {x0:.2f} {y0:.2f}) (xy {x1:.2f} {y1:.2f}))'
                          f' (stroke (width 0) (type default)) (uuid "{U()}"))')

    def nc(self, x, y):
        self.items.append(f'\t(no_connect (at {x:.2f} {y:.2f}) (uuid "{U()}"))')

    #place one unit of a part and dress every pin end with its rail, label or X.
    def part(self, ref, cx, cy, rot=0, unit=1):
        lid, fp, val, lcsc, dnp = design.COMPONENTS[ref]
        self.used.add(lid)
        pins = units_of(self.pidx[lid], unit)
        xs = [rotate(p[0], p[1], rot)[0] for p in pins.values()] or [0]
        ys = [rotate(p[0], p[1], rot)[1] for p in pins.values()] or [0]
        two = len(self.pidx[lid]) <= 2
        if two and rot in (90, 270):                     #horizontal: names above and below
            rp, vp = (cx, cy - 3.0, "center"), (cx, cy + 4.0, "center")
        elif two:                                        #vertical: names to the right
            rp, vp = (cx + 2.2, cy - 0.6, "left"), (cx + 2.2, cy + 1.6, "left")
        else:
            rp = (cx + min(xs) + 2.54, cy + min(ys) - 4.2, "left")
            vp = (cx + min(xs) + 2.54, cy + max(ys) + 4.4, "left")
        fa = 90 if rot in (90, 270) else 0      #KiCad turns a field with its part, so undo it
        in_bom = "no" if (ref.startswith(("FID", "LOGO")) or lid in ("Connector:TestPoint", "Connector:TC2030")) else "yes"
        just = lambda j: "" if j == "center" else f" (justify {j})"
        self.items += [f'\t(symbol (lib_id "{lid}") (at {cx:.2f} {cy:.2f} {rot}) (unit {unit})',
                       f'\t\t(exclude_from_sim no) (in_bom {in_bom}) (on_board yes)'
                       f' (dnp {"yes" if dnp else "no"})',
                       f'\t\t(uuid "{U()}")',
                       f'\t\t(property "Reference" "{ref}" (at {rp[0]:.2f} {rp[1]:.2f} {fa})'
                       f' (effects (font (size 1.27 1.27)){just(rp[2])}))',
                       f'\t\t(property "Value" "{val}" (at {vp[0]:.2f} {vp[1]:.2f} {fa})'
                       f' (effects (font (size 1.0 1.0)){just(vp[2])}))',
                       f'\t\t(property "Footprint" "{fp}" (at {cx:.2f} {cy:.2f} 0)'
                       ' (effects (font (size 1.27 1.27)) hide))',
                       f'\t\t(property "LCSC" "{lcsc}" (at {cx:.2f} {cy:.2f} 0)'
                       ' (effects (font (size 1.27 1.27)) hide))']
        for num in pins:
            self.items.append(f'\t\t(pin "{num}" (uuid "{U()}"))')
        self.items += [f'\t\t(instances (project "{NAME}" (path "{self.path}"'
                       f' (reference "{ref}") (unit {unit}))))', '\t)']
        for num, (px, py, prot) in pins.items():
            dx, dy = rotate(px, py, rot)
            ax, ay = snap(cx + dx), snap(cy + dy)
            face = facing(prot, rot)
            net = self.pin_net.get((ref, num))
            step = {"L": (-STUB, 0), "R": (STUB, 0), "U": (0, -STUB), "D": (0, STUB)}[face]
            if net is None:
                self.nc(ax, ay)
            elif net in POWER:
                if not two:                      #clear of the pin number first
                    self.wire(ax, ay, ax + step[0], ay + step[1])
                    ax, ay = ax + step[0], ay + step[1]
                self.power(net, ax, ay, face)
            elif two and face in "UD":           #a stub, then the name reads across
                self.wire(ax, ay, ax, ay + step[1])
                self.label(net, ax, ay + step[1], "R")
            else:
                self.label(net, ax, ay, face)

    def write(self, sheet_uuid, page, lib_symbols, paper, notes):
        out = ['(kicad_sch', '\t(version 20231120)', '\t(generator "navcore-gen")',
               '\t(generator_version "9.0")', f'\t(uuid "{sheet_uuid}")', f'\t(paper "{paper}")',
               *title_block(self.title, page), '\t(lib_symbols']
        for lid in sorted(self.used):
            out.append("\t\t" + lib_symbols[lid].replace("\n", "\n\t\t"))
        out.append('\t)')
        out += self.items
        out += ['\t(embedded_fonts no)', ')']
        open(os.path.join(ROOT, self.fname), "w").write("\n".join(out) + "\n")


#{pin: (x, y, rot)} for one unit; unit 0 pins are on every unit.
def units_of(pidx_lid, unit):
    return {n: (x, y, r) for n, (x, y, r, u) in pidx_lid.items() if u in (0, unit)}

def unit_list(pidx_lid):
    us = sorted({u for (_x, _y, _r, u) in pidx_lid.values()} - {0})
    return us or [1]

def title_block(sheet, page):
    return ['\t(title_block', f'\t\t(title "NAVCORE-SoOP: {sheet}")', f'\t\t(date "{DATE}")',
            f'\t\t(rev "{REV}")', '\t\t(company "Krystian Filipek - University of Cambridge / CUSF")',
            '\t\t(comment 1 "6-layer STM32H743 flight controller with Iridium Doppler navigation")',
            '\t\t(comment 2 "Generated by tools/gen_sch.py from tools/design.py")', '\t)']


#the box a placed unit needs, pin ends plus the labels hanging off them.
def extent(lib, pidx, pin_net, ref, rot, unit):
    lid = design.COMPONENTS[ref][0]
    pins = units_of(pidx[lid], unit)
    if not pins: return -3, -3, 3, 3
    x0 = y0 = 1e9; x1 = y1 = -1e9
    for num, (px, py, prot) in pins.items():
        dx, dy = rotate(px, py, rot)
        net = pin_net.get((ref, num))
        f = facing(prot, rot)
        two_pin = len(pidx[lid]) <= 2
        if net is None: n = 0
        elif net in POWER: n = 4.5 + (0 if two_pin else STUB) + (0.85 * len(net) + 1 if f in "LR" else 0)
        else: n = 0.8 * len(net) + 3.5
        if two_pin and net and net not in POWER and f in "UD":
            ey = dy + (STUB + 1.5) * (1 if f == "D" else -1)
            ex = dx + 0.8 * len(net) + 3.5
            x0, x1, y0, y1 = min(x0, dx), max(x1, ex), min(y0, dy, ey), max(y1, dy, ey)
            continue
        ex, ey = dx + {"L": -n, "R": n}.get(f, 0), dy + {"U": -n, "D": n}.get(f, 0)
        x0, x1, y0, y1 = min(x0, dx, ex), max(x1, dx, ex), min(y0, dy, ey), max(y1, dy, ey)
    two = len(pidx[lid]) <= 2
    if two and rot in (0, 180):
        x1 = max(x1, 2.2 + 0.9 * max(len(ref), len(design.COMPONENTS[ref][2])))
        y0, y1 = min(y0, -6.5), max(y1, 6.5)
    if two and rot in (90, 270): y0, y1 = min(y0, -4.5), max(y1, 5.0)
    if not two: y0, y1 = y0 - 5.5, y1 + 5.5
    return x0 - 1.0, y0 - 1.0, x1 + 1.0, y1 + 1.0


def main():
    lib = collect()
    pidx = {lid: {p['num']: (p['x'], p['y'], p['rot'], p.get('unit', 1)) for p in pins}
            for lid, (blk, pins) in lib.items()}
    lib_symbols = {lid: blk for lid, (blk, pins) in lib.items()}
    lib_symbols.update(power_symbols())

    #net lookup: (ref, pin number) -> net; a net on one pin is left as a no-connect
    pin_net = {}
    for net, specs in design.NETS.items():
        if len(specs) == 1: continue
        for sp in specs:
            ref, pin = sp.split(".", 1)
            if ref not in design.COMPONENTS: continue
            lid = design.COMPONENTS[ref][0]
            num = resolve_pin(lid, pidx[lid], pin, lib)
            if num: pin_net[(ref, num)] = net

    #which host each part sits with: listed hosts, then design.ADJACENCY, then the signal
    #nets it shares
    gone = [h for _f, _t, _n, hl in SHEETS for h, _ in hl if h not in design.COMPONENTS]
    if gone: raise SystemExit(f"SHEETS lists parts design.py no longer has: {gone}")
    hosts = [h for _f, _t, _n, hl in SHEETS for h, _ in hl]
    host = {h: h for h in hosts}
    host.update(HOST_FIX)
    nets_of, members = defaultdict(set), defaultdict(set)
    for net, specs in design.NETS.items():
        for sp in specs:
            r = sp.split(".", 1)[0]
            nets_of[r].add(net); members[net].add(r)
    def find(r, depth=0):
        if r in host: return host[r]
        a = design.ADJACENCY.get(r)
        if a and depth < 5 and find(a[0], depth + 1): return find(a[0], depth + 1)
        votes = Counter(host[m] for n in sorted(nets_of[r] - set(POWER)) for m in sorted(members[n])
                        if m in host and m != r)
        return votes.most_common(1)[0][0] if votes else None
    for _ in range(4):
        for r in design.COMPONENTS:
            if r not in host and find(r): host[r] = find(r)
    stray = [r for r in design.COMPONENTS if r not in host]
    if stray: raise SystemExit(f"no sheet for {stray}: add them to HOST_FIX")
    for box, merged in MERGE.items():
        for m in merged:
            for r, h in list(host.items()):
                if h == m: host[r] = box

    root_uuid = U()
    sheet_uuids = [U() for _ in SHEETS]
    lines = []
    PAPERS = [("A4", 297, 210), ("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841)]
    for si, (fname, title, notes, hl) in enumerate(SHEETS):
        sh = Sheet(f"{NAME}-{fname}.kicad_sch", title, lib, pidx, pin_net, None)
        sh.path = f"/{root_uuid}/{sheet_uuids[si]}"
        #each box: its host, then the rest by kind and number
        boxes = []
        for h, btitle in hl:
            if h in sum(MERGE.values(), []): continue
            refs = [r for r in design.COMPONENTS if host[r] == h]
            order = lambda r: (r != h, r in sum(MERGE.values(), []) and r != h,
                               len(pidx[design.COMPONENTS[r][0]]) <= 2,
                               re.sub(r"\d", "", r), int(re.sub(r"\D", "", r) or 0))
            refs.sort(key=order)
            boxes.append((h, btitle, refs))
        placed = [layout_box(sh, lib, pidx, pin_net, b) for b in boxes]
        #pack the boxes in rows, choosing the smallest paper they fit
        #in reading order if it fits, else tallest first, on the smallest paper either way
        tallest = sorted(range(len(placed)), key=lambda i: -placed[i][1])
        for paper, W, H in PAPERS:
            top = 34.0 + 4.5 * len(notes)
            pos = pack(placed, W - 25.0, H, top)
            if pos: break
            p2 = pack([placed[i] for i in tallest], W - 25.0, H, top)
            if p2:
                pos = [None] * len(placed)
                for k, i in enumerate(tallest): pos[i] = p2[k]
                break
        sh.text(title, 15.0, 24.0, size=3.0, bold=True)
        for k, s in enumerate(notes):
            sh.text(s, 15.0, 31.0 + 4.5 * k, size=1.6)
        for (bw, bh, draw), (ox, oy) in zip(placed, pos):
            draw(ox, oy)
        sh.write(sheet_uuids[si], si + 2, lib_symbols, paper, notes)
        lines.append((fname, title, sh.fname, sheet_uuids[si], paper, len([r for r in design.COMPONENTS if host[r] in [b[0] for b in boxes]])))

    write_root(root_uuid, lines)
    n = sum(l[5] for l in lines)
    print(f"schematic: {len(design.COMPONENTS)} parts on {len(lines)} sheets "
          + ", ".join(f"{l[1]} {l[5]} ({l[4]})" for l in lines))
    if n != len(design.COMPONENTS):
        raise SystemExit(f"{len(design.COMPONENTS) - n} parts were not drawn")


#arrange one box: the host on the left, then its parts in columns. Returns its size and
#a function that draws it at a given corner.
def layout_box(sh, lib, pidx, pin_net, b):
    h, btitle, refs = b
    cells = []
    for r in refs:
        lid = design.COMPONENTS[r][0]
        two = len(pidx[lid]) <= 2
        rot = 0
        if two:
            nets = [pin_net.get((r, n)) for n in sorted(pidx[lid])]
            pw = [n in POWER for n in nets]
            if not any(pw) and all(nets): rot = 90                  #signal to signal: lie flat
            else:
                #the rail on top and ground at the bottom: pin 1 is the top pin unturned
                top = pidx[lid].get("1", (0, 1, 0, 1))[1] > 0
                p1, p2 = (pin_net.get((r, "1")), pin_net.get((r, "2")))
                want_top = p2 if (p1 == "GND" or (p2 in POWER and p2 != "GND" and p1 not in POWER)) else p1
                rot = 0 if (want_top == p1) == top else 180
        for u in unit_list(pidx[lid]):
            x0, y0, x1, y1 = extent(lib, pidx, pin_net, r, rot, u)
            cells.append((r, u, rot, x0, y0, x1, y1, two))
    #big parts in a row first, then the two-pin parts in a grid under them
    big = [c for c in cells if not c[7]]
    small = [c for c in cells if c[7]]
    place, x, rowh = [], 0.0, 0.0
    for c in big:
        r, u, rot, x0, y0, x1, y1, _ = c
        place.append((c, x - x0, -y0)); x += (x1 - x0) + 5.08; rowh = max(rowh, y1 - y0)
    W = max(x - 5.08, 0.0)
    y = rowh + (5.08 if big else 0.0)
    if small:
        #left to right, each part its own width, rows wrapping at a width that keeps the box
        #about 1.4 wide to 1 high
        import math
        sizes = [(c[5] - c[3] + 1.27, c[6] - c[4] + 1.27) for c in small]
        target = max(W, math.sqrt(sum(w * h for w, h in sizes) * 1.4), max(w for w, _ in sizes))
        cx, rowh, widest = 0.0, 0.0, 0.0
        for c, (w, h) in zip(small, sizes):
            if cx > 0 and cx + w > target:
                y += rowh; cx, rowh = 0.0, 0.0
            place.append((c, cx - c[3], y - c[4]))
            cx += w; rowh = max(rowh, h); widest = max(widest, cx)
        y += rowh
        W = max(W, widest)
    Hh = y
    pad, head = 3.81, (6.35 if btitle else 2.54)
    BW, BH = max(W + 2 * pad, 1.15 * len(btitle or "") + 6.0), Hh + head + pad
    def draw(ox, oy):
        if btitle:
            sh.box(ox, oy, ox + BW, oy + BH)
            sh.text(btitle, ox + 2.0, oy + 4.2, size=1.6, bold=True)
        for (r, u, rot, *_), px, py in place:
            sh.part(r, snap(ox + pad + px), snap(oy + head + py), rot, u)
    return BW, BH, draw


#drop each box into the lowest spot it fits (a skyline), keeping clear of the title
#block. None if they don't all fit.
def pack(placed, W, H, top):
    GAP, X0 = 6.35, 15.0
    sky = [(X0, W - X0, top)]                    #(x, width, y): the free edge, left to right
    out = []
    for bw, bh, _ in placed:
        best = None
        for k in range(len(sky)):
            x = sky[k][0]
            if x + bw > W: break
            #the highest edge under [x, x + bw]
            y, span = 0.0, 0.0
            for sx, sw, sy in sky[k:]:
                if sx >= x + bw: break
                y = max(y, sy)
            if y + bh > H - 12 or (x + bw > W - 110 and y + bh > H - 45):
                continue
            if best is None or (y, x) < (best[1], best[0]):
                best = (x, y)
        if best is None: return None
        x, y = best
        out.append((snap(x), snap(y)))
        #raise the skyline under the new box
        x1 = x + bw + GAP
        new = []
        for sx, sw, sy in sky:
            ex = sx + sw
            if ex <= x or sx >= x1:
                new.append((sx, sw, sy)); continue
            if sx < x: new.append((sx, x - sx, sy))
            if ex > x1: new.append((x1, ex - x1, sy))
        new.append((x, x1 - x, y + bh + GAP))
        sky = sorted(new)
    return out


def write_root(root_uuid, lines):
    out = ['(kicad_sch', '\t(version 20231120)', '\t(generator "navcore-gen")',
           '\t(generator_version "9.0")', f'\t(uuid "{root_uuid}")', '\t(paper "A4")',
           *title_block("overview", 1), '\t(lib_symbols)']
    t = lambda s, x, y, size=1.27, bold=False: out.append(
        f'\t(text "{s}" (exclude_from_sim no) (at {x:.2f} {y:.2f} 0) (effects (font (size {size} {size})'
        f'{" (bold yes)" if bold else ""}) (justify left bottom)) (uuid "{U()}"))')
    t("NAVCORE-SoOP Rev E", 20.32, 25.4, 3.0, True)
    t("46.5 x 47.2 mm, 6 layers. STM32H743 running ArduPilot, with an Iridium receiver", 20.32, 33.02, 1.6)
    t("so it can find its position from satellite Doppler when GPS is jammed or gone.", 20.32, 37.59, 1.6)
    DESC = {"power": "3S-5S in, +5V, +5V payload, +3V3, +3V3A",
            "mcu": "STM32H743, crystal, SWD, LEDs, buzzer",
            "receiver": "MAX2112 tuner, I/Q amp, antenna bias",
            "sensors": "Two IMUs, baro, temperature, flash, microSD",
            "connectors": "USB, ESC, GPS, RC, CAN, serial, servos, ESD"}
    for k, (fname, title, sfile, suuid, paper, n) in enumerate(lines):
        x, y = 20.32 + (k % 3) * 88.9, 55.88 + (k // 3) * 55.88
        out += [f'\t(sheet (at {x:.2f} {y:.2f}) (size 76.2 25.4) (fields_autoplaced yes)',
                '\t\t(stroke (width 0.1524) (type solid)) (fill (color 0 0 0 0.0000))',
                f'\t\t(uuid "{suuid}")',
                f'\t\t(property "Sheetname" "{title}" (at {x:.2f} {y - 0.7:.2f} 0)'
                ' (effects (font (size 1.8 1.8) (bold yes)) (justify left bottom)))',
                f'\t\t(property "Sheetfile" "{sfile}" (at {x:.2f} {y + 26.0:.2f} 0)'
                ' (effects (font (size 1.0 1.0)) (justify left top)))',
                f'\t\t(instances (project "{NAME}" (path "/{root_uuid}" (page "{k + 2}"))))', '\t)']
        t(DESC[fname], x + 2.54, y + 12.7, 1.4)
        t(f"{n} parts", x + 2.54, y + 19.05, 1.2)
    out += ['\t(sheet_instances (path "/" (page "1")))', '\t(embedded_fonts no)', ')']
    open(os.path.join(ROOT, f"{NAME}.kicad_sch"), "w").write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
