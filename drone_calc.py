#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DroneCalc Pro v1.0
Kapsamlı Animasyonlu Drone Hesaplama Uygulaması
ecalc.ch tarzında profesyonel drone performans analizi
"""

import tkinter as tk
from tkinter import ttk
import customtkinter as ctk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import math
import time

# ============================================================
# RENK TEMASI
# ============================================================
BG      = "#0D1117"
BG2     = "#161B22"
BG3     = "#1C2128"
BG4     = "#21262D"
ACCENT  = "#00D4FF"
ACCENT2 = "#39D353"
ACCENT3 = "#FF7B54"
ACCENT4 = "#DA70D6"
FG      = "#CDD9E5"
FG2     = "#768390"
BORDER  = "#30363D"
WARN    = "#FFD700"

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

INCHES_TO_M = 0.0254
AIR_DENSITY  = 1.225   # kg/m³
G_MS2        = 9.81    # m/s²

# ============================================================
# HESAPLAMA FONKSİYONLARI
# ============================================================

def calc_motor_rpm(kv: float, voltage: float) -> float:
    return kv * voltage

def calc_prop_thrust(d_in: float, p_in: float, rpm: float) -> float:
    """Statik itme (gram) – ampirik formül (ecalc benzeri)"""
    if rpm <= 0 or d_in <= 0 or p_in <= 0:
        return 0.0
    return (d_in**4 * p_in * (rpm / 1000.0)**2) / 1406.25

def calc_prop_power(d_in: float, p_in: float, rpm: float, eff: float = 0.65) -> float:
    """Pervane güç tüketimi (Watt) – momentum teorisi"""
    if rpm <= 0 or eff <= 0:
        return 0.0
    thrust_g = calc_prop_thrust(d_in, p_in, rpm)
    thrust_n  = thrust_g / 1000.0 * G_MS2
    pitch_v   = p_in * INCHES_TO_M * rpm / 60.0
    return max(0.0, thrust_n * pitch_v / eff)

def calc_flight_time(cap_mah: float, avg_a: float, eff: float = 0.80) -> float:
    """Uçuş süresi (dakika)"""
    if avg_a <= 0:
        return 0.0
    return (cap_mah * eff / 1000.0) / avg_a * 60.0

def esc_standard(required_a: float) -> int:
    for s in [10,12,15,20,25,30,35,40,45,50,60,80,100,120]:
        if s >= required_a:
            return s
    return int(required_a)

# ============================================================
# ANİMASYONLU WIDGET'LAR
# ============================================================

class DroneHeaderCanvas(tk.Canvas):
    """Animasyonlu drone + başlık"""
    def __init__(self, parent, **kw):
        super().__init__(parent, bg=BG, highlightthickness=0, **kw)
        self._angle    = 0.0
        self._hover    = 0.0
        self._hover_d  = 1.0
        self._particle_timer = 0
        self._particles = []
        self._after_id = None
        self.bind("<Configure>", lambda e: self._draw_bg())
        self.after(100, self._draw_bg)
        self.after(200, self._loop)

    def _draw_bg(self):
        self.delete("bg")
        w = self.winfo_width() or 1100
        h = self.winfo_height() or 160
        # Izgara
        for x in range(0, w, 50):
            self.create_line(x,0,x,h, fill="#111820", width=1, tags="bg")
        for y in range(0, h, 50):
            self.create_line(0,y,w,y, fill="#111820", width=1, tags="bg")
        # Başlık
        self.create_text(w//2, 22, text="DroneCalc Pro",
                         font=("Consolas",22,"bold"), fill=ACCENT, tags="bg")
        self.create_text(w//2, 44, text="Kapsamlı Drone Performans Hesaplama Sistemi",
                         font=("Consolas",9), fill=FG2, tags="bg")
        # Ayırıcı çizgi
        self.create_line(w//2-300, 55, w//2+300, 55,
                         fill=ACCENT, width=1, dash=(4,8), tags="bg")

    def _draw_drone(self):
        self.delete("drone")
        w = self.winfo_width() or 1100
        h = self.winfo_height() or 160
        cx = w // 2
        cy = h - 55 + self._hover

        arm = 50
        corners = []
        for a in (45, 135, 225, 315):
            rad = math.radians(a)
            corners.append((cx + arm*math.cos(rad), cy + arm*math.sin(rad)))

        # Kollar
        for mx, my in corners:
            self.create_line(cx, cy, mx, my,
                             fill=ACCENT, width=3, tags="drone")

        # Pervane blur halkası (hıza göre)
        for (mx, my) in corners:
            self.create_oval(mx-18, my-18, mx+18, my+18,
                             outline=ACCENT, width=1,
                             stipple="gray25", tags="drone")

        # Dönen kanatlar
        for i, (mx, my) in enumerate(corners):
            a0 = self._angle * (1 if i % 2 == 0 else -1) + i*90
            for b in range(2):
                ba = math.radians(a0 + b*90)
                r = 17
                x1 = mx + r*math.cos(ba);  y1 = my + r*math.sin(ba)
                x2 = mx - r*math.cos(ba);  y2 = my - r*math.sin(ba)
                col = ACCENT if i < 2 else ACCENT2
                self.create_line(x1, y1, x2, y2,
                                 fill=col, width=2, tags="drone")
            # Motor merkezi
            self.create_oval(mx-5, my-5, mx+5, my+5,
                             fill=BG3, outline=FG2, tags="drone")

        # Gövde
        self.create_oval(cx-14, cy-14, cx+14, cy+14,
                         fill=BG2, outline=ACCENT, width=2, tags="drone")
        # LED
        led_on = int(time.time()*4) % 3 != 0
        led_c  = ACCENT3 if led_on else "#402010"
        self.create_oval(cx-5, cy-5, cx+5, cy+5,
                         fill=led_c, outline="", tags="drone")

        # Gölge
        sh_y = h - 8
        opa  = max(0.2, 1 - abs(self._hover)/12)
        self.create_oval(cx-22*opa, sh_y-3, cx+22*opa, sh_y+3,
                         fill="#050810", outline="", tags="drone")

    def _loop(self):
        self._hover  += 0.4 * self._hover_d
        if abs(self._hover) >= 8:
            self._hover_d *= -1
        self._angle = (self._angle + 14) % 360
        self._draw_drone()
        self._after_id = self.after(33, self._loop)

    def stop(self):
        if self._after_id:
            self.after_cancel(self._after_id)


class PropCanvas(tk.Canvas):
    """Detaylı dönen pervane animasyonu"""
    def __init__(self, parent, w=220, h=220, **kw):
        super().__init__(parent, width=w, height=h,
                         bg=BG2, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self.cx = w//2; self.cy = h//2
        self._angle   = 0.0
        self._speed   = 8.0
        self._rpm_val = 15000.0
        self._after_id= None
        self._loop()

    def set_rpm(self, rpm: float):
        self._rpm_val = max(0, rpm)
        # Animate speed: 0→60000 RPM maps to 1→40 deg/frame
        self._speed = max(0.5, min(40.0, rpm / 1500.0))

    def _draw(self):
        self.delete("p")
        r = 82
        hub = 11

        # Disk (yarı saydam blur)
        lim = min(40, int(self._speed * 1.5))
        for i in range(lim, 0, -8):
            alpha_hex = format(max(0,min(255, i*4)), '02x')
            self.create_oval(self.cx-r+2, self.cy-r+2,
                             self.cx+r-2, self.cy+r-2,
                             outline=f"#00{alpha_hex}ff",
                             width=1, tags="p")

        # Kanatlar (2 bıçak)
        for b in range(2):
            ba = math.radians(self._angle + b*90)
            perp = ba + math.pi/2
            tip_x = self.cx + r*math.cos(ba)
            tip_y = self.cy + r*math.sin(ba)
            w = 7

            pts = [
                self.cx + hub*math.cos(ba),     self.cy + hub*math.sin(ba),
                tip_x - w*math.cos(perp),       tip_y - w*math.sin(perp),
                tip_x + 2*math.cos(perp),       tip_y + 2*math.sin(perp),
                self.cx - hub*math.cos(ba),     self.cy - hub*math.sin(ba),
            ]
            # Renk: hıza göre
            if self._speed < 12:   col = "#4488ff"
            elif self._speed < 24: col = "#44cc66"
            else:                  col = "#ff7044"
            self.create_polygon(pts, fill=col, outline=col, tags="p")

        # Merkez + hub
        self.create_oval(self.cx-hub, self.cy-hub,
                         self.cx+hub, self.cy+hub,
                         fill=BG3, outline=ACCENT, width=2, tags="p")
        self.create_oval(self.cx-4, self.cy-4,
                         self.cx+4, self.cy+4,
                         fill=ACCENT3, outline="", tags="p")

        # RPM yazısı
        self.create_text(self.cx, self.cy+r+16,
                         text=f"{self._rpm_val:,.0f} RPM",
                         font=("Consolas",9,"bold"),
                         fill=ACCENT, tags="p")

        # Çap/pitch bilgisi
        if hasattr(self, '_d_in') and hasattr(self, '_p_in'):
            self.create_text(self.cx, self.cy+r+30,
                             text=f"Ø {self._d_in}\"  Pitch {self._p_in}\"",
                             font=("Consolas",8),
                             fill=FG2, tags="p")

    def _loop(self):
        self._angle = (self._angle + self._speed) % 360
        self._draw()
        self._after_id = self.after(30, self._loop)

    def stop(self):
        if self._after_id:
            self.after_cancel(self._after_id)


class GaugeCanvas(tk.Canvas):
    """Analog gösterge"""
    def __init__(self, parent, title="", unit="",
                 lo=0, hi=100, w=190, h=140, **kw):
        super().__init__(parent, width=w, height=h,
                         bg=BG2, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self.cx   = w//2
        self.cy   = h - 28
        self._r   = min(self.cx, self.cy) - 12
        self.lo   = lo
        self.hi   = hi
        self.title = title
        self.unit  = unit
        self._cur  = float(lo)
        self._tgt  = float(lo)
        self._aeid = None
        self._loop()

    def set_value(self, v: float):
        self._tgt = max(self.lo, min(self.hi, v))

    def _angle_for(self, v: float) -> float:
        ratio = (v - self.lo) / max(1e-9, self.hi - self.lo)
        return 210 - ratio * 240          # 210° → -30°  (CCW)

    def _draw(self):
        self.delete("g")
        r = self._r
        cx, cy = self.cx, self.cy

        # Yay şeritleri (yeşil / sarı / kırmızı)
        for start_a, extent, col in [
            (210, -80,  "#1a5c1a"),
            (130, -80,  "#5c5c1a"),
            ( 50, -80,  "#5c1a1a"),
        ]:
            self.create_arc(cx-r, cy-r, cx+r, cy+r,
                            start=start_a, extent=extent,
                            style=tk.ARC, outline=col,
                            width=9, tags="g")

        # Tick çizgileri
        for i in range(11):
            a = math.radians(210 - i*24)
            inner = r - 14
            outer = r - 4
            x1 = cx + inner*math.cos(a); y1 = cy - inner*math.sin(a)
            x2 = cx + outer*math.cos(a); y2 = cy - outer*math.sin(a)
            self.create_line(x1,y1,x2,y2, fill=FG2, width=1, tags="g")

        # İbre
        na = math.radians(self._angle_for(self._cur))
        nx = cx + (r-16)*math.cos(na)
        ny = cy - (r-16)*math.sin(na)
        # İbre gölgesi
        self.create_line(cx, cy, nx+1, ny+1,
                         fill=BG, width=3, tags="g")
        self.create_line(cx, cy, nx, ny,
                         fill=ACCENT3, width=2.5, tags="g")

        # Merkez top
        self.create_oval(cx-5, cy-5, cx+5, cy+5,
                         fill=FG, outline="", tags="g")

        # Değer
        self.create_text(cx, cy - r//2 + 4,
                         text=f"{self._cur:.1f}",
                         font=("Consolas",13,"bold"),
                         fill=FG, tags="g")
        self.create_text(cx, cy - r//2 + 22,
                         text=self.unit,
                         font=("Consolas",7),
                         fill=FG2, tags="g")
        # Başlık
        self.create_text(cx, h - 10 if (h := self.winfo_height() or 140) else 130,
                         text=self.title,
                         font=("Consolas",8,"bold"),
                         fill=FG2, tags="g")

    def _loop(self):
        diff = self._tgt - self._cur
        if abs(diff) > 0.05:
            self._cur += diff * 0.12
        else:
            self._cur = self._tgt
        self._draw()
        self._aeid = self.after(30, self._loop)

    def stop(self):
        if self._aeid:
            self.after_cancel(self._aeid)


class BatteryCanvas(tk.Canvas):
    """Animasyonlu LiPo batarya"""
    def __init__(self, parent, w=110, h=290, **kw):
        super().__init__(parent, width=w, height=h,
                         bg=BG2, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self.W = w; self.H = h
        self._cells   = 4
        self._charge  = 100.0
        self._tgt     = 100.0
        self._bolt_y  = 0.0
        self._aeid    = None
        self._loop()

    def set_charge(self, pct: float):
        self._tgt = max(0, min(100, pct))

    def set_cells(self, n: int):
        self._cells = max(1, n)

    def _draw(self):
        self.delete("b")
        W, H = self.W, self.H
        bx = 14; by = 18
        bw = W - 28; bh = H - 55

        # Terminal
        tw = bw//3
        tx = bx + (bw - tw)//2
        self.create_rectangle(tx, by-10, tx+tw, by+4,
                              fill="#444", outline="", tags="b")

        # Dış çerçeve
        self.create_rectangle(bx, by, bx+bw, by+bh,
                              fill=BG3, outline=ACCENT, width=2, tags="b")

        # Dolum rengi
        if self._charge > 60:   col = ACCENT2
        elif self._charge > 30: col = WARN
        else:                   col = "#FF4444"

        # Dolum barı
        fill_h = int((bh - 8) * self._charge / 100.0)
        if fill_h > 0:
            fy = by + bh - 4 - fill_h
            self.create_rectangle(bx+4, fy,
                                  bx+bw-4, by+bh-4,
                                  fill=col, outline="", tags="b")

        # Hücre bölücü çizgileri
        cell_h = (bh - 8) / max(1, self._cells)
        for i in range(1, self._cells):
            ly = by + 4 + i * cell_h
            self.create_line(bx+4, ly, bx+bw-4, ly,
                             fill=BG2, width=2, tags="b")

        # Yüzde yazısı
        cx = bx + bw//2
        cy = by + bh//2
        self.create_text(cx, cy,
                         text=f"%{self._charge:.0f}",
                         font=("Consolas",14,"bold"),
                         fill="white", tags="b")

        # Şimşek animasyonu
        if self._charge < 95:
            bx2 = cx + 2
            by2 = cy - 30 + int(self._bolt_y * 4)
            self.create_text(bx2, by2, text="⚡",
                             font=("Arial",18), fill=WARN, tags="b")

        # S - etiket
        self.create_text(cx, by + bh + 15,
                         text=f"{self._cells}S  LiPo",
                         font=("Consolas",9,"bold"),
                         fill=FG2, tags="b")

        # Voltaj
        v = 3.7 * self._cells * self._charge / 100 + 3.3 * self._cells * (1 - self._charge/100)
        self.create_text(cx, by + bh + 30,
                         text=f"{v:.1f} V",
                         font=("Consolas",8),
                         fill=ACCENT, tags="b")

    def _loop(self):
        diff = self._tgt - self._charge
        if abs(diff) > 0.3:
            self._charge += diff * 0.08
        else:
            self._charge = self._tgt
        self._bolt_y = (self._bolt_y + 0.15) % 3.0
        self._draw()
        self._aeid = self.after(40, self._loop)

    def stop(self):
        if self._aeid:
            self.after_cancel(self._aeid)


class MotorDiagram(tk.Canvas):
    """Motor kesit çizimi (animasyonlu)"""
    def __init__(self, parent, w=220, h=220, **kw):
        super().__init__(parent, width=w, height=h,
                         bg=BG2, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self.cx = w//2; self.cy = h//2
        self._angle = 0.0
        self._aeid  = None
        self._loop()

    def _draw(self):
        self.delete("m")
        cx, cy = self.cx, self.cy
        r_stator = 55
        r_rotor  = 42
        n_poles  = 12

        # Stator dış
        self.create_oval(cx-r_stator, cy-r_stator,
                         cx+r_stator, cy+r_stator,
                         fill=BG3, outline=ACCENT, width=2, tags="m")

        # Stator dişleri (sabit)
        for i in range(n_poles):
            a  = math.radians(i * 360 / n_poles)
            x1 = cx + r_rotor*math.cos(a)
            y1 = cy + r_rotor*math.sin(a)
            x2 = cx + (r_stator-4)*math.cos(a)
            y2 = cy + (r_stator-4)*math.sin(a)
            coil_col = "#2244aa" if i%3==0 else "#aa4422" if i%3==1 else "#22aa44"
            self.create_line(x1,y1,x2,y2, fill=coil_col, width=5, tags="m")

        # Rotor (dönen)
        n_magnets = 14
        for i in range(n_magnets):
            a  = math.radians(self._angle + i*360/n_magnets)
            x1 = cx + 14*math.cos(a)
            y1 = cy + 14*math.sin(a)
            x2 = cx + r_rotor*math.cos(a)
            y2 = cy + r_rotor*math.sin(a)
            col = ACCENT if i%2==0 else ACCENT3
            self.create_line(x1,y1,x2,y2, fill=col, width=4, tags="m")

        # Merkez mil
        self.create_oval(cx-12, cy-12, cx+12, cy+12,
                         fill=BG, outline=FG2, width=2, tags="m")
        self.create_oval(cx-4,  cy-4,  cx+4,  cy+4,
                         fill=FG2, outline="", tags="m")

        self.create_text(cx, cy+r_stator+16,
                         text="Motor Kesit Görünümü",
                         font=("Consolas",8), fill=FG2, tags="m")

    def _loop(self):
        self._angle = (self._angle + 3) % 360
        self._draw()
        self._aeid = self.after(30, self._loop)

    def stop(self):
        if self._aeid:
            self.after_cancel(self._aeid)


class ESCDiagram(tk.Canvas):
    """ESC sıcaklık animasyonu"""
    def __init__(self, parent, w=180, h=220, **kw):
        super().__init__(parent, width=w, height=h,
                         bg=BG2, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self.W = w; self.H = h
        self._temp      = 25.0
        self._tgt_temp  = 25.0
        self._pulse     = 0.0
        self._aeid      = None
        self._loop()

    def set_temp(self, t: float):
        self._tgt_temp = max(0, min(150, t))

    def _draw(self):
        self.delete("e")
        W, H = self.W, self.H
        t = self._temp

        # PCB arka plan
        pcb_x, pcb_y = 20, 20
        pcb_w, pcb_h = W-40, H-80
        self.create_rectangle(pcb_x, pcb_y, pcb_x+pcb_w, pcb_y+pcb_h,
                              fill="#0a2a0a", outline="#1a5a1a", width=2, tags="e")

        # MOSFET devreler
        for row in range(3):
            for col in range(2):
                rx = pcb_x + 10 + col*((pcb_w-20)//2)
                ry = pcb_y + 10 + row*((pcb_h-20)//3)
                rw = (pcb_w-20)//2 - 5
                rh = (pcb_h-20)//3 - 8

                # Sıcaklığa göre renk
                heat = min(1.0, (t - 25) / 100.0)
                r_c  = int(heat * 200)
                g_c  = int((1-heat) * 100)
                chip_col = f"#{r_c:02x}{g_c:02x}00"

                self.create_rectangle(rx, ry, rx+rw, ry+rh,
                                      fill=chip_col, outline="#333", tags="e")
                # Chip markası
                self.create_text(rx+rw//2, ry+rh//2,
                                 text="MOS", font=("Consolas",6), fill="#888", tags="e")

        # İz çizgileri (PCB trace)
        for i in range(5):
            y = pcb_y + 15 + i * (pcb_h-15)//5
            self.create_line(pcb_x+5, y, pcb_x+pcb_w-5, y,
                             fill="#1a4a1a", width=1, dash=(3,6), tags="e")

        # Isı dalgası animasyonu
        pulse_y = pcb_y - 5 - int(self._pulse * 8)
        if t > 60:
            for wi in range(3):
                ww = 15 + wi * 10
                wy = pulse_y - wi * 6
                opacity = max(0, 1.0 - wi*0.3)
                op_hex = format(int(opacity*100), '02x')
                self.create_text(pcb_x + pcb_w//2, wy,
                                 text="~~~", font=("Consolas",9),
                                 fill=f"#ff{op_hex}00", tags="e")

        # Sıcaklık göstergesi (termometre)
        tx = W - 28
        ty = 20
        th = H - 90
        self.create_rectangle(tx-5, ty, tx+5, ty+th,
                              fill=BG3, outline=FG2, width=1, tags="e")
        fill_h = int(th * min(1.0, (t-25)/(130-25)))
        if fill_h > 0:
            heat2 = min(1.0, (t-25)/100)
            fc = f"#{int(heat2*255):02x}{int((1-heat2)*100):02x}00"
            self.create_rectangle(tx-4, ty+th-fill_h,
                                  tx+4, ty+th,
                                  fill=fc, outline="", tags="e")
        # Ampul
        self.create_oval(tx-8, ty+th-1, tx+8, ty+th+15,
                         fill="#cc2200", outline="", tags="e")

        # Sıcaklık yazısı
        if t < 60:   tc = ACCENT2
        elif t < 90: tc = WARN
        else:        tc = "#FF4444"
        self.create_text(W//2, H-55,
                         text=f"{t:.0f} °C",
                         font=("Consolas",16,"bold"), fill=tc, tags="e")
        self.create_text(W//2, H-35,
                         text="ESC Sıcaklık",
                         font=("Consolas",8), fill=FG2, tags="e")

    def _loop(self):
        diff = self._tgt_temp - self._temp
        if abs(diff) > 0.2:
            self._temp += diff * 0.06
        else:
            self._temp = self._tgt_temp
        self._pulse = (self._pulse + 0.1) % 3.0
        self._draw()
        self._aeid = self.after(40, self._loop)

    def stop(self):
        if self._aeid:
            self.after_cancel(self._aeid)


# ============================================================
# YARDIMCI UI FONKSİYONLARI
# ============================================================

def section_label(parent, text, **kw):
    tk.Label(parent, text=text, bg=BG2, fg=ACCENT,
             font=("Consolas", 11, "bold"), **kw).pack(pady=(12,6))

def result_row(parent, label, var, unit="", highlight=False):
    f  = tk.Frame(parent, bg=BG2)
    f.pack(fill=tk.X, padx=16, pady=2)
    tk.Label(f, text=f"{label}:", bg=BG2, fg=FG2,
             font=("Consolas",9), width=22, anchor="w").pack(side=tk.LEFT)
    col = ACCENT if highlight else FG
    tk.Label(f, textvariable=var, bg=BG2, fg=col,
             font=("Consolas",9,"bold")).pack(side=tk.LEFT)
    if unit:
        tk.Label(f, text=unit, bg=BG2, fg=FG2,
                 font=("Consolas",8)).pack(side=tk.LEFT, padx=4)

def input_field(parent, label, key, default, unit, store: dict):
    f = tk.Frame(parent, bg=BG2)
    f.pack(fill=tk.X, padx=14, pady=4)
    tk.Label(f, text=label, bg=BG2, fg=FG2,
             font=("Consolas",9)).pack(anchor="w")
    row = tk.Frame(f, bg=BG4, pady=1)
    row.pack(fill=tk.X)
    var = tk.StringVar(value=default)
    store[key] = var
    tk.Entry(row, textvariable=var, bg=BG4, fg=FG,
             insertbackground=FG, relief=tk.FLAT,
             font=("Consolas",10), bd=5).pack(side=tk.LEFT,
                                               fill=tk.X, expand=True)
    if unit:
        tk.Label(row, text=unit, bg=BG4, fg=FG2,
                 font=("Consolas",8), padx=6).pack(side=tk.RIGHT)
    return var

def calc_button(parent, cmd):
    tk.Button(parent, text="  Hesapla  ",
              bg=ACCENT, fg=BG,
              font=("Consolas",10,"bold"),
              relief=tk.FLAT, cursor="hand2",
              activebackground="#00a0c8",
              command=cmd).pack(pady=12, padx=14, fill=tk.X)

def left_panel(parent):
    f = tk.Frame(parent, bg=BG2, width=295)
    f.pack(side=tk.LEFT, fill=tk.Y)
    f.pack_propagate(False)
    return f

def right_panel(parent):
    f = tk.Frame(parent, bg=BG)
    f.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    return f


# ============================================================
# ANA UYGULAMA
# ============================================================

class DroneCalcApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("DroneCalc Pro — Drone Hesaplama Sistemi")
        self.geometry("1200x840")
        self.configure(fg_color=BG)
        self.resizable(True, True)
        self.results: dict = {}
        self._build_ui()

    # ─── UI İSKELETİ ────────────────────────────────────────

    def _build_ui(self):
        # Header
        hdr = DroneHeaderCanvas(self, width=1200, height=155)
        hdr.pack(fill=tk.X, side=tk.TOP)

        # Notebook style
        sty = ttk.Style()
        sty.theme_use("clam")
        for n, cfg in [
            ("Custom.TNotebook",     {"background": BG, "borderwidth":0}),
            ("Custom.TNotebook.Tab", {"background": BG2, "foreground": FG2,
                                      "padding":[16,7], "borderwidth":0,
                                      "font":("Consolas",9,"bold")}),
        ]:
            sty.configure(n, **cfg)
        sty.map("Custom.TNotebook.Tab",
                background=[("selected", BG3)],
                foreground=[("selected", ACCENT)])

        nb = ttk.Notebook(self, style="Custom.TNotebook")
        nb.pack(fill=tk.BOTH, expand=True)

        tabs = {}
        for name, title in [
            ("motor",   "  ⚙  Motor  "),
            ("prop",    "  ✦  Pervane  "),
            ("batt",    "  ⚡  Batarya  "),
            ("weight",  "  ↑  Agirlik & Itme  "),
            ("esc",     "  ≡  ESC  "),
            ("summary", "  ★  Ozet  "),
        ]:
            f = tk.Frame(nb, bg=BG)
            nb.add(f, text=title)
            tabs[name] = f

        self._build_motor(tabs["motor"])
        self._build_propeller(tabs["prop"])
        self._build_battery(tabs["batt"])
        self._build_weight(tabs["weight"])
        self._build_esc(tabs["esc"])
        self._build_summary(tabs["summary"])

    # ─── MOTOR SEKMESİ ──────────────────────────────────────

    def _build_motor(self, frame):
        self._minp = {}
        lp = left_panel(frame)
        rp = right_panel(frame)

        section_label(lp, "Motor Parametreleri")
        for lbl, key, dflt, unit in [
            ("Motor Adı",        "name",    "Generic 2306",  ""),
            ("KV Değeri",        "kv",      "2400",          "RPM/V"),
            ("Max Akım",         "max_a",   "25",            "A"),
            ("İç Direnç",        "resist",  "0.10",          "Ω"),
            ("Motor Ağırlığı",   "weight",  "30",            "g"),
            ("Çalışma Voltajı",  "voltage", "14.8",          "V"),
        ]:
            v = input_field(lp, lbl, key, dflt, unit, self._minp)
            v.trace("w", lambda *a: self._calc_motor())

        calc_button(lp, self._calc_motor)

        # Motor diyagramı
        tk.Label(lp, text="Motor Kesit Diyagramı",
                 bg=BG2, fg=FG2, font=("Consolas",8)).pack(pady=(10,2))
        MotorDiagram(lp, w=200, h=200).pack(padx=10, pady=4)

        # Sağ panel: gauge'lar
        gf = tk.Frame(rp, bg=BG)
        gf.pack(fill=tk.X, padx=10, pady=10)

        self._mrpm_g  = GaugeCanvas(gf, "Max RPM",      "RPM", 0, 120000)
        self._mpow_g  = GaugeCanvas(gf, "Max Güç",      "W",   0, 2500)
        self._meff_g  = GaugeCanvas(gf, "Verimlilik",   "%",   0, 100)
        self._mkv_g   = GaugeCanvas(gf, "G/W Oranı",    "g/W", 0, 40)
        for g in (self._mrpm_g, self._mpow_g, self._meff_g, self._mkv_g):
            g.pack(side=tk.LEFT, padx=6)

        # Tablo başlığı
        tk.Label(rp, text="Voltaj / Hücre Bazlı Performans Tablosu",
                 bg=BG, fg=ACCENT, font=("Consolas",10,"bold")).pack(pady=(8,2))
        tf = tk.Frame(rp, bg=BG)
        tf.pack(fill=tk.X, padx=12)

        hdrs = ["Hücre", "Voltaj (V)", "Max RPM",
                "Max Güç (W)", "Max Akım (A)", "Kalkış İtmesi*"]
        for c, h in enumerate(hdrs):
            tk.Label(tf, text=h, bg=BG3, fg=ACCENT2,
                     font=("Consolas",8,"bold"), width=13,
                     pady=5, relief=tk.FLAT).grid(
                         row=0, column=c, padx=1, pady=1, sticky="ew")

        self._mtbl = []
        for r in range(6):
            row_vars = [tk.StringVar(value="—") for _ in range(6)]
            self._mtbl.append(row_vars)
            for c, var in enumerate(row_vars):
                bg = BG4 if r % 2 else BG2
                tk.Label(tf, textvariable=var, bg=bg, fg=FG,
                         font=("Consolas",8), width=13,
                         pady=4).grid(row=r+1, column=c,
                                      padx=1, pady=1, sticky="ew")

        tk.Label(rp, text="* Tahmini pervane itkisi (50% gaz, 5x4.5 prop)",
                 bg=BG, fg=FG2, font=("Consolas",7)).pack()

        self._calc_motor()

    def _calc_motor(self, *_):
        try:
            kv     = float(self._minp["kv"].get())
            max_a  = float(self._minp["max_a"].get())
            resist = float(self._minp["resist"].get())
            volt   = float(self._minp["voltage"].get())

            rpm      = calc_motor_rpm(kv, volt)
            max_pow  = volt * max_a
            loss     = max_a**2 * resist
            eff      = max(0, min(100, (1 - loss/max(1,max_pow))*100))
            thrust_g = calc_prop_thrust(5, 4.5, rpm*0.5) * 0.7  # 50% gaz örneği

            self._mrpm_g.set_value(rpm)
            self._mpow_g.set_value(max_pow)
            self._meff_g.set_value(eff)
            gw = thrust_g / max(1, max_pow) * 4
            self._mkv_g.set_value(gw)

            cells_v = [(1,3.7),(2,7.4),(3,11.1),(4,14.8),(5,18.5),(6,22.2)]
            for i,(c,v) in enumerate(cells_v):
                r  = calc_motor_rpm(kv, v)
                pw = v * max_a
                th = calc_prop_thrust(5,4.5,r*0.5)*0.7
                self._mtbl[i][0].set(f"{c}S")
                self._mtbl[i][1].set(f"{v:.1f}")
                self._mtbl[i][2].set(f"{r:,.0f}")
                self._mtbl[i][3].set(f"{pw:.0f}")
                self._mtbl[i][4].set(f"{max_a:.0f}")
                self._mtbl[i][5].set(f"{th:.0f} g")

            self.results.update(motor_kv=kv, motor_max_a=max_a,
                                motor_volt=volt, motor_rpm=rpm,
                                motor_pow=max_pow)
        except (ValueError, ZeroDivisionError):
            pass

    # ─── PERVANE SEKMESİ ────────────────────────────────────

    def _build_propeller(self, frame):
        self._pinp = {}
        lp = left_panel(frame)
        rp = right_panel(frame)

        section_label(lp, "Pervane Parametreleri")
        for lbl, key, dflt, unit in [
            ("Çap",              "dia",    "5",    "inç"),
            ("Pitch",            "pitch",  "4.5",  "inç"),
            ("RPM",              "rpm",    "15000",""),
            ("Kanat Sayısı",     "blades", "2",    "adet"),
            ("Pervane Verimi",   "eff",    "0.70", "0–1"),
        ]:
            v = input_field(lp, lbl, key, dflt, unit, self._pinp)
            v.trace("w", lambda *a: self._calc_prop())

        calc_button(lp, self._calc_prop)

        # Hazır seçenekler
        tk.Label(lp, text="Hazır Boyutlar", bg=BG2, fg=FG2,
                 font=("Consolas",8,"bold")).pack(pady=(8,4))
        bf = tk.Frame(lp, bg=BG2)
        bf.pack(fill=tk.X, padx=10)
        presets = [("3x2.5",3,2.5),("5x4.5",5,4.5),("5x3",5,3),
                   ("6x4.5",6,4.5),("7x4",7,4),("10x4.5",10,4.5),
                   ("12x4.5",12,4.5),("13x5",13,5)]
        for i,(name,d,p) in enumerate(presets):
            def _set(dd=d,pp=p):
                self._pinp["dia"].set(str(dd))
                self._pinp["pitch"].set(str(pp))
                self._calc_prop()
            tk.Button(bf, text=name, bg=BG3, fg=FG2,
                      font=("Consolas",8), relief=tk.FLAT,
                      cursor="hand2", command=_set,
                      activebackground=BG4).grid(
                          row=i//2, column=i%2,
                          padx=3, pady=3, sticky="ew")
        bf.columnconfigure(0,weight=1); bf.columnconfigure(1,weight=1)

        # Sağ: animasyon
        self._prop_canvas = PropCanvas(rp, w=240, h=240)
        self._prop_canvas.pack(side=tk.LEFT, padx=15, pady=15)

        res = tk.Frame(rp, bg=BG)
        res.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        gf = tk.Frame(res, bg=BG)
        gf.pack(fill=tk.X, pady=8)
        self._pthr_g  = GaugeCanvas(gf, "İtme",     "g",    0, 3000)
        self._ppow_g  = GaugeCanvas(gf, "Güç",      "W",    0, 800)
        self._pgw_g   = GaugeCanvas(gf, "g / W",    "g/W",  0, 25)
        for g in (self._pthr_g, self._ppow_g, self._pgw_g):
            g.pack(side=tk.LEFT, padx=6)

        det = tk.Frame(res, bg=BG2)
        det.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        section_label(det, "Detaylı Sonuçlar")

        self._pvars = {}
        for key, lbl in [
            ("thrust",      "Statik İtme"),
            ("power",       "Güç Tüketimi"),
            ("gw",          "g/W Oranı"),
            ("tip_speed",   "Kanat Ucu Hızı"),
            ("pitch_speed", "Pitch Hızı"),
            ("disk_load",   "Disk Yüklemesi"),
            ("fom",         "Verim Faktörü"),
        ]:
            self._pvars[key] = tk.StringVar(value="—")
            result_row(det, lbl, self._pvars[key])

        self._calc_prop()

    def _calc_prop(self, *_):
        try:
            d   = float(self._pinp["dia"].get())
            p   = float(self._pinp["pitch"].get())
            rpm = float(self._pinp["rpm"].get())
            eff = float(self._pinp["eff"].get())

            thrust = calc_prop_thrust(d, p, rpm) * eff
            power  = calc_prop_power(d, p, rpm, eff)
            gw     = thrust / max(1e-3, power)

            d_m   = d * INCHES_TO_M
            tip_v = math.pi * d_m * rpm / 60
            pit_v = p * INCHES_TO_M * rpm / 60
            A     = math.pi * (d_m/2)**2
            disk_load = thrust / max(1e-6, A)   # g/m²

            thrust_N = thrust / 1000 * G_MS2
            fom = thrust_N * math.sqrt(thrust_N/(2*AIR_DENSITY*A)) / max(1e-3, power) if power>0 else 0

            self._pthr_g.set_value(thrust)
            self._ppow_g.set_value(power)
            self._pgw_g.set_value(gw)
            self._prop_canvas.set_rpm(rpm)
            self._prop_canvas._d_in = d
            self._prop_canvas._p_in = p

            self._pvars["thrust"].set(f"{thrust:.1f} g")
            self._pvars["power"].set(f"{power:.1f} W")
            self._pvars["gw"].set(f"{gw:.2f}")
            self._pvars["tip_speed"].set(f"{tip_v:.1f} m/s")
            self._pvars["pitch_speed"].set(f"{pit_v:.1f} m/s")
            self._pvars["disk_load"].set(f"{disk_load:.0f} g/m²")
            self._pvars["fom"].set(f"{fom:.3f}")

            self.results.update(prop_thrust=thrust, prop_power=power, prop_rpm=rpm)
        except (ValueError, ZeroDivisionError):
            pass

    # ─── BATARYA SEKMESİ ────────────────────────────────────

    def _build_battery(self, frame):
        self._binp = {}
        lp = left_panel(frame)
        rp = right_panel(frame)

        section_label(lp, "Batarya Parametreleri")
        for lbl, key, dflt, unit in [
            ("Kapasite",         "cap",     "1500",  "mAh"),
            ("Hücre Sayısı (S)", "cells",   "4",     "S"),
            ("Deşarj Oranı",     "crate",   "75",    "C"),
            ("İç Direnç",        "resist",  "15",    "mΩ"),
            ("Batarya Ağırlığı", "weight",  "180",   "g"),
            ("Ort. Akım Çekişi", "avg_i",   "18",    "A"),
        ]:
            v = input_field(lp, lbl, key, dflt, unit, self._binp)
            v.trace("w", lambda *a: self._calc_batt())

        calc_button(lp, self._calc_batt)

        # LiPo kimya tablosu
        tk.Label(lp, text="LiPo Hücre Voltajları",
                 bg=BG2, fg=FG2, font=("Consolas",8,"bold")).pack(pady=(10,3))
        for lbl, val, col in [
            ("Tam Şarj",    "4.20 V", ACCENT2),
            ("Nominal",     "3.70 V", ACCENT),
            ("Depolama",    "3.85 V", WARN),
            ("Min Kesim",   "3.50 V", "#FF4444"),
        ]:
            f = tk.Frame(lp, bg=BG2)
            f.pack(fill=tk.X, padx=14, pady=1)
            tk.Label(f, text=lbl, bg=BG2, fg=FG2,
                     font=("Consolas",8), width=13, anchor="w").pack(side=tk.LEFT)
            tk.Label(f, text=val, bg=BG2, fg=col,
                     font=("Consolas",8,"bold")).pack(side=tk.LEFT)

        # Sağ: batarya animasyonu
        self._batt_canvas = BatteryCanvas(rp, w=110, h=290)
        self._batt_canvas.pack(side=tk.LEFT, padx=20, pady=20)

        res = tk.Frame(rp, bg=BG)
        res.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        gf = tk.Frame(res, bg=BG)
        gf.pack(fill=tk.X, pady=8)
        self._bv_g  = GaugeCanvas(gf, "Nominal V",    "V",  0, 30)
        self._bft_g = GaugeCanvas(gf, "Uçus Süresi",  "dk", 0, 60)
        self._bwh_g = GaugeCanvas(gf, "Enerji",       "Wh", 0, 100)
        for g in (self._bv_g, self._bft_g, self._bwh_g):
            g.pack(side=tk.LEFT, padx=6)

        det = tk.Frame(res, bg=BG2)
        det.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        section_label(det, "Batarya Özellikleri")

        self._bvars = {}
        for key, lbl in [
            ("nom_v",   "Nominal Voltaj"),
            ("max_v",   "Tam Şarj Voltajı"),
            ("min_v",   "Min Kesim Voltajı"),
            ("max_i",   "Anlık Max Akım"),
            ("cont_i",  "Sürekli Max Akım (75%)"),
            ("energy",  "Toplam Enerji"),
            ("flight",  "Tahmini Uçuş Süresi"),
            ("wh_g",    "Enerji Yoğunluğu"),
            ("vol_drop","Voltaj Düşüşü (Max Akım)"),
        ]:
            self._bvars[key] = tk.StringVar(value="—")
            result_row(det, lbl, self._bvars[key],
                       highlight=(key in ("flight","max_i")))

        self._calc_batt()

    def _calc_batt(self, *_):
        try:
            cap    = float(self._binp["cap"].get())
            cells  = int(float(self._binp["cells"].get()))
            crate  = float(self._binp["crate"].get())
            resist = float(self._binp["resist"].get()) / 1000.0
            weight = float(self._binp["weight"].get())
            avg_i  = float(self._binp["avg_i"].get())

            nom_v = 3.70 * cells
            max_v = 4.20 * cells
            min_v = 3.50 * cells
            max_i = cap / 1000.0 * crate
            energy= nom_v * cap / 1000.0
            ft    = calc_flight_time(cap, avg_i)
            wh_g  = energy / max(1, weight)
            vdrop = max_i * resist

            charge_pct = max(5, 100 - (avg_i/max(1,max_i))*50)

            self._bv_g.set_value(nom_v)
            self._bft_g.set_value(ft)
            self._bwh_g.set_value(energy)
            self._batt_canvas.set_charge(charge_pct)
            self._batt_canvas.set_cells(cells)

            self._bvars["nom_v"].set(f"{nom_v:.2f} V")
            self._bvars["max_v"].set(f"{max_v:.2f} V")
            self._bvars["min_v"].set(f"{min_v:.2f} V")
            self._bvars["max_i"].set(f"{max_i:.1f} A")
            self._bvars["cont_i"].set(f"{max_i*0.75:.1f} A")
            self._bvars["energy"].set(f"{energy:.2f} Wh")
            self._bvars["flight"].set(f"{ft:.1f} dakika")
            self._bvars["wh_g"].set(f"{wh_g:.4f} Wh/g")
            self._bvars["vol_drop"].set(f"{vdrop:.3f} V")

            self.results.update(batt_cells=cells, batt_volt=nom_v,
                                batt_cap=cap, batt_max_i=max_i,
                                flight_time=ft)
        except (ValueError, ZeroDivisionError):
            pass

    # ─── AĞIRLIK & İTME SEKMESİ ─────────────────────────────

    def _build_weight(self, frame):
        self._winp = {}
        lp = left_panel(frame)
        rp = right_panel(frame)

        section_label(lp, "Drone Bileşenleri")
        for lbl, key, dflt, unit in [
            ("Motor Sayısı",          "n_motors", "4",   "adet"),
            ("Motor Başına Max İtme", "thr_per",  "600", "g"),
            ("Frame Ağırlığı",        "frame_w",  "100", "g"),
            ("Motor Ağırlığı (adet)", "motor_w",  "28",  "g"),
            ("Batarya Ağırlığı",      "batt_w",   "180", "g"),
            ("ESC Ağırlığı (adet)",   "esc_w",    "8",   "g"),
            ("FC + Elektronik",       "fc_w",     "45",  "g"),
            ("Kamera / Gimbal",       "cam_w",    "0",   "g"),
            ("Payload",               "pay_w",    "0",   "g"),
        ]:
            v = input_field(lp, lbl, key, dflt, unit, self._winp)
            v.trace("w", lambda *a: self._calc_weight())

        calc_button(lp, self._calc_weight)

        # Sağ
        gf = tk.Frame(rp, bg=BG)
        gf.pack(fill=tk.X, padx=10, pady=10)
        self._tw_g    = GaugeCanvas(gf, "T/W Oranı",   ":1",  0, 6)
        self._hov_g   = GaugeCanvas(gf, "Hover Gaz",   "%",   0, 100)
        self._tmass_g = GaugeCanvas(gf, "Toplam Agirl.","g",  0, 3000)
        self._tthr_g  = GaugeCanvas(gf, "Max İtme",    "g",   0, 8000)
        for g in (self._tw_g, self._hov_g, self._tmass_g, self._tthr_g):
            g.pack(side=tk.LEFT, padx=5)

        # Pasta grafik
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8,3.5),
                                        facecolor=BG)
        ax1.set_facecolor(BG3); ax2.set_facecolor(BG3)
        self._wax_pie = ax1
        self._wax_bar = ax2
        self._wfig    = fig
        self._wfig.tight_layout(pad=2)
        cvs = FigureCanvasTkAgg(fig, rp)
        cvs.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self._wcvs = cvs

        self._calc_weight()

    def _calc_weight(self, *_):
        try:
            n   = int(float(self._winp["n_motors"].get()))
            tp  = float(self._winp["thr_per"].get())
            fw  = float(self._winp["frame_w"].get())
            mw  = float(self._winp["motor_w"].get())
            bw  = float(self._winp["batt_w"].get())
            ew  = float(self._winp["esc_w"].get())
            fcw = float(self._winp["fc_w"].get())
            cw  = float(self._winp["cam_w"].get())
            pw  = float(self._winp["pay_w"].get())

            total_m = n*mw; total_e = n*ew
            total_w = fw + total_m + bw + total_e + fcw + cw + pw
            total_t = n * tp
            tw      = total_t / max(1, total_w)
            hover   = min(100, 100.0/max(0.01, tw))

            self._tw_g.set_value(tw)
            self._hov_g.set_value(hover)
            self._tmass_g.set_value(total_w)
            self._tthr_g.set_value(total_t)

            # Pie chart
            comps = {"Frame":fw, f"Motor ({n}×)":total_m,
                     "Batarya":bw, f"ESC ({n}×)":total_e,
                     "FC/Elek.":fcw, "Kamera":cw, "Payload":pw}
            comps = {k:v for k,v in comps.items() if v>0}
            cols  = [ACCENT, ACCENT2, ACCENT3, ACCENT4,
                     WARN, "#48b4e0", "#c8a8e9"]

            ax = self._wax_pie
            ax.clear(); ax.set_facecolor(BG3)
            if comps:
                wedges, texts, autos = ax.pie(
                    list(comps.values()),
                    labels=list(comps.keys()),
                    colors=cols[:len(comps)],
                    autopct="%1.0f%%",
                    startangle=90,
                    textprops={"color":FG,"fontsize":7},
                    wedgeprops={"linewidth":0.5,"edgecolor":BG})
                for at in autos:
                    at.set_color(BG); at.set_fontsize(7)
            ax.set_title(f"Ağırlık Dağılımı  (Toplam: {total_w:.0f}g)",
                         color=FG, fontsize=8, pad=6)

            # T/W bar
            ax2 = self._wax_bar
            ax2.clear(); ax2.set_facecolor(BG3)
            labels = ["Min\n(Havada)", "Hover", "İyi", "Mükemmel"]
            vals   = [1.0, 1/max(0.01,tw)*tw, 2.0, 4.0]
            bar_v  = [tw, 2.0, 3.0, 4.0]
            colors = [ACCENT3, WARN, ACCENT2, ACCENT]
            ax2.barh(labels, [tw]*4, color=[ACCENT3,WARN,ACCENT2,ACCENT],
                     alpha=0.3)
            ax2.axvline(tw, color=ACCENT, linewidth=2.5, label=f"T/W = {tw:.2f}")
            ax2.set_xlim(0, 5)
            ax2.set_facecolor(BG3)
            ax2.tick_params(colors=FG, labelsize=8)
            ax2.set_xlabel("İtme/Ağırlık Oranı", color=FG2, fontsize=8)
            ax2.set_title("T/W Karşılaştırması", color=FG, fontsize=8)
            ax2.spines[:].set_color(BORDER)
            ax2.legend(fontsize=8, labelcolor=FG,
                       facecolor=BG2, edgecolor=BORDER)

            self._wfig.tight_layout(pad=2)
            self._wcvs.draw()

            self.results.update(total_w=total_w, total_t=total_t,
                                tw=tw, hover=hover, n_motors=n)
        except (ValueError, ZeroDivisionError):
            pass

    # ─── ESC SEKMESİ ────────────────────────────────────────

    def _build_esc(self, frame):
        self._einp = {}
        lp = left_panel(frame)
        rp = right_panel(frame)

        section_label(lp, "ESC Hesaplama")
        for lbl, key, dflt, unit in [
            ("Motor Max Akımı",  "mi",      "25",    "A"),
            ("Motor Sayısı",     "en",      "4",     "adet"),
            ("Güvenlik Payı",    "safety",  "25",    "%"),
            ("Batarya Voltajı",  "ev",      "14.8",  "V"),
            ("ESC İç Direnci",   "er",      "0.005", "Ω"),
            ("Ortam Sıcaklığı",  "t_amb",   "25",    "°C"),
        ]:
            v = input_field(lp, lbl, key, dflt, unit, self._einp)
            v.trace("w", lambda *a: self._calc_esc())

        calc_button(lp, self._calc_esc)

        # ESC devre çizimi
        tk.Label(lp, text="ESC PCB Görünümü",
                 bg=BG2, fg=FG2, font=("Consolas",8)).pack(pady=(10,2))
        self._esc_diag = ESCDiagram(lp, w=200, h=200)
        self._esc_diag.pack(padx=10, pady=4)

        # Sağ
        gf = tk.Frame(rp, bg=BG)
        gf.pack(fill=tk.X, padx=10, pady=10)
        self._et_g   = GaugeCanvas(gf, "ESC Sicaklik", "°C",  0, 130)
        self._el_g   = GaugeCanvas(gf, "Güç Kaybi",    "W",   0, 80)
        self._eeff_g = GaugeCanvas(gf, "ESC Verimi",   "%",   80, 100)
        for g in (self._et_g, self._el_g, self._eeff_g):
            g.pack(side=tk.LEFT, padx=6)

        det = tk.Frame(rp, bg=BG2)
        det.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        section_label(det, "ESC Seçim Rehberi")

        self._evars = {}
        for key, lbl in [
            ("req",   "Gereken Min. Akım"),
            ("rec",   "Önerilen ESC"),
            ("loss",  "Toplam Güç Kaybı"),
            ("esc_v", "ESC Voltaj Sınıfı"),
            ("warn",  "Durum"),
        ]:
            self._evars[key] = tk.StringVar(value="—")
            result_row(det, lbl, self._evars[key],
                       highlight=(key=="rec"))

        # Standart ESC butonları
        tk.Label(det, text="Standart Değerler (Önerilen yeşil)",
                 bg=BG2, fg=FG2, font=("Consolas",8,"bold")).pack(pady=(14,4))
        bf = tk.Frame(det, bg=BG2)
        bf.pack(padx=14)
        self._esc_btns: list[tuple[int,tk.Label]] = []
        ratings = [10,12,15,20,25,30,35,40,45,50,60,80,100]
        for i, r in enumerate(ratings):
            lbl = tk.Label(bf, text=f"{r}A",
                           bg=BG3, fg=FG2,
                           font=("Consolas",8), width=5, pady=4)
            lbl.grid(row=i//5, column=i%5, padx=2, pady=2, sticky="ew")
            self._esc_btns.append((r, lbl))

        self._calc_esc()

    def _calc_esc(self, *_):
        try:
            mi     = float(self._einp["mi"].get())
            en     = int(float(self._einp["en"].get()))
            safety = float(self._einp["safety"].get()) / 100.0
            ev     = float(self._einp["ev"].get())
            er     = float(self._einp["er"].get())
            t_amb  = float(self._einp["t_amb"].get())

            req    = mi * (1 + safety)
            rec    = esc_standard(req)
            loss_1 = mi**2 * er
            total_l= loss_1 * en
            eff_pct= (1 - loss_1/(mi*ev)) * 100 if ev > 0 else 99
            eff_pct= max(80, min(100, eff_pct))
            t_rise = loss_1 * 18
            temp   = t_amb + t_rise

            self._et_g.set_value(temp)
            self._el_g.set_value(total_l)
            self._eeff_g.set_value(eff_pct)
            self._esc_diag.set_temp(temp)

            self._evars["req"].set(f"{req:.1f} A")
            self._evars["rec"].set(f"{rec} A")
            self._evars["loss"].set(f"{total_l:.2f} W ({en}x ESC)")
            self._evars["esc_v"].set(f"Max {ev:.0f}V sistemine uygun")
            if temp > 100:
                ws = "KRITIK - Aktif sogutma zorunlu!"
            elif temp > 75:
                ws = "SICAK - Sogutma deliklerini kontrol edin"
            elif rec > 60:
                ws = "Yuksek akım — kaliteli ESC seçin"
            else:
                ws = "Normal çalışma aralığı ✓"
            self._evars["warn"].set(ws)

            for r, lbl in self._esc_btns:
                if r == rec:
                    lbl.config(bg=ACCENT2, fg=BG)
                elif r > req:
                    lbl.config(bg=BG3, fg=FG2)
                else:
                    lbl.config(bg="#3a1010", fg="#ff6666")

            self.results.update(esc_rec=rec, esc_temp=temp)
        except (ValueError, ZeroDivisionError):
            pass

    # ─── ÖZET SEKMESİ ───────────────────────────────────────

    def _build_summary(self, frame):
        lp = left_panel(frame)
        lp.configure(width=360)
        rp = right_panel(frame)

        section_label(lp, "Build Özeti")
        tk.Label(lp, text="Tüm sekmelerdeki hesap değerleri",
                 bg=BG2, fg=FG2, font=("Consolas",8)).pack()

        tk.Button(lp, text="  Özeti Güncelle  ",
                  bg=ACCENT2, fg=BG,
                  font=("Consolas",10,"bold"),
                  relief=tk.FLAT, cursor="hand2",
                  command=self._update_summary).pack(
                      pady=10, padx=14, fill=tk.X)

        # Özet satırları
        self._svars: dict[str, tk.StringVar] = {}
        groups = [
            ("Motor", [
                ("m_kv",  "KV Değeri"),
                ("m_rpm", "Max RPM"),
                ("m_pow", "Max Güç"),
            ]),
            ("Pervane", [
                ("p_thr",  "İtme (tek)"),
                ("p_pow",  "Güç (tek)"),
            ]),
            ("Batarya", [
                ("b_v",    "Voltaj"),
                ("b_cap",  "Kapasite"),
                ("b_maxi", "Max Akım"),
                ("b_ft",   "Uçuş Süresi"),
            ]),
            ("Uçuş Performansı", [
                ("tw",     "T/W Oranı"),
                ("hover",  "Hover Gaz"),
                ("mass",   "Toplam Ağırlık"),
                ("thr_t",  "Toplam İtme"),
            ]),
            ("ESC", [
                ("esc_r",  "Önerilen ESC"),
                ("esc_t",  "ESC Sıcaklık"),
            ]),
        ]
        for sec, items in groups:
            tk.Label(lp, text=f"── {sec} ──",
                     bg=BG2, fg=ACCENT2,
                     font=("Consolas",8,"bold")).pack(
                         pady=(10,3), padx=14, anchor="w")
            for key, lbl in items:
                self._svars[key] = tk.StringVar(value="—")
                result_row(lp, lbl, self._svars[key])

        # Tavsiyeler
        self._rec_var = tk.StringVar(value="")
        tk.Label(lp, text="── Tavsiyeler ──",
                 bg=BG2, fg=ACCENT3,
                 font=("Consolas",8,"bold")).pack(pady=(14,3), padx=14, anchor="w")
        tk.Label(lp, textvariable=self._rec_var,
                 bg=BG2, fg=FG, font=("Consolas",8),
                 justify=tk.LEFT, wraplength=330).pack(padx=14, anchor="w")

        # Sağ: Radar chart + bar chart
        fig = plt.figure(figsize=(7,5.5), facecolor=BG)
        gs  = fig.add_gridspec(2, 2, hspace=0.45, wspace=0.35)
        self._s_radar = fig.add_subplot(gs[:,0], polar=True, facecolor=BG2)
        self._s_bar   = fig.add_subplot(gs[0,1], facecolor=BG2)
        self._s_score = fig.add_subplot(gs[1,1], facecolor=BG2)
        self._sfig    = fig
        cvs = FigureCanvasTkAgg(fig, rp)
        cvs.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self._scvs = cvs

        self._update_summary()

    def _update_summary(self):
        r = self.results
        kv   = r.get("motor_kv",  0)
        rpm  = r.get("motor_rpm", 0)
        mpow = r.get("motor_pow", 0)
        pthr = r.get("prop_thrust", 0)
        ppow = r.get("prop_power",  0)
        bv   = r.get("batt_volt",  0)
        bc   = r.get("batt_cap",   0)
        bmi  = r.get("batt_max_i", 0)
        ft   = r.get("flight_time",0)
        tw   = r.get("tw",         0)
        hov  = r.get("hover",      0)
        mass = r.get("total_w",    0)
        tthr = r.get("total_t",    0)
        erec = r.get("esc_rec",    0)
        etemp= r.get("esc_temp",   25)

        self._svars["m_kv"].set(f"{kv:.0f} RPM/V")
        self._svars["m_rpm"].set(f"{rpm:,.0f} RPM")
        self._svars["m_pow"].set(f"{mpow:.0f} W")
        self._svars["p_thr"].set(f"{pthr:.1f} g")
        self._svars["p_pow"].set(f"{ppow:.1f} W")
        self._svars["b_v"].set(f"{bv:.2f} V")
        self._svars["b_cap"].set(f"{bc:.0f} mAh")
        self._svars["b_maxi"].set(f"{bmi:.1f} A")
        self._svars["b_ft"].set(f"{ft:.1f} dakika")
        self._svars["tw"].set(f"{tw:.2f} : 1")
        self._svars["hover"].set(f"% {hov:.0f}")
        self._svars["mass"].set(f"{mass:.0f} g")
        self._svars["thr_t"].set(f"{tthr:.0f} g")
        self._svars["esc_r"].set(f"{erec} A")
        self._svars["esc_t"].set(f"{etemp:.0f} °C")

        # Tavsiyeler
        recs = []
        if tw < 2.0:
            recs.append("⚠ T/W < 2:1 — daha güçlü motor veya hafif çerçeve")
        elif tw >= 4.0:
            recs.append("✓ Mükemmel T/W oranı — çok çevik uçuş")
        else:
            recs.append("✓ T/W iyi aralıkta")
        if ft > 0 and ft < 5:
            recs.append("⚠ Uçuş süresi kısa — büyük batarya deneyin")
        elif ft >= 15:
            recs.append("✓ Uzun uçuş süresi")
        if hov > 65:
            recs.append("⚠ Hover gaz %65 üzeri — batarya hızlı tüketilecek")
        if etemp > 90:
            recs.append("⚠ ESC sıcaklığı kritik — soğutma şart!")
        if not recs:
            recs.append("Tüm değerler normal aralıkta görünüyor.")
        self._rec_var.set("\n".join(recs))

        # Radar
        cats   = ["İtme\nGücü","Uçuş\nSüresi","Çeviklik",
                  "Verimlilik","Enerji\nYoğ."]
        scores = [
            min(1.0, pthr/2000),
            min(1.0, ft/30),
            min(1.0, tw/5),
            min(1.0, (pthr/max(1,ppow))/15),
            min(1.0, (bc/1000*bv)/80),
        ]
        N = len(cats)
        angles = np.linspace(0, 2*np.pi, N, endpoint=False).tolist()
        angles += angles[:1]; scores += scores[:1]

        ax = self._s_radar
        ax.clear(); ax.set_facecolor(BG2)
        ax.plot(angles, scores, "o-", lw=2, color=ACCENT)
        ax.fill(angles, scores, alpha=0.22, color=ACCENT)
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(cats, fontsize=7, color=FG)
        ax.set_ylim(0,1)
        ax.set_yticks([.25,.5,.75,1]); ax.set_yticklabels(["25","50","75","100"],
                                                            fontsize=5, color=FG2)
        ax.grid(color=BORDER, alpha=0.5)
        ax.spines["polar"].set_visible(False)
        ax.set_title("Performans\nProfili", color=FG, fontsize=9, pad=10)

        # Bar chart — bileşen kütleleri
        ax2 = self._s_bar; ax2.clear(); ax2.set_facecolor(BG3)
        nm = int(r.get("n_motors",4))
        bw_val = 180  # örnek
        comps  = ["Frame","Motor","Batarya","ESC","FC","Payload"]
        vals2  = [100, nm*28, 180, nm*8, 45, 0]
        colors = [ACCENT,ACCENT2,ACCENT3,ACCENT4,WARN,"#888"]
        ax2.barh(comps, vals2, color=colors, height=0.55)
        ax2.set_xlabel("Ağırlık (g)", color=FG2, fontsize=7)
        ax2.set_title("Bileşen Ağırlıkları", color=FG, fontsize=8)
        ax2.tick_params(colors=FG, labelsize=7)
        ax2.spines[:].set_color(BORDER)

        # Çizgi grafik — Uçuş süresi vs akım
        ax3 = self._s_score; ax3.clear(); ax3.set_facecolor(BG3)
        cap_val = max(1, bc)
        currents = np.linspace(1, bmi or 30, 60)
        ft_vals  = [calc_flight_time(cap_val, i) for i in currents]
        ax3.plot(currents, ft_vals, color=ACCENT2, lw=2)
        if bmi > 0:
            ax3.axvline(bmi*0.5, color=WARN, lw=1.2, linestyle="--",
                        label="Ort. Akım")
        ax3.fill_between(currents, ft_vals, alpha=0.15, color=ACCENT2)
        ax3.set_xlabel("Akım (A)", color=FG2, fontsize=7)
        ax3.set_ylabel("Uçuş Süresi (dk)", color=FG2, fontsize=7)
        ax3.set_title("Uçuş Süresi / Akım", color=FG, fontsize=8)
        ax3.tick_params(colors=FG, labelsize=7)
        ax3.spines[:].set_color(BORDER)
        ax3.legend(fontsize=7, labelcolor=FG, facecolor=BG2, edgecolor=BORDER)

        self._sfig.tight_layout(pad=2)
        self._scvs.draw()


# ============================================================
# GİRİŞ NOKTASI
# ============================================================
if __name__ == "__main__":
    app = DroneCalcApp()
    app.mainloop()

