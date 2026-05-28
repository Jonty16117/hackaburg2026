from PIL import Image, ImageDraw, ImageFont
import os

W, H = 1600, 1100
img = Image.new("RGB", (W, H), "#1a1a2e")
draw = ImageDraw.Draw(img)

try:
    font_title = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 28)
    font_hdr = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 20)
    font_body = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 16)
    font_small = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 13)
    font_mono = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", 14)
except Exception:
    font_title = ImageFont.load_default()
    font_hdr = font_title
    font_body = font_title
    font_small = font_title
    font_mono = font_title


def box(x, y, w, h, fill, outline, text="", font=None, text_color="#ffffff", radius=8):
    if radius:
        draw.rounded_rectangle([x, y, x + w, y + h], radius=radius, fill=fill, outline=outline, width=2)
    else:
        draw.rectangle([x, y, x + w, y + h], fill=fill, outline=outline, width=2)
    if text and font:
        lines = text.split("\n")
        tw, th = 0, 0
        for line in lines:
            bbox = draw.textbbox((0, 0), line, font=font)
            tw = max(tw, bbox[2] - bbox[0])
            th += bbox[3] - bbox[1] + 4
        ty = y + (h - th) // 2
        for line in lines:
            bbox = draw.textbbox((0, 0), line, font=font)
            line_w = bbox[2] - bbox[0]
            draw.text((x + (w - line_w) // 2, ty), line, fill=text_color, font=font)
            ty += bbox[3] - bbox[1] + 4


def line(x1, y1, x2, y2, color, width=2):
    draw.line([(x1, y1), (x2, y2)], fill=color, width=width)


def label(x, y, text, color="#cccccc", font=None):
    f = font or font_small
    draw.text((x, y), text, fill=color, font=f)


def arrow_line(x1, y1, x2, y2, color, width=2):
    draw.line([(x1, y1), (x2, y2)], fill=color, width=width)
    import math
    angle = math.atan2(y2 - y1, x2 - x1)
    arrow_len = 10
    ax = x2 - arrow_len * math.cos(angle - 0.4)
    ay = y2 - arrow_len * math.sin(angle - 0.4)
    bx = x2 - arrow_len * math.cos(angle + 0.4)
    by = y2 - arrow_len * math.sin(angle + 0.4)
    draw.polygon([(x2, y2), (ax, ay), (bx, by)], fill=color)


# ── title ──
draw.text((W // 2, 30), "DuckBot — Assembly Wiring Diagram", fill="#ffffff", font=font_title, anchor="ma")

# ── LAYOUT ──
# Battery at top center
# Two ESCs below (left and right)
# Pi in bottom center
# Two thrusters at bottom (left and right)

# ── BATTERY ──
bx, by, bw, bh = W//2 - 120, 70, 240, 90
box(bx, by, bw, bh, "#2d1f1f", "#e74c3c", "LiPo Battery\n4S 14.8V", font_hdr)
label(bx + 50, by + 12, "5000mAh+", font=font_small, color="#e74c3c")

# Battery terminals
term_x = bx + bw // 2
line(term_x - 30, by + bh, term_x - 30, by + bh + 15, "#e74c3c", 3)
line(term_x + 30, by + bh, term_x + 30, by + bh + 15, "#111111", 3)
label(term_x - 70, by + bh + 18, "(+) RED", font=font_small, color="#e74c3c")
label(term_x + 25, by + bh + 18, "(-) BLACK", font=font_small, color="#888888")

# ── LEFT ESC ──
lx, ly, lw, lh = 130, 280, 300, 200
box(lx, ly, lw, lh, "#16213e", "#3498db", "LEFT ESC\nAPISQUEEN 30A", font_hdr)
label(lx + 60, ly + 55, "Bi-directional", font=font_small, color="#3498db")

# ESC left-side labels (motor side)
label(lx + 15, ly + 85, "MOTOR SIDE:", font=font_small, color="#3498db")
line(lx, ly + 90, lx + lw, ly + 90, "#3498db", 1)
box(lx + 20, ly + 96, 14, 14, "#111111", "#555555")
box(lx + 20, ly + 116, 14, 14, "#111111", "#555555")
box(lx + 20, ly + 136, 14, 14, "#111111", "#555555")
label(lx + 42, ly + 95, "Black 1 — Phase U", font=font_small, color="#2ecc71")
label(lx + 42, ly + 115, "Black 2 — Phase V", font=font_small, color="#f1c40f")
label(lx + 42, ly + 135, "Black 3 — Phase W", font=font_small, color="#5dade2")

# ESC right-side labels (power/signal)
label(lx + 155, ly + 85, "SIGNAL SIDE:", font=font_small, color="#e74c3c")
line(lx, ly + 150, lx + lw, ly + 150, "#e74c3c", 1)
box(lx + 175, ly + 158, 14, 14, "#e74c3c", "#e74c3c")
box(lx + 175, ly + 178, 14, 14, "#8B4513", "#8B4513")
box(lx + 220, ly + 168, 14, 14, "#f39c12", "#f39c12")
label(lx + 195, ly + 157, "B+ (red)", font=font_small, color="#e74c3c")
label(lx + 195, ly + 177, "B- (black)", font=font_small, color="#888888")
label(lx + 195, ly + 167, "Signal 3-pin", font=font_small, color="#f39c12")

# ── RIGHT ESC ──
rx, ry, rw, rh = W - 430, 280, 300, 200
box(rx, ry, rw, rh, "#16213e", "#3498db", "RIGHT ESC\nAPISQUEEN 30A", font_hdr)
label(rx + 60, ry + 55, "Bi-directional", font=font_small, color="#3498db")

label(rx + 15, ry + 85, "MOTOR SIDE:", font=font_small, color="#3498db")
line(rx, ry + 90, rx + rw, ry + 90, "#3498db", 1)
box(rx + 20, ry + 96, 14, 14, "#111111", "#555555")
box(rx + 20, ry + 116, 14, 14, "#111111", "#555555")
box(rx + 20, ry + 136, 14, 14, "#111111", "#555555")
label(rx + 42, ry + 95, "Black 1 — Phase U", font=font_small, color="#2ecc71")
label(rx + 42, ry + 115, "Black 2 — Phase V", font=font_small, color="#f1c40f")
label(rx + 42, ry + 135, "Black 3 — Phase W", font=font_small, color="#5dade2")

label(rx + 155, ry + 85, "SIGNAL SIDE:", font=font_small, color="#e74c3c")
line(rx, ry + 150, rx + rw, ry + 150, "#e74c3c", 1)
box(rx + 175, ry + 158, 14, 14, "#e74c3c", "#e74c3c")
box(rx + 175, ry + 178, 14, 14, "#8B4513", "#8B4513")
box(rx + 220, ry + 168, 14, 14, "#f39c12", "#f39c12")
label(rx + 195, ry + 157, "B+ (red)", font=font_small, color="#e74c3c")
label(rx + 195, ry + 177, "B- (black)", font=font_small, color="#888888")
label(rx + 195, ry + 167, "Signal 3-pin", font=font_small, color="#f39c12")

# ── RASPBERRY PI ──
px, py, pw, ph = W//2 - 220, 540, 440, 200
box(px, py, pw, ph, "#1a1a1a", "#2ecc71", "Raspberry Pi", font_hdr)
label(px + 120, py + 35, "USB Powered (5V)", font=font_small, color="#2ecc71")

# GPIO pins representation
label(px + 20, py + 60, "GPIO 12 (pin 32)", font=font_small, color="#e67e22")
label(px + 20, py + 82, "GPIO 13 (pin 33)", font=font_small, color="#e67e22")
label(px + 20, py + 104, "GND (pin 6)", font=font_small, color="#888888")
label(px + 20, py + 126, "GND (pin 14)", font=font_small, color="#888888")
label(px + 20, py + 148, "BEC 5V — DO NOT", font=font_small, color="#e74c3c")
label(px + 20, py + 168, "  CONNECT", font=font_small, color="#e74c3c")

# GPIO mark on right side of Pi box
gpio_rx = px + pw - 80
for i, lbl in enumerate(["GPIO12", "GPIO13", "GND 6", "GND 14", "RED (nc)", ""]):
    y = py + 60 + i * 22
    if lbl:
        box(gpio_rx, y - 6, 70, 18, "#16213e", "#e67e22", lbl, font=font_small, text_color="#e67e22")

# ── LEFT THRUSTER ──
ltx, lty, ltw, lth = 100, 820, 340, 140
box(ltx, lty, ltw, lth, "#1b2a1b", "#2ecc71", "LEFT U01 THRUSTER", font_hdr)
label(ltx + 80, lty + 35, "CCW — 2Kg thrust", font=font_small, color="#2ecc71")
label(ltx + 30, lty + 60, "3-wire: Green / Yellow / Sky Blue", font=font_small, color="#aaaaaa")
label(ltx + 30, lty + 85, "75×75mm  178g  12-16V  390W  17A", font=font_small, color="#aaaaaa")

# ── RIGHT THRUSTER ──
rtx, rty, rtw, rth = W - 440, 820, 340, 140
box(rtx, rty, rtw, rth, "#1b2a1b", "#2ecc71", "RIGHT U01 THRUSTER", font_hdr)
label(rtx + 80, rty + 35, "CW — 2Kg thrust", font=font_small, color="#2ecc71")
label(rtx + 30, rty + 60, "3-wire: Green / Yellow / Sky Blue", font=font_small, color="#aaaaaa")
label(rtx + 30, rty + 85, "75×75mm  178g  12-16V  390W  17A", font=font_small, color="#aaaaaa")

# ── POWER WIRES (Battery → ESCs) ──
# Left side
line(bx + bw//2 - 30, by + bh + 15, lx + 175 + 7, ly + 158 + 7, "#e74c3c", 3)
line(bx + bw//2 + 30, by + bh + 15, lx + 175 + 7, ly + 178 + 7, "#555555", 3)
# Right side
line(bx + bw//2 - 30, by + bh + 15, rx + 175 + 7, ry + 158 + 7, "#e74c3c", 3)
line(bx + bw//2 + 30, by + bh + 15, rx + 175 + 7, ry + 178 + 7, "#555555", 3)

label(term_x - 130, by + bh + 65, "RED (+)", font=font_small, color="#e74c3c")
label(term_x, by + bh + 65, "BLACK (-)", font=font_small, color="#888888")

# ── MOTOR WIRES (ESCs → Thrusters) ──
# Left ESC → Left Thruster
line(lx + 27, ly + 103, ltx + 40, lty + 10, "#2ecc71", 2)
line(lx + 27, ly + 123, ltx + 100, lty + 10, "#f1c40f", 2)
line(lx + 27, ly + 143, ltx + 160, lty + 10, "#5dade2", 2)
label(lx + 30, ly + 180, "G Y SB", font=font_small, color="#2ecc71")

# Right ESC → Right Thruster
line(rx + 27, ry + 103, rtx + 40, rty + 10, "#2ecc71", 2)
line(rx + 27, ry + 123, rtx + 100, rty + 10, "#f1c40f", 2)
line(rx + 27, ry + 143, rtx + 160, rty + 10, "#5dade2", 2)
label(rx + 30, ry + 180, "G Y SB", font=font_small, color="#2ecc71")

# ── SIGNAL WIRES (ESCs → Pi) ──
# Left ESC signal
sig_lx, sig_ly = lx + 220 + 7, ly + 168 + 7
line(sig_lx, sig_ly, px + 30, py + 64, "#f39c12", 2)
label(sig_lx - 60, sig_ly - 10, "Orange →", font=font_small, color="#f39c12")

# Left ESC GND
line(lx + 175 + 7, ly + 168 + 7, px + 30, py + 108, "#8B4513", 2)
label(lx + 40, ly + 200, "Brown → GND (pin 6)", font=font_small, color="#8B4513")

# Right ESC signal
sig_rx, sig_ry = rx + 220 + 7, ry + 168 + 7
line(sig_rx, sig_ry, px + pw - 30, py + 64, "#f39c12", 2)
label(sig_rx + 20, sig_ry - 10, "← Orange", font=font_small, color="#f39c12")

# Right ESC GND
line(rx + 175 + 7, ry + 168 + 7, px + pw - 30, py + 130, "#8B4513", 2)
label(rx + 40, ry + 200, "Brown → GND (pin 14)", font=font_small, color="#8B4513")

# ── LEGEND ──
leg_x, leg_y = 30, H - 55
label(leg_x, leg_y, "POWER:", font=font_small, color="#888888")
line(leg_x + 70, leg_y + 10, leg_x + 110, leg_y + 10, "#e74c3c", 3)
label(leg_x + 115, leg_y, "RED (+)    ", font=font_small, color="#e74c3c")
line(leg_x + 200, leg_y + 10, leg_x + 240, leg_y + 10, "#555555", 3)
label(leg_x + 245, leg_y, "BLACK (-)", font=font_small, color="#888888")
label(leg_x + 70, leg_y + 20, "SIGNAL:", font=font_small, color="#888888")
line(leg_x + 130, leg_y + 30, leg_x + 170, leg_y + 30, "#f39c12", 2)
label(leg_x + 175, leg_y + 20, "ORANGE (PWM)", font=font_small, color="#f39c12")
line(leg_x + 305, leg_y + 30, leg_x + 345, leg_y + 30, "#8B4513", 2)
label(leg_x + 350, leg_y + 20, "BROWN (GND)", font=font_small, color="#8B4513")

# ── WARNING ──
warn_y = H - 100
draw.text((W//2, warn_y), "⚠  DO NOT connect ESC red signal wire (BEC 5V) to Pi — Pi is USB-powered  ⚠", fill="#e74c3c", font=font_small, anchor="ma")
draw.text((W//2, warn_y + 25), "Both ESCs share the same LiPo battery via parallel connector or Y-harness", fill="#aaaaaa", font=font_small, anchor="ma")

# ── save ──
out_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
out_path = os.path.join(out_dir, "assembly_diagram.jpg")
img.save(out_path, "JPEG", quality=95)
print(f"Saved: {out_path}")
