"""
NE555 单稳态延时电路 —— PCB 级原理图
====================================
生成与实际电路原理图一致的 NE555 单稳态接线图。
DIP-8 封装、外围阻容、触发微分、去耦 — 一比一对应。

运行: python ne555_schematic.py
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.font_manager as fm
import numpy as np

# ---- 中文字体适配 ----
_zh_font = None
for _f in fm.fontManager.ttflist:
    if any(k in _f.name for k in ['SimHei', 'Microsoft YaHei', 'WenQuanYi', 'Noto Sans CJK']):
        _zh_font = _f.name
        break
if _zh_font:
    plt.rcParams["font.family"] = _zh_font
else:
    plt.rcParams["font.family"] = "sans-serif"

plt.rcParams.update({
    "figure.dpi": 180,
    "font.size": 9,
})

# ============================================================
# 画图引擎: 原理图原语
# ============================================================

def draw_chip_box(ax, x, y, w, h, label="NE555"):
    """DIP-8 芯片本体"""
    rect = mpatches.FancyBboxPatch((x, y), w, h,
                                   boxstyle="round,pad=0.12",
                                   facecolor="#1a1a2e", edgecolor="#0f0f23",
                                   linewidth=2.2, zorder=3)
    ax.add_patch(rect)
    # 半圆缺口(1脚标记)
    notch = mpatches.Arc((x + w/2, y + h), w*0.28, h*0.18,
                         theta1=0, theta2=180, color="white", linewidth=1.6, zorder=4)
    ax.add_patch(notch)
    # 标签
    ax.text(x + w/2, y + h/2, label, ha="center", va="center",
            fontsize=11, fontweight="bold", color="white", zorder=5)

def draw_pin(ax, x, y, num, name, side, ax_other=None):
    """
    画引脚。
    side='L' 左边 (1-4), side='R' 右边 (5-8)
    """
    lead_len = 0.55
    if side == 'L':
        dx = -lead_len
        ha = 'right'
        txt_x = x - lead_len - 0.08
    else:
        dx = +lead_len
        ha = 'left'
        txt_x = x + lead_len + 0.08

    # 引脚线
    ax.plot([x, x + dx], [y, y], color='#FFD54F', linewidth=2.2, zorder=2, solid_capstyle='round')
    # 焊盘
    ax.plot(x, y, 'o', color='#FFD54F', markersize=6, zorder=3, markeredgecolor='#FFA000', markeredgewidth=1.2)

    # 编号
    ax.text(txt_x, y + 0.13, str(num), fontsize=8, ha=ha, va='bottom',
            color='#FFD54F', fontweight='bold')
    # 功能名
    ax.text(txt_x, y - 0.13, name, fontsize=7, ha=ha, va='top',
            color='#B0BEC5')

def draw_wire(ax, points, **kwargs):
    """画任意折线"""
    xs, ys = zip(*points)
    ax.plot(xs, ys, color='#ECEFF1', linewidth=1.3, **kwargs)

def draw_resistor(ax, x, y, angle=0):
    """画电阻符号 (水平)"""
    # zigzag
    l = 0.35
    segs = 7
    xs = [x - l/2]
    ys = [y]
    amp = 0.06
    for i in range(segs):
        xs.append(xs[-1] + l/segs)
        ys.append(y + amp if i % 2 == 0 else y - amp)
    ax.plot(xs, ys, color='#ECEFF1', linewidth=1.4, solid_capstyle='round')

def draw_capacitor(ax, x, y, polar=False):
    """画电容符号 (水平). polar=True 加 '+' 标识"""
    gap = 0.10
    plate_h = 0.28
    ax.plot([x - gap/2, x - gap/2], [y - plate_h/2, y + plate_h/2],
            color='#ECEFF1', linewidth=1.8)
    ax.plot([x + gap/2, x + gap/2], [y - plate_h/2, y + plate_h/2],
            color='#ECEFF1', linewidth=1.8)
    if polar:
        ax.text(x - gap/2 - 0.03, y + plate_h/2 + 0.05, '+', color='#ECEFF1', fontsize=7, ha='center')

def draw_ground(ax, x, y):
    """画接地符号"""
    for i, dy in enumerate([0, -0.06, -0.12]):
        w = 0.14 - i * 0.04
        ax.plot([x - w, x + w], [y + dy, y + dy], color='#ECEFF1', linewidth=1.1)

def draw_named_node(ax, x, y, text, ha='left', va='bottom', offset=(0.06, 0.06)):
    """节点标签"""
    ax.text(x + offset[0], y + offset[1], text, fontsize=8,
            color='#81D4FA', ha=ha, va=va,
            bbox=dict(boxstyle='round,pad=0.15', fc='#1a1a2e', ec='none', alpha=0.8))

def draw_junction(ax, x, y):
    """连接点"""
    ax.plot(x, y, 'o', color='#ECEFF1', markersize=4, zorder=6)

# ============================================================
# 主函数
# ============================================================
def draw_full_schematic():
    fig, ax = plt.subplots(figsize=(15, 9.5))
    ax.set(xlim=(-1.2, 14.2), ylim=(-1.5, 11.2))
    ax.set_aspect('equal')
    ax.axis('off')
    fig.patch.set_facecolor('#0d1117')
    ax.set_facecolor('#0d1117')

    # ---- 芯片 ----
    chip_x, chip_y, chip_w, chip_h = 5.2, 3.0, 2.6, 5.5
    draw_chip_box(ax, chip_x, chip_y, chip_w, chip_h)

    # 引脚坐标 (左排 1-4, 右排 5-8)
    pin_y = [chip_y + chip_h - 0.7,  # 1
             chip_y + chip_h - 2.0,  # 2
             chip_y + 2.0,           # 3
             chip_y + 0.7]           # 4

    pin_names_L = ['GND', 'TRIG', 'OUT', 'RESET']
    pin_nums_L  = [1, 2, 3, 4]
    pin_names_R = ['Vcc', 'THRES', 'DISCH', 'CONT']
    pin_nums_R  = [8, 6, 7, 5]

    # 画全部引脚
    for i in range(4):
        draw_pin(ax, chip_x,           pin_y[i], pin_nums_L[i], pin_names_L[i], 'L')
        draw_pin(ax, chip_x + chip_w,  pin_y[i], pin_nums_R[i], pin_names_R[i], 'R')

    # ===== 外围电路 =====
    p1_y = pin_y[0]   # GND
    p2_y = pin_y[1]   # TRIG
    p3_y = pin_y[2]   # OUT
    p4_y = pin_y[3]   # RESET
    p8_y = pin_y[0]   # Vcc
    p6_y = pin_y[1]   # THRES
    p7_y = pin_y[2]   # DISCH
    p5_y = pin_y[3]   # CONT

    left_x  = chip_x
    right_x = chip_x + chip_w

    # ---------- 电源轨 ----------
    rail_y_top = chip_y + chip_h + 0.9
    rail_y_bot = chip_y - 0.9

    # Vcc rail (顶部)
    ax.plot([4.0, right_x + 2.8], [rail_y_top, rail_y_top],
            color='#EF5350', linewidth=1.6)
    # GND rail (底部)
    ax.plot([4.0, right_x + 2.8], [rail_y_bot, rail_y_bot],
            color='#455A64', linewidth=1.6)

    # ---- 8脚 Vcc 接电源轨 ----
    draw_wire(ax, [(right_x, p8_y), (right_x + 1.0, p8_y), (right_x + 1.0, rail_y_top)])
    draw_named_node(ax, right_x + 1.0, rail_y_top, '+Vcc (5V)', ha='left', va='bottom', offset=(0.06, 0.05))

    # Vcc 去耦电容 (靠近 8脚)
    cap_dec_x = right_x + 1.6
    draw_capacitor(ax, cap_dec_x, p8_y - 0.8)
    ax.text(cap_dec_x + 0.4, p8_y - 0.8, '100nF\n(去耦)', fontsize=6, color='#90A4AE', ha='left', va='center')
    draw_wire(ax, [(right_x + 1.0, p8_y), (cap_dec_x, p8_y)])
    draw_wire(ax, [(cap_dec_x, p8_y - 0.8 - 0.14), (cap_dec_x, p8_y - 1.6)])
    draw_junction(ax, right_x + 1.0, p8_y)

    # ---- 1脚 GND 接地 ----
    draw_wire(ax, [(left_x, p1_y), (left_x - 1.0, p1_y), (left_x - 1.0, rail_y_bot)])
    draw_ground(ax, left_x - 1.0, rail_y_bot)
    draw_named_node(ax, left_x - 1.0, rail_y_bot, 'GND', ha='right', va='top', offset=(-0.06, -0.1))

    # ---- 4脚 RESET 上拉 ----
    draw_wire(ax, [(left_x, p4_y), (left_x - 1.2, p4_y), (left_x - 1.2, rail_y_top)])
    draw_resistor(ax, left_x - 1.2, p4_y + 0.9, angle=90)
    ax.text(left_x - 1.5, p4_y + 0.9, '10kΩ', fontsize=7, color='#90A4AE', ha='right', va='center')
    draw_junction(ax, left_x - 1.2, rail_y_top)

    # ---- 5脚 CONT 对地电容 ----
    draw_wire(ax, [(right_x, p5_y), (right_x + 0.7, p5_y)])
    draw_capacitor(ax, right_x + 0.7, p5_y - 0.5)
    ax.text(right_x + 1.2, p5_y - 0.5, '10nF\n(抗扰)', fontsize=6, color='#90A4AE', ha='left', va='center')
    draw_wire(ax, [(right_x + 0.7, p5_y), (right_x + 0.7, p5_y - 0.35)])
    draw_wire(ax, [(right_x + 0.7, p5_y - 0.65), (right_x + 0.7, p5_y - 1.0)])
    draw_ground(ax, right_x + 0.7, p5_y - 1.0)

    # ---- 定时元件 R (7脚 DISCH <--> Vcc) ----
    # 7脚内部接放电管，外部接 R 到 Vcc
    r_x = right_x + 2.0
    draw_wire(ax, [(right_x, p7_y), (r_x, p7_y)])
    draw_resistor(ax, r_x, p7_y)
    draw_wire(ax, [(r_x + 0.25, p7_y), (r_x + 0.25, rail_y_top)])
    draw_junction(ax, r_x + 0.25, rail_y_top)
    ax.text(r_x, p7_y - 0.28, 'R_timing', fontsize=8, color='#FFCC80', ha='center', fontweight='bold')
    ax.text(r_x, p7_y - 0.45, '100kΩ', fontsize=7, color='#90A4AE', ha='center')

    # ---- 定时电容 C (6+7脚 <--> GND) ----
    # 6脚和7脚在外部短接
    draw_wire(ax, [(right_x, p6_y), (r_x, p6_y)])
    draw_junction(ax, r_x, p6_y)
    draw_junction(ax, r_x, p7_y)
    draw_wire(ax, [(r_x, p6_y), (r_x, p7_y)])  # 6-7 短接线

    # C 向下到地
    c_y = p6_y - 1.0
    draw_wire(ax, [(r_x, p6_y), (r_x, c_y + 0.3)])
    draw_capacitor(ax, r_x, c_y, polar=True)
    ax.text(r_x + 0.45, c_y, 'C_timing', fontsize=8, color='#FFCC80', ha='left', va='center', fontweight='bold')
    ax.text(r_x + 0.45, c_y - 0.18, '10μF', fontsize=7, color='#90A4AE', ha='left', va='center')
    draw_wire(ax, [(r_x, c_y - 0.14 - 0.1), (r_x, c_y - 0.7)])
    draw_ground(ax, r_x, c_y - 0.7)

    # ---- 触发电路 (2脚 TRIG) ----
    trig_node_x = left_x - 2.5
    draw_wire(ax, [(left_x, p2_y), (trig_node_x, p2_y)])

    # 上拉电阻到 Vcc
    draw_wire(ax, [(trig_node_x, p2_y), (trig_node_x, rail_y_top)])
    draw_resistor(ax, trig_node_x, p2_y + 1.3)
    ax.text(trig_node_x - 0.35, p2_y + 1.3, '10kΩ', fontsize=7, color='#90A4AE', ha='right', va='center')
    draw_junction(ax, trig_node_x, rail_y_top)

    # 耦合电容 -> 按键 -> GND (微分电路)
    trig_cap_y = p2_y - 0.8
    draw_wire(ax, [(trig_node_x, p2_y), (trig_node_x, trig_cap_y + 0.25)])
    draw_capacitor(ax, trig_node_x, trig_cap_y)
    ax.text(trig_node_x + 0.4, trig_cap_y - 0.05, '10nF\n(耦合)', fontsize=6, color='#90A4AE', ha='left', va='center')

    # 按键
    btn_y = trig_cap_y - 0.7
    draw_wire(ax, [(trig_node_x, trig_cap_y - 0.19), (trig_node_x, btn_y + 0.15)])
    btn = mpatches.FancyBboxPatch((trig_node_x - 0.22, btn_y - 0.07), 0.44, 0.22,
                                  boxstyle="round,pad=0.02",
                                  facecolor='#37474F', edgecolor='#ECEFF1', linewidth=1.2)
    ax.add_patch(btn)
    ax.text(trig_node_x, btn_y + 0.04, 'SW1', fontsize=7, color='#ECEFF1', ha='center', va='center')
    draw_wire(ax, [(trig_node_x, btn_y - 0.07), (trig_node_x, trig_cap_y - 1.6)])
    draw_ground(ax, trig_node_x, trig_cap_y - 1.6)
    draw_named_node(ax, trig_node_x + 0.05, trig_node_x, '触发按键\n(低有效)', ha='left', va='bottom', offset=(0.1, -0.1))

    # ---- 输出 (3脚 OUT) ----
    out_x = left_x - 2.0
    draw_wire(ax, [(left_x, p3_y), (out_x, p3_y)])
    # LED + 限流电阻
    led_r_y = p3_y
    draw_wire(ax, [(out_x, led_r_y), (out_x - 0.7, led_r_y)])
    draw_resistor(ax, out_x - 0.7, led_r_y)
    ax.text(out_x - 0.7, led_r_y - 0.25, '330Ω', fontsize=7, color='#90A4AE', ha='center', va='top')

    # LED 符号
    led_x = out_x - 1.6
    draw_wire(ax, [(out_x - 0.7 - 0.17, led_r_y), (led_x + 0.2, led_r_y)])
    # 二极管三角形
    tri = mpatches.Polygon([[led_x, led_r_y - 0.2], [led_x, led_r_y + 0.2], [led_x + 0.35, led_r_y]],
                           facecolor='#EF5350', edgecolor='#EF5350', linewidth=1.2, alpha=0.9)
    ax.add_patch(tri)
    ax.plot([led_x + 0.35, led_x + 0.5], [led_r_y, led_r_y + 0.25], color='#EF5350', linewidth=1.2)
    ax.plot([led_x + 0.35, led_x + 0.5], [led_r_y, led_r_y - 0.25], color='#EF5350', linewidth=1.2)
    draw_wire(ax, [(led_x + 0.5, led_r_y), (led_x + 0.5, led_r_y - 0.8)])
    draw_ground(ax, led_x + 0.5, led_r_y - 0.8)
    draw_named_node(ax, led_x, led_r_y + 0.35, 'LED 指示\n(高亮=t_d内)', ha='center', va='bottom', offset=(0, 0.02))

    # ---- 去耦电容补充 (Vcc-GND 大电容) ----
    bulk_x = right_x + 3.0
    draw_wire(ax, [(bulk_x, rail_y_top), (bulk_x, p8_y + 0.3)])
    draw_capacitor(ax, bulk_x, p8_y - 0.3, polar=True)
    ax.text(bulk_x + 0.4, p8_y - 0.3, '10μF\n(储能)', fontsize=6, color='#90A4AE', ha='left', va='center')
    draw_wire(ax, [(bulk_x, p8_y - 0.44), (bulk_x, p8_y - 1.4)])
    draw_ground(ax, bulk_x, p8_y - 1.4)

    # ===== 内部功能标注 =====
    # 比较器区
    ax.annotate('R1\n5kΩ', xy=(chip_x + chip_w*0.32, chip_y + chip_h*0.78),
                fontsize=6, color='#78909C', ha='center', va='center')
    ax.annotate('R2\n5kΩ', xy=(chip_x + chip_w*0.32, chip_y + chip_h*0.52),
                fontsize=6, color='#78909C', ha='center', va='center')
    ax.annotate('R3\n5kΩ', xy=(chip_x + chip_w*0.32, chip_y + chip_h*0.26),
                fontsize=6, color='#78909C', ha='center', va='center')
    ax.annotate('C1', xy=(chip_x + chip_w*0.65, chip_y + chip_h*0.78),
                fontsize=6, color='#78909C', ha='center', va='center')
    ax.annotate('C2', xy=(chip_x + chip_w*0.65, chip_y + chip_h*0.26),
                fontsize=6, color='#78909C', ha='center', va='center')
    ax.annotate('SR\nFF', xy=(chip_x + chip_w*0.5, chip_y + chip_h*0.52),
                fontsize=6, color='#78909C', ha='center', va='center')
    ax.annotate('放电管', xy=(chip_x + chip_w*0.38, chip_y + chip_h*0.12),
                fontsize=6, color='#78909C', ha='center', va='center')

    # ===== 时序波形预览窗 =====
    box_inset = mpatches.FancyBboxPatch((0.25, 7.5), 4.2, 3.2,
                                        boxstyle="round,pad=0.15",
                                        facecolor='#161b22', edgecolor='#30363d', linewidth=1.0, zorder=8)
    ax.add_patch(box_inset)
    ax.text(2.35, 10.5, '时序预览', fontsize=9, fontweight='bold', color='#81D4FA', ha='center', zorder=9)

    # 简化波形
    t = np.linspace(0, 4, 200)
    w_trig = np.zeros_like(t)
    w_trig[(t > 0.4) & (t < 0.55)] = 1
    w_out = np.zeros_like(t)
    w_out[(t > 0.55) & (t < 2.8)] = 1
    w_cap = np.zeros_like(t)
    w_cap[(t > 0.55)] = 3.3 * (1 - np.exp(-(t[(t > 0.55)] - 0.55) / 0.7))
    w_cap[:np.argmax(t > 0.55)] = 0

    base_y = 7.8
    scale_x = 0.95
    scale_y = 0.55
    ax.plot(0.6 + t * scale_x, base_y + 2.3 + w_trig * scale_y * 0.7, color='#2196F3', linewidth=1.3, zorder=9)
    ax.text(0.3, base_y + 2.65, 'TRIG', fontsize=6, color='#2196F3', ha='right', va='center', zorder=9)
    ax.plot(0.6 + t * scale_x, base_y + 1.4 + w_out * scale_y * 0.7, color='#E91E63', linewidth=1.3, zorder=9)
    ax.text(0.3, base_y + 1.75, 'OUT', fontsize=6, color='#E91E63', ha='right', va='center', zorder=9)
    ax.plot(0.6 + t * scale_x, base_y + 0.5 + w_cap * scale_y, color='#4CAF50', linewidth=1.3, zorder=9)
    ax.text(0.3, base_y + 0.85, 'Vcap', fontsize=6, color='#4CAF50', ha='right', va='center', zorder=9)
    ax.axhline(base_y + 0.5 + 3.3 * scale_y * 2/3, xmin=0.05, xmax=0.98,
               color='red', ls=':', linewidth=0.8, zorder=9)
    ax.text(4.2, base_y + 0.5 + 3.3 * scale_y * 2/3, '2/3Vcc', fontsize=5.5, color='red', zorder=9)

    # ===== 公式/设计参数盒 =====
    box_eq = mpatches.FancyBboxPatch((9.2, 7.5), 5.3, 3.2,
                                     boxstyle="round,pad=0.15",
                                     facecolor='#161b22', edgecolor='#30363d', linewidth=1.0, zorder=8)
    ax.add_patch(box_eq)

    eq_lines = [
        ('核心公式', '#81D4FA', 1),
        ('t_d = 1.1 x R x C', '#FFCC80', 2),
        ('1.1 = -ln(1/3) ~ 1.0986', '#90A4AE', -1),
        ('设计约束:', '#EF5350', 1),
        ('  R in [1k, 10M] Ohm', '#90A4AE', -1),
        ('  C in [100p, 1000u] F', '#90A4AE', -1),
        ('  I_leak * R << Vcc', '#90A4AE', -1),
        ('示例: R=100k, C=10u -> 1.1s', '#A5D6A7', -1),
    ]
    y0 = 10.3
    for i, (text, color, weight) in enumerate(eq_lines):
        if not text:
            continue
        w = 'bold' if weight > 0 else 'normal'
        s = 9 if weight >= 2 else (8 if weight >= 1 else 7)
        ax.text(9.5, y0 - i * 0.30, text, fontsize=s, color=color, fontweight=w, family='monospace', zorder=9)

    # ===== 标题 =====
    ax.text(7.5, 11.0, 'NE555 单稳态延时电路  — 工程原理图',
            fontsize=14, fontweight='bold', color='#ECEFF1', ha='center')
    ax.text(7.5, 10.68, 'Monostable One-Shot Timer  |  DIP-8  |  $t_d = 1.1RC$',
            fontsize=8, color='#78909C', ha='center')

    fig.tight_layout(pad=1.0)
    return fig


# ============================================================
if __name__ == '__main__':
    fig = draw_full_schematic()
    fig.savefig('ne555_schematic.png', dpi=220, bbox_inches='tight',
                facecolor='#0d1117', edgecolor='none')
    print("原理图已保存: ne555_schematic.png")
    plt.show()
