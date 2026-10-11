"""
NE555 单稳态延时电路 —— 严实点论 Python 仿真
==============================================
覆盖:
  1. RC 充放电核心方程
  2. 完整时序波形 (TRIG / Vcap / OUT)
  3. 组件容差 Monte Carlo 分析
  4. 电源纹波对定时精度的影响
  5. 电解电容漏电流灾难场景
  6. (R,C) 设计参数安全区间图
  7. 3D 延时曲面 + 灵敏度等高线

依赖: pip install numpy matplotlib
"""

import numpy as np
from numpy import log, exp, pi, sqrt
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

# ---- 中文字体适配 ----
_zh_font = None
for _f in fm.fontManager.ttflist:
    if any(k in _f.name for k in ['SimHei', 'Microsoft YaHei', 'WenQuanYi', 'Noto Sans CJK']):
        _zh_font = _f.name
        break
if _zh_font:
    plt.rcParams["font.family"] = _zh_font
    print(f"[字体] 使用: {_zh_font}")

# ============================================================
# 0. 全局美学
# ============================================================
plt.rcParams.update({
    "figure.dpi": 150, "savefig.dpi": 200,
    "font.size": 9, "axes.titlesize": 11, "axes.labelsize": 10,
    "xtick.labelsize": 8, "ytick.labelsize": 8,
    "legend.fontsize": 7, "figure.titlesize": 13,
    "axes.grid": True, "grid.alpha": 0.35,
    "lines.linewidth": 1.4,
})

# ============================================================
# 1. NE555 物理模型
# ============================================================
class NE555:
    """NE555 双极型单稳态完整时域模型"""

    def __init__(self, Vcc=5.0, R=100e3, C=10e-6,
                 R_disch=12.0, I_leak=250e-9, V_sat=1.2, I_q=5e-3):
        self.Vcc     = Vcc
        self.R       = R
        self.C       = C
        self.R_disch = R_disch
        self.I_leak  = I_leak
        self.V_sat   = V_sat
        self.I_q     = I_q

    def t_delay_ideal(self):
        """理想延时: td = -ln(1/3) * R * C"""
        return -log(1.0 / 3.0) * self.R * self.C

    def V_cap_ideal(self, t):
        return self.Vcc * (1.0 - exp(-t / (self.R * self.C)))

    def V_cap_real(self, t):
        """含漏电流的电容电压"""
        Veff = max(self.Vcc - self.I_leak * self.R, 0.0)
        return Veff * (1.0 - exp(-t / (self.R * self.C)))

    def V_cap_with_leak(self, t, I_extra=0.0):
        """含额外漏电流 (模拟老化/受潮)"""
        Veff = max(self.Vcc - (self.I_leak + I_extra) * self.R, 0.0)
        return Veff * (1.0 - exp(-t / (self.R * self.C)))

    def find_threshold_crossing(self, dt=1e-6, extra_leak=0.0):
        """查找电容电压穿越 2/3 Vcc 的时刻"""
        V_th = (2.0 / 3.0) * self.Vcc
        t_ideal = self.t_delay_ideal()
        t_arr = np.linspace(t_ideal * 0.6, t_ideal * 2.0, 3000)
        v_arr = self.V_cap_with_leak(t_arr, extra_leak)
        for i in range(len(t_arr)):
            if v_arr[i] >= V_th:
                lo, hi = t_arr[max(0, i-1)], t_arr[i]
                for _ in range(30):
                    mid = (lo + hi) / 2.0
                    if self.V_cap_with_leak(mid, extra_leak) >= V_th:
                        hi = mid
                    else:
                        lo = mid
                return (lo + hi) / 2.0
        return None

# ============================================================
# 2. 核心时序图
# ============================================================
def plot_core_timing(chip, t_end=None):
    if t_end is None:
        t_end = chip.t_delay_ideal() * 2.5

    t = np.linspace(0, t_end, 3000)
    td_ideal = chip.t_delay_ideal()
    Vth = (2.0 / 3.0) * chip.Vcc
    trig_pos = t_end * 0.12
    trig_width = 30e-6

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    fig.subplots_adjust(hspace=0.2)

    # TRIG
    w = np.zeros_like(t)
    w[(t >= trig_pos) & (t < trig_pos + trig_width)] = chip.Vcc
    ax1.plot(t * 1e3, w, '#2196F3', linewidth=1.8, label='TRIG (2脚)')
    ax1.axvline(trig_pos * 1e3, color='#FF5722', ls='--', alpha=0.7, label='触发时刻')
    ax1.set(ylabel='V', title='TRIG 触发引脚', ylim=(-0.3, chip.Vcc * 1.2))
    ax1.legend()

    # Vcap
    t_c = np.linspace(0, t_end - trig_pos - trig_width, 3000)
    t_abs = trig_pos + trig_width + t_c
    v_id = chip.V_cap_ideal(t_c)
    v_re = chip.V_cap_real(t_c)
    ax3.plot(t_abs * 1e3, v_id, '#4CAF50', linewidth=1.8, label='Vcap 理想')
    ax3.plot(t_abs * 1e3, v_re, '#FF9800', linewidth=1.8, ls='--', label='Vcap 含漏电流')
    ax3.axhline(Vth, color='red', ls=':', linewidth=1.2, label=f'2/3·Vcc = {Vth:.2f}V')
    ax3.axvline((trig_pos + trig_width + td_ideal) * 1e3, color='#4CAF50',
                ls='-.', alpha=0.7)
    ax3.fill_betweenx([0, chip.Vcc],
                      (trig_pos + trig_width) * 1e3,
                      (trig_pos + trig_width + td_ideal) * 1e3,
                      color='#4CAF50', alpha=0.08)
    ax3.annotate(f'$t_d$ = {td_ideal*1e3:.2f} ms',
                 xy=((trig_pos + trig_width + td_ideal * 0.45) * 1e3, Vth * 0.55),
                 fontsize=9, color='#2E7D32',
                 bbox=dict(boxstyle='round', fc='white', alpha=0.9, ec='#4CAF50'))
    ax3.set(ylabel='V', title='THRES / Vcap')
    ax3.legend()

    # OUT
    out = np.zeros_like(t)
    mask = (t >= trig_pos + trig_width) & (t < trig_pos + trig_width + td_ideal)
    out[mask] = chip.Vcc - chip.V_sat
    ax2.plot(t * 1e3, out, '#E91E63', linewidth=1.8, label='OUT (3脚)')
    ax2.set(ylabel='V', title=f'OUT 输出 (驱动 ±200mA) | 静态功耗={chip.I_q*1e3:.1f}mA',
            ylim=(-0.3, chip.Vcc * 1.2))
    ax2.legend()

    ax3.set_xlabel('时间 (ms)')
    info = (f'R={chip.R/1e3:.0f}kΩ  C={chip.C*1e6:.2f}μF  '
            f'$t_d$={td_ideal*1e3:.2f}ms')
    fig.suptitle(f'NE555 单稳态核心时序  {info}', fontweight='bold')
    return fig

# ============================================================
# 3. Monte Carlo 容差分析
# ============================================================
def plot_monte_carlo(Vcc=5.0, R_nom=100e3, C_nom=10e-6, N=5000):
    np.random.seed(42)
    R_s = np.random.normal(R_nom, R_nom * 0.05 / 3.0, N)
    C_s = np.random.normal(C_nom, C_nom * 0.10 / 3.0, N)

    td_s = -log(1.0 / 3.0) * R_s * C_s
    td_nom = -log(1.0 / 3.0) * R_nom * C_nom

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.hist(td_s * 1e3, bins=100, color='steelblue', edgecolor='white', alpha=0.85)
    ax.axvline(td_nom * 1e3, color='red', ls='--', linewidth=1.5,
               label=f'名义值 {td_nom*1e3:.3f} ms')

    sigma = np.std(td_s)
    pct = np.percentile(td_s, [1, 99])
    ax.axvline(td_nom * 1e3 + 3 * sigma * 1e3, color='darkorange', ls='--',
               linewidth=1.2, label=f'+3σ')
    ax.axvline(td_nom * 1e3 - 3 * sigma * 1e3, color='darkorange', ls='--',
               linewidth=1.2, label=f'-3σ')

    info = (f'mean = {np.mean(td_s)*1e3:.3f} ms\n'
            f'std  = {sigma*1e3:.4f} ms\n'
            f'±3σ  = ±{3*sigma/td_nom*100:.1f}%\n'
            f'范围 [{pct[0]*1e3:.3f}, {pct[1]*1e3:.3f}] ms')
    ax.text(0.98, 0.98, info, transform=ax.transAxes, fontsize=9,
            ha='right', va='top', family='monospace',
            bbox=dict(boxstyle='round', fc='white', ec='gray', alpha=0.9))
    ax.set(title='Monte Carlo 容差分析 (R±5%, C±10%)', xlabel='$t_d$ (ms)', ylabel='频次')
    ax.legend()
    fig.tight_layout()
    return fig

# ============================================================
# 4. 供电纹波对定时精度的影响
# ============================================================
def plot_ripple_effect(Vcc=5.0, R=100e3, C=10e-6):
    chip = NE555(Vcc=Vcc, R=R, C=C)
    td_nom = chip.t_delay_ideal()
    t = np.linspace(0, td_nom * 1.3, 5000)
    Vth_nom = (2.0 / 3.0) * Vcc
    v_norip = chip.V_cap_ideal(t)

    fig, ax = plt.subplots(figsize=(9, 4.5))

    # 不同纹波幅度
    for amp, col, lab in [(0, 'black', '无纹波'),
                           (50e-3, '#2196F3', '50mVpp'),
                           (150e-3, '#FF9800', '150mVpp'),
                           (300e-3, '#F44336', '300mVpp')]:
        if amp == 0:
            ax.plot(t * 1e3, v_norip, color=col, linewidth=1.8, label=lab)
        else:
            # 纹波调制等效阈值 → 电容电压不变，阈值在抖动
            # 找最坏情况: 阈值 ±amp
            cross_times = []
            for phase_shift in np.linspace(0, 2*pi, 20):
                Vth_r = Vth_nom + amp * np.sin(phase_shift)
                idx = np.argmax(v_norip >= Vth_r)
                if idx > 0:
                    cross_times.append(t[idx])
            if cross_times:
                ax.axvspan(min(cross_times)*1e3, max(cross_times)*1e3,
                           color=col, alpha=0.12)
                jitter = (max(cross_times) - min(cross_times)) / td_nom * 100
                ax.annotate(f'{lab}\n抖动±{jitter:.1f}%',
                            xy=(td_nom * 1e3, Vth_nom * (0.7 - amp/0.5)),
                            fontsize=8, color=col, fontweight='bold')

    ax.plot(t * 1e3, v_norip, 'black', linewidth=1.8, label='Vcap')
    ax.axhline(Vth_nom, color='gray', ls='--', alpha=0.5, label=f'2/3 Vcc = {Vth_nom:.2f}V')
    ax.set(title='供电纹波 → 阈值参考电平抖动 → 定时漂移',
           xlabel='时间 (ms)', ylabel='V')
    ax.legend(fontsize=8)
    fig.tight_layout()
    return fig

# ============================================================
# 5. 电解电容漏电流灾难场景
# ============================================================
def plot_leakage_disaster(Vcc=5.0, R=1e6, C=100e-6):
    chip = NE555(Vcc=Vcc, R=R, C=C)
    td_ideal = chip.t_delay_ideal()
    t = np.linspace(0, td_ideal * 2.5, 4000)
    Vth = (2.0 / 3.0) * Vcc

    fig, ax = plt.subplots(figsize=(9, 5))

    leaks = [0, 0.5e-6, 2e-6, 5e-6, 10e-6, 20e-6]
    labels = ['理想 (0)', '0.5 μA (新)', '2 μA', '5 μA (老化)', '10 μA (受潮)', '20 μA (损坏)']
    colors = plt.cm.plasma(np.linspace(0.1, 1.0, len(leaks)))

    for lk, lab, col in zip(leaks, labels, colors):
        v = chip.V_cap_with_leak(t, lk)
        Vfinal = max(Vcc - lk * R, 0)
        ax.plot(t * 1e3, v, color=col, linewidth=1.5, label=f'{lab}  |  Vmax={Vfinal:.2f}V')

        if Vfinal < Vth:
            ax.annotate(f'⚠ 充不到阈值!', xy=(td_ideal * 2.1 * 1e3, Vfinal + 0.04),
                        fontsize=8, color=col, fontweight='bold',
                        bbox=dict(boxstyle='round', fc='#FFCDD2', alpha=0.8))

    ax.axhline(Vth, color='red', ls='--', linewidth=1.5, label=f'阈值 {Vth:.2f}V')
    ax.set(title=f'[!] 电解电容漏电流  --  R=1M, C=100uF  |  $t_d$(理想)={td_ideal:.2f}s',
           xlabel='时间 (ms)', ylabel='Vcap (V)')
    ax.legend(fontsize=7, ncol=2, loc='lower right')
    fig.tight_layout()
    return fig

# ============================================================
# 6. (R,C) 设计参数安全区间图
# ============================================================
def plot_design_space():
    fig, ax = plt.subplots(figsize=(10, 7))

    R = np.logspace(2, 7.5, 200)
    C = np.logspace(-10, -2, 200)
    RR, CC = np.meshgrid(R, C)
    TD = -log(1.0 / 3.0) * RR * CC

    # 填充
    cs = ax.contourf(RR, CC, TD, levels=10, cmap='viridis_r',
                     alpha=0.75, norm=plt.matplotlib.colors.LogNorm(vmin=1e-9, vmax=1e4))

    # 等高线
    levels = [1e-6, 1e-3, 0.1, 10, 1000]
    cl = ax.contour(RR, CC, TD, levels=levels, colors='white', linewidths=0.8, alpha=0.7)
    ax.clabel(cl, inline=True, fontsize=7, fmt=lambda v: f'{v:.0e}s' if v < 1 else f'{v:.1f}s')

    # 禁止区
    ax.fill_between([100, 1e3], 1e-10, 1e-2, color='red', alpha=0.12)
    ax.text(150, 5e-8, '禁\n止\n区', fontsize=7, color='darkred', ha='center', fontweight='bold')
    ax.annotate('R<1kΩ → 放电管过流', xy=(300, 3e-8), fontsize=7, color='darkred',
                rotation=90, va='center')
    ax.fill_between([10e6, 5e7], 1e-10, 1e-2, color='red', alpha=0.12)
    ax.annotate('R>10MΩ → 漏电主导', xy=(15e6, 2e-8), fontsize=7, color='darkred')
    ax.fill_between([100, 5e7], 1e-10, 500e-12, color='orange', alpha=0.1)
    ax.fill_between([100, 5e7], 1000e-6, 1e-2, color='orange', alpha=0.1)
    ax.fill_between([1e6, 5e7], 100e-6, 1e-2, color='red', alpha=0.1, hatch='//')

    # 推荐框
    ax.add_patch(Rectangle((1e3, 1e-9), 1e6 - 1e3, 10e-6 - 1e-9,
                           linewidth=2, edgecolor='limegreen', facecolor='none'))
    ax.annotate('推荐工作区', xy=(5e4, 5e-8), fontsize=9, color='limegreen',
                fontweight='bold',
                bbox=dict(boxstyle='round', fc='white', alpha=0.9, ec='limegreen'))

    ax.set(xscale='log', yscale='log', xlabel='R (Ω)', ylabel='C (F)',
           title='NE555 单稳态设计空间 — R-C 参数选择图')
    plt.colorbar(cs, ax=ax, label='$t_d$ (s)')
    fig.tight_layout()
    return fig

# ============================================================
# 7. 3D 延时曲面 + 灵敏度等高线
# ============================================================
def plot_3d_surface():
    fig = plt.figure(figsize=(13, 5.5))

    # 3D
    ax3d = fig.add_subplot(1, 2, 1, projection='3d')
    R = np.logspace(2.5, 6.5, 50)
    C = np.logspace(-9, -4, 50)
    RR, CC = np.meshgrid(R, C)
    TD = -log(1.0 / 3.0) * RR * CC
    ax3d.plot_surface(np.log10(RR), np.log10(CC), TD * 1e3,
                      cmap='Spectral_r', edgecolor='none', alpha=0.9, antialiased=True)
    ax3d.set(xlabel='log10(R/Ω)', ylabel='log10(C/F)', zlabel='$t_d$ (ms)',
             title='$t_d(R,C)$ 曲面')
    ax3d.view_init(elev=28, azim=-55)

    # 灵敏度
    ax2d = fig.add_subplot(1, 2, 2)
    dr = np.linspace(-0.3, 0.3, 100)
    dc = np.linspace(-0.3, 0.3, 100)
    Dr, Dc = np.meshgrid(dr, dc)
    T_rel = (1 + Dr) * (1 + Dc)
    cs2 = ax2d.contourf(Dr * 100, Dc * 100, T_rel,
                        cmap='RdYlGn_r', alpha=0.85, levels=10)
    ax2d.contour(Dr * 100, Dc * 100, T_rel,
                 colors='black', linewidths=0.6, alpha=0.5, levels=10)
    ax2d.axvline(0, color='gray', ls='--', alpha=0.4)
    ax2d.axhline(0, color='gray', ls='--', alpha=0.4)
    ax2d.set(xlabel='ΔR (%)', ylabel='ΔC (%)',
             title='$t_d$ 相对误差灵敏度  |  $t_d ∝ R×C$')
    plt.colorbar(cs2, ax=ax2d, label='$t_d / t_{d0}$')

    fig.suptitle('NE555 延时灵敏度  —  $t_d = 1.1RC$ 线性乘积敏感', fontweight='bold')
    fig.tight_layout()
    return fig

# ============================================================
# 8. 主程序
# ============================================================
if __name__ == '__main__':
    chip = NE555(Vcc=5.0, R=100e3, C=10e-6)
    print(f"设计参数: R={chip.R/1e3:.0f}kΩ  C={chip.C*1e6:.1f}μF")
    print(f"理想延时 td = -ln(1/3)×R×C = {chip.t_delay_ideal()*1e3:.2f} ms\n")

    print("[1/6] 核心时序波形...")
    fig1 = plot_core_timing(chip)

    print("[2/6] Monte Carlo 容差...")
    fig2 = plot_monte_carlo()

    print("[3/6] 供电纹波影响...")
    fig3 = plot_ripple_effect()

    print("[4/6] 电解电容漏电流灾难...")
    fig4 = plot_leakage_disaster()

    print("[5/6] 设计参数安全区间...")
    fig5 = plot_design_space()

    print("[6/6] 3D 灵敏度曲面...")
    fig6 = plot_3d_surface()

    print("\n全部完成!")
    plt.show()
