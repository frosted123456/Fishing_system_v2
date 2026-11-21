"""D207 / D208: XL6009 restart after an idle rest, simulated (not measured). python tools/sim_boost_restart.py
Part 1: plain restart against the power-on that happens today. Part 2: enable pin pulsed (software soft start)."""
# XL6009 restart after an idle rest: peak-current-mode boost, no soft start, from a 3 x L91 pack.
# Module: L 47 uH, Cin 220 uF, Cout 100 uF, fsw 400 kHz, switch limit 4 A, diode 0.4 V, divider 7.9 k.
import math
def run(voc, rpack, v0, ilim=4.0, esp_w=0.45, label=""):
    L=47e-6; Cin=220e-6; Cout=100e-6; fsw=400e3; T=1/fsw; Rl=0.15; Rsw=0.1; Vd=0.45; Rdiv=7900.0; Dmax=0.9
    vin=voc - (esp_w/voc)*rpack; vout=v0; il=0.0; t=0; dt=T/50
    vmin=vin; t_reg=None; imax_pack=0; q=0
    target=31.5
    while t<0.08:
        # one switching cycle, switch on until ilim or Dmax, unless vout >= target (skip)
        on = vout < target
        swon=on
        for k in range(50):
            if swon and (k*dt >= Dmax*T or il >= ilim): swon=False
            sw = swon
            if sw:
                dil=(vin - il*(Rl+Rsw))/L*dt
                iout=0
            else:
                if il>0:
                    dil=(vin - il*Rl - Vd - vout)/L*dt
                    if vin - Vd > vout: pass
                    iout=il
                else: dil=0; iout=0
            il=max(0.0, il+dil)
            # pass-through when vout < vin - Vd handled by the same equation (il rises through the diode)
            iesp=esp_w/max(vin,1.5)/0.9
            ipack=(voc-vin)/rpack
            vin += (ipack - il - iesp)/Cin*dt
            vout += (iout - vout/Rdiv - 0.0005)/Cout*dt
            imax_pack=max(imax_pack,ipack); vmin=min(vmin,vin); q+=ipack*dt
            t+=dt
        if t_reg is None and vout>=target: t_reg=t
    print(f"{label:34s} pack {voc:.1f} V / {rpack:.2f} ohm, rail from {v0:4.1f} V: back at 31.5 V after {t_reg*1e3 if t_reg else float('nan'):5.1f} ms, pack-side minimum {vmin:.2f} V, peak pack current {imax_pack:.2f} A")
for lab,voc,r in (("fresh, 20 C",4.8,0.40),("part used, 20 C",4.4,0.55),("cold -20 C, part used",4.3,0.90),("cold, near empty",3.9,1.20)):
    run(voc,r,12.0,label=lab+" (1 s rest)")
    run(voc,r,voc-0.5,label=lab+" (power-on, today)")
# rail after a rest of t seconds: tau = 7.9k * 100 uF
for t in (0.75,0.88,1.0,1.5): print(f"rest {t:.2f} s -> rail {4.0+(31.5-4.0)*math.exp(-t/0.79):.1f} V")

# same converter, EN pulsed (software soft start): on_us of every per_us, for n pulses, then EN high
def run2(voc, rpack, v0, on_us, per_us, n, label, ilim=4.0, esp_w=0.45):
    L=47e-6; Cin=220e-6; Cout=100e-6; T=2.5e-6; Rl=0.15; Rsw=0.1; Vd=0.45; Rdiv=7900.0; Dmax=0.9
    vin=voc-(esp_w/voc)*rpack; vout=v0; il=0.0; t=0.0; dt=T/25; vmin=vin; t_reg=None; v_after=None
    tp=n*per_us*1e-6
    while t<0.15:
        en = (t>=tp) or ((t*1e6)%per_us < on_us)
        on = en and vout<31.5
        swon=on
        for k in range(25):
            if swon and (k*dt>=Dmax*T or il>=ilim): swon=False
            if swon: dil=(vin-il*(Rl+Rsw))/L*dt; iout=0
            elif il>0: dil=(vin-il*Rl-Vd-vout)/L*dt; iout=il
            else: dil=0; iout=0
            il=max(0.0,il+dil)
            iesp=esp_w/max(vin,1.5)/0.9; ipack=(voc-vin)/rpack
            vin+=(ipack-il-iesp)/Cin*dt; vout+=(iout-vout/Rdiv-0.0005)/Cout*dt
            vmin=min(vmin,vin); t+=dt
        if v_after is None and t>=tp: v_after=vout
        if t_reg is None and vout>=31.5: t_reg=t
    print(f"{label:24s} {on_us:3d}/{per_us} us x{n:3d}: rail after the pulses {v_after:5.1f} V, at 31.5 V after {t_reg*1e3:5.1f} ms, pack-side minimum {vmin:.2f} V")
for lab,voc,r in (("fresh 20 C",4.8,0.40),("cold, part used",4.3,0.90),("cold, near empty",3.9,1.20)):
    run2(voc,r,12.0,0,500,0,lab+" plain")
    for on,per,n in ((40,500,120),(60,500,120),(100,500,120)):
        run2(voc,r,12.0,on,per,n,lab)
