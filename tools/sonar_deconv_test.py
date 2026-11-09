"""Offline test of the sharpened echogram (docs/SONAR_DECONV.md), 2026-11-08. EXPERIMENTAL, not in the firmware.
Steps per ping: D126e baseline = floor + ring-down decay fitted through the lowest points (was ALS),
subtract in linear amplitude (no margin), Richardson-Lucy (100 iterations) with the single-target hump measured
on the ball (07nov 14:28), re-blur ~5 cm, each lobe height-matched to the raw peak (narrower, same height, small bumps kept); area per lobe core. Same algorithm as sharpenEcho() in web/suite.html and the bench page. Window: blind zone .. floor + 0.35 m.
Usage: python tools/sonar_deconv_test.py log/<recording>.txt [out.png]   (needs numpy, scipy, matplotlib)"""
import sys, warnings
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
warnings.filterwarnings("ignore")
BIN = 0.025
def load(path):
    rows, px = [], []
    for l in open(path):
        if l.startswith("SONAR "):
            parts = l.split(); nfq = int(parts[2]); r = parts[8:8 + nfq]; row = r[1] if nfq == 3 else r[0]
            rows.append([-100 + int(row[i:i + 2], 16) * 95 / 255 for i in range(0, len(row), 2)])
        elif l.startswith("PINGX"):
            px.append(dict(x.split("=", 1) for x in l.split()[2:] if "=" in x))
    return np.array(rows), px
def als_var(y, lamv, p=0.02, it=10):
    n = len(y); D = sparse.diags([1., -2., 1.], [0, -1, -2], shape=(n, n - 2)); L = sparse.diags(np.asarray(lamv[:n - 2], float))
    H = D.dot(L).dot(D.T); w = np.ones(n)
    for _ in range(it):
        W = sparse.spdiags(w, 0, n, n); z = spsolve((W + H).tocsc(), w * y); w = p * (y > z) + (1 - p) * (y <= z)
    return z
lin = lambda db: 10 ** (db / 20)
dbf = lambda a: 20 * np.log10(np.maximum(a, 1e-9))
def rl(y, h, c, it=100):
    n = len(y); x = np.full(n, max(y.mean(), 1e-9)); hf = h[::-1]; L = len(h)
    conv = lambda v, k, ref: np.convolve(v, k)[ref:ref + n]
    for _ in range(it):
        x = x * conv(y / (conv(x, h, c) + 1e-12), hf, L - 1 - c)
    return x
HRAW = np.array([0, 0.58, 0.86, 1.0, 0.88, 0.56, 0.28, 0.10, 0.04]); C = 3   # 32 cycles, 2.5 cm bins, peak = ref
HN = HRAW / HRAW.sum()
G = np.exp(-0.5 * (np.arange(-3, 4) / 0.85) ** 2)
NTAU, TAU0, TAU1, BIT = 30, 0.02, 2.0, 8
RAW_DIP_DB = 3.0
DIP_DB = 6.0   # D126e: two peaks are drawn apart (each down to the baseline) only when the dip between them is >= 6 dB
               # in displayed level (dB of excess + baseline): a plateau or ripple a few dB deep stays one echo
def decay_base(seg, nf, p=0.02):
    """D126e baseline: power = F + A exp(-d / tau) (floor + ring-down tail), F, A >= 0, fitted through the lowest points:
    weighted least squares in relative power (~dB), points above the curve weigh p (like ALS), 8 passes, tau on a
    log grid 2 cm .. 2 m. One continuous, never-rising line (Frank's sketch); never under the noise floor."""
    n=len(seg); P=10**(seg/10); w=np.ones(n)
    for _ in range(BIT):
        best=None
        for t in range(NTAU):
            tau=TAU0*(TAU1/TAU0)**(t/(NTAU-1)); E=np.exp(-np.arange(n)*BIN/tau)
            sw=w/(P*P)   # weight of (model - P)^2: relative error
            s11=np.sum(sw); s12=np.sum(sw*E); s22=np.sum(sw*E*E); r1=np.sum(sw*P); r2=np.sum(sw*E*P)
            cands=[]
            det=s11*s22-s12*s12
            if det>0:
                F=(r1*s22-r2*s12)/det; A=(r2*s11-r1*s12)/det
                if F>=0 and A>=0: cands.append((F,A))
            if s11>0: cands.append((max(r1/s11,0.0),0.0))
            if s22>0: cands.append((0.0,max(r2/s22,0.0)))
            for F,A in cands:   # pick tau by a symmetric ratio error (q - 1)^2 / q, q = model / data (~ dB^2, no log)
                m=F+A*E; r=np.sum(w*(m-P)**2/np.maximum(m*P,1e-60))
                if best is None or r<best[0]: best=(r,m)
        m=best[1]; w=np.where(P>m,p,1.0)
    return np.maximum(10*np.log10(np.maximum(m,1e-30)),nf)
def sharpen(d, nf, floor_m, dead_m=0.30, buf_m=0.35, it=100, ring_m=0.15, dip_db=DIP_DB):
    a=int(round(dead_m/BIN)); b=min(len(d),int(round((floor_m+buf_m)/BIN)))
    seg=d[a:b]; n=len(seg); k=int(round(ring_m/BIN))
    B=decay_base(seg,nf)
    e=np.maximum(lin(seg)-lin(B),0)                 # no margin: small bumps stay
    x=rl(e+1e-12,HN,C,it)                            # flux = area
    s=np.convolve(x/HRAW.sum(),G,'same')
    # lobes between local minima of s; neighbours merged when the dip between them is < dip_db (no false split of a
    # plateau or a ripple), and everything from 6 cm above the bottom on is one group (the bottom echo and its tail)
    k=min(k,n); mins=[k]+[i for i in range(k+1,n-1) if s[i]<=s[i-1] and s[i]<=s[i+1]]+[n]   # lobes after the ring zone only
    kb=int(round((floor_m-0.06)/BIN))-a if floor_m>0 else n
    c=e.max()/max(s.max(),1e-30); lvl=lambda i: dbf(s[i]*c+lin(B[i]))   # displayed level (dB) of s, roughly height-matched
    groups=[]
    for i0,i1 in zip(mins[:-1],mins[1:]):
        if i1<=i0: continue
        if groups:
            g0,g1=groups[-1][:2]; pl=max(lvl(q) for q in range(g0,g1)); pr=max(lvl(q) for q in range(i0,i1))
            jl=g0+int(np.argmax(s[g0:g1])); jr=i0+int(np.argmax(s[i0:i1])); rdip=min(seg[jl],seg[jr])-seg[jl:jr+1].min()
            # split only when the deconvolved dip is >= dip_db AND the raw ping itself dips >= RAW_DIP_DB between the two
            # (Richardson-Lucy breaks a flat plateau - barrel wall, weeds, a school - into spikes: those stay one echo)
            jrb=i0+np.argmax(s[i0:i1])>=kb; jlb=g0+np.argmax(s[g0:g1])>=kb   # D136: the bottom zone never swallows an echo above it
            if ((not (jrb and not jlb)) and (min(pl,pr)-lvl(i0)<dip_db or rdip<RAW_DIP_DB)) or (jrb and jlb):
                groups[-1]=(g0,i1,groups[-1][2],i0+int(np.argmax(s[i0:i1]))); continue
        groups.append((i0,i1,i0+int(np.argmax(s[i0:i1])),i0+int(np.argmax(s[i0:i1]))))
    out=np.zeros(n); area=np.zeros(n)
    for i0,i1,jf,jl in groups:
        L=i1-i0; l0=s[i0] if i0>k else 0.0; l1=s[i1-1] if i1<n else 0.0   # window edges are not valleys
        ln=np.linspace(l0,l1,L) if L>1 else np.array([l0])
        v=np.maximum(s[i0:i1]-ln,0)                  # valley to valley: the group comes down to the baseline at its edges
        j=int(np.argmax(v)); pv=v[j]
        if pv<=0: continue
        jj=i0+j; raw=e[max(0,jj-1):min(n,jj+2)].max()
        out[i0:i1]=v*(raw/pv)
        # merged group (plateau / school / weeds): between its first and last peak it never dips more than 6 dB under
        # the raw ping (the Richardson-Lucy spikes of a flat echo are not drawn as separate fish)
        if i1==n: jl=n-1   # window end: not a valley either
        if jl>jf: out[jf:jl+1]=np.maximum(out[jf:jl+1],e[jf:jl+1]*0.5)
        fl=x[i0:i1].sum()*(v.sum()/max(s[i0:i1].sum(),1e-30))
        area[i0:i1]=np.where(v>=0.5*pv, fl, 0)
    out[:k]=e[:k]; area[:k]=0   # ring zone: left raw
    o=d.copy(); o[a:b]=np.maximum(dbf(out+lin(B)),B)
    ad=d.copy(); ad[a:b]=np.where(area>0, dbf(area/HRAW.sum()+lin(B)), np.maximum(dbf(out+lin(B)),B))   # area-equivalent dB (point target: = its peak)
    return o,ad,a,b,B

if __name__ == "__main__":
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    R, px = load(sys.argv[1]); S = np.array([sharpen(d, float(px[k]["nf"]), float(px[k]["bottom"]))[0] for k, d in enumerate(R)])
    z0, z1 = int(0.3 / BIN), int(min(len(R[0]) * BIN, np.median([float(p["bottom"]) for p in px]) + 0.5) / BIN)
    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    for i, (M, t) in enumerate(((R, "raw"), (S, "sharpened (experimental)"))):
        ax[i].imshow(M[:, z0:z1].T, aspect="auto", cmap="inferno", vmin=-80, vmax=-30, extent=[0, len(M), z1 * BIN, z0 * BIN]); ax[i].set_title(t); ax[i].set_ylabel("m")
    plt.tight_layout(); plt.savefig(sys.argv[2] if len(sys.argv) > 2 else "deconv.png", dpi=90); print("ok")
