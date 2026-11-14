import numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
rng=np.random.default_rng(7)
BIN=0.025; N=240; T=240           # 6 m, 60 s at 4 lines/s
HUMP={'old32':[0,.58,.86,1,.88,.56,.28,.10,.04],'s8':[0,.40,.95,1,.60,.22,.03,0,0],'d16':[0,.10,.63,1,.88,.44,.11,.02,0]}
def hump(h,x):   # x = bins from peak ref (index 3)
    i=x+3; H=HUMP[h]; k=np.floor(i).astype(int); f=i-k
    ok=(k>=0)&(k<8); out=np.zeros_like(i,float); kk=np.clip(k,0,7)
    out[ok]=(np.array(H)[kk]*(1-f)+np.array(H)[kk+1]*f)[ok]; return out
lin=lambda db:10**(db/20)
# measured anchors (14 Nov, 30 V, gain 0): ball -18 dB at 0.48 m (8 cycles); weight ~ -37 dB at 0.53 m (range -32..-42); floor -79 dB
BALL0,R0B=-18.0,0.48; WGT0,R0W=-37.0,0.53
def lvl(db0,r0,r): return db0-40*np.log10(r/r0)
CFG={ # name: (hump, target offset dB, floor dB, colour range dB)
 'before':('old32',-31.0,-77.0,46),   # burst-count fault: about -34 dB on echoes, 32 cycles (+3), floor as on the morning recordings (gain 3)
 'sharp': ('s8',   0.0,-79.0,60),
 'deep':  ('d16',  2.6,-79.0,60)}
BOT=5.0
def scene(t):
    s=t/4.0; tg=[]
    jig=4.00+0.04*np.sin(2*np.pi*s/3.0)*(1 if (s%20)<12 else 0.2)
    tg.append((jig,'w',0))
    # fish A: ball-size, rises from 4.75 to 0.12 m under the jig, holds, leaves
    if 15<=s<50:
        if s<25: d=4.75-(4.75-(jig+0.12))*(s-15)/10
        elif s<40: d=jig+0.12+0.01*np.sin(s*2)
        else: d=jig+0.12+(4.8-jig-0.12)*(s-40)/10
        tg.append((d,'b',0))
    if 5<=s<30: tg.append((2.6-0.5*np.sin((s-5)/25*np.pi),'b',-8))   # smaller fish passing mid-water
    return tg,jig
def run(name):
    h,off,nf,_=CFG[name]; E=np.zeros((N,T)); x=np.arange(N)
    for t in range(T):
        tg,_=scene(t); acc=np.zeros(N)
        for b in range(4):   # avg 4 bursts
            a=np.zeros(N,complex)
            for d,k,o in tg:
                L=(lvl(BALL0,R0B,d) if k=='b' else lvl(WGT0,R0W,d))+o+off+rng.normal(0,1.0)
                a+=lin(L)*hump(h,x-d/BIN)*np.exp(1j*rng.uniform(0,6.28))
            # bottom: extended target, 20 log R, strong, with a tail
            Lb=-22-20*np.log10(BOT/0.7)+off
            xb=x-BOT/BIN; a+=lin(Lb)*(hump(h,xb)+np.where(xb>0,np.exp(-xb/4.0),0))*np.exp(1j*rng.uniform(0,6.28,N))
            # ring-down near the transducer
            a+=lin(-8+off*0.3)*np.exp(-x*BIN/0.045)
            a+=lin(nf)*(rng.normal(size=N)+1j*rng.normal(size=N))/np.sqrt(2)
            acc+=np.abs(a)**2
        E[:,t]=10*np.log10(acc/4)
    return E
PAL=[(0,(10,30,38)),(.16,(17,65,76)),(.32,(27,119,120)),(.48,(88,174,124)),(.62,(211,192,74)),(.76,(238,139,47)),(.9,(216,67,47)),(1,(255,230,210))]
cm=LinearSegmentedColormap.from_list('sonar',[(p,tuple(c/255 for c in rgb)) for p,rgb in PAL])
R={k:run(k) for k in CFG}
TITLE={'before':'Before today\n32-cycle ping, burst fault, contrast 46 dB','sharp':'Now, Sharp (default)\n8-cycle ping, contrast 60 dB','deep':'Now, Deep\n16-cycle ping, contrast 60 dB'}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig=plt.figure(figsize=(13.5,12.4),dpi=150,facecolor='white')
gs=fig.add_gridspec(3,3,height_ratios=[1.25,1.0,0.9],hspace=0.42,wspace=0.16,left=0.06,right=0.985,top=0.865,bottom=0.055)
ft=lambda m:m*3.28084
for c,k in enumerate(CFG):
    h,off,nf,rg=CFG[k]; E=R[k]
    ax=fig.add_subplot(gs[0,c]); im=ax.imshow(E,aspect='auto',cmap=cm,vmin=nf+4,vmax=nf+4+rg-4,extent=[-60,0,N*BIN,0],interpolation='nearest')
    ax.set_title(TITLE[k],fontsize=11,loc='left',color='#1a1a1a'); ax.set_ylim(5.6,0); ax.set_xlabel('seconds ago',color='#555')
    if c==0: ax.set_ylabel('depth (m)',color='#555')
    ax.tick_params(colors='#555')
    if c==1:
        ax.annotate('small fish',xy=(-42,2.15),xytext=(-30,1.3),color='white',fontsize=9,arrowprops=dict(arrowstyle='-',color='white',lw=.8))
        ax.annotate('jig',xy=(-57,4.0),xytext=(-57,3.3),color='white',fontsize=9,arrowprops=dict(arrowstyle='-',color='white',lw=.8))
        ax.annotate('fish at the jig',xy=(-28,4.14),xytext=(-22,3.2),color='white',fontsize=9,arrowprops=dict(arrowstyle='-',color='white',lw=.8))
        ax.annotate('bottom, 5.0 m (16.4 ft)',xy=(-12,4.97),xytext=(-24,4.55),color='white',fontsize=9,arrowprops=dict(arrowstyle='-',color='white',lw=.8))
    az=fig.add_subplot(gs[1,c]); az.imshow(E,aspect='auto',cmap=cm,vmin=nf+4,vmax=nf+rg,extent=[-60,0,N*BIN,0],interpolation='nearest')
    az.set_ylim(4.55,3.65); az.set_xlim(-48,-8); az.set_xlabel('seconds ago',color='#555'); az.tick_params(colors='#555')
    az.set_title('Zoom on the jig, 3.65–4.55 m',fontsize=10,loc='left',color='#1a1a1a')
    if c==0: az.set_ylabel('depth (m)',color='#555')
    ap=fig.add_subplot(gs[2,c]); t0=T-28*4; seg=E[:,t0-8:t0+8]; d=np.arange(N)*BIN
    ap.fill_between(d,seg.min(1)-nf,seg.max(1)-nf,color='#1b7778',alpha=.25,lw=0); ap.plot(d,np.median(seg,1)-nf,color='#1b7778',lw=2)
    ap.axhline(10,color='#888',lw=1,ls=(0,(4,3))); ap.text(4.74,11.2,'detection threshold',color='#555',fontsize=8.5,ha='right')
    tg,jig=scene(t0); ap.set_xlim(3.65,4.75); ap.set_ylim(-4,34); ap.grid(axis='y',color='#e6e6e6',lw=.8); ap.set_axisbelow(True)
    ap.set_xlabel('depth (m), fish 12 cm under the jig',color='#555'); ap.tick_params(colors='#555')
    ap.set_title('One moment: echo above the noise floor (dB)',fontsize=10,loc='left',color='#1a1a1a')
    ap.annotate('jig',xy=(jig,-3.2),xytext=(jig,-3.2),ha='center',fontsize=8.5,color='#333'); ap.annotate('fish',xy=(jig+.12,-3.2),ha='center',fontsize=8.5,color='#333')
    ap.axvline(jig,color='#bbb',lw=.8,ymax=.92); ap.axvline(jig+.12,color='#bbb',lw=.8,ymax=.92)
    if c==0: ap.set_ylabel('dB over floor',color='#555')
    # numbers
    pk=lambda dd:float(np.median(seg,1)[int(round(dd/BIN))-1:int(round(dd/BIN))+3].max()-nf)
    print(k,'jig %.1f dB, fish %.1f dB over floor'%(pk(jig),pk(jig+.12)))
fig.suptitle('Simulated 5 m hole: jig head with a fish rising to it, before today and with the new settings',x=0.06,y=0.985,ha='left',fontsize=14,fontweight='bold',color='#111')
fig.text(0.06,0.925,'Simulation scaled from the 14 Nov bin measurements (ball, quarter-ounce weight, noise floor).\nOpen water, targets on the beam axis. Not a lake recording.',fontsize=9.5,color='#555',linespacing=1.5)
cax=fig.add_axes([0.765,0.94,0.18,0.011]); cb=fig.colorbar(plt.cm.ScalarMappable(cmap=cm),cax=cax,orientation='horizontal'); cb.set_ticks([0,1]); cb.set_ticklabels(['noise floor','top of scale']); cb.ax.tick_params(labelsize=8.5,colors='#555',length=0); cb.outline.set_visible(False)
import sys; fig.savefig(sys.argv[1])
