#!/usr/bin/env python3
"""Summarise a log saved by the page's Record button:  tools/analyze_log.py fusion-log-....txt

Reports radar targets against time, whether targets in one frame share a
speed (the signature of stationary scenery seen from a moving rig), range and
signal by distance band. Drops records made by the pre-15-Nov framing bug.
"""
import sys, collections, statistics as st
frames=[]   # (clock_ms, [targets]) with bogus records (trailer/header bytes read as a target) dropped
bogus=0; c0=None; cN=None; lid=[]
for line in open(sys.argv[1], errors="replace"):
    if line.startswith("#"): continue
    f=line.strip().split(",")
    if len(f)<9 or f[1]!="D": continue
    try: v=[int(x) for x in f[2:]]
    except ValueError: continue
    c=int(f[0]); c0=c0 or c; cN=c
    lid.append(v[1])
    if v[6]<0: continue
    r=v[7:]; tg=[]
    for i in range(0,len(r)-len(r)%5,5):
        a,d,di,k,s=r[i:i+5]
        if di not in (0,1) or (a,d)==(120,247) or (a,d)==(115,242): bogus+=1; break
        tg.append((a,d,di,k,s))
    frames.append((c,tg))
print("radar frames %d, with >=1 valid target %d, bogus records dropped %d"%(len(frames),sum(1 for f in frames if f[1]),bogus))
T=[(c,)+t for c,tg in frames for t in tg]
app=[t for t in T if t[3]==1]; rec=[t for t in T if t[3]==0]
print("valid target reports %d: approaching %d, receding %d"%(len(T),len(app),len(rec)))
print("targets per frame:",sorted(collections.Counter(len(tg) for c,tg in frames).items()))
# same-speed test: frames with >=2 approaching targets
multi=[[t for t in tg if t[2]==1] for c,tg in frames]; multi=[m for m in multi if len(m)>=2]
spread=[max(t[3] for t in m)-min(t[3] for t in m) for m in multi]
print("frames with >=2 approaching targets: %d; speed spread within frame: 0 km/h in %d, <=1 in %d, >3 in %d"%(len(multi),sum(1 for s in spread if s==0),sum(1 for s in spread if s<=1),sum(1 for s in spread if s>3)))
dsp=[max(t[1] for t in m)-min(t[1] for t in m) for m in multi]
print("  range spread within those frames: median %d m, max %d m"%(st.median(dsp),max(dsp)))
print("approaching distance: median %d m, 90th pct %d m, max %d m"%(st.median(t[2] for t in app), sorted(t[2] for t in app)[int(len(app)*0.9)], max(t[2] for t in app)))
print("approaching by distance band: n, median signal")
for lo,hi in ((0,10),(10,25),(25,50),(50,75),(75,100)):
    b=[t for t in app if lo<=t[2]<hi]
    if b: print("  %2d-%3d m: %4d reports, signal median %3d, speed median %d km/h"%(lo,hi,len(b),st.median(t[5] for t in b),st.median(t[4] for t in b)))
print("angles of approaching targets:",sorted(collections.Counter((t[1]//10)*10 for t in app).items()))
print("angle exactly 0: %d of %d"%(sum(1 for t in app if t[1]==0),len(app)))
print("\nspeed trace, 5 s bins: t, approaching reports, median km/h (mph), farthest m | receding n, median km/h, median dist")
bins=collections.defaultdict(lambda:([],[],[],[]))
for t in app: b=bins[(t[0]-c0)//5000]; b[0].append(t[4]); b[1].append(t[2])
for t in rec: b=bins[(t[0]-c0)//5000]; b[2].append(t[4]); b[3].append(t[2])
for k in range(0,(cN-c0)//5000+1):
    a,d,r,rd=bins[k]
    print("%4d s  %3d  %s  %s | %3d %s %s"%(k*5,len(a),("%2d (%2d)"%(st.median(a),st.median(a)/1.609)) if a else "  -    ",("%3d"%max(d)) if d else "  -",len(r),("%2d"%st.median(r)) if r else " -",("%2d"%st.median(rd)) if rd else " -"))
ok=[m for m in lid if 30<=m<=12000]
print("\nlidar: %d of %d readings in range; value 20000 seen %d times"%(len(ok),len(lid),sum(1 for m in lid if m==20000)))
