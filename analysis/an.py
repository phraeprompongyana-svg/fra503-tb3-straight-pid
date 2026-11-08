import csv,math,statistics as st,json,sys
from collections import defaultdict
rows=list(csv.DictReader(open(sys.argv[1] if len(sys.argv)>1 else 'data/assignment9/lab_results.csv')))
f=lambda x: float(x) if x not in ('',None) else None
cells=defaultdict(list)
for r in rows:
    if r['valid'].upper()=='N': continue
    cells[(r['condition'],float(r['v']))].append(r)
def ms(xs):
    xs=[x for x in xs if x is not None]
    return (st.mean(xs), st.stdev(xs) if len(xs)>1 else 0.0, len(xs)) if xs else (None,None,0)
out={}
for k in sorted(cells,key=lambda k:(['NORMAL','COM_OFFSET','BATT_TILT'].index(k[0]),k[1])):
    g=cells[k]
    tape=ms([f(r['real_lat_cm']) for r in g])
    odl=ms([f(r['lat_end_cm']) for r in g])
    odlabs=ms([abs(f(r['lat_end_cm'])) for r in g])
    yaw=ms([f(r['dyaw_deg']) for r in g])
    bat=ms([f(r['batt_v']) for r in g])
    dist=ms([f(r['odom_cm']) for r in g])
    out[k]=dict(n=len(g),tape=tape,odl=odl,odlabs=odlabs,yaw=yaw,bat=bat,dist=dist)
    print(k, 'n',len(g),'tape %.2f±%.2f(n%d)'%tape,'odomLat %+.2f±%.2f'%odl[:2],'|odomLat| %.2f'%odlabs[0],'yaw %+.2f±%.2f'%yaw[:2],'batt %.2f'%bat[0],'dist %.1f'%dist[0])
# tape sign check: all positive?
print('tape min',min(f(r['real_lat_cm']) for r in rows if f(r['real_lat_cm']) is not None))
# correlation tape vs |odom lat| per condition
import itertools
for c in ['NORMAL','COM_OFFSET','BATT_TILT']:
    xs=[(f(r['real_lat_cm']),abs(f(r['lat_end_cm']))) for r in rows if r['condition']==c and f(r['real_lat_cm']) is not None]
    a=[x for x,_ in xs]; b=[y for _,y in xs]
    ma,mb=st.mean(a),st.mean(b)
    cov=sum((x-ma)*(y-mb) for x,y in xs); 
    r_=cov/math.sqrt(sum((x-ma)**2 for x in a)*sum((y-mb)**2 for y in b))
    print(c,'corr tape vs |odom lat|: %.2f'%r_, 'n',len(xs))
json.dump({'%s|%s'%k:v for k,v in out.items()},open('cells.json','w'))
