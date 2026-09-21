"""Render captioned demo frames from real Engine exports; requires Pillow/numpy."""
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output/demo'
FRAMES = Path(os.environ.get('TRACKFUSION_FRAMES', '/tmp/trackfusion-video/frames'))
FRAMES.mkdir(parents=True, exist_ok=True)
DATA = json.loads((OUT / 'trace.json').read_text())
TRACE = DATA['frames']
W,H,FPS,DURATION = 1280,720,24,51
BG='#080f1d'; PANEL='#101d2e'; BORDER='#25374c'; WHITE='#f1f6fd'; MUTED='#91a6bf'
CYAN='#53e2db'; PURPLE='#b6a0ff'; AMBER='#ffbe6a'; GREY='#607489'
COLORS={'aaa111':CYAN,'bbb222':PURPLE}
NAMES={'aaa111':'ALPHA','bbb222':'BRAVO'}
FONT='/System/Library/Fonts/Avenir Next.ttc'
fonts={}
def font(size,bold=False):
    key=(size,bold)
    if key not in fonts: fonts[key]=ImageFont.truetype(FONT,size,index=0 if bold else 5)
    return fonts[key]
def txt(d,xy,s,size=22,color=WHITE,bold=False):
    d.text(xy,s,font=font(size,bold),fill=color)
def box(d,b,fill=PANEL,outline=BORDER,r=16): d.rounded_rectangle(b,r,fill=fill,outline=outline,width=1)
def line(d,a,b,c=BORDER,width=1): d.line([a,b],fill=c,width=width)
def base(t):
    im=Image.new('RGB',(W,H),BG); d=ImageDraw.Draw(im)
    txt(d,(48,28),'TRACKFUSION',20,CYAN,True)
    txt(d,(236,30),' /  SAMUEL BIXEL',16,MUTED)
    txt(d,(1000,30),'ENGINE OUTPUT / DEMO',14,MUTED)
    line(d,(48,65),(1232,65))
    line(d,(48,697),(1232,697))
    line(d,(48,697),(48+1184*min(1,t/DURATION),697),CYAN,3)
    txt(d,(48,665),'SIMULATED AIRCRAFT  /  ACTUAL GO TRACKER',13,MUTED)
    txt(d,(1060,665),f'{int(t):02d} / {DURATION:02d} SEC',13,MUTED)
    return im,d
def aircraft(d,p,v,c,size=11):
    x,y=p; vx,vy=v; a=math.atan2(-vy,vx)
    verts=[(size,0),(-size*.8,-size*.65),(-size*.35,0),(-size*.8,size*.65)]
    pts=[(x+u*math.cos(a)-v*math.sin(a),y+u*math.sin(a)+v*math.cos(a)) for u,v in verts]
    d.polygon(pts,fill=c)
def simtime(t):
    if t<14:return max(0,(t-5)/9*48)
    if t<24:return 48+(t-14)/10*48
    if t<34:return 96+(t-24)/10*20
    return min(160,116+(t-34)/6*44)
def interpolated(seconds):
    idx=min(40,int(seconds//4)); f=TRACE[idx]; nxt=TRACE[min(40,idx+1)]
    a=(seconds-f['Seconds'])/4
    tracks=[]
    for tr in f['Tracks']:
        x=list(tr['State']['X'])
        future=next((n for n in nxt['Tracks'] if n['ID']==tr['ID']),None)
        if future:
            x=[u+(v-u)*a for u,v in zip(x,future['State']['X'])]
        tracks.append((tr,x))
    return idx,f,tracks
def scope(d,rect,seconds,raw=False):
    x0,y0,x1,y1=rect; box(d,rect)
    cx=(x0+x1)/2;cy=(y0+y1)/2; scale=min((x1-x0-44)/44000,(y1-y0-44)/24000)
    def pos(e,n):return (cx+e*scale,cy-n*scale)
    for km in (-20,-10,0,10,20):
        x,y=pos(km*1000,0)
        if x0+15<x<x1-15:line(d,(x,y0+16),(x,y1-16),'#17283b')
    for km in (-10,0,10):
        x,y=pos(0,km*1000)
        if y0+15<y<y1-15:line(d,(x0+16,y),(x1-16,y),'#17283b')
    for km in (5,10):
        r=km*1000*scale;d.ellipse((cx-r,cy-r,cx+r,cy+r),outline='#1d3045',width=1)
    d.ellipse((cx-3,cy-3,cx+3,cy+3),fill=GREY)
    idx,f,tracks=interpolated(seconds)
    def inside(p):return x0+18<p[0]<x1-18 and y0+25<p[1]<y1-22
    for o in f['Observations']:
        p=pos(o['e'],o['n'])
        if inside(p):
            x,y=p;line(d,(x-4,y-4),(x+4,y+4),AMBER,2);line(d,(x-4,y+4),(x+4,y-4),AMBER,2)
    if not raw:
        for tr,x in tracks:
            identity=tr['Identity']; c=COLORS.get(identity,GREY);p=pos(x[0],x[1])
            if not inside(p):continue
            if identity:
                trail=[]
                for past in TRACE[max(0,idx-10):idx+1]:
                    old=next((z for z in past['Tracks'] if z['ID']==tr['ID']),None)
                    if old:trail.append(pos(*old['State']['X'][:2]))
                trail.append(p)
                if len(trail)>1:d.line(trail,fill=c,width=2)
                # A 95% position-covariance ellipse, in the display's actual scale.
                cov=np.array(tr['State']['P'])[:2,:2]
                vals,vec=np.linalg.eigh(cov)
                ellipse=[]
                for theta in np.linspace(0,2*math.pi,48):
                    offset=vec @ (np.sqrt(np.maximum(vals,0)*5.991)*np.array([math.cos(theta),math.sin(theta)]))
                    ellipse.append(pos(x[0]+offset[0],x[1]+offset[1]))
                d.line(ellipse,fill=c,width=1)
                aircraft(d,p,x[2:],c)
                label=NAMES[identity]
                ly=p[1]+16 if identity=='aaa111' else p[1]-37
                txt(d,(p[0]+15,ly),label,17,c,True)
                if tr['MissCounter']>0:txt(d,(p[0]+15,ly+23),'PREDICTING',11,AMBER,True)
            else:
                xx,yy=p;d.ellipse((xx-3,yy-3,xx+3,yy+3),outline=GREY,width=1)
    txt(d,(x0+20,y0+12),'LOCAL AIRSPACE  /  EAST–NORTH',12,MUTED)
    txt(d,(x1-180,y0+12),f'SIM TIME  {seconds:05.1f}s',12,MUTED)
    length=5000*scale;line(d,(x0+20,y1-24),(x0+20+length,y1-24),MUTED,2)
    txt(d,(x0+26+length,y1-33),'5 km',12,MUTED)
    return f
def caption(d,main,sub):
    txt(d,(48,592),main,27,WHITE,True)
    txt(d,(48,631),sub,17,MUTED)
def render(t):
    im,d=base(t)
    if t<5:
        txt(d,(48,119),'Two sensors.',60,WHITE,True)
        txt(d,(48,188),'One air picture.',60,CYAN,True)
        txt(d,(50,285),'A multi-sensor aircraft tracker',27,WHITE)
        txt(d,(50,325),'built in Go.',27,WHITE)
        box(d,(50,415,531,486))
        txt(d,(70,435),'IDENTIFY  /  ASSOCIATE  /  TRACK',19,CYAN,True)
        scope(d,(586,116,1232,535),40+t*4)
        caption(d,'Aircraft data is noisy. The picture should stay coherent.',
                'A 51-second look at the problem, the behavior, and the engineering.')
    elif t<40:
        seconds=simtime(t)
        stage=0 if t<14 else 1 if t<24 else 2 if t<34 else 3
        headings=['01  /  Start with imperfect observations','02  /  Turn observations into tracks','03  /  Keep estimating through a gap','04  /  Reconnect when detections return']
        txt(d,(48,89),headings[stage],35,WHITE,True)
        subtitles=['An initial ADS-B fix supplies identity; simulated radar adds noisy, anonymous returns.',
                    'Estimate motion, then assign each scan across the available tracks.',
                    'ALPHA deliberately misses three radar sweeps in this scenario.',
                    'The next radar return updates ALPHA’s existing track.']
        txt(d,(48,144),subtitles[stage],20,MUTED)
        f=scope(d,(48,191,890,567),seconds,raw=(t<8.5))
        box(d,(912,191,1232,567))
        txt(d,(934,208),'WHAT YOU ARE SEEING',15,MUTED,True)
        if stage==0:
            txt(d,(934,252),'×',30,AMBER,True);txt(d,(974,258),'Radar observations',18,WHITE)
            txt(d,(934,305),'ALPHA',20,CYAN,True)
            txt(d,(934,337),'BRAVO',20,PURPLE,True)
            txt(d,(934,376),'Labels come from ADS-B.',17,MUTED)
            txt(d,(934,416),'Some returns are clutter.',17,AMBER)
            txt(d,(934,442),'They can start temporary',16,MUTED)
            txt(d,(934,465),'anonymous tracks.',16,MUTED)
            txt(d,(934,519),'Grey rings = unnamed tracks',14,GREY)
            caption(d,'Which observation belongs to which aircraft?',
                    'Radar points carry no identity. The engine has to associate them.')
        elif stage==1:
            txt(d,(934,247),'PREDICT',17,CYAN,True)
            txt(d,(934,278),'Where should it be next?',17,WHITE)
            txt(d,(934,328),'ASSOCIATE',17,PURPLE,True)
            txt(d,(934,359),'Which return fits the track?',17,WHITE)
            txt(d,(934,409),'UPDATE',17,CYAN,True)
            txt(d,(934,440),'Refine position and speed.',17,WHITE)
            txt(d,(934,519),'Trails show estimated motion.',14,MUTED)
            caption(d,'Follow the same aircraft as their paths pass nearby.',
                    'Kalman prediction + uncertainty-based gating + global assignment.')
        elif stage==2:
            a=next(x for x in f['Tracks'] if x['Identity']=='aaa111')
            miss=a['MissCounter']
            txt(d,(934,245),'ALPHA',23,CYAN,True)
            txt(d,(934,283),'PREDICTION ONLY' if miss else 'RADAR UPDATE',18,AMBER if miss else CYAN,True)
            txt(d,(934,330),f'{miss}',52,WHITE,True)
            txt(d,(997,354),'missed sweeps',17,MUTED)
            txt(d,(934,422),'Position uncertainty',17,WHITE)
            # Actual trace of sqrt(Pee + Pnn), not an illustrative score.
            line(d,(937,511),(1208,511),BORDER)
            pts=[]
            for old in TRACE[22:min(30,int(seconds//4)+1)]:
                tr=next(z for z in old['Tracks'] if z['Identity']=='aaa111')
                sigma=math.sqrt(tr['State']['P'][0][0]+tr['State']['P'][1][1])
                pts.append((937+(old['Seconds']-88)/28*271,511-sigma/250*55))
            if len(pts)>1:d.line(pts,fill=AMBER,width=3)
            txt(d,(934,525),'Grows during the gap; shrinks on update.',12,MUTED)
            caption(d,'A missing detection does not immediately erase a track.',
                    'The estimate continues forward while its uncertainty grows.')
        else:
            txt(d,(934,245),'ALPHA',23,CYAN,True)
            txt(d,(934,287),'RADAR UPDATE RESTORED',16,CYAN,True)
            txt(d,(934,349),'Same track.',26,WHITE,True)
            txt(d,(934,388),'Fresh observation.',26,WHITE,True)
            txt(d,(934,452),'The filter corrects the',18,MUTED)
            txt(d,(934,480),'predicted position.',18,MUTED)
            caption(d,'The estimated track reconnects with incoming observations.',
                    'This is one repeatable scenario, run through the actual Go pipeline.')
    elif t<46:
        txt(d,(48,102),'Built to inspect. Built to replay.',44,WHITE,True)
        txt(d,(48,165),'The visualization is new. The tracking comes from the existing engine.',22,MUTED)
        cards=[('01','ESTIMATE','Kalman filter',['Position + velocity','Local ENU coordinates'],CYAN),
               ('02','ASSOCIATE','Global assignment',['Mahalanobis gating','Hungarian algorithm'],PURPLE),
               ('03','REPRODUCE','Offline replay',['JSONL scan recordings','93 tests across 8 packages'],AMBER)]
        for i,(num,label,title,items,c) in enumerate(cards):
            x=48+i*401;box(d,(x,234,x+382,510))
            txt(d,(x+24,254),num,18,c,True)
            txt(d,(x+24,306),label,17,c,True)
            txt(d,(x+24,344),title,25,WHITE,True)
            for j,s in enumerate(items):txt(d,(x+24,412+j*30),s,18,MUTED)
        caption(d,'Same inputs. Same pipeline. Repeatable results.',
                'Live mode integrates OpenSky ADS-B using OAuth2. This demo runs fully offline.')
    else:
        txt(d,(48,125),'From noisy observations',55,WHITE,True)
        txt(d,(48,195),'to continuous tracks.',55,CYAN,True)
        txt(d,(50,303),'TRACKFUSION',24,WHITE,True)
        txt(d,(50,345),'Designed and built by Samuel Bixel',26,MUTED)
        box(d,(48,425,1232,521))
        txt(d,(75,449),'github.com/sambixel/trackfusion',35,CYAN,True)
        caption(d,'Explore the code. Replay the scenario.',
                'Go  /  state estimation  /  sensor integration  /  deterministic replay')
    return im

if __name__=='__main__':
    if '--preview' in sys.argv:
        times=[2,7,12,20,28,32.5,37,43,48]
        tiles=[]
        for i,t in enumerate(times):
            im=render(t);im.save(OUT/f'preview-{i}.png');tiles.append(im.resize((640,360)))
        sheet=Image.new('RGB',(1920,1080),BG)
        for i,im in enumerate(tiles):sheet.paste(im,((i%3)*640,(i//3)*360))
        sheet.save(OUT/'storyboard.jpg',quality=93)
        render(20).save(OUT/'poster.jpg',quality=95)
    else:
        for i in range(FPS*DURATION):
            render(i/FPS).save(FRAMES/f'{i:05d}.jpg',quality=93,subsampling=0)
            if i%240==0:print(f'Rendered {i}/{FPS*DURATION} frames',flush=True)
