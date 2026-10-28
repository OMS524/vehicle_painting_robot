#!/usr/bin/env python3
"""NLP 배치 결과(log + joint CSV + 입력 CSV)를 요약 JSON 으로 뽑는다."""
import csv, json, math, os, re, sys
import numpy as np
from pathlib import Path
from paint_viz.gantry_env import load_gantry_env

# 갠트리 환경: $GANTRY_ENV (기본 legacy). legacy 는 종전 경로(log/, out/, summary.json), 그 외는 *_<env> 접미사 (env.sh 와 동일 규칙)
ENV=load_gantry_env(os.environ.get('GANTRY_ENV'))
SUF='' if ENV.name=='legacy' else f'_{ENV.name}'
NLP=Path(__file__).resolve().parent
LOG=NLP/f'log{SUF}'; OUT=NLP/f'out{SUF}'; IN=NLP/'input'; SUMMARY=NLP/f'summary{SUF}.json'
PARTS=['ES300h_hood','ES300h_front_bumper','NX350h_right_fender','ES300h_trunk']
PLACE={
 'ES300h_hood':          ([-0.751,-1.316,1.879],[0,0,0]),
 'ES300h_front_bumper':  ([-1.002,-1.040,4.872],[1.5707963,1.5707963,0]),
 'NX350h_right_fender':  ([-0.990,0.051,2.280],[0,0,0]),
 'ES300h_trunk':         ([-0.806,-1.367,1.803],[0,0,0]),
}
# 갠트리 조인트 한계는 env yaml(=URDF) 에서. 포화 판정은 하한/상한 모두 본다 (코너 원점 환경은 [0, stroke]).
LIM=ENV.joint_limits(); (GXL,GXU),(GYL,GYU)=LIM['gantry_x'],LIM['gantry_y']
def rpyR(r,p,y):
    sr,cr=math.sin(r),math.cos(r); sp,cp=math.sin(p),math.cos(p); sy,cy=math.sin(y),math.cos(y)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@
            np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@
            np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))

result={}
for part in PARTS:
    if not (LOG/f'{part}.log').is_file(): print(f'{part}: {LOG}/{part}.log 없음 — 건너뜀'); continue
    log=(LOG/f'{part}.log').read_text()
    rows=[]
    for m in re.finditer(r'\[nlp\]\[time\]   (trajectory_\d+\.csv)\s+points=\s*(\d+) seed=\s*([\d.]+)s NLP=\s*([\d.]+)s total=\s*([\d.]+)s \(\s*([\d.]+) ms/pt\) path=\s*([\d.]+)s ratio=\s*([\d.]+)x', log):
        rows.append(dict(name=m.group(1),points=int(m.group(2)),seed=float(m.group(3)),
                         nlp=float(m.group(4)),total=float(m.group(5)),ms_pt=float(m.group(6)),
                         path=float(m.group(7)),ratio=float(m.group(8))))
    tot=re.search(r'TOTAL\s+points=\s*(\d+) seed=\s*([\d.]+)s NLP=\s*([\d.]+)s total=\s*([\d.]+)s \(\s*([\d.]+) ms/pt\) path=\s*([\d.]+)s ratio=\s*([\d.]+)x', log)
    wall=re.search(r'wall clock.*?([\d.]+)s', log)
    mets=[dict(pos=float(a),rot=float(b),manip=float(c)) for a,b,c in
          re.findall(r'max_pos_err=([\d.e+-]+)m max_rot_err=([\d.e+-]+)deg min_manip=([\d.e+-]+)',log)]

    # joint CSV 이어붙이기
    T=[];Q=[];QD=[];QDD=[];bounds=[]
    t_off=0.0
    for f in sorted(OUT.glob(f'{part}/trajectory_*_nlp_joint.csv')):
        data=list(csv.reader(f.open())); hdr=data[0]; arr=np.array(data[1:],dtype=float)
        T.append(arr[:,0]+t_off); Q.append(arr[:,2:10]); QD.append(arr[:,10:18]); QDD.append(arr[:,18:26])
        t_off+=arr[-1,0]; bounds.append(t_off)
    T=np.concatenate(T);Q=np.vstack(Q);QD=np.vstack(QD);QDD=np.vstack(QDD)
    sat_x=float(((Q[:,0]<GXL+0.02)|(Q[:,0]>GXU-0.02)).mean()); sat_y=float(((Q[:,1]<GYL+0.02)|(Q[:,1]>GYU-0.02)).mean())

    # TCP 월드 경로 (입력 CSV + 배치변환)
    t_place,rpy=PLACE[part]; t_place=ENV.part_xyz(t_place); R=rpyR(*rpy); tvec=np.array(t_place)   # legacy 기준 배치 + env 오프셋
    P=[];paint=[];rowidx=[]
    for ri,f in enumerate(sorted(IN.glob(f'{part}/trajectory_*.csv'))):
        for r in csv.DictReader(f.open()):
            P.append([float(r['position_x']),float(r['position_y']),float(r['position_z'])])
            paint.append(int(float(r['paint']))); rowidx.append(ri)
    P=(np.array(P)@R.T)+tvec

    ds=max(1,len(T)//1500)
    result[part]=dict(
        timing_rows=rows,
        total=dict(points=int(tot.group(1)),seed=float(tot.group(2)),nlp=float(tot.group(3)),
                   plan=float(tot.group(4)),ms_pt=float(tot.group(5)),path=float(tot.group(6)),
                   ratio=float(tot.group(7)),wall=float(wall.group(1))) if tot else None,
        metrics=dict(max_pos_err_mm=max(m['pos'] for m in mets)*1000,
                     max_rot_err_deg=max(m['rot'] for m in mets),
                     min_manip=min(m['manip'] for m in mets), n_rows=len(mets)),
        gantry=dict(x_min=float(Q[:,0].min()),x_max=float(Q[:,0].max()),
                    y_min=float(Q[:,1].min()),y_max=float(Q[:,1].max()),
                    sat_x=sat_x,sat_y=sat_y,lim_x=[GXL,GXU],lim_y=[GYL,GYU]),
        vel_max=dict(gantry=float(np.abs(QD[:,:2]).max()),arm_deg=float(np.degrees(np.abs(QD[:,2:]).max()))),
        acc_max=dict(gantry=float(np.abs(QDD[:,:2]).max()),arm_deg=float(np.degrees(np.abs(QDD[:,2:]).max()))),
        series=dict(t=np.round(T[::ds],2).tolist(),
                    q=[np.round(Q[::ds,j],4).tolist() for j in range(8)],
                    row_bounds=np.round(bounds,1).tolist()),
        tcp=dict(x=np.round(P[::2,0],3).tolist(),y=np.round(P[::2,1],3).tolist(),
                 z=np.round(P[::2,2],3).tolist(),paint=paint[::2],row=rowidx[::2]),
        placement=dict(xyz=t_place,rpy=rpy),
        gantry_env=ENV.name,
    )
json.dump(result,open(SUMMARY,'w'))
for p in PARTS:
    if p not in result: continue
    r=result[p]
    if r['total'] is None:   # 마지막 row Infeasible 등으로 TOTAL 줄이 없는 로그
        print(f"{p:22s} rows={r['metrics']['n_rows']} (NLP 중단 — TOTAL 없음) pos_err={r['metrics']['max_pos_err_mm']:.4f}mm gantry_sat=({r['gantry']['sat_x']:.1%},{r['gantry']['sat_y']:.1%})"); continue
    print(f"{p:22s} plan={r['total']['plan']:7.1f}s ({r['total']['ms_pt']:5.1f}ms/pt) path={r['total']['path']:6.1f}s "
          f"pos_err={r['metrics']['max_pos_err_mm']:.4f}mm rot_err={r['metrics']['max_rot_err_deg']:.4f}deg "
          f"min_manip={r['metrics']['min_manip']:.3f} gantry_sat=({r['gantry']['sat_x']:.1%},{r['gantry']['sat_y']:.1%}) "
          f"vmax(arm)={r['vel_max']['arm_deg']:.1f}deg/s amax(arm)={r['acc_max']['arm_deg']:.1f}deg/s2")
