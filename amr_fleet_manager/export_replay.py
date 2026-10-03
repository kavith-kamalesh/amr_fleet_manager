"""
export_replay.py -- record real trajectories from fleet_pibt_bench and write a
single self-contained dashboard.html (no server, no internet, no libraries).

  python3 fleet_pibt_bench.py --trials 50 --out results     # first: makes the CSVs
  python3 export_replay.py --out results
  open results/dashboard.html
"""
import argparse
import csv
import json
import os

import fleet_pibt_bench as b

SIZES = [12, 24, 48]
SEED = 42
FAIL_N = 24
FAIL_KEYS = ["central_live", "central_dead", "decentral"]


def record(n_robots, trial_seed):
    nbrs = b.build_graph()
    sc = b.make_scenario(n_robots, trial_seed)
    out = {"n": n_robots, "starts": sc[0], "goals": sc[1], "policies": {}}
    for p in b.POLICIES:
        trace = []
        r = b.run_trial(p, sc, nbrs=nbrs, trace=trace)
        out["policies"][p] = {"frames": trace, "result": r}
    return out


def record_failure(n_robots, trial):
    """Trial 0 of the failure experiment (fixed rule, not hand-picked)."""
    nbrs = b.build_graph()
    sc = b.make_scenario(n_robots, SEED * 100003 + n_robots * 1009 + trial)
    out = {"n": n_robots, "starts": sc[0], "goals": sc[1], "keys": FAIL_KEYS,
           "label": f"{n_robots} robots + breakdown + planner outage",
           "note": (f"One robot freezes for good at tick {b.BREAK_TICK} (shown as a dark "
                    f"red X). In the middle panel the central planner also goes down at "
                    f"that moment. This replay is trial {trial} of the 50-trial experiment "
                    f"(fixed rule, not hand-picked); see the table below for all trials."),
           "policies": {}}
    for name, pol, brk, dead in b.FAIL_CASES:
        if name not in FAIL_KEYS:
            continue
        ev = b.make_events(sc, trial, brk, dead)
        trace = []
        r = b.run_trial(pol, sc, nbrs=nbrs, trace=trace, events=ev)
        out["policies"][name] = {"frames": trace, "result": r}
        out["broken"], out["break_tick"] = ev["broken"], ev["break_tick"]
    return out


def load_csv(out_dir, name):
    path = os.path.join(out_dir, name)
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    scenarios = [record(n, SEED * 100003 + n * 1009 + 0) for n in SIZES]
    scenarios.append(record_failure(FAIL_N, 0))
    data = {
        "grid": b.GRID,
        "max_steps": b.MAX_STEPS,
        "scenarios": scenarios,
        "summary": load_csv(a.out, "scalability_summary.csv"),
        "failure": load_csv(a.out, "failure_summary.csv"),
    }
    html = TEMPLATE.replace("__DATA__", json.dumps(data, separators=(",", ":")))
    path = os.path.join(a.out, "dashboard.html")
    with open(path, "w") as f:
        f.write(html)
    print(f"Wrote {path} ({len(html) / 1024:.0f} KB). Open it in a browser.")


TEMPLATE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AMR Fleet Coordination - SIH26123 replay dashboard</title>
<style>
:root{--bg:#0f1318;--panel:#171d25;--ink:#e6edf3;--mute:#8b98a5;--line:#2a3441;
--red:#f0605d;--blue:#58a6ff;--green:#3fb950;--amber:#d29922}
@media (prefers-color-scheme: light){:root{--bg:#f4f6f8;--panel:#fff;--ink:#14202b;--mute:#5b6975;--line:#d5dce3}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{padding:16px 20px;border-bottom:1px solid var(--line)}
h1{margin:0;font-size:18px}header p{margin:4px 0 0;color:var(--mute);font-size:13px}
.bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;padding:12px 20px}
button,select{background:var(--panel);color:var(--ink);border:1px solid var(--line);
border-radius:6px;padding:6px 12px;font:inherit;cursor:pointer}
input[type=range]{accent-color:var(--blue)}
.scnote{padding:0 20px 10px;color:var(--mute);font-size:12.5px;max-width:1100px}
.grid3{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px;padding:0 20px 14px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{margin:0 0 2px;font-size:15px}.card .sub{color:var(--mute);font-size:12px;margin-bottom:8px}
canvas{width:100%;aspect-ratio:1/1;display:block;background:var(--bg);border-radius:6px}
.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:10px;text-align:center}
.stats div{background:var(--bg);border-radius:6px;padding:6px 2px}
.stats b{display:block;font-size:18px}.stats span{font-size:11px;color:var(--mute)}
.bad b{color:var(--red)}.ok b{color:var(--green)}
table{border-collapse:collapse;width:100%;font-size:13px}
th,td{padding:6px 8px;text-align:right;border-bottom:1px solid var(--line)}
th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){text-align:left}
tr.ours td{font-weight:600;background:rgba(63,185,80,.08)}
section{padding:0 20px 24px}section h3{margin:14px 0 8px;font-size:14px}
.note{color:var(--mute);font-size:12px;margin-top:8px;max-width:1100px}
</style></head><body>
<header><h1>Decentralised AMR fleet coordination &mdash; replay dashboard</h1>
<p>SIH26123 &middot; every frame below is recorded from <code>fleet_pibt_bench.py</code> (same scenario for every panel). Simulation, not hardware.</p></header>
<div class="bar">
<label>Scenario <select id="sel"></select></label>
<button id="play">Pause</button><button id="restart">Restart</button>
<label>Speed <input id="speed" type="range" min="1" max="12" value="4"></label>
<label>Tick <input id="scrub" type="range" min="0" max="0" value="0" style="width:220px"></label>
<span id="tickLabel"></span></div>
<div class="scnote" id="scnote"></div>
<div class="grid3" id="panels"></div>
<section><h3>Benchmark summary (from results/scalability_summary.csv)</h3>
<div id="summary"></div>
<p class="note">Penalised gain counts every unfinished stop-and-wait trial as the 400-tick timeout, so it overstates the benefit. The commonly-solved column compares only trials both policies finished.</p></section>
<section><h3>Robot breakdown + planner outage (from results/failure_summary.csv)</h3>
<div id="failure"></div>
<p class="note">One robot freezes for good at tick 3. Success = every other robot docked (a robot whose goal is the frozen robot's own cell cannot be served by any policy and is excluded). &ldquo;Planner down&rdquo; assumes robots keep their cached routes and fall back to the stop-and-wait rule; the control row (planner down, no breakdown) shows how much of that loss comes from the fallback rule alone, so read the planner-down rows as a bound under that assumption, not as a universal result. A robot waiting for the frozen robot's bay can still block an aisle, which accounts for the few unfinished robots in the decentralised rows.</p></section>
<script>
const DATA=__DATA__;
const G=DATA.grid;
const META={
 stop_and_wait:["Stop-and-wait baseline","halts while the next cell is occupied","#f0605d"],
 central:["Central planner","global space-time reservations","#58a6ff"],
 decentral:["Decentralised (ours)","local priority inheritance, no global plan","#3fb950"],
 central_live:["Central planner (alive)","re-plans around the frozen robot","#58a6ff"],
 central_dead:["Central planner DOWN","cached routes + local stop-and-wait","#e3873a"]};
const DEFAULT_KEYS=["stop_and_wait","central","decentral"];
let KEYS=DEFAULT_KEYS, sc=null, tick=0, playing=true, acc=0, last=performance.now(), END=400;
let ui={};
const sel=document.getElementById("sel");
DATA.scenarios.forEach((s,i)=>{const o=document.createElement("option");o.value=i;o.textContent=s.label||(s.n+" robots");if(i===1)o.selected=true;sel.appendChild(o);});
const panels=document.getElementById("panels");

function buildPanels(){
  panels.innerHTML="";ui={};
  KEYS.forEach(k=>{
    const d=document.createElement("div");d.className="card";
    d.innerHTML=`<h2 style="color:${META[k][2]}">${META[k][0]}</h2><div class="sub">${META[k][1]}</div>
    <canvas width="520" height="520"></canvas>
    <div class="stats"><div><b data-f="done">0</b><span>docked</span></div>
    <div><b data-f="wait">0</b><span>waiting</span></div>
    <div class="okc"><b data-f="coll">0</b><span>collisions</span></div>
    <div><b data-f="state">-</b><span>status</span></div></div>`;
    panels.appendChild(d);
    ui[k]={cv:d.querySelector("canvas"),els:{done:d.querySelector('[data-f=done]'),wait:d.querySelector('[data-f=wait]'),
    coll:d.querySelector('[data-f=coll]'),state:d.querySelector('[data-f=state]'),okc:d.querySelector('.okc')}};
  });
}
function broken(){return new Set(sc.broken||[]);}
function isBroken(i,t){return sc.broken&&sc.broken.includes(i)&&t>=sc.break_tick;}
function nTarget(){return sc.n-(sc.broken?sc.broken.length:0);}
function DATA_FRAMES(k){return sc.policies[k].frames}
function cellKey(p){return p?p[0]+","+p[1]:null}
function frameDone(f){const b=broken();return f.every((p,i)=>!p||b.has(i));}

function stats(k,t){
  const fr=DATA_FRAMES(k), n=sc.n, tt=Math.min(t,fr.length-1), f=fr[tt], b=broken();
  const prev=fr[Math.max(0,tt-1)];
  let done=0,wait=0,coll=0;
  for(let i=0;i<n;i++){ if(b.has(i)) continue; if(!f[i]) done++; else if(prev[i]&&cellKey(prev[i])===cellKey(f[i])) wait++; }
  for(let s=1;s<=tt;s++){
    const a=fr[s-1],c=fr[s],seen=new Set();
    for(let i=0;i<n;i++){ if(!c[i]) continue; const kk=cellKey(c[i]); if(seen.has(kk)) coll++; seen.add(kk);
      for(let j=i+1;j<n;j++){ if(a[i]&&a[j]&&c[i]&&c[j]&&cellKey(c[i])===cellKey(a[j])&&cellKey(c[j])===cellKey(a[i])&&cellKey(a[i])!==cellKey(c[i])) coll++; } }
  }
  return {done,wait,coll,finished:done===nTarget()};
}
const cache={};
function cachedStats(k,t){const id=sc.label+sc.n+k+t;return cache[id]||(cache[id]=stats(k,t));}

function draw(k,t){
  const {cv,els}=ui[k], c=cv.getContext("2d"), W=cv.width, cell=W/G;
  const css=getComputedStyle(document.body);
  c.clearRect(0,0,W,W);
  c.strokeStyle=css.getPropertyValue("--line");c.lineWidth=1;
  for(let i=0;i<=G;i++){c.beginPath();c.moveTo(i*cell,0);c.lineTo(i*cell,W);c.stroke();c.beginPath();c.moveTo(0,i*cell);c.lineTo(W,i*cell);c.stroke();}
  const fr=DATA_FRAMES(k), tt=Math.min(t,fr.length-1), f=fr[tt], prev=fr[Math.max(0,tt-1)];
  sc.goals.forEach((g,i)=>{ if(!f[i]||isBroken(i,t)) return; c.strokeStyle=META[k][2];c.globalAlpha=.45;
    c.strokeRect(g[0]*cell+cell*.3,g[1]*cell+cell*.3,cell*.4,cell*.4);c.globalAlpha=1;});
  for(let i=0;i<sc.n;i++){ if(!f[i]) continue;
    const x=f[i][0]*cell+cell/2,y=f[i][1]*cell+cell/2, bk=isBroken(i,t);
    const waiting=!bk&&prev[i]&&cellKey(prev[i])===cellKey(f[i]);
    c.beginPath();c.arc(x,y,cell*.36,0,6.283);
    c.fillStyle=bk?"#7a1f1f":(waiting&&t>0?"#d29922":META[k][2]);c.fill();
    if(bk){c.strokeStyle="#ffffff";c.lineWidth=3;c.beginPath();const r=cell*.17;
      c.moveTo(x-r,y-r);c.lineTo(x+r,y+r);c.moveTo(x+r,y-r);c.lineTo(x-r,y+r);c.stroke();c.lineWidth=1;}
    else if(cell>26){c.fillStyle="#0b0f14";c.font=`${Math.floor(cell*.34)}px system-ui`;c.textAlign="center";c.textBaseline="middle";c.fillText(i,x,y);} }
  const s=cachedStats(k,t);
  els.done.textContent=s.done+"/"+nTarget();els.wait.textContent=s.wait;els.coll.textContent=s.coll;
  els.okc.className=s.coll===0?"ok":"bad";
  const stalled=!s.finished&&tt>=5&&JSON.stringify(fr[tt])===JSON.stringify(fr[tt-5]);
  const lbl=els.state.nextElementSibling, ft=finishTick(k);
  lbl.textContent=(s.finished&&ft>=0)?"done at tick "+ft:(stalled?"no progress":"status");
  els.state.textContent=s.finished?"DONE":(stalled?"DEADLOCK":"running");
  els.state.style.color=s.finished?"var(--green)":(stalled?"var(--red)":"var(--ink)");
}
function isResolved(k,t){const fr=DATA_FRAMES(k),tt=Math.min(t,fr.length-1),f=fr[tt];
  if(frameDone(f)) return true;
  return tt>=5&&JSON.stringify(f)===JSON.stringify(fr[tt-5]);}
function endTick(){for(let t=0;t<=400;t++){if(KEYS.every(k=>isResolved(k,t))) return Math.min(t+8,400);} return 400;}
function finishTick(k){return DATA_FRAMES(k).findIndex(f=>frameDone(f));}
function maxLen(){return END;}
function render(){KEYS.forEach(k=>draw(k,tick));
  document.getElementById("scrub").value=tick;
  document.getElementById("tickLabel").textContent="tick "+tick+" / "+maxLen();}
function setScenario(i){sc=DATA.scenarios[i];KEYS=sc.keys||DEFAULT_KEYS;buildPanels();END=endTick();tick=0;
  document.getElementById("scnote").textContent=sc.note||"";
  document.getElementById("scrub").max=maxLen();render();}
sel.onchange=()=>setScenario(+sel.value);
document.getElementById("play").onclick=e=>{playing=!playing;e.target.textContent=playing?"Pause":"Play"};
document.getElementById("restart").onclick=()=>{tick=0;playing=true;document.getElementById("play").textContent="Pause";render()};
document.getElementById("scrub").oninput=e=>{tick=+e.target.value;render()};
function loop(now){const dt=now-last;last=now;
  if(playing){acc+=dt;const step=1000/(+document.getElementById("speed").value*2.5);
    while(acc>step){acc-=step;if(tick<maxLen())tick++;}}
  render();requestAnimationFrame(loop);}

(function(){const rows=DATA.summary;if(!rows.length){document.getElementById("summary").textContent="Run fleet_pibt_bench.py first, then export_replay.py again to embed the summary.";return;}
  let h="<table><tr><th>Robots</th><th>Policy</th><th>Completed %</th><th>Collisions</th><th>Mean ticks</th><th>Gain % (penalised)</th><th>Gain % (commonly solved)</th></tr>";
  rows.forEach(r=>{const g=parseFloat(r.gain_on_commonly_solved_pct);
    h+=`<tr${r.policy==="decentral"?' class="ours"':""}><td>${r.n_robots}</td><td>${META[r.policy][0]}</td><td>${(+r.success_pct).toFixed(0)}</td><td>${r.collisions}</td><td>${(+r.mean_completion).toFixed(1)}</td><td>${(+r.gain_vs_stop_and_wait_pct).toFixed(1)}</td><td>${isNaN(g)?"n/a":g.toFixed(1)}</td></tr>`;});
  document.getElementById("summary").innerHTML=h+"</table>";})();
(function(){const rows=DATA.failure;const el=document.getElementById("failure");
  if(!rows.length){el.textContent="Run fleet_pibt_bench.py (with 24 and 48 in --sizes), then export_replay.py again.";return;}
  const L={central_live:"Central, planner alive (re-plans)",central_dead:"Central, planner DOWN (cached routes + stop-and-wait)",
   decentral:"Decentralised (ours)",central_dead_no_breakdown:"Control: planner down, no breakdown"};
  let h="<table><tr><th>Robots</th><th>Case</th><th>Success %</th><th>Collisions</th><th>Mean ticks</th><th>Unfinished (avg robots)</th></tr>";
  rows.forEach(r=>{h+=`<tr${r.case==="decentral"?' class="ours"':""}><td>${r.n_robots}</td><td>${L[r.case]||r.case}</td><td>${(+r.success_pct).toFixed(0)}</td><td>${r.collisions}</td><td>${(+r.mean_completion).toFixed(1)}</td><td>${(+r.mean_unfinished).toFixed(2)}</td></tr>`;});
  el.innerHTML=h+"</table>";})();
setScenario(1);requestAnimationFrame(loop);
</script></body></html>
"""

if __name__ == "__main__":
    main()
