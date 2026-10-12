'use strict';
const canvas=document.querySelector('#world'), ctx=canvas.getContext('2d');
const $=s=>document.querySelector(s), names=['RL','RR','FR','FL'];
$('#motors').innerHTML=names.map(n=>`<div class="motor"><span>${n}</span><div class="bar"><div class="fill" id="bar-${n}"></div></div><strong id="value-${n}">—</strong></div>`).join('');
let state=null, yaw=-.75, elevation=.65, distance=.38, drag=null, rotor=0;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]), dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const n=Math.hypot(...a);return a.map(v=>v/n)};
function rotate(p,q){const u=q.slice(0,3), t=cross(u,p).map(v=>v*2);return add(add(p,t.map(v=>v*q[3])),cross(u,t))}
function resize(){const r=canvas.getBoundingClientRect(),d=Math.min(devicePixelRatio||1,2);canvas.width=r.width*d;canvas.height=r.height*d;ctx.setTransform(d,0,0,d,0,0)}
new ResizeObserver(resize).observe(canvas);
function angles(q){const [x,y,z,w]=q;return [Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),Math.asin(clamp(2*(w*y-z*x),-1,1)),Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))].map(v=>v*180/Math.PI)}
async function poll(){
 const controller=new AbortController(), timer=setTimeout(()=>controller.abort(),5000);
 try{
  const response=await fetch('/api/state',{cache:'no-store',signal:controller.signal});
  if(!response.ok)throw new Error('viewer response '+response.status);
  const next=await response.json();
  state=next;update();
 }catch(error){
  // Keep the last pose visible, while making unavailable telemetry explicit.
  if(state)state.telemetry=null;
  if(browserArmed)disarmBrowser();
  $('#connection').className='connection stale';$('#connection span').textContent='Reconnecting';
  $('#flight-state').textContent='Unavailable';$('#state-detail').textContent='Controller telemetry is unavailable.';
  $('#freshness').textContent='Reconnecting';$('#empty').hidden=false;
  $('#empty').textContent=controller.signal.aborted?'Response timed out. Retrying…':'Cannot reach the viewer. Retrying…';
  names.forEach(n=>{$('#value-'+n).textContent='—';$('#bar-'+n).style.width='0%'});
 }finally{clearTimeout(timer);setTimeout(poll,200)}
}
function primary(){if(state.pose)return state.pose;const poses=Object.values(state.poses||{});return poses.length?poses[0]:null}
function swarmView(){const sw=state.swarm, live=sw&&state.server_unix-sw.written_unix<3;document.body.classList.toggle('swarm-mode',!!sw);
 if(!sw)return;$('#swarm-message').textContent=live?sw.message:'Coordinator not running';$('#swarm-cycle').textContent=live?'Flight '+sw.cycle:'Stale';
 const labels=['Boot','Standby','Maintenance','Armed','Failsafe','Fault'];
 $('#swarm-list').innerHTML=sw.drones.map((d,i)=>{const p=(state.poses||{})[d.name], m=live&&d.motors?d.motors.reduce((a,b)=>a+b,0)/4*100:0, st=d.state==null?'—':labels[d.state]||d.state;
  return `<div class="drone-row"><i style="background:${palette[i%palette.length]}"></i><div><strong>${d.name}</strong><span>ID ${d.system_id} · ${d.phase}</span></div><div><strong>${st}</strong><span>${p?p.position[2].toFixed(2)+' m':'—'}</span></div><div class="mini-bar"><div style="width:${m.toFixed(0)}%"></div></div></div>`}).join('')}
const palette=['#2f7f6b','#c0773f','#4f6fb0','#a5508a','#6b8f2f','#b5483f','#3f8fb5','#8a6b3f'];
function update(){syncControlButtons();swarmView();const pose=primary(),now=state.server_unix, live=pose&&now-pose.received_unix<2;
 $('#connection').className='connection '+(live?'live':'stale');$('#connection span').textContent=live?'Live simulation':'Waiting for simulation';$('#empty').hidden=!!pose;$('#empty').textContent=state.error||'Connecting to Gazebo…';$('#freshness').textContent=live?'Live':'Stale / unavailable';
 if(pose){const p=pose.position;$('#height').textContent=p[2].toFixed(3);$('#x').textContent=p[0].toFixed(3);$('#y').textContent=p[1].toFixed(3);angles(pose.quaternion_xyzw).forEach((v,i)=>$('#'+['roll','pitch','yaw'][i]).textContent=v.toFixed(1)+'°')}
 const t=state.telemetry, fresh=t&&now-t.motor_unix<3, hb=t&&now-t.heartbeat_unix<3;
 $('#flight-state').textContent=hb?(t.fault?'Fault':t.armed?'Armed':'Disarmed'):'Waiting';$('#state-detail').textContent=hb?'Live Rust controller state.':'Waiting for controller telemetry.';
 names.forEach((n,i)=>{const value=fresh?t.motors[i]*100:null;$('#value-'+n).textContent=value===null?'—':value.toFixed(0)+'%';$('#bar-'+n).style.width=(value||0)+'%'})
}
canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY,yaw,elevation];canvas.setPointerCapture(e.pointerId)});
canvas.addEventListener('pointermove',e=>{if(drag){cameraFeedback('Orbit view');yaw=drag[2]-(e.clientX-drag[0])*.006;elevation=clamp(drag[3]+(e.clientY-drag[1])*.005,.12,1.54)}});
canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();distance=clamp(distance*Math.exp(e.deltaY*.001),.19,swarmMode()?6:1.2)},{passive:false});
const swarmMode=()=>!!state?.swarm||Object.keys(state?.poses||{}).some(n=>n!=='flix');
function cameraFeedback(message,selected=null){
 $('#camera-feedback').textContent=message;
 $('#orbit').setAttribute('aria-pressed',String(selected==='orbit'));
 $('#top').setAttribute('aria-pressed',String(selected==='top'));
}
$('#orbit').onclick=()=>{yaw=-.75;elevation=.65;cameraFeedback('Isometric view selected','orbit')};
$('#top').onclick=()=>{elevation=1.54;cameraFeedback('Top view selected','top')};
$('#reset').onclick=()=>{yaw=-.75;elevation=.65;distance=swarmMode()?2.2:.38;cameraFeedback('Camera angle and zoom reset','orbit')};
let swarmFramed=false;
function draw(time){const {width:W,height:H}=canvas.getBoundingClientRect();ctx.clearRect(0,0,W,H);
 const swarm=swarmMode(), drones=swarm?Object.values(state.poses||{}).sort((a,b)=>a.name.localeCompare(b.name)):(state?.pose?[state.pose]:[]);
 if(!drones.length){requestAnimationFrame(draw);return}
 if(swarm&&!swarmFramed){distance=2.2;swarmFramed=true}
 const centre=drones.reduce((c,d)=>add(c,d.position.map(v=>v/drones.length)),[0,0,0]);
 const pos=swarm?centre:drones[0].position,target=[pos[0],pos[1],Math.max(.035,pos[2])];
 const eye=add(target,[distance*Math.cos(elevation)*Math.cos(yaw),distance*Math.cos(elevation)*Math.sin(yaw),distance*Math.sin(elevation)]),forward=norm(sub(target,eye)),right=norm(cross(forward,[0,0,1])),up=cross(right,forward),f=Math.min(W,H)*.95;
 function project(p){const d=sub(p,eye),z=dot(d,forward);return [W*.50+f*dot(d,right)/z,H*.57-f*dot(d,up)/z,z]}
 function line(points,color,width=1){const ps=points.map(project);if(ps.some(p=>p[2]<.02))return;ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke()}
 function poly(points,fill,stroke){const ps=points.map(project);if(ps.some(p=>p[2]<.02))return;ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=fill;ctx.fill();if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=1;ctx.stroke()}}
 const step=swarm?.2:.05,cx=Math.round(pos[0]/step)*step,cy=Math.round(pos[1]/step)*step;
 for(let i=-12;i<=12;i++){const k=i*step,e=12*step;line([[cx+k,cy-e,0],[cx+k,cy+e,0]],i===0?'#c4d2c5':'#dde4dc');line([[cx-e,cy+k,0],[cx+e,cy+k,0]],i===0?'#c4d2c5':'#dde4dc')}
 const items=[], sw=state.swarm, fresh=sw&&state.server_unix-sw.written_unix<3;
 drones.forEach((drone,index)=>{
 const pos=drone.position,q=drone.quaternion_xyzw;
 const shadow=Array.from({length:48},(_,i)=>[pos[0]+.085*Math.cos(i*Math.PI/24),pos[1]+.07*Math.sin(i*Math.PI/24),.0001]);poly(shadow,'rgba(40,66,45,.07)');
 const world=p=>add(rotate(p,q),pos), put=(points,fill,stroke)=>items.push({points:points.map(world),fill,stroke});
 const info=swarm&&fresh?sw.drones.find(d=>d.name===drone.name):null;
 const motorsOf=i=>swarm?(info&&info.motors?info.motors[i]:0):(state.telemetry && state.server_unix-state.telemetry.motor_unix<3 ? state.telemetry.motors[i] : 0);
 if(info&&info.target_m){const t=info.target_m;line([pos,t],'rgba(47,127,107,.35)',1);poly(Array.from({length:16},(_,i)=>[t[0]+.02*Math.cos(i*Math.PI/8),t[1]+.02*Math.sin(i*Math.PI/8),t[2]]),'rgba(47,127,107,.12)','rgba(47,127,107,.5)')}
 const cube=(center,size)=>{const [x,y,z]=center,[a,b,c]=size.map(v=>v/2),v=[[-a,-b,-c],[a,-b,-c],[a,b,-c],[-a,b,-c],[-a,-b,c],[a,-b,c],[a,b,c],[-a,b,c]].map(p=>add(p,[x,y,z]));[[[0,1,5,4],'#30443d'],[[1,2,6,5],'#405b4c'],[[2,3,7,6],'#526a58'],[[3,0,4,7],'#496350'],[[4,5,6,7],'#647e62']].forEach(([face,color])=>put(face.map(i=>v[i]),color,'#263e32'))};
 const centers=[[-.04243,.04243,.0142],[-.04243,-.04243,.0142],[.04243,-.04243,.0142],[.04243,.04243,.0142]], colors=['#d29769','#d29769','#4e9b89','#4e9b89'];
 centers.forEach(c=>line([world([0,0,.003]),world(c)],'#35493c',Math.max(3,f*.005/distance)));
 cube([0,0,.004],[.065,.041,.016]);cube([-.002,0,.016],[.041,.025,.009]);
 centers.forEach((c,i)=>{const ring=Array.from({length:40},(_,j)=>add(c,[.0275*Math.cos(j*Math.PI/20),.0275*Math.sin(j*Math.PI/20),.002]));put(ring,i<2?'rgba(199,141,100,.14)':'rgba(72,155,135,.14)',colors[i]);const m=motorsOf(i),a=time*.012*m+i*Math.PI/3;const blade=[[-.023,-.003,0],[.023,-.003,0],[.025,.002,0],[-.025,.002,0]].map(p=>add(c,[p[0]*Math.cos(a)-p[1]*Math.sin(a),p[0]*Math.sin(a)+p[1]*Math.cos(a),.003]));put(blade,colors[i]);const hub=Array.from({length:16},(_,j)=>add(c,[.006*Math.cos(j*Math.PI/8),.006*Math.sin(j*Math.PI/8),.004]));put(hub,'#384f41','#253b2f')});
 if(swarm){const top=project(world([0,0,.05]));if(top[2]>.02){ctx.font='600 11px -apple-system,sans-serif';ctx.fillStyle=palette[index%palette.length];ctx.fillText(drone.name,top[0]-14,top[1]-6)}}
 });
 items.sort((a,b)=>b.points.reduce((s,p)=>s+project(p)[2],0)/b.points.length-a.points.reduce((s,p)=>s+project(p)[2],0)/a.points.length).forEach(item=>poly(item.points,item.fill,item.stroke));
 if(!swarm){const q=drones[0].quaternion_xyzw,p0=drones[0].position,world=p=>add(rotate(p,q),p0);const axis=[[[.115,0,0],'#4c957d','X'],[[0,.115,0],'#ba8a67','Y']];axis.forEach(([p,color,label])=>{line([world([0,0,.012]),world(p)],color,1.2);const a=project(world(p));ctx.fillStyle=color;ctx.font='10px -apple-system,sans-serif';ctx.fillText(label,a[0]+5,a[1])})}
 requestAnimationFrame(draw)}
let controlToken=null, controlReady=false, browserArmed=false, inputBusy=false;
let flightInput={roll:0,pitch:0,yaw:0,thrust:0};
function syncControlButtons(){
 $('#arm').disabled=!controlReady||browserArmed;
 $('#throttle').disabled=!browserArmed;
 document.querySelectorAll('[data-axis]').forEach(b=>b.disabled=!browserArmed);
}
async function controlRequest(action,values,keepalive=false){
 if(!controlToken)throw new Error('Flight controls are connecting');
 const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),action==='reset'?18000:5000);
 try{
  const response=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json','X-Flix-Token':controlToken},body:JSON.stringify({action,values}),signal:controller.signal,keepalive});
  const result=await response.json();if(!result.ok)throw new Error(result.error);
  return result;
 }finally{clearTimeout(timer)}
}
function resetFlightInput(){flightInput={roll:0,pitch:0,yaw:0,thrust:0};$('#throttle').value=0;$('#throttle-value').textContent='0%';document.querySelectorAll('.held').forEach(b=>b.classList.remove('held'))}
async function disarmBrowser(){browserArmed=false;resetFlightInput();syncControlButtons();try{await controlRequest('disarm',undefined,true);$('#control-message').textContent='Disarm requested.'}catch(e){$('#control-message').textContent=e.message}}
$('#arm').onclick=async()=>{try{resetFlightInput();$('#control-message').textContent='Checking arming conditions…';await controlRequest('arm');browserArmed=true;syncControlButtons();$('#control-message').textContent='Armed. Increase thrust gradually.'}catch(e){$('#control-message').textContent=e.message}};
$('#disarm').onclick=disarmBrowser;
$('#clear-fault').onclick=async()=>{await disarmBrowser();try{await controlRequest('clear-fault');$('#control-message').textContent='Fault clear requested. Arm explicitly when ready.'}catch(e){$('#control-message').textContent=e.message}};
$('#reset-drone').onclick=async()=>{browserArmed=false;controlReady=false;resetFlightInput();syncControlButtons();$('#reset-drone').disabled=true;$('#control-message').textContent='Resetting drone upright…';try{await controlRequest('reset');$('#control-message').textContent='Simulation restarted. Wait for Ready, then arm explicitly.'}catch(e){$('#control-message').textContent=e.message}finally{$('#reset-drone').disabled=false}};
$('#throttle').oninput=e=>{flightInput.thrust=Number(e.target.value);$('#throttle-value').textContent=Math.round(flightInput.thrust*100)+'%'};
document.querySelectorAll('[data-axis]').forEach(button=>{
 const release=()=>{flightInput[button.dataset.axis]=0;button.classList.remove('held')};
 button.addEventListener('pointerdown',e=>{if(!browserArmed)return;button.setPointerCapture(e.pointerId);flightInput[button.dataset.axis]=Number(button.dataset.value);button.classList.add('held')});
 button.addEventListener('pointerup',release);button.addEventListener('pointercancel',release);button.addEventListener('lostpointercapture',release);
});
const flightKeys={w:['pitch',.2],s:['pitch',-.2],a:['roll',-.2],d:['roll',.2],q:['yaw',-.3],e:['yaw',.3]};
window.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();disarmBrowser();return}if(!browserArmed)return;const key=e.key.toLowerCase();if(flightKeys[key]){e.preventDefault();const [axis,value]=flightKeys[key];flightInput[axis]=value}else if(e.key==='ArrowUp'||e.key==='ArrowDown'){e.preventDefault();flightInput.thrust=clamp(Math.round((flightInput.thrust+(e.key==='ArrowUp'?.02:-.02))*100)/100,0,.8);$('#throttle').value=flightInput.thrust;$('#throttle-value').textContent=Math.round(flightInput.thrust*100)+'%'}});
window.addEventListener('keyup',e=>{const key=e.key.toLowerCase();if(flightKeys[key])flightInput[flightKeys[key][0]]=0});
window.addEventListener('blur',()=>{if(browserArmed)disarmBrowser()});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&browserArmed)disarmBrowser()});
window.addEventListener('pagehide',()=>{if(browserArmed)disarmBrowser()});
async function refreshControl(){try{const response=await fetch('/api/session',{cache:'no-store'}),session=await response.json();controlToken=session.token;if(session.control.enabled===false){$('.flight-controls').hidden=true;$('.pill').textContent='READ ONLY'}const wasReady=controlReady;controlReady=session.control.ready;if(browserArmed&&!session.control.armed){browserArmed=false;resetFlightInput();$('#control-message').textContent='Control stopped. Arm explicitly to resume.'}if(!browserArmed&&!controlReady)$('#control-message').textContent=session.control.error||'Connecting flight controls…';else if(!browserArmed&&controlReady&&!wasReady)$('#control-message').textContent='Ready. Press Arm to begin.';syncControlButtons()}catch(_){controlReady=false;syncControlButtons()}finally{setTimeout(refreshControl,500)}}
setInterval(async()=>{if(!browserArmed||inputBusy)return;inputBusy=true;try{await controlRequest('input',flightInput)}catch(e){browserArmed=false;resetFlightInput();syncControlButtons();$('#control-message').textContent=e.message}finally{inputBusy=false}},100);
syncControlButtons();refreshControl();poll();requestAnimationFrame(draw);
