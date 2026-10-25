"use strict";
const $ = id => document.getElementById(id);
let config = null, chassis = null, deadman = false, commandTimer = null;
let activeMotionPreset = null;
let maintenanceHeld = false, maintenanceTimer = null, latestStatus = null;
let maintenanceTargetAngleDeg = null;
let maintenanceCommandId = "";
let zeroPreviewToken = "";
let translation = {x: 0, y: 0};
let odomTrail = [], trailFrame = null, mapSpan = 5;
let modelScale = 1;
let occupancyMap = null, occupancyRaster = null, occupancyValues = null, mapRevision = -1;
let selectedNavGoal = null;
let navGoalDrag = null;
let scene3d = null, scene3dRevision = -1, sceneCenter = {x:0,y:0,z:0};
let rackView = null;
let sceneMode = "semantic", sceneAutoFit = true;
let sceneTopDown = false, sceneFollowRobot = false;
let cameraUrls = {rgb:null, mask:null}, cameraPolling = false;
let sceneYaw = -0.65, scenePitch = 0.62, sceneZoom = 74, sceneDrag = null;

function authHeaders() {
  const token = sessionStorage.getItem("mobileBaseToken") || "";
  return {"Content-Type": "application/json", ...(token ? {Authorization: `Bearer ${token}`} : {})};
}
async function api(path, method="GET", body=null) {
  const response = await fetch(path, {method, headers: authHeaders(), body: body ? JSON.stringify(body) : null, cache:"no-store"});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}
function number(value, digits=2) { return Number.isFinite(value) ? value.toFixed(digits) : "—"; }
function setConnection(online, text) { $("serverDot").className=`dot ${online?"online":"offline"}`; $("serverText").textContent=text; }
function limits() { return config?.safety || {max_linear_x:0, max_linear_y:0, max_angular_z:0}; }
function currentCommand() {
  const max = limits(),scale=Number($("motionSpeed").value)/100;
  return {vx:-translation.y*max.max_linear_x*scale, vy:-translation.x*max.max_linear_y*scale, wz:Number($("rotation").value)/100*max.max_angular_z*scale};
}
function renderCommand() {
  const c=currentCommand(); $("vx").textContent=number(c.vx); $("vy").textContent=number(c.vy); $("wz").textContent=number(c.wz); $("rotationValue").value=number(c.wz); $("motionSpeedValue").value=`${$("motionSpeed").value}%`;
}
function clearMotionPreset() { document.querySelectorAll(".motion-preset").forEach(button=>button.classList.remove("active"));activeMotionPreset=null; }
function resetMotion() { clearMotionPreset();translation={x:0,y:0}; $("stick").style.transform="translate(0px,0px)"; $("rotation").value=0; renderCommand(); }
async function sendCommand(enabled=deadman) {
  try { await api("/api/control/cmd_vel","POST",{deadman:enabled,...currentCommand()}); }
  catch(error) { endDeadman(); alert(error.message); }
}
async function beginDeadman(event) { event?.preventDefault(); if(deadman)return; try{await api("/api/navigation/cancel","POST",{})}catch(error){alert(error.message);return}deadman=true; $("deadman").classList.add("held"); $("deadman").textContent="点击停止整车控制"; sendCommand(); clearInterval(commandTimer); commandTimer=setInterval(()=>sendCommand(),100); }
function endDeadman() { clearMotionPreset();if(!deadman)return; deadman=false; clearInterval(commandTimer); commandTimer=null; $("deadman").classList.remove("held"); $("deadman").textContent="点击启用整车控制"; sendCommand(false); }
function toggleDeadman(event) { deadman ? endDeadman() : beginDeadman(event); }
function updateControls() { const mode=latestStatus?.mode||"DISABLED",normal=mode==="NORMAL"; $("deadman").disabled=!normal;document.querySelectorAll(".motion-preset").forEach(button=>button.disabled=!normal); $("maintenanceDeadman").disabled=mode!=="MAINTENANCE"; $("controlBadge").className=`badge ${normal?"active":""}`; $("controlBadge").textContent=mode; }

function updateVisualization(status) {
  const hasMapPose=Boolean(occupancyMap&&status.map_pose?.online&&status.map_pose.value),frame=hasMapPose?"map":"odom",odom=(hasMapPose?status.map_pose.value:status.odometry?.value)||{}, valid=Number.isFinite(odom.x)&&Number.isFinite(odom.y)&&Number.isFinite(odom.yaw);
  if(valid){
    if(trailFrame!==frame){odomTrail=[];trailFrame=frame}
    const last=odomTrail.at(-1);
    if(!last||Math.hypot(odom.x-last.x,odom.y-last.y)>.01||Math.abs(odom.yaw-last.yaw)>.02){odomTrail.push({x:odom.x,y:odom.y,yaw:odom.yaw});if(odomTrail.length>3000)odomTrail.shift()}
    $("mapPose").innerHTML=`<span>X ${metricCoordinateLabel(odom.x)}</span><span>Y ${metricCoordinateLabel(odom.y)}</span><span>航向 ${number(odom.yaw*180/Math.PI,1)}°</span>`;
  }
  const labels={front_left:"左前轮",front_right:"右前轮",rear_left:"左后轮",rear_right:"右后轮"},joints=status.joint_states?.joints||{};
  $("modelWheelData").innerHTML=wheelOrder.map(name=>{const steer=joints[`${name}_steer_joint`],drive=joints[`${name}_drive_joint`],angle=steer?.position*180/Math.PI,velocity=Number(drive?.velocity),rpm=velocity*60/(2*Math.PI),rolling=!Number.isFinite(velocity)||Math.abs(velocity)<.02?"停止":velocity>0?"正向滚动":"反向滚动",mapping=Number(chassis?.wheels?.[name]?.drive?.direction)||1;return `<div><span>${labels[name]} · 驱动映射 ${mapping>0?"+1":"−1"}</span><b>${number(angle,1)}° · ${number(rpm,0)} rpm · ${rolling}</b></div>`}).join("");
  const nav=status.navigation||{state:"IDLE",message:"等待 Nav2"};$("navState").textContent=nav.message||nav.state;$("navDetail").textContent=Number.isFinite(nav.distance_remaining)?`剩余 ${number(nav.distance_remaining,2)} m · ${number(nav.navigation_time,1)} s`:(selectedNavGoal?`目标 x ${number(selectedNavGoal.x,2)} · y ${number(selectedNavGoal.y,2)}`:"目标使用 map 坐标系");
  drawMap();drawChassisTopView();if(sceneFollowRobot){const pose=currentScenePose();if(pose&&Number.isFinite(pose.x)&&Number.isFinite(pose.y)){sceneCenter={...sceneCenter,x:pose.x,y:pose.y};drawScene3d()}}
}
function decodeMapRle(encoded, expected){const data=new Int8Array(expected);let offset=0;for(let i=0;i<encoded.length;i+=2){const value=encoded[i],count=encoded[i+1];data.fill(value,offset,Math.min(expected,offset+count));offset+=count}if(offset!==expected)throw new Error("地图压缩数据长度不匹配");return data}
function buildOccupancyRaster(map){const raster=document.createElement("canvas");raster.width=map.width;raster.height=map.height;const ctx=raster.getContext("2d"),image=ctx.createImageData(map.width,map.height),data=decodeMapRle(map.data_rle,map.width*map.height);occupancyValues=data;for(let i=0;i<data.length;i++){const p=i*4,value=data[i];if(value<0){image.data[p]=44;image.data[p+1]=58;image.data[p+2]=53;image.data[p+3]=145}else if(value>=65){image.data[p]=210;image.data[p+1]=226;image.data[p+2]=220;image.data[p+3]=235}else if(value>20){image.data[p]=112;image.data[p+1]=142;image.data[p+2]=130;image.data[p+3]=130}else{image.data[p]=9;image.data[p+1]=19;image.data[p+2]=16;image.data[p+3]=105}}ctx.putImageData(image,0,0);return raster}
async function pollMap(){try{const result=await api(`/api/map?revision=${mapRevision}`);const badge=$("mapSourceBadge");if(!result.available){badge.className="badge map-source";badge.textContent="仅里程计";occupancyMap=null;occupancyRaster=null;occupancyValues=null;mapRevision=-1;drawMap();return}badge.className="badge map-source active";badge.textContent=`SLAM · ${number(result.age,1)}s`;if(!result.unchanged){occupancyMap=result;mapRevision=result.revision;occupancyRaster=buildOccupancyRaster(result)}drawMap()}catch(error){$("mapSourceBadge").className="badge map-source";$("mapSourceBadge").textContent="地图离线"}}
async function pollScene3d(){if(sceneMode!=="nvblox")return;try{const result=await api(`/api/scene3d?revision=${scene3dRevision}`);if(!result.available){$("scene3dStatus").textContent="等待 Nvblox Mesh · 请启用 nvblox";drawScene3d();return}if(!result.unchanged){scene3d=result;scene3dRevision=result.revision;const p=result.points;if(p.length&&!sceneFollowRobot&&sceneAutoFit){let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity,minZ=Infinity,maxZ=-Infinity;for(let i=0;i<p.length;i+=6){minX=Math.min(minX,p[i]);maxX=Math.max(maxX,p[i]);minY=Math.min(minY,p[i+1]);maxY=Math.max(maxY,p[i+1]);minZ=Math.min(minZ,p[i+2]);maxZ=Math.max(maxZ,p[i+2])}sceneCenter={x:(minX+maxX)/2,y:(minY+maxY)/2,z:(minZ+maxZ)/2}}}$("scene3dStatus").textContent=`Nvblox · ${result.block_count??scene3d?.block_count} 块 · ${result.sample_count??scene3d?.sample_count} 点 · ${number(result.age,1)}s`;drawScene3d()}catch(error){$("scene3dStatus").textContent="Nvblox 三维数据离线"}}
function fitSemanticScene(){if(sceneMode!=="semantic")return;const racks=rackView?.scene_racks||[],points=racks.flatMap(r=>r.layers?.flatMap(l=>l.boundary||[])||[]).filter(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite));const layers=racks.reduce((n,r)=>n+(r.layers?.length||0),0);$("scene3dStatus").textContent=racks.length?`层架 ${racks.length} 个 · ${layers} 层 · ${rackView.scene_frame||"odom"}`:"等待层架观测 · 需 RGB-D、分割和位姿";if(!sceneAutoFit||sceneFollowRobot||!points.length)return;let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity,minZ=Infinity,maxZ=-Infinity;for(const p of points){minX=Math.min(minX,p[0]);maxX=Math.max(maxX,p[0]);minY=Math.min(minY,p[1]);maxY=Math.max(maxY,p[1]);minZ=Math.min(minZ,p[2]);maxZ=Math.max(maxZ,p[2])}sceneCenter={x:(minX+maxX)/2,y:(minY+maxY)/2,z:(minZ+maxZ)/2};const canvas=$("scene3dCanvas"),width=canvas.clientWidth||480,height=canvas.clientHeight||360,span=Math.max(maxX-minX,maxY-minY,1),vertical=Math.max(maxZ-minZ,1);sceneZoom=Math.max(8,Math.min(110,Math.min(width/(span*1.6),height/(vertical*2.5))))}
async function pollRacks(){try{rackView=await api("/api/racks");const count=rackView.racks?.length||0,levels=rackView.racks?.reduce((n,r)=>n+r.layers.length,0)||0,badge=$("rackSourceBadge");badge.className=`badge map-source ${rackView.available?"active":""}`;badge.textContent=rackView.available?`层架 ${count} · ${levels} 层`:'等待 /rack_map/model';$("rackList").innerHTML=count?rackView.racks.map(r=>`<div class="rack-list-row"><strong>${escapeHtml(r.stable_id)}</strong><span>${r.layers.map(l=>`<span class="rack-layer ${l.observed?"":"estimated"}">${l.level_number} 层 · ${escapeHtml(l.stable_id)}${l.observed?"":"（遮挡）"}</span>`).join("")}</span></div>`).join(""):'<span class="rack-empty">尚未观察到层架</span>';fitSemanticScene();drawMap();drawScene3d()}catch(error){rackView=null;$("rackSourceBadge").textContent="层架数据离线";$("rackList").textContent="层架数据离线";fitSemanticScene();drawScene3d()}}
async function pollCamera(){
  if(cameraPolling||!document.querySelector('[data-page-panel="visualization"]').classList.contains("active"))return;
  cameraPolling=true;
  try{
    const results=await Promise.all(["rgb","mask"].map(async kind=>{
      const response=await fetch(`/api/camera/${kind}.jpg`,{headers:authHeaders(),cache:"no-store"});
      if(!response.ok)throw new Error(`camera ${kind}: HTTP ${response.status}`);
      return [kind,URL.createObjectURL(await response.blob())];
    }));
    for(const [kind,url] of results){const previous=cameraUrls[kind];cameraUrls[kind]=url;$(kind==="rgb"?"cameraRgb":"cameraMask").src=url;if(previous)URL.revokeObjectURL(previous)}
    $("cameraStatus").className="badge map-source active";$("cameraStatus").textContent="实时画面";
  }catch(error){$("cameraStatus").className="badge map-source";$("cameraStatus").textContent="等待相机或分割"}
  finally{cameraPolling=false}
}
function robotGeometry(){const g=chassis?.geometry||{};return{bodyLength:Number(g.body_length)||.62,bodyWidth:Number(g.body_width)||.58,padding:Number(g.footprint_padding)||.02,wheelbase:Number(g.wheelbase)||.419,trackWidth:Number(g.track_width)||.45,wheelRadius:Number(g.wheel_radius)||.0855,wheelWidth:Number(g.wheel_width)||.0683,wheelOffset:Number(g.wheel_center_offset)||.043}}
function wheelGeometry(name,g,steer=0){const front=name.startsWith("front"),left=name.endsWith("left"),pivotX=(front?1:-1)*g.wheelbase/2,pivotY=(left?1:-1)*g.trackWidth/2,offsetY=(left?1:-1)*g.wheelOffset;return{pivotX,pivotY,centerX:pivotX-Math.sin(steer)*offsetY,centerY:pivotY+Math.cos(steer)*offsetY,steer}}
function wheelCommandVector(wheel){const command=latestStatus?.last_command||{},vx=Number(command.vx)||0,vy=Number(command.vy)||0,wz=Number(command.wz)||0;return{x:vx-wz*wheel.pivotY,y:vy+wz*wheel.pivotX}}
function drawVectorArrow(ctx,start,end,color,dashed=false){const dx=end.x-start.x,dy=end.y-start.y,length=Math.hypot(dx,dy);if(length<2)return;const ux=dx/length,uy=dy/length,head=Math.min(8,Math.max(5,length*.28));ctx.save();if(dashed)ctx.setLineDash([5,4]);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(start.x,start.y);ctx.lineTo(end.x,end.y);ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.moveTo(end.x,end.y);ctx.lineTo(end.x-ux*head-uy*head*.55,end.y-uy*head+ux*head*.55);ctx.lineTo(end.x-ux*head+uy*head*.55,end.y-uy*head-ux*head*.55);ctx.closePath();ctx.fill();ctx.restore()}
function drawRobotFootprint(ctx,pose,originX,originY,ppm,goal=false){
  const g=robotGeometry(),joints=latestStatus?.joint_states?.joints||{};ctx.save();ctx.translate(originX+pose.x*ppm,originY-pose.y*ppm);ctx.rotate(-pose.yaw);
  ctx.setLineDash([6,4]);ctx.lineWidth=goal?1.5:2;ctx.strokeStyle=goal?"#62a8ffbb":"#ffd18f";ctx.strokeRect(-(g.bodyLength/2+g.padding)*ppm,-(g.bodyWidth/2+g.padding)*ppm,(g.bodyLength+2*g.padding)*ppm,(g.bodyWidth+2*g.padding)*ppm);ctx.setLineDash([]);
  ctx.fillStyle=goal?"#62a8ff24":"#ffb45e80";ctx.strokeStyle=goal?"#62a8ff":"#ffe0b8";ctx.lineWidth=1.5;ctx.fillRect(-g.bodyLength*ppm/2,-g.bodyWidth*ppm/2,g.bodyLength*ppm,g.bodyWidth*ppm);ctx.strokeRect(-g.bodyLength*ppm/2,-g.bodyWidth*ppm/2,g.bodyLength*ppm,g.bodyWidth*ppm);
  if(!goal){for(const name of wheelOrder){const steer=joints[`${name}_steer_joint`]?.position||0,wheel=wheelGeometry(name,g,steer);ctx.save();ctx.translate(wheel.centerX*ppm,-wheel.centerY*ppm);ctx.rotate(-steer);ctx.fillStyle="#17201d";ctx.strokeStyle="#91a69d";ctx.lineWidth=1;ctx.fillRect(-g.wheelRadius*ppm,-g.wheelWidth*ppm/2,g.wheelRadius*2*ppm,g.wheelWidth*ppm);ctx.strokeRect(-g.wheelRadius*ppm,-g.wheelWidth*ppm/2,g.wheelRadius*2*ppm,g.wheelWidth*ppm);ctx.restore()}}
  ctx.fillStyle=goal?"#62a8ff":"#fff1d7";ctx.beginPath();ctx.moveTo(g.bodyLength*ppm/2+5,0);ctx.lineTo(g.bodyLength*ppm/2-3,-5);ctx.lineTo(g.bodyLength*ppm/2-3,5);ctx.closePath();ctx.fill();
  if(!goal&&ppm>38){ctx.fillStyle="#fff1d7";ctx.font="bold 9px system-ui";ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillText(`${Math.round(g.bodyLength*100)}×${Math.round(g.bodyWidth*100)}cm`,0,0)}ctx.restore();
}
function mapViewMetrics(w,h){
  const mapPose=latestStatus?.map_pose?.online?latestStatus.map_pose.value:null,odomPose=latestStatus?.odometry?.value||null;
  const pose=occupancyMap?mapPose:odomPose,valid=pose&&Number.isFinite(pose.x)&&Number.isFinite(pose.y);
  const o=occupancyMap?.origin,halfX=occupancyMap?occupancyMap.width*occupancyMap.resolution/2:0,halfY=occupancyMap?occupancyMap.height*occupancyMap.resolution/2:0,c=Math.cos(o?.yaw||0),s=Math.sin(o?.yaw||0);
  const center=valid?pose:(occupancyMap?{x:o.x+halfX*c-halfY*s,y:o.y+halfX*s+halfY*c}:(odomTrail.at(-1)||{x:0,y:0}));
  const ppm=Math.min(w,h)/mapSpan;return{pose:valid?pose:null,ppm,cx:w/2-center.x*ppm,cy:h/2+center.y*ppm,center,frame:occupancyMap?"map":"odom"}
}
function occupancyAtWorld(x,y){
  if(!occupancyMap||!occupancyValues)return null;
  const origin=occupancyMap.origin,angle=origin.yaw||0,c=Math.cos(angle),s=Math.sin(angle),dx=x-origin.x,dy=y-origin.y;
  const ix=Math.floor((dx*c+dy*s)/occupancyMap.resolution),iy=Math.floor((-dx*s+dy*c)/occupancyMap.resolution);
  return ix>=0&&iy>=0&&ix<occupancyMap.width&&iy<occupancyMap.height?occupancyValues[iy*occupancyMap.width+ix]:null
}
function drawOccupancyCellGrid(ctx,cx,cy,ppm,w,h){
  if(!occupancyMap||ppm*occupancyMap.resolution<10)return;
  const map=occupancyMap,o=map.origin,angle=o.yaw||0,c=Math.cos(angle),s=Math.sin(angle),corners=[[0,0],[w,0],[0,h],[w,h]].map(([px,py])=>{const dx=(px-cx)/ppm-o.x,dy=(cy-py)/ppm-o.y;return[(dx*c+dy*s)/map.resolution,(-dx*s+dy*c)/map.resolution]}),xs=corners.map(p=>p[0]),ys=corners.map(p=>p[1]);
  const x0=Math.max(0,Math.floor(Math.min(...xs))),x1=Math.min(map.width,Math.ceil(Math.max(...xs))),y0=Math.max(0,Math.floor(Math.min(...ys))),y1=Math.min(map.height,Math.ceil(Math.max(...ys)));
  ctx.save();ctx.translate(cx,cy);ctx.scale(ppm,-ppm);ctx.translate(o.x,o.y);ctx.rotate(angle);ctx.scale(map.resolution,map.resolution);ctx.strokeStyle="#b9d3c333";ctx.lineWidth=.7/(ppm*map.resolution);ctx.beginPath();for(let x=x0;x<=x1;x++){ctx.moveTo(x,y0);ctx.lineTo(x,y1)}for(let y=y0;y<=y1;y++){ctx.moveTo(x0,y);ctx.lineTo(x1,y)}ctx.stroke();ctx.restore()
}
function drawMap(){
  const canvas=$("mapCanvas"),rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return;const dpr=Math.min(window.devicePixelRatio||1,2),w=Math.round(rect.width),h=Math.round(rect.height);if(canvas.width!==w*dpr||canvas.height!==h*dpr){canvas.width=w*dpr;canvas.height=h*dpr}const ctx=canvas.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.fillStyle="#07100e";ctx.fillRect(0,0,w,h);
  const {pose:current,ppm,cx,cy,center,frame}=mapViewMetrics(w,h),grid=metricTickStep(ppm),labelEvery=Math.max(1,Math.ceil(80/(grid*ppm)));
  if(occupancyMap&&occupancyRaster){ctx.save();ctx.translate(cx,cy);ctx.scale(ppm,-ppm);ctx.translate(occupancyMap.origin.x,occupancyMap.origin.y);ctx.rotate(occupancyMap.origin.yaw||0);ctx.scale(occupancyMap.resolution,occupancyMap.resolution);ctx.imageSmoothingEnabled=false;ctx.drawImage(occupancyRaster,0,0);ctx.restore()}
  drawOccupancyCellGrid(ctx,cx,cy,ppm,w,h);
  ctx.fillStyle="#07130fe0";ctx.fillRect(0,0,w,28);ctx.fillRect(0,28,66,h-28);
  ctx.lineWidth=1;ctx.font="10px system-ui";ctx.textBaseline="middle";for(let k=Math.floor((-cx/ppm)/grid);k<=Math.ceil(((w-cx)/ppm)/grid);k++){const px=cx+k*grid*ppm;if(px<0||px>w)continue;ctx.strokeStyle=k===0?"#bd8251":"#42635399";ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,h);ctx.stroke();ctx.beginPath();ctx.moveTo(px,21);ctx.lineTo(px,28);ctx.stroke();if(k%labelEvery===0&&px>65&&px<w-65){ctx.fillStyle="#e3c39d";ctx.textAlign="center";ctx.fillText(metricCoordinateLabel(k*grid),px,14)}}for(let k=Math.floor(((cy-h)/ppm)/grid);k<=Math.ceil((cy/ppm)/grid);k++){const py=cy-k*grid*ppm;if(py<0||py>h)continue;ctx.strokeStyle=k===0?"#5d9ccc":"#42635399";ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(w,py);ctx.stroke();ctx.beginPath();ctx.moveTo(58,py);ctx.lineTo(66,py);ctx.stroke();if(k%labelEvery===0&&py>38&&py<h-50){ctx.fillStyle="#b4dcfa";ctx.textAlign="right";ctx.fillText(metricCoordinateLabel(k*grid),54,py)}}
  ctx.font="bold 10px system-ui";ctx.fillStyle="#e3c39d";ctx.textAlign="left";ctx.fillText("X / cm →",w-62,14);ctx.fillStyle="#b4dcfa";ctx.fillText("Y / cm ↑",6,39);
  if(odomTrail.length>1&&trailFrame===frame){ctx.strokeStyle="#77e2a8";ctx.lineWidth=2;ctx.shadowColor="#77e2a8";ctx.shadowBlur=7;ctx.beginPath();odomTrail.forEach((p,i)=>{const x=cx+p.x*ppm,y=cy-p.y*ppm;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.shadowBlur=0}
  if(rackView?.map_frame===frame&&rackView.map_racks?.length){ctx.font="bold 11px system-ui";ctx.textBaseline="middle";for(const rack of rackView.map_racks){const outline=rack.layers?.filter(layer=>layer.boundary?.length===4).sort((a,b)=>(b.observed_ratio||0)-(a.observed_ratio||0))[0];if(outline){ctx.save();ctx.beginPath();outline.boundary.forEach((p,i)=>{const x=cx+p[0]*ppm,y=cy-p[1]*ppm;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.closePath();ctx.fillStyle="#54c99225";ctx.strokeStyle=outline.observed?"#7be2a5":"#8ba7af";ctx.lineWidth=2;if(!outline.observed)ctx.setLineDash([5,4]);ctx.fill();ctx.stroke();ctx.restore()}const [rx,ry]=rack.position,x=cx+rx*ppm,y=cy-ry*ppm;if(x<0||x>w||y<0||y>h)continue;const label=`${rack.stable_id} · ${rack.layers.length}层`,width=ctx.measureText(label).width+12;ctx.fillStyle="#10281fe6";ctx.strokeStyle="#7be2a5";ctx.lineWidth=1;ctx.fillRect(x-width/2,y-13,width,23);ctx.strokeRect(x-width/2,y-13,width,23);ctx.fillStyle="#dcffe9";ctx.textAlign="center";ctx.fillText(label,x,y-1)}}
  if(current)drawRobotFootprint(ctx,current,cx,cy,ppm,false);
  if(selectedNavGoal&&frame==="map")drawRobotFootprint(ctx,selectedNavGoal,cx,cy,ppm,true);
  $("mapScale").textContent=`视野 ${metricDistanceLabel(mapSpan)}`;
  $("mapReference").textContent=`${frame} · 中心 X ${metricCoordinateLabel(center.x)}、Y ${metricCoordinateLabel(center.y)} · 网格 ${metricDistanceLabel(grid)}${occupancyMap?` · 栅格 ${metricDistanceLabel(occupancyMap.resolution)}/像素`:""}`;
}
function drawChassisTopView(){
  const canvas=$("vehicleCanvas");if(!canvas)return;const rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return;
  const dpr=Math.min(window.devicePixelRatio||1,2),w=Math.round(rect.width),h=Math.round(rect.height);if(canvas.width!==w*dpr||canvas.height!==h*dpr){canvas.width=w*dpr;canvas.height=h*dpr}
  const ctx=canvas.getContext("2d"),g=robotGeometry(),joints=latestStatus?.joint_states?.joints||{},extentX=Math.max(g.bodyLength/2+g.padding,g.wheelbase/2+g.wheelOffset+g.wheelRadius),extentY=Math.max(g.bodyWidth/2+g.padding,g.trackWidth/2+g.wheelOffset+g.wheelRadius),ppm=Math.min((w-80)/(2*extentY),(h-80)/(2*extentX))*modelScale,cx=w/2,cy=h/2;
  const point=(x,y)=>({x:cx-y*ppm,y:cy-x*ppm});ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);const grad=ctx.createRadialGradient(cx,cy,10,cx,cy,Math.max(w,h)*.7);grad.addColorStop(0,"#173029");grad.addColorStop(1,"#050b09");ctx.fillStyle=grad;ctx.fillRect(0,0,w,h);
  ctx.lineWidth=1;ctx.strokeStyle="#1d342c";const grid=.1;for(let x=-Math.ceil(extentX/grid)*grid;x<=extentX+1e-9;x+=grid){const a=point(x,-extentY),b=point(x,extentY);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()}for(let y=-Math.ceil(extentY/grid)*grid;y<=extentY+1e-9;y+=grid){const a=point(-extentX,y),b=point(extentX,y);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()}
  const body=[point(g.bodyLength/2,g.bodyWidth/2),point(g.bodyLength/2,-g.bodyWidth/2),point(-g.bodyLength/2,-g.bodyWidth/2),point(-g.bodyLength/2,g.bodyWidth/2)];ctx.beginPath();body.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fillStyle="#286d50cc";ctx.strokeStyle="#9ff0c1";ctx.lineWidth=2;ctx.fill();ctx.stroke();
  ctx.save();ctx.setLineDash([6,4]);ctx.strokeStyle="#ffd18f88";const footprint=[point(g.bodyLength/2+g.padding,g.bodyWidth/2+g.padding),point(g.bodyLength/2+g.padding,-g.bodyWidth/2-g.padding),point(-g.bodyLength/2-g.padding,-g.bodyWidth/2-g.padding),point(-g.bodyLength/2-g.padding,g.bodyWidth/2+g.padding)];ctx.beginPath();footprint.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.stroke();ctx.restore();
  const labels={front_left:"左前",front_right:"右前",rear_left:"左后",rear_right:"右后"};for(const name of wheelOrder){const steer=Number(joints[`${name}_steer_joint`]?.position)||0,driveVelocity=Number(joints[`${name}_drive_joint`]?.velocity)||0,wheel=wheelGeometry(name,g,steer),pivot=point(wheel.pivotX,wheel.pivotY),center=point(wheel.centerX,wheel.centerY),axis={x:Math.cos(steer),y:Math.sin(steer)},side={x:-Math.sin(steer),y:Math.cos(steer)},corners=[];for(const along of [-g.wheelRadius,g.wheelRadius])for(const across of [-g.wheelWidth/2,g.wheelWidth/2])corners.push(point(wheel.centerX+axis.x*along+side.x*across,wheel.centerY+axis.y*along+side.y*across));
    ctx.save();ctx.setLineDash([4,4]);ctx.strokeStyle="#77e2a833";ctx.beginPath();ctx.arc(pivot.x,pivot.y,(g.wheelOffset+g.wheelRadius)*ppm,0,Math.PI*2);ctx.stroke();ctx.restore();ctx.strokeStyle="#ffb45e";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(pivot.x,pivot.y);ctx.lineTo(center.x,center.y);ctx.stroke();ctx.beginPath();ctx.arc(pivot.x,pivot.y,4,0,Math.PI*2);ctx.fillStyle="#ffcf8f";ctx.fill();
    const order=[corners[0],corners[1],corners[3],corners[2]];ctx.beginPath();order.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fillStyle="#151d1a";ctx.strokeStyle="#b3c5bd";ctx.lineWidth=1.5;ctx.fill();ctx.stroke();const axisA=point(wheel.centerX-axis.x*g.wheelRadius,wheel.centerY-axis.y*g.wheelRadius),axisB=point(wheel.centerX+axis.x*g.wheelRadius,wheel.centerY+axis.y*g.wheelRadius);ctx.strokeStyle="#9eb4ab";ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(axisA.x,axisA.y);ctx.lineTo(axisB.x,axisB.y);ctx.stroke();
    const commanded=wheelCommandVector(wheel),commandMagnitude=Math.hypot(commanded.x,commanded.y);if(commandMagnitude>.001){const commandLength=g.wheelRadius*1.45,commandTip=point(wheel.centerX+commanded.x/commandMagnitude*commandLength,wheel.centerY+commanded.y/commandMagnitude*commandLength);drawVectorArrow(ctx,center,commandTip,"#ffb45e",true)}
    if(Math.abs(driveVelocity)>.02){const actualSign=Math.sign(driveVelocity),actualLength=g.wheelRadius*1.25,actualTip=point(wheel.centerX+axis.x*actualSign*actualLength,wheel.centerY+axis.y*actualSign*actualLength);drawVectorArrow(ctx,center,actualTip,"#55c8ff")}
    const mapping=Number(chassis?.wheels?.[name]?.drive?.direction)||1,rpm=driveVelocity*60/(2*Math.PI);ctx.fillStyle="#eaf8f2";ctx.font="11px system-ui";ctx.textAlign="center";ctx.fillText(`${labels[name]} ${number(steer*180/Math.PI,1)}°`,center.x,center.y-12);ctx.fillStyle="#9eb4ab";ctx.font="9px system-ui";ctx.fillText(`${number(rpm,0)} rpm · D${mapping>0?"+1":"−1"}`,center.x,center.y+17)}
  const front=point(g.bodyLength/2,0),arrow=point(g.bodyLength/2+.08,0);ctx.strokeStyle="#fff1d7";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(front.x,front.y);ctx.lineTo(arrow.x,arrow.y);ctx.stroke();ctx.fillStyle="#fff1d7";ctx.beginPath();ctx.moveTo(arrow.x,arrow.y-7);ctx.lineTo(arrow.x-5,arrow.y+2);ctx.lineTo(arrow.x+5,arrow.y+2);ctx.closePath();ctx.fill();ctx.font="bold 11px system-ui";ctx.textAlign="left";ctx.fillText(`车体 ${g.bodyLength.toFixed(3)} × ${g.bodyWidth.toFixed(3)} m`,14,22);ctx.fillStyle="#9eb4ab";ctx.font="10px system-ui";ctx.fillText(`轴距 ${g.wheelbase.toFixed(3)} · 轮距 ${g.trackWidth.toFixed(3)} · 轮心偏置 ${g.wheelOffset.toFixed(3)} m`,14,39)
}
function sceneProjector(w,h){if(sceneTopDown)return(x,y,z)=>({x:w/2+(x-sceneCenter.x)*sceneZoom,y:h/2-(y-sceneCenter.y)*sceneZoom,d:-z,p:1});const cy=Math.cos(sceneYaw),sy=Math.sin(sceneYaw),cp=Math.cos(scenePitch),sp=Math.sin(scenePitch);return(xw,yw,zw)=>{const dx=xw-sceneCenter.x,dy=yw-sceneCenter.y,dz=zw-sceneCenter.z,x=dx*cy-dy*sy,y=dx*sy+dy*cy,depth=y*cp+dz*sp,vertical=dz*cp-y*sp,perspective=Math.max(.55,Math.min(1.7,1/(1+depth*.055)));return{x:w/2+x*sceneZoom*perspective,y:h*.53-vertical*sceneZoom*perspective,d:depth,p:perspective}}}
function currentScenePose(){const frame=(sceneMode==="semantic"?rackView?.scene_frame:scene3d?.frame_id)||"odom";return(frame.includes("map")?latestStatus?.map_pose?.value:latestStatus?.odometry?.value)||null}
function metricTickStep(zoom){return zoom>=160?.1:zoom>=90?.25:zoom>=40?.5:zoom>=20?1:2}
function metricDistanceLabel(meters){return `${Math.round(meters*100)} cm`}
function metricCoordinateLabel(value){return `${Math.round(value*100)} cm`}
function drawWorldGrid(ctx,project,w,h){
  const span=Math.max(w,h)/Math.max(sceneZoom,8)*1.4,step=metricTickStep(sceneZoom),
    firstX=Math.floor((sceneCenter.x-span)/step),lastX=Math.ceil((sceneCenter.x+span)/step),
    firstY=Math.floor((sceneCenter.y-span)/step),lastY=Math.ceil((sceneCenter.y+span)/step);
  ctx.save();ctx.lineWidth=1;
  for(let k=firstX;k<=lastX;k++){const x=k*step,a=project(x,firstY*step,0),b=project(x,lastY*step,0);ctx.strokeStyle=k===0?"#e9a45b":"#2a4b3d99";ctx.lineWidth=k===0?2:1;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()}
  for(let k=firstY;k<=lastY;k++){const y=k*step,a=project(firstX*step,y,0),b=project(lastX*step,y,0);ctx.strokeStyle=k===0?"#69b9fa":"#2a4b3d99";ctx.lineWidth=k===0?2:1;ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke()}
  const origin=project(0,0,0);if(origin.x>-20&&origin.x<w+20&&origin.y>-20&&origin.y<h+20){ctx.fillStyle="#ffffff";ctx.beginPath();ctx.arc(origin.x,origin.y,4,0,Math.PI*2);ctx.fill();ctx.font="bold 11px system-ui";ctx.fillText("O (0,0)",origin.x+7,origin.y-7)}ctx.restore()
}
function drawMetricRulers(ctx,project,w,h,step){
  ctx.save();ctx.font="10px system-ui";ctx.textBaseline="middle";
  if(sceneTopDown){
    const bottom=h-125,left=70,halfX=w/(2*sceneZoom),halfY=h/(2*sceneZoom),labelEvery=Math.max(1,Math.ceil(75/(step*sceneZoom)));
    ctx.fillStyle="#07130fdc";ctx.fillRect(0,bottom-7,w,39);ctx.fillRect(0,42,96,h-160);
    ctx.strokeStyle="#ffb45e";ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(0,bottom);ctx.lineTo(w,bottom);ctx.stroke();
    ctx.strokeStyle="#69b9fa";ctx.beginPath();ctx.moveTo(left,42);ctx.lineTo(left,h-112);ctx.stroke();
    for(let k=Math.floor((sceneCenter.x-halfX)/step);k<=Math.ceil((sceneCenter.x+halfX)/step);k++){const value=k*step,x=project(value,sceneCenter.y,0).x;if(x<105||x>w-25)continue;ctx.strokeStyle="#ffb45e";ctx.beginPath();ctx.moveTo(x,bottom);ctx.lineTo(x,bottom+6);ctx.stroke();if(k%labelEvery===0){ctx.fillStyle="#ffe0ba";ctx.textAlign="center";ctx.fillText(metricCoordinateLabel(value),x,bottom+17)}}
    for(let k=Math.floor((sceneCenter.y-halfY)/step);k<=Math.ceil((sceneCenter.y+halfY)/step);k++){const value=k*step,y=project(sceneCenter.x,value,0).y;if(y<55||y>h-145)continue;ctx.strokeStyle="#69b9fa";ctx.beginPath();ctx.moveTo(left-6,y);ctx.lineTo(left,y);ctx.stroke();if(k%labelEvery===0){ctx.fillStyle="#c0e4ff";ctx.textAlign="right";ctx.fillText(metricCoordinateLabel(value),left-9,y)}}
    ctx.textAlign="left";ctx.fillStyle="#ffe0ba";ctx.fillText("X 坐标 / cm →",w-112,bottom-14);ctx.fillStyle="#c0e4ff";ctx.fillText("Y 坐标 / cm ↑",9,49)
  }else{
    const zUnit=sceneZoom>=150?.1:sceneZoom>=75?.25:.5,o=project(sceneCenter.x,sceneCenter.y,0),tip=project(sceneCenter.x,sceneCenter.y,1),pixelsPerMeter=Math.hypot(tip.x-o.x,tip.y-o.y),baseX=31,baseY=h-115,maxZ=Math.min(2.5,Math.floor((h-190)/Math.max(pixelsPerMeter,1)/zUnit)*zUnit),zLabelEvery=Math.max(1,Math.ceil(26/(zUnit*pixelsPerMeter)));
    if(maxZ>=zUnit){ctx.fillStyle="#07130fdd";ctx.fillRect(4,baseY-maxZ*pixelsPerMeter-24,108,maxZ*pixelsPerMeter+43);ctx.strokeStyle="#cba9ff";ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(baseX,baseY);ctx.lineTo(baseX,baseY-maxZ*pixelsPerMeter);ctx.stroke();ctx.fillStyle="#dcc7ff";ctx.textAlign="left";ctx.fillText("Z 高度 / cm",baseX+5,baseY-maxZ*pixelsPerMeter-12);for(let k=0;k<=Math.round(maxZ/zUnit);k++){const z=k*zUnit,y=baseY-z*pixelsPerMeter;ctx.beginPath();ctx.moveTo(baseX-4,y);ctx.lineTo(baseX+4,y);ctx.stroke();if(k%zLabelEvery===0)ctx.fillText(metricCoordinateLabel(z),baseX+8,y)}}
    const span=Math.max(w,h)/Math.max(sceneZoom,8)*.8,anchorX=Math.round(sceneCenter.x/step)*step,anchorY=Math.round(sceneCenter.y/step)*step,labelEvery=Math.max(1,Math.ceil(90/(step*sceneZoom*.7)));
    for(const axis of ["X","Y"]){const isX=axis==="X",start=isX?project(sceneCenter.x-span,anchorY,0):project(anchorX,sceneCenter.y-span,0),end=isX?project(sceneCenter.x+span,anchorY,0):project(anchorX,sceneCenter.y+span,0),color=isX?"#ffb45e":"#69b9fa";ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(start.x,start.y);ctx.lineTo(end.x,end.y);ctx.stroke();const center=isX?sceneCenter.x:sceneCenter.y;for(let k=Math.floor((center-span)/step);k<=Math.ceil((center+span)/step);k++){const value=k*step,p=isX?project(value,anchorY,0):project(anchorX,value,0);if(p.x<118||p.x>w-80||p.y<78||p.y>h-130)continue;ctx.fillStyle=color;ctx.beginPath();ctx.arc(p.x,p.y,2.5,0,Math.PI*2);ctx.fill();if(k%labelEvery===0){const label=`${axis} ${metricCoordinateLabel(value)}`,width=ctx.measureText(label).width+8,x=p.x+5,y=p.y+(isX?-17:14);ctx.fillStyle="#07130fe6";ctx.fillRect(x-3,y-8,width,16);ctx.fillStyle=color;ctx.textAlign="left";ctx.fillText(label,x,y)}}}
  }
  ctx.restore()
}
function drawSceneReference(ctx,project,w,h){
  const frame=(sceneMode==="semantic"?rackView?.scene_frame:scene3d?.frame_id)||"odom",step=metricTickStep(sceneZoom),o=project(sceneCenter.x,sceneCenter.y,0);
  $("sceneReference").textContent=`${frame} · ${sceneTopDown?"正北俯视":"斜视"} · 中心 X ${metricCoordinateLabel(sceneCenter.x)}、Y ${metricCoordinateLabel(sceneCenter.y)} · 格距 ${metricDistanceLabel(step)}`;
  drawMetricRulers(ctx,project,w,h,step);
  ctx.save();ctx.font="bold 12px system-ui";ctx.textAlign="left";
  const tick=project(sceneCenter.x+step,sceneCenter.y,0),scale=Math.hypot(tick.x-o.x,tick.y-o.y);ctx.strokeStyle="#d2eee0";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(100,h-28);ctx.lineTo(100+scale,h-28);ctx.moveTo(100,h-33);ctx.lineTo(100,h-23);ctx.moveTo(100+scale,h-33);ctx.lineTo(100+scale,h-23);ctx.stroke();ctx.fillStyle="#d2eee0";ctx.font="11px system-ui";ctx.fillText(`${metricDistanceLabel(step)}（中心处）`,100,h-37);ctx.restore()
}
function boxFaces(cx,cy,cz,length,width,height,yaw,color){const c=Math.cos(yaw),s=Math.sin(yaw),corners=[];for(const z of [-height/2,height/2])for(const y of [-width/2,width/2])for(const x of [-length/2,length/2])corners.push({x:cx+x*c-y*s,y:cy+x*s+y*c,z:cz+z});return{corners,color,faces:[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]}}
function drawSceneRobot(ctx,project){
  const pose=currentScenePose();if(!pose||![pose.x,pose.y,pose.yaw].every(Number.isFinite))return;const g=robotGeometry(),joints=latestStatus?.joint_states?.joints||{},boxes=[boxFaces(pose.x,pose.y,.16,g.bodyLength,g.bodyWidth,.20,pose.yaw,{fill:"#e58d3dcc",stroke:"#ffe0b8"})];
  for(const name of wheelOrder){const steer=joints[`${name}_steer_joint`]?.position||0,wheel=wheelGeometry(name,g,steer),c=Math.cos(pose.yaw),s=Math.sin(pose.yaw),wx=pose.x+wheel.centerX*c-wheel.centerY*s,wy=pose.y+wheel.centerX*s+wheel.centerY*c;boxes.push(boxFaces(wx,wy,g.wheelRadius,g.wheelRadius*2,g.wheelWidth,g.wheelRadius*2,pose.yaw+steer,{fill:"#1c2824ee",stroke:"#a8bcb4"}))}
  const safety=[[g.bodyLength/2+g.padding,g.bodyWidth/2+g.padding],[g.bodyLength/2+g.padding,-g.bodyWidth/2-g.padding],[-g.bodyLength/2-g.padding,-g.bodyWidth/2-g.padding],[-g.bodyLength/2-g.padding,g.bodyWidth/2+g.padding]],c=Math.cos(pose.yaw),s=Math.sin(pose.yaw),outline=safety.map(([x,y])=>project(pose.x+x*c-y*s,pose.y+x*s+y*c,.015));ctx.save();ctx.setLineDash([5,4]);ctx.strokeStyle="#ffd18f";ctx.lineWidth=1.5;ctx.beginPath();outline.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.stroke();ctx.restore();
  const faces=[];for(const box of boxes){const points=box.corners.map(p=>project(p.x,p.y,p.z));for(const indexes of box.faces)faces.push({points:indexes.map(i=>points[i]),depth:indexes.reduce((sum,i)=>sum+points[i].d,0)/indexes.length,color:box.color})}faces.sort((a,b)=>b.depth-a.depth);for(const face of faces){ctx.beginPath();face.points.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.fillStyle=face.color.fill;ctx.strokeStyle=face.color.stroke;ctx.lineWidth=1;ctx.fill();ctx.stroke()}
  const front=project(pose.x+Math.cos(pose.yaw)*g.bodyLength/2,pose.y+Math.sin(pose.yaw)*g.bodyLength/2,.29),center=project(pose.x,pose.y,.29);ctx.strokeStyle="#fff1d7";ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(center.x,center.y);ctx.lineTo(front.x,front.y);ctx.stroke();ctx.fillStyle="#fff1d7";ctx.font="bold 10px system-ui";ctx.textAlign="center";ctx.fillText(`车体 ${g.bodyLength.toFixed(2)} × ${g.bodyWidth.toFixed(2)} m`,center.x,center.y-10);
}
function drawScene3d(){
  const canvas=$("scene3dCanvas"),rect=canvas.getBoundingClientRect();if(!rect.width||!rect.height)return;const dpr=Math.min(window.devicePixelRatio||1,2),w=Math.round(rect.width),h=Math.round(rect.height);if(canvas.width!==w*dpr||canvas.height!==h*dpr){canvas.width=w*dpr;canvas.height=h*dpr}const ctx=canvas.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);const grad=ctx.createRadialGradient(w*.5,h*.42,10,w*.5,h*.5,Math.max(w,h)*.65);grad.addColorStop(0,"#173029");grad.addColorStop(1,"#050b09");ctx.fillStyle=grad;ctx.fillRect(0,0,w,h);
  const project=sceneProjector(w,h);
  drawWorldGrid(ctx,project,w,h);
  if(sceneMode==="semantic"){
    const racks=rackView?.scene_racks||[];
    if(racks.length)drawSemanticRacks(ctx,project,racks,sceneZoom,sceneTopDown);
    else{ctx.fillStyle="#8aa99b";ctx.font="13px system-ui";ctx.textAlign="center";ctx.fillText("等待层架观测；模型由分割、深度和位姿生成",w/2,h/2)}
    drawSceneRobot(ctx,project);drawSceneReference(ctx,project,w,h);return;
  }
  if(!scene3d?.points?.length){ctx.fillStyle="#71877e";ctx.font="13px system-ui";ctx.textAlign="center";ctx.fillText("启用 Nvblox 后显示实时三维环境结构",w/2,h/2);drawSceneRobot(ctx,project);drawSceneReference(ctx,project,w,h);return}
  const p=scene3d.points,projected=[];for(let i=0;i<p.length;i+=6){const point=project(p[i],p[i+1],p[i+2]);projected.push({...point,r:p[i+3],g:p[i+4],b:p[i+5],s:Math.max(1,2.2*point.p)})}projected.sort((a,b)=>b.d-a.d);for(const point of projected){if(point.x<0||point.x>w||point.y<0||point.y>h)continue;ctx.fillStyle=`rgb(${point.r} ${point.g} ${point.b})`;ctx.globalAlpha=.72;ctx.fillRect(point.x,point.y,point.s,point.s)}ctx.globalAlpha=1;drawSceneRobot(ctx,project);
  if(rackView?.scene_frame&&rackView.scene_racks?.length){ctx.textAlign="center";ctx.textBaseline="middle";ctx.font="bold 10px system-ui";for(const rack of rackView.scene_racks){for(const layer of rack.layers){const point=project(...layer.center);if(point.x<0||point.x>w||point.y<0||point.y>h)continue;const label=`${rack.stable_id} / ${layer.level_number}层 ${layer.stable_id}`,width=ctx.measureText(label).width+10;ctx.fillStyle=layer.observed?"#10281feb":"#27312ce6";ctx.fillRect(point.x-width/2,point.y-12,width,20);ctx.strokeStyle=layer.observed?"#7be2a5":"#a1aba4";ctx.strokeRect(point.x-width/2,point.y-12,width,20);ctx.fillStyle=layer.observed?"#e2ffeb":"#c3cac6";ctx.fillText(label,point.x,point.y-2)}}}
  drawSceneReference(ctx,project,w,h);
}
function applyModelView(){drawChassisTopView()}
function showPage(name,pushHash=true){if(!document.querySelector(`[data-page-panel="${name}"]`))name="overview";document.querySelectorAll("[data-page-panel]").forEach(page=>page.classList.toggle("active",page.dataset.pagePanel===name));document.querySelectorAll(".nav-item").forEach(button=>button.classList.toggle("active",button.dataset.page===name));if(pushHash&&location.hash!==`#${name}`)history.pushState(null,"",`#${name}`);if(name!=="control")endDeadman();if(name!=="maintenance")endMaintenance();if(name==="visualization")requestAnimationFrame(()=>{drawMap();drawScene3d()});window.scrollTo({top:0,behavior:"smooth"})}

async function poll() {
  try {
    const s=await api("/api/status"); latestStatus=s; setConnection(true,"服务在线");
    const cans=s.can||[], canOk=cans.length&&cans.every(c=>c.available&&c.up&&["ERROR-ACTIVE","UNKNOWN"].includes(c.can_state));
    $("canValue").textContent=canOk?"正常":cans.some(c=>c.available)?"需检查":"未发现";
    $("canDetail").textContent=cans.map(c=>`${c.interface} ${c.can_state||c.operstate||"不可用"}${c.bitrate?` · ${c.bitrate/1000}k`:""}`).join(" / ")||"未配置";
    const jointCount=Object.keys(s.joint_states.joints||{}).length; $("jointValue").textContent=s.joint_states.online?`${jointCount} 个在线`:"离线"; $("jointDetail").textContent=s.joint_states.age==null?"尚未收到消息":`${number(s.joint_states.age,1)} 秒前更新`;
    $("odomValue").textContent=s.odometry.online?"在线":"离线"; $("odomDetail").textContent=s.odometry.age==null?"尚未收到消息":`${number(s.odometry.age,1)} 秒前更新`;
    const steeringHealth=s.steering_feedback||{healthy:false,reasons:["等待状态"]},controllerNames=Object.entries(s.controllers||{}).filter(([,state])=>state==="active").map(([name])=>name);
    $("diagnosticValue").textContent=steeringHealth.healthy?"正常":"联锁";$("diagnosticDetail").textContent=steeringHealth.healthy?`${s.mode} · ${controllerNames.join(", ")||"无活动控制器"}`:(steeringHealth.reasons||[]).join("；");
    $("joints").innerHTML=jointCount?Object.entries(s.joint_states.joints).map(([name,j])=>`<div class="joint"><strong>${escapeHtml(name)}</strong><span>位置 ${number(j.position,3)}</span><span>速度 ${number(j.velocity,3)}</span></div>`).join(""):"<p class='empty'>还没有收到关节反馈</p>";
    const o=s.odometry.value||{}; $("pose").innerHTML=`<span>x ${number(o.x,3)} m</span><span>y ${number(o.y,3)} m</span><span>yaw ${number(o.yaw,3)} rad</span>`;updateVisualization(s);
    $("events").innerHTML=(s.events||[]).slice().reverse().map(e=>`<div class="event ${e.level}"><time>${escapeHtml(e.time)}</time><span>${escapeHtml(e.message)}</span></div>`).join("")||"<p class='empty'>暂无事件</p>";
    $("modeBadge").textContent=s.mode||"DISABLED";
    updateControls();
    const wheel=$("maintenanceWheel").value, steer=s.joint_states.joints?.[`${wheel}_steer_joint`], drive=s.joint_states.joints?.[`${wheel}_drive_joint`];
    $("wheelFeedback").innerHTML=`<span>实际转角 ${number(steer?.position*180/Math.PI,1)}°</span><span>实际轮速 ${number(drive?.velocity*60/(2*Math.PI),1)} rpm</span><span>目标角 ${number(s.maintenance_command?.angle_deg,1)}°</span>`;
  } catch(error) { setConnection(false,"服务离线"); }
}
function escapeHtml(value){const node=document.createElement("div");node.textContent=String(value);return node.innerHTML;}
async function loadConfig(){config=await api("/api/config");for(const [key,value] of Object.entries(config.safety))if($("configForm").elements[key])$("configForm").elements[key].value=value;renderCommand();}

const wheelOrder=["front_left","front_right","rear_left","rear_right"];
function wheelCard(name,wheel){return `<article class="wheel-card" data-wheel="${name}"><h3>${escapeHtml(wheel.label)}</h3><div class="wheel-fields">
  <label>达妙 Motor ID<input name="motor_id" value="${wheel.steering.motor_id}"></label>
  <label>达妙 Master ID<input name="master_id" value="${wheel.steering.master_id}"></label>
  <label>转向方向<select name="steering_direction"><option value="1" ${wheel.steering.direction===1?"selected":""}>+1</option><option value="-1" ${wheel.steering.direction===-1?"selected":""}>-1</option></select></label>
  <label>转向零偏 rad<input name="offset_rad" type="number" step="0.001" value="${wheel.steering.offset_rad}"></label>
  <label>转向速度 rad/s<input name="max_velocity_rad_s" type="number" step="0.01" min="0.01" value="${wheel.steering.max_velocity_rad_s}"></label>
  <label>ZLAC Node ID<input name="node_id" value="${wheel.drive.node_id}"></label>
  <label>ZLAC 通道<select name="channel"><option value="1" ${wheel.drive.channel===1?"selected":""}>1</option><option value="2" ${wheel.drive.channel===2?"selected":""}>2</option></select></label>
  <label>行走方向<select name="drive_direction"><option value="1" ${wheel.drive.direction===1?"selected":""}>+1</option><option value="-1" ${wheel.drive.direction===-1?"selected":""}>-1</option></select></label>
  <label>减速比<input name="gear_ratio" type="number" step="0.01" value="${wheel.drive.gear_ratio}"></label>
  <label>最大电机 rpm<input name="max_motor_rpm" type="number" value="${wheel.drive.max_motor_rpm}"></label>
</div></article>`}
async function loadChassis(){
  chassis=await api("/api/chassis");const f=$("chassisForm").elements;
  f.steering_interface.value=chassis.can.steering.interface;f.steering_bitrate.value=chassis.can.steering.bitrate;
  f.drive_interface.value=chassis.can.drive.interface;f.drive_bitrate.value=chassis.can.drive.bitrate;
  f.wheel_radius.value=chassis.geometry.wheel_radius;f.wheel_width.value=chassis.geometry.wheel_width;f.wheel_center_offset.value=chassis.geometry.wheel_center_offset;f.wheelbase.value=chassis.geometry.wheelbase;f.track_width.value=chassis.geometry.track_width;f.body_length.value=chassis.geometry.body_length;f.body_width.value=chassis.geometry.body_width;f.footprint_padding.value=chassis.geometry.footprint_padding;
  f.max_wheel_angular_velocity.value=chassis.safety.max_wheel_angular_velocity;f.max_wheel_angular_acceleration.value=chassis.safety.max_wheel_angular_acceleration;f.steering_limit_deg.value=chassis.safety.steering_limit_deg;f.drive_alignment_tolerance_deg.value=chassis.safety.drive_alignment_tolerance_deg;f.controller_command_timeout.value=chassis.safety.controller_command_timeout;
  f.maintenance_max_rpm.value=chassis.safety.maintenance_max_rpm;f.maintenance_max_steering_angle_deg.value=chassis.safety.maintenance_max_steering_angle_deg;f.maintenance_max_steering_step_deg.value=chassis.safety.maintenance_max_steering_step_deg;updateMaintenanceAngleMode();
  $("wheelConfigs").innerHTML=wheelOrder.map(name=>wheelCard(name,chassis.wheels[name])).join("");
  $("maintenanceWheel").innerHTML=wheelOrder.map(name=>`<option value="${name}">${escapeHtml(chassis.wheels[name].label)}</option>`).join("");
  $("bodyLegend").lastChild.textContent=`车体 ${Number(chassis.geometry.body_length).toFixed(2)}×${Number(chassis.geometry.body_width).toFixed(2)}m`;
  drawChassisTopView();
}
function collectChassis(){
  const f=$("chassisForm").elements,result=structuredClone(chassis);
  result.can.steering.interface=f.steering_interface.value;result.can.steering.bitrate=Number(f.steering_bitrate.value);
  result.can.drive.interface=f.drive_interface.value;result.can.drive.bitrate=Number(f.drive_bitrate.value);
  result.geometry.wheel_radius=Number(f.wheel_radius.value);result.geometry.wheel_width=Number(f.wheel_width.value);result.geometry.wheel_center_offset=Number(f.wheel_center_offset.value);result.geometry.wheelbase=Number(f.wheelbase.value);result.geometry.track_width=Number(f.track_width.value);result.geometry.body_length=Number(f.body_length.value);result.geometry.body_width=Number(f.body_width.value);result.geometry.footprint_padding=Number(f.footprint_padding.value);
  result.safety.max_wheel_angular_velocity=Number(f.max_wheel_angular_velocity.value);result.safety.max_wheel_angular_acceleration=Number(f.max_wheel_angular_acceleration.value);result.safety.steering_limit_deg=Number(f.steering_limit_deg.value);result.safety.drive_alignment_tolerance_deg=Number(f.drive_alignment_tolerance_deg.value);result.safety.controller_command_timeout=Number(f.controller_command_timeout.value);
  result.safety.maintenance_max_rpm=Number(f.maintenance_max_rpm.value);result.safety.maintenance_max_steering_angle_deg=Number(f.maintenance_max_steering_angle_deg.value);result.safety.maintenance_max_steering_step_deg=Number(f.maintenance_max_steering_step_deg.value);
  document.querySelectorAll(".wheel-card").forEach(card=>{const name=card.dataset.wheel,w=result.wheels[name],e=card.querySelectorAll("input,select"),v={};e.forEach(x=>v[x.name]=x.value);w.steering.motor_id=v.motor_id;w.steering.master_id=v.master_id;w.steering.direction=Number(v.steering_direction);w.steering.offset_rad=Number(v.offset_rad);w.steering.max_velocity_rad_s=Number(v.max_velocity_rad_s);w.drive.node_id=v.node_id;w.drive.channel=Number(v.channel);w.drive.direction=Number(v.drive_direction);w.drive.gear_ratio=Number(v.gear_ratio);w.drive.max_motor_rpm=Number(v.max_motor_rpm)});
  return result;
}
async function setMode(mode){
  try{const body={mode,acknowledge_wheels_lifted:mode!=="MAINTENANCE"||confirm("确认四个轮子已经架空、整车控制器已停止，并且现场有人监护？")};if(mode==="MAINTENANCE"&&!body.acknowledge_wheels_lifted)return;await api("/api/mode","POST",body);$("maintenanceDeadman").disabled=mode!=="MAINTENANCE";}catch(e){alert(e.message)}
}
function updateMaintenanceAngleMode(){if(!chassis)return;const absolute=$("maintenanceAngleMode").value==="absolute",limit=absolute?chassis.safety.steering_limit_deg:chassis.safety.maintenance_max_steering_step_deg;$("maintenanceAngleLabel").textContent=absolute?"绝对目标角 °":"相对当前角 °";$("maintenanceAngle").min=-limit;$("maintenanceAngle").max=limit;$("maintenanceNotice").textContent=absolute?"绝对目标角以标定零位为 0°；超出范围会自动修正到最近机械限位。":"相对量会加到按下瞬间的实际转角；最终目标超限时自动修正。"}
async function sendMaintenance(enabled=maintenanceHeld){const mode=$("maintenanceAngleMode").value,body={deadman:enabled,wheel:$("maintenanceWheel").value,rpm:Number($("maintenanceRpm").value),angle_mode:mode,command_id:maintenanceCommandId};if(mode==="absolute")body.target_angle_deg=maintenanceTargetAngleDeg;else body.relative_angle_deg=maintenanceTargetAngleDeg;try{const r=await api("/api/maintenance/command","POST",body);if(r.command){if(r.command.clamped){$("maintenanceAngle").value=number(r.command.angle_deg,1);$("maintenanceNotice").textContent=`输入超出范围，已自动修正为 ${number(r.command.angle_deg,1)}°。`}else{$("maintenanceNotice").textContent=`已锁存目标 ${number(r.command.angle_deg,1)}°。`}}}catch(e){endMaintenance();alert(e.message)}}
function beginMaintenance(e){e.preventDefault();const wheel=$("maintenanceWheel").value,position=latestStatus?.joint_states?.joints?.[`${wheel}_steer_joint`]?.position,input=Number($("maintenanceAngle").value);if(!Number.isFinite(position)||!Number.isFinite(input)){alert("缺少有效转向反馈或目标角");return}maintenanceTargetAngleDeg=input;maintenanceCommandId=crypto.randomUUID?crypto.randomUUID():`${Date.now()}-${Math.random()}`;maintenanceHeld=true;$("maintenanceDeadman").classList.add("held");sendMaintenance();clearInterval(maintenanceTimer);maintenanceTimer=setInterval(()=>sendMaintenance(),100)}
function endMaintenance(){if(!maintenanceHeld)return;maintenanceHeld=false;clearInterval(maintenanceTimer);maintenanceTimer=null;$("maintenanceDeadman").classList.remove("held");sendMaintenance(false);maintenanceTargetAngleDeg=null;maintenanceCommandId=""}

const joystick=$("joystick");
function moveJoystick(event){if(!joystick.hasPointerCapture(event.pointerId))return;clearMotionPreset();const rect=joystick.getBoundingClientRect(),radius=rect.width/2-34;let x=event.clientX-(rect.left+rect.width/2),y=event.clientY-(rect.top+rect.height/2);const length=Math.hypot(x,y);if(length>radius){x=x/length*radius;y=y/length*radius;}translation={x:x/radius,y:y/radius};$("stick").style.transform=`translate(${x}px,${y}px)`;renderCommand();}
joystick.addEventListener("pointerdown",e=>{joystick.setPointerCapture(e.pointerId);moveJoystick(e)});joystick.addEventListener("pointermove",moveJoystick);joystick.addEventListener("pointerup",()=>{translation={x:0,y:0};$("stick").style.transform="translate(0px,0px)";renderCommand();});
$("rotation").addEventListener("input",()=>{clearMotionPreset();renderCommand()});
$("motionSpeed").addEventListener("input",renderCommand);
async function activateMotionPreset(button){
  const vx=Number(button.dataset.vx),vy=Number(button.dataset.vy),wz=Number(button.dataset.wz),stopped=Math.abs(vx)+Math.abs(vy)+Math.abs(wz)<1e-6;
  if(stopped){$("stop").click();return}
  if(deadman&&activeMotionPreset===button){endDeadman();resetMotion();return}
  clearMotionPreset();activeMotionPreset=button;button.classList.add("active");
  translation={x:-vy,y:-vx};const rect=joystick.getBoundingClientRect(),radius=rect.width/2-34;$("stick").style.transform=`translate(${translation.x*radius}px,${translation.y*radius}px)`;$("rotation").value=Math.max(-100,Math.min(100,wz*100));renderCommand();
  if(!deadman){await beginDeadman();if(!deadman)clearMotionPreset()}else{sendCommand(true)}
}
document.querySelectorAll(".motion-preset").forEach(button=>button.addEventListener("click",()=>activateMotionPreset(button)));
$("stop").addEventListener("click",async()=>{endDeadman();try{await api("/api/control/stop","POST",{})}catch(e){alert(e.message)}resetMotion();updateControls();});
$("deadman").addEventListener("click",toggleDeadman);
$("saveConfig").addEventListener("click",async()=>{const f=$("configForm").elements,body={safety:{max_linear_x:Number(f.max_linear_x.value),max_linear_y:Number(f.max_linear_y.value),max_angular_z:Number(f.max_angular_z.value)}};try{const r=await api("/api/config","PUT",body);config=r.config;$("configNotice").textContent=r.restart_required?"已保存；部分设置需重启节点。":"已保存并生效。";renderCommand()}catch(e){alert(e.message)}});
$("setToken").addEventListener("click",()=>{const current=sessionStorage.getItem("mobileBaseToken")||"",value=prompt("输入本次浏览器会话使用的 MOBILE_BASE_WEB_TOKEN：",current);if(value!==null)sessionStorage.setItem("mobileBaseToken",value.trim())});
$("modeDisabled").addEventListener("click",()=>setMode("DISABLED"));$("modeMaintenance").addEventListener("click",()=>setMode("MAINTENANCE"));$("modeNormal").addEventListener("click",()=>setMode("NORMAL"));
$("maintenanceDeadman").addEventListener("pointerdown",beginMaintenance);["pointerup","pointercancel","pointerleave"].forEach(name=>$("maintenanceDeadman").addEventListener(name,endMaintenance));
$("maintenanceAngleMode").addEventListener("change",updateMaintenanceAngleMode);
$("readZero").addEventListener("click",async()=>{try{const r=await api("/api/calibration/read","POST",{wheel:$("maintenanceWheel").value});zeroPreviewToken=r.preview_token;$("zeroPreview").textContent=`绝对位置 ${number(r.raw_position,6)} rad；旧零偏 ${number(r.old_offset,6)}；建议新零偏 ${number(r.proposed_offset,6)} rad。预览 60 秒内有效。`;$("saveZero").disabled=false}catch(e){zeroPreviewToken="";$("saveZero").disabled=true;alert(e.message)}});
$("saveZero").addEventListener("click",async()=>{if(!zeroPreviewToken)return;if(!confirm("确认所选轮当前已经机械朝向正前方？保存后需要重启底盘并低速验证。"))return;try{const r=await api("/api/calibration/save","POST",{preview_token:zeroPreviewToken,confirm_wheel_is_straight:true});zeroPreviewToken="";$("saveZero").disabled=true;$("zeroPreview").textContent=`${r.wheel} 零偏已保存为 ${number(r.offset_rad,6)} rad，重启底盘后生效。`;await loadChassis()}catch(e){alert(e.message)}});
$("maintenanceWheel").addEventListener("change",()=>{zeroPreviewToken="";$("saveZero").disabled=true;$("zeroPreview").textContent="轮组已切换，请重新读取绝对位置。"});
$("saveChassis").addEventListener("click",async()=>{try{const r=await api("/api/chassis","PUT",collectChassis());chassis=r.config;$("chassisNotice").textContent=`配置已保存，重启底盘后生效${r.backup?"；原配置已备份":""}。`}catch(e){alert(e.message)}});
document.querySelectorAll(".nav-item").forEach(button=>button.addEventListener("click",()=>showPage(button.dataset.page)));
window.addEventListener("popstate",()=>showPage(location.hash.slice(1)||"overview",false));
document.querySelectorAll("[data-stop]").forEach(button=>button.addEventListener("click",()=>$("stop").click()));
$("mapZoomIn").addEventListener("click",()=>{mapSpan=Math.max(.625,mapSpan/2);drawMap()});
$("mapZoomOut").addEventListener("click",()=>{mapSpan=Math.min(40,mapSpan*2);drawMap()});
$("mapCenter").addEventListener("click",drawMap);
$("mapClear").addEventListener("click",()=>{odomTrail=odomTrail.slice(-1);drawMap()});
function mapPointerWorld(event){const rect=$("mapCanvas").getBoundingClientRect(),{ppm,cx,cy}=mapViewMetrics(rect.width,rect.height);return{x:(event.clientX-rect.left-cx)/ppm,y:(cy-(event.clientY-rect.top))/ppm}}
function showMapProbe(point){const cell=occupancyAtWorld(point.x,point.y),className=cell===null?"地图外":cell<0?"未知":cell>=65?"障碍":cell>20?"过渡区":"可通行";$("mapProbe").textContent=`X ${metricCoordinateLabel(point.x)} · Y ${metricCoordinateLabel(point.y)} · ${className}${cell!==null&&cell>=0?` ${cell}%`:""}`}
function updateNavGoalText(){if(!selectedNavGoal)return;const degrees=selectedNavGoal.yaw*180/Math.PI;$("navYaw").value=number(degrees,1);$("navDetail").textContent=`x ${number(selectedNavGoal.x,2)} · y ${number(selectedNavGoal.y,2)} · 朝向 ${number(degrees,1)}°`}
const mapCanvas=$("mapCanvas");
mapCanvas.addEventListener("pointerdown",event=>{event.preventDefault();const point=mapPointerWorld(event),current=mapViewMetrics(mapCanvas.clientWidth,mapCanvas.clientHeight).pose||{yaw:0};selectedNavGoal={x:point.x,y:point.y,yaw:current.yaw||0};navGoalDrag={id:event.pointerId};mapCanvas.setPointerCapture(event.pointerId);$("sendNavGoal").disabled=false;$("navYaw").disabled=false;$("navState").textContent="已选择目标；拖动可设置朝向";updateNavGoalText();drawMap()});
mapCanvas.addEventListener("pointermove",event=>{const point=mapPointerWorld(event);showMapProbe(point);if(!navGoalDrag||navGoalDrag.id!==event.pointerId||!selectedNavGoal)return;const dx=point.x-selectedNavGoal.x,dy=point.y-selectedNavGoal.y;if(Math.hypot(dx,dy)>.03){selectedNavGoal.yaw=Math.atan2(dy,dx);updateNavGoalText();drawMap()}});
mapCanvas.addEventListener("pointerleave",()=>{$("mapProbe").textContent="移动鼠标查看坐标与栅格占用"});
function finishNavGoalDrag(){navGoalDrag=null;$("navState").textContent="目标位置和朝向已设置"}
mapCanvas.addEventListener("pointerup",finishNavGoalDrag);mapCanvas.addEventListener("pointercancel",finishNavGoalDrag);
$("navYaw").addEventListener("input",()=>{if(!selectedNavGoal)return;const degrees=Math.max(-180,Math.min(180,Number($("navYaw").value)||0));selectedNavGoal.yaw=degrees*Math.PI/180;updateNavGoalText();drawMap()});
$("sendNavGoal").addEventListener("click",async()=>{if(!selectedNavGoal)return;endDeadman();try{await api("/api/navigation/goal","POST",selectedNavGoal);$("navState").textContent="正在提交导航目标"}catch(e){alert(e.message)}});
$("cancelNavGoal").addEventListener("click",async()=>{try{await api("/api/navigation/cancel","POST",{});$("navState").textContent="导航已取消"}catch(e){alert(e.message)}});
$("resetView").addEventListener("click",()=>{modelScale=1;sceneYaw=-.65;scenePitch=.62;sceneZoom=74;sceneTopDown=false;sceneFollowRobot=false;sceneAutoFit=true;$("sceneTopDown").classList.remove("active");$("sceneTopDown").textContent="正北俯视";$("sceneFollowRobot").classList.remove("active");fitSemanticScene();applyModelView();drawScene3d()});
$("sceneTopDown").addEventListener("click",()=>{sceneTopDown=!sceneTopDown;$("sceneTopDown").classList.toggle("active",sceneTopDown);$("sceneTopDown").textContent=sceneTopDown?"返回斜视":"正北俯视";drawScene3d()});
$("sceneFollowRobot").addEventListener("click",()=>{sceneFollowRobot=!sceneFollowRobot;sceneAutoFit=!sceneFollowRobot;$("sceneFollowRobot").classList.toggle("active",sceneFollowRobot);if(sceneFollowRobot){const pose=currentScenePose();if(pose)sceneCenter={...sceneCenter,x:pose.x,y:pose.y}}else fitSemanticScene();drawScene3d()});
function showSpatialView(mode){sceneMode=mode;const scene=mode!=="vehicle";$("scene3dStage").classList.toggle("hidden",!scene);$("vehicleStage").style.display=scene?"none":"block";$("modelWheelData").classList.toggle("hidden",scene);$("semanticLegend").classList.toggle("hidden",mode!=="semantic");$("showScene3d").classList.toggle("active",mode==="semantic");$("showNvblox3d").classList.toggle("active",mode==="nvblox");$("showVehicle3d").classList.toggle("active",mode==="vehicle");if(mode==="semantic")fitSemanticScene();if(mode==="nvblox")pollScene3d();requestAnimationFrame(scene?drawScene3d:drawChassisTopView)}
$("showScene3d").addEventListener("click",()=>showSpatialView("semantic"));$("showNvblox3d").addEventListener("click",()=>showSpatialView("nvblox"));$("showVehicle3d").addEventListener("click",()=>showSpatialView("vehicle"));
const sceneStage=$("scene3dStage");
sceneStage.addEventListener("pointerdown",event=>{sceneDrag={id:event.pointerId,x:event.clientX,y:event.clientY};sceneStage.setPointerCapture(event.pointerId)});
sceneStage.addEventListener("pointermove",event=>{if(!sceneDrag||sceneDrag.id!==event.pointerId)return;const dx=event.clientX-sceneDrag.x,dy=event.clientY-sceneDrag.y;if(sceneTopDown){sceneFollowRobot=false;sceneAutoFit=false;$("sceneFollowRobot").classList.remove("active");sceneCenter={...sceneCenter,x:sceneCenter.x-dx/sceneZoom,y:sceneCenter.y+dy/sceneZoom}}else{sceneYaw+=dx*.008;scenePitch=Math.max(.12,Math.min(1.42,scenePitch+dy*.006))}sceneDrag.x=event.clientX;sceneDrag.y=event.clientY;drawScene3d()});
sceneStage.addEventListener("pointerup",()=>sceneDrag=null);sceneStage.addEventListener("pointercancel",()=>sceneDrag=null);sceneStage.addEventListener("wheel",event=>{event.preventDefault();sceneAutoFit=false;sceneZoom=Math.max(8,Math.min(190,sceneZoom-event.deltaY*.12));drawScene3d()},{passive:false});
const vehicleStage=$("vehicleStage");
vehicleStage.addEventListener("wheel",event=>{event.preventDefault();modelScale=Math.max(.7,Math.min(1.45,modelScale-event.deltaY*.001));applyModelView()},{passive:false});
window.addEventListener("resize",()=>{drawMap();drawScene3d();drawChassisTopView()});
document.addEventListener("visibilitychange",()=>{if(document.hidden){endDeadman();endMaintenance()}});window.addEventListener("blur",()=>{endDeadman();endMaintenance()});window.addEventListener("beforeunload",()=>{endDeadman();endMaintenance()});
showPage(location.hash.slice(1)||"overview",false);applyModelView();showSpatialView("semantic");updateControls();loadConfig().catch(()=>{});loadChassis().catch(()=>{});poll();pollMap();pollScene3d();pollRacks();pollCamera();setInterval(poll,500);setInterval(pollMap,2000);setInterval(pollScene3d,1500);setInterval(pollRacks,1500);setInterval(pollCamera,1000);
