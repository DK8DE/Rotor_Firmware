#pragma once

// SoftAP SPA – Design analog Inklinometer (Farben/Menü), Tabs: Status / Update / Reset
static const char WEB_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rotor DK8DE</title>
<meta http-equiv="Cache-Control" content="no-store">
<style>
:root{
  color-scheme:dark;
  --bg:#09131a;--panel:#111d25;--panel2:#162833;--line:#2c3e4a;
  --text:#eaf2f8;--muted:#a9bac7;--blue:#55a9e6;--blue2:#8bc9f4;
  --blue-deep:#d9efff;--blue-light:#112b3d;--cyan:#58cbe1;
  --ok:#67d88b;--warn:#fbbf24;--err:#e07a72;--white:#ffffff;--gray:#a9bac7;
  --shadow:0 16px 38px rgba(0,0,0,.30);
}
*{box-sizing:border-box}
[hidden]{display:none!important}
body{
  margin:0;min-height:100vh;color:var(--text);
  font:15px/1.45 "Segoe UI",system-ui,sans-serif;
  background:
    radial-gradient(circle at 80% 0%,rgba(56,184,211,.14),transparent 32%),
    radial-gradient(900px 480px at 0% -10%,rgba(18,101,168,.22),transparent 55%),
    var(--bg);
}
header{
  display:flex;align-items:center;justify-content:space-between;gap:1rem;
  padding:1.1rem 1.4rem;border-bottom:1px solid var(--line);
  background:color-mix(in srgb,var(--panel) 92%,transparent);backdrop-filter:blur(13px);
  position:sticky;top:0;z-index:5;
}
.brand{display:flex;flex-direction:column;gap:.15rem}
.brand b{font-size:1.15rem;letter-spacing:.02em;color:var(--white)}
.brand span{color:var(--muted);font-size:.85rem}
.hdr-right{display:flex;align-items:center;gap:.55rem;flex-wrap:wrap;justify-content:flex-end}
.lang{display:flex;gap:.35rem}
.lang-btn{
  appearance:none;border:1px solid var(--line);background:var(--panel2);color:var(--text);
  border-radius:.55rem;padding:.35rem .55rem;cursor:pointer;font:inherit;font-size:.8rem;font-weight:700;
  letter-spacing:.04em;min-width:2.6rem
}
.lang-btn:hover{border-color:var(--blue);background:var(--blue-light)}
.lang-btn.active{color:var(--blue-deep);border-color:var(--blue);background:var(--blue-light)}
.pill{
  display:inline-flex;align-items:center;gap:.4rem;
  padding:.35rem .7rem;border-radius:999px;border:1px solid var(--line);
  background:var(--panel2);color:var(--gray);font-size:.8rem
}
.dot{width:.55rem;height:.55rem;border-radius:50%;background:var(--warn)}
.dot.on{background:var(--ok);box-shadow:0 0 0 4px rgba(103,216,139,.15)}
.dot.off{background:var(--err);box-shadow:0 0 0 4px rgba(224,122,114,.15)}
main{max-width:920px;margin:0 auto;padding:1.25rem}
.nav-wrap{margin-bottom:1rem;position:relative}
.nav-burger{
  display:none;appearance:none;border:1px solid var(--line);background:var(--panel);
  border-radius:.7rem;width:2.7rem;height:2.55rem;padding:.55rem .6rem;cursor:pointer;
  flex-direction:column;justify-content:center;gap:.32rem
}
.nav-burger span{display:block;height:2px;border-radius:2px;background:var(--text);transition:transform .2s,opacity .2s}
.nav-wrap.open .nav-burger span:nth-child(1){transform:translateY(7px) rotate(45deg)}
.nav-wrap.open .nav-burger span:nth-child(2){opacity:0}
.nav-wrap.open .nav-burger span:nth-child(3){transform:translateY(-7px) rotate(-45deg)}
nav{display:flex;gap:.5rem;flex-wrap:wrap}
nav button{
  appearance:none;border:1px solid var(--line);background:var(--panel);
  color:var(--muted);padding:.55rem 1rem;border-radius:.7rem;cursor:pointer;font:inherit
}
nav button.active{
  color:var(--blue-deep);border-color:var(--blue);
  background:var(--blue-light);box-shadow:inset 0 -2px var(--blue)
}
@media(max-width:535px){
  .nav-wrap{display:flex;flex-direction:column;align-items:stretch;z-index:20}
  .nav-burger{display:inline-flex;align-self:flex-end;position:relative;z-index:21}
  nav{
    display:none;flex-direction:column;gap:.4rem;
    position:absolute;top:calc(100% + .35rem);left:0;right:0;
    padding:.55rem;border:1px solid var(--line);border-radius:.85rem;
    background:var(--panel);box-shadow:var(--shadow);z-index:20
  }
  .nav-wrap.open nav{display:flex}
  nav button{width:100%;text-align:left}
}
.card{
  background:var(--panel);
  border:1px solid var(--line);border-radius:1rem;padding:1.1rem 1.2rem;margin-bottom:1rem;
  box-shadow:var(--shadow)
}
.card h2{margin:0 0 .85rem;font-size:1rem;color:var(--cyan);font-weight:600}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:.75rem}
.kv{background:var(--blue-light);border:1px solid var(--line);border-radius:.75rem;padding:.7rem .8rem}
.kv .k{color:var(--muted);font-size:.75rem;text-transform:uppercase;letter-spacing:.04em}
.kv .v{margin-top:.35rem;color:var(--white);word-break:break-all;font-size:2em;font-weight:700;line-height:1.15}
.field{margin-bottom:.15rem}
label{display:block;color:var(--muted);font-size:.8rem;margin:0 0 .35rem}
input,button{font:inherit}
input[type=file]{
  width:100%;padding:.65rem .75rem;border-radius:.65rem;border:1px solid var(--line);
  background:#0c1a22;color:var(--text);outline:none;margin-bottom:.35rem
}
.actions{display:flex;flex-wrap:wrap;gap:.55rem;margin-top:.4rem}
.btn{
  appearance:none;border:0;border-radius:.7rem;padding:.65rem 1rem;cursor:pointer;
  font-weight:600;color:var(--white);background:var(--blue);
  display:inline-flex;align-items:center;justify-content:center;font:inherit;font-size:.95rem
}
.btn:hover{filter:brightness(1.06)}
.btn.sec{background:var(--panel2);color:var(--text);border:1px solid var(--line)}
.btn.sec:hover{border-color:var(--blue);background:var(--blue-light);filter:none}
.btn.danger{background:var(--err);color:var(--white)}
.btn:disabled{opacity:.5;cursor:not-allowed}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.muted{color:var(--muted);font-size:.9rem}
.progress{height:10px;background:#0c1a22;border:1px solid var(--line);border-radius:999px;overflow:hidden;margin:1rem 0}
.bar{height:100%;width:0%;background:linear-gradient(90deg,var(--blue),var(--cyan))}
.toast{
  position:fixed;right:16px;bottom:16px;background:var(--panel2);border:1px solid var(--line);
  padding:.7rem .9rem;border-radius:.85rem;display:none;max-width:360px;z-index:50;
  box-shadow:var(--shadow);color:var(--text)
}
.toast.show{display:block}
.toast.ok{border-color:rgba(103,216,139,.55)}
.toast.bad{border-color:rgba(224,122,114,.55)}
.foot{color:var(--muted);font-size:.8rem;text-align:center;padding:1rem 0 2rem}
</style>
</head>
<body>
<header>
  <div class="brand">
    <b>Rotor DK8DE</b>
    <span data-i18n="subtitle">SoftAP · Status · OTA</span>
  </div>
  <div class="hdr-right">
    <div class="lang" id="langSwitch">
      <button type="button" class="lang-btn active" data-lang="de">DE</button>
      <button type="button" class="lang-btn" data-lang="en">EN</button>
    </div>
    <div class="pill"><span class="dot" id="connDot"></span><span id="connBadge" data-i18n="badge_loading">…</span></div>
  </div>
</header>
<main>
  <div class="nav-wrap" id="navWrap">
    <button type="button" class="nav-burger" id="navBurger" aria-label="Menu" aria-expanded="false" aria-controls="mainNav">
      <span></span><span></span><span></span>
    </button>
    <nav id="mainNav">
      <button class="active" data-tab="status" data-i18n="tab_status">Status</button>
      <button data-tab="update" data-i18n="tab_update">Update</button>
      <button data-tab="reset" data-i18n="tab_reset">Reset</button>
    </nav>
  </div>

  <section id="tab-status">
    <div class="card">
      <h2 data-i18n="h_rotor">Rotordaten</h2>
      <div class="grid">
        <div class="kv"><div class="k" data-i18n="k_angle">Winkel (°)</div><div class="v mono" id="vAngle">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_tempA">Temp Umgebung (°C)</div><div class="v mono" id="vTempA">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_tempM">Temp Motor (°C)</div><div class="v mono" id="vTempM">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_fw">Firmware</div><div class="v mono" id="vFw">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_homed">Gehomt</div><div class="v mono" id="vHomed">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_pwm">SETPWM (%)</div><div class="v mono" id="vPwm">—</div></div>
        <div class="kv" id="kvWindKmh" hidden><div class="k" data-i18n="k_wind">Wind (km/h)</div><div class="v mono" id="vWindKmh">—</div></div>
        <div class="kv" id="kvWindDir" hidden><div class="k" data-i18n="k_windDir">Windrichtung (°)</div><div class="v mono" id="vWindDir">—</div></div>
        <div class="kv"><div class="k">SoftAP</div><div class="v mono" id="vAp">—</div></div>
        <div class="kv"><div class="k">UID</div><div class="v mono" id="vUid">—</div></div>
      </div>
    </div>
  </section>

  <section id="tab-update" hidden>
    <div class="card">
      <h2 data-i18n="h_update">Firmware-Update (OTA)</h2>
      <div class="grid" style="margin-bottom:.85rem">
        <div class="kv"><div class="k" data-i18n="k_cur">Aktuell</div><div class="v mono" id="updCur">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_part">Partition</div><div class="v mono" id="updPart">—</div></div>
      </div>
      <div class="field">
        <label for="fwFile">firmware.bin</label>
        <input type="file" id="fwFile" accept=".bin">
      </div>
      <div class="progress"><div class="bar" id="updBar"></div></div>
      <div class="actions">
        <button class="btn" id="btnUpload" data-i18n="btn_upload">Hochladen &amp; flashen</button>
      </div>
      <div class="muted" style="margin-top:.65rem" id="updMsg" data-i18n="hint_ota">Nur App-Firmware (Dual-OTA). Nach Erfolg Neustart.</div>
    </div>
  </section>

  <section id="tab-reset" hidden>
    <div class="card">
      <h2 data-i18n="h_reset">Neustart / Werkseinstellung</h2>
      <p class="muted" data-i18n="p_reset">Neustart: SoftAP bleibt aus bis SETWLAN:1. Werkseinstellung: FRAM + NVS löschen.</p>
      <div class="actions">
        <button class="btn sec" id="btnReboot" data-i18n="btn_reboot">Neustart</button>
        <button class="btn danger" id="btnFactory" data-i18n="btn_factory">Werkseinstellung</button>
      </div>
    </div>
  </section>

  <div class="foot">&copy; DK8DE J&ouml;rg K&ouml;rner 2026</div>
</main>
<div class="toast" id="toast"></div>
<script>
const $ = (id)=>document.getElementById(id);
let lang = 'de';
const I18N = {
de:{
  subtitle:'SoftAP · Status · OTA',
  badge_loading:'laden…', badge_ok:'OK', badge_offline:'offline',
  tab_status:'Status', tab_update:'Update', tab_reset:'Reset',
  h_rotor:'Rotordaten',
  k_angle:'Winkel (°)', k_tempA:'Temp Umgebung (°C)', k_tempM:'Temp Motor (°C)',
  k_fw:'Firmware', k_homed:'Gehomt', k_pwm:'SETPWM (%)',
  k_wind:'Wind (km/h)', k_windDir:'Windrichtung (°)',
  h_update:'Firmware-Update (OTA)', k_cur:'Aktuell', k_part:'Partition',
  btn_upload:'Hochladen & flashen', hint_ota:'Nur firmware.bin (App-OTA). Nicht factory.bin. Nach Erfolg Neustart.',
  upd_choose:'Datei wählen', upd_uploading:'Upload…', upd_ok:'OK – Neustart…', upd_fail:'Update fehlgeschlagen',
  h_reset:'Neustart / Werkseinstellung',
  p_reset:'Neustart: SoftAP bleibt aus bis SETWLAN:1. Werkseinstellung: FRAM + NVS löschen und Neustart.',
  btn_reboot:'Neustart', btn_factory:'Werkseinstellung',
  confirm_factory:'Wirklich Werkseinstellung und Neustart?',
  toast_update_ok:'Update OK', toast_update_fail:'Update fehlgeschlagen',
  toast_reboot:'Neustart…', toast_factory:'Factory…', auth_required:'Anmeldung erforderlich'
},
en:{
  subtitle:'SoftAP · Status · OTA',
  badge_loading:'loading…', badge_ok:'OK', badge_offline:'offline',
  tab_status:'Status', tab_update:'Update', tab_reset:'Reset',
  h_rotor:'Rotor data',
  k_angle:'Angle (°)', k_tempA:'Ambient temp (°C)', k_tempM:'Motor temp (°C)',
  k_fw:'Firmware', k_homed:'Homed', k_pwm:'SETPWM (%)',
  k_wind:'Wind (km/h)', k_windDir:'Wind direction (°)',
  h_update:'Firmware update (OTA)', k_cur:'Current', k_part:'Partition',
  btn_upload:'Upload & flash', hint_ota:'firmware.bin only (app OTA). Not factory.bin. Reboot after success.',
  upd_choose:'Choose file', upd_uploading:'Upload…', upd_ok:'OK – rebooting…', upd_fail:'Update failed',
  h_reset:'Reboot / factory reset',
  p_reset:'Reboot: SoftAP stays off until SETWLAN:1. Factory: clear FRAM + NVS and reboot.',
  btn_reboot:'Reboot', btn_factory:'Factory reset',
  confirm_factory:'Really factory reset and reboot?',
  toast_update_ok:'Update OK', toast_update_fail:'Update failed',
  toast_reboot:'Rebooting…', toast_factory:'Factory…', auth_required:'Login required'
}
};
function t(k){return (I18N[lang]&&I18N[lang][k])||(I18N.de[k])||k;}
function applyI18n(){
  document.querySelectorAll('[data-i18n]').forEach(el=>{
    const k=el.getAttribute('data-i18n'); if(k) el.textContent=t(k);
  });
  document.querySelectorAll('.lang-btn').forEach(b=>b.classList.toggle('active', b.dataset.lang===lang));
}
function toast(msg, ok){
  const el=$('toast'); el.textContent=msg; el.className='toast show '+(ok?'ok':'bad');
  setTimeout(()=>el.classList.remove('show'), 2800);
}
function api(path, opts){
  var u=path+(path.indexOf('?')>=0?'&':'?')+'_='+Date.now();
  var o={credentials:'same-origin'};
  if(opts){
    if(opts.method) o.method=opts.method;
    if(opts.body) o.body=opts.body;
    if(opts.headers) o.headers=opts.headers;
  }
  return fetch(u, o).then(function(r){
    if(r.status===401){ toast(t('auth_required'), false); throw new Error('auth'); }
    return r;
  });
}
function showTab(name){
  var tabs=['status','update','reset'];
  for(var i=0;i<tabs.length;i++){
    var sec=document.getElementById('tab-'+tabs[i]);
    if(sec){
      if(tabs[i]===name){ sec.removeAttribute('hidden'); sec.style.display=''; }
      else { sec.setAttribute('hidden',''); sec.style.display='none'; }
    }
  }
  var buttons=document.querySelectorAll('nav button[data-tab]');
  for(var j=0;j<buttons.length;j++){
    var b=buttons[j];
    if(b.getAttribute('data-tab')===name) b.classList.add('active');
    else b.classList.remove('active');
  }
  var wrap=document.getElementById('navWrap'); if(wrap) wrap.classList.remove('open');
  var burger=document.getElementById('navBurger'); if(burger) burger.setAttribute('aria-expanded','false');
  if(name==='update') loadUpdInfo();
}
function setConn(ok){
  var d=document.getElementById('connDot'); if(d) d.className='dot '+(ok?'on':'off');
  var b=document.getElementById('connBadge'); if(b) b.textContent=ok?t('badge_ok'):t('badge_offline');
}
function fmt(n, d){ if(n==null||!isFinite(n)) return '-'; return Number(n).toFixed(d); }
function setTxt(id, v){ var el=document.getElementById(id); if(el) el.textContent=v; }
function setHtml(id, v){ var el=document.getElementById(id); if(el) el.innerHTML=v; }
function showEl(id, on){
  var el=document.getElementById(id); if(!el) return;
  if(on){ el.removeAttribute('hidden'); el.style.display=''; }
  else { el.setAttribute('hidden',''); el.style.display='none'; }
}
function pollLive(){
  api('/api/live').then(function(r){ return r.json(); }).then(function(j){
    setTxt('vAngle', fmt(j.angle, 2));
    setTxt('vTempA', fmt(j.tempA, 1));
    setTxt('vTempM', fmt(j.tempM, 1));
    setTxt('vFw', j.fw||'-');
    setTxt('vHomed', (j.homed!=null)?String(j.homed):'-');
    setTxt('vPwm', fmt(j.pwm, 1));
    var windOn=!!j.windEn;
    showEl('kvWindKmh', windOn);
    showEl('kvWindDir', windOn);
    if(windOn){
      setTxt('vWindKmh', fmt(j.windKmh, 1));
      setTxt('vWindDir', fmt(j.windDir, 1));
    }
    var ssid=j.ssid||'-';
    var ip=j.ip||'-';
    setHtml('vAp', ssid+'<br>'+ip);
    setTxt('vUid', j.uid||'-');
    setConn(true);
  }).catch(function(){ setConn(false); });
}
function loadUpdInfo(){
  api('/api/update/info').then(function(r){ return r.json(); }).then(function(j){
    setTxt('updCur', j.fw||'-');
    setTxt('updPart', (j.partition||'-')+(j.size?(' · '+j.size):''));
  }).catch(function(){});
}
var btnUpload=document.getElementById('btnUpload');
if(btnUpload) btnUpload.onclick=function(){
  var inp=document.getElementById('fwFile');
  var f=inp&&inp.files&&inp.files[0];
  if(!f){ toast(t('upd_choose'), false); return; }
  setTxt('updMsg', t('upd_uploading'));
  var bar=document.getElementById('updBar'); if(bar) bar.style.width='0%';
  var xhr=new XMLHttpRequest();
  xhr.open('POST','/api/update');
  xhr.withCredentials=true;
  xhr.upload.onprogress=function(ev){
    if(ev.lengthComputable&&bar) bar.style.width=Math.round(100*ev.loaded/ev.total)+'%';
  };
  xhr.onload=function(){
    var ok=false;
    try{ ok=JSON.parse(xhr.responseText).ok; }catch(e){}
    if(ok){
      setTxt('updMsg', t('upd_ok'));
      toast(t('toast_update_ok'), true);
      setTimeout(function(){ api('/api/reboot',{method:'POST'}); }, 600);
    }else{
      setTxt('updMsg', t('upd_fail'));
      toast(t('toast_update_fail'), false);
    }
  };
  xhr.onerror=function(){ setTxt('updMsg', t('upd_fail')); toast(t('toast_update_fail'), false); };
  var fd=new FormData(); fd.append('update', f);
  xhr.send(fd);
};
var btnReboot=document.getElementById('btnReboot');
if(btnReboot) btnReboot.onclick=function(){
  toast(t('toast_reboot'), true);
  api('/api/reboot',{method:'POST'}).catch(function(){});
};
var btnFactory=document.getElementById('btnFactory');
if(btnFactory) btnFactory.onclick=function(){
  if(!confirm(t('confirm_factory'))) return;
  toast(t('toast_factory'), true);
  api('/api/factory',{method:'POST'}).catch(function(){});
};
var navBtns=document.querySelectorAll('nav button[data-tab]');
for(var ni=0;ni<navBtns.length;ni++){
  (function(btn){
    btn.onclick=function(){ showTab(btn.getAttribute('data-tab')); };
  })(navBtns[ni]);
}
var burgerEl=document.getElementById('navBurger');
if(burgerEl) burgerEl.onclick=function(){
  var w=document.getElementById('navWrap'); if(!w) return;
  var open=!w.classList.contains('open');
  if(open) w.classList.add('open'); else w.classList.remove('open');
  burgerEl.setAttribute('aria-expanded', open?'true':'false');
};
var langBtns=document.querySelectorAll('.lang-btn');
for(var li=0;li<langBtns.length;li++){
  (function(btn){
    btn.onclick=function(){ lang=btn.getAttribute('data-lang'); applyI18n(); };
  })(langBtns[li]);
}
applyI18n();
showTab('status');
pollLive();
setInterval(pollLive, 500);
</script>
</body>
</html>
)HTML";
