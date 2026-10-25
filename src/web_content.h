#pragma once

// SoftAP SPA – Tabs: Status / Bewegen / Setup / Benutzer / WLAN / Update / Reset
static const char WEB_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rotorsteuerung DK8DE</title>
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
  box-shadow:var(--shadow);overflow:hidden
}
.card h2{
  margin:-1.1rem -1.2rem .85rem;padding:.9rem 1.2rem;
  font-size:1.28rem;font-weight:600;color:var(--white);
  background:rgb(21,43,62);width:auto;box-sizing:border-box
}
.card-head{
  margin:-1.1rem -1.2rem 0;padding:.9rem 1.2rem;
  background:rgb(21,43,62)
}
.card-head h2{
  margin:0;padding:0;background:transparent;font-size:1.28rem
}
.move-angle-wrap{
  float:right;margin:10px;padding:0
}
.move-angle-wrap .kv{
  margin:0;min-width:6.5rem;padding:.4rem .65rem
}
.move-angle-wrap .kv .k{font-size:.68rem}
.move-angle-wrap .kv .v{margin-top:.15rem;font-size:1.35em;line-height:1.1}
#tab-move .move-intro{margin:10px 10px 10px 0}
#tab-move .move-clear{clear:both}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:.75rem}
.kv{background:var(--blue-light);border:1px solid var(--line);border-radius:.75rem;padding:.7rem .8rem}
.kv .k{color:var(--muted);font-size:.75rem;text-transform:uppercase;letter-spacing:.04em}
.kv .v{margin-top:.35rem;color:var(--white);word-break:break-all;font-size:2em;font-weight:700;line-height:1.15}
#vAp{font-size:1.5em;line-height:1.2}
#updPart{font-size:1.15em;line-height:1.35;font-weight:700}
#wifiSsid{font-size:1.35rem;font-weight:700;color:var(--white);margin:.35rem 0 .85rem;line-height:1.3}
.field{margin-bottom:.75rem}
label{display:block;color:var(--muted);font-size:.8rem;margin:0 0 .35rem}
input,select,button{font:inherit}
input[type=text],input[type=password],input[type=number],select{
  width:100%;padding:.65rem .75rem;border-radius:.65rem;border:1px solid var(--line);
  background:#0c1a22;color:var(--text);outline:none
}
input[type=range]{
  width:100%;accent-color:var(--blue);height:1.4rem;cursor:pointer;margin:.35rem 0
}
.jog-row{display:flex;gap:.75rem;flex-wrap:wrap;margin:.5rem 0 1rem}
.jog-row .btn{flex:1;min-width:8rem;min-height:3.2rem;font-size:1.05rem;user-select:none;touch-action:none}
.pwm-meta{display:flex;justify-content:space-between;gap:1rem;color:var(--muted);font-size:.82rem;margin-top:.25rem}
.pwm-val{font-size:1.35rem;font-weight:700;color:var(--white);margin:.2rem 0 .4rem}
input[type=file]{
  width:100%;padding:.65rem .75rem;border-radius:.65rem;border:1px solid var(--line);
  background:#0c1a22;color:var(--text);outline:none;margin-bottom:.35rem
}
.actions{display:flex;flex-wrap:wrap;gap:.55rem;margin-top:.4rem}
.btn{
  appearance:none;border:0;border-radius:.7rem;padding:.65rem 1rem;cursor:pointer;
  font-weight:600;color:var(--white);background:var(--blue);
  display:inline-flex;align-items:center;justify-content:center;font:inherit;font-size:.95rem;user-select:none
}
.btn:hover{filter:brightness(1.06)}
.btn.sec{background:var(--panel2);color:var(--text);border:1px solid var(--line)}
.btn.sec:hover{border-color:var(--blue);background:var(--blue-light);filter:none}
.btn.danger{background:var(--err);color:var(--white)}
.btn:disabled{opacity:.5;cursor:not-allowed}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.muted{color:var(--muted);font-size:.9rem}
.hint{color:var(--muted);font-size:.82rem;margin:.25rem 0 .6rem}
.errbox{color:var(--err);font-size:.9rem;margin:.4rem 0}
.okbox{color:var(--ok);font-size:.9rem;margin:.4rem 0}
.steps{display:flex;flex-wrap:wrap;gap:.35rem;margin-bottom:.85rem}
.step-pill{
  font-size:.72rem;padding:.25rem .5rem;border-radius:999px;border:1px solid var(--line);
  color:var(--muted);background:var(--panel2)
}
.step-pill.on{color:var(--blue-deep);border-color:var(--blue);background:var(--blue-light)}
.step-pill.done{color:var(--ok);border-color:rgba(103,216,139,.45)}
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
.movebadge{display:inline-block;padding:.2rem .55rem;border-radius:.5rem;border:1px solid var(--line);font-size:.8rem}
.movebadge.on{color:var(--warn);border-color:var(--warn)}
</style>
</head>
<body>
<header>
  <div class="brand">
    <b data-i18n="brand_title">Rotorsteuerung DK8DE</b>
    <span id="brandSub" data-i18n="brand_sub">Konfiguration FW: —</span>
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
      <button data-tab="move" data-i18n="tab_move">Bewegen</button>
      <button data-tab="setup" data-i18n="tab_setup">Setup</button>
      <button data-tab="user" data-i18n="tab_user">Benutzer</button>
      <button data-tab="wifi" data-i18n="tab_wifi">WLAN</button>
      <button data-tab="update" data-i18n="tab_update">Update</button>
      <button data-tab="reset" data-i18n="tab_reset">Reset</button>
    </nav>
  </div>

  <section id="tab-status">
    <div class="card">
      <h2 data-i18n="h_rotor">Rotordaten</h2>
      <div class="grid">
        <div class="kv"><div class="k" data-i18n="k_angle">Winkel (°)</div><div class="v mono" id="vAngle">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_tempA">Außentemperatur (°C)</div><div class="v mono" id="vTempA">—</div></div>
        <div class="kv"><div class="k" data-i18n="k_tempM">Motortemperatur (°C)</div><div class="v mono" id="vTempM">—</div></div>
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

  <section id="tab-move" hidden>
    <div class="card">
      <div class="card-head">
        <h2 data-i18n="h_move">Bewegen</h2>
      </div>
      <div class="move-angle-wrap">
        <div class="kv">
          <div class="k" data-i18n="k_angle">Winkel (°)</div>
          <div class="v mono" id="movePos">—</div>
        </div>
      </div>
      <p class="errbox move-intro" data-i18n="warn_move_speed">Geschwindigkeit niedrig einstellen — Web-JOG fährt ohne Rampen (sofort Start/Stopp).</p>
      <p class="muted move-intro" data-i18n="p_move">Taste halten = fahren. Achsgrenzen (Amin/Amax) bleiben aktiv. Deadman/Positions-Timeout sind während SoftAP-JOG aus.</p>
      <div class="move-clear"></div>
      <div class="jog-row">
        <button type="button" class="btn" id="moveJogLeft" data-i18n="btn_jog_left">Jog Links</button>
        <button type="button" class="btn" id="moveJogRight" data-i18n="btn_jog_right">Jog Rechts</button>
      </div>
      <div class="field">
        <label for="movePwm" data-i18n="lbl_pwm">Geschwindigkeit (SETPWM)</label>
        <div class="pwm-val mono"><span id="movePwmVal">—</span> %</div>
        <input type="range" id="movePwm" min="0" max="100" step="0.5" value="50">
        <div class="pwm-meta">
          <span>Min <span class="mono" id="movePwmMin">—</span> %</span>
          <span>Max <span class="mono" id="movePwmMax">—</span> %</span>
        </div>
      </div>
      <p class="hint" data-i18n="hint_move">Regler = JOG-Geschwindigkeit, von Min-PWM bis Max-PWM. Nur Laufzeit, nicht gespeichert. Keine Rampen.</p>
      <div id="moveSsiZeroBox" hidden>
        <p class="hint" data-i18n="hint_ssi_zero">SSI: Rotor auf 0° ausrichten, dann Null setzen.</p>
        <div class="actions">
          <button type="button" class="btn sec" id="btnMoveEncZero" data-i18n="btn_enc_zero">Encoder auf 0 (SETENCZERO)</button>
        </div>
      </div>
    </div>
  </section>

  <section id="tab-setup" hidden>
    <div class="card">
      <h2 data-i18n="h_setup">Inbetriebnahme</h2>
      <p class="muted" data-i18n="p_setup">Nur Rotorwerte (NVS/FRAM). Nach jedem Speichern wird der Wert zurückgelesen.</p>
      <div class="steps" id="setupSteps"></div>
      <div class="actions" style="margin-bottom:.75rem">
        <span class="movebadge" id="setupMove">GETMOVE: —</span>
        <span class="muted mono" id="setupPos">pos —</span>
      </div>
      <div id="setupBody"></div>
      <div class="errbox" id="setupErr"></div>
      <div class="actions" id="setupNav">
        <button class="btn sec" id="btnSetupBack" data-i18n="btn_back">Zurück</button>
        <button class="btn" id="btnSetupNext" data-i18n="btn_next">Weiter</button>
        <button class="btn sec" id="btnSetupSkip" data-i18n="btn_skip">Überspringen</button>
      </div>
    </div>
  </section>

  <section id="tab-user" hidden>
    <div class="card">
      <h2 data-i18n="h_user">Benutzer</h2>
      <p class="muted" data-i18n="p_user">Login für die Konfigurationsseite (HTTP Basic Auth). Wird im NVS gespeichert.</p>
      <div class="field">
        <label for="userName" data-i18n="lbl_user">Benutzername</label>
        <input type="text" id="userName" autocomplete="username" maxlength="31">
      </div>
      <div class="field">
        <label for="userPass" data-i18n="lbl_pass">Neues Passwort</label>
        <input type="password" id="userPass" autocomplete="new-password" maxlength="63">
      </div>
      <div class="field">
        <label for="userPass2" data-i18n="lbl_pass2">Passwort wiederholen</label>
        <input type="password" id="userPass2" autocomplete="new-password" maxlength="63">
      </div>
      <p class="hint" data-i18n="hint_user">Mindestens 4 Zeichen. Nach dem Speichern ggf. erneut anmelden.</p>
      <div class="actions">
        <button class="btn" id="btnUserSave" data-i18n="btn_save">Speichern</button>
      </div>
    </div>
  </section>

  <section id="tab-wifi" hidden>
    <div class="card">
      <h2 data-i18n="h_wifi">WLAN (SoftAP)</h2>
      <p class="muted" data-i18n="p_wifi">Passwort des Hotspots. SSID bleibt Rotor_&lt;UID&gt;.</p>
      <div class="mono" id="wifiSsid">SSID: —</div>
      <div class="field">
        <label for="wifiPass" data-i18n="lbl_wifipass">Neuer WLAN-Schlüssel</label>
        <input type="password" id="wifiPass" autocomplete="new-password" maxlength="63">
      </div>
      <div class="field">
        <label for="wifiPass2" data-i18n="lbl_wifipass2">Neuer WLAN-Schlüssel wiederholen</label>
        <input type="password" id="wifiPass2" autocomplete="new-password" maxlength="63">
      </div>
      <p class="hint" data-i18n="hint_wifi">8–63 Zeichen (WPA2). Beim Speichern startet ein Neustart — die Verbindung bricht ab. Danach in RotorTcpBridge unter Einstellungen → Update das WLAN wieder aktivieren und mit dem neuen Schlüssel verbinden. Unter Windows ggf. das bekannte alte WLAN „vergessen“, damit der neue Schlüssel abgefragt wird.</p>
      <div class="actions">
        <button class="btn" id="btnWifiSave" data-i18n="btn_save">Speichern</button>
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
      <div class="muted" style="margin-top:.65rem" id="updMsg" data-i18n="hint_ota">Nur firmware.bin (App-OTA). Nicht factory.bin. Nach erfolgreichem Update wird der Rotor neugestartet und WLAN ist wieder aus. Einschalten wieder über RotorTcpBridge unter Einstellungen → Update.</div>
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
const $=id=>document.getElementById(id);
let lang='de';
const I18N={
de:{
  brand_title:'Rotorsteuerung DK8DE',
  brand_sub:'Konfiguration FW: —',
  badge_loading:'laden…',badge_ok:'OK',badge_offline:'offline',
  tab_status:'Status',tab_move:'Bewegen',tab_setup:'Setup',tab_user:'Benutzer',tab_wifi:'WLAN',tab_update:'Update',tab_reset:'Reset',
  h_rotor:'Rotordaten',h_move:'Bewegen',h_setup:'Inbetriebnahme',
  warn_move_speed:'Geschwindigkeit niedrig einstellen — Web-JOG fährt ohne Rampen (sofort Start/Stopp).',
  p_move:'Taste halten = fahren. Achsgrenzen (Amin/Amax) bleiben aktiv. Deadman/Positions-Timeout sind während SoftAP-JOG aus.',
  btn_jog_left:'Jog Links',btn_jog_right:'Jog Rechts',
  lbl_pwm:'Geschwindigkeit (SETPWM)',
  hint_move:'Regler = JOG-Geschwindigkeit, von Min-PWM bis Max-PWM. Nur Laufzeit, nicht gespeichert. Keine Rampen.',
  hint_ssi_zero:'SSI: Rotor auf 0° ausrichten, dann Null setzen.',
  btn_enc_zero:'Encoder auf 0 (SETENCZERO)',
  toast_enc_zero_ok:'Encoder-Null gesetzt',
  p_setup:'Nur Rotorwerte (NVS/FRAM). Nach jedem Speichern wird der Wert zurückgelesen.',
  k_angle:'Winkel (°)',k_tempA:'Außentemperatur (°C)',k_tempM:'Motortemperatur (°C)',
  k_fw:'Firmware',k_homed:'Gehomt',k_pwm:'SETPWM (%)',
  k_wind:'Wind (km/h)',k_windDir:'Windrichtung (°)',
  h_user:'Benutzer',p_user:'Login für die Konfigurationsseite (HTTP Basic Auth). Wird im NVS gespeichert.',
  lbl_user:'Benutzername',lbl_pass:'Neues Passwort',lbl_pass2:'Passwort wiederholen',
  hint_user:'Mindestens 4 Zeichen. Nach dem Speichern ggf. erneut anmelden.',
  h_wifi:'WLAN (SoftAP)',p_wifi:'Passwort des Hotspots. SSID bleibt Rotor_<UID>.',
  lbl_ssid:'SSID',lbl_wifipass:'Neuer WLAN-Schlüssel',lbl_wifipass2:'Neuer WLAN-Schlüssel wiederholen',
  hint_wifi:'8–63 Zeichen (WPA2). Beim Speichern startet ein Neustart — die Verbindung bricht ab. Danach in RotorTcpBridge unter Einstellungen → Update das WLAN wieder aktivieren und mit dem neuen Schlüssel verbinden. Unter Windows ggf. das bekannte alte WLAN „vergessen“, damit der neue Schlüssel abgefragt wird.',
  toast_user_ok:'Benutzer gespeichert',toast_wifi_ok:'WLAN-Schlüssel gespeichert — Neustart…',
  err_mismatch:'Passwörter stimmen nicht',err_pass:'Passwort ungültig',err_user:'Benutzername ungültig',
  h_update:'Firmware-Update (OTA)',k_cur:'Aktuell',k_part:'Partition',
  btn_upload:'Hochladen & flashen',hint_ota:'Nur firmware.bin (App-OTA). Nicht factory.bin. Nach erfolgreichem Update wird der Rotor neugestartet und WLAN ist wieder aus. Einschalten wieder über RotorTcpBridge unter Einstellungen → Update.',
  upd_choose:'Datei wählen',upd_uploading:'Upload…',upd_ok:'OK – Neustart…',upd_fail:'Update fehlgeschlagen',
  h_reset:'Neustart / Werkseinstellung',
  p_reset:'Neustart: SoftAP bleibt aus bis SETWLAN:1. Werkseinstellung: FRAM + NVS löschen und Neustart.',
  btn_reboot:'Neustart',btn_factory:'Werkseinstellung',
  confirm_factory:'Wirklich Werkseinstellung und Neustart?',
  toast_update_ok:'Update OK',toast_update_fail:'Update fehlgeschlagen',
  toast_reboot:'Neustart…',toast_factory:'Factory…',auth_required:'Anmeldung erforderlich',
  btn_back:'Zurück',btn_next:'Weiter',btn_skip:'Überspringen',btn_save:'Speichern',
  btn_jog_cw:'JOG CW',btn_jog_ccw:'JOG CCW',btn_jog_stop:'Stop',
  s0:'START',s1:'ENCODER',s2:'DRIVE',s3:'ENDSTOPS',s4:'WEATHER',s5:'TEMP',
  s6:'CAL',s7:'ZERO',s8:'VERIFY',s9:'ANTENNA',s10:'DONE',
  reboot_hint:'Encoder-Typ ändern löst einen Neustart aus — WLAN geht aus. Danach in RotorTcpBridge unter Einstellungen → Update das WLAN wieder aktivieren, neu verbinden und Setup fortsetzen.',
  cpr_reboot_hint:'CPR ändern (mit Endschaltern) löst einen Neustart aus — WLAN geht aus. Danach in RotorTcpBridge unter Einstellungen → Update das WLAN wieder aktivieren, neu verbinden und Setup fortsetzen. Ohne Endschalter: kein Neustart.',
  temp_hint:'Außenfühler kurz mit der Hand anfassen. Steigt die Motortemperatur statt der Außentemperatur → Zuordnung ist verdreht, dann tauschen.',
  temp_swap:'Temperaturen tauschen',temp_swapped:'Zuordnung getauscht',
  srvjog_hint:'Service-JOG: ohne Referenz/Stall (Deadman bleibt). Taste halten = fahren.'
},
en:{
  brand_title:'Rotator Control DK8DE',
  brand_sub:'Configuration FW: —',
  badge_loading:'loading…',badge_ok:'OK',badge_offline:'offline',
  tab_status:'Status',tab_move:'Move',tab_setup:'Setup',tab_user:'User',tab_wifi:'Wi‑Fi',tab_update:'Update',tab_reset:'Reset',
  h_rotor:'Rotator data',h_move:'Move',h_setup:'Commissioning',
  warn_move_speed:'Set a low speed — Web JOG has no ramps (immediate start/stop).',
  p_move:'Hold button to drive. Axis limits (Amin/Amax) stay active. Deadman/position timeout are off during SoftAP JOG.',
  btn_jog_left:'Jog Left',btn_jog_right:'Jog Right',
  lbl_pwm:'Speed (SETPWM)',
  hint_move:'Slider = JOG speed, from Min-PWM to Max-PWM. Runtime only, not stored. No ramps.',
  hint_ssi_zero:'SSI: align rotator to 0°, then set zero.',
  btn_enc_zero:'Encoder to 0 (SETENCZERO)',
  toast_enc_zero_ok:'Encoder zero set',
  p_setup:'Rotator values only (NVS/FRAM). Each save is verified with a read-back.',
  k_angle:'Angle (°)',k_tempA:'Outdoor temp (°C)',k_tempM:'Motor temp (°C)',
  k_fw:'Firmware',k_homed:'Homed',k_pwm:'SETPWM (%)',
  k_wind:'Wind (km/h)',k_windDir:'Wind direction (°)',
  h_user:'User',p_user:'Login for the config page (HTTP Basic Auth). Stored in NVS.',
  lbl_user:'Username',lbl_pass:'New password',lbl_pass2:'Repeat password',
  hint_user:'At least 4 characters. You may need to log in again after saving.',
  h_wifi:'Wi‑Fi (SoftAP)',p_wifi:'Hotspot password. SSID stays Rotor_<UID>.',
  lbl_ssid:'SSID',lbl_wifipass:'New Wi‑Fi key',lbl_wifipass2:'Repeat new Wi‑Fi key',
  hint_wifi:'8–63 characters (WPA2). Saving triggers a reboot — the connection drops. Then in RotorTcpBridge under Settings → Update re-enable Wi‑Fi and connect with the new key. On Windows you may need to “forget” the old network so Windows prompts for the new key.',
  toast_user_ok:'User saved',toast_wifi_ok:'Wi‑Fi key saved — rebooting…',
  err_mismatch:'Passwords do not match',err_pass:'Invalid password',err_user:'Invalid username',
  h_update:'Firmware update (OTA)',k_cur:'Current',k_part:'Partition',
  btn_upload:'Upload & flash',hint_ota:'firmware.bin only (app OTA). Not factory.bin. After a successful update the rotator reboots and Wi‑Fi is off again. Re-enable it in RotorTcpBridge under Settings → Update.',
  upd_choose:'Choose file',upd_uploading:'Upload…',upd_ok:'OK – rebooting…',upd_fail:'Update failed',
  h_reset:'Reboot / factory reset',
  p_reset:'Reboot: SoftAP stays off until SETWLAN:1. Factory: clear FRAM + NVS and reboot.',
  btn_reboot:'Reboot',btn_factory:'Factory reset',
  confirm_factory:'Really factory reset and reboot?',
  toast_update_ok:'Update OK',toast_update_fail:'Update failed',
  toast_reboot:'Rebooting…',toast_factory:'Factory…',auth_required:'Login required',
  btn_back:'Back',btn_next:'Next',btn_skip:'Skip',btn_save:'Save',
  btn_jog_cw:'JOG CW',btn_jog_ccw:'JOG CCW',btn_jog_stop:'Stop',
  s0:'START',s1:'ENCODER',s2:'DRIVE',s3:'ENDSTOPS',s4:'WEATHER',s5:'TEMP',
  s6:'CAL',s7:'ZERO',s8:'VERIFY',s9:'ANTENNA',s10:'DONE',
  reboot_hint:'Changing the encoder type reboots the device — Wi‑Fi turns off. Then in RotorTcpBridge under Settings → Update re-enable Wi‑Fi, reconnect, and continue setup.',
  cpr_reboot_hint:'Changing CPR (with endstops) reboots the device — Wi‑Fi turns off. Then in RotorTcpBridge under Settings → Update re-enable Wi‑Fi, reconnect, and continue setup. Without endstops: no reboot.',
  temp_hint:'Briefly warm the outdoor sensor with your hand. If motor temperature rises instead of outdoor → mapping is swapped, then swap.',
  temp_swap:'Swap temperatures',temp_swapped:'Mapping swapped',
  srvjog_hint:'Service JOG: no ref/stall (deadman stays). Hold button to move.'
}
};
function t(k){return (I18N[lang]&&I18N[lang][k])||(I18N.de[k])||k;}
var liveFw='—';
function brandSubText(){
  var base=t('brand_sub');
  var fw=liveFw||'—';
  if(base.indexOf('FW:')>=0)return base.replace(/FW:.*/,'FW: '+fw);
  return base+' FW: '+fw;
}
function applyI18n(){
  document.querySelectorAll('[data-i18n]').forEach(function(el){
    var k=el.getAttribute('data-i18n');
    if(!k)return;
    if(k==='brand_sub'){el.textContent=brandSubText();return;}
    el.textContent=t(k);
  });
  document.title=t('brand_title');
  document.querySelectorAll('.lang-btn').forEach(b=>b.classList.toggle('active',b.dataset.lang===lang));
  if(document.getElementById('tab-setup')&&!document.getElementById('tab-setup').hidden) renderSetup();
}
function toast(msg,ok){
  const el=$('toast');el.textContent=msg;el.className='toast show '+(ok?'ok':'bad');
  setTimeout(()=>el.classList.remove('show'),2800);
}
function api(path,opts){
  var u=path+(path.indexOf('?')>=0?'&':'?')+'_='+Date.now();
  var o={credentials:'same-origin'};
  if(opts){if(opts.method)o.method=opts.method;if(opts.body)o.body=opts.body;if(opts.headers)o.headers=opts.headers;}
  return fetch(u,o).then(function(r){
    if(r.status===401){toast(t('auth_required'),false);throw new Error('auth');}
    return r;
  });
}
function showTab(name){
  if(name!=='move')stopMoveJog();
  var tabs=['status','move','setup','user','wifi','update','reset'];
  for(var i=0;i<tabs.length;i++){
    var sec=document.getElementById('tab-'+tabs[i]);
    if(!sec)continue;
    if(tabs[i]===name){sec.removeAttribute('hidden');sec.style.display='';}
    else{sec.setAttribute('hidden','');sec.style.display='none';}
  }
  document.querySelectorAll('nav button[data-tab]').forEach(function(b){
    b.classList.toggle('active',b.getAttribute('data-tab')===name);
  });
  var wrap=$('navWrap');if(wrap)wrap.classList.remove('open');
  var burger=$('navBurger');if(burger)burger.setAttribute('aria-expanded','false');
  if(name==='update')loadUpdInfo();
  if(name==='setup'){setupStep=0;loadSetupState().then(renderSetup);}
  if(name==='user')loadUser();
  if(name==='wifi')loadWifi();
  if(name==='move')syncMoveFromLive();
}
function credErr(code){
  if(code==='mismatch')return t('err_mismatch');
  if(code==='user')return t('err_user');
  if(code==='pass')return t('err_pass');
  return code||'err';
}
function loadUser(){
  api('/api/user').then(function(r){return r.json();}).then(function(j){
    var el=$('userName');if(el)el.value=j.user||'';
    var p1=$('userPass');if(p1)p1.value='';
    var p2=$('userPass2');if(p2)p2.value='';
  }).catch(function(){});
}
function loadWifi(){
  api('/api/wifi').then(function(r){return r.json();}).then(function(j){
    setTxt('wifiSsid','SSID: '+(j.ssid||'-'));
    var p1=$('wifiPass');if(p1)p1.value='';
    var p2=$('wifiPass2');if(p2)p2.value='';
  }).catch(function(){});
}
function setConn(ok){
  var d=$('connDot');if(d)d.className='dot '+(ok?'on':'off');
  var b=$('connBadge');if(b)b.textContent=ok?t('badge_ok'):t('badge_offline');
}
function fmt(n,d){if(n==null||!isFinite(n))return '-';return Number(n).toFixed(d);}
function setTxt(id,v){var el=$(id);if(el)el.textContent=v;}
function setHtml(id,v){var el=$(id);if(el)el.innerHTML=v;}
function showEl(id,on){
  var el=$(id);if(!el)return;
  if(on){el.removeAttribute('hidden');el.style.display='';}
  else{el.setAttribute('hidden','');el.style.display='none';}
}
var movePwmBusy=false;
var moveJogTimer=null;
var lastLive=null;
function syncMoveFromLive(j){
  j=j||lastLive;if(!j)return;
  var mn=Number(j.minPwm);if(!isFinite(mn))mn=0;
  var mx=Number(j.maxPwm);if(!isFinite(mx))mx=100;
  if(mx<mn){var tmp=mn;mn=mx;mx=tmp;}
  if(mx<=mn)mx=mn+1;
  var sl=$('movePwm');
  if(sl){
    sl.min=String(mn);sl.max=String(mx);
    if(!movePwmBusy){
      var cur=Number(j.pwm);if(!isFinite(cur))cur=mn;
      if(cur<mn)cur=mn;if(cur>mx)cur=mx;
      sl.value=String(cur);
      setTxt('movePwmVal',fmt(cur,1));
    }
  }
  setTxt('movePwmMin',fmt(mn,1));
  setTxt('movePwmMax',fmt(mx,1));
  setTxt('movePos',fmt(j.angle,1));
  showEl('moveSsiZeroBox',Number(j.encType)===3);
}
var moveJogDir=0;
var moveJogPtrId=null;
function stopMoveJog(){
  if(moveJogTimer){clearInterval(moveJogTimer);moveJogTimer=null;}
  var was=moveJogDir;
  moveJogDir=0;
  moveJogPtrId=null;
  if(was!==0){
    api('/api/setup/jog',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({dir:0,limits:0})}).catch(function(){});
  }
}
function startMoveJog(dir){
  if(dir!==1&&dir!==2)return;
  // Gleiche Richtung: kein Stopp/Neustart (sonst Ruckeln ohne Rampen)
  if(moveJogDir===dir&&moveJogTimer)return;
  if(moveJogTimer){clearInterval(moveJogTimer);moveJogTimer=null;}
  moveJogDir=dir;
  function tick(){
    if(moveJogDir!==dir)return;
    api('/api/setup/jog',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({dir:dir,limits:0})}).catch(function(){});
  }
  tick();
  moveJogTimer=setInterval(tick,100);
}
function bindMoveControls(){
  function hold(btn,dir){
    if(!btn)return;
    btn.addEventListener('pointerdown',function(e){
      if(e.button!=null&&e.button!==0)return;
      e.preventDefault();
      try{btn.setPointerCapture(e.pointerId);}catch(err){}
      moveJogPtrId=e.pointerId;
      startMoveJog(dir);
    });
    function endPtr(e){
      if(moveJogPtrId!=null&&e.pointerId!==moveJogPtrId)return;
      try{btn.releasePointerCapture(e.pointerId);}catch(err){}
      stopMoveJog();
    }
    btn.addEventListener('pointerup',endPtr);
    btn.addEventListener('pointercancel',endPtr);
    // mouseleave absichtlich NICHT — Capture haelt bis pointerup
  }
  hold($('moveJogLeft'),1);
  hold($('moveJogRight'),2);
  window.addEventListener('blur',stopMoveJog);
  var bz=$('btnMoveEncZero');
  if(bz)bz.onclick=function(){
    stopMoveJog();
    applyCmd('SETENCZERO','').then(function(){
      toast(t('toast_enc_zero_ok'),true);
      return pollLive();
    }).catch(function(err){toast(String(err&&err.message||err||'ERR'),false);});
  };
  var sl=$('movePwm');
  if(!sl)return;
  var pwmTimer=null;
  function sendPwm(){
    var v=Number(sl.value);
    if(!isFinite(v))return;
    setTxt('movePwmVal',fmt(v,1));
    applyCmd('SETPWM',fmt(v,1)).catch(function(){});
  }
  sl.addEventListener('input',function(){
    movePwmBusy=true;
    setTxt('movePwmVal',fmt(Number(sl.value),1));
    if(pwmTimer)clearTimeout(pwmTimer);
    pwmTimer=setTimeout(function(){sendPwm();movePwmBusy=false;},120);
  });
  sl.addEventListener('change',function(){sendPwm();movePwmBusy=false;});
}
function pollLive(){
  api('/api/live').then(r=>r.json()).then(function(j){
    lastLive=j;
    setTxt('vAngle',fmt(j.angle,1));
    setTxt('vTempA',fmt(j.tempA,1));
    setTxt('vTempM',fmt(j.tempM,1));
    setTxt('vFw',j.fw||'-');
    setTxt('vHomed',(j.homed!=null)?String(j.homed):'-');
    setTxt('vPwm',fmt(j.pwm,1));
    if(j.fw){liveFw=String(j.fw);var bs=$('brandSub');if(bs)bs.textContent=brandSubText();}
    var windOn=!!j.windEn;
    showEl('kvWindKmh',windOn);showEl('kvWindDir',windOn);
    if(windOn){setTxt('vWindKmh',fmt(j.windKmh,1));setTxt('vWindDir',fmt(j.windDir,1));}
    setHtml('vAp',(j.ssid||'-')+'<br>'+(j.ip||'-'));
    setTxt('vUid',j.uid||'-');
    var moveSec=$('tab-move');
    if(moveSec&&!moveSec.hidden)syncMoveFromLive(j);
    setConn(true);
  }).catch(function(){setConn(false);});
}

/* ===== Setup wizard ===== */
var setupStep=0;
var setupState=null;
var setupPollTimer=null;
var jogTimer=null;
var STEP_KEYS=['s0','s1','s2','s3','s4','s5','s6','s7','s8','s9','s10'];
function encType(){return setupState?Number(setupState.encType||1):1;}
function endstc(){return setupState?Number(setupState.endstc||0):0;}
function rotorType(){return setupState?Number(setupState.rotorType||1):1;}
function stepVisible(i){
  var e=encType();
  if(i===4) return e===1||e===2; // weather
  if(i===9) return rotorType()===1; // antenna az only
  return true;
}
function nextVisible(from,dir){
  var i=from;
  for(var n=0;n<12;n++){
    i+=dir;
    if(i<0)return 0;
    if(i>10)return 10;
    if(stepVisible(i))return i;
  }
  return from;
}
function loadSetupState(){
  return api('/api/setup/state').then(r=>r.json()).then(function(j){
    setupState=j;
    var mb=$('setupMove');
    if(mb){mb.textContent='GETMOVE: '+(j.move?1:0);mb.className='movebadge'+(j.move?' on':'');}
    setTxt('setupPos','pos '+fmt(j.pos,2)+'° | ref '+(j.ref?1:0));
    return j;
  });
}
function applyCmd(cmd,params){
  return api('/api/setup/apply',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:cmd,params:params==null?'':String(params)})
  }).then(r=>r.json()).then(function(j){
    if(!j.ok){throw new Error(j.err||'ERR');}
    return j;
  });
}
function setSetupErr(msg){var el=$('setupErr');if(el)el.textContent=msg||'';}
function field(id,label,val){
  return '<div class="field"><label for="'+id+'">'+label+'</label>'+
    '<input type="text" id="'+id+'" value="'+(val==null?'':String(val).replace(/"/g,'&quot;'))+'"></div>';
}
function jogBarHtml(){
  return '<p class="hint">'+t('srvjog_hint')+'</p>'+
    '<div class="actions">'+
    '<button type="button" class="btn" id="jogCw">'+t('btn_jog_cw')+'</button>'+
    '<button type="button" class="btn" id="jogCcw">'+t('btn_jog_ccw')+'</button>'+
    '<button type="button" class="btn sec" id="jogStop">'+t('btn_jog_stop')+'</button>'+
    '</div>';
}
function bindJog(){
  function start(dir){
    stopJog();
    function tick(){api('/api/setup/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({dir:dir})}).catch(function(){});}
    tick();
    jogTimer=setInterval(tick,200);
  }
  function stopJog(){
    if(jogTimer){clearInterval(jogTimer);jogTimer=null;}
    api('/api/setup/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({dir:0})}).catch(function(){});
  }
  var cw=$('jogCw'),ccw=$('jogCcw'),st=$('jogStop');
  if(cw){['mousedown','touchstart'].forEach(ev=>cw.addEventListener(ev,e=>{e.preventDefault();start(2);},{passive:false}));
    ['mouseup','mouseleave','touchend','touchcancel'].forEach(ev=>cw.addEventListener(ev,stopJog));}
  if(ccw){['mousedown','touchstart'].forEach(ev=>ccw.addEventListener(ev,e=>{e.preventDefault();start(1);},{passive:false}));
    ['mouseup','mouseleave','touchend','touchcancel'].forEach(ev=>ccw.addEventListener(ev,stopJog));}
  if(st)st.onclick=stopJog;
}
function renderSetup(){
  var stepsEl=$('setupSteps');
  if(stepsEl){
    var html='';
    for(var i=0;i<=10;i++){
      if(!stepVisible(i))continue;
      var cls='step-pill'+(i===setupStep?' on':(i<setupStep?' done':''));
      html+='<span class="'+cls+'">'+t(STEP_KEYS[i])+'</span>';
    }
    stepsEl.innerHTML=html;
  }
  var body=$('setupBody');
  if(!body)return;
  var s=setupState||{};
  var e=encType();
  var html='';
  if(setupStep===0){
    html+='<div class="grid"><div class="kv"><div class="k">FW</div><div class="v mono" style="font-size:1.2em">'+(s.fw||'-')+'</div></div>'+
      '<div class="kv"><div class="k">ERR</div><div class="v mono" style="font-size:1.2em">'+(s.err||'0')+'</div></div>'+
      '<div class="kv"><div class="k">WARN</div><div class="v mono" style="font-size:1.1em">'+(s.warn||'0')+'</div></div></div>'+
      '<p class="hint">ENCTYPE='+e+' ENDSTC='+endstc()+' WIND='+(s.windEn?1:0)+' SWAP='+(s.swapTmp?1:0)+'</p>'+
      (String(s.err||'0')!=='0'?'<p class="errbox">Fehler vorhanden – ggf. SETREF:0 zum Quittieren.</p>':'');
  } else if(setupStep===1){
    html+='<div class="field"><label>Encoder</label><select id="fEnc">'+
      '<option value="1"'+(e===1?' selected':'')+'>1 Motorachse</option>'+
      '<option value="2"'+(e===2?' selected':'')+'>2 Ring/Abtrieb</option>'+
      '<option value="3"'+(e===3?' selected':'')+'>3 SSI absolut</option>'+
      '<option value="4"'+(e===4?' selected':'')+'>4 Inklinometer</option></select></div>'+
      '<p class="hint">'+t('reboot_hint')+'</p>'+
      '<div class="actions"><button class="btn" id="btnSaveEnc">'+t('btn_save')+'</button></div>';
  } else if(setupStep===2){
    html+=field('fRty','SETROTORTYPE (1 AZ / 2 EL90 / 3 EL180)',s.rotorType)+
      field('fAmin','SETBEGINDG',fmt(s.amin,2))+
      field('fAmax','SETMAXDG',fmt(s.amax,2))+
      field('fMinPwm','SETMINPWM %',fmt(s.minPwm,1))+
      field('fIwarn','SETIWARN mV',s.iwarn)+
      field('fImax','SETIMAX mV',s.imax)+
      field('fHomePwm','SETHOMEPWM %',fmt(s.homePwm,1))+
      field('fSeekPwm','SETHOMESEEKPPWM %',fmt(s.homeSeekPwm,1))+
      field('fRamp','SETRAMP °',fmt(s.ramp,1))+
      field('fArrtol','SETARRTOL °',fmt(s.arrtol,2))+
      '<p class="hint">Min-PWM für JOG ohne Endschalter; ARRTOL bei Typ 3/4 großzügiger (z.B. 1,00).</p>'+
      '<div class="actions"><button class="btn" id="btnSaveDrive">'+t('btn_save')+'</button></div>';
  } else if(setupStep===3){
    html+='<div class="field"><label>SETENDSTC</label><select id="fEnd">'+
      '<option value="0"'+(endstc()===0?' selected':'')+'>0 keine</option>'+
      (e!==4?'<option value="1"'+(endstc()===1?' selected':'')+'>1 ein Kontakt</option>':'')+
      '<option value="2"'+(endstc()===2?' selected':'')+'>2 zwei Kontakte</option></select></div>'+
      '<div class="actions"><button class="btn" id="btnSaveEnd">'+t('btn_save')+'</button></div>';
  } else if(setupStep===4){
    html+='<div class="field"><label>SETWINDENABLE</label><select id="fWind">'+
      '<option value="0"'+(s.windEn? '':' selected')+'>0 aus</option>'+
      '<option value="1"'+(s.windEn?' selected':'')+'>1 ein</option></select></div>'+
      field('fWindOf','SETWINDDIROF °',fmt(s.windDirOf,1))+
      '<p class="hint">Anemo '+fmt(s.windKmh,1)+' km/h · Dir '+fmt(s.windDir,1)+'°</p>'+
      '<div class="actions"><button class="btn" id="btnSaveWind">'+t('btn_save')+'</button></div>';
  } else if(setupStep===5){
    html+='<div class="grid"><div class="kv"><div class="k">'+t('k_tempA')+'</div><div class="v mono" style="font-size:1.4em" id="liveTA">'+fmt(s.tempA,1)+'</div></div>'+
      '<div class="kv"><div class="k">'+t('k_tempM')+'</div><div class="v mono" style="font-size:1.4em" id="liveTM">'+fmt(s.tempM,1)+'</div></div>'+
      '<div class="kv"><div class="k">'+t('temp_swapped')+'</div><div class="v mono" style="font-size:1.4em">'+(s.swapTmp?(lang==='en'?'yes':'ja'):(lang==='en'?'no':'nein'))+'</div></div></div>'+
      '<p class="hint">'+t('temp_hint')+'</p>'+
      '<div class="actions"><button class="btn" id="btnSwap">'+t('temp_swap')+'</button></div>';
  } else if(setupStep===6){
    if(e===1||e===2){
      html+=field('fCpr',e===1?'SETENCCAX':'SETENCCRI',s.cpr)+
        '<p class="hint">Counts '+s.encCount+' · CPR einmessen: Marke → JOG bekannten Winkel → Counts → berechnen.</p>'+
        (endstc()===0?jogBarHtml():'<p class="hint">Mit Endschaltern: Homing im nächsten Schritt.</p><p class="hint">'+t('cpr_reboot_hint')+'</p>')+
        '<div class="actions"><button class="btn" id="btnSaveCpr">'+t('btn_save')+'</button></div>';
    } else if(e===3){
      html+='<p class="hint">SSI: kein CPR. Null im nächsten Schritt mit SETENCZERO.</p>'+
        field('fDgcal','SETDGCAL °',fmt(s.dgcal,2))+
        '<div class="actions"><button class="btn" id="btnSaveDgcal">'+t('btn_save')+'</button></div>';
    } else {
      html+=field('fIncCal','SETINCCAL',s.incCal||'')+
        field('fIncSwap','SETINCSWAPXY 0/1',s.incSwapXY||'0')+
        field('fIncMount','SETINCMOUNT 0/90/180/270',s.incMount||'0')+
        field('fIncLim','SETINCLIM180 0/1',s.incLim180||'0')+
        field('fIncFilt','SETINCFILT',s.incFilt||'')+
        field('fIncInvR','SETINCINVREF 0/1',s.incInvRef||'0')+
        field('fIncInvS','SETINCINVSEN 0/1',s.incInvSen||'0')+
        field('fIncInvO','SETINCINVROT 0/1',s.incInvRot||'0')+
        field('fIncWlan','SETINCWLAN 0/1',s.incWlan||'0')+
        '<div class="actions"><button class="btn" id="btnSaveInc">'+t('btn_save')+'</button></div>';
    }
  } else if(setupStep===7){
    if(e===1||e===2){
      html+='<p class="hint">'+(endstc()>0?'SETREF:1 startet Homing.':'Mechanisch auf Null stellen, dann SETREF:1 (manuelle Referenz).')+'</p>'+
        (endstc()===0?jogBarHtml():'')+
        '<div class="actions"><button class="btn" id="btnSetRef">SETREF:1</button>'+
        '<button class="btn sec" id="btnAckErr">SETREF:0 (ACK ERR)</button></div>'+
        '<p class="hint">REF='+(s.ref?1:0)+' HOMING='+(s.homing?1:0)+'</p>';
    } else if(e===3){
      html+='<p class="hint">Rotor auf 0° stellen, dann SETENCZERO.</p>'+
        '<div class="actions"><button class="btn" id="btnEncZero">SETENCZERO</button></div>'+
        '<p class="hint">pos '+fmt(s.pos,2)+' ref '+(s.ref?1:0)+'</p>';
    } else {
      html+='<p class="hint">Inklinometer: Null über SETINCCAL (voriger Schritt). GETREF bei gültigem Winkel.</p>'+
        '<p class="okbox">REF='+(s.ref?1:0)+' pos='+fmt(s.pos,2)+'</p>';
    }
  } else if(setupStep===8){
    html+=field('fTestPos','SETPOSDG Testwinkel',fmt((Number(s.amin)+Number(s.amax))/2,2))+
      '<div class="actions"><button class="btn" id="btnTestPos">SETPOSDG</button>'+
      '<button class="btn sec" id="btnStop">STOP</button></div>'+
      '<p class="hint">MOVE='+(s.move?1:0)+' pos='+fmt(s.pos,2)+' ref='+(s.ref?1:0)+' err='+(s.err||'0')+'</p>';
  } else if(setupStep===9){
    html+=field('fAnt1','SETANTNAME1 (max 9)',s.ant1||'')+
      field('fAnt2','SETANTNAME2',s.ant2||'')+
      field('fAnt3','SETANTNAME3',s.ant3||'')+
      '<p class="hint">Zeichen # : $ verboten.</p>'+
      '<div class="actions"><button class="btn" id="btnSaveAnt">'+t('btn_save')+'</button></div>';
  } else {
    html+='<div class="okbox">Fertig.</div><ul class="muted">'+
      '<li>Encoder '+e+' · Endstc '+endstc()+' · Max '+fmt(s.amax,2)+'°</li>'+
      '<li>MinPWM '+fmt(s.minPwm,1)+'% · Wind '+(s.windEn?1:0)+' · Swap '+(s.swapTmp?1:0)+'</li>'+
      '<li>Ref '+(s.ref?1:0)+' · Ant '+[s.ant1,s.ant2,s.ant3].filter(Boolean).join(' / ')+'</li></ul>'+
      '<p class="hint">Werte liegen im Rotor (Typ 4 teils Inklinometer-NVS).</p>';
  }
  body.innerHTML=html;
  setSetupErr('');
  bindSetupButtons();
  bindJog();
}
function bindSetupButtons(){
  var se=$('btnSaveEnc');
  if(se)se.onclick=function(){
    var v=$('fEnc').value;
    applyCmd('SETENCTYPE',v).then(function(){
      toast(t('reboot_hint'),true);
      return loadSetupState().then(renderSetup);
    }).catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sd=$('btnSaveDrive');
  if(sd)sd.onclick=function(){
    var chain=[
      ['SETROTORTYPE',$('fRty').value],
      ['SETBEGINDG',$('fAmin').value],
      ['SETMAXDG',$('fAmax').value],
      ['SETMINPWM',$('fMinPwm').value],
      ['SETIWARN',$('fIwarn').value],
      ['SETIMAX',$('fImax').value],
      ['SETHOMEPWM',$('fHomePwm').value],
      ['SETHOMESEEKPPWM',$('fSeekPwm').value],
      ['SETRAMP',$('fRamp').value],
      ['SETARRTOL',$('fArrtol').value]
    ];
    var p=Promise.resolve();
    chain.forEach(function(c){p=p.then(function(){return applyCmd(c[0],c[1]);});});
    p.then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
     .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var send=$('btnSaveEnd');
  if(send)send.onclick=function(){
    applyCmd('SETENDSTC',$('fEnd').value).then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sw=$('btnSaveWind');
  if(sw)sw.onclick=function(){
    applyCmd('SETWINDENABLE',$('fWind').value)
      .then(function(){return applyCmd('SETWINDDIROF',$('fWindOf').value);})
      .then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var bs=$('btnSwap');
  if(bs)bs.onclick=function(){
    var nv=setupState&&setupState.swapTmp?0:1;
    applyCmd('SETSWAPTEMP',nv).then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sc=$('btnSaveCpr');
  if(sc)sc.onclick=function(){
    var cmd=encType()===1?'SETENCCAX':'SETENCCRI';
    var willReboot=endstc()>0;
    applyCmd(cmd,$('fCpr').value).then(function(){
      toast(willReboot?t('cpr_reboot_hint'):'OK',true);
      return loadSetupState().then(renderSetup);
    }).catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sdg=$('btnSaveDgcal');
  if(sdg)sdg.onclick=function(){
    applyCmd('SETDGCAL',$('fDgcal').value).then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var si=$('btnSaveInc');
  if(si)si.onclick=function(){
    var ops=[
      ['SETINCCAL',$('fIncCal').value],['SETINCSWAPXY',$('fIncSwap').value],
      ['SETINCMOUNT',$('fIncMount').value],['SETINCLIM180',$('fIncLim').value],
      ['SETINCFILT',$('fIncFilt').value],['SETINCINVREF',$('fIncInvR').value],
      ['SETINCINVSEN',$('fIncInvS').value],['SETINCINVROT',$('fIncInvO').value],
      ['SETINCWLAN',$('fIncWlan').value]
    ];
    var p=Promise.resolve();
    ops.forEach(function(c){if(c[1]!=='')p=p.then(function(){return applyCmd(c[0],c[1]);});});
    p.then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
     .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sr=$('btnSetRef');
  if(sr)sr.onclick=function(){
    applyCmd('SETREF','1').then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sa=$('btnAckErr');
  if(sa)sa.onclick=function(){
    applyCmd('SETREF','0').then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var ez=$('btnEncZero');
  if(ez)ez.onclick=function(){
    applyCmd('SETENCZERO','').then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var tp=$('btnTestPos');
  if(tp)tp.onclick=function(){
    applyCmd('SETPOSDG',$('fTestPos').value).then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var st=$('btnStop');
  if(st)st.onclick=function(){
    applyCmd('STOP','').then(function(){return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
  var sa2=$('btnSaveAnt');
  if(sa2)sa2.onclick=function(){
    applyCmd('SETANTNAME1',$('fAnt1').value)
      .then(function(){return applyCmd('SETANTNAME2',$('fAnt2').value);})
      .then(function(){return applyCmd('SETANTNAME3',$('fAnt3').value);})
      .then(function(){toast('OK',true);return loadSetupState().then(renderSetup);})
      .catch(function(err){setSetupErr(String(err.message||err));});
  };
}
function startSetupPoll(){
  if(setupPollTimer)clearInterval(setupPollTimer);
  setupPollTimer=setInterval(function(){
    var sec=$('tab-setup');
    if(!sec||sec.hidden)return;
    loadSetupState().then(function(j){
      var ta=$('liveTA'),tm=$('liveTM');
      if(ta)ta.textContent=fmt(j.tempA,1);
      if(tm)tm.textContent=fmt(j.tempM,1);
    }).catch(function(){});
  },2000);
}
$('btnSetupBack').onclick=function(){setupStep=nextVisible(setupStep,-1);renderSetup();};
$('btnSetupNext').onclick=function(){
  if(setupStep>=10){showTab('status');return;}
  setupStep=nextVisible(setupStep,1);
  loadSetupState().then(renderSetup);
};
$('btnSetupSkip').onclick=function(){setupStep=nextVisible(setupStep,1);renderSetup();};

function loadUpdInfo(){
  api('/api/update/info').then(r=>r.json()).then(function(j){
    setTxt('updCur',j.fw||'-');
    var run=(j.partition||'-')+(j.size?(' · '+j.size):'');
    var ota=j.ota?(j.ota+(j.otaSize?(' · '+j.otaSize):'')):'';
    setHtml('updPart',ota? (run+'<br>'+ota) : run);
  }).catch(function(){});
}
var btnUpload=$('btnUpload');
if(btnUpload)btnUpload.onclick=function(){
  var inp=$('fwFile');var f=inp&&inp.files&&inp.files[0];
  if(!f){toast(t('upd_choose'),false);return;}
  if(/factory/i.test(f.name)){setTxt('updMsg',t('upd_fail')+' (not_app)');toast(t('toast_update_fail'),false);return;}
  setTxt('updMsg',t('upd_uploading'));
  var bar=$('updBar');if(bar)bar.style.width='0%';
  var xhr=new XMLHttpRequest();xhr.open('POST','/api/update');xhr.withCredentials=true;
  xhr.upload.onprogress=function(ev){if(ev.lengthComputable&&bar)bar.style.width=Math.round(100*ev.loaded/ev.total)+'%';};
  xhr.onload=function(){
    var ok=false,err='';try{var j=JSON.parse(xhr.responseText);ok=!!j.ok;err=j.err||'';}catch(e){}
    if(ok){setTxt('updMsg',t('upd_ok'));toast(t('toast_update_ok'),true);setTimeout(function(){api('/api/reboot',{method:'POST'});},600);}
    else{setTxt('updMsg',t('upd_fail')+(err?(' ('+err+')'):(' HTTP '+xhr.status)));toast(t('toast_update_fail'),false);}
  };
  xhr.onerror=function(){setTxt('updMsg',t('upd_fail'));toast(t('toast_update_fail'),false);};
  var fd=new FormData();fd.append('update',f);xhr.send(fd);
};
$('btnUserSave').onclick=function(){
  var u=($('userName')&&$('userName').value||'').trim();
  var p=$('userPass')&&$('userPass').value||'';
  var p2=$('userPass2')&&$('userPass2').value||'';
  if(p!==p2){toast(t('err_mismatch'),false);return;}
  if(u.length<1||u.length>31){toast(t('err_user'),false);return;}
  if(p.length<4||p.length>63){toast(t('err_pass'),false);return;}
  api('/api/user',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({user:u,pass:p,pass2:p2})}).then(function(r){return r.json().then(function(j){return {ok:r.ok,j:j};});})
    .then(function(x){
      if(x.ok&&x.j&&x.j.ok){toast(t('toast_user_ok'),true);if($('userPass'))$('userPass').value='';if($('userPass2'))$('userPass2').value='';}
      else toast(credErr(x.j&&x.j.err),false);
    }).catch(function(){toast(t('err_pass'),false);});
};
$('btnWifiSave').onclick=function(){
  var p=$('wifiPass')&&$('wifiPass').value||'';
  var p2=$('wifiPass2')&&$('wifiPass2').value||'';
  if(p!==p2){toast(t('err_mismatch'),false);return;}
  if(p.length<8||p.length>63){toast(t('err_pass'),false);return;}
  api('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({pass:p,pass2:p2})}).then(function(r){return r.json().then(function(j){return {ok:r.ok,j:j};});})
    .then(function(x){
      if(x.ok&&x.j&&x.j.ok){toast(t('toast_wifi_ok'),true);if($('wifiPass'))$('wifiPass').value='';if($('wifiPass2'))$('wifiPass2').value='';}
      else toast(credErr(x.j&&x.j.err),false);
    }).catch(function(){toast(t('err_pass'),false);});
};
$('btnReboot').onclick=function(){toast(t('toast_reboot'),true);api('/api/reboot',{method:'POST'}).catch(function(){});};
$('btnFactory').onclick=function(){
  if(!confirm(t('confirm_factory')))return;
  toast(t('toast_factory'),true);api('/api/factory',{method:'POST'}).catch(function(){});
};
document.querySelectorAll('nav button[data-tab]').forEach(function(btn){
  btn.onclick=function(){showTab(btn.getAttribute('data-tab'));};
});
$('navBurger').onclick=function(){
  var w=$('navWrap');if(!w)return;
  var open=!w.classList.contains('open');
  if(open)w.classList.add('open');else w.classList.remove('open');
  $('navBurger').setAttribute('aria-expanded',open?'true':'false');
};
document.querySelectorAll('.lang-btn').forEach(function(btn){
  btn.onclick=function(){lang=btn.getAttribute('data-lang');applyI18n();};
});
bindMoveControls();
applyI18n();
showTab('status');
pollLive();
setInterval(pollLive,500);
startSetupPoll();
</script>
</body>
</html>
)HTML";
