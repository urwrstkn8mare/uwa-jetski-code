const API = '';
const ONDEVICE = window.ONDEVICE !== false;

let es = null;
let calMode = false;

function $(id) { return document.getElementById(id); }
function setText(id, text) { const el = $(id); if (el) el.textContent = text; }

function connectSSE() {
    if (es) { es.close(); es = null; }
    es = new EventSource(API + '/api/events');
    es.onmessage = (e) => {
        if (!e.data || e.data[0] !== '{') return;
        const d = JSON.parse(e.data);
        if ('cal_mode' in d) {
            calMode = d.cal_mode;
            setText('headerCal', calMode ? 'CAL' : '');
        }
        if ('elevon_left_deg' in d) {
            setText('tElevonL', Number(d.elevon_left_deg).toFixed(2) + '°');
        }
        if ('elevon_right_deg' in d) {
            setText('tElevonR', Number(d.elevon_right_deg).toFixed(2) + '°');
        }
        if (d.type === 'servo' && d.handle !== undefined) {
            updateServoCard(d.handle, d);
        }
    };
    es.onerror = () => {
        $('connLed').classList.add('error');
        if (es) { es.close(); es = null; }
        setTimeout(connectSSE, 2000);
    };
    es.onopen = () => {
        $('connLed').classList.remove('error');
    };
}

document.querySelectorAll('.tabs button').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tabs button').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
        btn.classList.add('active');
        document.getElementById('panel-' + btn.dataset.tab).classList.add('active');
        if (btn.dataset.tab === 'servos') loadServos();
        if (btn.dataset.tab === 'config') loadConfig();
        if (btn.dataset.tab === 'data') openDataTab();
        if (btn.dataset.tab === 'offsets') { loadOffsets(); startAttitudePoll(); startTimePoll(); }
        else { stopAttitudePoll(); stopTimePoll(); }
    });
});

$('chkHeightEnabled').addEventListener('change', () => {
    $('heightEnabledLabel').textContent = $('chkHeightEnabled').checked ? 'Loop ON' : 'Loop OFF';
});

$('btnSaveConfig').addEventListener('click', () => {
    const fields = [
        'height_kp','height_ki','height_kd',
        'pitch_kp','pitch_ki','pitch_kd',
        'roll_kp','roll_ki','roll_kd',
        'rudder_exponent_x100','rudder_max_roll_deg',
        'elevon_max_diff_deg','pitch_target_max_deg','height_target_cm'
    ];
    const data = {};
    fields.forEach(f => { data[f] = parseFloat($('cfg_'+f).value) || 0; });
    data.height_enabled = $('chkHeightEnabled').checked;
    fetch(API + '/api/config', {
        method: 'PUT',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify(data)
    }).then(r => {
        $('saveMsg').textContent = r.ok ? 'Saved!' : 'Error';
        setTimeout(() => $('saveMsg').textContent = '', 2000);
    }).catch(() => {
        $('saveMsg').textContent = 'Error';
        setTimeout(() => $('saveMsg').textContent = '', 2000);
    });
});

function populateConfigForm(cfg) {
    const fields = [
        'height_kp','height_ki','height_kd',
        'pitch_kp','pitch_ki','pitch_kd',
        'roll_kp','roll_ki','roll_kd',
        'rudder_exponent_x100','rudder_max_roll_deg',
        'elevon_max_diff_deg','pitch_target_max_deg','height_target_cm'
    ];
    fields.forEach(f => {
        const el = $('cfg_'+f);
        if (el && cfg[f] !== undefined) el.value = cfg[f];
    });
    if (cfg.height_enabled !== undefined) {
        $('chkHeightEnabled').checked = cfg.height_enabled;
        $('heightEnabledLabel').textContent = cfg.height_enabled ? 'Loop ON' : 'Loop OFF';
    }
}

function loadConfig() {
    fetch(API + '/api/config').then(r => r.json()).then(populateConfigForm).catch(() => {});
}

let attitudePoll = null;
function startAttitudePoll() {
    if (attitudePoll) return;
    const tick = () => {
        fetch(API + '/api/attitude').then(r => r.json()).then(d => {
            $('liveAttPitch').textContent = Number(d.pitch_deg).toFixed(2) + '°';
            $('liveAttRoll').textContent  = Number(d.roll_deg).toFixed(2) + '°';
            $('liveRudder').textContent   = d.rudder_ready ? Number(d.rudder_deg).toFixed(2) + '°' : '--';
        }).catch(() => {});
    };
    tick();
    attitudePoll = setInterval(tick, 250);
}
function stopAttitudePoll() {
    if (attitudePoll) { clearInterval(attitudePoll); attitudePoll = null; }
}

function populateOffsetsForm(cfg) {
    if (cfg.pitch_offset_deg_x10 !== undefined) {
        $('off_pitch_deg').value = (cfg.pitch_offset_deg_x10 / 10).toFixed(1);
    }
    if (cfg.roll_offset_deg_x10 !== undefined) {
        $('off_roll_deg').value = (cfg.roll_offset_deg_x10 / 10).toFixed(1);
    }
}

function loadOffsets() {
    fetch(API + '/api/imu').then(r => r.json()).then(populateOffsetsForm).catch(() => {});
}

$('btnSaveOffsets').addEventListener('click', () => {
    const data = {
        pitch_offset_deg_x10: Math.round((parseFloat($('off_pitch_deg').value) || 0) * 10),
        roll_offset_deg_x10:  Math.round((parseFloat($('off_roll_deg').value)  || 0) * 10),
    };
    fetch(API + '/api/imu', {
        method: 'PUT',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify(data)
    }).then(r => {
        $('offsetsSaveMsg').textContent = r.ok ? 'Saved!' : 'Error';
        setTimeout(() => $('offsetsSaveMsg').textContent = '', 2000);
    }).catch(() => {
        $('offsetsSaveMsg').textContent = 'Error';
        setTimeout(() => $('offsetsSaveMsg').textContent = '', 2000);
    });
});

$('btnResetOffsets').addEventListener('click', () => {
    if (!confirm('Reset IMU offsets to defaults?\n(Form will be populated; click Save Offsets to persist.)')) return;
    fetch(API + '/api/imu/defaults').then(r => r.json()).then(cfg => {
        populateOffsetsForm(cfg);
        $('offsetsSaveMsg').textContent = 'Defaults loaded';
        setTimeout(() => $('offsetsSaveMsg').textContent = '', 2000);
    }).catch(() => {
        $('offsetsSaveMsg').textContent = 'Error';
        setTimeout(() => $('offsetsSaveMsg').textContent = '', 2000);
    });
});

let timePoll = null;
function startTimePoll() {
    if (timePoll) return;
    const tick = () => {
        fetch(API + '/api/time').then(r => r.json()).then(d => {
            $('timeNow').textContent = d.epoch_s ? new Date(d.epoch_s * 1000).toLocaleString() : '--';
            $('timeSource').textContent = d.source === 'none' ? 'not set' : d.source.toUpperCase();
        }).catch(() => {});
    };
    tick();
    timePoll = setInterval(tick, 1000);
}
function stopTimePoll() {
    if (timePoll) { clearInterval(timePoll); timePoll = null; }
}

$('btnSyncTime').addEventListener('click', () => {
    fetch(API + '/api/time', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({epoch_s: Math.floor(Date.now() / 1000)})
    }).then(r => {
        $('timeSyncMsg').textContent = r.ok ? 'Synced!' : 'Error';
        setTimeout(() => $('timeSyncMsg').textContent = '', 2000);
    }).catch(() => {
        $('timeSyncMsg').textContent = 'Error';
        setTimeout(() => $('timeSyncMsg').textContent = '', 2000);
    });
});

$('btnZeroRudder').addEventListener('click', () => {
    if (!confirm('Zero the rudder encoder at its current position?')) return;
    fetch(API + '/api/rudder/zero', {method:'POST'}).then(async r => {
        if (r.ok) {
            $('rudderZeroMsg').textContent = 'Zeroed!';
        } else {
            let msg = 'Error';
            try {
                const body = await r.json();
                if (body && body.error) msg = body.error;
            } catch (_) {}
            $('rudderZeroMsg').textContent = msg;
        }
        setTimeout(() => $('rudderZeroMsg').textContent = '', r.ok ? 2000 : 10000);
    }).catch(() => {
        $('rudderZeroMsg').textContent = 'Error';
        setTimeout(() => $('rudderZeroMsg').textContent = '', 2000);
    });
});

let canMonTimer = null;
function refreshCanStats() {
    fetch(API + '/api/can/stats').then(r => r.json()).then(d => {
        $('canTxStats').textContent =
            'TX ok: ' + d.tx_ok + '  TX failed (no ack on bus): ' + d.tx_fail;
        const rows = (d.rx || []).sort((a, b) => parseInt(a.id, 16) - parseInt(b.id, 16))
            .map(f => '<tr><td style="padding:2px 10px 2px 0">' + f.id +
                '</td><td style="text-align:right;padding:2px 10px 2px 0">' + f.count +
                '</td><td style="text-align:right;padding:2px 10px 2px 0">' + f.len +
                '</td><td style="padding:2px 10px 2px 0">' + f.data +
                '</td><td style="text-align:right">' + f.age_ms + '</td></tr>');
        $('canStatsTable').querySelector('tbody').innerHTML = rows.join('');
    }).catch(() => {});
}
$('canMonitor').addEventListener('toggle', () => {
    if ($('canMonitor').open) {
        refreshCanStats();
        canMonTimer = setInterval(refreshCanStats, 500);
    } else if (canMonTimer) {
        clearInterval(canMonTimer);
        canMonTimer = null;
    }
});

$('btnResetConfig').addEventListener('click', () => {
    if (!confirm('Reset all control config fields to defaults?\n(Form will be populated; click Save Config to persist.)')) return;
    fetch(API + '/api/config/defaults').then(r => r.json()).then(cfg => {
        populateConfigForm(cfg);
        $('saveMsg').textContent = 'Defaults loaded';
        setTimeout(() => $('saveMsg').textContent = '', 2000);
    }).catch(() => {
        $('saveMsg').textContent = 'Error';
        setTimeout(() => $('saveMsg').textContent = '', 2000);
    });
});

function loadServos() {
    fetch(API + '/api/servos').then(r => r.json()).then(servos => {
        const list = $('servoList');
        list.innerHTML = '';
        servos.forEach(s => {
            const card = document.createElement('div');
            card.className = 'servo-card' + (s.simulated ? ' sim' : '') + (s.cal_mode ? ' cal' : '');
            card.id = 'servo-card-' + s.handle;
            card.innerHTML = `
                <div class="servo-card-header">
                    <h4>Servo GPIO ${s.gpio} (handle ${s.handle})</h4>
                    <span class="servo-badge ${s.simulated ? 'sim' : (s.cal_mode ? 'cal' : 'hw')}">${s.simulated ? 'SIM' : (s.cal_mode ? 'CAL' : 'HW')}</span>
                </div>
                <div class="servo-live"><span id="servo-deg-${s.handle}">${s.cmd_deg !== undefined ? Number(s.cmd_deg).toFixed(2) : '--'}</span><span class="unit"> °</span></div>
                <div class="cal-grid">
                    <div class="cal-group"><label>Min PW µs</label><input type="number" id="cal-min-pw-${s.handle}" step="0.1" value="${s.cal.min_pw_us}"></div>
                    <div class="cal-group"><label>Zero PW µs</label><input type="number" id="cal-zero-pw-${s.handle}" step="0.1" value="${s.cal.zero_pw_us}"></div>
                    <div class="cal-group"><label>Max PW µs</label><input type="number" id="cal-max-pw-${s.handle}" step="0.1" value="${s.cal.max_pw_us}"></div>
                    <div class="cal-group"><label>Min °</label><input type="number" id="cal-min-ag-${s.handle}" step="0.1" value="${s.cal.min_angle_deg}"></div>
                    <div class="cal-group"><label>Max °</label><input type="number" id="cal-max-ag-${s.handle}" step="0.1" value="${s.cal.max_angle_deg}"></div>
                </div>
                ${s.cal_mode ? `<div class="raw-pw-row" id="raw-pw-row-${s.handle}">
                    <label>Raw PW µs</label>
                    <input type="range" id="raw-pw-slider-${s.handle}" min="500" max="2500" step="1" value="${s.cal.zero_pw_us}" oninput="$('raw-pw-val-${s.handle}').textContent=Math.round(this.value)+' µs'" onchange="sendRawPw(${s.handle}, this.value)">
                    <span class="raw-pw-val" id="raw-pw-val-${s.handle}">${Math.round(s.cal.zero_pw_us)} µs</span>
                    <div class="capture-btns">
                        <button class="btn-capture" onclick="captureRawPw(${s.handle},'min')">← Min</button>
                        <button class="btn-capture" onclick="captureRawPw(${s.handle},'zero')">← Zero</button>
                        <button class="btn-capture" onclick="captureRawPw(${s.handle},'max')">← Max</button>
                    </div>
                </div>` : ''}
                <div class="save-row">
                    <button class="btn btn-save" style="min-width:100px" onclick="saveServoCal(${s.handle})">Save Cal</button>
                    <button class="btn ${s.cal_mode ? 'btn-cal-on' : 'btn-cal-off'}" style="min-width:100px" id="cal-btn-${s.handle}" onclick="toggleServoCal(${s.handle})">${s.cal_mode ? 'Exit CAL' : 'Calibrate'}</button>
                </div>
            `;
            list.appendChild(card);
        });
        if (servos.length === 0) {
            list.innerHTML = '<div class="hint">No servo instances open.</div>';
        }
    }).catch(() => {
        $('servoList').innerHTML = '<div class="hint">Failed to load servos.</div>';
    });
}

function updateServoCard(handle, data) {
    const card = $('servo-card-' + handle);
    if (!card) return;

    if (data.cmd_deg !== undefined) {
        const degEl = $('servo-deg-' + handle);
        if (degEl) degEl.textContent = Number(data.cmd_deg).toFixed(2);
    }

    const badge = card.querySelector('.servo-badge');
    if (badge && data.cal_mode !== undefined) {
        badge.textContent = data.simulated ? 'SIM' : (data.cal_mode ? 'CAL' : 'HW');
        badge.className = 'servo-badge ' + (data.simulated ? 'sim' : (data.cal_mode ? 'cal' : 'hw'));
        card.className = 'servo-card' + (data.simulated ? ' sim' : '') + (data.cal_mode ? ' cal' : '');
    }

    const calBtn = $('cal-btn-' + handle);
    if (calBtn && data.cal_mode !== undefined) {
        calBtn.textContent = data.cal_mode ? 'Exit CAL' : 'Calibrate';
        calBtn.className = 'btn ' + (data.cal_mode ? 'btn-cal-on' : 'btn-cal-off');
    }
}

function saveServoCal(handle) {
    const min_pw = parseFloat($('cal-min-pw-' + handle).value) || 0;
    const zero_pw = parseFloat($('cal-zero-pw-' + handle).value) || 0;
    const max_pw = parseFloat($('cal-max-pw-' + handle).value) || 0;
    const min_ag = parseFloat($('cal-min-ag-' + handle).value) || 0;
    const max_ag = parseFloat($('cal-max-ag-' + handle).value) || 0;
    fetch(API + '/api/servos', {
        method: 'PUT',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({
            handle: handle,
            min_pw_us: min_pw,
            zero_pw_us: zero_pw,
            max_pw_us: max_pw,
            min_angle_deg: min_ag,
            max_angle_deg: max_ag
        })
    }).then(r => r.json()).then(d => {
        if (d.error) { alert('Save failed: ' + d.error); } else { loadServos(); }
    }).catch(() => { alert('Save failed: network error'); });
}

function toggleServoCal(handle) {
    const btn = $('cal-btn-' + handle);
    const enabled = btn && btn.textContent === 'Calibrate';
    fetch(API + '/api/servos', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({handle: handle, enabled: enabled})
    }).then(() => loadServos()).catch(() => {});
}

function captureRawPw(handle, field) {
    const slider = $('raw-pw-slider-' + handle);
    if (!slider) return;
    const val = Math.round(Number(slider.value));
    const map = {min: 'cal-min-pw-', zero: 'cal-zero-pw-', max: 'cal-max-pw-'};
    const input = $(map[field] + handle);
    if (input) input.value = val;
}

function sendRawPw(handle, pulse_us) {
    fetch(API + '/api/servos/raw_pw', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({handle: handle, pulse_us: Math.round(Number(pulse_us))})
    }).catch(() => {});
}

/* ── Data tab: sessions, binary parsing, graphs, track map ── */
const DATALOG_MAGIC = 0x444B534A; // "JSKD" LE
const DATALOG_VERSION = 4;
const REC_SIZE = 70;
const HEADER_SIZE = 28;
const CFG_SIZE = 91;
const CFG_EVENT_SIZE = 95;

// key, label, colour; either a fixed-point field (off+scale) or a derived fn
const CH = [
  {k:'pitch',    l:'Pitch °',      c:'#5390d9', off:4,  sc:0.1},
  {k:'roll',     l:'Roll °',       c:'#4ecca3', off:6,  sc:0.1},
  {k:'yaw',      l:'Yaw °',        c:'#9b5de5', off:8,  sc:0.1},
  {k:'height',   l:'Height cm',    c:'#ffd93d', off:10, sc:1},
  {k:'rudder',   l:'Rudder °',     c:'#f15bb5', off:12, sc:0.1},
  {k:'pitch_t',  l:'Pitch tgt °',  c:'#3a6ea5', off:14, sc:0.1, dash:true},
  {k:'roll_t',   l:'Roll tgt °',   c:'#2a9d8f', off:16, sc:0.1, dash:true},
  {k:'height_t', l:'Height tgt cm',c:'#bfa00f', off:18, sc:1,   dash:true},
  {k:'joy',      l:'Joy trim °',   c:'#e94560', off:20, sc:0.1},
  {k:'elL',      l:'Elevon L °',   c:'#00bbf9', off:22, sc:0.1},
  {k:'elR',      l:'Elevon R °',   c:'#80ed99', off:24, sc:0.1},
  {k:'center',   l:'Center °',     c:'#fee440', der:(d,b)=>(d.getInt16(b+22,true)+d.getInt16(b+24,true))/2*0.1},
  {k:'diff',     l:'Diff °',       c:'#ff9e00', der:(d,b)=>(d.getInt16(b+22,true)-d.getInt16(b+24,true))/2*0.1},
  {k:'speed',    l:'Speed kn',     c:'#06d6a0', off:26, sc:0.01},
  {k:'course',   l:'Course °',     c:'#8338ec', off:28, sc:0.1},
  {k:'height_err', l:'Height err cm', c:'#f4a261', off:40, sc:0.1},
  {k:'height_p',   l:'Height P °',    c:'#e76f51', off:42, sc:0.1},
  {k:'height_i',   l:'Height I °',    c:'#2a9d8f', off:44, sc:0.1},
  {k:'height_d',   l:'Height D °',    c:'#264653', off:46, sc:0.1},
  {k:'height_out', l:'Height out °',  c:'#b56576', off:48, sc:0.1},
  {k:'pitch_err',  l:'Pitch err °',   c:'#ffb703', off:50, sc:0.1},
  {k:'pitch_p',    l:'Pitch P °',     c:'#fb8500', off:52, sc:0.1},
  {k:'pitch_i',    l:'Pitch I °',     c:'#219ebc', off:54, sc:0.1},
  {k:'pitch_d',    l:'Pitch D °',     c:'#023047', off:56, sc:0.1},
  {k:'pitch_out',  l:'Pitch out °',   c:'#8ecae6', off:58, sc:0.1},
  {k:'roll_err',   l:'Roll err °',    c:'#c77dff', off:60, sc:0.1},
  {k:'roll_p',     l:'Roll P °',      c:'#9d4edd', off:62, sc:0.1},
  {k:'roll_i',     l:'Roll I °',      c:'#7b2cbf', off:64, sc:0.1},
  {k:'roll_d',     l:'Roll D °',      c:'#5a189a', off:66, sc:0.1},
  {k:'roll_out',   l:'Roll out °',    c:'#e0aaff', off:68, sc:0.1},
];
const DEFAULT_ON = ['pitch','roll','pitch_t'];
const BASE_KEYS = ['pitch','roll','yaw','height','rudder','pitch_t','roll_t','height_t','joy','elL','elR','center','diff','speed','course'];
const GRAPH_MODES = {
  all: {
    label: 'All',
    keys: BASE_KEYS,
    defaults: DEFAULT_ON,
  },
  pitch: {
    label: 'Pitch PID',
    keys: ['pitch','pitch_t','pitch_err','pitch_out','pitch_p','pitch_i','pitch_d'],
    defaults: ['pitch','pitch_t','pitch_out'],
    panels: [
      {title:'Pitch response', keys:['pitch','pitch_t','pitch_err']},
      {title:'Pitch controller', keys:['pitch_out','pitch_p','pitch_i','pitch_d']},
    ],
  },
  roll: {
    label: 'Roll PID',
    keys: ['roll','roll_t','roll_err','roll_out','roll_p','roll_i','roll_d'],
    defaults: ['roll','roll_t','roll_out'],
    panels: [
      {title:'Roll response', keys:['roll','roll_t','roll_err']},
      {title:'Roll controller', keys:['roll_out','roll_p','roll_i','roll_d']},
    ],
  },
  height: {
    label: 'Height PID',
    keys: ['height','height_t','height_err','height_out','height_p','height_i','height_d'],
    defaults: ['height','height_t','height_out'],
    panels: [
      {title:'Height response', keys:['height','height_t','height_err']},
      {title:'Height controller', keys:['height_out','height_p','height_i','height_d']},
    ],
  },
};

let dataSession = null;          // {id, n, t, cols{}, lat, lon, gps}
let visible = new Set(DEFAULT_ON);
let graphMode = 'all';
let selIndex = -1;
let viewT0 = 0, viewT1 = 0;      // zoomed x-range in seconds; T1<=T0 = full session
let sessionsMeta = [], currentSessionId = 0, localFile = null;
let map = null, trackLine = null, selMarker = null;
let selectedConfigEvent = null;
const CH_BY_KEY = {};
CH.forEach(ch => { CH_BY_KEY[ch.k] = ch; });

const CFG_FIELDS = [
  ['height_kp','Height P','i32',0], ['height_ki','Height I','i32',4], ['height_kd','Height D','i32',8],
  ['pitch_kp','Pitch P','i32',12], ['pitch_ki','Pitch I','i32',16], ['pitch_kd','Pitch D','i32',20],
  ['roll_kp','Roll P','i32',24], ['roll_ki','Roll I','i32',28], ['roll_kd','Roll D','i32',32],
  ['rudder_exponent_x100','Rudder exponent x100','i16',36], ['rudder_max_roll_deg','Rudder max roll','i16',38],
  ['height_enabled','Height loop','bool',40],
  ['elevon_max_diff_deg','Elevon max diff','i16',41], ['pitch_target_max_deg','Pitch target max','i16',43],
  ['height_target_cm','Height target','i16',45],
  ['imu_pitch_offset_x10','IMU pitch offset x10','i16',47], ['imu_roll_offset_x10','IMU roll offset x10','i16',49],
];

function decodeConfig(raw){
  const dv = raw instanceof DataView ? raw : new DataView(raw.buffer || raw, raw.byteOffset || 0, raw.byteLength || CFG_SIZE);
  const cfg = {raw: raw instanceof Uint8Array ? raw : new Uint8Array(dv.buffer, dv.byteOffset, dv.byteLength)};
  CFG_FIELDS.forEach(f => {
    if (f[2] === 'i32') cfg[f[0]] = dv.getInt32(f[3], true);
    else if (f[2] === 'i16') cfg[f[0]] = dv.getInt16(f[3], true);
    else cfg[f[0]] = dv.getUint8(f[3]) !== 0;
  });
  cfg.servo = [];
  let off = 51;
  for (let s=0; s<2; s++) {
    const vals = [];
    for (let i=0; i<5; i++, off+=4) vals.push(dv.getFloat32(off, true));
    cfg.servo.push(vals);
  }
  return cfg;
}

function parseSessions(buf){
  const dv = new DataView(buf), out = [];
  let o = 0;
  while (o + HEADER_SIZE <= buf.byteLength){
    if (dv.getUint32(o, true) !== DATALOG_MAGIC) break;
    const version = dv.getUint16(o+4, true);
    const recSize = dv.getUint16(o+6, true) || REC_SIZE;
    const id = dv.getUint32(o+8, true);
    const n  = dv.getUint32(o+12, true);
    const cfgSize = dv.getUint16(o+16, true) || CFG_EVENT_SIZE;
    const cfgN = dv.getUint32(o+18, true);
    const hz = dv.getUint16(o+22, true) || 10;
    const startEpoch = dv.getUint32(o+24, true);
    o += HEADER_SIZE;
    if (version !== DATALOG_VERSION || recSize !== REC_SIZE || cfgSize !== CFG_EVENT_SIZE) break;
    if (o + n*recSize > buf.byteLength) break;
    const s = {id, n, hz, startEpoch, t:new Float64Array(n), cols:{},
               lat:new Float32Array(n), lon:new Float32Array(n), gps:new Uint8Array(n), cfgEvents:[]};
    CH.forEach(ch => s.cols[ch.k] = new Float32Array(n));
    for (let i=0;i<n;i++){
      const b = o + i*recSize;
      s.t[i] = dv.getUint32(b, true)/1000;
      CH.forEach(ch => { s.cols[ch.k][i] = ch.der ? ch.der(dv,b) : dv.getInt16(b+ch.off,true)*ch.sc; });
      s.lat[i] = dv.getFloat32(b+30, true);
      s.lon[i] = dv.getFloat32(b+34, true);
      s.gps[i] = dv.getUint8(b+38) & 0x02;
    }
    o += n*recSize;
    if (o + cfgN*cfgSize > buf.byteLength) break;
    for (let i=0;i<cfgN;i++){
      const b = o + i*cfgSize;
      const raw = new Uint8Array(buf.slice(b+4, b+4+CFG_SIZE));
      s.cfgEvents.push({t: dv.getUint32(b, true)/1000, cfg: decodeConfig(raw), raw});
    }
    out.push(s);
    o += cfgN*cfgSize;
  }
  return out;
}

function fmtDur(ms){ const s=Math.round(ms/1000); return s<60 ? s+'s' : Math.floor(s/60)+'m'+(s%60)+'s'; }
function fmtStart(epoch_s){
  return epoch_s ? new Date(epoch_s*1000).toLocaleString([], {day:'numeric', month:'short', hour:'2-digit', minute:'2-digit'}) : '';
}

function openDataTab(){
  loadSessions();
  buildModeControls();
  buildLegend();
  ensureMap();
  setTimeout(()=>{ if(map) map.invalidateSize(); }, 60);
  drawChart();
}

function loadSessions(){
  if (!ONDEVICE) {
    localFile = localFile || null;
    sessionsMeta = [];
    currentSessionId = 0;
    $('storageFill').style.width = '0%';
    $('storageText').textContent = 'Open a downloaded .bin session file.';
    renderSessionList();
    return;
  }
  localFile = null;
  fetch(API+'/api/sessions').then(r=>r.json()).then(d=>{
    sessionsMeta = d.sessions || [];
    currentSessionId = d.current;
    if (d.hz) $('selHz').value = String(d.hz);
    const pct = d.total ? Math.round(d.used/d.total*100) : 0;
    $('storageFill').style.width = pct+'%';
    $('storageText').textContent = (d.used/1048576).toFixed(1)+' / '+(d.total/1048576).toFixed(1)+' MB used ('+pct+'%)';
    renderSessionList();
  }).catch(()=>{ $('storageText').textContent='Failed to load sessions'; });
}

function renderSessionList(){
  const list = $('sessList'); list.innerHTML='';
  if (localFile){
    const hdr = document.createElement('div'); hdr.className='data-hint';
    hdr.innerHTML = ONDEVICE
      ? `Viewing file: <b>${localFile.name}</b> · <a href="#" id="closeFile" style="color:#5390d9">use device sessions</a>`
      : `Viewing file: <b>${localFile.name}</b>`;
    list.appendChild(hdr);
    localFile.sessions.forEach(s=>{
      const row=document.createElement('div');
      row.className='sess-row'+(dataSession===s?' active':'');
      const start=fmtStart(s.startEpoch);
      row.innerHTML=`<span class="sess-name">Session ${s.id}</span><span class="sess-meta">${start?start+' · ':''}${s.n} pts</span>`;
      row.addEventListener('click',()=>selectSession(s));
      list.appendChild(row);
    });
    const close = $('closeFile');
    if (close) close.addEventListener('click',e=>{ e.preventDefault(); loadSessions(); });
    return;
  }
  if (!sessionsMeta.length){ list.innerHTML='<div class="data-hint">No sessions yet.</div>'; return; }
  sessionsMeta.forEach(s=>{
    const row=document.createElement('div');
    row.className='sess-row'+(dataSession && !dataSession.local && dataSession.id===s.id ? ' active':'');
    const live=s.id===currentSessionId ? ' · REC':'';
    const start=fmtStart(s.start_epoch_s);
    row.innerHTML=`<span class="sess-name">Session ${s.id}</span>`+
      (s.at_risk?'<span class="warn" title="auto-deleted next when space runs low">⚠</span>':'')+
      `<span class="sess-meta">${start?start+' · ':''}${s.records} pts · ${fmtDur(s.duration_ms)}${live}</span>`;
    row.addEventListener('click',()=>loadSessionData(s.id));
    const del=document.createElement('button');
    del.className='del'; del.textContent='✕'; del.title='Delete';
    del.addEventListener('click',ev=>{ ev.stopPropagation(); deleteSession(s.id, s.id===currentSessionId); });
    row.appendChild(del);
    list.appendChild(row);
  });
}

function deleteSession(id, isCurrent){
  if (isCurrent){ alert('Cannot delete the active session — start a new one first.'); return; }
  if (!confirm('Delete session '+id+'?')) return;
  fetch(API+'/api/session/delete?id='+id, {method:'POST'}).then(()=>loadSessions()).catch(()=>{});
}

function loadSessionData(id){
  fetch(API+'/api/session?id='+id).then(r=>r.arrayBuffer()).then(buf=>{
    const ss = parseSessions(buf);
    if (!ss.length){ $('chartTitle').textContent='Graphs — (empty)'; return; }
    selectSession(ss[0]);
  }).catch(()=>{ $('chartTitle').textContent='Graphs — load failed'; });
}

function selectSession(s){
  dataSession = s;
  selIndex = -1;
  viewT0 = viewT1 = 0;
  const start=fmtStart(s.startEpoch);
  $('chartTitle').textContent = 'Graphs — Session '+s.id+' ('+s.n+' pts · '+s.hz+' Hz'+(start?' · started '+start:'')+')';
  renderSessionList();
  buildModeControls();
  buildLegend();
  drawChart();
  updateTrack();
  updateConfigPanel();
  $('readout').textContent = s.n ? '' : 'No samples in this session.';
}

function buildLegend(){
  const el=$('legend'); el.innerHTML='';
  const mode = GRAPH_MODES[graphMode] || GRAPH_MODES.all;
  mode.keys.map(k => CH_BY_KEY[k]).filter(Boolean).forEach(ch=>{
    const chip=document.createElement('span');
    chip.className='chip'+(visible.has(ch.k)?' on':'');
    chip.innerHTML=`<span class="sw line${ch.dash?' dash':''}" style="border-color:${ch.c}"></span>${ch.l}${ch.dash?' <span class="dash-label">dashed</span>':''}`;
    chip.addEventListener('click',()=>{
      if (visible.has(ch.k)) visible.delete(ch.k); else visible.add(ch.k);
      chip.classList.toggle('on'); drawChart(); updateReadout();
    });
    el.appendChild(chip);
  });
  const cfgChip=document.createElement('span');
  cfgChip.className='chip on static';
  cfgChip.innerHTML='<span class="sw cfg-change"></span>Config change';
  el.appendChild(cfgChip);
}

function buildModeControls(){
  const legend = $('legend');
  if (!legend) return;
  let el = $('chartModes');
  if (!el) {
    el = document.createElement('div');
    el.id = 'chartModes';
    el.className = 'mode-bar';
    legend.parentNode.insertBefore(el, legend);
  }
  el.innerHTML = '';
  Object.keys(GRAPH_MODES).forEach(k => {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'mode-btn' + (graphMode === k ? ' active' : '');
    btn.textContent = GRAPH_MODES[k].label;
    btn.addEventListener('click', () => {
      graphMode = k;
      visible = new Set(GRAPH_MODES[k].defaults);
      buildModeControls();
      buildLegend();
      drawChart();
      updateReadout();
    });
    el.appendChild(btn);
  });
}

function nearestByTime(tt){
  const t=dataSession.t, n=dataSession.n;
  let lo=0, hi=n-1;
  while(lo<hi){ const m=(lo+hi)>>1; if(t[m]<tt) lo=m+1; else hi=m; }
  if(lo>0 && Math.abs(t[lo-1]-tt)<Math.abs(t[lo]-tt)) lo--;
  return lo;
}

function nearestByLatLng(la,lo){
  let best=-1, bd=Infinity;
  for(let i=0;i<dataSession.n;i++){
    if(!dataSession.gps[i]) continue;
    const dx=dataSession.lat[i]-la, dy=dataSession.lon[i]-lo, d=dx*dx+dy*dy;
    if(d<bd){ bd=d; best=i; }
  }
  return best;
}

function gpsDistanceM(aLat, aLon, bLat, bLon){
  const latM=(bLat-aLat)*111320;
  const lonM=(bLon-aLon)*111320*Math.cos(((aLat+bLat)*0.5)*Math.PI/180);
  return Math.sqrt(latM*latM + lonM*lonM);
}

function displayGpsIndexes(){
  if(!dataSession || !dataSession.n) return [];
  const raw=[];
  for(let i=0;i<dataSession.n;i++) if(dataSession.gps[i]) raw.push(i);
  if(raw.length <= 2) return raw;
  const out=[raw[0]];
  let last=raw[0];
  for(let j=1;j<raw.length-1;j++){
    const i=raw[j];
    if(gpsDistanceM(dataSession.lat[last], dataSession.lon[last], dataSession.lat[i], dataSession.lon[i]) >= 1.5){
      out.push(i);
      last=i;
    }
  }
  const final=raw[raw.length-1];
  if(out[out.length-1] !== final) out.push(final);
  return out;
}

function drawChart(){
  const cv=$('chart'), wrap=$('chartWrap');
  const w=wrap.clientWidth||600, h=cv.clientHeight||320, dpr=window.devicePixelRatio||1;
  cv.width=w*dpr; cv.height=h*dpr; cv.style.height=h+'px';
  const ctx=cv.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
  if(!dataSession||!dataSession.n) return;
  const mode = GRAPH_MODES[graphMode] || GRAPH_MODES.all;
  if (mode.panels) {
    const gap = 24;
    const panelH = (h - gap) / 2;
    drawChartPanel(ctx, w, 0, panelH, mode.panels[0].keys, mode.panels[0].title);
    drawChartPanel(ctx, w, panelH + gap, panelH, mode.panels[1].keys, mode.panels[1].title);
    return;
  }
  drawChartPanel(ctx, w, 0, h, mode.keys, null);
}

function viewRange(){
  const full=dataSession.t[dataSession.n-1]||1;
  return viewT1>viewT0 ? [viewT0,viewT1] : [0,full];
}

function fmtT(ts){
  return 'T+'+(ts>=60 ? Math.floor(ts/60)+':'+(ts%60).toFixed(2).padStart(5,'0') : ts.toFixed(2)+'s');
}

function drawConfigLines(ctx, padL, padT, plotW, plotH, tA, tB){
  if(dataSession.cfgEvents && dataSession.cfgEvents.length > 1){
    ctx.save();
    ctx.strokeStyle='#ffd93d';
    ctx.globalAlpha=.7;
    ctx.setLineDash([3,5]);
    dataSession.cfgEvents.slice(1).forEach(ev=>{
      if(ev.t < tA || ev.t > tB) return;
      const x=padL+((ev.t-tA)/(tB-tA))*plotW;
      ctx.beginPath();
      ctx.moveTo(x,padT);
      ctx.lineTo(x,padT+plotH);
      ctx.stroke();
    });
    ctx.restore();
  }
}

function drawChartPanel(ctx, w, y0, h, keys, title){
  const padL=46,padR=8,padT=y0+18,padB=18, plotW=w-padL-padR, plotH=h-28-padB;
  const t=dataSession.t, n=dataSession.n;
  const [tA,tB]=viewRange(), tSpan=(tB-tA)||1;
  /* sample range covering the view, extended one sample past each edge so
   * lines run to the plot border when zoomed */
  let i0=nearestByTime(tA), i1=nearestByTime(tB);
  if(i0>0) i0--;
  if(i1<n-1) i1++;
  const vis=keys.map(k => CH_BY_KEY[k]).filter(ch=>ch && visible.has(ch.k));
  let mn=Infinity,mx=-Infinity;
  vis.forEach(ch=>{ const a=dataSession.cols[ch.k]; for(let i=i0;i<=i1;i++){ const v=a[i]; if(v<mn)mn=v; if(v>mx)mx=v; } });
  if(mn===Infinity){ mn=-1; mx=1; }
  if(mn===mx){ mn-=1; mx+=1; }
  const p=(mx-mn)*0.08; mn-=p; mx+=p;
  const X=i=>padL+((t[i]-tA)/tSpan)*plotW, Y=v=>padT+(1-(v-mn)/(mx-mn))*plotH;
  ctx.font='10px monospace'; ctx.lineWidth=1;
  if (title) {
    ctx.fillStyle='#9aa6c2';
    ctx.fillText(title, padL, y0 + 10);
  }
  for(let g=0;g<=4;g++){
    const yy=padT+plotH*g/4, val=mx-(mx-mn)*g/4;
    ctx.strokeStyle='#23314f'; ctx.beginPath(); ctx.moveTo(padL,yy); ctx.lineTo(w-padR,yy); ctx.stroke();
    ctx.fillStyle='#7a89a8'; ctx.fillText(val.toFixed(1).padStart(6), 2, yy+3);
  }
  if(mn<0&&mx>0){ ctx.strokeStyle='#44557a'; ctx.beginPath(); ctx.moveTo(padL,Y(0)); ctx.lineTo(w-padR,Y(0)); ctx.stroke(); }
  ctx.fillStyle='#7a89a8';
  for(let g=0;g<=4;g++){
    const lbl=fmtT(tA+tSpan*g/4);
    ctx.textAlign = g===0 ? 'left' : (g===4 ? 'right' : 'center');
    ctx.fillText(lbl, padL+plotW*g/4, padT+plotH+12);
  }
  ctx.textAlign='left';
  drawConfigLines(ctx, padL, padT, plotW, plotH, tA, tB);
  ctx.save();
  ctx.beginPath(); ctx.rect(padL,padT,plotW,plotH); ctx.clip();
  ctx.lineWidth=1.5;
  vis.forEach(ch=>{
    const a=dataSession.cols[ch.k];
    ctx.strokeStyle=ch.c; ctx.setLineDash(ch.dash?[4,3]:[]); ctx.beginPath();
    for(let i=i0;i<=i1;i++){ const x=X(i),y=Y(a[i]); i>i0?ctx.lineTo(x,y):ctx.moveTo(x,y); }
    ctx.stroke();
  });
  ctx.setLineDash([]);
  if(selIndex>=0&&selIndex<n&&t[selIndex]>=tA&&t[selIndex]<=tB){
    const x=X(selIndex);
    ctx.strokeStyle='#fff'; ctx.globalAlpha=.5; ctx.beginPath(); ctx.moveTo(x,padT); ctx.lineTo(x,padT+plotH); ctx.stroke(); ctx.globalAlpha=1;
    vis.forEach(ch=>{ ctx.fillStyle=ch.c; ctx.beginPath(); ctx.arc(x,Y(dataSession.cols[ch.k][selIndex]),2.5,0,7); ctx.fill(); });
  }
  ctx.restore();
}

function findConfigEvent(t){
  if(!dataSession || !dataSession.cfgEvents || !dataSession.cfgEvents.length) return null;
  let best = dataSession.cfgEvents[0];
  for (let i=1;i<dataSession.cfgEvents.length;i++) {
    if (dataSession.cfgEvents[i].t <= t) best = dataSession.cfgEvents[i];
    else break;
  }
  return best;
}

function updateReadout(){
  if(selIndex<0||!dataSession){ $('readout').textContent=''; return; }
  const parts=['t='+dataSession.t[selIndex].toFixed(2)+'s'];
  CH.filter(ch=>visible.has(ch.k)).forEach(ch=>parts.push(ch.l+'='+dataSession.cols[ch.k][selIndex].toFixed(1)));
  $('readout').textContent=parts.join('   ');
}

function updateConfigPanel(){
  const el = $('configAt');
  const btn = $('btnRestoreConfig');
  if (!el) return;
  selectedConfigEvent = null;
  if (!dataSession || selIndex < 0) {
    el.textContent = dataSession && dataSession.cfgEvents && dataSession.cfgEvents.length ? 'Select a graph or track point.' : 'No config events in this session.';
    if (btn) btn.style.display = 'none';
    return;
  }
  selectedConfigEvent = findConfigEvent(dataSession.t[selIndex]);
  if (!selectedConfigEvent) {
    el.textContent = 'No config event at this time.';
    if (btn) btn.style.display = 'none';
    return;
  }
  const c = selectedConfigEvent.cfg;
  $('configAtTitle').textContent = 'Config at T+' + dataSession.t[selIndex].toFixed(2) + 's';
  el.innerHTML = [
    ['Height', c.height_enabled ? 'on' : 'off', 'Target ' + c.height_target_cm + ' cm', 'PID ' + c.height_kp + '/' + c.height_ki + '/' + c.height_kd],
    ['Pitch', 'Max target ' + c.pitch_target_max_deg + '°', 'PID ' + c.pitch_kp + '/' + c.pitch_ki + '/' + c.pitch_kd],
    ['Roll', 'Max diff ' + c.elevon_max_diff_deg + '°', 'PID ' + c.roll_kp + '/' + c.roll_ki + '/' + c.roll_kd],
    ['Rudder', 'Exp ' + c.rudder_exponent_x100, 'Max roll ' + c.rudder_max_roll_deg + '°'],
    ['IMU', 'Pitch ' + (c.imu_pitch_offset_x10/10).toFixed(1) + '°', 'Roll ' + (c.imu_roll_offset_x10/10).toFixed(1) + '°'],
    ['Servo 0', c.servo[0].map(v=>v.toFixed(1)).join(' / ')],
    ['Servo 1', c.servo[1].map(v=>v.toFixed(1)).join(' / ')],
  ].map(row => '<div class="config-row"><b>'+row[0]+'</b><span>'+row.slice(1).join(' · ')+'</span></div>').join('');
  if (btn) btn.style.display = (ONDEVICE && !localFile && dataSession.id === currentSessionId) ? 'block' : 'none';
}

function pickIndex(i){ selIndex=i; drawChart(); updateReadout(); updateSelMarker(); updateConfigPanel(); }

function chartFrac(x){
  const r=$('chart').getBoundingClientRect(), padL=46,padR=8, plotW=r.width-padL-padR;
  return Math.max(0,Math.min(1,(x-r.left-padL)/plotW));
}

function chartPick(e){
  if(!dataSession||!dataSession.n) return;
  const [tA,tB]=viewRange();
  pickIndex(nearestByTime(tA+chartFrac(e.clientX)*(tB-tA)));
}

/* Horizontal drag selects a time range to zoom into; a plain click/tap picks a
 * point; double-click resets to the full session. */
let dragX0=-1, dragX1=-1, dragging=false;

function drawDragOverlay(){
  const cv=$('chart'), r=cv.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  const ctx=cv.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0);
  const x0=Math.min(dragX0,dragX1)-r.left, x1=Math.max(dragX0,dragX1)-r.left;
  ctx.fillStyle='rgba(83,144,217,0.22)';
  ctx.fillRect(x0,0,x1-x0,cv.clientHeight);
}

$('chart').addEventListener('pointerdown',e=>{
  if(!dataSession||!dataSession.n) return;
  dragX0=dragX1=e.clientX; dragging=false;
  e.currentTarget.setPointerCapture(e.pointerId);
});
$('chart').addEventListener('pointermove',e=>{
  if(!dataSession||!dataSession.n) return;
  if(dragX0>=0){
    dragX1=e.clientX;
    if(Math.abs(dragX1-dragX0)>6) dragging=true;
    if(dragging){ drawChart(); drawDragOverlay(); }
    return;
  }
  if(e.pointerType==='mouse' && e.buttons===0) chartPick(e);  /* hover scrub */
});
$('chart').addEventListener('pointerup',e=>{
  if(dragX0<0) return;
  if(dragging){
    const [tA,tB]=viewRange();
    const ta=tA+chartFrac(Math.min(dragX0,dragX1))*(tB-tA);
    const tb=tA+chartFrac(Math.max(dragX0,dragX1))*(tB-tA);
    if(tb-ta>0.01){ viewT0=ta; viewT1=tb; }
    drawChart();
  } else {
    chartPick(e);
  }
  dragX0=dragX1=-1; dragging=false;
});
$('chart').addEventListener('pointercancel',()=>{ dragX0=dragX1=-1; dragging=false; drawChart(); });
$('chart').addEventListener('dblclick',()=>{ viewT0=viewT1=0; drawChart(); });

function ensureMap(){
  if(map || typeof L==='undefined') return;
  map=L.map('map',{attributionControl:false}).setView([0,0],2);
  cachedTileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}').addTo(map);
  map.on('click', e=>{ if(!dataSession) return; const i=nearestByLatLng(e.latlng.lat,e.latlng.lng); if(i>=0) pickIndex(i); });
}

function updateTrack(){
  ensureMap(); if(!map) return;
  if(trackLine){ trackLine.remove(); trackLine=null; }
  if(selMarker){ selMarker.remove(); selMarker=null; }
  const gpsIdx=displayGpsIndexes();
  const pts=gpsIdx.map(i=>[dataSession.lat[i],dataSession.lon[i]]);
  if(pts.length){
    trackLine=L.polyline(pts,{color:'#4ecca3',weight:3}).addTo(map);
    map.fitBounds(trackLine.getBounds(),{padding:[20,20]});
  }
  setTimeout(()=>map.invalidateSize(),50);
}

function updateSelMarker(){
  if(!map) return;
  if(selIndex<0||!dataSession||!dataSession.gps[selIndex]){ if(selMarker){selMarker.remove();selMarker=null;} return; }
  const ll=[dataSession.lat[selIndex],dataSession.lon[selIndex]];
  if(selMarker) selMarker.setLatLng(ll);
  else selMarker=L.circleMarker(ll,{radius:6,color:'#fff',weight:2,fillColor:'#e94560',fillOpacity:1}).addTo(map);
}

/* Leaflet tile layer that caches tiles in IndexedDB, so areas loaded once
 * (online / cellular) still render when offline on the AP. Uncached + offline
 * tiles stay blank → the track draws on a plain background. */
let tileDb=null;
function openTileDb(){
  return new Promise(res=>{
    if(tileDb) return res(tileDb);
    const rq=indexedDB.open('tilecache',1);
    rq.onupgradeneeded=()=>rq.result.createObjectStore('t');
    rq.onsuccess=()=>{ tileDb=rq.result; res(tileDb); };
    rq.onerror=()=>res(null);
  });
}
function tileGet(k){ return new Promise(res=>openTileDb().then(db=>{ if(!db)return res(null); const r=db.transaction('t').objectStore('t').get(k); r.onsuccess=()=>res(r.result||null); r.onerror=()=>res(null); })); }
function tilePut(k,b){ openTileDb().then(db=>{ if(db) db.transaction('t','readwrite').objectStore('t').put(b,k); }); }

function cachedTileLayer(url){
  const Cached=L.TileLayer.extend({
    createTile(coords, done){
      const img=document.createElement('img'); img.alt='';
      const key=this.getTileUrl(coords);
      tileGet(key).then(blob=>{
        if(blob){ img.src=URL.createObjectURL(blob); done(null,img); return; }
        fetch(key).then(r=>r.ok?r.blob():Promise.reject())
          .then(b=>{ tilePut(key,b); img.src=URL.createObjectURL(b); done(null,img); })
          .catch(()=>done(null,img));   // offline & uncached → blank
      });
      return img;
    }
  });
  return new Cached(url,{maxZoom:19});
}

$('btnNewSession').addEventListener('click',()=>{ fetch(API+'/api/session/new?hz='+$('selHz').value,{method:'POST'}).then(()=>loadSessions()).catch(()=>{}); });
$('btnClearSessions').addEventListener('click',()=>{ if(!confirm('Delete ALL saved sessions except the active one?'))return; fetch(API+'/api/session/clear',{method:'POST'}).then(()=>loadSessions()).catch(()=>{}); });
$('btnDownload').addEventListener('click',()=>{ window.location.href=API+'/api/download'; });
$('btnOpenFile').addEventListener('click',()=>$('fileInput').click());
$('fileInput').addEventListener('change',e=>{
  const f=e.target.files[0]; if(!f) return;
  f.arrayBuffer().then(buf=>{
    const ss=parseSessions(buf);
    if(!ss.length){ alert('No valid session data in this file.'); return; }
    ss.forEach(s=>s.local=true);
    localFile={name:f.name, sessions:ss};
    renderSessionList();
    selectSession(ss[0]);
  });
  e.target.value='';
});
window.addEventListener('resize',()=>{ if($('panel-data').classList.contains('active')) drawChart(); });

$('btnRestoreConfig').addEventListener('click',()=>{
  if(!selectedConfigEvent) return;
  fetch(API+'/api/config/restore', {method:'POST', body:selectedConfigEvent.raw})
    .then(r=>r.json()).then(d=>{
      $('restoreMsg').textContent = d.error ? 'Error' : 'Applied';
      setTimeout(()=>$('restoreMsg').textContent='', 2000);
    }).catch(()=>{
      $('restoreMsg').textContent = 'Error';
      setTimeout(()=>$('restoreMsg').textContent='', 2000);
    });
});

if (!ONDEVICE) {
  document.querySelectorAll('.tabs button').forEach(b => b.classList.toggle('active', b.dataset.tab === 'data'));
  document.querySelectorAll('.panel').forEach(p => p.classList.toggle('active', p.id === 'panel-data'));
  openDataTab();
}
