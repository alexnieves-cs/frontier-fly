/** Markup for the indoor sim page and small DOM helpers. */

import { siteNavLinks } from './siteNav';

export function shell(): string {
  return `
  <div class="shell" id="shell">
    <header class="topbar">
      <div class="brand"><div class="mark" aria-hidden="true"></div><div><h1>DRONE SIM</h1><p>FRONTIER-FLY</p></div></div>
      <nav class="page-links" aria-label="Pages">${siteNavLinks('sim')}</nav>
      <div class="tabs" role="toolbar" aria-label="View shortcuts">
        <span class="tabs-label">VIEW</span>
        <button type="button" data-act="tab" data-cam="chase" class="active">SIMULATION</button>
        <button type="button" data-act="tab" data-cam="overview">ENVIRONMENT</button>
        <button type="button" data-act="tab" data-cam="fpv">DRONE</button>
        <button type="button" data-act="sensor" data-sensor="ir">SENSORS</button>
        <button type="button" data-act="train" id="train-tab">TRAINING</button>
      </div>
      <div class="status-pills">
        <span class="pill"><i class="dot"></i><span id="link" title="Simulation, training, and inference all run in this browser tab">IN-BROWSER</span></span>
        <span class="pill" id="bat-num">100%</span>
        <span class="pill" id="clock">12:00</span>
        <span class="pill" id="fps">-- fps</span>
        <span class="pill" id="backend">WEBGL2</span>
      </div>
    </header>
    <main class="workspace">
      <aside class="rail">
        <div class="group">
          <p class="kicker">ENVIRONMENT</p>
          <div class="row"><span>Day / Night</span><span id="day-label">Day</span></div>
          <div class="seg">
            <button type="button" data-act="day">Toggle</button>
            <button type="button" data-act="lights">Lights <span id="light-label">On</span></button>
            <button type="button" data-act="flash">Flash</button>
          </div>
          <div class="row"><span>Time</span></div>
          <input id="tod" aria-label="Time of day" class="slider" type="range" min="0" max="24" step="0.1" value="13" />
        </div>
        <div class="group">
          <p class="kicker">CAMERA</p>
          <div class="stack">
            <button type="button" class="text-btn" data-act="cam" data-cam="fpv">FPV</button>
            <button type="button" class="text-btn active" data-act="cam" data-cam="chase">Chase</button>
            <button type="button" class="text-btn" data-act="cam" data-cam="overview">Overview</button>
          </div>
        </div>
        <div class="group">
          <p class="kicker">PILOT</p>
          <div class="seg">
            <button type="button" data-act="pilot" data-pilot="keys">Keys</button>
            <button type="button" data-act="pilot" data-pilot="heuristic">Heuristic</button>
            <button type="button" data-act="pilot" data-pilot="policy">Policy</button>
          </div>
        </div>
        <div class="group">
          <p class="kicker">SENSORS</p>
          <div class="sensor"><span>LiDAR</span><span class="live">ACTIVE</span></div>
          <div class="sensor"><span>IMU</span><span class="live">ACTIVE</span></div>
          <div class="seg">
            <button type="button" data-act="sensor" data-sensor="rgb">RGB</button>
            <button type="button" data-act="sensor" data-sensor="ir">IR</button>
            <button type="button" data-act="sensor" data-sensor="depth">Depth</button>
          </div>
        </div>
        <div class="group">
          <p class="kicker">RUN</p>
          <div class="row"><span>Seed</span><input id="seed" aria-label="Building seed" inputmode="numeric" class="field" style="width:88px" value="42" /></div>
          <div class="row"><span>Stage</span>
            <select id="stage" aria-label="Curriculum stage" class="field" style="width:88px">
              <option value="auto">auto</option>
              <option>0</option><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option>
            </select>
          </div>
          <div class="stack">
            <button type="button" class="text-btn primary" data-act="seed">New seed</button>
            <button type="button" class="text-btn" id="pause-btn" data-act="pause">Pause</button>
            <button type="button" class="text-btn primary" id="train-btn" data-act="train">Train</button>
          </div>
        </div>
        <div class="group">
          <p class="kicker">CHECKPOINT</p>
          <div class="stack">
            <button type="button" class="text-btn" data-act="save">Download</button>
            <button type="button" class="text-btn" data-act="idb-save">Save local</button>
            <button type="button" class="text-btn" data-act="idb-load">Load local</button>
            <input id="file" type="file" aria-label="Load a checkpoint file" accept=".json,.bin,application/json,application/octet-stream" />
            <div class="seg">
              <button type="button" data-act="quality" data-quality="low">Low</button>
              <button type="button" data-act="quality" data-quality="high">High</button>
            </div>
          </div>
        </div>
        <p class="help">WASD fly · Q/E yaw · Space/C up/down · 1/2/3 cameras · F flash · L lights · P pause · T train</p>
      </aside>
      <section class="stage-col">
        <div class="stage-wrap" id="stage-wrap">
          <canvas id="view" role="img" aria-label="3D view of the drone in the building"></canvas>
          <canvas id="hud"></canvas>
          <div class="readout" id="readout"></div>
          <p class="error" id="boot-note">Spinning up the building…</p>
        </div>
        <div class="bottom">
          <section class="panel">
            <div class="panel-head"><h2>EXPLORATION MAP</h2><span class="live" id="cov-num">0%</span></div>
            <canvas class="map-canvas" id="map" role="img" aria-label="Exploration map" width="480" height="180"></canvas>
            <div class="legend"><span><i class="swatch" style="background:#1ad6b0"></i>Explored</span><span><i class="swatch" style="background:#ff8a3d"></i>Frontier</span><span><i class="swatch" style="background:#c47b3a"></i>Shaft</span></div>
          </section>
          <section class="panel">
            <div class="panel-head"><h2>LIVE TRAINING</h2><span class="live" id="stage-num">0</span></div>
            <div class="metrics">
              <div class="metric"><span>UPDATE</span><strong id="ep-num">0</strong></div>
              <div class="metric"><span>RETURN</span><strong class="good" id="ret-num">0</strong></div>
              <div class="metric"><span>BEST</span><strong id="best-num">0</strong></div>
              <div class="metric"><span>STEPS/S</span><strong id="rate-num">0</strong></div>
            </div>
            <div class="charts">
              <div class="chart-box"><p>RETURN</p><div id="chart-return"></div></div>
              <div class="chart-box"><p>COVERAGE % · ENTROPY</p><div id="chart-cover"></div><div id="chart-entropy"></div></div>
            </div>
            <div class="replay-row">
              <button type="button" class="text-btn" data-act="replay" data-which="latest">Latest</button>
              <button type="button" class="text-btn" data-act="replay" data-which="best">Best</button>
              <button type="button" class="text-btn" data-act="replay" data-which="worst">Worst</button>
              <input id="scrub" aria-label="Replay position" class="slider" type="range" min="0" max="1" value="0" />
              <button type="button" class="text-btn" data-act="speed" data-speed="1">1×</button>
              <button type="button" class="text-btn" data-act="speed" data-speed="4">4×</button>
              <button type="button" class="text-btn" data-act="speed" data-speed="16">16×</button>
              <span id="replay-label"></span>
            </div>
          </section>
        </div>
      </section>
      <aside class="side">
        <div class="pip-wrap">
          <canvas id="pip" role="img" aria-label="Drone camera feed"></canvas>
          <canvas id="pip-hud"></canvas>
          <span class="pip-tag">FPV FEED</span>
        </div>
        <section class="panel detect-panel">
          <div class="panel-head"><h2>DETECTIONS</h2><span class="live" id="detect-count">0</span></div>
          <p class="help" id="detect-clear">seen-empty</p>
          <ul id="detect-list" class="detect-list"></ul>
        </section>
        <div class="stat-grid">
          <div><span>ALT</span><strong id="alt-num">0 m</strong></div>
          <div><span>SPD</span><strong id="spd-num">0 m/s</strong></div>
          <div><span>MODE</span><strong id="mode-label">DAY</strong></div>
          <div><span>KL</span><strong id="kl-num">0</strong></div>
        </div>
        <section class="panel" id="policy-panel" style="flex:1" hidden>
          <div class="panel-head"><h2>POLICY MAP</h2></div>
          <canvas class="heat-canvas" id="heat" width="300" height="220"></canvas>
        </section>
      </aside>
    </main>
  </div>`;
}

export function must<T extends Element>(root: ParentNode, sel: string): T {
  const el = root.querySelector(sel);
  if (!el) throw new Error(`missing ${sel}`);
  return el as T;
}

export function fitCanvas(canvas: HTMLCanvasElement): void {
  const w = Math.max(2, canvas.clientWidth || canvas.width);
  const h = Math.max(2, canvas.clientHeight || canvas.height);
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
}
