/**
 * ============================================================
 * SWARM UAV FORMATION GUIDANCE & APF SIMULATOR ENGINE (60 FPS)
 * Real-Time Leader-Follower Kinematics, APF Vector Physics & Tactical HUD
 * Author: Kadir Mercan (Mercan Studio)
 * ============================================================
 */

// ─── 1. VECTOR2D MATHEMATICS ENGINE ─────────────────────────
class Vector2D {
  constructor(x = 0, y = 0) {
    this.x = Number(x);
    this.y = Number(y);
  }

  add(v) { return new Vector2D(this.x + v.x, this.y + v.y); }
  sub(v) { return new Vector2D(this.x - v.x, this.y - v.y); }
  mult(n) { return new Vector2D(this.x * n, this.y * n); }
  mag() { return Math.hypot(this.x, this.y); }

  normalize() {
    const m = this.mag();
    return m === 0 ? new Vector2D(0, 0) : new Vector2D(this.x / m, this.y / m);
  }

  heading() {
    return Math.atan2(this.y, this.x);
  }

  rotate(angle) {
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    return new Vector2D(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
  }
}

// ─── 2. DRONE AGENT DEFINITION ──────────────────────────────
class Drone {
  constructor(id, role, x, y, color) {
    this.id = id;
    this.role = role; // 'leader' or 'follower'
    this.pos = new Vector2D(x, y);
    this.vel = new Vector2D(0, 0);
    this.heading = 0;
    this.maxSpeed = role === 'leader' ? 3.2 : 4.0; // followers have surplus thrust to catch up
    this.color = color;
    this.trail = [];
    this.maxTrail = 40;

    // Last computed forces for visualization
    this.fAtt = new Vector2D(0, 0);
    this.fRep = new Vector2D(0, 0);
    this.targetPos = new Vector2D(x, y);
    this.isAvoiding = false;
  }

  update(force, dt = 0.016) {
    this.vel = this.vel.add(force.mult(dt * 10));
    
    // Clamp to max speed
    if (this.vel.mag() > this.maxSpeed) {
      this.vel = this.vel.normalize().mult(this.maxSpeed);
    }

    this.pos = this.pos.add(this.vel);
    if (this.vel.mag() > 0.1) {
      this.heading = this.vel.heading();
    }

    // Save trail
    this.trail.push(new Vector2D(this.pos.x, this.pos.y));
    if (this.trail.length > this.maxTrail) {
      this.trail.shift();
    }
  }
}

// ─── 3. SIMULATOR ENVIRONMENT & STATE ───────────────────────
const sim = {
  canvas: null,
  ctx: null,
  width: 0,
  height: 0,
  
  drones: [],
  obstacles: [],
  
  formationType: 'v_formation',
  leaderMode: 'circle', // 'circle', 'figure8', 'mouse'
  simTime: 0,
  mousePos: new Vector2D(0, 0),

  // APF Tunable Parameters
  kAtt: 1.4,
  kRep: 55.0,
  d0: 80.0, // Obstacle influence threshold in px

  showVectors: true,
  showTrails: true,

  // Geometry slots relative to Leader (Body Frame: X = Forward, Y = Right)
  formationOffsets: {
    v_formation: [
      new Vector2D(-75, -55),  // Follower 1 (Port Wing)
      new Vector2D(-75, 55)    // Follower 2 (Starboard Wing)
    ],
    column: [
      new Vector2D(-70, 0),   // Follower 1 (Direct Trail)
      new Vector2D(-140, 0)   // Follower 2 (Rear Trail)
    ],
    echelon: [
      new Vector2D(-50, 60),   // Follower 1 (Right Echelon)
      new Vector2D(-100, 120)  // Follower 2 (Far Right)
    ],
    diamond: [
      new Vector2D(-70, -60),  // Follower 1
      new Vector2D(-70, 60)    // Follower 2
    ]
  }
};

// ─── 4. APF GUIDANCE VECTOR CALCULATOR ───────────────────────
function calculateAPF(drone, targetSlot, obstacles) {
  // 1. Attractive Force towards designated formation slot
  const toTarget = targetSlot.sub(drone.pos);
  const distTarget = toTarget.mag();
  const fAtt = toTarget.normalize().mult(sim.kAtt * Math.min(distTarget * 0.08, 4.0));

  // 2. Repulsive Force from Obstacles (Non-linear decay)
  let fRep = new Vector2D(0, 0);
  let isNearObstacle = false;

  obstacles.forEach(obs => {
    const toDrone = drone.pos.sub(obs.pos);
    const dist = toDrone.mag() - obs.radius;

    if (dist < sim.d0 && dist > 1.0) {
      isNearObstacle = true;
      // Repulsive potential derivative: F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * norm(vector)
      const repMag = sim.kRep * (1.0 / dist - 1.0 / sim.d0) * (1.0 / (dist * dist)) * 800;
      fRep = fRep.add(toDrone.normalize().mult(Math.min(repMag, 12.0)));
    }
  });

  // 3. Inter-Agent Avoidance (Followers repel each other to prevent mid-air collision)
  sim.drones.forEach(other => {
    if (other.id !== drone.id) {
      const distOther = drone.pos.sub(other.pos).mag();
      if (distOther < 45.0 && distOther > 1.0) {
        const repAgent = drone.pos.sub(other.pos).normalize().mult(3.5);
        fRep = fRep.add(repAgent);
      }
    }
  });

  drone.fAtt = fAtt;
  drone.fRep = fRep;
  drone.isAvoiding = isNearObstacle;
  drone.targetPos = targetSlot;

  // Resultant force
  return fAtt.add(fRep);
}

// ─── 5. LEADER TRAJECTORY PATH GENERATOR ─────────────────────
function updateLeader(dt) {
  const leader = sim.drones[0];
  sim.simTime += dt;

  const cx = sim.width / 2;
  const cy = sim.height / 2;

  let targetPos;

  if (sim.leaderMode === 'circle') {
    const radius = Math.min(sim.width, sim.height) * 0.32;
    const speed = 0.5;
    targetPos = new Vector2D(
      cx + Math.cos(sim.simTime * speed) * radius,
      cy + Math.sin(sim.simTime * speed) * radius
    );
  } else if (sim.leaderMode === 'figure8') {
    const rx = Math.min(sim.width, sim.height) * 0.36;
    const ry = Math.min(sim.width, sim.height) * 0.22;
    const speed = 0.45;
    targetPos = new Vector2D(
      cx + Math.sin(sim.simTime * speed) * rx,
      cy + Math.sin(sim.simTime * speed * 2) * ry
    );
  } else {
    // Mouse tracking
    targetPos = sim.mousePos;
  }

  // Smooth steering to target
  const diff = targetPos.sub(leader.pos);
  const steerForce = diff.normalize().mult(Math.min(diff.mag() * 0.1, 3.5));
  leader.update(steerForce, dt);
}

// ─── 6. RENDER ENGINE & RADAR DISPLAY ────────────────────────
function drawRadarGrid() {
  const ctx = sim.ctx;
  const w = sim.width;
  const h = sim.height;
  const cx = w / 2;
  const cy = h / 2;

  // Background
  ctx.fillStyle = '#05070a';
  ctx.fillRect(0, 0, w, h);

  // Concentric Radar Rings
  ctx.strokeStyle = 'rgba(16, 185, 129, 0.08)';
  ctx.lineWidth = 1;
  const maxR = Math.hypot(cx, cy);
  for (let r = 80; r < maxR; r += 80) {
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // Distance label in NED meters (10px = 1m)
    ctx.fillStyle = 'rgba(16, 185, 129, 0.25)';
    ctx.font = '10px "JetBrains Mono"';
    ctx.fillText(`${r / 10}m`, cx + r + 4, cy - 4);
  }

  // Crosshairs
  ctx.beginPath();
  ctx.moveTo(0, cy); ctx.lineTo(w, cy);
  ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
  ctx.strokeStyle = 'rgba(16, 185, 129, 0.12)';
  ctx.stroke();
}

function drawDrone(d) {
  const ctx = sim.ctx;

  // 1. Draw Flight Trail
  if (sim.showTrails && d.trail.length > 1) {
    ctx.beginPath();
    ctx.moveTo(d.trail[0].x, d.trail[0].y);
    for (let i = 1; i < d.trail.length; i++) {
      ctx.lineTo(d.trail[i].x, d.trail[i].y);
    }
    ctx.strokeStyle = d.color + '40';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  // 2. Draw Target Slot & Vector Overlay
  if (d.role === 'follower') {
    // Virtual Slot
    ctx.beginPath();
    ctx.arc(d.targetPos.x, d.targetPos.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.25)';
    ctx.fill();

    // Dotted tether line
    ctx.beginPath();
    ctx.setLineDash([3, 3]);
    ctx.moveTo(d.pos.x, d.pos.y);
    ctx.lineTo(d.targetPos.x, d.targetPos.y);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.stroke();
    ctx.setLineDash([]);

    // Force Vectors
    if (sim.showVectors) {
      drawVector(d.pos, d.fAtt.mult(15), '#10b981'); // Green F_att
      if (d.isAvoiding) {
        drawVector(d.pos, d.fRep.mult(8), '#ef4444');  // Red F_rep
      }
      drawVector(d.pos, d.vel.mult(12), '#38bdf8');   // Cyan Resultant V
    }
  }

  // 3. Draw Aircraft Airframe (Triangle with Heading)
  ctx.save();
  ctx.translate(d.pos.x, d.pos.y);
  ctx.rotate(d.heading);

  const size = d.role === 'leader' ? 14 : 11;
  ctx.beginPath();
  ctx.moveTo(size * 1.4, 0);
  ctx.lineTo(-size, -size * 0.8);
  ctx.lineTo(-size * 0.5, 0);
  ctx.lineTo(-size, size * 0.8);
  ctx.closePath();

  ctx.fillStyle = d.color;
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.2;
  ctx.stroke();

  ctx.restore();

  // 4. Aircraft Label & Speed
  ctx.font = '10px "JetBrains Mono"';
  ctx.fillStyle = '#f3f4f6';
  ctx.fillText(`UAV-${d.id} [${(d.vel.mag() * 0.8).toFixed(1)}m/s]`, d.pos.x + 14, d.pos.y - 10);
}

function drawVector(origin, vec, color) {
  if (vec.mag() < 1) return;
  const ctx = sim.ctx;
  const endX = origin.x + vec.x;
  const endY = origin.y + vec.y;

  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(endX, endY);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();

  // Arrowhead
  const angle = Math.atan2(vec.y, vec.x);
  ctx.beginPath();
  ctx.moveTo(endX, endY);
  ctx.lineTo(endX - 6 * Math.cos(angle - Math.PI / 6), endY - 6 * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(endX - 6 * Math.cos(angle + Math.PI / 6), endY - 6 * Math.sin(angle + Math.PI / 6));
  ctx.fillStyle = color;
  ctx.fill();
}

function drawObstacle(obs) {
  const ctx = sim.ctx;

  // Influence boundary
  ctx.beginPath();
  ctx.arc(obs.pos.x, obs.pos.y, obs.radius + sim.d0, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(239, 68, 68, 0.1)';
  ctx.lineWidth = 1;
  ctx.stroke();

  // Solid Core
  ctx.beginPath();
  ctx.arc(obs.pos.x, obs.pos.y, obs.radius, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(239, 68, 68, 0.25)';
  ctx.fill();
  ctx.strokeStyle = '#ef4444';
  ctx.lineWidth = 2;
  ctx.stroke();

  // Threat Cross
  ctx.beginPath();
  ctx.moveTo(obs.pos.x - 6, obs.pos.y); ctx.lineTo(obs.pos.x + 6, obs.pos.y);
  ctx.moveTo(obs.pos.x, obs.pos.y - 6); ctx.lineTo(obs.pos.x, obs.pos.y + 6);
  ctx.strokeStyle = '#ef4444';
  ctx.stroke();

  ctx.font = '10px "JetBrains Mono"';
  ctx.fillStyle = '#fca5a5';
  ctx.fillText('RADAR THREAT', obs.pos.x - 35, obs.pos.y + obs.radius + 14);
}

// ─── 7. MAIN SIMULATION LOOP (60 FPS) ───────────────────────
let lastFrameTime = performance.now();
let fpsCount = 0;
let fpsTimer = 0;

function loop(currentTime) {
  const dt = Math.min((currentTime - lastFrameTime) / 1000, 0.05);
  lastFrameTime = currentTime;

  // FPS Counter
  fpsCount++;
  if (currentTime - fpsTimer >= 1000) {
    document.getElementById('hudSimFps').textContent = `${fpsCount} FPS`;
    fpsCount = 0;
    fpsTimer = currentTime;
  }

  // 1. Update Leader
  updateLeader(dt);
  const leader = sim.drones[0];

  // 2. Update Followers with APF
  const slots = sim.formationOffsets[sim.formationType];

  for (let i = 1; i < sim.drones.length; i++) {
    const follower = sim.drones[i];
    const offsetBody = slots[i - 1]; // Body frame offset

    // Rotate body offset by Leader's heading into Global NED Frame
    const offsetWorld = offsetBody.rotate(leader.heading);
    const targetSlot = leader.pos.add(offsetWorld);

    // Compute resultant APF steering force
    const apfForce = calculateAPF(follower, targetSlot, sim.obstacles);
    follower.update(apfForce, dt);
  }

  // 3. Render Canvas
  drawRadarGrid();
  sim.obstacles.forEach(drawObstacle);
  sim.drones.forEach(drawDrone);

  // 4. Update Cockpit Telemetry Table
  updateHUDTable();

  requestAnimationFrame(loop);
}

function updateHUDTable() {
  const d0 = sim.drones[0];
  const d1 = sim.drones[1];
  const d2 = sim.drones[2];

  document.getElementById('speed0').textContent = `${(d0.vel.mag() * 0.8).toFixed(1)} m/s`;
  document.getElementById('speed1').textContent = `${(d1.vel.mag() * 0.8).toFixed(1)} m/s`;
  document.getElementById('speed2').textContent = `${(d2.vel.mag() * 0.8).toFixed(1)} m/s`;

  const err1 = d1.pos.sub(d1.targetPos).mag() * 0.05;
  const err2 = d2.pos.sub(d2.targetPos).mag() * 0.05;

  document.getElementById('err1').textContent = `${err1.toFixed(2)} m`;
  document.getElementById('err2').textContent = `${err2.toFixed(2)} m`;

  const tag1 = document.getElementById('statTag1');
  const tag2 = document.getElementById('statTag2');

  tag1.className = d1.isAvoiding ? 'stat-tag avoiding' : 'stat-tag ok';
  tag1.textContent = d1.isAvoiding ? 'AVOIDING' : 'IN FORM';

  tag2.className = d2.isAvoiding ? 'stat-tag avoiding' : 'stat-tag ok';
  tag2.textContent = d2.isAvoiding ? 'AVOIDING' : 'IN FORM';
}

// ─── 8. PUBLIC USER INTERFACE ACTIONS ───────────────────────
window.setFormation = function(type) {
  sim.formationType = type;
  document.querySelectorAll('.form-btn').forEach(b => b.classList.remove('active'));
  const btnMap = {
    v_formation: 'btnFormV',
    column: 'btnFormColumn',
    echelon: 'btnFormEchelon',
    diamond: 'btnFormDiamond'
  };
  const activeBtn = document.getElementById(btnMap[type]);
  if (activeBtn) activeBtn.classList.add('active');
};

window.setLeaderFlightMode = function(mode) {
  sim.leaderMode = mode;
  document.querySelectorAll('.mission-btn').forEach(b => b.classList.remove('active'));
  const btnMap = {
    circle: 'btnPatrolCircle',
    figure8: 'btnPatrolFigure8',
    mouse: 'btnFollowMouse'
  };
  const activeBtn = document.getElementById(btnMap[mode]);
  if (activeBtn) activeBtn.classList.add('active');
};

window.updateApfParams = function() {
  sim.kAtt = parseFloat(document.getElementById('sliderKatt').value);
  sim.kRep = parseFloat(document.getElementById('sliderKrep').value);
  sim.d0 = parseFloat(document.getElementById('sliderD0').value);

  document.getElementById('valKatt').textContent = sim.kAtt.toFixed(1);
  document.getElementById('valKrep').textContent = sim.kRep.toFixed(1);
  document.getElementById('valD0').textContent = `${sim.d0} px`;
};

window.clearObstacles = function() {
  sim.obstacles = [];
};

window.resetSwarm = function() {
  const cx = sim.width / 2;
  const cy = sim.height / 2;
  sim.drones[0].pos = new Vector2D(cx, cy);
  sim.drones[1].pos = new Vector2D(cx - 80, cy - 60);
  sim.drones[2].pos = new Vector2D(cx - 80, cy + 60);
  sim.drones.forEach(d => { d.vel = new Vector2D(0, 0); d.trail = []; });
};

// ─── 9. INITIALIZATION & EVENT LISTENERS ────────────────────
window.addEventListener('DOMContentLoaded', () => {
  sim.canvas = document.getElementById('radarCanvas');
  sim.ctx = sim.canvas.getContext('2d');

  function resize() {
    const parent = sim.canvas.parentElement;
    sim.width = parent.clientWidth;
    sim.height = parent.clientHeight;
    sim.canvas.width = sim.width;
    sim.canvas.height = sim.height;
  }
  window.addEventListener('resize', resize);
  resize();

  const cx = sim.width / 2;
  const cy = sim.height / 2;
  sim.mousePos = new Vector2D(cx, cy);

  // Initialize Drones (1 Leader + 2 Followers)
  sim.drones = [
    new Drone(0, 'leader', cx, cy, '#3b82f6'),         // Leader (Blue)
    new Drone(1, 'follower', cx - 80, cy - 60, '#06b6d4'), // Follower 1 (Cyan)
    new Drone(2, 'follower', cx - 80, cy + 60, '#10b981')  // Follower 2 (Green)
  ];

  // Spawn Initial Obstacles
  sim.obstacles = [
    { pos: new Vector2D(cx + 120, cy - 40), radius: 24 },
    { pos: new Vector2D(cx - 100, cy + 80), radius: 20 }
  ];

  // Canvas Click: Spawn/Remove Obstacles
  sim.canvas.addEventListener('click', (e) => {
    const rect = sim.canvas.getBoundingClientRect();
    const clickPos = new Vector2D(e.clientX - rect.left, e.clientY - rect.top);

    // Check if clicked an existing obstacle to delete
    const existingIdx = sim.obstacles.findIndex(obs => obs.pos.sub(clickPos).mag() <= obs.radius + 8);
    if (existingIdx !== -1) {
      sim.obstacles.splice(existingIdx, 1);
    } else {
      // Spawn new obstacle
      sim.obstacles.push({ pos: clickPos, radius: 22 });
    }
  });

  // Track Mouse Pos
  sim.canvas.addEventListener('mousemove', (e) => {
    const rect = sim.canvas.getBoundingClientRect();
    sim.mousePos = new Vector2D(e.clientX - rect.left, e.clientY - rect.top);
  });

  // Checkbox bindings
  document.getElementById('chkShowVectors').addEventListener('change', (e) => {
    sim.showVectors = e.target.checked;
  });
  document.getElementById('chkShowTrails').addEventListener('change', (e) => {
    sim.showTrails = e.target.checked;
  });

  // Start 60 FPS Loop
  requestAnimationFrame(loop);
});
