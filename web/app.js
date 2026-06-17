const state = {
  config: null,
  status: null,
  robotId: "unitree",
  armed: false,
  drive: { linear: 0, angular: 0 },
  lastStatusAt: 0,
};

const $ = (selector) => document.querySelector(selector);

function fmtTime(date = new Date()) {
  return date.toLocaleTimeString("en-GB", { hour12: false });
}

function setText(selector, value) {
  const node = $(selector);
  if (node) node.textContent = value;
}

function statusClass(cam) {
  if (!cam.exists) return "bad";
  if (cam.busy) return "warn";
  return "good";
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

async function fetchJson(path) {
  const response = await fetch(path, { cache: "no-store" });
  if (!response.ok) throw new Error(`${path} ${response.status}`);
  return response.json();
}

async function boot() {
  state.config = await fetchJson("/api/config");
  renderRobotSelect();
  renderTopicMap();
  renderCameraGrid();
  await refreshStatus();
  setInterval(refreshStatus, 2500);
  setInterval(updateClock, 500);
  requestAnimationFrame(drawScene);
}

function updateClock() {
  setText("#clock", fmtTime());
  if (state.lastStatusAt) {
    const age = Math.max(0, (Date.now() - state.lastStatusAt) / 1000);
    setText("#status-age", `${age.toFixed(1)}s`);
  }
}

function renderRobotSelect() {
  const select = $("#robot-select");
  const robots = state.config.robots;
  select.innerHTML = Object.values(robots)
    .map((robot) => `<option value="${robot.id}">${escapeHtml(robot.label)}</option>`)
    .join("");
  select.value = state.robotId;
  select.addEventListener("change", () => {
    state.robotId = select.value;
    setText("#robot-pill", robots[state.robotId].label);
    renderTopicMap();
  });
  setText("#robot-pill", robots[state.robotId].label);
}

function renderTopicMap() {
  const robot = state.config.robots[state.robotId];
  const map = $("#topic-map");
  map.innerHTML = Object.entries(robot.topics)
    .map(
      ([key, topic]) => `
        <div class="topic-row">
          <b>${escapeHtml(key)}</b>
          <code>${escapeHtml(topic)}</code>
        </div>
      `,
    )
    .join("");
}

function renderCameraGrid() {
  const grid = $("#camera-grid");
  grid.innerHTML = state.config.cameras
    .map(
      (cam, index) => `
        <article class="camera-card" id="cam-${cam.id}">
          <div class="camera-head">
            <span class="badge">${index + 1}. ${escapeHtml(cam.label)}</span>
            <span class="badge" data-cam-status="${cam.id}">WAIT</span>
          </div>
          <img src="/stream/${cam.id}" alt="${escapeHtml(cam.label)} stream" onerror="this.style.display='none'; this.nextElementSibling.style.display='grid';" />
          <div class="camera-empty" style="display:none">
            <div>
              <b>${escapeHtml(cam.role)}</b>
              <p>${escapeHtml(cam.device)}</p>
            </div>
          </div>
          <div class="camera-foot">
            <span class="badge">${escapeHtml(cam.device)}</span>
            <span class="badge">${cam.fov} deg</span>
          </div>
        </article>
      `,
    )
    .join("");
}

async function refreshStatus() {
  try {
    state.status = await fetchJson("/api/status");
    state.lastStatusAt = Date.now();
    renderStatus();
  } catch (error) {
    setText("#status-age", "offline");
    console.error(error);
  }
}

function renderStatus() {
  renderCameraStatus();
  renderRosStatus();
  renderDockerStatus();
}

function renderCameraStatus() {
  const cams = state.status.cameras || [];
  const live = cams.filter((cam) => cam.exists && !cam.busy).length;
  setText("#camera-summary", `${live}/${cams.length}`);

  cams.forEach((cam) => {
    const node = document.querySelector(`[data-cam-status="${cam.id}"]`);
    if (!node) return;
    node.className = `badge ${statusClass(cam)}`;
    node.textContent = !cam.exists ? "MISSING" : cam.busy ? "BUSY" : "READY";
  });

  $("#tab-cameras").innerHTML = cams
    .map((cam) => {
      const users = cam.users?.length
        ? cam.users.map((user) => `${user.command || "process"}:${user.pid}`).join(", ")
        : "none";
      const cardType = cam.v4l2?.card_type || "not detected";
      return `
        <div class="status-row">
          <div class="split-line"><b>${escapeHtml(cam.label)}</b><span class="${statusClass(cam)}">${cam.exists ? "present" : "missing"}</span></div>
          <div class="split-line"><span>alias</span><span>${escapeHtml(cam.device)}</span></div>
          <div class="split-line"><span>target</span><span>${escapeHtml(cam.resolved || "-")}</span></div>
          <div class="split-line"><span>card</span><span>${escapeHtml(cardType)}</span></div>
          <div class="split-line"><span>users</span><span>${escapeHtml(users)}</span></div>
        </div>
      `;
    })
    .join("");
}

function renderRosStatus() {
  const ros = state.status.ros || { available: false, topics: [] };
  if (!ros.available) {
    $("#tab-ros").innerHTML = `
      <div class="status-row">
        <b>ROS 2 host CLI unavailable</b>
        <code>source /opt/ros/humble/setup.bash or bridge through container</code>
      </div>
      ${renderExpectedTopics()}
    `;
    return;
  }
  $("#tab-ros").innerHTML =
    `<div class="status-row"><b>${ros.count} topics detected</b><code>ros2 topic list -t</code></div>` +
    ros.topics
      .slice(0, 42)
      .map(
        (topic) => `
          <div class="status-row">
            <b>${escapeHtml(topic.name)}</b>
            <code>${escapeHtml(topic.type)}</code>
          </div>
        `,
      )
      .join("");
}

function renderExpectedTopics() {
  const robot = state.config.robots[state.robotId];
  return Object.values(robot.topics)
    .map(
      (topic) => `
        <div class="status-row">
          <b>${escapeHtml(topic)}</b>
          <code>expected ${escapeHtml(robot.label)} topic</code>
        </div>
      `,
    )
    .join("");
}

function renderDockerStatus() {
  const docker = state.status.docker || { containers: [], deviceAccess: [] };
  const accessNames = new Set((docker.deviceAccess || []).map((item) => item.name));
  $("#tab-docker").innerHTML =
    (docker.deviceAccess || [])
      .map(
        (item) => `
          <div class="container-row">
            <b>${escapeHtml(item.name)}</b>
            <code>has broad /dev access</code>
          </div>
        `,
      )
      .join("") +
    (docker.containers || [])
      .map((item) => {
        const device = accessNames.has(item.name) ? "device-visible" : "no camera bind";
        const status = item.status.includes("Restarting") ? "bad" : "good";
        return `
          <div class="container-row">
            <div class="split-line"><b>${escapeHtml(item.name)}</b><span class="${status}">${escapeHtml(item.status)}</span></div>
            <div class="split-line"><span>${escapeHtml(device)}</span><span>${escapeHtml(item.id)}</span></div>
          </div>
        `;
      })
      .join("");
}

function setupInteractions() {
  $("#refresh-btn").addEventListener("click", refreshStatus);

  document.querySelectorAll(".tab").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".tab").forEach((tab) => tab.classList.remove("active"));
      document.querySelectorAll(".tab-view").forEach((view) => view.classList.remove("active"));
      button.classList.add("active");
      $(`#tab-${button.dataset.tab}`).classList.add("active");
    });
  });

  const arm = $("#arm-button");
  arm.addEventListener("mousedown", () => {
    state.armed = true;
    arm.classList.add("armed");
    arm.textContent = "ARMED";
    setText("#teleop-state", "ARMED");
  });
  ["mouseup", "mouseleave", "blur"].forEach((eventName) => {
    arm.addEventListener(eventName, () => {
      state.armed = false;
      state.drive = { linear: 0, angular: 0 };
      arm.classList.remove("armed");
      arm.textContent = "Hold To Arm";
      setText("#teleop-state", "LOCKED");
      updateDriveMeters();
    });
  });

  document.querySelectorAll("[data-drive]").forEach((button) => {
    button.addEventListener("mousedown", () => {
      if (!state.armed) return;
      const drive = button.dataset.drive;
      state.drive = {
        forward: { linear: 0.35, angular: 0 },
        back: { linear: -0.25, angular: 0 },
        left: { linear: 0, angular: 0.45 },
        right: { linear: 0, angular: -0.45 },
        stop: { linear: 0, angular: 0 },
      }[drive];
      updateDriveMeters();
    });
    button.addEventListener("mouseup", () => {
      state.drive = { linear: 0, angular: 0 };
      updateDriveMeters();
    });
  });
}

function updateDriveMeters() {
  setText("#linear-meter", state.drive.linear.toFixed(2));
  setText("#angular-meter", state.drive.angular.toFixed(2));
}

function project(point, width, height, yaw) {
  const [x0, y0, z0] = point;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const x = x0 * cy - z0 * sy;
  const z = x0 * sy + z0 * cy;
  const y = y0;
  const scale = 420 / (z + 8);
  return [width * 0.5 + x * scale, height * 0.55 - y * scale, scale];
}

function drawLine(ctx, a, b, color, width = 2) {
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
}

function drawDot(ctx, p, radius, color) {
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(p[0], p[1], radius, 0, Math.PI * 2);
  ctx.fill();
}

function drawScene(now = 0) {
  const canvas = $("#scene-canvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width * dpr));
  const height = Math.max(1, Math.floor(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, width, height);
  ctx.save();
  ctx.scale(dpr, dpr);
  drawSceneInner(ctx, rect.width, rect.height, now / 1000);
  ctx.restore();
  requestAnimationFrame(drawScene);
}

function drawSceneInner(ctx, width, height, t) {
  const yaw = Math.sin(t * 0.35) * 0.55;
  const gridY = height * 0.72;

  ctx.strokeStyle = "rgba(97, 213, 255, 0.12)";
  ctx.lineWidth = 1;
  for (let i = -8; i <= 8; i++) {
    const a = project([i, -1.0, 0], width, height, yaw);
    const b = project([i, -1.0, 8], width, height, yaw);
    drawLine(ctx, a, b, "rgba(97, 213, 255, 0.10)", 1);
  }
  for (let z = 0; z <= 8; z++) {
    const a = project([-8, -1.0, z], width, height, yaw);
    const b = project([8, -1.0, z], width, height, yaw);
    drawLine(ctx, a, b, "rgba(97, 213, 255, 0.10)", 1);
  }

  const walk = Math.sin(t * 3.2) * 0.24;
  const body = project([0, 0.42 + Math.sin(t * 3.2) * 0.025, 2.4], width, height, yaw);
  const head = project([0, 1.02, 2.38], width, height, yaw);
  const rear = project([0, 0.36, 2.95], width, height, yaw);
  const front = project([0, 0.42, 1.86], width, height, yaw);
  drawLine(ctx, rear, front, "#d8f4ff", 9);
  drawLine(ctx, body, head, "#86a8ff", 5);
  drawDot(ctx, head, 9, "#61d5ff");
  drawDot(ctx, body, 7, "#5ee493");

  const legs = [
    [-0.48, 0.15, 1.9, walk],
    [0.48, 0.15, 1.9, -walk],
    [-0.48, 0.12, 2.9, -walk],
    [0.48, 0.12, 2.9, walk],
  ];
  legs.forEach(([x, y, z, phase]) => {
    const hip = project([x, y, z], width, height, yaw);
    const knee = project([x * 1.05, -0.42, z + phase], width, height, yaw);
    const foot = project([x * 1.12, -0.98, z - phase * 0.65], width, height, yaw);
    drawLine(ctx, hip, knee, "#d8f4ff", 4);
    drawLine(ctx, knee, foot, "#86a8ff", 4);
    drawDot(ctx, foot, 5, "#ffbf5e");
  });

  const cam = project([0, 0.82, 1.76], width, height, yaw);
  const frustum = [
    project([-1.55, -0.35, 4.4], width, height, yaw),
    project([1.55, -0.35, 4.4], width, height, yaw),
    project([1.15, 1.05, 4.4], width, height, yaw),
    project([-1.15, 1.05, 4.4], width, height, yaw),
  ];
  frustum.forEach((p) => drawLine(ctx, cam, p, "rgba(94, 228, 147, 0.52)", 1.5));
  for (let i = 0; i < frustum.length; i++) {
    drawLine(ctx, frustum[i], frustum[(i + 1) % frustum.length], "rgba(94, 228, 147, 0.62)", 1.5);
  }

  for (let i = 0; i < 70; i++) {
    const x = Math.sin(i * 12.989 + t * 0.2) * 1.5;
    const y = -0.75 + ((i * 37) % 100) / 260;
    const z = 3.5 + ((i * 19) % 90) / 28;
    const p = project([x, y, z], width, height, yaw);
    drawDot(ctx, p, Math.max(1.1, 2.8 * p[2] * 0.014), `rgba(255, 191, 94, ${0.25 + (i % 5) * 0.09})`);
  }

  const labelY = Math.max(28, gridY - 260);
  ctx.fillStyle = "rgba(237, 244, 247, 0.92)";
  ctx.font = "12px ui-sans-serif, system-ui";
  ctx.fillText("Unitree adapter preview", 16, labelY);
  ctx.fillStyle = "rgba(140, 153, 167, 0.95)";
  ctx.fillText("TF, JointState, depth cloud hooks are isolated in config/robots", 16, labelY + 20);
}

setupInteractions();
boot().catch((error) => {
  console.error(error);
  document.body.insertAdjacentHTML(
    "beforeend",
    `<div style="position:fixed;inset:16px;display:grid;place-items:center;background:#090b0fcc;color:#ff6b72;font:14px system-ui">FMS failed to boot: ${escapeHtml(error.message)}</div>`,
  );
});

