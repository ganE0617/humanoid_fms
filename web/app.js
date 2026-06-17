const state = {
  config: null,
  status: null,
  robotId: "unitree",
  armed: false,
  drive: { linear: 0, angular: 0 },
  lastStatusAt: 0,
  urdf: null,
  robotState: null,
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
  await loadUrdf();
  await refreshStatus();
  await refreshRobotState();
  setInterval(refreshStatus, 2500);
  setInterval(refreshRobotState, 80);
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
    loadUrdf();
  });
  setText("#robot-pill", robots[state.robotId].label);
}

async function loadUrdf() {
  try {
    const payload = await fetchJson(`/api/robots/${state.robotId}/urdf`);
    state.urdf = parseUrdf(payload);
    setText("#scene-root", `${state.urdf.model || state.config.robots[state.robotId].label} / ${state.urdf.root || payload.rootFrame || "root"}`);
    setText("#scene-joints", `${state.urdf.movingJoints.length}/${state.urdf.joints.length} moving joints`);
    setText("#scene-links", `${state.urdf.links.size} links`);
  } catch (error) {
    state.urdf = null;
    setText("#scene-root", "URDF missing");
    setText("#scene-source", "run sync script");
    setText("#scene-joints", "0 joints");
    setText("#scene-links", "0 links");
    console.error(error);
  }
}

async function refreshRobotState() {
  try {
    state.robotState = await fetchJson("/api/robot-state");
    const source = state.robotState.source || "unknown";
    const age = state.robotState.lastJointTime
      ? `${Math.max(0, Date.now() / 1000 - state.robotState.lastJointTime).toFixed(1)}s`
      : "";
    const tfAge = state.robotState.lastTfTime
      ? `${Math.max(0, Date.now() / 1000 - state.robotState.lastTfTime).toFixed(1)}s`
      : "";
    const label =
      source === "live"
        ? `live /joint_states ${age}`
        : source === "tf"
          ? `live /tf ${tfAge}`
          : "waiting for /joint_states or /tf";
    setText("#scene-source", label.trim());
  } catch (error) {
    state.robotState = { source: "offline", joints: {}, transforms: [] };
    setText("#scene-source", "state offline");
  }
}

function parseVector(value, fallback = [0, 0, 0]) {
  if (!value) return fallback;
  const parts = value.trim().split(/\s+/).map(Number);
  return parts.length >= 3 && parts.every(Number.isFinite) ? parts.slice(0, 3) : fallback;
}

function parseUrdf(payload) {
  const doc = new DOMParser().parseFromString(payload.xml, "application/xml");
  const links = new Set([...doc.querySelectorAll("link")].map((link) => link.getAttribute("name")).filter(Boolean));
  const joints = [...doc.querySelectorAll("joint")]
    .map((joint, index) => {
      const origin = joint.querySelector("origin");
      return {
        index,
        name: joint.getAttribute("name") || `joint_${index}`,
        type: joint.getAttribute("type") || "fixed",
        parent: joint.querySelector("parent")?.getAttribute("link") || "",
        child: joint.querySelector("child")?.getAttribute("link") || "",
        xyz: parseVector(origin?.getAttribute("xyz"), [0, 0, 0]),
        rpy: parseVector(origin?.getAttribute("rpy"), [0, 0, 0]),
        axis: parseVector(joint.querySelector("axis")?.getAttribute("xyz"), [1, 0, 0]),
      };
    })
    .filter((joint) => joint.parent && joint.child);
  const children = new Map();
  const childLinks = new Set();
  joints.forEach((joint) => {
    childLinks.add(joint.child);
    if (!children.has(joint.parent)) children.set(joint.parent, []);
    children.get(joint.parent).push(joint);
  });
  const root = payload.rootFrame || [...links].find((link) => !childLinks.has(link)) || "base_link";
  const movingJoints = joints.filter((joint) => joint.type !== "fixed");
  return {
    path: payload.path,
    source: payload.source,
    model: payload.model,
    root,
    links,
    joints,
    movingJoints,
    children,
  };
}

function renderTopicMap() {
  const robot = state.config.robots[state.robotId];
  const map = $("#topic-map");
  const urdf = robot.urdf || {};
  const metaRows = [
    ["model", robot.model || robot.label],
    ["repo", robot.repo || urdf.source],
    ["urdf", urdf.path],
    ["root", urdf.rootFrame],
  ];
  const metadata = metaRows
    .filter(([, value]) => value)
    .map(
      ([key, value]) => `
        <div class="topic-row robot-meta-row">
          <b>${escapeHtml(key)}</b>
          <code>${escapeHtml(value)}</code>
        </div>
      `,
    )
    .join("");
  const topics = Object.entries(robot.topics)
    .map(
      ([key, topic]) => `
        <div class="topic-row">
          <b>${escapeHtml(key)}</b>
          <code>${escapeHtml(topic)}</code>
        </div>
      `,
    )
    .join("");
  map.innerHTML = metadata + topics;
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

function matIdentity() {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function matMul(a, b) {
  const out = new Array(16).fill(0);
  for (let row = 0; row < 4; row++) {
    for (let col = 0; col < 4; col++) {
      for (let k = 0; k < 4; k++) out[row * 4 + col] += a[row * 4 + k] * b[k * 4 + col];
    }
  }
  return out;
}

function matTranslate([x, y, z]) {
  return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
}

function matRotX(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
}

function matRotY(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}

function matRotZ(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

function matRpy([r, p, y]) {
  return matMul(matMul(matRotZ(y), matRotY(p)), matRotX(r));
}

function matAxis(axis, angle) {
  const [x0, y0, z0] = axis;
  const len = Math.hypot(x0, y0, z0) || 1;
  const x = x0 / len, y = y0 / len, z = z0 / len;
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0,
    0, 0, 0, 1,
  ];
}

function matQuat([x, y, z, w]) {
  const len = Math.hypot(x, y, z, w) || 1;
  x /= len;
  y /= len;
  z /= len;
  w /= len;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;
  return [
    1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0,
    2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0,
    2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0,
    0, 0, 0, 1,
  ];
}

function normalizeFrameId(frameId) {
  return String(frameId || "").replace(/^\/+/, "");
}

function tfByChild() {
  const map = new Map();
  const transforms = state.robotState?.transforms || [];
  transforms.forEach((transform) => {
    const child = normalizeFrameId(transform.child);
    const parent = normalizeFrameId(transform.parent);
    if (!child || !parent) return;
    map.set(child, {
      parent,
      local: matMul(matTranslate(transform.translation || [0, 0, 0]), matQuat(transform.rotation || [0, 0, 0, 1])),
    });
  });
  return map;
}

function transformPoint(m, [x, y, z]) {
  return [
    m[0] * x + m[1] * y + m[2] * z + m[3],
    m[4] * x + m[5] * y + m[6] * z + m[7],
    m[8] * x + m[9] * y + m[10] * z + m[11],
  ];
}

function buildUrdfFrames() {
  if (!state.urdf) return null;
  const frames = new Map([[state.urdf.root, matIdentity()]]);
  const edges = [];
  const jointsByName = state.robotState?.joints || {};
  const transforms = tfByChild();
  const visit = (link) => {
    const parentMat = frames.get(link);
    const childJoints = state.urdf.children.get(link) || [];
    childJoints.forEach((joint) => {
      let angle = Number(jointsByName[joint.name] || 0);
      if (!Number.isFinite(angle)) angle = 0;
      const tf = transforms.get(normalizeFrameId(joint.child));
      let local;
      if (tf && tf.parent === normalizeFrameId(link)) {
        local = tf.local;
      } else {
        local = matMul(matTranslate(joint.xyz), matRpy(joint.rpy));
        if (joint.type === "revolute" || joint.type === "continuous") {
          local = matMul(local, matAxis(joint.axis, angle));
        } else if (joint.type === "prismatic") {
          local = matMul(local, matTranslate(joint.axis.map((axis) => axis * angle)));
        }
      }
      const childMat = matMul(parentMat, local);
      frames.set(joint.child, childMat);
      edges.push({ joint, parent: transformPoint(parentMat, [0, 0, 0]), child: transformPoint(childMat, [0, 0, 0]) });
      visit(joint.child);
    });
  };
  visit(state.urdf.root);
  const points = edges.flatMap((edge) => [edge.parent, edge.child]);
  const bbox = points.length
    ? {
        min: points[0].map((_, axis) => Math.min(...points.map((point) => point[axis]))),
        max: points[0].map((_, axis) => Math.max(...points.map((point) => point[axis]))),
      }
    : { min: [0, 0, 0], max: [0, 0, 1] };
  return { frames, edges, bbox };
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
  const yaw = Math.sin(t * 0.18) * 0.28;
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

  const urdfFrames = buildUrdfFrames();
  if (urdfFrames && urdfFrames.edges.length) {
    const min = urdfFrames.bbox.min;
    const max = urdfFrames.bbox.max;
    const size = max.map((value, axis) => Math.max(value - min[axis], 0.001));
    const center = max.map((value, axis) => (value + min[axis]) / 2);
    const scale = Math.min(3.6 / Math.max(size[1], 0.55), 3.4 / Math.max(size[2], 0.9), 3.2);
    const mapPoint = (point) => [
      (point[1] - center[1]) * scale,
      (point[2] - min[2]) * scale - 1.05,
      (point[0] - center[0]) * scale + 2.9,
    ];
    urdfFrames.edges.forEach((edge) => {
      const parent = project(mapPoint(edge.parent), width, height, yaw);
      const child = project(mapPoint(edge.child), width, height, yaw);
      const moving = edge.joint.type !== "fixed";
      drawLine(ctx, parent, child, moving ? "#d8f4ff" : "rgba(134, 168, 255, 0.55)", moving ? 3.5 : 1.5);
      drawDot(ctx, child, moving ? 4 : 2.4, moving ? "#5ee493" : "#86a8ff");
    });
    const rootMat = urdfFrames.frames.get(state.urdf.root) || matIdentity();
    const rootPoint = project(mapPoint(transformPoint(rootMat, [0, 0, 0])), width, height, yaw);
    drawDot(ctx, rootPoint, 7, "#61d5ff");
  } else {
    const body = project([0, 0.42, 2.4], width, height, yaw);
    const head = project([0, 1.02, 2.38], width, height, yaw);
    const rear = project([0, 0.36, 2.95], width, height, yaw);
    const front = project([0, 0.42, 1.86], width, height, yaw);
    drawLine(ctx, rear, front, "#d8f4ff", 9);
    drawLine(ctx, body, head, "#86a8ff", 5);
    drawDot(ctx, head, 9, "#61d5ff");
    drawDot(ctx, body, 7, "#5ee493");
  }

  const labelY = Math.max(28, gridY - 260);
  ctx.fillStyle = "rgba(237, 244, 247, 0.92)";
  ctx.font = "12px ui-sans-serif, system-ui";
  ctx.fillText(
    state.urdf ? `${state.urdf.model || state.config.robots[state.robotId].label} URDF tree from ${state.urdf.source}` : "Unitree URDF unavailable",
    16,
    labelY,
  );
  ctx.fillStyle = "rgba(140, 153, 167, 0.95)";
  const sourceLabel =
    state.robotState?.source === "live"
      ? "Live /joint_states applied"
      : state.robotState?.source === "tf"
        ? "Live /tf transforms applied"
        : "Zero pose; waiting for live /joint_states or /tf";
  ctx.fillText(sourceLabel, 16, labelY + 20);
}

setupInteractions();
boot().catch((error) => {
  console.error(error);
  document.body.insertAdjacentHTML(
    "beforeend",
    `<div style="position:fixed;inset:16px;display:grid;place-items:center;background:#090b0fcc;color:#ff6b72;font:14px system-ui">FMS failed to boot: ${escapeHtml(error.message)}</div>`,
  );
});
