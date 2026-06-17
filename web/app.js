import * as THREE from "three";
import { ColladaLoader } from "/lib/ColladaLoader.js";
import { STLLoader } from "/lib/STLLoader.js";

const state = {
  config: null,
  status: null,
  robotId: "unitree",
  armed: false,
  drive: { linear: 0, angular: 0 },
  lastStatusAt: 0,
  urdf: null,
  robotState: null,
  depthState: null,
  mesh: {
    renderer: null,
    scene: null,
    camera: null,
    root: null,
    depthGroup: null,
    depthPoints: null,
    depthFrustum: null,
    links: new Map(),
    stlLoader: new STLLoader(),
    colladaLoader: new ColladaLoader(),
    assetCache: new Map(),
    loadToken: 0,
  },
};

const $ = (selector) => document.querySelector(selector);

function fmtTime(date = new Date()) {
  return date.toLocaleTimeString("en-GB", { hour12: false });
}

function setText(selector, value) {
  const node = $(selector);
  if (node) node.textContent = value;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function fetchJson(path) {
  return fetch(path, { cache: "no-store" }).then((response) => {
    if (!response.ok) throw new Error(`${path} ${response.status}`);
    return response.json();
  });
}

function statusClass(cam) {
  if (!cam.exists) return "bad";
  if (cam.busy) return "warn";
  return "good";
}

async function boot() {
  state.config = await fetchJson("/api/config");
  initMeshViewer();
  renderRobotSelect();
  renderTopicMap();
  renderCameraGrid();
  await loadUrdf();
  await refreshStatus();
  await refreshRobotState();
  await refreshDepthState();
  setupInteractions();
  setInterval(refreshStatus, 2500);
  setInterval(refreshRobotState, 80);
  setInterval(refreshDepthState, 180);
  setInterval(updateClock, 500);
  requestAnimationFrame(renderScene);
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
    setText("#scene-root", `${state.urdf.model || state.config.robots[state.robotId].label} / ${state.urdf.root}`);
    setText("#scene-joints", `${state.urdf.movingJoints.length}/${state.urdf.joints.length} moving joints`);
    setText("#scene-links", `${state.urdf.links.size} links`);
    setText("#scene-meshes", "loading meshes");
    await loadRobotMeshes();
  } catch (error) {
    state.urdf = null;
    clearRobotMeshes();
    setText("#scene-root", "URDF missing");
    setText("#scene-source", "run sync script");
    setText("#scene-joints", "0 joints");
    setText("#scene-links", "0 links");
    setText("#scene-meshes", "0 meshes");
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
        : source === "lowstate"
          ? `live /lowstate ${age}`
        : source === "tf"
          ? `live /tf ${tfAge}`
          : "waiting for /joint_states or /tf";
    setText("#scene-source", label.trim());
  } catch (error) {
    state.robotState = { source: "offline", joints: {}, transforms: [] };
    setText("#scene-source", "state offline");
  }
}

async function refreshDepthState() {
  try {
    state.depthState = await fetchJson("/api/depth-state");
    updateDepthGeometry();
    renderDepthLabel();
  } catch (error) {
    state.depthState = { source: "offline", points: [], config: {} };
    setText("#scene-depth", "depth offline");
  }
}

function renderDepthLabel() {
  const depth = state.depthState || {};
  const points = depth.points || [];
  if (depth.error) {
    setText("#scene-depth", depth.error.slice(0, 58));
    return;
  }
  if (!depth.lastDepthTime || !points.length) {
    const topic = depth.config?.pointCloudTopic || state.config?.robots?.[state.robotId]?.topics?.pointCloud || "depth";
    setText("#scene-depth", `${topic} waiting`);
    return;
  }
  const age = Math.max(0, Date.now() / 1000 - depth.lastDepthTime).toFixed(1);
  const nearest = depth.nearestMeters ? ` near ${Number(depth.nearestMeters).toFixed(2)}m` : "";
  setText("#scene-depth", `${points.length} pts${nearest} / ${normalizeFrameId(depth.frameId)} / ${age}s`);
}

function parseVector(value, fallback = [0, 0, 0]) {
  if (!value) return fallback;
  const parts = value.trim().split(/\s+/).map(Number);
  return parts.length >= 3 && parts.every(Number.isFinite) ? parts.slice(0, 3) : fallback;
}

function directChildren(element, tagName) {
  return [...element.children].filter((child) => child.tagName === tagName);
}

function parseVisuals(linkElement) {
  return directChildren(linkElement, "visual")
    .map((visual, index) => {
      const origin = visual.querySelector("origin");
      const mesh = visual.querySelector("geometry mesh");
      if (!mesh) return null;
      return {
        index,
        xyz: parseVector(origin?.getAttribute("xyz"), [0, 0, 0]),
        rpy: parseVector(origin?.getAttribute("rpy"), [0, 0, 0]),
        filename: mesh.getAttribute("filename") || "",
        scale: parseVector(mesh.getAttribute("scale"), [1, 1, 1]),
      };
    })
    .filter((visual) => visual?.filename);
}

function parseUrdf(payload) {
  const doc = new DOMParser().parseFromString(payload.xml, "application/xml");
  const linkVisuals = new Map();
  const links = new Set();
  [...doc.querySelectorAll("link")].forEach((link) => {
    const name = link.getAttribute("name");
    if (!name) return;
    links.add(name);
    linkVisuals.set(name, parseVisuals(link));
  });
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
  const meshCount = [...linkVisuals.values()].reduce((total, visuals) => total + visuals.length, 0);
  return {
    path: payload.path,
    source: payload.source,
    model: payload.model,
    root,
    links,
    joints,
    movingJoints,
    children,
    linkVisuals,
    meshCount,
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
      .slice(0, 38)
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

  document.querySelectorAll("[data-toggle-panel]").forEach((button) => {
    const panel = button.dataset.togglePanel;
    button.classList.toggle("active", !document.body.classList.contains(`${panel}-collapsed`));
    button.addEventListener("click", () => {
      const collapsedClass = `${panel}-collapsed`;
      document.body.classList.toggle(collapsedClass);
      button.classList.toggle("active", !document.body.classList.contains(collapsedClass));
      resizeRenderer();
    });
  });

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

function matOpticalToRobotForward() {
  return [
    0, 0, 1, 0,
    -1, 0, 0, 0,
    0, -1, 0, 0,
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
  const defaultJoints = state.robotState?.defaultJoints || {};
  const transforms = tfByChild();
  const visit = (link) => {
    const parentMat = frames.get(link);
    const childJoints = state.urdf.children.get(link) || [];
    childJoints.forEach((joint) => {
      let angle = Number(jointsByName[joint.name] ?? defaultJoints[joint.name] ?? 0);
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
  return { frames, edges };
}

function buildSceneFrames() {
  const urdfFrames = buildUrdfFrames();
  if (!urdfFrames) return new Map();
  const frames = new Map(urdfFrames.frames);
  const transforms = tfByChild();
  let changed = true;
  let guard = 0;
  while (changed && guard < 80) {
    changed = false;
    guard += 1;
    transforms.forEach((tf, child) => {
      if (frames.has(child) || !frames.has(tf.parent)) return;
      frames.set(child, matMul(frames.get(tf.parent), tf.local));
      changed = true;
    });
  }
  return frames;
}

function depthFrameMatrix() {
  if (!state.urdf) return null;
  const depth = state.depthState || {};
  const config = depth.config || state.config?.robots?.[state.robotId]?.depthSensor || {};
  const frames = buildSceneFrames();
  const frame = normalizeFrameId(depth.frameId);
  if (frame && frames.has(frame)) return frames.get(frame);

  const preferred = config.preferredFrames || [];
  for (const candidate of preferred) {
    const normalized = normalizeFrameId(candidate);
    if (frames.has(normalized)) return frames.get(normalized);
  }

  const parent = normalizeFrameId(config.fallbackParentFrame || state.urdf.root);
  const parentMatrix = frames.get(parent) || matIdentity();
  const origin = Array.isArray(config.fallbackOrigin) ? config.fallbackOrigin : [0.28, 0, 0.18];
  const rpy = Array.isArray(config.fallbackRpy) ? config.fallbackRpy : [0, 0, 0];
  return matMul(parentMatrix, matMul(matMul(matTranslate(origin), matRpy(rpy)), matOpticalToRobotForward()));
}

function initMeshViewer() {
  const canvas = $("#scene-canvas");
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setClearColor(0x081018, 1);
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x081018, 4, 9);

  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 50);
  camera.position.set(2.8, 1.45, 3.4);
  camera.lookAt(0, 0.65, 0);

  const hemi = new THREE.HemisphereLight(0xd8f4ff, 0x0b1218, 2.2);
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0xffffff, 2.6);
  key.position.set(3, 4, 2);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x61d5ff, 1.2);
  rim.position.set(-2, 2.2, -2);
  scene.add(rim);

  const grid = new THREE.GridHelper(4.8, 24, 0x22445a, 0x132837);
  grid.position.y = -0.005;
  scene.add(grid);

  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2;
  scene.add(root);

  const depthGroup = new THREE.Group();
  depthGroup.matrixAutoUpdate = false;
  const pointGeometry = new THREE.BufferGeometry();
  const pointMaterial = new THREE.PointsMaterial({
    size: 0.018,
    vertexColors: true,
    transparent: true,
    opacity: 0.92,
    depthWrite: false,
  });
  const depthPoints = new THREE.Points(pointGeometry, pointMaterial);
  depthPoints.frustumCulled = false;
  depthGroup.add(depthPoints);
  const frustumGeometry = new THREE.BufferGeometry();
  const frustumMaterial = new THREE.LineBasicMaterial({ color: 0x5ee493, transparent: true, opacity: 0.72 });
  const depthFrustum = new THREE.LineSegments(frustumGeometry, frustumMaterial);
  depthFrustum.frustumCulled = false;
  depthGroup.add(depthFrustum);
  root.add(depthGroup);

  state.mesh.renderer = renderer;
  state.mesh.scene = scene;
  state.mesh.camera = camera;
  state.mesh.root = root;
  state.mesh.depthGroup = depthGroup;
  state.mesh.depthPoints = depthPoints;
  state.mesh.depthFrustum = depthFrustum;
  resizeRenderer();
  window.addEventListener("resize", resizeRenderer);
}

function matrixFromUrdf(values) {
  const matrix = new THREE.Matrix4();
  matrix.set(
    values[0], values[1], values[2], values[3],
    values[4], values[5], values[6], values[7],
    values[8], values[9], values[10], values[11],
    values[12], values[13], values[14], values[15],
  );
  return matrix;
}

function resolveMeshUrl(filename) {
  if (!state.urdf) return "";
  if (filename.startsWith("package://")) {
    const parts = filename.replace("package://", "").split("/");
    const packageName = parts.shift();
    const urdfParts = state.urdf.path.split("/");
    const packageIndex = urdfParts.lastIndexOf(packageName);
    if (packageIndex >= 0) {
      return `/${urdfParts.slice(0, packageIndex + 1).concat(parts).join("/")}`;
    }
    return `/${state.urdf.path.split("/").slice(0, -1).concat(parts).join("/")}`;
  }
  if (filename.startsWith("file://")) return filename.replace("file://", "");
  if (filename.startsWith("/")) return filename;
  const base = state.urdf.path.split("/").slice(0, -1).join("/");
  return `/${base}/${filename}`;
}

function clearRobotMeshes() {
  if (!state.mesh.root) return;
  state.mesh.links.forEach((group) => state.mesh.root.remove(group));
  state.mesh.links.clear();
}

function materialForLink(linkName) {
  const color =
    linkName.includes("left") || linkName.includes("_L") ? 0x9fc8ff :
    linkName.includes("right") || linkName.includes("_R") ? 0xd7e3ee :
    linkName.includes("head") ? 0xbcecff :
    0xcfd6df;
  return new THREE.MeshStandardMaterial({
    color,
    roughness: 0.48,
    metalness: 0.18,
    side: THREE.DoubleSide,
  });
}

function cloneWithRuntimeMaterial(object, linkName) {
  const clone = object.clone(true);
  clone.traverse((child) => {
    if (child.isMesh) {
      child.material = child.material || materialForLink(linkName);
      child.castShadow = false;
      child.receiveShadow = false;
    }
  });
  return clone;
}

async function objectForMeshUrl(url, linkName) {
  const lower = url.toLowerCase();
  if (!state.mesh.assetCache.has(url)) {
    if (lower.endsWith(".dae")) {
      state.mesh.assetCache.set(
        url,
        state.mesh.colladaLoader.loadAsync(url).then((collada) => {
          const scene = collada.scene;
          if (
            Math.abs(scene.rotation.x + Math.PI / 2) < 0.000001 &&
            Math.abs(scene.rotation.y) < 0.000001 &&
            Math.abs(scene.rotation.z) < 0.000001
          ) {
            scene.rotation.set(0, 0, 0);
            scene.updateMatrix();
          }
          return scene;
        }),
      );
    } else if (lower.endsWith(".stl")) {
      state.mesh.assetCache.set(
        url,
        state.mesh.stlLoader.loadAsync(url).then((geometry) => {
          geometry.computeVertexNormals();
          return geometry;
        }),
      );
    } else {
      state.mesh.assetCache.set(url, Promise.reject(new Error(`unsupported mesh format: ${url}`)));
    }
  }
  const asset = await state.mesh.assetCache.get(url);
  if (asset.isBufferGeometry) {
    const mesh = new THREE.Mesh(asset, materialForLink(linkName));
    mesh.castShadow = false;
    mesh.receiveShadow = false;
    return mesh;
  }
  return cloneWithRuntimeMaterial(asset, linkName);
}

function depthColor(depth, near, far) {
  const t = Math.max(0, Math.min(1, (depth - near) / Math.max(0.001, far - near)));
  if (t < 0.35) return [1.0, 0.36 + t, 0.2];
  if (t < 0.72) return [0.25, 0.92, 0.72 + (t - 0.35) * 0.55];
  return [0.35, 0.7 - (t - 0.72) * 0.55, 1.0];
}

function updateDepthGeometry() {
  const { depthPoints, depthFrustum } = state.mesh;
  if (!depthPoints || !depthFrustum) return;
  const depth = state.depthState || {};
  const config = depth.config || state.config?.robots?.[state.robotId]?.depthSensor || {};
  const near = Number(config.nearMeters || 0.18);
  const far = Number(config.farMeters || 3.0);
  const hFov = THREE.MathUtils.degToRad(Number(config.horizontalFovDeg || 87));
  const vFov = THREE.MathUtils.degToRad(Number(config.verticalFovDeg || 58));
  const x = Math.tan(hFov / 2) * far;
  const y = Math.tan(vFov / 2) * far;
  const corners = [
    [-x, -y, far],
    [x, -y, far],
    [x, y, far],
    [-x, y, far],
  ];
  const lines = [
    [0, 0, 0], corners[0],
    [0, 0, 0], corners[1],
    [0, 0, 0], corners[2],
    [0, 0, 0], corners[3],
    corners[0], corners[1],
    corners[1], corners[2],
    corners[2], corners[3],
    corners[3], corners[0],
  ].flat();
  depthFrustum.geometry.setAttribute("position", new THREE.Float32BufferAttribute(lines, 3));
  depthFrustum.geometry.computeBoundingSphere();

  const points = Array.isArray(depth.points) ? depth.points : [];
  const positions = new Float32Array(points.length * 3);
  const colors = new Float32Array(points.length * 3);
  points.forEach((point, index) => {
    const px = Number(point[0] || 0);
    const py = Number(point[1] || 0);
    const pz = Number(point[2] || 0);
    positions[index * 3] = px;
    positions[index * 3 + 1] = py;
    positions[index * 3 + 2] = pz;
    const color = depthColor(Math.hypot(px, py, pz), near, far);
    colors[index * 3] = color[0];
    colors[index * 3 + 1] = color[1];
    colors[index * 3 + 2] = color[2];
  });
  depthPoints.geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  depthPoints.geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  depthPoints.geometry.computeBoundingSphere();
}

function updateDepthPose() {
  const { depthGroup } = state.mesh;
  if (!depthGroup) return;
  const matrix = depthFrameMatrix();
  if (!matrix) {
    depthGroup.visible = false;
    return;
  }
  depthGroup.visible = true;
  depthGroup.matrix.copy(matrixFromUrdf(matrix));
  depthGroup.matrixWorldNeedsUpdate = true;
}

async function loadRobotMeshes() {
  const token = ++state.mesh.loadToken;
  clearRobotMeshes();
  if (!state.urdf || !state.mesh.root) return;

  let loaded = 0;
  const tasks = [];
  state.urdf.links.forEach((linkName) => {
    const linkGroup = new THREE.Group();
    linkGroup.matrixAutoUpdate = false;
    state.mesh.root.add(linkGroup);
    state.mesh.links.set(linkName, linkGroup);
    const visuals = state.urdf.linkVisuals.get(linkName) || [];
    visuals.forEach((visual) => {
      const url = resolveMeshUrl(visual.filename);
      tasks.push(
        objectForMeshUrl(url, linkName)
          .then((object) => {
            if (token !== state.mesh.loadToken) return;
            const visualGroup = new THREE.Group();
            visualGroup.matrixAutoUpdate = false;
            visualGroup.matrix.copy(matrixFromUrdf(matMul(matTranslate(visual.xyz), matRpy(visual.rpy))));
            visualGroup.matrixWorldNeedsUpdate = true;
            object.scale.multiply(new THREE.Vector3(visual.scale[0], visual.scale[1], visual.scale[2]));
            visualGroup.add(object);
            linkGroup.add(visualGroup);
            loaded += 1;
            setText("#scene-meshes", `${loaded}/${state.urdf.meshCount} meshes`);
          })
          .catch((error) => {
            console.warn(`mesh failed: ${url}`, error);
          }),
      );
    });
  });
  await Promise.allSettled(tasks);
  if (token === state.mesh.loadToken) {
    updateRobotMeshTransforms();
    fitCameraToRobot();
    setText("#scene-meshes", `${loaded}/${state.urdf.meshCount} meshes`);
  }
}

function updateRobotMeshTransforms() {
  const frames = buildUrdfFrames();
  if (!frames) return;
  state.mesh.links.forEach((group, linkName) => {
    const matrix = frames.frames.get(linkName);
    if (!matrix) return;
    group.matrix.copy(matrixFromUrdf(matrix));
    group.matrixWorldNeedsUpdate = true;
  });
  updateDepthPose();
  groundRobotOnGrid();
}

function groundRobotOnGrid() {
  const { root } = state.mesh;
  if (!root) return;
  root.position.y = 0;
  root.updateMatrixWorld(true);
  const box = new THREE.Box3();
  let hasRobotMesh = false;
  state.mesh.links.forEach((group) => {
    const linkBox = new THREE.Box3().setFromObject(group);
    if (!Number.isFinite(linkBox.min.y) || linkBox.isEmpty()) return;
    box.union(linkBox);
    hasRobotMesh = true;
  });
  if (!hasRobotMesh || box.isEmpty()) return;
  root.position.y = Math.max(0, -box.min.y + 0.015);
  root.updateMatrixWorld(true);
}

function fitCameraToRobot() {
  const { root, camera, scene } = state.mesh;
  if (!root || !camera || !scene) return;
  root.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(root);
  if (!Number.isFinite(box.min.x) || box.isEmpty()) return;

  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  const maxDim = Math.max(size.x, size.y, size.z, 0.45);
  const distance = Math.max(1.2, maxDim * 1.55);

  camera.position.set(
    center.x + distance * 0.95,
    center.y + Math.max(0.45, maxDim * 0.55),
    center.z + distance * 1.05,
  );
  camera.lookAt(center.x, center.y + size.y * 0.08, center.z);
  camera.near = Math.max(0.01, distance / 100);
  camera.far = Math.max(20, distance * 8);
  camera.updateProjectionMatrix();
  scene.fog = new THREE.Fog(0x081018, distance * 1.4, distance * 4.5);
}

function resizeRenderer() {
  const { renderer, camera } = state.mesh;
  if (!renderer || !camera) return;
  const canvas = renderer.domElement;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(rect.width));
  const height = Math.max(1, Math.floor(rect.height));
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function renderScene() {
  const { renderer, scene, camera } = state.mesh;
  if (renderer && scene && camera) {
    resizeRenderer();
    updateRobotMeshTransforms();
    renderer.render(scene, camera);
  }
  requestAnimationFrame(renderScene);
}

boot().catch((error) => {
  console.error(error);
  document.body.insertAdjacentHTML(
    "beforeend",
    `<div style="position:fixed;inset:16px;display:grid;place-items:center;background:#090b0fcc;color:#ff6b72;font:14px system-ui">FMS failed to boot: ${escapeHtml(error.message)}</div>`,
  );
});
