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
  missionState: null,
  stableRobotState: {
    joints: {},
    transforms: [],
    defaultJoints: {},
    lastJointTime: 0,
    lastTfTime: 0,
  },
  mesh: {
    renderer: null,
    scene: null,
    camera: null,
    root: null,
    depthGroup: null,
    depthPoints: null,
    depthSurface: null,
    depthFrustum: null,
    depthObjectMarkers: null,
    controls: {
      target: new THREE.Vector3(0, 0.45, 0),
      distance: 3.2,
      minDistance: 0.25,
      maxDistance: 12,
      yaw: 0.78,
      pitch: 0.34,
      pointerId: null,
      mode: "rotate",
      lastX: 0,
      lastY: 0,
      userInteracted: false,
      depthFitDone: false,
    },
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

function postJson(path, payload) {
  return fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  }).then((response) => {
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
  loadUrdf();
  await refreshStatus();
  await refreshRobotState();
  await refreshDepthState();
  await refreshMissionState();
  setupInteractions();
  setInterval(refreshStatus, 2500);
  setInterval(refreshRobotState, 80);
  setInterval(refreshDepthState, 800);
  setInterval(refreshMissionState, 1000);
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
    state.robotState = stabilizeRobotState(await fetchJson("/api/robot-state"));
    const source = state.robotState.source || "unknown";
    const age = state.robotState.lastJointTime
      ? `${Math.max(0, Date.now() / 1000 - state.robotState.lastJointTime).toFixed(1)}s`
      : "";
    const tfAge = state.robotState.lastTfTime
      ? `${Math.max(0, Date.now() / 1000 - state.robotState.lastTfTime).toFixed(1)}s`
      : "";
    const label =
      state.robotState.holdingPose
        ? `holding last pose ${age || tfAge}`
        :
      source === "live"
        ? `live /joint_states ${age}`
        : source === "lowstate"
          ? `live /lowstate ${age}`
        : source === "tf"
          ? `live /tf ${tfAge}`
          : "waiting for /joint_states or /tf";
    setText("#scene-source", label.trim());
  } catch (error) {
    state.robotState = stabilizeRobotState({ source: "offline", error: String(error), joints: {}, transforms: [] });
    const age = state.robotState.lastJointTime
      ? `${Math.max(0, Date.now() / 1000 - state.robotState.lastJointTime).toFixed(1)}s`
      : "";
    setText("#scene-source", state.robotState.holdingPose ? `holding last pose ${age}` : "state offline");
  }
}

function stabilizeRobotState(payload) {
  const cache = state.stableRobotState;
  const now = Date.now() / 1000;
  const incomingJoints = payload?.joints || {};
  const incomingTransforms = Array.isArray(payload?.transforms) ? payload.transforms : [];
  const hasJoints = Object.keys(incomingJoints).length > 0 && Number(payload?.lastJointTime || 0) > 0;
  const hasTransforms = incomingTransforms.length > 0 && Number(payload?.lastTfTime || 0) > 0;

  if (payload?.defaultJoints) cache.defaultJoints = payload.defaultJoints;
  if (hasJoints) {
    cache.joints = { ...cache.joints, ...incomingJoints };
    cache.lastJointTime = Number(payload.lastJointTime || now);
  }
  if (hasTransforms) {
    cache.transforms = incomingTransforms;
    cache.lastTfTime = Number(payload.lastTfTime || now);
  }

  const joints = hasJoints ? { ...cache.joints, ...incomingJoints } : cache.joints;
  const transforms = hasTransforms ? incomingTransforms : cache.transforms;
  const lastJointTime = hasJoints ? Number(payload.lastJointTime || cache.lastJointTime) : cache.lastJointTime;
  const lastTfTime = hasTransforms ? Number(payload.lastTfTime || cache.lastTfTime) : cache.lastTfTime;
  const hasCachedPose = Object.keys(joints || {}).length > 0 || (transforms || []).length > 0;
  const jointAge = lastJointTime ? now - lastJointTime : Infinity;
  const tfAge = lastTfTime ? now - lastTfTime : Infinity;
  return {
    ...payload,
    source: payload?.source || (hasCachedPose ? "holding" : "waiting"),
    defaultJoints: cache.defaultJoints || payload?.defaultJoints || {},
    joints: joints || {},
    transforms: transforms || [],
    lastJointTime,
    lastTfTime,
    holdingPose: hasCachedPose && (!hasJoints || !hasTransforms || Math.min(jointAge, tfAge) > 0.6 || payload?.source === "offline"),
  };
}

async function refreshDepthState() {
  try {
    state.depthState = await fetchJson("/api/depth-state");
    updateDepthGeometry();
    renderDepthLabel();
  } catch (error) {
    if (state.depthState?.lastDepthTime) {
      const age = Math.max(0, Date.now() / 1000 - state.depthState.lastDepthTime).toFixed(1);
      setText("#scene-depth", `holding depth ${age}s`);
      return;
    }
    state.depthState = { source: "offline", points: [], objects: [], config: {} };
    setText("#scene-depth", "depth waiting");
  }
}

function renderDepthLabel() {
  const depth = state.depthState || {};
  const points = depth.points || [];
  renderDepthHud(depth);
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
  const prep = depth.stats?.preprocess?.processedValid ? " filtered" : "";
  const source = depth.dataSource ? `${depth.dataSource} ` : "";
  setText("#scene-depth", `${source}${points.length} pts${prep}${nearest} / ${normalizeFrameId(depth.frameId)} / ${age}s`);
}

function formatMeters(value) {
  const number = Number(value || 0);
  return number > 0 ? `${number.toFixed(2)} m` : "-- m";
}

function renderDepthHud(depth) {
  const preview = $("#depth-preview");
  const stats = depth.stats || {};
  const config = depth.config || {};
  if (preview && depth.previewUrl) {
    const url = depth.previewUrl.includes("?") ? `${depth.previewUrl}&r=${Math.floor(Date.now() / 500)}` : `${depth.previewUrl}?r=${Math.floor(Date.now() / 500)}`;
    if (preview.dataset.src !== url) {
      preview.dataset.src = url;
      preview.src = url;
    }
  }
  setText("#depth-center", formatMeters(stats.centerMeters));
  setText("#depth-nearest", formatMeters(stats.nearestMeters || depth.nearestMeters));
  setText("#depth-coverage", Number.isFinite(Number(stats.coverage)) ? `${Math.round(Number(stats.coverage) * 100)}%` : "--");
  setText("#depth-near", formatMeters(stats.nearMeters || config.nearMeters));
  setText("#depth-far", formatMeters(stats.farMeters || config.farMeters));
  renderDepthAssist(depth);
}

function formatSignedMeters(value) {
  const number = Number(value || 0);
  if (!Number.isFinite(number) || number === 0) return "0.00 m";
  return `${number > 0 ? "+" : ""}${number.toFixed(2)} m`;
}

function updateImageSource(image, rawUrl) {
  if (!image || !rawUrl) return;
  const url = rawUrl.includes("?") ? `${rawUrl}&r=${Math.floor(Date.now() / 500)}` : `${rawUrl}?r=${Math.floor(Date.now() / 500)}`;
  if (image.dataset.src !== url) {
    image.dataset.src = url;
    image.src = url;
  }
}

function renderDepthAssist(depth) {
  const stats = depth.stats || {};
  const target = Number(stats.targetMeters || stats.centerMeters || 0);
  const desired = Number(stats.desiredMeters || 0.45);
  const coverage = Number(stats.targetCoverage ?? stats.coverage ?? 0);
  const guidance = String(stats.guidance || (target ? "DEPTH READY" : "NO DEPTH"));
  setText("#grasp-distance", formatMeters(target));
  setText("#grasp-guidance", guidance);
  setText("#grasp-nearest", formatMeters(stats.targetNearestMeters || stats.nearestMeters));
  setText("#grasp-confidence", Number.isFinite(coverage) ? `${Math.round(coverage * 100)}%` : "--");
  setText("#grasp-delta", target ? formatSignedMeters(target - desired) : "--");

  const panel = $(".depth-view-panel");
  if (panel) {
    panel.dataset.guidance = guidance.toLowerCase().replaceAll(" ", "-");
  }
  const marker = $("#grasp-range-marker");
  if (marker) {
    const maxRange = Math.max(desired * 2.4, 1.2);
    marker.style.left = `${Math.round(clamp(target / maxRange, 0, 1) * 100)}%`;
  }
}

async function refreshMissionState() {
  try {
    state.missionState = await fetchJson("/api/mission-state");
    renderMissionPanel();
  } catch (error) {
    setText("#mission-stage", "OFFLINE");
    setText("#mission-last", "mission API offline");
  }
}

async function sendMissionSignal(signal) {
  setText("#mission-last", `${signal.toUpperCase()} sending`);
  try {
    state.missionState = await postJson("/api/mission-signal", { signal });
    renderMissionPanel();
  } catch (error) {
    setText("#mission-last", `${signal.toUpperCase()} failed`);
    console.error(error);
  }
}

function renderMissionPanel() {
  const mission = state.missionState || { stage: "idle", events: [] };
  const stage = String(mission.stage || "idle").toUpperCase();
  setText("#mission-stage", stage);
  setText("#mission-topic", mission.topic || "/fms/mission_events");
  const last = (mission.events || []).at(-1);
  const label = last
    ? `${last.label || last.signal} #${last.seq} ${new Date(last.time * 1000).toLocaleTimeString("en-GB", { hour12: false })}`
    : "waiting";
  setText("#mission-last", label);
  const panel = $(".mission-panel");
  if (panel) {
    panel.dataset.stage = mission.stage || "idle";
  }
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
    frameAliases: payload.frameAliases || {},
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
      (cam, index) => {
        const isDepthView = cam.id === "aux_4" || String(cam.role || "") === "depth-z16";
        return `
        <article class="camera-card ${isDepthView ? "is-depth-view" : ""}" id="cam-${cam.id}">
          <div class="camera-head">
            <span class="badge">${index + 1}. ${escapeHtml(cam.label)}</span>
            <span class="badge" data-cam-status="${cam.id}">WAIT</span>
          </div>
          ${
            isDepthView
              ? `<img id="depth-view" class="camera-stream depth-view-image" src="/stream/depth-view" alt="${escapeHtml(cam.label)} view" onerror="this.classList.add('hidden'); this.parentElement.querySelector('.camera-empty').style.display='grid';" />`
              : `<img class="camera-stream" src="/stream/${cam.id}" alt="${escapeHtml(cam.label)} stream" onerror="this.classList.add('hidden'); this.parentElement.querySelector('.camera-empty').style.display='grid';" />`
          }
          <div class="camera-empty" style="display:none">
            <div>
              <b>${escapeHtml(cam.role)}</b>
              <p>${escapeHtml(cam.device)}</p>
            </div>
          </div>
          ${
            isDepthView
              ? `
                <div class="depth-view-panel">
                  <div class="grasp-assist-head">
                    <span>Depth Assist</span>
                    <strong id="grasp-guidance">NO DEPTH</strong>
                  </div>
                  <div class="depth-distance-line">
                    <span>target</span>
                    <b id="grasp-distance">-- m</b>
                  </div>
                  <div class="grasp-range">
                    <i></i>
                    <em id="grasp-range-marker"></em>
                  </div>
                  <div class="grasp-metrics">
                    <span>nearest <b id="grasp-nearest">--</b></span>
                    <span>valid <b id="grasp-confidence">--</b></span>
                    <span>delta <b id="grasp-delta">--</b></span>
                  </div>
                </div>
              `
              : ""
          }
          <div class="camera-foot">
            <span class="badge">${escapeHtml(cam.device)}</span>
            <span class="badge">${cam.fov} deg</span>
          </div>
        </article>
      `;
      },
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

  document.querySelectorAll("[data-mission]").forEach((button) => {
    button.addEventListener("click", () => sendMissionSignal(button.dataset.mission));
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
  const normalized = String(frameId || "").replace(/^\/+/, "");
  return state.urdf?.frameAliases?.[normalized] || normalized;
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
  const surfaceGeometry = new THREE.BufferGeometry();
  const surfaceMaterial = new THREE.MeshStandardMaterial({
    vertexColors: true,
    transparent: true,
    opacity: 0.9,
    roughness: 0.82,
    metalness: 0.0,
    side: THREE.DoubleSide,
    depthTest: true,
    depthWrite: false,
  });
  const depthSurface = new THREE.Mesh(surfaceGeometry, surfaceMaterial);
  depthSurface.frustumCulled = false;
  depthSurface.renderOrder = 2;
  depthGroup.add(depthSurface);
  const pointGeometry = new THREE.BufferGeometry();
  const pointMaterial = new THREE.PointsMaterial({
    size: 0.014,
    map: makeDepthPointTexture(),
    vertexColors: true,
    transparent: true,
    opacity: 0.96,
    alphaTest: 0.05,
    depthTest: true,
    depthWrite: false,
  });
  const depthPoints = new THREE.Points(pointGeometry, pointMaterial);
  depthPoints.frustumCulled = false;
  depthPoints.renderOrder = 3;
  depthGroup.add(depthPoints);
  const frustumGeometry = new THREE.BufferGeometry();
  const frustumMaterial = new THREE.LineBasicMaterial({ color: 0x5ee493, transparent: true, opacity: 0.82, depthTest: false });
  const depthFrustum = new THREE.LineSegments(frustumGeometry, frustumMaterial);
  depthFrustum.frustumCulled = false;
  depthGroup.add(depthFrustum);
  const depthObjectMarkers = new THREE.Group();
  depthObjectMarkers.frustumCulled = false;
  depthGroup.add(depthObjectMarkers);
  root.add(depthGroup);

  state.mesh.renderer = renderer;
  state.mesh.scene = scene;
  state.mesh.camera = camera;
  state.mesh.root = root;
  state.mesh.depthGroup = depthGroup;
  state.mesh.depthPoints = depthPoints;
  state.mesh.depthSurface = depthSurface;
  state.mesh.depthFrustum = depthFrustum;
  state.mesh.depthObjectMarkers = depthObjectMarkers;
  updateOrbitCamera();
  bindSceneControls(canvas);
  resizeRenderer();
  window.addEventListener("resize", resizeRenderer);
}

function makeDepthPointTexture() {
  const canvas = document.createElement("canvas");
  canvas.width = 64;
  canvas.height = 64;
  const ctx = canvas.getContext("2d");
  const gradient = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
  gradient.addColorStop(0, "rgba(255,255,255,1)");
  gradient.addColorStop(0.55, "rgba(255,255,255,0.9)");
  gradient.addColorStop(1, "rgba(255,255,255,0)");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, 64, 64);
  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  return texture;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function updateOrbitCamera() {
  const { camera, controls } = state.mesh;
  if (!camera || !controls) return;
  const pitch = clamp(controls.pitch, -1.22, 1.22);
  controls.pitch = pitch;
  controls.distance = clamp(controls.distance, controls.minDistance, controls.maxDistance);
  const cosPitch = Math.cos(pitch);
  camera.position.set(
    controls.target.x + controls.distance * Math.sin(controls.yaw) * cosPitch,
    controls.target.y + controls.distance * Math.sin(pitch),
    controls.target.z + controls.distance * Math.cos(controls.yaw) * cosPitch,
  );
  camera.lookAt(controls.target);
}

function panOrbitTarget(dx, dy) {
  const { camera, controls } = state.mesh;
  if (!camera || !controls) return;
  const canvas = camera.parent?.domElement || state.mesh.renderer?.domElement;
  const height = Math.max(1, canvas?.clientHeight || 1);
  const scale = controls.distance / height;
  const forward = new THREE.Vector3();
  camera.getWorldDirection(forward);
  const right = new THREE.Vector3().crossVectors(forward, camera.up).normalize();
  const up = new THREE.Vector3().crossVectors(right, forward).normalize();
  controls.target.addScaledVector(right, dx * scale);
  controls.target.addScaledVector(up, dy * scale);
  updateOrbitCamera();
}

function zoomOrbit(deltaY) {
  const controls = state.mesh.controls;
  if (!controls) return;
  controls.distance = clamp(
    controls.distance * Math.exp(deltaY * 0.0012),
    controls.minDistance,
    controls.maxDistance,
  );
  updateOrbitCamera();
}

function bindSceneControls(canvas) {
  canvas.addEventListener("contextmenu", (event) => event.preventDefault());
  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      state.mesh.controls.userInteracted = true;
      zoomOrbit(event.deltaY);
    },
    { passive: false },
  );
  canvas.addEventListener("pointerdown", (event) => {
    const controls = state.mesh.controls;
    controls.userInteracted = true;
    controls.pointerId = event.pointerId;
    controls.mode = event.button === 2 || event.shiftKey ? "pan" : "rotate";
    controls.lastX = event.clientX;
    controls.lastY = event.clientY;
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    const controls = state.mesh.controls;
    if (controls.pointerId !== event.pointerId) return;
    const dx = event.clientX - controls.lastX;
    const dy = event.clientY - controls.lastY;
    controls.lastX = event.clientX;
    controls.lastY = event.clientY;
    if (controls.mode === "pan") {
      panOrbitTarget(-dx, dy);
      return;
    }
    controls.yaw -= dx * 0.006;
    controls.pitch = clamp(controls.pitch - dy * 0.006, -1.22, 1.22);
    updateOrbitCamera();
  });
  ["pointerup", "pointercancel", "pointerleave"].forEach((eventName) => {
    canvas.addEventListener(eventName, (event) => {
      const controls = state.mesh.controls;
      if (controls.pointerId !== event.pointerId) return;
      controls.pointerId = null;
      try {
        canvas.releasePointerCapture(event.pointerId);
      } catch (error) {
        // Pointer capture can already be released by the browser.
      }
    });
  });
  canvas.addEventListener("dblclick", () => {
    state.mesh.controls.userInteracted = true;
    fitCameraToRobot();
  });
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

function disposeRuntimeObject(object) {
  object.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    const materials = Array.isArray(child.material) ? child.material : [child.material];
    materials.filter(Boolean).forEach((material) => material.dispose?.());
  });
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

function objectColor(id, objects) {
  const object = objects.find((item) => Number(item.id) === Number(id));
  if (!object?.color) return null;
  return object.color.map((channel) => clamp(Number(channel) / 255, 0, 1));
}

function colorForDepthPoint(point, near, far, objects) {
  const objectId = Number(point?.[3] ?? -1);
  const clusterColor = objectId >= 0 ? objectColor(objectId, objects) : null;
  if (clusterColor) return clusterColor;
  const px = Number(point?.[0] || 0);
  const py = Number(point?.[1] || 0);
  const pz = Number(point?.[2] || 0);
  return depthColor(Math.hypot(px, py, pz), near, far);
}

function bboxLinePositions(min, max) {
  const [x0, y0, z0] = min;
  const [x1, y1, z1] = max;
  return [
    x0, y0, z0, x1, y0, z0,
    x1, y0, z0, x1, y1, z0,
    x1, y1, z0, x0, y1, z0,
    x0, y1, z0, x0, y0, z0,
    x0, y0, z1, x1, y0, z1,
    x1, y0, z1, x1, y1, z1,
    x1, y1, z1, x0, y1, z1,
    x0, y1, z1, x0, y0, z1,
    x0, y0, z0, x0, y0, z1,
    x1, y0, z0, x1, y0, z1,
    x1, y1, z0, x1, y1, z1,
    x0, y1, z0, x0, y1, z1,
  ];
}

function updateDepthGeometry() {
  const { depthPoints, depthSurface, depthFrustum, depthObjectMarkers } = state.mesh;
  if (!depthPoints || !depthSurface || !depthFrustum || !depthObjectMarkers) return;
  const depth = state.depthState || {};
  const config = depth.config || state.config?.robots?.[state.robotId]?.depthSensor || {};
  const objects = Array.isArray(depth.objects) ? depth.objects : [];
  const near = Number(config.nearMeters || 0.18);
  const far = Number(config.focusFarMeters || config.farMeters || 1.05);
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

  const surface = depth.surface || {};
  const surfacePoints = Array.isArray(surface.points) ? surface.points : [];
  const surfaceWidth = Number(surface.width || 0);
  const surfaceHeight = Number(surface.height || 0);
  const maxDepthDelta = Number(surface.maxDepthDelta || config.surfaceMaxDepthDelta || 0.18);
  const vertexIndex = new Int32Array(surfacePoints.length).fill(-1);
  const surfacePositions = [];
  const surfaceColors = [];
  surfacePoints.forEach((point, index) => {
    if (!Array.isArray(point)) return;
    const px = Number(point[0]);
    const py = Number(point[1]);
    const pz = Number(point[2]);
    if (![px, py, pz].every(Number.isFinite)) return;
    vertexIndex[index] = surfacePositions.length / 3;
    surfacePositions.push(px, py, pz);
    const color = colorForDepthPoint(point, near, far, objects);
    surfaceColors.push(color[0], color[1], color[2]);
  });
  const indices = [];
  const addTriangle = (a, b, c) => {
    const ia = vertexIndex[a];
    const ib = vertexIndex[b];
    const ic = vertexIndex[c];
    if (ia < 0 || ib < 0 || ic < 0) return;
    const za = surfacePoints[a][2];
    const zb = surfacePoints[b][2];
    const zc = surfacePoints[c][2];
    if (Math.max(za, zb, zc) - Math.min(za, zb, zc) > maxDepthDelta) return;
    indices.push(ia, ib, ic);
  };
  if (surfaceWidth > 1 && surfaceHeight > 1) {
    for (let row = 0; row < surfaceHeight - 1; row += 1) {
      for (let col = 0; col < surfaceWidth - 1; col += 1) {
        const a = row * surfaceWidth + col;
        const b = a + 1;
        const c = a + surfaceWidth;
        const d = c + 1;
        addTriangle(a, c, b);
        addTriangle(b, c, d);
      }
    }
  }
  depthSurface.geometry.setIndex(indices);
  depthSurface.geometry.setAttribute("position", new THREE.Float32BufferAttribute(surfacePositions, 3));
  depthSurface.geometry.setAttribute("color", new THREE.Float32BufferAttribute(surfaceColors, 3));
  depthSurface.geometry.computeVertexNormals();
  depthSurface.geometry.computeBoundingSphere();
  if (
    surfacePositions.length &&
    state.urdf &&
    state.mesh.links.size &&
    !state.mesh.controls.depthFitDone &&
    !state.mesh.controls.userInteracted
  ) {
    state.mesh.controls.depthFitDone = true;
    requestAnimationFrame(() => {
      updateDepthPose();
      if (state.mesh.depthGroup?.visible) {
        state.mesh.root?.updateMatrixWorld(true);
        fitCameraToRobot();
      } else {
        state.mesh.controls.depthFitDone = false;
      }
    });
  }

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
    const color = colorForDepthPoint(point, near, far, objects);
    colors[index * 3] = color[0];
    colors[index * 3 + 1] = color[1];
    colors[index * 3 + 2] = color[2];
  });
  depthPoints.geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  depthPoints.geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  depthPoints.geometry.computeBoundingSphere();

  [...depthObjectMarkers.children].forEach((child) => {
    depthObjectMarkers.remove(child);
    disposeRuntimeObject(child);
  });
  objects.forEach((object) => {
    const color = objectColor(object.id, objects) || [1, 1, 1];
    const lineMaterial = new THREE.LineBasicMaterial({
      color: new THREE.Color(color[0], color[1], color[2]),
      transparent: true,
      opacity: 0.95,
      depthTest: false,
    });
    if (Array.isArray(object.bboxMin) && Array.isArray(object.bboxMax)) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.Float32BufferAttribute(bboxLinePositions(object.bboxMin, object.bboxMax), 3));
      const box = new THREE.LineSegments(geometry, lineMaterial);
      box.renderOrder = 8;
      depthObjectMarkers.add(box);
    }
  });

  const primary = objects[0];
  if (primary) {
    const size = Array.isArray(primary.size) ? primary.size.map((value) => Number(value).toFixed(2)).join("x") : "";
    setText("#scene-objects", `${objects.length} objects / ${primary.label} ${Number(primary.distanceMeters || 0).toFixed(2)}m ${size}m`);
  } else {
    setText("#scene-objects", "0 objects");
  }
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
  const { root, camera, scene, depthGroup, depthSurface, depthFrustum, links } = state.mesh;
  if (!root || !camera || !scene) return;
  root.updateMatrixWorld(true);
  const box = new THREE.Box3();
  let hasBounds = false;
  links.forEach((group) => {
    group.updateMatrixWorld(true);
    const linkBox = new THREE.Box3().setFromObject(group);
    if (!Number.isFinite(linkBox.min.x) || linkBox.isEmpty()) return;
    box.union(linkBox);
    hasBounds = true;
  });
  if (depthGroup?.visible) {
    if (depthSurface?.geometry?.attributes?.position?.count) {
      depthSurface.updateMatrixWorld(true);
      const depthBox = new THREE.Box3().setFromObject(depthSurface);
      if (Number.isFinite(depthBox.min.x) && !depthBox.isEmpty()) {
        box.union(depthBox);
        hasBounds = true;
      }
    }
    if (depthFrustum?.geometry?.attributes?.position?.count) {
      depthFrustum.updateMatrixWorld(true);
      const frustumBox = new THREE.Box3().setFromObject(depthFrustum);
      if (Number.isFinite(frustumBox.min.x) && !frustumBox.isEmpty()) {
        box.union(frustumBox);
        hasBounds = true;
      }
    }
  }
  if (!hasBounds || !Number.isFinite(box.min.x) || box.isEmpty()) return;

  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  const maxDim = Math.max(size.x, size.y, size.z, 0.45);
  const distance = Math.max(1.2, maxDim * 1.55);
  const target = new THREE.Vector3(center.x, center.y + size.y * 0.08, center.z);

  const controls = state.mesh.controls;
  controls.target.copy(target);
  controls.distance = distance * 1.58;
  controls.minDistance = Math.max(0.08, maxDim * 0.22);
  controls.maxDistance = Math.max(10, distance * 6);
  controls.yaw = 0.74;
  controls.pitch = Math.atan2(Math.max(0.45, maxDim * 0.55), distance * 1.1);
  updateOrbitCamera();
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
