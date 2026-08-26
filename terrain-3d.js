import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const container = document.getElementById('terrain3D');
if (!container) {
  throw new Error('terrain3D container not found');
}

const loading = document.getElementById('terrain3DLoading');
const info = document.getElementById('terrain3DInfo');
const scaleNote = document.getElementById('terrain3DScaleNote');
const resetViewButton = document.getElementById('reset3DView');
const layerButtons = [...document.querySelectorAll('[data-terrain-layer]')];
const zScaleButtons = [...document.querySelectorAll('[data-z-scale]')];
const slopeRange = document.getElementById('slopeLimit');
const stepRange = document.getElementById('stepLimit');

const RESOLUTION_M = 0.05;
const COLS = 76;
const ROWS = 46;
const SUPPORT_RADIUS_CELLS = 4;
const EPS = 1e-9;

let activeLayer = document.querySelector('[data-terrain-layer].active')?.dataset.terrainLayer || 'traversability';
let maxSlopeDeg = Number(slopeRange?.value || 15);
let maxStepM = Number(stepRange?.value || 8) / 100;
let zScale = 3;
let selectedCell = null;

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function smoothstep(edge0, edge1, value) {
  const t = clamp((value - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

function gaussian(x, y, cx, cy, sx, sy) {
  const dx = (x - cx) / sx;
  const dy = (y - cy) / sy;
  return Math.exp(-0.5 * (dx * dx + dy * dy));
}

function cellIndex(x, y) {
  return y * COLS + x;
}

function inBounds(x, y) {
  return x >= 0 && y >= 0 && x < COLS && y < ROWS;
}

function buildTerrain() {
  const cells = new Array(COLS * ROWS);
  for (let y = 0; y < ROWS; y += 1) {
    for (let x = 0; x < COLS; x += 1) {
      const unknownA = ((x - 21) ** 2) / 24 + ((y - 24) ** 2) / 18 < 1;
      const unknownB = ((x - 61) ** 2) / 17 + ((y - 35) ** 2) / 10 < 1;
      const sparseEdge = x < 4 && y < 13;
      const observed = !(unknownA || unknownB || sparseEdge);

      let z = 0.004 * Math.sin(x * 0.16) * Math.cos(y * 0.13);
      z += 0.006 * (x / COLS);

      // The raised platform introduces an abrupt 12 cm discontinuity.
      if (x >= 38 && y < 30) z += 0.12;

      // A continuous ramp gives the planner a feasible way onto the raised side.
      if (y >= 30) z += 0.12 * smoothstep(27, 47, x);

      // Smooth mound for slope-dependent risk.
      z += 0.085 * gaussian(x, y, 58, 13, 8.5, 7.2);

      // Deterministic rough region. The V0 hard roughness gate is disabled.
      const roughMask = smoothstep(47, 51, x) * (1 - smoothstep(66, 70, x)) *
        smoothstep(29, 32, y) * (1 - smoothstep(42, 45, y));
      z += roughMask * 0.012 * Math.sin(x * 1.35 + y * 0.62) * Math.cos(y * 1.18);

      cells[cellIndex(x, y)] = { x, y, z, observed };
    }
  }
  return cells;
}

const terrain = buildTerrain();

function getCell(x, y) {
  if (!inBounds(x, y)) return null;
  return terrain[cellIndex(x, y)];
}

function getObservedZ(x, y, fallback = null) {
  const cell = getCell(x, y);
  return cell && cell.observed ? cell.z : fallback;
}

function gradientAt(x, y) {
  const center = getObservedZ(x, y, 0);
  const left = getObservedZ(x - 2, y, getObservedZ(x - 1, y, center));
  const right = getObservedZ(x + 2, y, getObservedZ(x + 1, y, center));
  const down = getObservedZ(x, y - 2, getObservedZ(x, y - 1, center));
  const up = getObservedZ(x, y + 2, getObservedZ(x, y + 1, center));
  return {
    gx: (right - left) / (4 * RESOLUTION_M),
    gy: (up - down) / (4 * RESOLUTION_M)
  };
}

function computeMetrics() {
  const result = new Array(COLS * ROWS);
  for (let y = 0; y < ROWS; y += 1) {
    for (let x = 0; x < COLS; x += 1) {
      const cell = getCell(x, y);
      if (!cell.observed) {
        result[cellIndex(x, y)] = {
          slopeDeg: NaN,
          roughnessM: NaN,
          stepM: NaN,
          supportRatio: 0
        };
        continue;
      }

      const { gx, gy } = gradientAt(x, y);
      const slopeDeg = Math.atan(Math.hypot(gx, gy)) * 180 / Math.PI;

      let stepM = 0;
      for (let oy = -1; oy <= 1; oy += 1) {
        for (let ox = -1; ox <= 1; ox += 1) {
          if (ox === 0 && oy === 0) continue;
          const neighborZ = getObservedZ(x + ox, y + oy, null);
          if (neighborZ !== null) stepM = Math.max(stepM, Math.abs(neighborZ - cell.z));
        }
      }

      let residualSum = 0;
      let residualSqSum = 0;
      let residualCount = 0;
      for (let oy = -2; oy <= 2; oy += 1) {
        for (let ox = -2; ox <= 2; ox += 1) {
          if (ox * ox + oy * oy > 4) continue;
          const neighbor = getCell(x + ox, y + oy);
          if (!neighbor || !neighbor.observed) continue;
          const predicted = cell.z + gx * ox * RESOLUTION_M + gy * oy * RESOLUTION_M;
          const residual = neighbor.z - predicted;
          residualSum += residual;
          residualSqSum += residual * residual;
          residualCount += 1;
        }
      }
      const residualMean = residualCount ? residualSum / residualCount : 0;
      const roughnessM = residualCount > 1
        ? Math.sqrt(Math.max(0, residualSqSum / residualCount - residualMean * residualMean))
        : 0;

      let observedCount = 0;
      let totalCount = 0;
      for (let oy = -SUPPORT_RADIUS_CELLS; oy <= SUPPORT_RADIUS_CELLS; oy += 1) {
        for (let ox = -SUPPORT_RADIUS_CELLS; ox <= SUPPORT_RADIUS_CELLS; ox += 1) {
          if (ox * ox + oy * oy > SUPPORT_RADIUS_CELLS * SUPPORT_RADIUS_CELLS) continue;
          totalCount += 1;
          const sample = getCell(x + ox, y + oy);
          if (sample && sample.observed) observedCount += 1;
        }
      }

      result[cellIndex(x, y)] = {
        slopeDeg,
        roughnessM,
        stepM,
        supportRatio: totalCount ? observedCount / totalCount : 0
      };
    }
  }
  return result;
}

const metrics = computeMetrics();
const observedHeights = terrain.filter((cell) => cell.observed).map((cell) => cell.z);
const minHeight = Math.min(...observedHeights);
const maxHeight = Math.max(...observedHeights);
const maxRoughness = Math.max(...metrics.filter((m) => Number.isFinite(m.roughnessM)).map((m) => m.roughnessM), 0.001);

const WORLD_X0 = -((COLS - 1) * RESOLUTION_M) / 2;
const WORLD_Z0 = -((ROWS - 1) * RESOLUTION_M) / 2;

function worldX(x) {
  return WORLD_X0 + x * RESOLUTION_M;
}

function worldZ(y) {
  return WORLD_Z0 + y * RESOLUTION_M;
}

function humanReason(x, y) {
  const cell = getCell(x, y);
  if (!cell || !cell.observed) return 'UNKNOWN';
  const metric = metrics[cellIndex(x, y)];
  if (metric.supportRatio + EPS < 1) return 'INSUFFICIENT SUPPORT';
  if (metric.stepM > maxStepM + EPS) return 'STEP LIMIT';
  if (metric.slopeDeg > maxSlopeDeg + EPS) return 'SLOPE LIMIT';
  return 'TRAVERSABLE';
}

function mixColor(a, b, t) {
  return a.clone().lerp(b, clamp(t, 0, 1));
}

const palette = {
  dark: new THREE.Color('#24282a'),
  mid: new THREE.Color('#717875'),
  light: new THREE.Color('#d9dcd9'),
  slope: new THREE.Color('#b08a61'),
  rough: new THREE.Color('#8b8296'),
  invalid: new THREE.Color('#6f4646'),
  valid: new THREE.Color('#8e9692')
};

function colorForCell(x, y) {
  const cell = getCell(x, y);
  if (!cell || !cell.observed) return palette.dark;
  const metric = metrics[cellIndex(x, y)];

  if (activeLayer === 'height') {
    const t = (cell.z - minHeight) / Math.max(maxHeight - minHeight, EPS);
    return mixColor(palette.dark, palette.light, t);
  }
  if (activeLayer === 'slope') {
    const t = clamp(metric.slopeDeg / Math.max(maxSlopeDeg, 1), 0, 1.4) / 1.4;
    return mixColor(palette.mid, palette.slope, t);
  }
  if (activeLayer === 'roughness') {
    const t = clamp(metric.roughnessM / maxRoughness, 0, 1);
    return mixColor(palette.mid, palette.rough, t);
  }
  if (activeLayer === 'step') {
    const t = clamp(metric.stepM / Math.max(maxStepM, 0.001), 0, 1.4) / 1.4;
    return mixColor(palette.mid, palette.invalid, t);
  }

  return humanReason(x, y) === 'TRAVERSABLE' ? palette.valid : palette.invalid;
}

function addVertex(positions, colors, cells, x, y, sourceX, sourceY, color) {
  const cell = getCell(x, y);
  positions.push(worldX(x), cell.z, worldZ(y));
  colors.push(color.r, color.g, color.b);
  cells.push(sourceX, sourceY);
}

function buildSurfaceGeometry() {
  const positions = [];
  const colors = [];
  const cells = [];

  for (let y = 0; y < ROWS - 1; y += 1) {
    for (let x = 0; x < COLS - 1; x += 1) {
      const c00 = getCell(x, y);
      const c10 = getCell(x + 1, y);
      const c01 = getCell(x, y + 1);
      const c11 = getCell(x + 1, y + 1);
      if (![c00, c10, c01, c11].every((cell) => cell.observed)) continue;

      const color = colorForCell(x, y);
      // Winding is chosen so normals face +Y.
      addVertex(positions, colors, cells, x, y, x, y, color);
      addVertex(positions, colors, cells, x + 1, y + 1, x, y, color);
      addVertex(positions, colors, cells, x + 1, y, x, y, color);

      addVertex(positions, colors, cells, x, y, x, y, color);
      addVertex(positions, colors, cells, x, y + 1, x, y, color);
      addVertex(positions, colors, cells, x + 1, y + 1, x, y, color);
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geometry.setAttribute('cellX', new THREE.Float32BufferAttribute(cells.filter((_, i) => i % 2 === 0), 1));
  geometry.setAttribute('cellY', new THREE.Float32BufferAttribute(cells.filter((_, i) => i % 2 === 1), 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function buildStepWallGeometry() {
  const positions = [];
  const boundaryX = worldX(37.5);

  for (let y = 0; y < 29; y += 1) {
    const low0 = getCell(37, y);
    const high0 = getCell(38, y);
    const low1 = getCell(37, y + 1);
    const high1 = getCell(38, y + 1);
    if (![low0, high0, low1, high1].every((cell) => cell?.observed)) continue;

    const z0 = worldZ(y);
    const z1 = worldZ(y + 1);
    positions.push(
      boundaryX, low0.z, z0,
      boundaryX, high1.z, z1,
      boundaryX, high0.z, z0,
      boundaryX, low0.z, z0,
      boundaryX, low1.z, z1,
      boundaryX, high1.z, z1
    );
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.computeVertexNormals();
  return geometry;
}

const scene = new THREE.Scene();
scene.background = new THREE.Color('#0a0c0d');
scene.fog = new THREE.Fog('#0a0c0d', 5.8, 9.8);

const camera = new THREE.PerspectiveCamera(38, 1, 0.02, 30);
const defaultCameraPosition = new THREE.Vector3(4.2, 2.8, 4.35);
const defaultTarget = new THREE.Vector3(0, 0.12, 0);
camera.position.copy(defaultCameraPosition);

const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
container.prepend(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.copy(defaultTarget);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.minDistance = 2.2;
controls.maxDistance = 9;
controls.maxPolarAngle = Math.PI * 0.495;
controls.update();

scene.add(new THREE.HemisphereLight(0xf2f4f2, 0x202427, 1.65));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
keyLight.position.set(3.2, 5.8, 2.4);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0x9aa4a1, 0.75);
fillLight.position.set(-4, 2.6, -3);
scene.add(fillLight);

const grid = new THREE.GridHelper(5.2, 52, 0x4b5150, 0x272c2d);
grid.position.y = -0.018;
const gridMaterials = Array.isArray(grid.material) ? grid.material : [grid.material];
gridMaterials.forEach((material) => {
  material.transparent = true;
  material.opacity = 0.34;
});
scene.add(grid);

const surfaceGeometry = buildSurfaceGeometry();
const surfaceMaterial = new THREE.MeshStandardMaterial({
  vertexColors: true,
  roughness: 0.95,
  metalness: 0,
  side: THREE.DoubleSide
});
const surfaceMesh = new THREE.Mesh(surfaceGeometry, surfaceMaterial);
surfaceMesh.scale.y = zScale;
scene.add(surfaceMesh);

const wire = new THREE.LineSegments(
  new THREE.WireframeGeometry(surfaceGeometry),
  new THREE.LineBasicMaterial({ color: 0x1d2122, transparent: true, opacity: 0.22 })
);
wire.scale.y = zScale;
scene.add(wire);

const stepWallMaterial = new THREE.MeshStandardMaterial({
  color: maxStepM < 0.12 ? 0x6f4646 : 0x626966,
  roughness: 1,
  side: THREE.DoubleSide
});
const stepWall = new THREE.Mesh(buildStepWallGeometry(), stepWallMaterial);
stepWall.scale.y = zScale;
scene.add(stepWall);

function makeTextSprite(text) {
  const canvas = document.createElement('canvas');
  canvas.width = 768;
  canvas.height = 180;
  const context = canvas.getContext('2d');
  context.fillStyle = 'rgba(7,9,10,.82)';
  context.strokeStyle = 'rgba(255,255,255,.22)';
  context.lineWidth = 3;
  context.beginPath();
  context.roundRect(8, 8, 752, 164, 24);
  context.fill();
  context.stroke();
  context.fillStyle = '#f2f3f1';
  context.font = '600 48px Inter, Arial, sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(text, 384, 90);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(1.15, 0.27, 1);
  return sprite;
}

const labelDefs = [
  { text: 'SLOPE / RAMP', x: 36, y: 37, lift: 0.27 },
  { text: '12 cm STEP', x: 38, y: 17, lift: 0.34 },
  { text: 'RAISED PLATFORM', x: 49, y: 24, lift: 0.26 }
];
const labels = labelDefs.map((definition) => {
  const sprite = makeTextSprite(definition.text);
  scene.add(sprite);
  return { ...definition, sprite };
});

const selectionMarker = new THREE.Mesh(
  new THREE.RingGeometry(0.05, 0.068, 36),
  new THREE.MeshBasicMaterial({ color: 0xf4f4f0, side: THREE.DoubleSide, depthTest: false })
);
selectionMarker.rotation.x = -Math.PI / 2;
selectionMarker.visible = false;
scene.add(selectionMarker);

function updateLabelPositions() {
  labels.forEach((label) => {
    const cell = getCell(label.x, label.y);
    label.sprite.position.set(
      worldX(label.x),
      (cell?.z || 0) * zScale + label.lift,
      worldZ(label.y)
    );
  });
}

function updateSelectionMarker() {
  if (!selectedCell) {
    selectionMarker.visible = false;
    return;
  }
  const cell = getCell(selectedCell.x, selectedCell.y);
  if (!cell?.observed) {
    selectionMarker.visible = false;
    return;
  }
  selectionMarker.visible = true;
  selectionMarker.position.set(
    worldX(selectedCell.x),
    cell.z * zScale + 0.014,
    worldZ(selectedCell.y)
  );
}

function updateSurfaceColors() {
  const colorAttribute = surfaceGeometry.getAttribute('color');
  const cellXAttribute = surfaceGeometry.getAttribute('cellX');
  const cellYAttribute = surfaceGeometry.getAttribute('cellY');
  for (let i = 0; i < colorAttribute.count; i += 1) {
    const x = Math.round(cellXAttribute.getX(i));
    const y = Math.round(cellYAttribute.getX(i));
    const color = colorForCell(x, y);
    colorAttribute.setXYZ(i, color.r, color.g, color.b);
  }
  colorAttribute.needsUpdate = true;
  stepWallMaterial.color.set(maxStepM < 0.12 ? 0x6f4646 : 0x626966);
}

function applyZScale(nextScale) {
  zScale = nextScale;
  surfaceMesh.scale.y = zScale;
  wire.scale.y = zScale;
  stepWall.scale.y = zScale;
  updateLabelPositions();
  updateSelectionMarker();
  scaleNote.textContent = `Z ×${zScale} VISUAL SCALE`;
  zScaleButtons.forEach((button) => {
    button.classList.toggle('active', Number(button.dataset.zScale) === zScale);
  });
}

function updateInspector(x, y) {
  const cell = getCell(x, y);
  const metric = metrics[cellIndex(x, y)];
  selectedCell = { x, y };
  updateSelectionMarker();
  const status = humanReason(x, y);
  info.innerHTML = `
    <span>${status}</span>
    <strong>Grid ${x}, ${y}</strong>
    <dl>
      <div><dt>HEIGHT</dt><dd>${cell.z.toFixed(3)} m</dd></div>
      <div><dt>SLOPE</dt><dd>${metric.slopeDeg.toFixed(1)}°</dd></div>
      <div><dt>STEP</dt><dd>${(metric.stepM * 100).toFixed(1)} cm</dd></div>
      <div><dt>ROUGH</dt><dd>${(metric.roughnessM * 1000).toFixed(1)} mm</dd></div>
      <div><dt>SUPPORT</dt><dd>${(metric.supportRatio * 100).toFixed(0)}%</dd></div>
    </dl>`;
}

function resetCamera() {
  camera.position.copy(defaultCameraPosition);
  controls.target.copy(defaultTarget);
  controls.update();
}

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let pointerDown = null;

renderer.domElement.addEventListener('pointerdown', (event) => {
  pointerDown = { x: event.clientX, y: event.clientY };
});

renderer.domElement.addEventListener('pointerup', (event) => {
  if (!pointerDown) return;
  const movement = Math.hypot(event.clientX - pointerDown.x, event.clientY - pointerDown.y);
  pointerDown = null;
  if (movement > 5) return;

  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObject(surfaceMesh, false)[0];
  if (!hit || hit.faceIndex == null) return;
  const vertexIndex = hit.faceIndex * 3;
  const x = Math.round(surfaceGeometry.getAttribute('cellX').getX(vertexIndex));
  const y = Math.round(surfaceGeometry.getAttribute('cellY').getX(vertexIndex));
  if (inBounds(x, y) && getCell(x, y)?.observed) updateInspector(x, y);
});

layerButtons.forEach((button) => {
  button.addEventListener('click', () => {
    activeLayer = button.dataset.terrainLayer;
    updateSurfaceColors();
  });
});

slopeRange?.addEventListener('input', () => {
  maxSlopeDeg = Number(slopeRange.value);
  updateSurfaceColors();
  if (selectedCell) updateInspector(selectedCell.x, selectedCell.y);
});

stepRange?.addEventListener('input', () => {
  maxStepM = Number(stepRange.value) / 100;
  updateSurfaceColors();
  if (selectedCell) updateInspector(selectedCell.x, selectedCell.y);
});

zScaleButtons.forEach((button) => {
  button.addEventListener('click', () => applyZScale(Number(button.dataset.zScale)));
});

resetViewButton?.addEventListener('click', resetCamera);

const resizeObserver = new ResizeObserver(() => {
  const width = Math.max(1, container.clientWidth);
  const height = Math.max(1, container.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
});
resizeObserver.observe(container);

updateLabelPositions();
updateSurfaceColors();
applyZScale(3);
loading?.remove();

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();
