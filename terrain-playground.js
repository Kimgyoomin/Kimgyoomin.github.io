(() => {
  'use strict';

  const canvas = document.getElementById('terrainCanvas');
  if (!canvas) return;

  const ctx = canvas.getContext('2d');
  const RESOLUTION_M = 0.05;
  const COLS = 76;
  const ROWS = 46;
  const BASELINE = Object.freeze({
    maxSlopeDeg: 15,
    maxStepM: 0.08,
    pcaRadiusM: 0.30,
    supportRadiusM: 0.20,
    minSupportRatio: 1.0,
    samplingDistanceM: 0.30,
    samplesPerExpansion: 20,
    mergeRadiusM: 0.20,
    neighborRadiusM: 0.45,
    goalConnectionM: 0.45,
    slopeRiskWeight: 3.0,
    distanceWeight: 1.0,
    maxNodes: 680,
    maxExpansions: 520
  });

  const GOLDEN_ANGLE = Math.PI * (3 - Math.sqrt(5));
  const EPS = 1e-9;

  const state = {
    layer: 'traversability',
    tool: 'inspect',
    maxSlopeDeg: BASELINE.maxSlopeDeg,
    maxStepM: BASELINE.maxStepM,
    showGraph: true,
    showRejected: true,
    start: { x: 8.5, y: 38.5 },
    goal: { x: 68.5, y: 8.5 },
    selectedCell: null,
    plan: null,
    animationStart: 0,
    animationDuration: 1500,
    canvasWidth: 0,
    canvasHeight: 0,
    dpr: 1
  };

  const terrain = buildTerrain();
  const metrics = computeMetrics();

  const els = {
    slopeRange: document.getElementById('slopeLimit'),
    slopeValue: document.getElementById('slopeLimitValue'),
    stepRange: document.getElementById('stepLimit'),
    stepValue: document.getElementById('stepLimitValue'),
    runButton: document.getElementById('runPlanner'),
    resetButton: document.getElementById('resetPlanner'),
    graphToggle: document.getElementById('showGraph'),
    rejectedToggle: document.getElementById('showRejected'),
    interactionHint: document.getElementById('interactionHint'),
    layerLegend: document.getElementById('layerLegend'),
    planStatus: document.getElementById('planStatus'),
    planMetrics: document.getElementById('planMetrics'),
    inspector: document.getElementById('terrainInspector'),
    plannerDescription: document.getElementById('plannerDescription')
  };

  const layerButtons = [...document.querySelectorAll('[data-terrain-layer]')];
  const toolButtons = [...document.querySelectorAll('[data-terrain-tool]')];

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
        let observed = true;
        const unknownA = ((x - 21) ** 2) / 24 + ((y - 24) ** 2) / 18 < 1;
        const unknownB = ((x - 61) ** 2) / 17 + ((y - 35) ** 2) / 10 < 1;
        const sparseEdge = x < 4 && y < 13;
        if (unknownA || unknownB || sparseEdge) observed = false;

        let z = 0.004 * Math.sin(x * 0.16) * Math.cos(y * 0.13);
        z += 0.006 * (x / COLS);

        // A raised platform creates a direct 12 cm step.
        if (x >= 38 && y < 30) z += 0.12;

        // A longer ramp provides a feasible route around the step boundary.
        if (y >= 30) z += 0.12 * smoothstep(27, 47, x);

        // A smooth mound creates slope-dependent risk on the raised side.
        z += 0.085 * gaussian(x, y, 58, 13, 8.5, 7.2);

        // Deterministic rough patch. V0 visualizes roughness but does not hard-gate it.
        const roughMask = smoothstep(47, 51, x) * (1 - smoothstep(66, 70, x)) *
          smoothstep(29, 32, y) * (1 - smoothstep(42, 45, y));
        z += roughMask * 0.012 * Math.sin(x * 1.35 + y * 0.62) * Math.cos(y * 1.18);

        cells[cellIndex(x, y)] = { x, y, observed, z };
      }
    }
    return cells;
  }

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
    const supportRadiusCells = Math.round(BASELINE.supportRadiusM / RESOLUTION_M);

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

        let maxStep = 0;
        for (let oy = -1; oy <= 1; oy += 1) {
          for (let ox = -1; ox <= 1; ox += 1) {
            if (ox === 0 && oy === 0) continue;
            const neighborZ = getObservedZ(x + ox, y + oy, null);
            if (neighborZ !== null) maxStep = Math.max(maxStep, Math.abs(neighborZ - cell.z));
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

        let observed = 0;
        let total = 0;
        for (let oy = -supportRadiusCells; oy <= supportRadiusCells; oy += 1) {
          for (let ox = -supportRadiusCells; ox <= supportRadiusCells; ox += 1) {
            if (ox * ox + oy * oy > supportRadiusCells * supportRadiusCells) continue;
            total += 1;
            const sample = getCell(x + ox, y + oy);
            if (sample && sample.observed) observed += 1;
          }
        }

        result[cellIndex(x, y)] = {
          slopeDeg,
          roughnessM,
          stepM: maxStep,
          supportRatio: total ? observed / total : 0
        };
      }
    }
    return result;
  }

  function nearestCell(position) {
    return {
      x: Math.round(position.x - 0.5),
      y: Math.round(position.y - 0.5)
    };
  }

  function evaluateNode(position) {
    const index = nearestCell(position);
    if (!inBounds(index.x, index.y)) {
      return { valid: false, reason: 'out_of_bounds', index, position };
    }
    const cell = getCell(index.x, index.y);
    const metric = metrics[cellIndex(index.x, index.y)];
    if (!cell.observed) {
      return { valid: false, reason: 'unknown', index, position, cell, metric };
    }
    if (metric.supportRatio + EPS < BASELINE.minSupportRatio) {
      return { valid: false, reason: 'insufficient_footprint_support', index, position, cell, metric };
    }
    if (metric.stepM > state.maxStepM + EPS) {
      return { valid: false, reason: 'step_limit', index, position, cell, metric };
    }
    if (metric.slopeDeg > state.maxSlopeDeg + EPS) {
      return { valid: false, reason: 'slope_limit', index, position, cell, metric };
    }
    return { valid: true, reason: 'none', index, position, cell, metric, elevationM: cell.z };
  }

  function evaluateEdge(from, to) {
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const distanceCells = Math.hypot(dx, dy);
    if (distanceCells <= EPS) return { valid: false, reason: 'invalid_input' };

    const segmentCount = Math.max(1, Math.ceil(distanceCells / 0.5));
    const samples = [];
    let previousZ = null;
    let maxStepM = 0;

    // Match the C++ diagnostic precedence: bounds -> unknown -> step.
    for (let i = 0; i <= segmentCount; i += 1) {
      const t = i / segmentCount;
      const position = { x: from.x + dx * t, y: from.y + dy * t };
      const index = nearestCell(position);
      if (!inBounds(index.x, index.y)) {
        return { valid: false, reason: 'out_of_bounds', rejectedAt: position, maxStepM };
      }
      const cell = getCell(index.x, index.y);
      if (!cell.observed) {
        return { valid: false, reason: 'unknown', rejectedAt: position, maxStepM };
      }
      if (previousZ !== null) {
        const dz = Math.abs(cell.z - previousZ);
        maxStepM = Math.max(maxStepM, dz);
        if (dz > state.maxStepM + EPS) {
          return { valid: false, reason: 'step_limit', rejectedAt: position, maxStepM };
        }
      }
      previousZ = cell.z;
      samples.push({ position, cell });
    }

    let maxSlopeDeg = 0;
    let meanSlopeDeg = 0;
    let minSupportRatio = 1;
    for (const sample of samples) {
      const evaluation = evaluateNode(sample.position);
      if (!evaluation.valid) {
        return {
          valid: false,
          reason: evaluation.reason,
          rejectedAt: sample.position,
          maxStepM,
          maxSlopeDeg,
          minSupportRatio
        };
      }
      maxSlopeDeg = Math.max(maxSlopeDeg, evaluation.metric.slopeDeg);
      meanSlopeDeg += evaluation.metric.slopeDeg;
      minSupportRatio = Math.min(minSupportRatio, evaluation.metric.supportRatio);
    }
    meanSlopeDeg /= samples.length;

    let length3dM = 0;
    for (let i = 1; i < samples.length; i += 1) {
      const a = samples[i - 1];
      const b = samples[i];
      const dxy = Math.hypot(
        b.position.x - a.position.x,
        b.position.y - a.position.y
      ) * RESOLUTION_M;
      length3dM += Math.hypot(dxy, b.cell.z - a.cell.z);
    }

    const normalizedSlopeRisk = clamp(maxSlopeDeg / Math.max(state.maxSlopeDeg, EPS), 0, 1);
    const cost = length3dM *
      (BASELINE.distanceWeight + BASELINE.slopeRiskWeight * normalizedSlopeRisk);

    return {
      valid: true,
      reason: 'none',
      cost,
      length3dM,
      maxSlopeDeg,
      meanSlopeDeg,
      maxStepM,
      minSupportRatio,
      samples
    };
  }

  function pointDistance(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }

  function edgeKey(a, b) {
    return a < b ? `${a}:${b}` : `${b}:${a}`;
  }

  function runWavefrontPlanner() {
    const startedAt = performance.now();
    const startEvaluation = evaluateNode(state.start);
    const goalEvaluation = evaluateNode(state.goal);

    if (!startEvaluation.valid || !goalEvaluation.valid) {
      return {
        success: false,
        message: !startEvaluation.valid
          ? `Start is invalid: ${humanReason(startEvaluation.reason)}`
          : `Goal is invalid: ${humanReason(goalEvaluation.reason)}`,
        nodes: [], edges: [], rejected: [], path: [], elapsedMs: performance.now() - startedAt
      };
    }

    const nodes = [
      { id: 0, point: { ...state.start }, depth: 0, role: 'start', evaluation: startEvaluation },
      { id: 1, point: { ...state.goal }, depth: 0, role: 'goal', evaluation: goalEvaluation }
    ];
    const edges = [];
    const rejected = [];
    const frontier = [0];
    const edgeKeys = new Set();
    let frontierIndex = 0;
    let goalConnected = false;
    let expansions = 0;

    function reject(sourceId, candidate, kind, reason, detail = null) {
      rejected.push({ sourceId, candidate: { ...candidate }, kind, reason, detail });
    }

    function addEdge(from, to, evaluation, flags = {}) {
      if (from === to) return false;
      const key = edgeKey(from, to);
      if (edgeKeys.has(key)) return false;
      edgeKeys.add(key);
      edges.push({ id: edges.length, from, to, evaluation, cost: evaluation.cost, ...flags });
      return true;
    }

    function tryGoalConnection(sourceId) {
      if (sourceId === 1 || goalConnected) return;
      const source = nodes[sourceId];
      const goalDistanceCells = BASELINE.goalConnectionM / RESOLUTION_M;
      if (pointDistance(source.point, state.goal) > goalDistanceCells + EPS) return;
      const evaluation = evaluateEdge(source.point, state.goal);
      if (!evaluation.valid) {
        reject(sourceId, state.goal, 'goal_edge_invalid', evaluation.reason, evaluation);
        return;
      }
      if (addEdge(sourceId, 1, evaluation, { goalConnection: true })) goalConnected = true;
    }

    tryGoalConnection(0);

    const samplingRadiusCells = BASELINE.samplingDistanceM / RESOLUTION_M;
    const mergeRadiusCells = BASELINE.mergeRadiusM / RESOLUTION_M;
    const neighborRadiusCells = BASELINE.neighborRadiusM / RESOLUTION_M;

    while (
      frontierIndex < frontier.length &&
      nodes.length < BASELINE.maxNodes &&
      expansions < BASELINE.maxExpansions &&
      !goalConnected
    ) {
      const sourceId = frontier[frontierIndex++];
      const source = nodes[sourceId];
      expansions += 1;
      const phase = (source.depth * GOLDEN_ANGLE) % (2 * Math.PI);

      for (let sample = 0; sample < BASELINE.samplesPerExpansion; sample += 1) {
        const angle = phase + 2 * Math.PI * sample / BASELINE.samplesPerExpansion;
        const candidate = {
          x: source.point.x + samplingRadiusCells * Math.cos(angle),
          y: source.point.y + samplingRadiusCells * Math.sin(angle)
        };

        const nodeEvaluation = evaluateNode(candidate);
        if (!nodeEvaluation.valid) {
          reject(sourceId, candidate, 'node_invalid', nodeEvaluation.reason, nodeEvaluation);
          continue;
        }

        let mergeTarget = -1;
        let mergeDistance = Infinity;
        for (let i = 0; i < nodes.length; i += 1) {
          if (i === 1 || i === sourceId) continue;
          const distance = pointDistance(candidate, nodes[i].point);
          if (distance < mergeDistance) {
            mergeDistance = distance;
            mergeTarget = i;
          }
        }

        if (mergeTarget >= 0 && mergeDistance <= mergeRadiusCells + EPS) {
          const evaluation = evaluateEdge(source.point, nodes[mergeTarget].point);
          if (evaluation.valid) addEdge(sourceId, mergeTarget, evaluation, { loopClosure: true });
          else reject(sourceId, nodes[mergeTarget].point, 'merge_edge_invalid', evaluation.reason, evaluation);
          tryGoalConnection(mergeTarget);
          continue;
        }

        const expansionEvaluation = evaluateEdge(source.point, candidate);
        if (!expansionEvaluation.valid) {
          reject(sourceId, candidate, 'expansion_edge_invalid', expansionEvaluation.reason, expansionEvaluation);
          continue;
        }

        const newId = nodes.length;
        nodes.push({
          id: newId,
          point: candidate,
          depth: source.depth + 1,
          role: 'sampled',
          evaluation: nodeEvaluation
        });
        addEdge(sourceId, newId, expansionEvaluation, { expansion: true });

        for (let neighborId = 0; neighborId < newId; neighborId += 1) {
          if (neighborId === 1 || neighborId === sourceId) continue;
          const neighbor = nodes[neighborId];
          if (pointDistance(candidate, neighbor.point) > neighborRadiusCells + EPS) continue;
          const key = edgeKey(newId, neighborId);
          if (edgeKeys.has(key)) continue;
          const loopEvaluation = evaluateEdge(candidate, neighbor.point);
          if (loopEvaluation.valid) addEdge(newId, neighborId, loopEvaluation, { loopClosure: true });
          else reject(newId, neighbor.point, 'merge_edge_invalid', loopEvaluation.reason, loopEvaluation);
        }

        frontier.push(newId);
        tryGoalConnection(newId);
        if (goalConnected || nodes.length >= BASELINE.maxNodes) break;
      }
    }

    const pathNodeIds = goalConnected ? aStar(nodes, edges, 0, 1) : [];
    const path = pathNodeIds.map((id) => nodes[id].point);
    const pathEdges = [];
    let pathLengthM = 0;
    let pathCost = 0;
    let pathMaxSlopeDeg = 0;
    let pathMaxStepM = 0;

    for (let i = 1; i < pathNodeIds.length; i += 1) {
      const from = pathNodeIds[i - 1];
      const to = pathNodeIds[i];
      const edge = edges.find((candidate) =>
        (candidate.from === from && candidate.to === to) ||
        (candidate.from === to && candidate.to === from));
      if (!edge) continue;
      pathEdges.push(edge);
      pathLengthM += edge.evaluation.length3dM;
      pathCost += edge.cost;
      pathMaxSlopeDeg = Math.max(pathMaxSlopeDeg, edge.evaluation.maxSlopeDeg);
      pathMaxStepM = Math.max(pathMaxStepM, edge.evaluation.maxStepM);
    }

    const success = path.length > 0;
    return {
      success,
      message: success
        ? 'Goal connected and terrain-aware A* path found.'
        : goalConnected
          ? 'Goal connected, but graph search failed.'
          : 'Frontier exhausted before a valid goal connection.',
      nodes,
      edges,
      rejected,
      path,
      pathNodeIds,
      pathEdges,
      pathLengthM,
      pathCost,
      pathMaxSlopeDeg,
      pathMaxStepM,
      expansions,
      elapsedMs: performance.now() - startedAt
    };
  }

  function aStar(nodes, edges, startId, goalId) {
    const adjacency = Array.from({ length: nodes.length }, () => []);
    for (const edge of edges) {
      adjacency[edge.from].push({ id: edge.to, cost: edge.cost });
      adjacency[edge.to].push({ id: edge.from, cost: edge.cost });
    }

    const gScore = new Array(nodes.length).fill(Infinity);
    const parent = new Array(nodes.length).fill(-1);
    const closed = new Set();
    const heuristic = (id) => pointDistance(nodes[id].point, nodes[goalId].point) *
      RESOLUTION_M * BASELINE.distanceWeight;
    const open = [{ id: startId, g: 0, f: heuristic(startId) }];
    gScore[startId] = 0;

    while (open.length) {
      open.sort((a, b) => a.f - b.f || a.g - b.g || a.id - b.id);
      const current = open.shift();
      if (closed.has(current.id)) continue;
      if (current.id === goalId) break;
      closed.add(current.id);

      for (const adjacent of adjacency[current.id]) {
        const tentative = current.g + adjacent.cost;
        if (tentative + EPS >= gScore[adjacent.id]) continue;
        gScore[adjacent.id] = tentative;
        parent[adjacent.id] = current.id;
        open.push({ id: adjacent.id, g: tentative, f: tentative + heuristic(adjacent.id) });
      }
    }

    if (!Number.isFinite(gScore[goalId])) return [];
    const reversed = [];
    for (let id = goalId; id >= 0; id = parent[id]) {
      reversed.push(id);
      if (id === startId) break;
    }
    if (reversed[reversed.length - 1] !== startId) return [];
    return reversed.reverse();
  }

  function humanReason(reason) {
    const labels = {
      none: 'valid',
      out_of_bounds: 'outside the elevation map',
      unknown: 'unknown / unobserved terrain',
      insufficient_footprint_support: 'insufficient observed support',
      insufficient_pca_support: 'insufficient surface samples',
      slope_limit: 'slope limit exceeded',
      roughness_limit: 'roughness limit exceeded',
      step_limit: 'step-height limit exceeded',
      invalid_input: 'invalid input'
    };
    return labels[reason] || reason;
  }

  function colorMix(a, b, t) {
    const parse = (hex) => [
      parseInt(hex.slice(1, 3), 16),
      parseInt(hex.slice(3, 5), 16),
      parseInt(hex.slice(5, 7), 16)
    ];
    const aa = parse(a);
    const bb = parse(b);
    const rgb = aa.map((value, i) => Math.round(value + (bb[i] - value) * clamp(t, 0, 1)));
    return `rgb(${rgb.join(',')})`;
  }

  function layerColor(cell, metric) {
    if (!cell.observed) return '#d7d7d3';
    if (state.layer === 'height') {
      return colorMix('#f6f6f3', '#34363a', clamp((cell.z + 0.01) / 0.24, 0, 1));
    }
    if (state.layer === 'slope') {
      return colorMix('#f6f6f3', '#d17a24', clamp(metric.slopeDeg / 24, 0, 1));
    }
    if (state.layer === 'roughness') {
      return colorMix('#f6f6f3', '#675a8f', clamp(metric.roughnessM / 0.025, 0, 1));
    }
    if (state.layer === 'step') {
      return colorMix('#f6f6f3', '#b83a3a', clamp(metric.stepM / 0.14, 0, 1));
    }

    const evaluation = evaluateNode({ x: cell.x + 0.5, y: cell.y + 0.5 });
    if (!evaluation.valid) {
      if (evaluation.reason === 'unknown') return '#d7d7d3';
      if (evaluation.reason === 'insufficient_footprint_support') return '#aaa9a4';
      return '#2f3032';
    }
    const risk = clamp(metric.slopeDeg / Math.max(state.maxSlopeDeg, EPS), 0, 1);
    return colorMix('#f4f4f1', '#6d8aa5', risk);
  }

  function draw(progress = 1) {
    const width = state.canvasWidth;
    const height = state.canvasHeight;
    if (!width || !height) return;

    ctx.clearRect(0, 0, width, height);
    const cellW = width / COLS;
    const cellH = height / ROWS;

    for (let y = 0; y < ROWS; y += 1) {
      for (let x = 0; x < COLS; x += 1) {
        const cell = getCell(x, y);
        const metric = metrics[cellIndex(x, y)];
        ctx.fillStyle = layerColor(cell, metric);
        ctx.fillRect(x * cellW, (ROWS - 1 - y) * cellH, cellW + 0.4, cellH + 0.4);
        if (!cell.observed) {
          ctx.strokeStyle = 'rgba(90,90,90,.28)';
          ctx.lineWidth = 0.7;
          ctx.beginPath();
          ctx.moveTo(x * cellW, (ROWS - y) * cellH);
          ctx.lineTo((x + 1) * cellW, (ROWS - 1 - y) * cellH);
          ctx.stroke();
        }
      }
    }

    ctx.strokeStyle = 'rgba(0,0,0,.05)';
    ctx.lineWidth = 0.5;
    for (let x = 0; x <= COLS; x += 4) {
      ctx.beginPath();
      ctx.moveTo(x * cellW, 0);
      ctx.lineTo(x * cellW, height);
      ctx.stroke();
    }
    for (let y = 0; y <= ROWS; y += 4) {
      ctx.beginPath();
      ctx.moveTo(0, y * cellH);
      ctx.lineTo(width, y * cellH);
      ctx.stroke();
    }

    if (state.plan) drawPlan(state.plan, progress, cellW, cellH);
    drawEndpoint(state.start, 'S', '#2d8c8c', cellW, cellH);
    drawEndpoint(state.goal, 'G', '#b05272', cellW, cellH);

    if (state.selectedCell) {
      const x = state.selectedCell.x * cellW;
      const y = (ROWS - 1 - state.selectedCell.y) * cellH;
      ctx.strokeStyle = '#111';
      ctx.lineWidth = 2;
      ctx.strokeRect(x + 1, y + 1, Math.max(2, cellW - 2), Math.max(2, cellH - 2));
    }
  }

  function canvasPoint(position, cellW, cellH) {
    return { x: position.x * cellW, y: (ROWS - position.y) * cellH };
  }

  function drawEndpoint(position, label, color, cellW, cellH) {
    const p = canvasPoint(position, cellW, cellH);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#fff';
    ctx.font = '600 10px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, p.x, p.y + 0.5);
  }

  function drawPlan(plan, progress, cellW, cellH) {
    const graphProgress = clamp(progress / 0.72, 0, 1);
    const pathProgress = clamp((progress - 0.70) / 0.30, 0, 1);

    if (state.showGraph) {
      const edgeCount = Math.floor(plan.edges.length * graphProgress);
      ctx.lineWidth = 0.75;
      ctx.strokeStyle = 'rgba(35,42,46,.28)';
      for (let i = 0; i < edgeCount; i += 1) {
        const edge = plan.edges[i];
        const a = canvasPoint(plan.nodes[edge.from].point, cellW, cellH);
        const b = canvasPoint(plan.nodes[edge.to].point, cellW, cellH);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }

      const nodeCount = Math.floor(plan.nodes.length * graphProgress);
      ctx.fillStyle = 'rgba(28,34,37,.62)';
      for (let i = 2; i < nodeCount; i += 1) {
        const p = canvasPoint(plan.nodes[i].point, cellW, cellH);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 1.7, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    if (state.showRejected) {
      const rejectedCount = Math.min(650, Math.floor(plan.rejected.length * graphProgress));
      ctx.strokeStyle = 'rgba(168,45,45,.55)';
      ctx.lineWidth = 1;
      for (let i = 0; i < rejectedCount; i += 1) {
        const p = canvasPoint(plan.rejected[i].candidate, cellW, cellH);
        const size = 2.2;
        ctx.beginPath();
        ctx.moveTo(p.x - size, p.y - size);
        ctx.lineTo(p.x + size, p.y + size);
        ctx.moveTo(p.x + size, p.y - size);
        ctx.lineTo(p.x - size, p.y + size);
        ctx.stroke();
      }
    }

    if (plan.path.length > 1 && pathProgress > 0) {
      const totalSegments = plan.path.length - 1;
      const scaled = pathProgress * totalSegments;
      const completeSegments = Math.floor(scaled);
      const partial = scaled - completeSegments;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.strokeStyle = 'rgba(0,0,0,.85)';
      ctx.lineWidth = 7;
      drawPartialPath(plan.path, completeSegments, partial, cellW, cellH);
      ctx.strokeStyle = '#f2c94c';
      ctx.lineWidth = 4;
      drawPartialPath(plan.path, completeSegments, partial, cellW, cellH);
    }
  }

  function drawPartialPath(path, completeSegments, partial, cellW, cellH) {
    if (!path.length) return;
    const start = canvasPoint(path[0], cellW, cellH);
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    for (let i = 1; i <= completeSegments && i < path.length; i += 1) {
      const p = canvasPoint(path[i], cellW, cellH);
      ctx.lineTo(p.x, p.y);
    }
    if (completeSegments < path.length - 1) {
      const a = path[completeSegments];
      const b = path[completeSegments + 1];
      const p = canvasPoint({
        x: a.x + (b.x - a.x) * partial,
        y: a.y + (b.y - a.y) * partial
      }, cellW, cellH);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    state.dpr = Math.min(window.devicePixelRatio || 1, 2);
    state.canvasWidth = Math.max(320, rect.width);
    state.canvasHeight = Math.max(240, rect.height);
    canvas.width = Math.round(state.canvasWidth * state.dpr);
    canvas.height = Math.round(state.canvasHeight * state.dpr);
    ctx.setTransform(state.dpr, 0, 0, state.dpr, 0, 0);
    draw(1);
  }

  function animatePlan() {
    const progress = clamp((performance.now() - state.animationStart) / state.animationDuration, 0, 1);
    draw(progress);
    if (progress < 1) requestAnimationFrame(animatePlan);
  }

  function runPlanner({ animate = true } = {}) {
    state.plan = runWavefrontPlanner();
    updatePlanSummary();
    if (animate) {
      state.animationStart = performance.now();
      requestAnimationFrame(animatePlan);
    } else {
      draw(1);
    }
  }

  function updatePlanSummary() {
    const plan = state.plan;
    if (!plan) return;
    els.planStatus.textContent = plan.success ? 'PATH FOUND' : 'NO PATH';
    els.planStatus.dataset.state = plan.success ? 'success' : 'failure';
    els.plannerDescription.textContent = plan.message;

    const metricRows = [
      ['Nodes', plan.nodes.length || 0],
      ['Edges', plan.edges.length || 0],
      ['Rejected', plan.rejected.length || 0],
      ['Expansions', plan.expansions || 0],
      ['Path length', plan.success ? `${plan.pathLengthM.toFixed(2)} m` : '—'],
      ['Max path slope', plan.success ? `${plan.pathMaxSlopeDeg.toFixed(1)}°` : '—'],
      ['Max path step', plan.success ? `${(plan.pathMaxStepM * 100).toFixed(1)} cm` : '—'],
      ['Browser runtime', `${plan.elapsedMs.toFixed(1)} ms`]
    ];
    els.planMetrics.innerHTML = metricRows
      .map(([label, value]) => `<div><span>${label}</span><strong>${value}</strong></div>`)
      .join('');
  }

  function updateInspector(x, y) {
    if (!inBounds(x, y)) return;
    state.selectedCell = { x, y };
    const cell = getCell(x, y);
    const metric = metrics[cellIndex(x, y)];
    const evaluation = evaluateNode({ x: x + 0.5, y: y + 0.5 });
    const status = evaluation.valid ? 'TRAVERSABLE' : 'INVALID';
    const reason = evaluation.valid ? 'All active gates passed.' : humanReason(evaluation.reason);

    els.inspector.innerHTML = `
      <span class="planner-panel-label">Terrain Inspector</span>
      <div class="inspector-state" data-state="${evaluation.valid ? 'valid' : 'invalid'}">
        <span>${status}</span><strong>${reason}</strong>
      </div>
      <dl>
        <div><dt>Grid</dt><dd>${x}, ${y}</dd></div>
        <div><dt>Height</dt><dd>${cell.observed ? `${cell.z.toFixed(3)} m` : 'unknown'}</dd></div>
        <div><dt>Slope</dt><dd>${Number.isFinite(metric.slopeDeg) ? `${metric.slopeDeg.toFixed(1)}°` : '—'}</dd></div>
        <div><dt>Roughness</dt><dd>${Number.isFinite(metric.roughnessM) ? `${(metric.roughnessM * 1000).toFixed(1)} mm` : '—'}</dd></div>
        <div><dt>Local step</dt><dd>${Number.isFinite(metric.stepM) ? `${(metric.stepM * 100).toFixed(1)} cm` : '—'}</dd></div>
        <div><dt>Observed support</dt><dd>${(metric.supportRatio * 100).toFixed(0)}%</dd></div>
      </dl>`;
    draw(1);
  }

  function updateLayerLegend() {
    const content = {
      height: ['HEIGHT', 'Low elevation', 'High elevation'],
      slope: ['SLOPE', 'Flat', 'Steep'],
      roughness: ['ROUGHNESS', 'Smooth', 'Rough'],
      step: ['STEP', 'Continuous', 'Discontinuous'],
      traversability: ['TRAVERSABILITY', 'Feasible', 'Risk / invalid']
    }[state.layer];
    els.layerLegend.innerHTML = `
      <span>${content[0]}</span>
      <div class="legend-scale legend-${state.layer}"></div>
      <small>${content[1]}</small><small>${content[2]}</small>`;
  }

  function setTool(tool) {
    state.tool = tool;
    toolButtons.forEach((button) => {
      button.classList.toggle('active', button.dataset.terrainTool === tool);
    });
    const messages = {
      start: 'Click a traversable cell to place START.',
      goal: 'Click a traversable cell to place GOAL.',
      inspect: 'Click any cell to inspect terrain metrics and rejection reason.'
    };
    els.interactionHint.textContent = messages[tool];
  }

  function handleCanvasClick(event) {
    const rect = canvas.getBoundingClientRect();
    const px = clamp(event.clientX - rect.left, 0, rect.width - 1);
    const py = clamp(event.clientY - rect.top, 0, rect.height - 1);
    const x = Math.floor(px / rect.width * COLS);
    const y = ROWS - 1 - Math.floor(py / rect.height * ROWS);

    if (state.tool === 'inspect') {
      updateInspector(x, y);
      return;
    }

    const position = { x: x + 0.5, y: y + 0.5 };
    const evaluation = evaluateNode(position);
    updateInspector(x, y);
    if (!evaluation.valid) {
      els.interactionHint.textContent = `Cannot place ${state.tool.toUpperCase()}: ${humanReason(evaluation.reason)}.`;
      return;
    }

    state[state.tool] = position;
    state.plan = null;
    draw(1);
    setTool(state.tool === 'start' ? 'goal' : 'inspect');
  }

  function reset() {
    state.layer = 'traversability';
    state.maxSlopeDeg = BASELINE.maxSlopeDeg;
    state.maxStepM = BASELINE.maxStepM;
    state.showGraph = true;
    state.showRejected = true;
    state.start = { x: 8.5, y: 38.5 };
    state.goal = { x: 68.5, y: 8.5 };
    state.selectedCell = null;

    els.slopeRange.value = String(BASELINE.maxSlopeDeg);
    els.stepRange.value = String(Math.round(BASELINE.maxStepM * 100));
    els.graphToggle.checked = true;
    els.rejectedToggle.checked = true;
    els.inspector.innerHTML = '<span class="planner-panel-label">Terrain Inspector</span><p>Select <strong>Inspect</strong> and click a cell.</p>';

    layerButtons.forEach((button) => {
      button.classList.toggle('active', button.dataset.terrainLayer === state.layer);
    });
    setTool('inspect');
    updateSliderLabels();
    updateLayerLegend();
    runPlanner({ animate: true });
  }

  function updateSliderLabels() {
    els.slopeValue.textContent = `${state.maxSlopeDeg.toFixed(0)}°`;
    els.stepValue.textContent = `${Math.round(state.maxStepM * 100)} cm`;
  }

  layerButtons.forEach((button) => {
    button.addEventListener('click', () => {
      state.layer = button.dataset.terrainLayer;
      layerButtons.forEach((candidate) => candidate.classList.toggle('active', candidate === button));
      updateLayerLegend();
      draw(1);
    });
  });

  toolButtons.forEach((button) => {
    button.addEventListener('click', () => setTool(button.dataset.terrainTool));
  });

  els.slopeRange.addEventListener('input', () => {
    state.maxSlopeDeg = Number(els.slopeRange.value);
    updateSliderLabels();
    runPlanner({ animate: false });
  });

  els.stepRange.addEventListener('input', () => {
    state.maxStepM = Number(els.stepRange.value) / 100;
    updateSliderLabels();
    runPlanner({ animate: false });
  });

  els.graphToggle.addEventListener('change', () => {
    state.showGraph = els.graphToggle.checked;
    draw(1);
  });

  els.rejectedToggle.addEventListener('change', () => {
    state.showRejected = els.rejectedToggle.checked;
    draw(1);
  });

  els.runButton.addEventListener('click', () => runPlanner({ animate: true }));
  els.resetButton.addEventListener('click', reset);
  canvas.addEventListener('click', handleCanvasClick);
  window.addEventListener('resize', resizeCanvas, { passive: true });

  updateSliderLabels();
  updateLayerLegend();
  setTool('inspect');
  resizeCanvas();
  runPlanner({ animate: true });
})();
