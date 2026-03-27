const scoreValue = document.getElementById('score-value');
const blockCountValue = document.getElementById('block-count-value');
const stateValue = document.getElementById('state-value');
const motionValue = document.getElementById('motion-value');
const selectionValue = document.getElementById('selection-value');
const statusRibbon = document.getElementById('status-ribbon');
const previewMessage = document.getElementById('preview-message');
const bridgeStatus = document.getElementById('bridge-status');
const bridgeMessage = document.getElementById('bridge-message');
const blockList = document.getElementById('block-list');
const processList = document.getElementById('process-list');
const logList = document.getElementById('log-list');

async function postJson(url) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: '{}'
  });
  const data = await response.json();
  if (!response.ok) {
    throw new Error(data.message || `HTTP ${response.status}`);
  }
  return data;
}

function setButtonHandlers() {
  document.getElementById('start-btn').addEventListener('click', async () => {
    try {
      const result = await postJson('/api/start');
      previewMessage.textContent = result.message;
      pollStatus();
    } catch (error) {
      previewMessage.textContent = `Start failed: ${error.message}`;
    }
  });

  document.getElementById('next-btn').addEventListener('click', () => {
    previewMessage.textContent = 'Next Round is still not wired. Keep round control inside ROS for now.';
  });

  document.getElementById('end-btn').addEventListener('click', async () => {
    try {
      const result = await postJson('/api/stop');
      previewMessage.textContent = result.message;
      pollStatus();
    } catch (error) {
      previewMessage.textContent = `End Game failed: ${error.message}`;
    }
  });
}

function renderBlocks(blocks) {
  blockList.innerHTML = '';
  if (!blocks || blocks.length === 0) {
    const item = document.createElement('li');
    item.textContent = 'No detected blocks.';
    blockList.appendChild(item);
    return;
  }

  blocks.forEach((block) => {
    const item = document.createElement('li');
    item.innerHTML = `<strong>${block.color}</strong> · id ${block.id} · (${block.position.x}, ${block.position.y}, ${block.position.z}) · conf ${block.confidence}`;
    blockList.appendChild(item);
  });
}

function renderProcesses(processStatus) {
  processList.innerHTML = '';
  const entries = Object.entries(processStatus || {});
  if (entries.length === 0) {
    const item = document.createElement('li');
    item.textContent = 'No managed process data.';
    processList.appendChild(item);
    return;
  }

  entries.forEach(([name, info]) => {
    const item = document.createElement('li');
    item.innerHTML = `<strong>${name}</strong> · ${info.running ? 'running' : 'stopped'}${info.pid ? ` · pid ${info.pid}` : ''}`;
    processList.appendChild(item);
  });
}

function renderLog(entries) {
  logList.innerHTML = '';
  if (!entries || entries.length === 0) {
    const item = document.createElement('li');
    item.textContent = 'No bridge log entries.';
    logList.appendChild(item);
    return;
  }

  entries.forEach((entry) => {
    const item = document.createElement('li');
    item.textContent = `${entry.time} - ${entry.message}`;
    logList.appendChild(item);
  });
}

function renderStatus(data) {
  scoreValue.textContent = data.score === null ? '-' : String(data.score);
  blockCountValue.textContent = String(data.detected_block_count || 0);
  stateValue.textContent = data.game_state || 'UNKNOWN';
  motionValue.textContent = data.motion_status || 'UNKNOWN';
  bridgeStatus.textContent = data.bridge_status || 'UNKNOWN';
  bridgeMessage.textContent = data.bridge_message || 'No bridge message.';

  if (data.player_selection) {
    selectionValue.textContent = `${data.player_selection.color} · id ${data.player_selection.block_id} · ${data.player_selection.selection_type} · conf ${data.player_selection.confidence}`;
  } else {
    selectionValue.textContent = 'None';
  }

  statusRibbon.textContent = data.last_update
    ? `Last bridge update: ${data.last_update}`
    : 'Waiting for live ROS data.';

  renderBlocks(data.detected_blocks || []);
  renderProcesses(data.process_status || {});
  renderLog(data.log || []);
}

async function pollStatus() {
  try {
    const response = await fetch('/api/status', { cache: 'no-store' });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const data = await response.json();
    renderStatus(data);
  } catch (error) {
    bridgeStatus.textContent = 'disconnected';
    bridgeMessage.textContent = `Could not read /api/status: ${error.message}`;
    statusRibbon.textContent = 'UI is up, but the bridge endpoint is not responding.';
  }
}

setButtonHandlers();
pollStatus();
setInterval(pollStatus, 1000);
