// Global variables
let isConnected = false;
let updateInterval = 1000; // 1 second
let autoUpdateEnabled = true;
let updateTimer = null;
let systemInfo = {};

// DOM elements
const connectionDot = document.getElementById('connection-dot');
const connectionStatus = document.getElementById('connection-status');
const controlMode = document.getElementById('control-mode');
const dutyCycle = document.getElementById('duty-cycle');
const inputVoltage = document.getElementById('input-voltage');
const manualDutyValue = document.getElementById('manual-duty-value');
const dutySlider = document.getElementById('duty-slider');
const targetVoltageValue = document.getElementById('target-voltage-value');
const voltageSlider = document.getElementById('voltage-slider');
const voltageInput = document.getElementById('voltage-input');
const voltageOutput1 = document.getElementById('voltage-output1');
const voltageOutput2 = document.getElementById('voltage-output2');
const voltageError = document.getElementById('voltage-error');
const enableControlBtn = document.getElementById('enable-control-btn');
const applyDutyBtn = document.getElementById('apply-duty-btn');
const systemLog = document.getElementById('system-log');
const updateIntervalSlider = document.getElementById('update-interval');
const updateIntervalValue = document.getElementById('update-interval-value');
const autoUpdateStatus = document.getElementById('auto-update-status');
const errorAlert = document.getElementById('error-alert');
const errorMessage = document.getElementById('error-message');
const uptimeElement = document.getElementById('uptime');
const footerUptime = document.getElementById('footer-uptime');

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    setupEventListeners();
    loadSettings();
    connectToESP();
    startAutoUpdate();
    
    // Display IP address
    const currentUrl = window.location.hostname;
    if (currentUrl) {
        document.getElementById('ip-address').textContent = currentUrl;
    }
});

function setupEventListeners() {
    // Duty slider
    dutySlider.addEventListener('input', function() {
        manualDutyValue.textContent = parseFloat(this.value).toFixed(1);
    });
    
    // Voltage slider
    voltageSlider.addEventListener('input', function() {
        targetVoltageValue.textContent = parseFloat(this.value).toFixed(1);
    });
    
    // Update interval slider
    updateIntervalSlider.addEventListener('input', function() {
        updateInterval = this.value * 1000;
        updateIntervalValue.textContent = parseFloat(this.value).toFixed(1);
        if (autoUpdateEnabled) {
            restartAutoUpdate();
        }
    });
}

function loadSettings() {
    // Load saved settings from localStorage
    const savedDuty = localStorage.getItem('lastDuty');
    const savedVoltage = localStorage.getItem('lastVoltage');
    const savedInterval = localStorage.getItem('updateInterval');
    
    if (savedDuty) {
        dutySlider.value = savedDuty;
        manualDutyValue.textContent = parseFloat(savedDuty).toFixed(1);
    }
    
    if (savedVoltage) {
        voltageSlider.value = savedVoltage;
        targetVoltageValue.textContent = parseFloat(savedVoltage).toFixed(1);
    }
    
    if (savedInterval) {
        updateIntervalSlider.value = savedInterval;
        updateInterval = savedInterval * 1000;
        updateIntervalValue.textContent = parseFloat(savedInterval).toFixed(1);
    }
}

function saveSettings() {
    localStorage.setItem('lastDuty', dutySlider.value);
    localStorage.setItem('lastVoltage', voltageSlider.value);
    localStorage.setItem('updateInterval', updateIntervalSlider.value);
}

// API Communication
async function connectToESP() {
    try {
        // Try to get initial status
        const response = await fetch('/api/status');
        if (response.ok) {
            const data = await response.json();
            updateUI(data);
            isConnected = true;
            connectionDot.className = 'connection-dot connected';
            connectionStatus.textContent = 'Connected';
            addLogEntry('Connected to ESP32', 'info');
        } else {
            throw new Error('Failed to connect');
        }
    } catch (error) {
        isConnected = false;
        connectionDot.className = 'connection-dot disconnected';
        connectionStatus.textContent = 'Disconnected';
        addLogEntry('Failed to connect to ESP32', 'error');
    }
}

async function sendCommand(endpoint, data) {
    if (!isConnected) {
        addLogEntry('Not connected to ESP32', 'error');
        return null;
    }
    
    try {
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(data)
        });
        
        if (response.ok) {
            const result = await response.json();
            addLogEntry(`Command sent: ${JSON.stringify(data)}`, 'info');
            return result;
        } else {
            const error = await response.text();
            addLogEntry(`Command failed: ${error}`, 'error');
            return null;
        }
    } catch (error) {
        addLogEntry(`Network error: ${error.message}`, 'error');
        isConnected = false;
        connectionDot.className = 'connection-dot disconnected';
        connectionStatus.textContent = 'Disconnected';
        return null;
    }
}

async function getSystemStatus() {
    try {
        const response = await fetch('/api/status');
        if (response.ok) {
            const data = await response.json();
            updateUI(data);
            saveSettings();
            return data;
        }
    } catch (error) {
        console.error('Failed to get system status:', error);
    }
    return null;
}

// Control functions
function setDuty(value) {
    dutySlider.value = value;
    manualDutyValue.textContent = value.toFixed(1);
}

async function applyDuty() {
    const duty = parseFloat(dutySlider.value);
    const result = await sendCommand('/api/control', { duty: duty });
    if (result && result.status === 'ok') {
        addLogEntry(`Duty set to ${result.duty}%`, 'info');
    }
}

function setVoltage(value) {
    voltageSlider.value = value;
    targetVoltageValue.textContent = value.toFixed(1);
}

async function applyVoltage() {
    const voltage = parseFloat(voltageSlider.value);
    const result = await sendCommand('/api/control', { voltage: voltage });
    if (result && result.status === 'ok') {
        addLogEntry(`Target voltage set to ${result.voltage}V`, 'info');
    }
}

async function toggleVoltageControl() {
    const currentMode = controlMode.textContent.toLowerCase();
    const newMode = currentMode === 'auto' ? 'manual' : 'auto';
    const result = await sendCommand('/api/control', { mode: newMode });
    if (result && result.status === 'ok') {
        addLogEntry(`Control mode changed to ${newMode}`, 'info');
    }
}

async function updateReadings() {
    await getSystemStatus();
}

async function getSystemInfo() {
    await getSystemStatus();
}

async function emergencyStop() {
    const result = await sendCommand('/api/emergency', {});
    if (result && result.status === 'ok') {
        addLogEntry('EMERGENCY STOP ACTIVATED', 'error');
        alert('Emergency stop activated! System set to minimum duty.');
    }
}

// Auto-update functions
function toggleAutoUpdate() {
    autoUpdateEnabled = !autoUpdateEnabled;
    autoUpdateStatus.textContent = autoUpdateEnabled ? 'ON' : 'OFF';
    
    if (autoUpdateEnabled) {
        startAutoUpdate();
        addLogEntry('Auto-update enabled', 'info');
    } else {
        stopAutoUpdate();
        addLogEntry('Auto-update disabled', 'info');
    }
}

function startAutoUpdate() {
    if (updateTimer) {
        clearInterval(updateTimer);
    }
    
    updateTimer = setInterval(async () => {
        if (isConnected) {
            await getSystemStatus();
        }
    }, updateInterval);
}

function stopAutoUpdate() {
    if (updateTimer) {
        clearInterval(updateTimer);
        updateTimer = null;
    }
}

function restartAutoUpdate() {
    if (autoUpdateEnabled) {
        stopAutoUpdate();
        startAutoUpdate();
    }
}

// UI Update functions
function updateUI(data) {
    systemInfo = data;
    
    // Update connection status
    if (data.system && data.system.running) {
        isConnected = true;
        connectionDot.className = 'connection-dot connected';
        connectionStatus.textContent = 'Connected';
    }
    
    // Update voltages
    if (data.voltages) {
        voltageInput.textContent = `${data.voltages.input.toFixed(2)}V`;
        voltageOutput1.textContent = `${data.voltages.output1.toFixed(2)}V`;
        voltageOutput2.textContent = `${data.voltages.output2.toFixed(2)}V`;
        inputVoltage.textContent = `${data.voltages.input.toFixed(1)}V`;
        
        // Calculate and display error
        const error = data.voltages.output1 - data.voltages.setpoint;
        voltageError.textContent = `${error.toFixed(2)}V`;
        if (Math.abs(error) > 0.5) {
            voltageError.className = 'status-value warning';
        } else {
            voltageError.className = 'status-value';
        }
    }
    
    // Update control status
    if (data.control) {
        dutyCycle.textContent = `${data.control.duty.toFixed(1)}%`;
        controlMode.textContent = data.control.mode.charAt(0).toUpperCase() + data.control.mode.slice(1);
        
        // Update duty slider to match current value
        if (data.control.mode === 'manual') {
            dutySlider.value = data.control.duty;
            manualDutyValue.textContent = data.control.duty.toFixed(1);
        }
        
        // Update voltage slider to match setpoint
        if (data.voltages) {
            voltageSlider.value = data.voltages.setpoint;
            targetVoltageValue.textContent = data.voltages.setpoint.toFixed(1);
        }
        
        // Update control button
        if (data.control.mode === 'auto') {
            enableControlBtn.textContent = 'Disable Voltage Control';
            enableControlBtn.className = 'btn btn-danger';
        } else {
            enableControlBtn.textContent = 'Enable Voltage Control';
            enableControlBtn.className = 'btn btn-success';
        }
    }
    
    // Update system info
    if (data.system) {
        uptimeElement.textContent = formatTime(data.system.uptime);
        footerUptime.textContent = data.system.uptime;
        
        // Show error if exists
        if (data.system.error) {
            errorAlert.style.display = 'flex';
            errorMessage.textContent = data.system.error;
        } else {
            errorAlert.style.display = 'none';
        }
    }
    
    // Update limits and settings
    if (data.limits) {
        document.getElementById('duty-range').textContent = 
            `${data.limits.min_duty}% to ${data.limits.max_duty}%`;
        document.getElementById('max-input').textContent = 
            `${data.limits.max_input}V`;
    }
    
    if (data.settings) {
        document.getElementById('switching-freq').textContent = 
            `${(data.settings.frequency / 1000).toFixed(1)} kHz`;
        document.getElementById('dead-time').textContent = 
            `${data.settings.deadtime} ns`;
        document.getElementById('control-loop').textContent = 
            `${CONTROL_LOOP_FREQ} Hz`;
        document.getElementById('pid-kp').textContent = 
            data.settings.pid_kp.toFixed(3);
        document.getElementById('pid-ki').textContent = 
            data.settings.pid_ki.toFixed(3);
        document.getElementById('pid-kd').textContent = 
            data.settings.pid_kd.toFixed(3);
    }
}

function formatTime(seconds) {
    if (seconds < 60) {
        return `${seconds}s`;
    } else if (seconds < 3600) {
        const minutes = Math.floor(seconds / 60);
        const secs = seconds % 60;
        return `${minutes}m ${secs}s`;
    } else {
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        return `${hours}h ${minutes}m`;
    }
}

// Log functions
function addLogEntry(message, type = 'info') {
    const now = new Date();
    const timeString = `[${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}]`;
    
    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry';
    
    const timeSpan = document.createElement('span');
    timeSpan.className = 'log-time';
    timeSpan.textContent = timeString;
    
    const messageSpan = document.createElement('span');
    messageSpan.className = `log-${type}`;
    messageSpan.textContent = ` ${message}`;
    
    logEntry.appendChild(timeSpan);
    logEntry.appendChild(messageSpan);
    
    systemLog.appendChild(logEntry);
    
    // Scroll to bottom
    systemLog.scrollTop = systemLog.scrollHeight;
    
    // Limit log entries
    const entries = systemLog.querySelectorAll('.log-entry');
    if (entries.length > 50) {
        entries[0].remove();
    }
}

function clearLog() {
    systemLog.innerHTML = '<div class="log-entry"><span class="log-time">[00:00:00]</span><span class="log-info">Log cleared</span></div>';
}

// Make functions available globally
window.setDuty = setDuty;
window.applyDuty = applyDuty;
window.setVoltage = setVoltage;
window.applyVoltage = applyVoltage;
window.toggleVoltageControl = toggleVoltageControl;
window.updateReadings = updateReadings;
window.getSystemInfo = getSystemInfo;
window.emergencyStop = emergencyStop;
window.toggleAutoUpdate = toggleAutoUpdate;
window.clearLog = clearLog;