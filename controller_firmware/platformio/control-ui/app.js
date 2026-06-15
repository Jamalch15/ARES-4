const state = {
  port: null,
  reader: null,
  writer: null,
  keepReading: false,
  direction: "FWD",
  microstep: 8,
  speedTimer: null,
  rampTimer: null,
  gearTimer: null,
  servoAngleTimer: null,
  stepperPinsDirty: false,
  servoSetupDirty: false,
  servoRange: 270,
};

const els = {
  connectButton: document.querySelector("#connectButton"),
  disconnectButton: document.querySelector("#disconnectButton"),
  connectionDot: document.querySelector("#connectionDot"),
  connectionLabel: document.querySelector("#connectionLabel"),
  tabs: document.querySelectorAll("[data-tab]"),
  tabPanels: document.querySelectorAll(".tab-panel"),
  rpmSlider: document.querySelector("#rpmSlider"),
  rpmOutput: document.querySelector("#rpmOutput"),
  rampSlider: document.querySelector("#rampSlider"),
  rampOutput: document.querySelector("#rampOutput"),
  gearRatioInput: document.querySelector("#gearRatioInput"),
  stepperDirPinInput: document.querySelector("#stepperDirPinInput"),
  stepperStepPinInput: document.querySelector("#stepperStepPinInput"),
  stepperM0PinInput: document.querySelector("#stepperM0PinInput"),
  stepperM1PinInput: document.querySelector("#stepperM1PinInput"),
  stepperM2PinInput: document.querySelector("#stepperM2PinInput"),
  stepperEnablePinInput: document.querySelector("#stepperEnablePinInput"),
  applyStepperPinsButton: document.querySelector("#applyStepperPinsButton"),
  directionGroup: document.querySelector("#directionGroup"),
  microstepGroup: document.querySelector("#microstepGroup"),
  runButton: document.querySelector("#runButton"),
  stopButton: document.querySelector("#stopButton"),
  estopButton: document.querySelector("#estopButton"),
  degreesInput: document.querySelector("#degreesInput"),
  rotationsInput: document.querySelector("#rotationsInput"),
  moveDegreesButton: document.querySelector("#moveDegreesButton"),
  moveRotationsButton: document.querySelector("#moveRotationsButton"),
  manualCommand: document.querySelector("#manualCommand"),
  sendManualButton: document.querySelector("#sendManualButton"),
  clearConsoleButton: document.querySelector("#clearConsoleButton"),
  consoleLog: document.querySelector("#consoleLog"),
  rpmValue: document.querySelector("#rpmValue"),
  modeValue: document.querySelector("#modeValue"),
  dirValue: document.querySelector("#dirValue"),
  targetRpmValue: document.querySelector("#targetRpmValue"),
  microstepValue: document.querySelector("#microstepValue"),
  gearRatioValue: document.querySelector("#gearRatioValue"),
  remainingValue: document.querySelector("#remainingValue"),
  directionNeedle: document.querySelector("#directionNeedle"),
  servoPinInput: document.querySelector("#servoPinInput"),
  servoFrequencyInput: document.querySelector("#servoFrequencyInput"),
  servoMinPulseInput: document.querySelector("#servoMinPulseInput"),
  servoMaxPulseInput: document.querySelector("#servoMaxPulseInput"),
  servoRangeInput: document.querySelector("#servoRangeInput"),
  servoMoveMsInput: document.querySelector("#servoMoveMsInput"),
  servoAttachButton: document.querySelector("#servoAttachButton"),
  servoTestButton: document.querySelector("#servoTestButton"),
  servoDetachButton: document.querySelector("#servoDetachButton"),
  servoAngleSlider: document.querySelector("#servoAngleSlider"),
  servoAngleOutput: document.querySelector("#servoAngleOutput"),
  servoPulseInput: document.querySelector("#servoPulseInput"),
  servoPulseButton: document.querySelector("#servoPulseButton"),
  servoSweepMinInput: document.querySelector("#servoSweepMinInput"),
  servoSweepMaxInput: document.querySelector("#servoSweepMaxInput"),
  servoSweepPeriodInput: document.querySelector("#servoSweepPeriodInput"),
  servoSweepButton: document.querySelector("#servoSweepButton"),
  servoStopButton: document.querySelector("#servoStopButton"),
  servoAngleValue: document.querySelector("#servoAngleValue"),
  servoAttachedValue: document.querySelector("#servoAttachedValue"),
  servoPinValue: document.querySelector("#servoPinValue"),
  servoPulseValue: document.querySelector("#servoPulseValue"),
  servoDutyValue: document.querySelector("#servoDutyValue"),
  servoRangeValue: document.querySelector("#servoRangeValue"),
  servoSweepValue: document.querySelector("#servoSweepValue"),
};

function logLine(text, source = "ui") {
  const timestamp = new Date().toLocaleTimeString();
  els.consoleLog.textContent += `[${timestamp}] ${source}> ${text}\n`;
  els.consoleLog.scrollTop = els.consoleLog.scrollHeight;
}

function setConnected(connected) {
  els.connectionDot.classList.toggle("connected", connected);
  els.connectionLabel.textContent = connected ? "Connected" : "Disconnected";
  els.connectButton.disabled = connected;
  els.disconnectButton.disabled = !connected;
}

function markUserEdited(input) {
  if (input) input.dataset.userEdited = "1";
}

function clearUserEdited(...inputs) {
  inputs.forEach((input) => {
    if (input) delete input.dataset.userEdited;
  });
}

function canSyncInput(input) {
  return input && input.dataset.userEdited !== "1" && document.activeElement !== input;
}

function syncInput(input, value) {
  if (canSyncInput(input)) input.value = value;
}

function selectedServoRange() {
  return Math.min(Math.max(Number(els.servoRangeInput.value) || 270, 1), 360);
}

function selectedServoMoveMs() {
  return Math.min(Math.max(Number(els.servoMoveMsInput.value) || 0, 0), 10000);
}

function applyServoRangeToControls(range) {
  state.servoRange = Math.min(Math.max(Number(range) || 270, 1), 360);
  const max = String(Math.round(state.servoRange));
  els.servoAngleSlider.max = max;
  els.servoSweepMinInput.max = max;
  els.servoSweepMaxInput.max = max;

  if (Number(els.servoAngleSlider.value) > state.servoRange) {
    els.servoAngleSlider.value = max;
  }

  if (canSyncInput(els.servoSweepMaxInput) && Number(els.servoSweepMaxInput.value) > state.servoRange) {
    els.servoSweepMaxInput.value = max;
  }

  els.servoRangeValue.textContent = state.servoRange.toFixed(0);
}

function selectedStepperValues() {
  return {
    rpm: Number(els.rpmSlider.value),
    ramp: Number(els.rampSlider.value),
    direction: state.direction,
    microstep: state.microstep,
    gearRatio: Number(els.gearRatioInput.value),
  };
}

function selectedStepperPins() {
  return {
    dir: Number(els.stepperDirPinInput.value),
    step: Number(els.stepperStepPinInput.value),
    m0: Number(els.stepperM0PinInput.value),
    m1: Number(els.stepperM1PinInput.value),
    m2: Number(els.stepperM2PinInput.value),
    enable: Number(els.stepperEnablePinInput.value),
  };
}

function selectedServoSetup() {
  return {
    pin: Number(els.servoPinInput.value),
    minUs: Number(els.servoMinPulseInput.value),
    maxUs: Number(els.servoMaxPulseInput.value),
    frequency: Number(els.servoFrequencyInput.value),
    range: selectedServoRange(),
  };
}

async function sendCommand(command) {
  if (!state.writer) {
    logLine("Not connected");
    return;
  }

  const encoded = new TextEncoder().encode(`${command}\n`);
  await state.writer.write(encoded);
  logLine(command, "tx");
}

async function connectSerial() {
  if (!("serial" in navigator)) {
    logLine("Web Serial is not available. Use Chrome or Edge on localhost.");
    return;
  }

  state.port = await navigator.serial.requestPort();
  await state.port.open({ baudRate: 115200 });
  state.writer = state.port.writable.getWriter();
  state.keepReading = true;
  setConnected(true);
  logLine("Serial port opened");
  readLoop();
  await sendCommand("STATUS");
}

async function disconnectSerial() {
  state.keepReading = false;

  if (state.reader) {
    await state.reader.cancel();
    state.reader.releaseLock();
    state.reader = null;
  }

  if (state.writer) {
    state.writer.releaseLock();
    state.writer = null;
  }

  if (state.port) {
    await state.port.close();
    state.port = null;
  }

  setConnected(false);
  logLine("Serial port closed");
}

async function readLoop() {
  const decoder = new TextDecoder();
  let pending = "";

  while (state.port?.readable && state.keepReading) {
    state.reader = state.port.readable.getReader();
    try {
      while (state.keepReading) {
        const { value, done } = await state.reader.read();
        if (done) break;
        pending += decoder.decode(value, { stream: true });
        const lines = pending.split(/\r?\n/);
        pending = lines.pop() ?? "";
        for (const line of lines) {
          if (line.trim()) {
            logLine(line.trim(), "rx");
            updateStatusFromLine(line.trim());
          }
        }
      }
    } catch (error) {
      logLine(`Read error: ${error.message}`);
    } finally {
      state.reader.releaseLock();
      state.reader = null;
    }
  }
}

function parseKeyValues(line, prefix) {
  return Object.fromEntries(
    line
      .replace(new RegExp(`^${prefix}\\s+`), "")
      .split(/\s+/)
      .map((part) => part.split("="))
      .filter(([key, value]) => key && value),
  );
}

function updateStatusFromLine(line) {
  if (line.startsWith("STEPPER_STATUS")) {
    updateStepperStatus(parseKeyValues(line, "STEPPER_STATUS"));
  }

  if (line.startsWith("SERVO_STATUS")) {
    updateServoStatus(parseKeyValues(line, "SERVO_STATUS"));
  }
}

function updateStepperStatus(values) {
  if (values.rpm) {
    const rpm = Number(values.rpm);
    els.rpmValue.textContent = rpm.toFixed(1);
    els.directionNeedle.style.transform = `translate(-50%, -100%) rotate(${Math.min(rpm, 300) * 0.9}deg)`;
  }
  if (values.mode) els.modeValue.textContent = values.mode;
  if (values.dir) {
    els.dirValue.textContent = values.dir;
    if (values.mode === "stop" && document.activeElement?.closest("#directionGroup") == null) {
      selectDirection(values.dir);
    }
  }
  if (values.target_rpm) els.targetRpmValue.textContent = Number(values.target_rpm).toFixed(0);
  if (values.microsteps) els.microstepValue.textContent = values.microsteps;
  if (values.gear_ratio) {
    els.gearRatioValue.textContent = Number(values.gear_ratio).toFixed(2);
    syncInput(els.gearRatioInput, Number(values.gear_ratio).toString());
  }
  if (values.remaining) els.remainingValue.textContent = values.remaining;
  if (values.pins) {
    const [dir, step, m0, m1, m2, enable] = values.pins.split(",");
    syncInput(els.stepperDirPinInput, dir);
    syncInput(els.stepperStepPinInput, step);
    syncInput(els.stepperM0PinInput, m0);
    syncInput(els.stepperM1PinInput, m1);
    syncInput(els.stepperM2PinInput, m2);
    syncInput(els.stepperEnablePinInput, enable);
  }
}

function updateServoStatus(values) {
  if (values.attached) els.servoAttachedValue.textContent = values.attached;
  if (values.pin) {
    els.servoPinValue.textContent = values.pin;
    syncInput(els.servoPinInput, values.pin);
  }
  if (values.angle) {
    const angle = Number(values.angle);
    els.servoAngleValue.textContent = angle.toFixed(1);
    if (canSyncInput(els.servoAngleSlider)) {
      els.servoAngleSlider.value = String(Math.round(angle));
      els.servoAngleOutput.textContent = `${Math.round(angle)} deg`;
    }
  }
  if (values.pulse_us) {
    els.servoPulseValue.textContent = values.pulse_us;
    syncInput(els.servoPulseInput, values.pulse_us);
  }
  if (values.duty) els.servoDutyValue.textContent = values.duty;
  if (values.min_us) syncInput(els.servoMinPulseInput, values.min_us);
  if (values.max_us) syncInput(els.servoMaxPulseInput, values.max_us);
  if (values.freq) syncInput(els.servoFrequencyInput, values.freq);
  if (values.range) {
    const range = Number(values.range);
    if (canSyncInput(els.servoRangeInput)) {
      applyServoRangeToControls(range);
      els.servoRangeInput.value = range.toFixed(0);
    }
  }
  if (values.sweep) els.servoSweepValue.textContent = values.sweep === "1" ? "on" : "off";
  if (values.sweep_min) syncInput(els.servoSweepMinInput, values.sweep_min);
  if (values.sweep_max) syncInput(els.servoSweepMaxInput, values.sweep_max);
  if (values.sweep_period_ms) syncInput(els.servoSweepPeriodInput, values.sweep_period_ms);
  if (values.move_ms) syncInput(els.servoMoveMsInput, values.move_ms);
}

function updateOutputs() {
  els.rpmOutput.textContent = `${els.rpmSlider.value} rpm`;
  els.rampOutput.textContent = `${els.rampSlider.value} ms`;
  els.targetRpmValue.textContent = els.rpmSlider.value;
  els.gearRatioValue.textContent = Number(els.gearRatioInput.value).toFixed(2);
  els.servoAngleOutput.textContent = `${els.servoAngleSlider.value} deg`;
  els.servoRangeValue.textContent = selectedServoRange().toFixed(0);
}

function sendDebounced(kind) {
  if (!state.writer) return;

  if (kind === "speed") {
    clearTimeout(state.speedTimer);
    state.speedTimer = setTimeout(() => {
      sendCommand(`SPEED ${Number(els.rpmSlider.value)}`);
    }, 180);
  }

  if (kind === "ramp") {
    clearTimeout(state.rampTimer);
    state.rampTimer = setTimeout(() => {
      sendCommand(`RAMP ${Number(els.rampSlider.value)}`);
    }, 180);
  }

  if (kind === "gear") {
    clearTimeout(state.gearTimer);
    state.gearTimer = setTimeout(() => {
      sendCommand(`GEAR ${Number(els.gearRatioInput.value)}`).then(() => {
        clearUserEdited(els.gearRatioInput);
      });
    }, 180);
  }

  if (kind === "servoAngle") {
    clearTimeout(state.servoAngleTimer);
    state.servoAngleTimer = setTimeout(() => {
      sendCommand(`SERVOANGLE ${Number(els.servoAngleSlider.value)} ${selectedServoMoveMs()}`).then(() => {
        clearUserEdited(els.servoAngleSlider, els.servoMoveMsInput);
      });
    }, 120);
  }
}

function selectDirection(direction) {
  state.direction = direction;
  document.querySelectorAll("[data-direction]").forEach((button) => {
    button.classList.toggle("selected", button.dataset.direction === direction);
  });
}

function selectMicrostep(microstep) {
  state.microstep = microstep;
  document.querySelectorAll("[data-microstep]").forEach((button) => {
    button.classList.toggle("selected", Number(button.dataset.microstep) === microstep);
  });
}

async function applyStepperPins() {
  const pins = selectedStepperPins();
  await sendCommand(
    `STEPPERPINS ${pins.dir} ${pins.step} ${pins.m0} ${pins.m1} ${pins.m2} ${pins.enable}`,
  );
  state.stepperPinsDirty = false;
  clearUserEdited(
    els.stepperDirPinInput,
    els.stepperStepPinInput,
    els.stepperM0PinInput,
    els.stepperM1PinInput,
    els.stepperM2PinInput,
    els.stepperEnablePinInput,
  );
}

async function attachServo() {
  const setup = selectedServoSetup();
  await sendCommand(`SERVOATTACH ${setup.pin} ${setup.minUs} ${setup.maxUs} ${setup.frequency}`);
  await sendCommand(`SERVORANGE ${setup.range}`);
  state.servoSetupDirty = false;
  clearUserEdited(
    els.servoPinInput,
    els.servoFrequencyInput,
    els.servoMinPulseInput,
    els.servoMaxPulseInput,
    els.servoRangeInput,
  );
}

function wait(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function runServoTest() {
  await attachServo();
  const range = selectedServoRange();
  const center = range / 2;
  const moveMs = selectedServoMoveMs();
  const waitMs = Math.max(700, moveMs + 200);
  await sendCommand(`SERVOANGLE ${center} ${moveMs}`);
  await wait(waitMs);
  await sendCommand(`SERVOANGLE 0 ${moveMs}`);
  await wait(waitMs);
  await sendCommand(`SERVOANGLE ${range} ${moveMs}`);
  await wait(waitMs);
  await sendCommand(`SERVOANGLE ${center} ${moveMs}`);
}

els.tabs.forEach((tab) => {
  tab.addEventListener("click", () => {
    const activeTab = tab.dataset.tab;
    els.tabs.forEach((candidate) => candidate.classList.toggle("selected", candidate === tab));
    els.tabPanels.forEach((panel) => {
      panel.classList.toggle("active", panel.id === `${activeTab}Tab`);
    });
  });
});

els.connectButton.addEventListener("click", () => connectSerial().catch((error) => logLine(error.message)));
els.disconnectButton.addEventListener("click", () => disconnectSerial().catch((error) => logLine(error.message)));
els.rpmSlider.addEventListener("input", () => {
  updateOutputs();
  sendDebounced("speed");
});
els.rampSlider.addEventListener("input", () => {
  updateOutputs();
  sendDebounced("ramp");
});
els.gearRatioInput.addEventListener("input", () => {
  markUserEdited(els.gearRatioInput);
  updateOutputs();
  sendDebounced("gear");
});

[
  els.stepperDirPinInput,
  els.stepperStepPinInput,
  els.stepperM0PinInput,
  els.stepperM1PinInput,
  els.stepperM2PinInput,
  els.stepperEnablePinInput,
].forEach((input) => {
  input.addEventListener("input", () => {
    state.stepperPinsDirty = true;
    markUserEdited(input);
  });
});

[
  els.servoPinInput,
  els.servoFrequencyInput,
  els.servoMinPulseInput,
  els.servoMaxPulseInput,
  els.servoRangeInput,
  els.servoMoveMsInput,
].forEach((input) => {
  input.addEventListener("input", () => {
    state.servoSetupDirty = true;
    markUserEdited(input);
    if (input === els.servoRangeInput) {
      applyServoRangeToControls(selectedServoRange());
      updateOutputs();
    }
  });
});

els.directionGroup.addEventListener("click", async (event) => {
  const button = event.target.closest("[data-direction]");
  if (!button) return;
  selectDirection(button.dataset.direction);
  await sendCommand(`DIR ${state.direction}`);
});

els.microstepGroup.addEventListener("click", async (event) => {
  const button = event.target.closest("[data-microstep]");
  if (!button) return;
  selectMicrostep(Number(button.dataset.microstep));
  await sendCommand(`MICROSTEP ${state.microstep}`);
});

els.applyStepperPinsButton.addEventListener("click", applyStepperPins);

els.runButton.addEventListener("click", async () => {
  const { rpm, ramp, direction, microstep, gearRatio } = selectedStepperValues();
  await applyStepperPins();
  await sendCommand(`RUN ${direction} ${rpm} ${microstep} ${ramp} ${gearRatio}`);
});

els.stopButton.addEventListener("click", () => sendCommand("STOP"));
els.estopButton.addEventListener("click", () => sendCommand("ESTOP"));

els.moveDegreesButton.addEventListener("click", async () => {
  const { rpm, ramp, direction, microstep, gearRatio } = selectedStepperValues();
  const degrees = Number(els.degreesInput.value);
  await applyStepperPins();
  await sendCommand(`DIR ${direction}`);
  await sendCommand(`MOVE ${direction} ${degrees} ${rpm} ${microstep} ${ramp} ${gearRatio}`);
});

els.moveRotationsButton.addEventListener("click", async () => {
  const { rpm, ramp, direction, microstep, gearRatio } = selectedStepperValues();
  const degrees = Number(els.rotationsInput.value) * 360;
  await applyStepperPins();
  await sendCommand(`DIR ${direction}`);
  await sendCommand(`MOVE ${direction} ${degrees} ${rpm} ${microstep} ${ramp} ${gearRatio}`);
});

els.servoAttachButton.addEventListener("click", attachServo);
els.servoTestButton.addEventListener("click", () => runServoTest().catch((error) => logLine(error.message)));
els.servoDetachButton.addEventListener("click", () => sendCommand("SERVODETACH"));
els.servoAngleSlider.addEventListener("input", () => {
  markUserEdited(els.servoAngleSlider);
  updateOutputs();
  sendDebounced("servoAngle");
});
els.servoPulseButton.addEventListener("click", () => {
  sendCommand(`SERVOPULSE ${Number(els.servoPulseInput.value)}`).then(() => {
    clearUserEdited(els.servoPulseInput);
  });
});
els.servoPulseInput.addEventListener("input", () => markUserEdited(els.servoPulseInput));
els.servoSweepButton.addEventListener("click", async () => {
  await attachServo();
  await sendCommand(
    `SERVOSWEEP ${Number(els.servoSweepMinInput.value)} ${Number(els.servoSweepMaxInput.value)} ${Number(els.servoSweepPeriodInput.value)}`,
  );
  clearUserEdited(els.servoSweepMinInput, els.servoSweepMaxInput, els.servoSweepPeriodInput);
});
els.servoStopButton.addEventListener("click", () => sendCommand("SERVOSTOP"));

[els.servoSweepMinInput, els.servoSweepMaxInput, els.servoSweepPeriodInput].forEach((input) => {
  input.addEventListener("input", () => markUserEdited(input));
});

els.sendManualButton.addEventListener("click", () => sendCommand(els.manualCommand.value.trim()));
els.manualCommand.addEventListener("keydown", (event) => {
  if (event.key === "Enter") sendCommand(els.manualCommand.value.trim());
});
els.clearConsoleButton.addEventListener("click", () => {
  els.consoleLog.textContent = "";
});

updateOutputs();
setConnected(false);
