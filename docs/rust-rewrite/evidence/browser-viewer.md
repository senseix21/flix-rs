# Browser viewer and keyboard controls

Date: 2026-10-05. Uncommitted local development source. [Input hashes](browser-viewer-inputs.json).

The viewer is running at **http://127.0.0.1:8765**. It draws a stylized drone using the actual Gazebo model's world position and quaternion, via a read-only `gz topic` subscriber. It does not feed ground truth to the flight estimator. Rust telemetry is observed in the existing UDP relay and written outside the flight owner at at most 20 Hz; the HTTP viewer does not send commands or claim the control endpoint.

The page has orbit/zoom, isometric/top/reset views, world height/position/orientation, controller arm/fault state, four motor bars, live/stale indicators and a responsive layout. Assets and HTTP server use only local files and Python's standard library. The server binds host loopback; its HTTP API accepts reads only. Observation file errors do not interrupt the simulation transport. Pose subscriptions run for 60 seconds between reconnects and stale data is explicitly marked.

## Verified

* Chromium browser: live pose, disarmed state, actual advancing timestamps, camera/view changes, no horizontal overflow at 390 px, stale motor values hidden, no page exceptions and POST rejected. [Browser result](browser-viewer-check.json), [screenshot](browser-viewer.png).
* Python: 12 tests pass, including existing client/log tests, parent-model pose selection, malformed/nonfinite poses, motor payload validation, observer I/O failure and keyboard zero-thrust arming/stick expiry/disarm. [Raw result](browser-viewer-python-tests.txt).
* Real interactive keyboard session: `a` acknowledged arming; consecutive `ww` inputs advanced requested thrust to 4%; the viewer observed armed state and positive motor outputs. [Armed snapshot](keyboard-armed-state.json). `Space` acknowledged disarm, followed by `x` exiting; the browser check subsequently observed disarmed state. The original mixer idle threshold explains motor outputs differing from the small thrust request.
* Python compilation, JavaScript syntax and whitespace checks pass. No Rust or SDK changes were made in this viewer delivery; previous target evidence remains separate.

The initial keyboard attempt sent a command before transport warmup and timed out. The final client uses a bounded parameter-read connection check before commands. Terminal input uses unbuffered byte reads to service consecutive keys without relying on another keypress.

## Use

```sh
cd /Users/abuhamzah/Dev/erik/flix-rust
# If not already running:
python3 tools/gazebo-viewer.py
# In a separate interactive terminal:
PYTHONPATH=tools python3 tools/gazebo-control.py
```

The viewer process is already running for the delivered session. Open the URL and start only the keyboard client. A arms; Space disarms; W/S changes thrust by 2%; I/K pitches; J/L rolls; Q/E yaws; C explicitly requests stopped fault recovery; X/Ctrl+C exits and disarms. Stick/yaw inputs return to center after 150 ms without repeated keys. Only use one control client at a time. All client control goes to the loopback simulation and remains subject to the Rust owner policy.

Gazebo and the viewer are left running, with the drone disarmed. This is not target hardware or flight qualification.

## Firefox disconnect correction

The reported page displayed “Viewer disconnected”. At investigation, the localhost HTTP API and Gazebo both returned current pose/telemetry. The exact initiating failure in the user’s Firefox session was not captured.

The polling loop now uses `AbortController` with a cleared five-second timer instead of requiring `AbortSignal.timeout()`, retains the last pose during network errors, hides unavailable motor/state data and retries automatically. Asset URLs were versioned to reload the updated script.

Playwright Firefox **148.0.2** verified live data, an injected fetch failure, preserved last pose, hidden unavailable telemetry and automatic reconnection with no page exceptions. [Firefox result](viewer-firefox-reconnect.json). Chromium also verified operation with `AbortSignal.timeout` explicitly absent, then failure/reconnection. [Compatibility result](viewer-compatible-reconnect.json), [changed input hashes](viewer-reconnect-inputs.json). The installed desktop Firefox is 157.0; automated tests used the separate Playwright browser. No physical or Rust flight behaviour was changed.

## Browser flight controls

The user clarified that flight controls were wanted. The original read-only delivery is superseded by explicit simulation controls. Default startup now creates one local pyflix client; `--view-only` preserves the terminal-client workflow.

The page provides Arm, Disarm, Clear fault, a bounded thrust slider, momentary pitch/roll/yaw buttons and keyboard input. A token and matching loopback Origin are required for POST. Values are validated atomically; eight queued commands are bounded and disarm uses a reserved flag. Arming remains explicit and starts at zero thrust. Browser input must arrive within 500 ms; expiry requests disarm. Blur, hidden page, close and viewer polling failure also request disarm. Reconnection creates a stopped client and never rearms automatically. The portable Rust owner still enforces source/sensor/arming and motor authority.

Firefox tests used actual Gazebo and Rust outputs: Arm acknowledged, thrust produced positive motor telemetry, direction button release returned its setpoint to zero, Disarm returned all motors to zero, a request without the session credentials was rejected, and blocked input led to zero motors. A brief 0.6-thrust virtual test increased actual world height by **0.026 m**, then disarmed and explicitly cleared faults after settling. [Browser result](browser-flight-controls.json), [screenshot](browser-flight-controls.png). Camera buttons additionally passed actual canvas-pixel change/restore and selected-button feedback checks in Firefox. [Camera result](camera-controls-firefox.json).

Fourteen Python tests pass, including control bounds/nonfinite/boolean rejection, atomic input validation and reserved disarm with a full command queue. [Raw result](browser-flight-python-tests.txt), [input hashes](browser-flight-inputs.json). Syntax and whitespace checks pass. This is simulator evidence, not a measured hardware stop deadline or flight qualification. Gazebo and the browser server are left running, disarmed, with zero motor output.

## Inverted drone recovery (2026-10-05)

Observed an actual inverted Gazebo world pose with zero motors and a latched
controller fault. Disarmed and restarted `flix-rust-gazebo`; the drone returned
to its initial upright pose, disarmed with zero motor outputs and no fault.
The native `gz world -w default -r` command hung and was cleaned up by restart.

Added **Reset drone**, distinct from the camera's **Reset view**. The authenticated
local HTTP action runs on the existing command owner: disarm, revoke inputs,
mark unavailable, cancel queued commands, and restart the configured simulation
container with a 12-second subprocess timeout. The entire world and controller
restart; reconnect requires explicit arming. No firmware tuning was changed.
Queued arm requests predating disarm are rejected. Clear fault remains a
controller command and cannot restore an inverted physical model.

Validation:

- `PYTHONPATH=tools python3 -m unittest discover -s tools/tests -v`: 15 passed,
  including restart timeout, queued arm cancellation, and disarmed recovery.
- `node /private/tmp/flix-browser-reset-check.cjs`: Firefox test against actual
  Gazebo passed. Armed with positive motor output, clicked Reset drone, verified
  browser thrust zero, fresh upright pose at origin, fresh Rust telemetry with
  zero motors, disarmed, and no fault. No page errors. Raw results and screenshot
  are `browser-reset.json` and `browser-reset.png`.

This verifies simulation recovery, not stable flight or physical hardware.
