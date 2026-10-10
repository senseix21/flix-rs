//! Single-owner orchestration. Networking only deposits bounded validated messages.
use flight_config::{PARAMETERS, Settings};
use flight_core::log::{Entry, Log};
use flight_core::{Flight, Sensor, State, Tick, Vec3};
use flight_protocol::ingress::Ingress;
use flight_protocol::wire::{self, MAX_FRAME, Policy};
use flight_protocol::{Console, Event};
#[derive(Clone, Copy)]
pub struct Reply {
    pub bytes: [u8; MAX_FRAME],
    pub len: usize,
}
impl Default for Reply {
    fn default() -> Self {
        Self {
            bytes: [0; MAX_FRAME],
            len: 0,
        }
    }
}
#[derive(Default)]
pub struct Counters {
    pub rejected: u64,
    pub reply_overflow: u64,
    pub overruns: u64,
    pub maximum_tick_us: u64,
    pub maximum_jitter_us: u64,
    /// Samples with a register at its endpoint, per axis (gyro x/y/z, accel x/y/z).
    pub imu_saturated_samples: [u64; 6],
}
pub struct Owner {
    pub flight: Flight,
    pub settings: Settings,
    pub ingress: Ingress,
    pub counters: Counters,
    pub timing: crate::metrics::Timing,
    log: Log<1000>,
    replies: [Option<Reply>; 4],
    sequence: u8,
    parameter_cursor: Option<usize>,
    log_cursor: Option<(usize, usize)>,
    metrics_transfer: Option<crate::metrics::Transfer>,
    last_log: Tick,
    last_attitude: Tick,
    last_heartbeat: Tick,
    last_motors: Tick,
    last_imu: Tick,
    last_rc: Tick,
    rc_channels: [u16; 8],
    rc_at: Tick,
    /// Axes saturated since the last operator notice, and when that notice was sent.
    saturation_pending: u8,
    last_saturation_notice: Option<Tick>,
    last_tick: Option<Tick>,
    pub save_requested: bool,
    pub calibration_requested: bool,
    pub driver_restart_requested: bool,
    /// Operator asked to clear a `SensorInit` fault; firmware rebuilds the driver and
    /// reports through `sensor_retry_result` (ADR 003 D2).
    pub sensor_retry_requested: bool,
    pilot_acro: bool,
    battery: Option<flight_core::battery::Guard>,
    battery_reading: flight_core::battery::Reading,
    pub battery_health: flight_core::battery::Health,
}
impl Default for Owner {
    fn default() -> Self {
        Self::new(Settings::default()).expect("valid default schema")
    }
}
impl Owner {
    pub const BOOT: Self = Self {
        flight: Flight::BOOT,
        settings: Settings::DEFAULT,
        ingress: Ingress::EMPTY,
        counters: Counters {
            rejected: 0,
            reply_overflow: 0,
            overruns: 0,
            maximum_tick_us: 0,
            maximum_jitter_us: 0,
            imu_saturated_samples: [0; 6],
        },
        timing: crate::metrics::Timing::EMPTY,
        log: Log::EMPTY,
        replies: [None; 4],
        sequence: 0,
        parameter_cursor: None,
        log_cursor: None,
        metrics_transfer: None,
        last_log: Tick(0),
        last_attitude: Tick(0),
        last_heartbeat: Tick(0),
        last_motors: Tick(0),
        last_imu: Tick(0),
        last_rc: Tick(0),
        rc_channels: [u16::MAX; 8],
        rc_at: Tick(0),
        saturation_pending: 0,
        last_saturation_notice: None,
        last_tick: None,
        save_requested: false,
        calibration_requested: false,
        driver_restart_requested: false,
        sensor_retry_requested: false,
        pilot_acro: false,
        battery: None,
        battery_reading: flight_core::battery::Reading::Unavailable,
        battery_health: flight_core::battery::Health::Unavailable,
    };

    pub fn new(settings: Settings) -> Result<Self, flight_config::Error> {
        settings.validate()?;
        let flight = Flight::new(settings.tuning(), settings.limits())
            .ok_or(flight_config::Error::Bounds)?;
        Ok(Self {
            flight,
            settings,
            ingress: Ingress::default(),
            counters: Counters::default(),
            timing: crate::metrics::Timing::default(),
            log: Log::default(),
            replies: [None; 4],
            sequence: 0,
            parameter_cursor: None,
            log_cursor: None,
            metrics_transfer: None,
            last_log: Tick(0),
            last_attitude: Tick(0),
            last_heartbeat: Tick(0),
            last_motors: Tick(0),
            last_imu: Tick(0),
            last_rc: Tick(0),
            rc_channels: [u16::MAX; 8],
            rc_at: Tick(0),
            saturation_pending: 0,
            last_saturation_notice: None,
            last_tick: None,
            save_requested: false,
            calibration_requested: false,
            driver_restart_requested: false,
            sensor_retry_requested: false,
            pilot_acro: false,
            battery: None,
            battery_reading: flight_core::battery::Reading::Unavailable,
            battery_health: flight_core::battery::Health::Unavailable,
        })
    }
    pub fn receive(&mut self, bytes: &[u8], policy: &mut Policy, at: Tick) -> bool {
        if let Policy::Signed { signing, .. } = policy {
            signing.advance(at.0);
        }
        let parsed = wire::decode(bytes, policy)
            .and_then(|packet| flight_protocol::event(&packet, self.system()));
        match parsed {
            Ok(event) => self.submit(event, at),
            Err(_) => {
                self.counters.rejected = self.counters.rejected.saturating_add(1);
                false
            }
        }
    }
    pub fn submit(&mut self, event: Event, at: Tick) -> bool {
        let event = match event {
            Event::Setpoint(
                flight_core::Source::Pilot,
                flight_core::control::Intent::Stabilized(p),
            ) if self.pilot_acro => Event::Setpoint(
                flight_core::Source::Pilot,
                flight_core::control::Intent::Acro(p),
            ),
            event => event,
        };
        self.ingress.push(event, at).is_ok()
    }
    pub fn reset(&mut self) {
        self.battery_reading = flight_core::battery::Reading::Unavailable;
        self.battery_health = flight_core::battery::Health::Unavailable;
        if let Some(guard) = &mut self.battery {
            guard.reset();
        }
        self.flight.reset();
        self.ingress = Ingress::default();
        self.replies = [None; 4];
        self.parameter_cursor = None;
        self.log_cursor = None;
        self.metrics_transfer = None;
        self.save_requested = false;
        self.calibration_requested = false;
        self.last_tick = None;
        self.last_log = Tick(0);
        self.last_attitude = Tick(0);
        self.last_heartbeat = Tick(0);
        self.last_motors = Tick(0);
        self.last_imu = Tick(0);
        self.last_rc = Tick(0);
        self.saturation_pending = 0;
        self.last_saturation_notice = None;
        self.log = Log::default();
        self.driver_restart_requested = false;
        self.sensor_retry_requested = false;
    }
    /// Stopped enrollment invalidates cached outputs, commands and queued replies.
    /// Caller must hold exclusive stopped Maintenance.
    pub fn discard_pending_authority(&mut self) {
        if !self.flight.stopped_maintenance() {
            return;
        }
        self.flight.disarm();
        self.flight.maintenance();
        self.ingress = Ingress::default();
        self.replies = [None; 4];
        self.parameter_cursor = None;
        self.log_cursor = None;
        self.metrics_transfer = None;
    }
    pub fn status_text(&mut self, message: &[u8]) {
        if message.len() > 70 {
            return;
        }
        let mut data = [0u8; 79];
        data[6] = 10;
        data[8] = message.len() as u8;
        data[9..9 + message.len()].copy_from_slice(message);
        self.emit(126, &data);
    }
    fn system(&self) -> u8 {
        self.settings.get("MAV_SYS_ID").unwrap_or(1.0) as u8
    }
    /// Explicit opt-in, only with stopped exclusive ownership and reviewed thresholds.
    pub fn configure_battery(&mut self, config: flight_core::battery::Config) -> bool {
        if !self.flight.stopped_maintenance() {
            return false;
        }
        self.discard_pending_authority();
        self.battery = Some(flight_core::battery::Guard::new(config));
        self.battery_reading = flight_core::battery::Reading::Unavailable;
        self.battery_health = flight_core::battery::Health::Unavailable;
        true
    }
    pub fn battery_reading(&mut self, reading: flight_core::battery::Reading) {
        self.battery_reading = reading;
    }
    fn emit(&mut self, id: u32, payload: &[u8]) {
        let mut reply = Reply::default();
        let Ok(len) = wire::encode(
            id,
            payload,
            self.sequence,
            self.system(),
            1,
            &mut reply.bytes,
        ) else {
            return;
        };
        reply.len = len;
        self.sequence = self.sequence.wrapping_add(1);
        if let Some(slot) = self.replies.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(reply);
        } else {
            self.counters.reply_overflow = self.counters.reply_overflow.saturating_add(1);
        }
    }
    fn free_replies(&self) -> usize {
        self.replies.iter().filter(|slot| slot.is_none()).count()
    }
    pub fn reply(&mut self) -> Option<Reply> {
        let reply = self.replies[0].take();
        self.replies.rotate_left(1);
        self.replies[3] = None;
        reply
    }
    fn parameter(&mut self, index: usize) {
        let Some(parameter) = PARAMETERS.get(index) else {
            return;
        };
        let Some(value) = self.settings.at(index) else {
            return;
        };
        let mut payload = [0; 25];
        payload[..4].copy_from_slice(&value.to_le_bytes());
        payload[4..6].copy_from_slice(&96u16.to_le_bytes());
        payload[6..8].copy_from_slice(&(index as u16).to_le_bytes());
        payload[8..8 + parameter.name.len()].copy_from_slice(parameter.name.as_bytes());
        payload[24] = 9;
        self.emit(22, &payload);
    }
    fn ack(&mut self, command: u16, accepted: bool) {
        self.emit(
            77,
            &flight_protocol::telemetry::ack(command, if accepted { 0 } else { 2 }, 255, 190),
        );
    }
    fn dispatch(&mut self, event: Event, now: Tick) {
        match event {
            Event::Arm => {
                let accepted = !self.driver_restart_requested
                    && (self.battery.is_none() || self.battery_health.permits_arm())
                    && !self.calibration_requested
                    && !self.save_requested
                    && self.flight.arm(now);
                self.ack(400, accepted);
            }
            Event::Select(source) => {
                let accepted = self.flight.select(source);
                self.ack(176, accepted);
            }
            Event::Mode(mode) => {
                let source = if mode == 3 {
                    flight_core::Source::Automatic
                } else {
                    flight_core::Source::Pilot
                };
                let accepted = self.flight.select(source);
                if accepted {
                    self.pilot_acro = mode == 1;
                }
                self.ack(176, accepted);
            }
            Event::ProtocolRequested(command) => {
                self.ack(command, true);
                self.emit(300, &flight_protocol::telemetry::protocol_version());
            }
            Event::VersionRequested(command) => {
                self.ack(command, true);
                self.emit(148, &flight_protocol::telemetry::autopilot_version());
            }
            Event::UnsupportedCommand(command) => {
                self.emit(77, &flight_protocol::telemetry::ack(command, 3, 255, 190))
            }
            Event::Diagnostic {
                motor,
                throttle,
                duration_us,
            } => {
                let mut values = [0.0; 4];
                let accepted = if motor < 4
                    && !self.driver_restart_requested
                    && !self.calibration_requested
                    && !self.save_requested
                {
                    values[motor as usize] = throttle;
                    self.flight.diagnostic(values, now, u64::from(duration_us))
                } else {
                    false
                };
                self.ack(209, accepted);
            }
            Event::ParameterRead { index, name } => {
                let index = if index >= 0 {
                    Some(index as usize)
                } else {
                    name_text(&name).and_then(Settings::index)
                };
                if let Some(index) = index {
                    self.parameter(index);
                }
            }
            Event::ParameterList => {
                if self.flight.snapshot().state != State::Armed {
                    self.parameter_cursor = Some(0);
                }
            }
            Event::ParameterSet { name, value } => {
                if !self.flight.stopped_maintenance() {
                    return;
                }
                if let Some(name) = name_text(&name) {
                    let mut next = self.settings;
                    if next.set(name, value).is_ok()
                        && self.flight.reconfigure(next.tuning(), next.limits())
                    {
                        self.settings = next;
                        self.driver_restart_requested |= name.starts_with("IMU_");
                        if let Some(index) = Settings::index(name) {
                            self.parameter(index);
                        }
                    }
                }
            }
            Event::Console(command) => match command {
                Console::Status => {
                    let snapshot = self.flight.snapshot();
                    let mut data = [0u8; 79];
                    let message = match snapshot.state {
                        State::Armed => b"armed".as_slice(),
                        State::Fault => b"fault".as_slice(),
                        State::Maintenance => b"maintenance".as_slice(),
                        _ => b"stopped".as_slice(),
                    };
                    data[6] = 10;
                    data[8] = message.len() as u8;
                    data[9..9 + message.len()].copy_from_slice(message);
                    self.emit(126, &data);
                }
                Console::Metrics(nonce) => {
                    if self.flight.stopped_maintenance() {
                        self.metrics_transfer =
                            Some(crate::metrics::Transfer::new(self.timing.snapshot(), nonce));
                    } else {
                        let mut denied = [0u8; 10];
                        denied[..4].copy_from_slice(b"FLXM");
                        denied[4..8].copy_from_slice(&nonce.to_le_bytes());
                        self.status_text(&denied);
                    }
                }
                Console::Maintenance => {
                    self.flight.maintenance();
                }
                Console::Exit => {
                    if !self.driver_restart_requested
                        && !self.calibration_requested
                        && !self.save_requested
                    {
                        self.flight.disarm();
                    }
                }
                Console::ClearFault => {
                    let snapshot = self.flight.snapshot();
                    if snapshot.state == State::Fault
                        && snapshot.reason == flight_core::Reason::SensorInit
                    {
                        // Clearing must prove the sensor works; never just drop the flag.
                        self.sensor_retry_requested = true;
                    } else {
                        self.flight.clear_fault();
                    }
                }
                Console::Save => {
                    self.save_requested = self.flight.stopped_maintenance();
                }
                Console::CalibrateGyro => {
                    self.calibration_requested = self.flight.stopped_maintenance();
                }
                Console::Disarm => self.flight.disarm(),
            },
            Event::Log { offset, count } => {
                if self.flight.stopped_maintenance() {
                    let total = 12 + self.log.len() * 64;
                    let offset = offset as usize;
                    if offset < total {
                        self.log_cursor =
                            Some((offset, offset.saturating_add(count as usize).min(total)));
                    }
                }
            }
            _ => {}
        }
    }
    pub fn tick(&mut self, now: Tick, sensor: Option<Sensor>) -> [f32; 4] {
        if let Some(last) = self.last_tick {
            if let Some(delta) = now.age(last) {
                self.counters.maximum_jitter_us =
                    self.counters.maximum_jitter_us.max(delta.abs_diff(1000));
                self.timing.jitter.record(delta.abs_diff(1000));
            }
        }
        self.last_tick = Some(now);
        if let Some(sensor) = sensor {
            if let Some(age) = now.age(sensor.at) {
                self.timing.sample_age.record(age);
            } else {
                self.timing.invalid_sample_age = self.timing.invalid_sample_age.saturating_add(1);
            }
        }
        if let Some(guard) = &mut self.battery {
            self.battery_health = guard.evaluate(self.battery_reading, now);
            if self.battery_health.requires_stop() && self.flight.snapshot().state == State::Armed {
                self.flight.fault(flight_core::Reason::Power);
            }
        }
        let stopped = self.ingress.take_stop();
        if stopped {
            self.flight.disarm();
            self.save_requested = false;
            self.calibration_requested = false;
            self.parameter_cursor = None;
            self.log_cursor = None;
            self.metrics_transfer = None;
            self.ack(400, true);
        }
        for (event, at) in self.ingress.take_setpoints().into_iter().flatten() {
            if let Event::Setpoint(source, intent) = event {
                self.flight.command(source, intent, at);
            }
        }
        // Process sensor before arm preconditions, then events; arm output starts next tick.
        let frame = self.flight.tick(now, sensor);
        if !stopped {
            for _ in 0..2 {
                if let Some((event, at)) = self.ingress.pop() {
                    if now.age(at).is_some_and(|age| age <= 100_000) {
                        self.dispatch(event, now);
                    }
                }
            }
        }
        let motors = self.flight.commit(frame, now);
        let snapshot = self.flight.snapshot();
        if !self.flight.stopped_maintenance() {
            self.metrics_transfer = None;
        }
        if snapshot.state != State::Maintenance
            && now.age(self.last_log).is_some_and(|age| age >= 10_000)
        {
            let e = snapshot.estimate;
            let a = e.attitude.angles();
            let r = e.rates;
            let target = snapshot.target_rates;
            let ta = snapshot.target_attitude.map_or(Vec3::ZERO, |q| q.angles());
            self.log.push(Entry {
                micros: now.0,
                values: [
                    now.0 as f32 * 1e-6,
                    r.0,
                    r.1,
                    r.2,
                    target.0,
                    target.1,
                    target.2,
                    a.0,
                    a.1,
                    a.2,
                    ta.0,
                    ta.1,
                    ta.2,
                    motors.iter().sum::<f32>() / 4.0,
                ],
            });
            self.last_log = now;
        }
        if snapshot.state == State::Armed {
            self.parameter_cursor = None;
            self.log_cursor = None;
            self.save_requested = false;
            self.calibration_requested = false;
        }
        // At most one large-transfer chunk per tick. All transfer bytes already initialized.
        if self.metrics_transfer.is_some() && self.replies.iter().any(Option::is_none) {
            if let Some(transfer) = &mut self.metrics_transfer {
                let (bytes, len, done) = transfer.next();
                self.status_text(&bytes[..len]);
                if done {
                    self.metrics_transfer = None;
                }
            }
        } else if let Some(index) = self.parameter_cursor {
            self.parameter(index);
            self.parameter_cursor = (index + 1 < PARAMETERS.len()).then_some(index + 1);
        } else if let Some((offset, end)) = self.log_cursor {
            if offset < end {
                let mut payload = [0u8; 97];
                payload[..4].copy_from_slice(&(offset as u32).to_le_bytes());
                let count = self
                    .log
                    .encode(offset, &mut payload[7..7 + (end - offset).min(90)]);
                payload[6] = count as u8;
                self.emit(120, &payload);
                self.log_cursor =
                    (offset + count < end && count != 0).then_some((offset + count, end));
            } else {
                self.log_cursor = None;
            }
        }
        // A due stream that finds too few free reply slots stays due and is sent on a
        // later tick instead of being dropped; reply_overflow then counts only real loss.
        if due(
            now,
            self.last_heartbeat,
            self.settings.get("MAV_RATE_SLOW").unwrap_or(0.0),
        ) && self.free_replies() >= 2
        {
            self.emit(
                0,
                &flight_protocol::telemetry::heartbeat(snapshot, self.pilot_acro),
            );
            let battery = if self.battery_health == flight_core::battery::Health::Healthy
                || matches!(
                    self.battery_health,
                    flight_core::battery::Health::Warning
                        | flight_core::battery::Health::PendingCritical
                        | flight_core::battery::Health::Critical
                ) {
                if let (Some(guard), flight_core::battery::Reading::Voltage { volts, .. }) =
                    (&self.battery, self.battery_reading)
                {
                    flight_protocol::telemetry::battery_voltage(volts, guard.config().cells())
                } else {
                    flight_protocol::telemetry::unknown_battery()
                }
            } else {
                flight_protocol::telemetry::unknown_battery()
            };
            self.emit(147, &battery);
            self.last_heartbeat = now;
        }
        if due(
            now,
            self.last_attitude,
            self.settings.get("MAV_RATE_ATT").unwrap_or(0.0),
        ) && self.free_replies() >= 1
        {
            self.emit(
                31,
                &flight_protocol::telemetry::attitude_quaternion(snapshot, now),
            );
            self.last_attitude = now;
        }
        if due(
            now,
            self.last_motors,
            self.settings.get("MAV_RATE_MOT").unwrap_or(0.0),
        ) && self.free_replies() >= 1
        {
            self.emit(140, &flight_protocol::telemetry::motors(motors, now));
            self.last_motors = now;
        }
        if due(
            now,
            self.last_imu,
            self.settings.get("MAV_RATE_IMU").unwrap_or(0.0),
        ) && self.free_replies() >= if sensor.is_some() { 2 } else { 0 }
        {
            if let Some(sensor) = sensor {
                // SCALED_IMU keeps upstream pyflix/QGC compatible; HIGHRES_IMU carries
                // the unclipped float sample.
                self.emit(26, &flight_protocol::telemetry::scaled_imu(sensor));
                self.emit(105, &flight_protocol::telemetry::highres_imu(sensor));
            }
            self.last_imu = now;
        }
        if due(
            now,
            self.last_rc,
            self.settings.get("MAV_RATE_RC").unwrap_or(0.0),
        ) && self.free_replies() >= 1
        {
            self.emit(
                35,
                &flight_protocol::telemetry::rc(self.rc_channels, self.rc_at),
            );
            self.last_rc = now;
        }
        // At most one STATUSTEXT warning per second naming every axis that saturated
        // since the last one; a full reply queue defers it rather than losing it.
        if self.saturation_pending != 0
            && self.free_replies() >= 1
            && self
                .last_saturation_notice
                .is_none_or(|last| now.age(last).is_none_or(|age| age >= 1_000_000))
        {
            self.emit(
                253,
                &flight_protocol::telemetry::imu_saturated(self.saturation_pending),
            );
            self.saturation_pending = 0;
            self.last_saturation_notice = Some(now);
        }
        motors
    }
    /// Record the driver's saturation mask for the sample passed to the next `tick`.
    /// Reporting only: the controller still uses the clipped value.
    pub fn record_imu_saturation(&mut self, mask: u8) {
        let mask = mask & 0b11_1111;
        for (axis, count) in self.counters.imu_saturated_samples.iter_mut().enumerate() {
            if mask & (1 << axis) != 0 {
                *count = count.saturating_add(1);
            }
        }
        self.saturation_pending |= mask;
    }
    /// Firmware result of a requested driver rebuild for a `SensorInit` fault. Only a
    /// passing rebuild clears it; a failing one leaves the owner faulted.
    pub fn sensor_retry_result(&mut self, configured: bool) {
        self.sensor_retry_requested = false;
        let snapshot = self.flight.snapshot();
        if configured
            && snapshot.state == State::Fault
            && snapshot.reason == flight_core::Reason::SensorInit
        {
            self.flight.clear_fault();
        }
    }
    pub fn record_execution(&mut self, microseconds: u64) {
        self.timing.owner.record(microseconds);
        self.counters.maximum_tick_us = self.counters.maximum_tick_us.max(microseconds);
        if microseconds > 1000 {
            self.counters.overruns = self.counters.overruns.saturating_add(1);
        }
    }
    pub fn receive_rc(&mut self, bytes: &[u8], at: Tick) -> bool {
        let Some(frame) = flight_core::rc::Sbus::decode(bytes) else {
            return false;
        };
        self.rc_channels.copy_from_slice(&frame.channels[..8]);
        self.rc_at = at;
        if frame.lost || frame.failsafe {
            if self.flight.snapshot().source == Some(flight_core::Source::Rc) {
                self.flight.disarm();
            }
            return false;
        }
        let names = ["RC_ROLL", "RC_PITCH", "RC_YAW", "RC_THROTTLE"];
        let mapping = names.map(|name| self.settings.get(name).unwrap_or(-1.0) as i32);
        if mapping.iter().any(|i| *i < 0 || *i >= 8) {
            return false;
        }
        let calibration = core::array::from_fn(|i| {
            let index = mapping[i] as usize;
            let zero = [
                "RC_ZERO_0",
                "RC_ZERO_1",
                "RC_ZERO_2",
                "RC_ZERO_3",
                "RC_ZERO_4",
                "RC_ZERO_5",
                "RC_ZERO_6",
                "RC_ZERO_7",
            ];
            let max = [
                "RC_MAX_0", "RC_MAX_1", "RC_MAX_2", "RC_MAX_3", "RC_MAX_4", "RC_MAX_5", "RC_MAX_6",
                "RC_MAX_7",
            ];
            let low = self.settings.get(zero[index]).unwrap_or(0.0) as u16;
            let high = self.settings.get(max[index]).unwrap_or(0.0) as u16;
            flight_core::rc::Calibration {
                low,
                center: low + (high - low) / 2,
                high,
            }
        });
        let Some(pilot) = flight_core::rc::pilot(frame, mapping.map(|i| i as usize), calibration)
        else {
            return false;
        };
        let mode = self.settings.get("RC_MODE").unwrap_or(-1.0) as i32;
        if (0..16).contains(&mode)
            && frame.channels[mode as usize] > 1750
            && self.flight.snapshot().source != Some(flight_core::Source::Rc)
        {
            self.flight.disarm();
            self.flight.select(flight_core::Source::Rc);
        }
        self.flight.command(
            flight_core::Source::Rc,
            flight_core::control::Intent::Stabilized(pilot),
            at,
        )
    }
}
fn due(now: Tick, last: Tick, rate: f32) -> bool {
    rate > 0.0 && now.age(last).is_some_and(|age| age >= (1e6 / rate) as u64)
}
fn name_text(name: &[u8; 16]) -> Option<&str> {
    let len = name.iter().position(|b| *b == 0).unwrap_or(16);
    let text = core::str::from_utf8(&name[..len]).ok()?;
    (!text.is_empty()).then_some(text)
}
