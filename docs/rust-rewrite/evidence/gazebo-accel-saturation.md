# Why Gazebo scenarios saturate the accelerometer

Date: 2026-10-10. Data: simulator-probes artifact of CI run 38025743862 (native amd64, Gazebo
11.10.2), all scenarios except `two-models`, whose single trace interleaves two models' ticks.
Hardware profile: none. This describes the simulator, not an airframe.

## Question

Nearly every scenario records one tick with all three accelerometer axes at the ±4 g register
limit. Is that a start-up artifact, a plugin bug, or physics?

## Finding: it is the post-disarm landing impact

Scenarios disarm the drone in the air. It free-falls (the accelerometer correctly reads about 0 g)
and hits the ground plane. The model and world set no contact softness, so ODE arrests the fall
almost rigidly within one 1 ms step.

| Scenario | Disarm height | Impact speed | Limited ticks |
| --- | ---: | ---: | ---: |
| level-thrust | 5.10 m | 10.5 m/s | 1, at the impact tick (10,566 m/s² raw) |
| pitch-pulse | 5.64 m | 11.1 m/s | 1, at the impact tick |
| long-level | 16.36 m | still falling at 16.9 m/s when the trace ends | 0 |
| diagnostic-stop | never lifts | — | 0 |
| landing-contact | 0.09 m | 1.1 m/s | 9 (repeated touchdowns) |

## Quantitative check

The accelerometer measures specific force, dv/dt + g. Predicting saturation as
|Δv_z/Δt + g| > 4 g from the traced world velocity, compared over 125,866 consecutive-tick pairs:

| | Saturated | Not saturated |
| --- | ---: | ---: |
| Predicted | 47 | 0 |
| Not predicted | 4 | 125,815 |

The four unpredicted saturations are three world or model resets (the teleport reads as a
1,800–3,700 m/s² impulse) and one mode-change crash whose acceleration is mostly lateral and
rotational, which the vertical-only predictor ignores.

With rigid contact and a 1 ms step, ±4 g allows at most 0.039 m/s of velocity change per step.
Touchdowns slower than that never saturated (1,094 contacts, the only exception being a reset);
faster ones usually do.

## Second finding: the IMU sample lags truth by one step

The prediction matches only if the IMU reading is compared with the velocity change of the
*previous* step. Example from level-thrust: velocity is arrested between 6,928 ms and 6,929 ms, but
the saturated reading appears at 6,930 ms. The plugin reads `imu->LinearAcceleration()` in its
world-update callback, where it reflects the sensor update of the preceding step, while the traced
pose and velocity are current. In this simulator the controller therefore sees acceleration 1 ms
late relative to the traced truth. Because accelerometer correction is a slow complementary
filter, the effect on attitude estimation is small. Paired C++/Rust comparisons and any
trajectory tolerance should account for it.

## What this means

- Neither the plugin's range model nor the new operator notice is wrong: a 5 m drop is far beyond
  ±4 g on real hardware too.
- The *magnitude* is unrealistic. A real airframe's legs, frame flex and foam spread the impact
  over several milliseconds. In this simulator any touchdown faster than about 4 cm/s saturates,
  so landing-phase saturation counts should not be used as hardware estimates.
- No physics change was made. Adding contact compliance to the legacy model would alter the frozen
  parity baseline, and would need its own recorded experiment and rollback.

## Reproduce

Download the `simulator-probes` artifact of a CI run (`gh run download <run> -n simulator-probes`).
For each consecutive tick pair (a, b) with 1 ms spacing, compute
`(b.velocity_world_m_s[2] - a.velocity_world_m_s[2]) / 0.001 + 9.80665`. Compare
`abs(value) > 4 * 9.80665` with the next tick's `sensor_limit_mask & 0b111000`.
