//! Schema facts that `#[expect(...)]` reasons in flight-config rely on. Regenerating
//! the schema with values that break them must fail here, not at a cast.
use flight_config::{PARAMETERS, Settings};

/// f32 represents every integer in this range exactly, so `v as i32 as f32 == v`
/// is an exact integrality test.
const EXACT_F32_INT: f32 = 16_777_216.0;

#[test]
fn integer_parameters_are_exactly_representable() {
    for p in &PARAMETERS {
        if p.integer {
            assert!(
                p.min >= -EXACT_F32_INT && p.max <= EXACT_F32_INT,
                "{}: {}..={} exceeds +/-2^24",
                p.name,
                p.min,
                p.max
            );
        }
    }
}

#[test]
fn pwm_resolution_bounds_the_shift() {
    let p = PARAMETERS
        .iter()
        .find(|p| p.name == "MOT_PWM_RES")
        .expect("schema has MOT_PWM_RES");
    assert!(
        p.integer && p.min >= 0.0 && p.max <= 31.0,
        "1u32 << res must not overflow"
    );
}

#[test]
fn rc_loss_time_fits_microseconds() {
    let p = PARAMETERS
        .iter()
        .find(|p| p.name == "SF_RC_LOSS_TIME")
        .expect("schema has SF_RC_LOSS_TIME");
    assert!(p.min > 0.0 && p.max * 1e6 < u64::MAX as f32);
}

#[test]
fn defaults_provide_every_tuning_and_limit_key() {
    // tuning()/limits() expect these keys; defaults must also validate.
    let s = Settings::default();
    s.validate().expect("defaults validate");
    let _ = s.tuning();
    let _ = s.limits();
}
