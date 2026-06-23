# Handoff: Frequency-Domain System Identification - Levy's Method (Phase 4, Iteration 2)

**Date:** 2026-06-22
**Status:** Implemented and verified
**Design doc:** [2026-06-22-frequency-domain-identification-design.md](2026-06-22-frequency-domain-identification-design.md)
**Backlog:** [docs/algorithm_backlog.md](../../algorithm_backlog.md)

## Scope of this iteration

Exactly the design doc's checklist - `FreqDomainIdentifier::fitLevy` only. Sanathanan-Koerner
generalization and complex-pole Vector Fitting remain deferred (see the design doc's
"Explicitly out of scope" and `docs/algorithm_backlog.md`'s "Frequency-Domain Identification
Extensions" section).

## Checklist

- [x] `lib/FreqDomainIdentifier.h` / `.cpp` - `fitLevy` (Levy 1959 linearised LS fit)
- [x] `lib/CMakeLists.txt` - added to `CTRL_CORE_SOURCES`
- [x] `lib/ControllerToolbox.h` - `#include "FreqDomainIdentifier.h"`
- [x] Feature registration - `CTRL_REGISTER_FEATURE(freq_domain_identifier)` at the bottom of
  the header (the design doc's step 4, "add a feature-flag entry to `lib/Features.h`", predates
  the M2 self-registration system; `Features.h` is now a thin wrapper over `ControllerRegistry`
  and needs no per-feature edit - this matches the existing pattern in `VectorFitting.h`,
  `BasicPID.h`, etc.)
- [x] `bindings/estimation_bindings.cpp` - `FreqDomainFitResult` + `FreqDomainIdentifier.fit_levy`
- [x] `bindings/smoke_test.py` - assertion
- [x] `tests/test_catch2_advanced.cpp` - 3 tests under `[freq_domain_id]`
- [x] `examples/python/ex107_frequency_domain_identification.py` - fits the README's plant from
  its own noiseless frequency response

## Correction made during this iteration: `tf2ss`'s numerator zero-padding convention

The design doc's testing plan names the existing minimal example `num={0.2}, den={1,-0.8}` (the
same one used in `bindings/smoke_test.py`'s `tf2ss` check) and the README's plant
`num={0.0048,0.0047}, den={1,-1.81,0.819}` as the two exact-recovery fixtures, generating the
"true" frequency response via `tf2ss` + `SystemAnalysis::getFrequencyResponse` (the design doc's
own suggested production path).

Both examples have `num.size() < den.size()`. `tf2ss` (`lib/PlantModel.cpp:52-58`) handles this
by **right-padding the numerator with leading zeros** (`num = padded ++ num`, not
`num ++ padded`) - so a short numerator is implicitly treated as the *trailing* (most-delayed)
coefficients, not the leading ones. Concretely, `TransferFunction({0.2}, {1.0,-0.8}, Ts)` does
**not** realise `H(z^-1) = 0.2/(1-0.8z^-1)`; it realises `H(z^-1) = 0.2*z^-1/(1-0.8z^-1)`
(relative degree 1, `b0=0` implied). This is the conventional and correct behaviour for the
common case of a strictly-proper sampled physical plant (no instantaneous feedthrough) - it is
not a bug, and `lib/PlantModel.cpp` is out of scope to change - but it means a test that
constructs ground truth via `tf2ss` from a short numerator and then asks `fitLevy` to recover
coefficients at `num_order = len(num)-1` will silently fit the wrong target.

Both tests were corrected to pass the numerator at its full realised length explicitly
(`{0.0, 0.2}` and `{0.0, 0.0048, 0.0047}` respectively, with `num_order` bumped to match) so the
fixture is unambiguous without requiring the reader to know `tf2ss`'s padding direction. The
example (`ex107`) carries a short comment pointing this out for the same reason. No other phase
of this work was affected - this is purely a test/example construction detail, not a property of
`fitLevy` itself, which only ever sees raw `(freqs, response)` arrays and is indifferent to how
the caller produced them.

## Verification

```
build/tests/test_catch2_advanced.exe "[freq_domain_id]"
  -> All tests passed (19 assertions in 3 test cases)

build/tests/test_catch2_advanced.exe   (full suite, regression check)
  -> All tests passed (3940 assertions in 250 test cases)

conda run -n soft_robotics -- python bindings/smoke_test.py
  -> ... FreqDomainIdentifier smoke test passed. ...
  -> All smoke tests passed.

conda run -n soft_robotics -- python examples/python/ex107_frequency_domain_identification.py
  ->   recovered num: [~0.0, 0.0048, 0.0047] (max error ~2e-17)
  ->   recovered den: [1.0, -1.81, 0.819] (max error ~8e-16)
  ->   rmse: ~1.6e-15
  -> [PASS] All checks passed.
  -> EXIT_CODE=0
```

`compile.bat`/`compile.sh` needed no new entries - `FreqDomainIdentifier.cpp` was added to the
existing `controller_toolbox` target's source list and the new tests to the existing
`test_catch2_advanced` target, both already listed.
