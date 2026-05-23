#!/usr/bin/env python3
"""
deploy.py  -  Deployment script for all tuned controllers

Input:  tuned_params.txt  (from tune_all binary)
        optional: --config  JSON override file

Output: deployment logs (stdout + deploy_YYYYMMDD_HHMMSS.log)
        deploy_manifest.json  (machine-readable deployment record)

Workflow:
  1. Validate that tuned_params.txt exists and is well-formed.
  2. Parse each controller's parameters.
  3. For each controller: validate params -> stage -> verify -> deploy.
  4. Write a deployment manifest with success/failure per controller.
  5. Exit code 0 if all controllers deployed; nonzero if any failed.

Usage:
    python deploy.py [--params tuned_params.txt] [--env production]
                     [--dry-run] [--verbose]
"""

import os
import sys
import re
import json
import argparse
import datetime
import traceback
import shutil
import platform


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
class Logger:
    def __init__(self, log_path: str, verbose: bool = False):
        self._verbose = verbose
        self._log = open(log_path, "w", encoding="utf-8")
        self._start = datetime.datetime.now()

    def _ts(self):
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _write(self, level: str, msg: str):
        line = f"[{self._ts()}] [{level:5}] {msg}"
        print(line)
        self._log.write(line + "\n")
        self._log.flush()

    def info(self, msg):    self._write("INFO",  msg)
    def ok(self, msg):      self._write("OK",    msg)
    def warn(self, msg):    self._write("WARN",  msg)
    def error(self, msg):   self._write("ERROR", msg)
    def debug(self, msg):
        if self._verbose:   self._write("DEBUG", msg)

    def section(self, title):
        bar = "=" * 60
        self._write("INFO", bar)
        self._write("INFO", f"  {title}")
        self._write("INFO", bar)

    def close(self):
        self._log.close()


# ---------------------------------------------------------------------------
# Parameter parsing
# ---------------------------------------------------------------------------
def parse_params_file(path: str) -> dict:
    """
    Parse INI-style tuned_params.txt into a dict of dicts:
      { "PID_IMC": {"Kp": 2.5, "Ki": 1.0, ...}, ... }
    Lines starting with '#' are comments.
    Section headers: [SECTION_NAME]
    Key-value pairs: key=value
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"Parameter file not found: {path}")

    result = {}
    current_section = None

    with open(path, encoding="utf-8") as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("[") and line.endswith("]"):
                current_section = line[1:-1]
                result[current_section] = {}
            elif "=" in line and current_section is not None:
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip()
                # Try numeric conversion; keep as string if not possible
                for fn in (int, float):
                    try:
                        value = fn(value)
                        break
                    except ValueError:
                        pass
                result[current_section][key] = value
            else:
                pass  # ignore malformed lines silently

    return result


# ---------------------------------------------------------------------------
# Parameter validators  (one per controller family)
# ---------------------------------------------------------------------------
VALIDATORS = {}

def validator(name):
    def decorator(fn):
        VALIDATORS[name] = fn
        return fn
    return decorator


@validator("PID_IMC")
@validator("PID_ZN")
@validator("PID_CC")
def validate_pid(section, params, log):
    errors = []
    for key in ("Kp", "Ki", "Kd"):
        if key not in params:
            errors.append(f"Missing {key}")
        elif not isinstance(params[key], (int, float)):
            errors.append(f"{key} is not numeric")
    if "uMin" in params and "uMax" in params:
        if params["uMin"] >= params["uMax"]:
            errors.append(f"uMin ({params['uMin']}) >= uMax ({params['uMax']})")
    if errors:
        for e in errors:
            log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] PID params valid: Kp={params['Kp']:.4f} Ki={params['Ki']:.4f} Kd={params['Kd']:.4f}")
    return True


@validator("LQR_Bryson")
def validate_lqr(section, params, log):
    errors = []
    if "Q_diag" not in params:
        errors.append("Missing Q_diag")
    if "R" not in params:
        errors.append("Missing R")
    else:
        if isinstance(params["R"], (int, float)) and params["R"] <= 0:
            errors.append(f"R must be positive (got {params['R']})")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] LQR params valid: R={params['R']}")
    return True


@validator("LQG_Bryson_Isotropic")
def validate_lqg(section, params, log):
    errors = []
    for key in ("Qf_sigma", "Rf_sigma"):
        if key not in params:
            errors.append(f"Missing {key}")
        elif isinstance(params[key], (int, float)) and params[key] <= 0:
            errors.append(f"{key} must be positive")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] LQG params valid")
    return True


@validator("MPC_HorizonTuner")
def validate_mpc(section, params, log):
    errors = []
    if "Np" not in params or "Nc" not in params:
        errors.append("Missing Np or Nc")
    else:
        if params["Nc"] > params["Np"]:
            errors.append(f"Nc ({params['Nc']}) > Np ({params['Np']})")
        if params["Np"] < 1:
            errors.append("Np must be >= 1")
    if "rho_y" in params and params["rho_y"] <= 0:
        errors.append("rho_y must be positive")
    if "rho_u" in params and params["rho_u"] <= 0:
        errors.append("rho_u must be positive")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] MPC params valid: Np={params['Np']} Nc={params['Nc']}")
    return True


@validator("LeadLag_LoopShaping")
def validate_leadlag(section, params, log):
    errors = []
    for key in ("continuousZero", "continuousPole", "gain"):
        if key not in params:
            errors.append(f"Missing {key}")
        elif isinstance(params[key], (int, float)) and params[key] <= 0:
            errors.append(f"{key} must be positive")
    if "continuousZero" in params and "continuousPole" in params:
        if params["continuousZero"] >= params["continuousPole"]:
            errors.append("Lead compensator requires pole > zero")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] LeadLag params valid")
    return True


@validator("SMC_BandwidthParam")
def validate_smc(section, params, log):
    errors = []
    for key in ("c_e", "K", "phi"):
        if key not in params:
            errors.append(f"Missing {key}")
        elif isinstance(params[key], (int, float)) and params[key] <= 0:
            errors.append(f"{key} must be positive")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] SMC params valid")
    return True


@validator("ADRC_Gao2003")
def validate_adrc(section, params, log):
    errors = []
    for key in ("b0", "omega_c", "omega_o"):
        if key not in params:
            errors.append(f"Missing {key}")
        elif isinstance(params[key], (int, float)) and params[key] <= 0:
            errors.append(f"{key} must be positive")
    if "omega_c" in params and "omega_o" in params:
        if params["omega_o"] <= params["omega_c"]:
            errors.append("omega_o should be > omega_c for stability")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] ADRC params valid")
    return True


@validator("ESC_PlantBW")
def validate_esc(section, params, log):
    errors = []
    for key in ("perturbFreq", "perturbAmp", "lpfCutoff", "hpfCutoff"):
        if key not in params:
            errors.append(f"Missing {key}")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] ESC params valid")
    return True


@validator("SmithPredictor_IMC")
def validate_smith(section, params, log):
    errors = []
    for key in ("inner_Kp", "inner_Ki", "delaySteps"):
        if key not in params:
            errors.append(f"Missing {key}")
    if "delaySteps" in params and isinstance(params["delaySteps"], (int, float)):
        if params["delaySteps"] < 0:
            errors.append("delaySteps must be >= 0")
    if errors:
        for e in errors: log.error(f"  [{section}] {e}")
        return False
    log.debug(f"  [{section}] SmithPredictor params valid")
    return True


def generic_validate(section, params, log):
    """Fallback: just check params dict is non-empty."""
    if not params:
        log.error(f"  [{section}] Empty parameter set")
        return False
    log.debug(f"  [{section}] Generic validation passed ({len(params)} keys)")
    return True


# ---------------------------------------------------------------------------
# Deployment stages
# ---------------------------------------------------------------------------

def stage_validate(section, params, log) -> bool:
    """Validate parameters using the registered validator."""
    log.info(f"  Validating {section}...")
    fn = VALIDATORS.get(section, generic_validate)
    return fn(section, params, log)


def stage_stage(section, params, deploy_dir, log, dry_run) -> bool:
    """Write parameters to a staging area JSON file."""
    staged_path = os.path.join(deploy_dir, "staging", f"{section}.json")
    if dry_run:
        log.info(f"  [DRY-RUN] Would write: {staged_path}")
        return True
    os.makedirs(os.path.dirname(staged_path), exist_ok=True)
    with open(staged_path, "w") as f:
        json.dump({"controller": section, "params": params,
                   "staged_at": datetime.datetime.now().isoformat()}, f, indent=2)
    log.ok(f"  Staged: {staged_path}")
    return True


def stage_verify(section, staged_path, log, dry_run) -> bool:
    """Re-read the staged file and confirm it round-trips correctly."""
    if dry_run:
        log.info(f"  [DRY-RUN] Would verify: {staged_path}")
        return True
    if not os.path.exists(staged_path):
        log.error(f"  Staged file missing: {staged_path}")
        return False
    with open(staged_path) as f:
        data = json.load(f)
    if data.get("controller") != section:
        log.error(f"  Verification failed: controller name mismatch in {staged_path}")
        return False
    log.ok(f"  Verified: {staged_path}")
    return True


def stage_deploy(section, params, staged_path, env, deploy_dir, log, dry_run) -> bool:
    """Move staged file to the deployment target directory."""
    target_dir  = os.path.join(deploy_dir, env, section)
    target_path = os.path.join(target_dir, "params.json")

    if dry_run:
        log.info(f"  [DRY-RUN] Would deploy {staged_path} -> {target_path}")
        return True

    os.makedirs(target_dir, exist_ok=True)
    shutil.copy2(staged_path, target_path)
    log.ok(f"  Deployed: {target_path}")

    # Write a deployment receipt
    receipt_path = os.path.join(target_dir, "deploy_receipt.json")
    with open(receipt_path, "w") as f:
        json.dump({
            "controller":    section,
            "environment":   env,
            "deployed_at":   datetime.datetime.now().isoformat(),
            "params_hash":   str(hash(json.dumps(params, sort_keys=True))),
            "platform":      platform.platform(),
            "python":        sys.version,
        }, f, indent=2)
    log.ok(f"  Receipt:  {receipt_path}")
    return True


# ---------------------------------------------------------------------------
# Main deployment loop
# ---------------------------------------------------------------------------
def deploy_all(params_all: dict, env: str, deploy_dir: str,
               log: Logger, dry_run: bool) -> dict:
    """
    Deploy every controller section found in params_all.
    Returns a manifest dict with per-controller status.
    """
    manifest = {
        "deploy_time": datetime.datetime.now().isoformat(),
        "environment": env,
        "dry_run":     dry_run,
        "controllers": {},
    }

    for section, params in params_all.items():
        log.section(f"Deploying: {section}")
        record = {"status": "PENDING", "stages": {}}
        staged_path = os.path.join(deploy_dir, "staging", f"{section}.json")

        stages = [
            ("validate", lambda s=section, p=params:
                stage_validate(s, p, log)),
            ("stage",    lambda s=section, p=params:
                stage_stage(s, p, deploy_dir, log, dry_run)),
            ("verify",   lambda sp=staged_path, s=section:
                stage_verify(s, sp, log, dry_run)),
            ("deploy",   lambda s=section, p=params, sp=staged_path:
                stage_deploy(s, p, sp, env, deploy_dir, log, dry_run)),
        ]

        all_ok = True
        for stage_name, stage_fn in stages:
            log.info(f"  Stage: {stage_name}")
            try:
                ok = stage_fn()
            except Exception as exc:
                log.error(f"  Exception in {stage_name}: {exc}")
                log.debug(traceback.format_exc())
                ok = False

            record["stages"][stage_name] = "OK" if ok else "FAILED"
            if not ok:
                log.error(f"  [{section}] Stage '{stage_name}' FAILED - aborting this controller.")
                all_ok = False
                break

        record["status"] = "OK" if all_ok else "FAILED"
        manifest["controllers"][section] = record

        if all_ok:
            log.ok(f"  [{section}] Successfully deployed to '{env}'")
        else:
            log.error(f"  [{section}] Deployment FAILED")

    return manifest


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Deploy tuned controllers to production.")
    parser.add_argument("--params",  default="tuned_params.txt",
                        help="Path to tuned_params.txt from tune_all (default: tuned_params.txt)")
    parser.add_argument("--env",     default="production",
                        help="Deployment environment name (default: production)")
    parser.add_argument("--deploy-dir", default="deployment",
                        help="Root directory for deployment artefacts (default: ./deployment)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate and stage only; do not write to deployment target")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable debug-level logging")
    args = parser.parse_args()

    # Setup
    ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = f"deploy_{ts}.log"
    log = Logger(log_path, verbose=args.verbose)

    log.section("Controller Deployment Script")
    log.info(f"Params file    : {os.path.abspath(args.params)}")
    log.info(f"Environment    : {args.env}")
    log.info(f"Deploy dir     : {os.path.abspath(args.deploy_dir)}")
    log.info(f"Dry-run        : {args.dry_run}")
    log.info(f"Log file       : {log_path}")
    log.info(f"Platform       : {platform.platform()}")
    log.info(f"Python         : {sys.version.split()[0]}")

    # Parse parameter file
    try:
        params_all = parse_params_file(args.params)
    except FileNotFoundError as e:
        log.error(str(e))
        log.error("Run tune_all binary first to generate tuned_params.txt")
        log.close()
        sys.exit(2)
    except Exception as e:
        log.error(f"Failed to parse {args.params}: {e}")
        log.close()
        sys.exit(2)

    log.info(f"Loaded {len(params_all)} controller section(s): {list(params_all.keys())}")

    if not params_all:
        log.error("No controller sections found in params file.")
        log.close()
        sys.exit(2)

    # Deploy
    manifest = deploy_all(params_all, args.env,
                           os.path.abspath(args.deploy_dir),
                           log, args.dry_run)

    # Write manifest
    manifest_path = f"deploy_manifest_{ts}.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    log.info(f"Manifest written: {manifest_path}")

    # Final summary
    log.section("Deployment Summary")
    n_ok     = sum(1 for v in manifest["controllers"].values() if v["status"] == "OK")
    n_failed = len(manifest["controllers"]) - n_ok

    for ctrl_name, record in manifest["controllers"].items():
        status_str = "OK" if record["status"] == "OK" else "FAILED"
        stages_str = " | ".join(f"{s}:{r}" for s, r in record["stages"].items())
        log.info(f"  {ctrl_name:<30} {status_str:<8} [{stages_str}]")

    log.info("")
    if n_failed == 0:
        log.ok(f"All {n_ok} controller(s) deployed successfully to '{args.env}'.")
    else:
        log.error(f"{n_failed} controller(s) FAILED; {n_ok} succeeded.")

    log.close()

    # Print log file location one final time on stdout
    print(f"\nFull log: {os.path.abspath(log_path)}")
    print(f"Manifest: {os.path.abspath(manifest_path)}")

    sys.exit(0 if n_failed == 0 else 1)


if __name__ == "__main__":
    main()
