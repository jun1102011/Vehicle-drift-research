"""Independent Step35c native-state audit. Does not import the production metrics."""
import argparse
import hashlib
import importlib.util
import json
import re
from pathlib import Path
import numpy as np

W = Path(__file__).resolve().parents[1]
DT = .0005
TARGET = np.array([12., -30., .5989797412715093])
TOL = np.array([.2, 2., .03])


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(4 << 20), b''):
            h.update(b)
    return h.hexdigest()


def blocks(mask):
    edges = np.diff(np.r_[False, np.asarray(mask, bool), False].astype(np.int8))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def event(t, mask, n):
    seq = [(a, b) for a, b in blocks(mask) if b-a >= n]
    if not seq:
        return None
    a, b = seq[0]
    return {'start_s': float(t[a]), 'confirmed_s': float(t[a+n-1]),
            'first_last_span_s': float(t[a+n-1]-t[a]), 'required_samples': n,
            'complete_qualifying_run_samples': int(b-a)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    run = args.run.resolve()
    outpath = args.output or W / ('audit/independent_' + run.name + '.json')
    if outpath.exists():
        raise FileExistsError('Preserve existing audit; specify a new --output')
    loaderpath = W.parent / 'step33c_asb_amplitude_20261005/internal/data_archive.py'
    spec = importlib.util.spec_from_file_location('independent_archive', loaderpath)
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    names = ['repair_plant_raw', 'repair_cycle_diag', 'repair_entry_diag',
             'repair_actuator_command', 'asb_import12', 'asb_suspension', 'asb_diag', 'feedback_diag']
    raw = {n: loader.load(run, n) for n in names}
    p, c, ed, ac, imp, sus, ad, fb = (raw[n] for n in names)
    t = p[:, 0]
    checks = {}
    def check(name, value):
        checks[name] = bool(value)
    check('native_shape_and_time', p.shape[1] == 51 and t[0] == 0 and
          np.all(np.isfinite(p)) and np.max(abs(np.diff(t)-DT)) < 1e-10)
    check('control_time', c.shape[1] == 17 and np.max(abs(c[:, 0]-np.arange(len(c))*.01)) < 1e-9)
    check('entry_diagnostics_control_time', ed.shape[1] == 71 and np.array_equal(ed[:, 0], c[:, 0]))
    check('native_inputs_and_physical_time', all(np.array_equal(a[:, 0], t) for a in [ac, imp, sus, ad]))
    if not all(checks.values()):
        raise ValueError('Invalid native/control data axes: ' + str(checks))
    # Integer native ticks avoid floating-point boundary ambiguity in ZOH mapping.
    ci = np.minimum(np.rint(t/DT).astype(np.int64)//20, len(c)-1)
    phase = c[ci, 1].astype(int)
    states = np.c_[np.hypot(p[:, 2], p[:, 3])/3.6, p[:, 4], np.deg2rad(p[:, 5])]
    first = lambda value: (int(np.flatnonzero(phase == value)[0]) if np.any(phase == value) else None)
    ie, ih, ix = first(1), first(3), first(4)
    allowed = np.zeros(len(t), bool)
    if ie is not None:
        allowed[ie:ih if ih is not None else len(t)] = True
    entry = event(t, allowed & np.all(abs(states-TARGET) <= TOL, axis=1) & (states[:, 2] > 0), 600)
    hold = None
    hold_pass = False
    fixed_hold_ok = False
    if ih is not None:
        end = ih+20000
        fixed_hold_ok = (end <= len(t) and np.all(phase[ih:end] == 3) and ix == end and
                         len(blocks(phase == 3)) == 1 and abs(t[ix]-t[ih]-10) < 1e-10)
        if end <= len(t):
            err = states[ih:end]-TARGET
            rmse = np.sqrt(np.mean(err*err, axis=0))
            bad = abs(err) > TOL
            protection = (np.any(abs(ac[ih:end, 5:9]) >= 3000-1e-6, axis=1) |
                          (ed[ci[ih:end], 66] > 0))
            maxrun = max((b-a for a, b in blocks(protection)), default=0)*DT
            frac = float(np.mean(protection))
            minfz = float(np.min(p[ih:end, 17:21]))
            hold_pass = bool(fixed_hold_ok and np.all(rmse <= TOL) and frac <= .05 and maxrun <= .2 and minfz > 0)
            hold = {'fixed_window_valid': bool(fixed_hold_ok), 'samples': 20000,
                    'start_s': float(t[ih]), 'end_exclusive_s': float(t[ih]+10),
                    'state_rmse': rmse.tolist(), 'state_peak_abs_error': np.max(abs(err), axis=0).tolist(),
                    'state_out_of_band_s': (np.sum(bad, axis=0)*DT).tolist(),
                    'state_longest_out_of_band_s': [max((b-a for a, b in blocks(bad[:, j])), default=0)*DT for j in range(3)],
                    'joint_out_of_band_s': float(np.sum(np.any(bad, axis=1))*DT),
                    'joint_longest_out_of_band_s': max((b-a for a, b in blocks(np.any(bad, axis=1))), default=0)*DT,
                    'torque_protection_fraction': frac, 'torque_protection_longest_s': maxrun,
                    'min_wheel_load_N': minfz, 'passed': hold_pass}
    exitmask = np.zeros(len(t), bool)
    if ix is not None:
        exitmask[ix:] = True
    exit_event = event(t, exitmask & np.all(abs(states-[12., 0., 0.]) <= [.2, 1., .02], axis=1), 2000)
    if exit_event:
        exit_event['elapsed_from_phase4_s'] = exit_event['start_s']-float(t[ix])
    # Actual imported inputs, not only controller requests.
    check('actual_eight_inputs_equal_commands', ac.shape[1] == 9 and imp.shape[1] == 13 and np.array_equal(ac, imp[:, :9]))
    check('external_asb_import_command_received_zero', np.all(imp[:, 9:13] == 0) and np.all(ad[:, 7:9] == 0) and np.all(sus[:, 5:9] == 0))
    index = read(run/'archive_index.json')
    check('unique_archive_55_verified_channels', index['channel_count'] == 55 and index['bitwise_readback'] is True and sha(run/'raw_numeric.mat') == index['archive_sha256'])
    summary = read(run/'summary.json')
    check('vehicle_completed_60s_without_error', not summary.get('error') and abs(t[-1]-60) < 1e-8)
    used = read(run/'used_config.json')
    identity = read(run/'source_identity.json')
    profile = used['profile']
    check('ordinary_initial_state', used.get('ordinary_initial_state') is True)
    check('five_feedback_all_nodes', all(np.shape(n['K']) == (3, 5) for n in profile['entry_nodes']+profile['exit_nodes']+[profile['entry_normal']]))
    check('no_path_runtime_flags', used['controller']['repair']['stadium_mode'] == 0 and used['controller']['path']['feedback_enabled'] == 0 and used['controller']['state']['path_enabled'] == 0)
    en = used['controller']['entry']
    check('rear_upper_disabled_identity', used.get('rear_slip_upper_disabled') is True and used.get('rear_slip_upper_value') == '+Inf')
    check('public_numeric_budgets', en['wheel_kw'] == 12 and en['omega_acceleration'] == 40 and en['torque_limit'] == 3000 and en['slip_rate'] == .5 and abs(en['steering_rate_rad_s']-.9563444369840329) < 1e-13)
    freezepath = Path(identity['source_freeze_path'])
    freeze = read(freezepath)
    source_ok = sha(freezepath) == identity['source_freeze_sha256']
    for row in freeze['files']:
        source_ok = source_ok and sha(W/row['path']) == row['sha256']
    for row in freeze.get('external_files', []):
        source_ok = source_ok and sha(row['path']) == row['sha256']
    check('actual_source_freeze_unchanged', source_ok)
    check('actual_profile_unchanged', sha(identity['profile_path']) == identity['profile_sha256'] == used['profile_sha256'])
    actualk = []
    echo = (run/'LastRun_echo.par.evidence').read_text(encoding='utf-8-sig', errors='replace')
    for j in [1, 2]:
        vals = re.findall(r'^MX_AUX_COEFFICIENT\('+str(j)+r'\)\s+([-+0-9.eE]+)', echo, re.M)
        actualk.append(float(vals[0]) if len(vals) == 1 else None)
    vehicle = used['vehicle_config']
    check('actual_internal_aux_stiffness', actualk == [vehicle['kf_deg'], vehicle['kr_deg']])
    auxres = float(np.max(abs(sus[:, 25:27]-sus[:, 1:3]*np.array(actualk)))) if None not in actualk else None
    check('aux_moment_linear_identity', auxres is not None and auxres < 1e-8)
    # Native capture latch must agree with independently detected state history.
    natural_event = event(t, np.all(abs(states-TARGET) <= TOL, axis=1) & (states[:, 2] > 0), 600)
    latch = np.flatnonzero(fb[:, 3] > .5)
    if natural_event:
        check('native_capture_event_matches', len(latch) > 0 and abs(fb[latch[0], 4]-natural_event['start_s']) < 1e-9 and abs(fb[latch[0], 5]-natural_event['confirmed_s']) < 1e-9)
    else:
        check('native_capture_event_matches', len(latch) == 0)
    protocol = bool(entry is not None and hold_pass and exit_event is not None)
    passed = protocol and all(checks.values())
    out = {'schema': 'step35c-independent-native-v1', 'passed': passed, 'protocol_PASS': protocol and all(checks.values()),
           'profile_id': profile['profile_id'], 'profile_sha256': identity['profile_sha256'],
           'run_id': run.name, 'raw_sha256': sha(run/'raw_numeric.mat'),
           'source_freeze_sha256': identity['source_freeze_sha256'], 'checks': checks,
           'failed_checks': [k for k, v in checks.items() if not v],
           'entry_first600': entry, 'hold_first_fixed_10s': hold, 'exit_first2000': exit_event,
           'native_samples': len(t), 'actual_end_s': float(t[-1]), 'min_wheel_load_all_N': float(np.min(p[:, 17:21])),
           'max_abs_actual_wheel_torque_Nm': float(np.max(abs(ac[:, 5:9]))),
           'aux_moment_identity_max_error_Nm': auxres,
           'scope': 'Independent full-native arithmetic and source checks; no vehicle calls, no use of production metrics. Half-open fixed 10 s; radius and natural heading are not acceptance gates.'}
    outpath.write_text(json.dumps(out, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'passed': passed, 'failed_checks': out['failed_checks'], 'output': str(outpath)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
