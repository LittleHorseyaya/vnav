#!/usr/bin/env python3
"""Predeclared five-seed robustness check on unchanged shared cached inputs."""
import csv
import datetime
import math
from pathlib import Path
import statistics
import shutil
import subprocess
import time

SEEDS = [12345, 23456, 34567, 45678, 56789]
METHODS = ['nister', 'eightpt', 'twopt', 'arun']
root = Path('/mnt/lab6_data/results')
cache = root / 'cache_full01'
if not Path(str(cache) + '_frames.csv').is_file():
    raise SystemExit('Shared cache missing; mount Lab6_data first.')
out = root / ('seed_validation_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
out.mkdir(exist_ok=False)
(out / 'protocol.txt').write_text(
    'Seeds: ' + ', '.join(map(str, SEEDS)) + '\n'
    'Cache: ' + str(cache) + '\n'
    'All bearing thresholds: 1e-6; Arun threshold: 0.05 m; max RANSAC iterations: 1000.\n'
    '5pt and 8pt refined; 2pt fixed-GT-rotation SVD; Arun refitted on inliers.\n'
    'All seeds retained. No ranking-based selection. Unchanged cached SIFT matches.\n'
    'Each frame restarts its RNG with the run seed.\n'
    'New runs use 2 s process watchdog; timeout rows are retained as failures.\n'
    'Seed 12345 reuses verified original run; seed 23456 reuses watchdog test.\n'
    'Timing is not compared across reused and isolated runs.\n')
with Path(str(cache) + '_frames.csv').open() as f:
    expected = [(r['frame_id'], r['previous_image_time_s'], r['image_time_s'])
                for r in csv.DictReader(f)]

def stats(v):
    v = sorted(v)
    if not v:
        return [float('nan')]*4
    return [statistics.mean(v), statistics.median(v),
            v[math.ceil(.95*len(v))-1], max(v)]

metrics = []
tic = time.monotonic()
print('Output:', out, flush=True)
print('Seeds fixed in advance:', SEEDS, flush=True)
for seed in SEEDS:
    prefix = out / ('seed_' + str(seed))
    print('\nRunning seed', seed, flush=True)
    reused = {12345: root / 'cache_full_eval01',
              23456: root / 'seed23456_timeout_test'}.get(seed)
    if reused is not None:
        for method in METHODS:
            shutil.copy2(Path(str(reused) + '_' + method + '.csv'),
                         Path(str(prefix) + '_' + method + '.csv'))
        (out / ('seed_' + str(seed) + '.log')).write_text('Reused completed run: ' + str(reused) + '\n')
        print('Reusing completed seed', seed, flush=True)
    else:
        with (out / ('seed_' + str(seed) + '.log')).open('w') as log:
            subprocess.run(['rosrun', 'lab_6', 'cache_eval', str(cache),
                            str(prefix), '1e-6', str(seed), '2'], stdout=log,
                           stderr=subprocess.STDOUT, check=True)
    for method in METHODS:
        path = Path(str(prefix) + '_' + method + '.csv')
        with path.open() as f:
            rows = list(csv.DictReader(f))
        pairs = [(r['frame_id'], r['previous_image_time_s'], r['image_time_s'])
                 for r in rows]
        # Compare numeric timestamps because different writers format floats differently.
        if len(pairs) != len(expected) or any(
            a[0] != b[0] or abs(float(a[1])-float(b[1])) > 1e-8 or
            abs(float(a[2])-float(b[2])) > 1e-8
            for a,b in zip(pairs,expected)):
            raise RuntimeError('Frame pairs differ: ' + str(path))
        good = [r for r in rows if r['status'] == 'ok']
        if any(not math.isfinite(float(r[k])) for r in good
               for k in ['rotation_frobenius', 'translation_error', 'solver_ms']):
            raise RuntimeError('Non-finite successful result: ' + str(path))
        rot = [float(r['rotation_frobenius']) for r in good]
        trans = [float(r['translation_error']) for r in good]
        item = dict(seed=seed, method=method, frames=len(rows),
                    success=len(good), failed=len(rows)-len(good),
                    timeout=sum(r['status']=='solver_timeout' for r in rows),
                    translation_unit='m' if method == 'arun' else 'direction')
        for label, values in [('rotation', rot), ('translation', trans)]:
            for suffix,value in zip(['mean','median','p95','max'],stats(values)):
                item[label + '_' + suffix] = value
        item['direction_below_0.1_fraction'] = (
            sum(v < .1 for v in trans)/len(trans)
            if method != 'arun' and trans else '')
        item['solver_ms_mean'] = statistics.mean(float(r['solver_ms']) for r in rows)
        metrics.append(item)
        print('%s: success=%d/%d translation mean=%.6g' %
              (method,len(good),len(rows),item['translation_mean']), flush=True)
        if seed == 12345:
            old_path = root / ('cache_full_eval01_' + method + '.csv')
            if old_path.is_file():
                with old_path.open() as f:
                    old = list(csv.DictReader(f))
                if len(old) != len(rows) or any(a['status'] != b['status'] for a,b in zip(old,rows)):
                    raise RuntimeError('Seed 12345 validation differs from previous fixed-seed run')
                diff = max(abs(float(a['translation_error'])-float(b['translation_error']))
                           for a,b in zip(old,rows) if a['status']=='ok')
                print('  seed 12345 max difference from previous:', diff, flush=True)
                if diff > 1e-10:
                    raise RuntimeError('Seed-setting change failed reproducibility check')
    with (out / 'per_run_metrics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(metrics[0]))
        writer.writeheader();writer.writerows(metrics)

summary=[]
for method in METHODS:
    selected=[r for r in metrics if r['method']==method]
    means=[r['translation_mean'] for r in selected]
    item={'method':method,'runs':len(selected),
          'translation_unit':'m' if method=='arun' else 'direction',
          'translation_run_mean_average':statistics.mean(means),
          'translation_run_mean_sample_std':statistics.stdev(means),
          'translation_run_mean_min':min(means),
          'translation_run_mean_max':max(means),
          'timeout_total':sum(r['timeout'] for r in selected),
          'failed_total':sum(r['failed'] for r in selected),
          'success_min':min(r['success'] for r in selected),
          'success_max':max(r['success'] for r in selected)}
    summary.append(item)
with (out / 'across_seed_summary.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(summary[0]))
    writer.writeheader();writer.writerows(summary)
print('\nAcross-seed translation means (average +/- SAMPLE std):',flush=True)
for r in summary:
    print('%s: %.6g +/- %.6g %s' %
          (r['method'],r['translation_run_mean_average'],
           r['translation_run_mean_sample_std'],r['translation_unit']),flush=True)
print('Total elapsed: %.1f seconds' % (time.monotonic()-tic),flush=True)
print('DONE:', out,flush=True)
