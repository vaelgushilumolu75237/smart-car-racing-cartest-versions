"""Execute production main-loop odometry with host encoder samples."""
from pathlib import Path
import argparse
import subprocess
import tempfile


def function(source, marker):
    start = source.index(marker)
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


PREFIX = r'''
#include <stdint.h>
#include <limits.h>
#include <assert.h>
#include <stdio.h>
#include "session_odometry.h"
enum { CAR_SPEED_OBSERVER_STATUS_WAITING=0, CAR_SPEED_OBSERVER_STATUS_SAMPLE=1 };
static struct { int64_t left_raw_delta_sum, right_raw_delta_sum; } car_encoder_diag;
static struct { uint8_t follow_session_active; } car_control_state;
static struct { uint8_t valid, status; } speed_observer_result;
static struct { int8_t follow_left_encoder_sign, follow_right_encoder_sign;
    uint32_t follow_left_encoder_counts_per_rev, follow_right_encoder_counts_per_rev; } app_config;
static uint64_t session_left_millirev, session_right_millirev, session_mean_forward_millirev;
static uint64_t odometry_left_fractional_remainder, odometry_right_fractional_remainder;
static int64_t odometry_last_left_raw_counts, odometry_last_right_raw_counts;
static uint8_t odometry_have_baseline, odometry_invalid_latched;
static uint32_t odometry_valid_sample_count, odometry_rejected_sample_count;
static int64_t parked_last_left_raw_counts, parked_last_right_raw_counts;
static uint64_t parked_left_millirev, parked_right_millirev;
static uint32_t parked_mean_forward_millirev;
static uint64_t parked_left_fractional_remainder, parked_right_fractional_remainder;
static uint8_t parked_odometry_have_baseline;
'''

CHECKS = r'''
int main(void) {
    app_config.follow_left_encoder_sign=1;
    app_config.follow_right_encoder_sign=-1;
    app_config.follow_left_encoder_counts_per_rev=1000;
    app_config.follow_right_encoder_counts_per_rev=1000;
    reset_session_odometry(); update_session_odometry();
    car_encoder_diag.left_raw_delta_sum=1000;
    car_encoder_diag.right_raw_delta_sum=-2000;
    update_session_odometry();
    assert(parked_mean_forward_millirev==1500 && !odometry_invalid_latched);
    car_encoder_diag.left_raw_delta_sum=900;
    update_session_odometry();
    assert(odometry_invalid_latched && parked_mean_forward_millirev==UINT32_MAX);
    car_encoder_diag.left_raw_delta_sum=2000;
    update_session_odometry();
    assert(odometry_invalid_latched && parked_mean_forward_millirev==UINT32_MAX);
    reset_session_odometry(); update_session_odometry();
    assert(!odometry_invalid_latched && parked_mean_forward_millirev==0);
    car_encoder_diag.left_raw_delta_sum=2500;
    car_encoder_diag.right_raw_delta_sum=-2500;
    update_session_odometry();
    assert(parked_mean_forward_millirev==500);
    reset_session_odometry(); app_config.follow_left_encoder_counts_per_rev=0;
    update_session_odometry(); update_session_odometry();
    assert(odometry_invalid_latched);
    app_config.follow_left_encoder_counts_per_rev=1000;
    reset_session_odometry(); car_control_state.follow_session_active=1;
    speed_observer_result.status=CAR_SPEED_OBSERVER_STATUS_WAITING;
    update_session_odometry(); assert(!odometry_invalid_latched);
    speed_observer_result.status=CAR_SPEED_OBSERVER_STATUS_SAMPLE;
    speed_observer_result.valid=1; update_session_odometry();
    car_encoder_diag.left_raw_delta_sum+=1000;
    car_encoder_diag.right_raw_delta_sum-=1000;
    update_session_odometry(); assert(session_mean_forward_millirev==1000);
    car_encoder_diag.left_raw_delta_sum-=1;
    update_session_odometry(); assert(odometry_invalid_latched);
    puts("parked/run odometry: forward mean, reverse latch, reset, bad CPR and waiting checks passed");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zig', required=True)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    source = (root / 'Src/main.c').read_text(encoding='utf-8')
    markers = ('static void reset_session_odometry(void)\n{',
               'static uint8_t normalize_forward_encoder_delta(',
               'static void update_session_odometry(void)\n{')
    with tempfile.TemporaryDirectory(prefix='cartest-entry-odometry-') as temp:
        output = args.output_dir or Path(temp)
        output.mkdir(parents=True, exist_ok=True)
        c_file, exe = output / 'entry_odometry.c', output / 'entry_odometry.exe'
        c_file.write_text(PREFIX + '\n'.join(function(source, marker) for marker in markers) + CHECKS, encoding='utf-8')
        command = [args.zig, 'cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(root / 'Inc'),
                   str(c_file), str(root / 'App/session_odometry.c'), '-o', str(exe)]
        compile_result = subprocess.run(command, capture_output=True, text=True)
        (output / 'compile.log').write_text(compile_result.stdout + compile_result.stderr, encoding='utf-8')
        compile_result.check_returncode()
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        (output / 'run.log').write_text(run.stdout + run.stderr, encoding='utf-8')
        print(run.stdout + run.stderr, end='')
        run.check_returncode()

if __name__ == '__main__':
    main()
