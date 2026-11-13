# LubanCat DC monitoring and cyclic scheduling fix

## Confirmed cause and changes

On `humanoid@192.168.5.208`, kernel `6.1.84` is not PREEMPT_RT. Before
this change the daemon's cyclic thread used ordinary `TS` scheduling, even
though it requested FIFO priority 80. `RealtimeLoop::prepare()` rolled back
FIFO, CPU affinity, and memory locking solely because the kernel was not RT.
A complete successful setup is now retained on this kernel; the hard realtime
qualification flag remains false. Partial setup failures still roll back.

The IgH backend now queues a DC synchrony monitor datagram before sending and
processes its response after the next receive. `dc_deviation_ns` contains the
IgH system-time-difference monitor's upper estimate in ns. It is a magnitude,
not a signed host-to-reference offset. Missing/invalid responses and startup
are represented by `-1`, not zero. This monitor alone is not a complete
host-to-SYNC0 phase or realtime qualification.

A separate ordinary-priority diagnostics thread prints one health sample per
second to stderr, including DC monitor, WKC, process-data validity, period,
maximum wake latency/execution time, missed deadlines, skipped releases and
safety flags. Formatting and recurring terminal I/O are outside the cyclic
thread. The startup line reports setup results; `rt_guarantee=0` remains
expected on this kernel even with FIFO successfully active.

## Correct interpretation of `ethercat master`

The installed IgH is 1.6.9. Its locally available source at
`/home/humanoid/ethercat_motor_runtime/third_party/ethercat/src/master/master.c`
sets `dc_ref_time` to the first nonzero application timestamp, only while
`dc_ref_time` is zero. This value is the common reference for DC start times,
not a periodically refreshed measurement of the reference slave's clock.
`Application time - DC reference time` therefore is not a synchronization
error. The earlier hundreds-of-seconds difference was not evidence of DC
failure.

## Validation

- Local shim tests: 4/4 passed; local realtime setup tests: 3/3 passed.
- Native ARM64 build against `/opt/etherlab` and installed CycloneDDS succeeded.
- Target CTest selection: 29/29 passed (IgH shim, realtime setup, fake backend).
- The board retains its existing CMake 3.22 minimum compatibility adjustment;
  the local repository's CMake 3.24 requirement is unchanged.
- Old daemon source and executable were backed up before replacement. No
  enable, reset, or motion command was sent. No policy host/jog process was
  present at the transition.
- After replacement: cyclic thread is `FF`, priority 80. The final run uses CPU 6. Memory lock
  and affinity succeeded. Slave entered OP and domain reached WKC 3/3.
- SDO checks: `0x6040 = 0`, `0x6071 = 0`, `0x603F = 0`, `0x6061 = 10` (CST).

This is disabled-drive communication validation, not motor-motion or loaded
system qualification. Startup fixed-PDO write warnings remain: the device
rejects rewriting read-only maps, while logged existing and requested entries
match. They did not prevent OP and complete WKC. No watchdog/safety limit,
PDO mapping, network coalescing, EEE or kernel configuration was relaxed.

## On-board logs and recovery

Directory: `/home/humanoid/policy_embodied_runtime/backups/dc-jitter-20261113/`.

- `before.tgz`: original sources and `build/robot-io-daemon`.
- `CMakeLists.before.txt`: original board build configuration.
- `configure.log`, `build.log`, `tests.log`: native validation results.
- `daemon.log`: initial run; `daemon-final.log`: CPU 0 follow-up;
  `daemon-cpu6.log`: final CPU 6 run.
- `daemon.pid`: background daemon PID.
- `dmesg-before.log`: kernel history before replacement.

The final executable is installed in the board's existing `build/robot-io-daemon`
path. Follow its log with:

```bash
tail -f /home/humanoid/policy_embodied_runtime/backups/dc-jitter-20261113/daemon-cpu6.log
```

If rollback is needed, stop the recorded daemon with SIGTERM and wait for it
to release the master, then restore `build/robot-io-daemon` from `before.tgz`.
Do not run two daemons against the same master.

## CPU selection and remaining kernel limitation

The initial FIFO run on CPU 0 still caught a 3.135 ms wake delay and three
missed/skipped releases at approximately 225 seconds, despite no new kernel
WKC failure. This demonstrates why a non-RT kernel cannot be declared fully
qualified merely from FIFO being active. NIC interrupt counters were
concentrated on CPU 0. The CLI now supports `--cpu CPU`, validates the number
and allowed affinity before hardware access, and the final board process was
started on CPU 6. Both CPU governors inspected were already `performance`.
No IRQ affinity or perf sysctl was changed.

Use the following command on this board after stopping the existing daemon:

```bash
sudo ./build/robot-io-daemon configs/robot_profiles/sea_linear_stage.json --cpu 6
```

The original command without `--cpu` retains CPU 0 for compatibility. CPU 6
is a board-specific choice, not a portable default for every robot. CLI help
and four malformed CPU/option cases were checked without accessing hardware.
The diagnostics safety value `4259840` is the existing timeout/invalid-command
state while no valid command publisher is present; the drive remains disabled.

CPU 6 verification reached approximately 288,000 cycles with `misses=0`,
`skipped=0`, maximum wake latency 120,961 ns and maximum execution time
204,175 ns. This included 30 seconds of ordinary-priority busy loops on CPU 6
and CPU 0. Once OP/WKC stabilized, the kernel log had no new WKC drops,
SKIPPED, UNMATCHED or TIMED OUT entries during this observation. An earlier
short CPU 0 load check also passed before its later isolated wake stall.
These are bounded observations, not a long-duration realtime guarantee.

Final check: 312,193 cycles (over five minutes), same timing maxima and zero
misses/skips; OP, WKC 3/3, controlword zero. Final background PID: 72692.
