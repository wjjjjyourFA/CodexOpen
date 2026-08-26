# FAST-Calib framework integration

This directory contains the ROS-independent calibration core migrated from
[`hku-mars/FAST-Calib`](https://github.com/hku-mars/FAST-Calib), staged at
commit `1018ecf`. The original algorithm and this migrated implementation are
licensed under GPLv2; see `LICENSE`.

ROS1 bag loading, parameter loading, and debug publishers live in
`ros1/src/tools/fast_calib`. Runtime configuration intentionally does not live
in this module. The only parameter/configuration source is:

```text
install/bin/config/FastCalib/
```

See `docs/WS_LPNC框架/FAST-Calib迁移说明.md` for build and usage details.
