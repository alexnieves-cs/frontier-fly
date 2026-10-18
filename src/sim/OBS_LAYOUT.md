# Observation layout

`OBS_DIM = 306`. Every value is a float. Lidar ranges are `distance / 8m` (1 = miss).

| Index | Count | Signal |
| --- | --- | --- |
| 0–31 | 32 | Horizontal lidar, yaw + `i * 2π/32`, normalized range |
| 32–39 | 8 | Tilted lidar, ±0.45 rad elevation, every 45° |
| 40–42 | 3 | Body-frame velocity (m/s), forward / left / up |
| 43 | 1 | Yaw rate (rad/s), noisy in quad mode |
| 44–45 | 2 | sin(yaw), cos(yaw) |
| 46–47 | 2 | Distance to floor and ceiling, divided by floor height |
| 48–301 | 254 | Egocentric 16×16 visited-voxel crop (0.5 m), last 2 cells dropped |
| 302–303 | 2 | Roll, pitch (radians). Zero in kinematic mode. Replaces the 2 dropped map cells |
| 304 | 1 | Floor index / (floors − 1) |
| 305 | 1 | Fraction of episode steps remaining |

Actions are `[vx, vy, vz, yawRate]` in `[-1, 1]`, scaled inside the env to 2 m/s, 2 m/s, 1 m/s, and 90°/s. `vx` is forward, `vy` is left, `vz` is up.
