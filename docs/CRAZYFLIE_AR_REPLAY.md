# AR flight replay

The flight log says where the Crazyflie thought it was: the flow deck's estimate, which drifts and can jump. A phone on a tripod sees where it really was. `scripts/crazyflie/ar_replay.py` puts the two together. It takes a phone video of a flight and the flight's log, and writes:

- **`ar.mp4`**, a dashboard: the video with the arena, the fence, the boxes, the drone's own estimate, its four ranger beams, the planner's command and the guard's verdicts drawn into the room, a 3x loupe on the drone, a top-down map with both tracks, and a timeline of the drift between the estimate and the camera;
- **`ar.json`** and **`track.csv`**: the camera's position for the drone in every frame, the estimate beside it, the drift, every jump of the estimate and how far the camera saw the drone move at that moment, and where it came to rest;
- the **`compare.py --landed X,Y`** command, with the landing spot measured from the video instead of a tape measure.

Nothing here has been filmed yet. The numbers below are from a **synthetic** video, rendered from a sim flight's true pose, so that every number the replay measures can be checked against what really happened.

## How it works

1. **The camera.** Printed ArUco markers lie on the floor at known spots (the setup file, `arenas/example-room.ar.json`). The replay finds them in about 48 frames, fits the phone's focal length to them, and solves the phone's pose from all of them at once. From then on every pixel maps to a ray in the arena frame.
2. **The drone.** The median of those frames is the empty room. In each frame, what differs from it are blobs: the drone, its shadow, a person walking by. Each blob is put back into the room on the plane at the height the drone's downward range sensor logged.
3. **The time.** The log's own clock and the video's are not the same. The replay slides the logged track along the video until the estimate lands on the blobs, first in 0.1 s steps, then in quarter frames.
4. **The track.** A tracker follows the drone from frame to frame. It predicts each frame from its own last position moved by the estimate's motion, with the estimate's jumps taken out (a logged step faster than 2.5 m/s is a jump, not the drone). So a jumping estimate never drags it along. Among the blobs near that prediction it prefers the one that also sits where the estimate, plus its slowly learned offset, says. This is what keeps it off the shadow: at take-off the drone and its shadow split from one blob, and on the plane at the logged height the shadow slides away while the drone goes straight up, as the estimate says it does.
5. **The numbers.** Drift is the estimate minus the camera. A jump is reported with how far the camera saw the drone move at that moment, less the move its own motion explains. The landing spot is the camera's position for the drone once the log has ended and the motors are off.

## Results on a synthetic video

A bench-profile explore flight (seed 3) with a flow-deck fault at 40 s: the estimate jumps 0.79 m, and the guard lands the drone. `ar_replay.py synth` rendered it as a 48 s, 1080p, 30 fps phone video from a tripod 2.1 m up in a corner, labelled SYNTHETIC in every frame.

| Measure | Replay | Truth |
| --- | --- | --- |
| Focal length | 1065 px (84.0°), fitted | 84.0° |
| Time offset | 2.308 s | 2.300 s |
| Drone found in flight | 100% of frames | |
| Camera vs the sim's true track | median 1.5 cm, 95% under 2.5 cm | |
| Drone's estimate vs the true track | median 6.3 cm | |
| The jump at 40.0 s | estimate 79 cm, camera 1 cm | estimate 79 cm, drone 0 |
| Landing spot | 2.20, 1.30 m | 2.20, 1.30 m |
| What the estimate said | 3.06, 1.31 m (86 cm off) | |

The first version of the tracker followed the shadow from take-off on, and was 61 cm off; the self-check against the sim's truth caught it. `tests/test_ar_replay.py` replays a smaller version (a square flight with the same fault, 640x360) in CI and holds it to 3 cm.

## Filming a flight

1. **Print the markers.** `python3 scripts/crazyflie/ar_replay.py sheet scripts/crazyflie/arenas/example-room.ar.json` writes A4 pages at 300 dpi with each marker at its true size (18 cm). Print at 100%, no "fit to page". Check one with a ruler.
2. **Lay them out.** Tape each marker flat at the x, y and heading in the setup file, measured from the arena's corner, the same corner the arena file uses. For your own room, copy the setup file and change the spots; spread the markers out, as at least two must stay in view and unhidden by the drone.
3. **Set up the phone.** On a tripod, high in a corner, so the whole floor of the arena is in view. Lock focus and exposure. Use the main lens at 1x: it has the least distortion. Landscape, 30 fps or more.
4. **Film.** Start recording before you arm, and stop after the motors are off and the drone has sat still for 2 s. Do not move the phone. Keep people out of the picture as best you can.
5. **Replay.**

```sh
pip install -r scripts/crazyflie/requirements-ar.txt
python3 scripts/crazyflie/ar_replay.py run flight.mp4 flights/<the log>.jsonl --setup scripts/crazyflie/arenas/example-room.ar.json
```

It prints what it found and writes a folder next to the log. `--no-video` writes only the numbers; `--stills 12.5,40.3` also writes the dashboard at those video times. If the phone's field of view is known, `--hfov` skips the fit; if the sync looks wrong, `--offset` sets it.

To try all of this before a flight, `ar_replay.py synth <sim log> --setup ... --out video.mp4` renders a synthetic phone video of any sim log, with its truth in `video.mp4.truth.json`. The replay finds that file and adds a self-check to its report.

## What it assumes

- **The camera stands still.** It solves the pose once. If the markers move between frames it says so.
- **The video is undistorted.** Phones correct their main lens. A lens with strong distortion would need a calibration this does not do.
- **The drone is at its logged height.** The downward range sensor is good to a centimetre or two over a flat floor. Over a box it reads the box, and the track bends there.
- **The drone is the blob near where it should be.** A person or a pet crossing the drone's path at its height can steal the track for a moment; `ar.json` has every frame, so such a moment shows as a spike.

## Files

| Path | What it is |
| --- | --- |
| `scripts/crazyflie/ar_replay.py` | `run`, `synth` and `sheet`: the replay and its dashboard |
| `scripts/crazyflie/arcam.py` | the camera model, marker detection, focal fit and pose |
| `scripts/crazyflie/arsynth.py` | the synthetic phone video: a ray-cast room, the Crazyflie and its shadow |
| `scripts/crazyflie/arenas/example-room.ar.json` | where the markers lie in the example room, and how tall its boxes are |
| `scripts/crazyflie/requirements-ar.txt` | OpenCV, NumPy and Pillow; not needed to fly |
| `scripts/crazyflie/tests/test_ar_replay.py` | the camera, the markers, and a whole replay of a synthetic flight |
