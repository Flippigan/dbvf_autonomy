# DBVF Competition Operations Guide

Step-by-step guide for operating the autonomy stack at the VFS DBVF competition.

**Competition Location:** Harford Airport, Churchville, MD (~39.567°N, 76.205°W)

---

## 1. Pre-Flight Setup (On the Ground, Before Arming)

1. Power on the drone (flight controller battery only — propulsion disconnected)
2. Connect laptop to Orin via SSH: `ssh dbvf@<orin-ip>`
3. Navigate to config:
   ```bash
   cd ~/ardu_ws/src/dbvf_autonomy/config/
   ```
4. Edit `mission_params.yaml` with coordinates received from organizers:
   - Replace `home_lat` / `home_lon` → Home (H) coordinates
   - Replace `landing_lat` / `landing_lon` → Landing zone (L) coordinates
   - Replace `wa_lat` / `wa_lon` → Water Autonomous (WA) coordinates
   - Replace `f1_lat` / `f1_lon` → Fire 1 (F1) coordinates
   - Replace `f2_lat` / `f2_lon` → Fire 2 (F2) coordinates
   - Set `drop_target` → `"F1"` or `"F2"` based on team strategy
   - Set `wa_offset_forward` / `wa_offset_right` → camera-to-mechanism offset (see §8)
5. Verify coordinates:
   ```bash
   grep -E 'lat|lon' mission_params.yaml
   ```
6. Build and launch:
   ```bash
   cd ~/ardu_ws
   colcon build --packages-select dbvf_autonomy
   source install/setup.bash
   ros2 launch dbvf_autonomy mission_sim.launch.py
   ```
7. Verify nodes are running:
   ```bash
   ros2 node list | grep dbvf
   ```
8. Verify parameters loaded:
   ```bash
   ros2 param get /mission_sequencer home_lat
   ros2 param get /mavlink_interface rc_start_channel
   ```

---

## 2. RC Switch Mapping

| Switch | RC Channel | Action | When to Use |
|--------|-----------|--------|-------------|
| [TBD — assign on transmitter] | 14 | Start mission | After arming in GUIDED mode, when ready to begin |
| [TBD — assign on transmitter] | 15 | Resume mission | After flagger raises flag at L, approving second takeoff |
| Mode switch | — | Abort (switch out of GUIDED) | Emergency — pilot takes manual control |

**Important:** Switches must be momentary or start in the LOW position. The system triggers on the LOW→HIGH transition (PWM crosses above 1700). Holding a switch high does not re-trigger.

---

## 3. Mission Flow

| Phase | What Happens | Pilot Action | Expected Duration |
|-------|-------------|-------------|-------------------|
| Pre-arm | Drone on ground at H, nodes running | Arm in GUIDED mode via MissionPlanner or RC | — |
| FM-1 Start | Flip channel 14 HIGH | None — hands off controls | — |
| FM-1 Takeoff | Drone climbs to 35ft at H | Observe vertical climb | ~10s |
| FM-1 Transit | Drone flies to L at 35ft | Observe horizontal flight | ~5s |
| FM-1 Land | Drone lands at L | Observe landing | ~10s |
| Wait Flagger | Drone idle on ground at L | Wait for flagger to raise flag | Variable |
| FM-2 Start | Flip channel 15 HIGH | None — hands off controls | — |
| FM-2 Takeoff | Drone climbs from L to 35ft | Observe vertical climb | ~10s |
| FM-2 Transit | Drone flies to F1 or F2 | Observe horizontal flight | ~10-20s |
| FM-2 Drop | Servo releases red payload | Observe payload release | ~2s |
| FM-3 Transit | Drone flies to WA | Observe horizontal flight | ~10s |
| FM-3 Land | Precision landing on AprilTag at WA (with camera-to-mechanism offset) | Observe slow descent | ~30s |
| FM-3 Takeoff | Drone climbs from WA to 35ft | Observe vertical climb | ~10s |
| FM-3 Drop | Servo releases yellow payload at F1/F2 | Observe payload release | ~2s |
| RTH | Drone flies back to H | Observe horizontal flight | ~10-20s |
| Land H | Drone lands at H | Observe landing | ~10s |
| Complete | Mission done, drone on ground | Disarm via RC or MissionPlanner | — |

---

## 4. Abort Procedure

- **Normal abort:** Pilot switches flight mode out of GUIDED on the RC transmitter. The mission sequencer detects the mode change and enters ABORT state. The drone follows ArduPilot's failsafe behavior for the selected mode (e.g., LAND = descend vertically, RTL = fly home).
- **Emergency kill:** Pull the propulsion battery plug (physical kill switch on airframe).
- **GCS abort:** Call abort service via terminal if available:
  ```bash
  ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{reason: 'Operator abort'}"
  ```

---

## 5. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Ch14 switch doesn't start mission | FSM not in IDLE state | Check `ros2 topic echo /dbvf/mission_state` — must show `IDLE` |
| Ch14 switch doesn't start mission | RC channel not mapped correctly | Verify ch14 on transmitter, check `ros2 topic echo /dbvf/rc_trigger` |
| Ch15 switch doesn't resume | FSM not in WAIT_FLAGGER | Check `ros2 topic echo /dbvf/mission_state` — must show `WAIT_FLAGGER` |
| Drone doesn't takeoff after start | Not armed or not in GUIDED mode | Arm and set GUIDED via MissionPlanner first |
| Wrong GPS coordinates | Config not reloaded | Rebuild (`colcon build`) and restart nodes after editing YAML |
| No heartbeat | Orin not connected to Cube Orange | Check serial cable, verify `ros2 topic echo /dbvf/heartbeat_status` |
| Mission times out | 9-minute timeout exceeded | Check `mission_timeout_s` parameter |

---

## 6. GPS Coordinate Format

Coordinates from organizers must be in **decimal degrees** (e.g., `39.56731, -76.20527`).

If received in a different format:
- **Degrees Minutes Seconds (DMS):** `decimal = degrees + minutes/60 + seconds/3600`
- **Degrees Decimal Minutes (DDM):** `decimal = degrees + decimal_minutes/60`
- **UTM:** Use an online converter to get decimal degrees

**Sanity check:** Competition coordinates should be approximately latitude ~39.567°, longitude ~-76.205° (Harford Airport area).

---

## 7. Monitoring During Flight

```bash
# Vehicle telemetry (position, altitude, mode)
ros2 topic echo /dbvf/vehicle_state

# Mission FSM state
ros2 topic echo /dbvf/mission_state

# Mission phase (FM1, FM2, FM3, RTH)
ros2 topic echo /dbvf/mission_phase

# RC trigger events
ros2 topic echo /dbvf/rc_trigger

# AprilTag detection status
ros2 topic echo /dbvf/tag_status

# Precision landing state (during WA landing)
ros2 topic echo /dbvf/landing_state
```

---

## 8. WA Precision Landing Offset (Camera-to-Mechanism)

The PiCam is mounted forward of center on the drone. During WA precision landing, the drone needs to position the **reload mechanism** (not the camera) over the AprilTag. The `wa_offset_forward` and `wa_offset_right` parameters in `mission_params.yaml` tell the landing system how far the mechanism is from the camera.

### How to Measure

1. Place drone on a flat, level surface.
2. Mark a point directly below the **center of the PiCam lens** on the surface.
3. Mark a point directly below the **center of the reload mechanism**.
4. Measure the **forward/backward** distance between marks (along the drone's nose-tail axis).
   - If the mechanism is **behind** the camera mark → **positive** value.
5. Measure the **left/right** distance between marks (perpendicular to nose-tail).
   - If the mechanism is **to the right** of the camera mark → **positive** value.
6. Convert inches to metres (divide by 39.37) and enter into `mission_params.yaml`:
   ```yaml
   wa_offset_forward: 0.12   # example: mechanism is 12cm behind camera
   wa_offset_right: 0.0      # example: mechanism is centered left-right
   ```

### Expected Values

For the current hardware layout (PiCam forward, mechanism at center):
- `wa_offset_forward` ≈ 0.10–0.15m
- `wa_offset_right` ≈ 0.0m

These offsets are only applied during WA landings. All other landings (L, H) use (0.0, 0.0) — camera centered over target.
