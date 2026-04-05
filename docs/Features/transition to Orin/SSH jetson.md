# SSH into Jetson Orin Nano

## Over USB (Recommended)

The Jetson and host PC are connected via USB-C. The Jetson appears at `192.168.55.1` on the USB network interface.

```bash
ssh badgerfly@192.168.55.1
# Password: 12345678
```

This works regardless of what WiFi networks each device is on.

## VS Code Remote SSH

1. Install the **Remote - SSH** extension in VS Code on your host PC
2. Connect to `badgerfly@192.168.55.1`
3. Open `~/ardu_ws` as the workspace folder
4. Edit files on the Jetson directly from your host PC's VS Code

## GitHub Remote (Primary Deployment Method)

The `dbvf_autonomy` repo is hosted at **https://github.com/Flippigan/dbvf_autonomy** (private). The `dbvf_msgs` package is included in the same repo.

### Jetson Setup (already done)

The repo is cloned on the Jetson at `~/ardu_ws/src/dbvf_autonomy` with a GitHub token embedded in the remote URL, so pulls work without re-authentication.

### Push updates from dev machine → Jetson

```bash
# 1. On dev machine: commit and push
git -C /home/finn/Documents/ardu_ws/src/dbvf_autonomy add -A
git -C /home/finn/Documents/ardu_ws/src/dbvf_autonomy commit -m "description of changes"
git -C /home/finn/Documents/ardu_ws/src/dbvf_autonomy push

# 2. On Jetson: pull and rebuild
cd ~/ardu_ws/src/dbvf_autonomy && git pull
cd ~/ardu_ws && colcon build --packages-select dbvf_msgs dbvf_autonomy
source install/setup.bash
```

## Jetson Workspace Layout

```
~/ardu_ws/
├── src/
│   ├── dbvf_autonomy/    # Cloned from GitHub (feat/wa-reload-mechanism branch)
│   ├── dbvf_msgs/        # Included in same GitHub repo
│   ├── apriltag_ros/     # Clone v3.3.0
│   └── apriltag_msgs/    # Clone v2.0.1
├── install/              # Built natively on Jetson (ARM64)
├── build/
└── log/
```

## Notes

- Jetson user: `badgerfly`, password: `12345678`
- The USB network interface on the host is `enxf2cb49b1ecb6` at `192.168.55.100`
- The Jetson does not need WiFi for SSH — the USB-C data cable handles it
- `sshpass` is available on the dev machine for scripted SSH: `sshpass -p '12345678' ssh badgerfly@192.168.55.1 "<command>"`
- If the Jetson needs internet (e.g., for apt install), share the host's connection over USB:

```bash
# On host PC:
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlo1 -j MASQUERADE

# On Jetson:
sudo ip route add default via 192.168.55.100
sudo bash -c 'echo "nameserver 8.8.8.8" > /etc/resolv.conf'
```
