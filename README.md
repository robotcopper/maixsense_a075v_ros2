# maixsense_a075v

Lifecycle driver for the Sipeed MaixSense A075V RGB-D time-of-flight module.
Official product page:
[MaixSense-A075V](https://wiki.sipeed.com/hardware/en/maixsense/maixsense-a075v/maixsense-a075v.html).

This is not a USB sensor. It is a small Linux computer that appears on the host
as an Ethernet gadget. The only public data API is HTTP. The point cloud is
computed on the host from the raw frame; Sipeed does not expose one.

## Hardware

Inspected on a live module (kernel `Linux sipeed 4.9.118`, 2023-01-09):

| Piece | What it is |
| --- | --- |
| SoC | Allwinner V831 (`sun8iw19p1`), one Cortex-A7, `armv7l` |
| RAM | ~122 MiB, 16 MiB CMA |
| Storage | 128 MiB eMMC: logo, cfg, env, boot, rootfs (~64 MiB), UDISK (~18 MiB on `/root`) |
| OS | Linux 4.9, musl, BusyBox |
| USB | RNDIS gadget. Host interface `enx…`, camera `192.168.233.1`, host typically `192.168.233.101` |
| Sensor path | VIN `/dev/video0`–`video3`; ToF Opnous OPN8008 (`opn87xx_mipi`); RGB GC2145. Depth is **not** a raw V4L2 read: `testisp` runs a closed `sw_isp_*` on `flashdump.bin`. |

The ToF/RGB pipeline and the HTTP server are the same closed binary,
`/root/maix_dist/testisp` (~321 KiB, stripped, no sources). `keep_app.sh`
restarts it and toggles GPIO 105 as a heartbeat. The only listening ports are
22 (dropbear) and 80 (`testisp`).

Sipeed firmware updates are “replace files in `/root/maix_dist`”. There is no
SDK for the sensor. The wiki sidebar **MaixSense → Using MaixPy3** is a
different board (Maix-II Dock, same SoC family), not this image.

This repository does not ship replacement firmware for the module. A 2026-09-20
spike stopped `testisp` and captured VIN: the buffers are empty without the
Opnous ISP inside `testisp`. See [ADR 0008](../../../docs/adr/0008-maixsense-a075v-http.md)
and [`firmware/NOTES.md`](firmware/NOTES.md). The driver uses the documented HTTP
API.

A stock eMMC image taken before that experiment is documented in
[`firmware/BACKUP.md`](firmware/BACKUP.md); the image file itself is not in git.

## Network and SSH

The `192.168.233.0/24` prefix must be free on the host. After plug-in, wait
10–15 s for the app to come up, then:

```bash
ssh -o HostKeyAlgorithms=+ssh-rsa root@192.168.233.1
```

Password `root`. Current OpenSSH refuses the camera’s `ssh-rsa` host key unless
that algorithm is re-enabled for this host.

Optional `~/.ssh/config` fragment, scoped to this device only:

```
Host a075v 192.168.233.1
    HostName 192.168.233.1
    User root
    HostKeyAlgorithms +ssh-rsa
    PubkeyAcceptedAlgorithms +ssh-rsa
```

## Public HTTP interface

These are the endpoints `testisp` serves. They are the contract, not a choice
of the ROS node.

| Path | Role |
| --- | --- |
| `GET /getinfo` | module identity and ToF intrinsics (`info_t`, 81 bytes) |
| `GET /get_lut` | 8-bit depth lookup table |
| `GET /CameraParms.json` | RGB–ToF extrinsics (`R_Matrix_data`, `T_Vec_data`, `Camera_Matrix_data`, `Distortion_Parm_data`) |
| `POST /set_cfg` | packed 12-byte acquisition config |
| `GET /getdeep` | one raw frame (header + depth + IR + status + RGB) |

`set_cfg` fields, little-endian `<BBBBBBBBi`:

| Field | Meaning |
| --- | --- |
| `trigger_mode` | 0 stop, 1 auto, 2 single |
| `deep_mode` | 0 16-bit, 1 8-bit |
| `deep_shift` | shift for 8-bit depth; 255 uses the LUT |
| `ir_mode` | 0 16-bit, 1 8-bit |
| `status_mode` | 0 16-bit, 1 2-bit, 2 8-bit, 3 1-bit |
| `status_mask` | 1-bit status mask |
| `rgb_mode` | 0 YUV, 1 JPEG, 2 none |
| `rgb_res` | 0 800×600, 1 1600×1200 |
| `expose_time` | 0 auto, otherwise exposure |

Depth units on the wire are millimetres. The on-module LUT saturates at index
2500, matching the 0.2–2.5 m range.

## Throughput (measured 2026-09-20)

Sixty `GET /getdeep` in a row, live module, host on the RNDIS link. Every
request returned a **new** `frame_id` (60/60). A fresh TCP connection per
request versus one persistent connection did not differ.

| RGB (`rgb_mode`) | Unique frames | Median latency | Payload | Throughput |
| --- | --- | --- | --- | --- |
| JPEG on (`1`) | 6.1–6.2 /s | 160 ms | 497 KiB | 3.0–3.1 MiB/s |
| off (`2`) | 8.1–8.3 /s | 122 ms | 384 KiB | 3.1–3.2 MiB/s |

The product of payload × rate is constant at about **3.1 MiB/s**. That is the
ceiling of `testisp` plus RNDIS, not of USB 2.0 (tens of MiB/s) and not of HTTP
headers (hundreds of bytes on a half-megabyte body). Dropping JPEG is the only
lever that raises depth/IR rate (~+35 %). Reusing the TCP connection is not.

### A TCP relay on the camera is not worth it

A process on the V831 that `GET`s `localhost` and forwards the body on a raw
TCP socket still talks HTTP to `testisp` and still ships the same 384–497 KiB
over the same USB gadget. The handshake is already free; the body is the cost.
That relay adds a hop, a second binary to keep alive next to `keep_app.sh`,
and a flash that a Sipeed update wipes — for no extra frames.

Replacing `testisp` itself is a different project (closed Opnous `sw_isp`,
see ADR 0008). Until that exists, the driver is an HTTP client.

The preview page at `http://192.168.233.1` exposes additional controls that
**do not** go through `set_cfg`. They run in the browser on already-decoded
pixels: spatial filter (Gaussian / bilateral, kernel size), temporal filter
(alpha), flying-point filter (threshold), and status-class masks (normal,
over-exposed, under-exposed, bad). Colormap and depth-range sliders are display
only.

## C++ SDK

`libmaixsense_a075v` contains no ROS dependency. Its installed API is deliberately
small:

```cpp
#include <maixsense_a075v/device.hpp>

maixsense_a075v::Device camera(
  "192.168.233.1", 80, std::chrono::seconds(2));
camera.wait_until_ready(std::chrono::seconds(20));
camera.load_calibration();

maixsense_a075v::AcquisitionConfig acquisition;
acquisition.trigger_mode = 1;
camera.set_acquisition_config(acquisition);

maixsense_a075v::DecodedFrame frame;
camera.read_frame(frame);
```

`types.hpp` exposes acquisition and filter configuration, decoded images and
calibration only. ROS, libcurl, OpenCV and the packed wire structures do not
leak into the public headers. HTTP, JSON parsing, JPEG decoding, calibration
and filters are implementation details in `src/`.

## ROS node

```
ros2 launch maixsense_a075v tof_camera.launch.py
```

The launch file configures then activates the lifecycle node. Parameters live
in `config/tof_camera.yaml`. Relative topic names, `SensorDataQoS`:

| Topic | Type |
| --- | --- |
| `cloud` | `sensor_msgs/PointCloud2` |
| `rgb/image` | `sensor_msgs/Image` (`bgr8`) |
| `rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `tof/depth` | `sensor_msgs/Image` (`mono16`, millimetres) |
| `tof/intensity` | `sensor_msgs/Image` (`mono16`, IR) |
| `tof/status` | `sensor_msgs/Image` (`mono16`) |
| `tof/camera_info` | `sensor_msgs/CameraInfo` |

`on_configure` reads calibration (`/getinfo`, `/get_lut`, `/CameraParms.json`).
It waits up to `configure_timeout` for the camera's 10–15 second boot instead
of requiring a launch delay.
`on_activate` posts `set_cfg` with trigger auto and starts the acquisition
thread. `on_deactivate` stops the thread and posts trigger stop.

### Frames

`a075v_link` is the body frame of the housing. Its origin is on the rear face,
on the axis of the circular boss of the front face, 22.5 mm behind it, with x
out of the lenses, y left and z up. That is a point you can measure on the part
you actually bolt to the robot, so the robot description only has to place
`a075v_link`.

The node publishes two static transforms from it, to
`a075v_tof_optical_frame` and `a075v_rgb_optical_frame`, both with the REP-103
optical rotation (z forward, x right, y down):

| Child frame | x | y | z |
| --- | --- | --- | --- |
| `a075v_tof_optical_frame` | 15.8 mm | 5.5 mm | −5.5 mm |
| `a075v_rgb_optical_frame` | 15.7 mm | −18.2 mm | 5.7 mm |

Those offsets are the sensor package centres in the manufacturer STEP model in
`asset/`, cross-checked against the 36 × 36 × 22.5 mm housing of the datasheet;
they are constants in `types.hpp`. They are *not* the factory RGB–ToF
extrinsics: `T_Vec_data` puts the two sensors 14 mm apart along the optical
axis, which the CAD contradicts — they are coplanar to 0.1 mm. `T_Vec_data`
compensates the projection used to colour the cloud, and is still used for
exactly that, inside the frame pipeline. Expect roughly a millimetre of error
on the table above, since a package centre is not the entrance pupil.

Set `publish_static_transforms` to `false` when the robot description owns
those two edges; every TF edge must have one publisher. ADR 0009 records why
the CAD wins over the factory extrinsics here.

All nine camera configuration fields except trigger mode are parameters.
Trigger mode is owned by the lifecycle. Host-side temporal, Gaussian/bilateral
spatial and flying-point filters, plus status-class masks, are live parameters.
Changing them with `ros2 param set` does not restart the node. Setting
`rgb_mode` to `2` stops RGB publication and reduces the payload.

This package sits under `harp3_hardware_interface/` because no upstream driver
exists for Lyrical. ADR 0003 still applies: it is a `LifecycleNode`, not a
`ros2_control` plugin. The eventual home is `harp3_sensors` once that package
is created.

## Layout

```
config/     tof_camera.yaml
launch/     tof_camera.launch.py
include/    public SDK types/API and the component header
src/        SDK implementation, private frame pipeline, lifecycle component
test/       SDK-only calibration and frame-decode tests
```
