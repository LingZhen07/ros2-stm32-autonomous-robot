# Orange Pi AI Pro CAN3 Device Tree

Status: **INSTALLED / BOOT VERIFIED / FROZEN**

This directory retains the reviewed Device Tree source and one-time installation tooling that
enabled the production CAN3 path. It is maintenance evidence, not a routine boot-time setup.

## Frozen production mapping

```text
40-pin header pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042 TXD
40-pin header pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042 RXD
CAN_TX3 pinctrl: offset 0x40, function 1
CAN_RX3 pinctrl: offset 0x44, function 1
mttcan@3 / mttcan-id=3 / 822d0000.mttcan -> SocketCAN can3
```

The live system has booted the CAN3 tree and probed `822d0000.mttcan`. The old
`822c0000.mttcan` / CAN2 path is historical and is not a production fallback.

## Source and container method

The matching Ascend 25.2.0 HDK package is retained on the Orange Pi under
`/data/projects/can3-dt/`. The accepted update started from the current official signed TF
container and replaced only board-ID `0x280B` entry index 11 in its fixed 55,296-byte DTB slot.
The HSDT index, offsets, metadata, all other board DTBs, and outer vendor signing format were
preserved. The failed historical approach of rebuilding and writing a complete vendor DT package
must not be repeated.

Tracked evidence:

| File | Purpose |
|---|---|
| `current_tf/entry-11-80-00-28-0b-original.dts` | Original board-ID `0x280B` tree |
| `current_tf/entry-11-80-00-28-0b-can3.dts` | Minimal CAN3 board tree |
| `verify_hsdt_can3.py` | Offline container/entry/diff verification |
| `flash_can3_dt_once.sh` | Audited one-time flash/readback workflow; not a startup service |

The installed Device Tree is frozen. Do not rerun the flash script or write raw TF slots during
normal operation. Any future Device Tree change is a separate privileged recovery-aware decision.

## Runtime boundary

Device Tree only exposes the controller and pinmux. Production link timing is owned by
`robot_stm32_bridge/systemd/robot-can3.service` and its idempotent `configure_can3.sh`:

```text
500 kbit/s nominal, 80.0% sample point
2 Mbit/s data, 82.5% sample point
CAN FD enabled; BRS set per Protocol 1.0 frame
```

Real Orange Pi <-> STM32 Protocol 1.0 traffic is accepted with SocketCAN Error Active, zero TX/RX
errors, and zero bus-off events. The robot remained DISARMED.
