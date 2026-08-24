# CHANGELOG.md

This file is the **cross-domain engineering communication log** for RobotProject.

It is intentionally **English-only**.

Record only changes that the other Codex domain must know about, shared-interface changes, safety/architecture changes, or project milestones. Do not record trivial local implementation details.

## Entry format

```text
## YYYY-MM-DDTHH:MM:SS±HH:MM — Title

Domain: Firmware | ROS | Interface | System
Impact: <what the other domain must know>

Changed:
- ...

Action required:
- None
```

---

## 2026-08-23T20:25:00+08:00 — STM32 real-time control phase established

Domain: System
Impact: The high-computing perception/SLAM/Navigation v1 stack is frozen. Active development moves to the STM32 real-time control domain.

Changed:
- STM32G474RET6 / DeveBox STM32G474R Ver:20 selected as the real-time controller.
- Pin Allocation v1 is frozen.
- HSE baseline is 8 MHz and SYSCLK target is 170 MHz.
- TIM1 motor PWM target is 10 kHz.
- TIM2 and TIM3 are reserved for hardware quadrature encoders.
- SPI1 is reserved for ICM-42688-P.
- ADC1_IN6 on PC0 is reserved for battery measurement.
- USART2 is reserved for debug.
- FDCAN1 on PA11/PA12 is the CAN controller with 500 kbit/s nominal bring-up bitrate.
- Safe startup requires MOTOR_STBY LOW, direction GPIO LOW, and IMU_CS HIGH.

Action required:
- Firmware domain: complete CubeMX baseline and safe MCU bring-up.
- ROS domain: remain on the frozen high-computing baseline until interface/bridge integration begins.
