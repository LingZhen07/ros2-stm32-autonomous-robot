#!/usr/bin/env bash
set -euo pipefail

readonly CAN_INTERFACE="can3"
readonly CAN_CONTROLLER="822d0000.mttcan"
readonly IP_BIN="${IP_BIN:-$(command -v ip || true)}"

if [[ ! -x "${IP_BIN}" ]]; then
  echo "robot-can3: ip tool not found at ${IP_BIN}" >&2
  exit 1
fi

for ((attempt = 0; attempt < 40; ++attempt)); do
  [[ -e "/sys/class/net/${CAN_INTERFACE}/device" ]] && break
  sleep 0.25
done

if [[ ! -e "/sys/class/net/${CAN_INTERFACE}/device" ]]; then
  echo "robot-can3: ${CAN_INTERFACE} was not created" >&2
  exit 1
fi

device_path="$(readlink -f "/sys/class/net/${CAN_INTERFACE}/device")"
if [[ "${device_path}" != */"${CAN_CONTROLLER}" ]]; then
  echo "robot-can3: refusing unexpected mapping ${CAN_INTERFACE} -> ${device_path}" >&2
  exit 1
fi

details="$(${IP_BIN} -details link show dev "${CAN_INTERFACE}" 2>/dev/null || true)"
if grep -Eq '<[^>]*UP[^>]*>' <<<"${details}" &&
   grep -q 'mtu 72' <<<"${details}" &&
   grep -Eq 'can <[^>]*BERR-REPORTING[^>]*>' <<<"${details}" &&
   grep -Eq 'can <[^>]*FD[^>]*> state ERROR-ACTIVE' <<<"${details}" &&
   grep -Eq 'bitrate 500000 sample-point 0\.800([[:space:]]|$)' <<<"${details}" &&
   grep -Eq 'dbitrate 2000000 dsample-point 0\.825([[:space:]]|$)' <<<"${details}"; then
  echo "robot-can3: ${CAN_INTERFACE} already ready"
  exit 0
fi

${IP_BIN} link set dev "${CAN_INTERFACE}" down
${IP_BIN} link set dev "${CAN_INTERFACE}" type can \
  bitrate 500000 sample-point 0.800 \
  dbitrate 2000000 dsample-point 0.825 \
  fd on berr-reporting on
${IP_BIN} link set dev "${CAN_INTERFACE}" up

details="$(${IP_BIN} -details link show dev "${CAN_INTERFACE}")"
grep -Eq '<[^>]*UP[^>]*>' <<<"${details}"
grep -q 'mtu 72' <<<"${details}"
grep -Eq 'can <[^>]*BERR-REPORTING[^>]*>' <<<"${details}"
grep -Eq 'can <[^>]*FD[^>]*> state ERROR-ACTIVE' <<<"${details}"
grep -Eq 'bitrate 500000 sample-point 0\.800([[:space:]]|$)' <<<"${details}"
grep -Eq 'dbitrate 2000000 dsample-point 0\.825([[:space:]]|$)' <<<"${details}"

echo "robot-can3: ${CAN_INTERFACE} ready at 500k@80% / 2M@82.5% CAN FD"
