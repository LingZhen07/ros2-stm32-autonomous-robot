#!/bin/bash
set -euo pipefail

IMG=/data/projects/can3-dt/offline-index11-20260830/dt-can3-0x280b-signed.img
EXPECTED_SHA256=35be12871218a2e5da2bca645b2de4204dc4c6e959eb6a85dc1f4ac9ec9a2316
VENDOR_UPDATE=/opt/opi_test/dt_img/update.sh
EXPECTED_UPDATE_SHA256=dd7877922e3fd8a75d027f22457e5fe95a159b6a2c0b9bc1d3363445630cd144
PREFLASH=/data/projects/can3-dt/preflash-20260830

root_source="$(findmnt -n -o SOURCE / | sed 's~\[.*\]~~')"
root_parent="$(lsblk -n -o PKNAME /dev/mmcblk1p2 | head -n1)"
test "$root_source" = /dev/mmcblk1p2
test "$root_parent" = mmcblk1
test -b /dev/mmcblk1

test -f "$IMG"
test "$(stat -c %s "$IMG")" = 735488
echo "$EXPECTED_SHA256  $IMG" | sha256sum -c -
echo "$EXPECTED_UPDATE_SHA256  $VENDOR_UPDATE" | sha256sum -c -

# Refuse to overwrite any previous preflash evidence.
test ! -e "$PREFLASH"
mkdir "$PREFLASH"

sudo -v

# Back up exactly the two 4096-sector slots used by the vendor script.
sudo dd if=/dev/mmcblk1 \
  of="$PREFLASH/dt-slot-114688-before.bin" \
  bs=512 skip=114688 count=4096 iflag=fullblock status=progress

sudo dd if=/dev/mmcblk1 \
  of="$PREFLASH/dt-slot-376832-before.bin" \
  bs=512 skip=376832 count=4096 iflag=fullblock status=progress

sudo sha256sum \
  "$PREFLASH/dt-slot-114688-before.bin" \
  "$PREFLASH/dt-slot-376832-before.bin" \
  | tee "$PREFLASH/backup-SHA256SUMS"

# Preserve the vendor update.sh write semantics exactly: count/seek/bs,
# no conv option, and only the two confirmed raw DT slots.
sudo dd if="$IMG" of=/dev/mmcblk1 count=4096 seek=114688 bs=512 status=progress
sudo dd if="$IMG" of=/dev/mmcblk1 count=4096 seek=376832 bs=512 status=progress
sync

# 735488 bytes is exactly 2873 blocks of 256 bytes. The skip values below
# address the same byte offsets as sectors 114688 and 376832 at bs=512.
sudo dd if=/dev/mmcblk1 \
  of="$PREFLASH/dt-slot-114688-readback.img" \
  bs=256 skip=229376 count=2873 iflag=fullblock status=progress

sudo dd if=/dev/mmcblk1 \
  of="$PREFLASH/dt-slot-376832-readback.img" \
  bs=256 skip=753664 count=2873 iflag=fullblock status=progress

test "$(sudo stat -c %s "$PREFLASH/dt-slot-114688-readback.img")" = 735488
test "$(sudo stat -c %s "$PREFLASH/dt-slot-376832-readback.img")" = 735488

echo "$EXPECTED_SHA256  $PREFLASH/dt-slot-114688-readback.img" \
  | sudo sha256sum -c -
echo "$EXPECTED_SHA256  $PREFLASH/dt-slot-376832-readback.img" \
  | sudo sha256sum -c -

sudo sha256sum \
  "$PREFLASH/dt-slot-114688-readback.img" \
  "$PREFLASH/dt-slot-376832-readback.img" \
  | tee "$PREFLASH/readback-SHA256SUMS"

echo CAN3_DT_FLASH_AND_READBACK=PASS
echo REBOOTING
sudo reboot
