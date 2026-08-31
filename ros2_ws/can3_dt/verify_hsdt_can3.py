#!/usr/bin/env python3
"""Verify an offline CAN3 replacement in the vendor HSDT DT container."""

from __future__ import annotations

import argparse
import hashlib
import struct
from pathlib import Path


SIGNED_HEADER_SIZE = 8448
IMAGE_PACK_HEADER_SIZE = 8192
ESBC_HEADER_SIZE = 256
HSDT_HEADER = struct.Struct("<4sII")
HSDT_ENTRY = struct.Struct("<4s4sIIIIQQ")
FDT_MAGIC = 0xD00DFEED

TARGET_INDEX = 11
TARGET_BOARD_ID = bytes.fromhex("80 00 28 0b")
TARGET_RELATIVE_OFFSET = 391168
TARGET_SLOT_SIZE = 55296
EXPECTED_ORIGINAL_SIGNED_SHA256 = (
    "820228f93773b6c2cc68df109d1dbd1d4665860f74d3b401f517aaa3432477c5"
)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def parse_hsdt(payload: bytes) -> tuple[int, list[tuple[object, ...]]]:
    magic, version, count = HSDT_HEADER.unpack_from(payload)
    require(magic == b"HSDT", f"bad HSDT magic: {magic!r}")
    require(version == 1, f"bad HSDT version: {version}")
    require(count == 17, f"bad HSDT entry count: {count}")
    entries = [
        HSDT_ENTRY.unpack_from(payload, HSDT_HEADER.size + index * HSDT_ENTRY.size)
        for index in range(count)
    ]
    table_end = HSDT_HEADER.size + count * HSDT_ENTRY.size
    require(payload[table_end : table_end + 4] == b"\0\0\0\0", "bad table terminator")
    return version, entries


def fdt_total_size(slot: bytes) -> int:
    magic, total_size = struct.unpack_from(">II", slot)
    require(magic == FDT_MAGIC, f"bad FDT magic: 0x{magic:08x}")
    require(0 < total_size <= len(slot), f"bad FDT total size: {total_size}")
    return total_size


def changed_summary(before: bytes, after: bytes, start: int, end: int) -> str:
    changed = [offset for offset in range(start, end) if before[offset] != after[offset]]
    if not changed:
        return f"[{start},{end}): 0 changed bytes"
    return (
        f"[{start},{end}): {len(changed)} changed bytes, "
        f"first={changed[0]}, last={changed[-1]}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("original_signed", type=Path)
    parser.add_argument("original_payload", type=Path)
    parser.add_argument("patched_payload", type=Path)
    parser.add_argument("candidate_signed", type=Path)
    parser.add_argument("expected_can3_dtb", type=Path)
    parser.add_argument("extracted_can3_dtb", type=Path)
    parser.add_argument("--reproduction", type=Path)
    args = parser.parse_args()

    original_signed = args.original_signed.read_bytes()
    original_payload = args.original_payload.read_bytes()
    patched_payload = args.patched_payload.read_bytes()
    candidate_signed = args.candidate_signed.read_bytes()
    expected_can3_dtb = args.expected_can3_dtb.read_bytes()

    require(sha256(original_signed) == EXPECTED_ORIGINAL_SIGNED_SHA256,
            "original signed container SHA-256 mismatch")
    require(len(original_signed) == len(candidate_signed) == 735488,
            "signed container size mismatch")
    require(len(original_payload) == len(patched_payload) == 727040,
            "HSDT payload size mismatch")
    require(original_signed[SIGNED_HEADER_SIZE:] == original_payload,
            "original payload is not the current signed container payload")
    require(candidate_signed[SIGNED_HEADER_SIZE:] == patched_payload,
            "candidate payload does not match patched payload")
    if args.reproduction:
        require(args.reproduction.read_bytes() == original_signed,
                "official sign_image reproduction differs from current official container")

    original_version, original_entries = parse_hsdt(original_payload)
    patched_version, patched_entries = parse_hsdt(patched_payload)
    require(original_version == patched_version == 1, "HSDT version changed")
    require(original_entries == patched_entries, "HSDT entry metadata changed")

    target = patched_entries[TARGET_INDEX]
    board_id, reserved, dtb_size, vrl_size, dtb_offset, vrl_offset, dtb_file, vrl_file = target
    require(board_id == TARGET_BOARD_ID, f"index 11 board-id changed: {board_id.hex()}")
    require(reserved == b"\0" * 4, "index 11 reserved field changed")
    require(dtb_size == TARGET_SLOT_SIZE, f"index 11 slot size changed: {dtb_size}")
    require(dtb_offset == TARGET_RELATIVE_OFFSET,
            f"index 11 offset changed: {dtb_offset}")
    require((vrl_size, vrl_offset, dtb_file, vrl_file) == (0, 0, 0, 0),
            "index 11 VRL/file metadata changed")

    slot_start = dtb_offset
    slot_end = slot_start + dtb_size
    require(original_payload[:slot_start] == patched_payload[:slot_start],
            "payload differs before index 11 slot")
    require(original_payload[slot_end:] == patched_payload[slot_end:],
            "payload differs after index 11 slot")

    original_slot = original_payload[slot_start:slot_end]
    patched_slot = patched_payload[slot_start:slot_end]
    original_dtb_size = fdt_total_size(original_slot)
    patched_dtb_size = fdt_total_size(patched_slot)
    original_dtb = original_slot[:original_dtb_size]
    patched_dtb = patched_slot[:patched_dtb_size]
    require(patched_dtb == expected_can3_dtb, "extracted CAN3 DTB differs from build output")
    require(set(original_slot[original_dtb_size:]) <= {0}, "original slot padding is not zero")
    require(set(patched_slot[patched_dtb_size:]) <= {0}, "patched slot padding is not zero")
    args.extracted_can3_dtb.write_bytes(patched_dtb)

    # Validate both deterministic vendor digest layers produced by sign_image().
    outer_code_hash = candidate_signed[64:96]
    outer_header_hash = candidate_signed[0x560:0x580]
    inner_code_hash = candidate_signed[IMAGE_PACK_HEADER_SIZE + 24:
                                       IMAGE_PACK_HEADER_SIZE + 56]
    inner_hash_padding = candidate_signed[IMAGE_PACK_HEADER_SIZE + 56:
                                          IMAGE_PACK_HEADER_SIZE + 88]
    inner_code_offset, inner_code_length = struct.unpack_from(
        "<II", candidate_signed, IMAGE_PACK_HEADER_SIZE + 88
    )
    require(outer_code_hash == hashlib.sha256(candidate_signed[IMAGE_PACK_HEADER_SIZE:]).digest(),
            "outer image hash is invalid")
    require(outer_header_hash == hashlib.sha256(candidate_signed[:0x560]).digest(),
            "outer header hash is invalid")
    require(inner_code_hash == hashlib.sha256(patched_payload).digest(),
            "ESBC payload hash is invalid")
    require(inner_hash_padding == b"\0" * 32, "ESBC hash padding is invalid")
    require(inner_code_offset == ESBC_HEADER_SIZE, "ESBC code offset is invalid")
    require(inner_code_length == len(patched_payload), "ESBC code length is invalid")

    # The official deterministic packaging may change only these digest fields plus the target slot.
    allowed_regions = [
        (64, 96, "outer code hash"),
        (0x560, 0x580, "outer header hash"),
        (IMAGE_PACK_HEADER_SIZE + 24, IMAGE_PACK_HEADER_SIZE + 56, "ESBC payload hash"),
        (SIGNED_HEADER_SIZE + slot_start, SIGNED_HEADER_SIZE + slot_end, "index 11 slot"),
    ]
    unexpected = []
    for offset, (before, after) in enumerate(zip(original_signed, candidate_signed)):
        if before == after:
            continue
        if not any(start <= offset < end for start, end, _ in allowed_regions):
            unexpected.append(offset)
    require(not unexpected,
            f"signed container changed outside allowed regions: {unexpected[:16]}")

    print("VERIFY=PASS")
    print(f"original_signed_sha256={sha256(original_signed)}")
    print(f"original_payload_sha256={sha256(original_payload)}")
    print(f"patched_payload_sha256={sha256(patched_payload)}")
    print(f"candidate_signed_sha256={sha256(candidate_signed)}")
    print(f"hsdt_offset={SIGNED_HEADER_SIZE}")
    print(f"hsdt_version={patched_version}")
    print(f"hsdt_entry_count={len(patched_entries)}")
    print(f"target_index={TARGET_INDEX}")
    print(f"target_board_id={board_id.hex(' ')}")
    print(f"target_relative_offset={dtb_offset}")
    print(f"target_absolute_offset={SIGNED_HEADER_SIZE + dtb_offset}")
    print(f"target_slot_size={dtb_size}")
    print(f"original_dtb_sha256={sha256(original_dtb)}")
    print(f"original_dtb_size={original_dtb_size}")
    print(f"patched_dtb_sha256={sha256(patched_dtb)}")
    print(f"patched_dtb_size={patched_dtb_size}")
    print(f"remaining_padding={dtb_size - patched_dtb_size}")
    print("unsigned_payload_outside_target_slot=IDENTICAL")
    print("hsdt_entry_metadata_all_17=IDENTICAL")
    for start, end, label in allowed_regions:
        print(f"signed_diff_{label.replace(' ', '_')}={changed_summary(original_signed, candidate_signed, start, end)}")
    print("signed_diff_outside_allowed_regions=0")
    print("outer_code_hash=VALID")
    print("outer_header_hash=VALID")
    print("esbc_payload_hash=VALID")


if __name__ == "__main__":
    main()
