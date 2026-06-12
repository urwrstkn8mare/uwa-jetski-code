# Rudder encoder protocol notes (Briter, CANopen variant)

**Discovered 2026-06-12 while debugging rudder zeroing.** The fitted Briter
encoder is the **CANopen variant**, not the plain "CANbus" (CAN2.0B) variant
that `useful/encoder_manual.pdf` documents. That manual is the wrong one for
this device — keep it only as a reference for the private read command below.

## Evidence (from the web UI CAN bus monitor)

| ID    | What it is | Observation |
|-------|------------|-------------|
| 0x181 | CANopen TPDO1 (0x180 + node id 1): cyclic position broadcast, raw 32-bit LE count in bytes 0-3, DLC 4, every ~20 ms | Works; this is the live readout |
| 0x001 | Reply to the CAN2.0B manual's 0x01 read-value command (`07 01 01 <count LE>`) | Answered exactly once (the boot-time read). **Write commands (0x04 set-mode, 0x05 set-period, 0x06 set-zero) on this ID are silently ignored — no ack, no effect** |
| 0x581 | CANopen SDO response (0x580 + node id) | SDO read of object 0x1000 returned `43 00 10 00 96 01 01 00` → device type 0x00010196 = **CiA 406 encoder profile (0x196 = 406), single-turn** |

Supporting evidence: the firmware commands a 50 ms auto-return period at every
boot via the CAN2.0B 0x05 command, yet the encoder keeps broadcasting every
20 ms — the write never takes effect.

So the device speaks CANopen (SDO/PDO) and additionally answers the private
CAN2.0B *read* for compatibility, which is what made the variant hard to spot:
reads work, writes vanish.

## How to talk to it

- Node id: 1. SDO request → **0x601**, SDO response → **0x581** (0x600/0x580 + node id).
- Position: TPDO1 on **0x181**, raw 32-bit LE count, no header bytes.
- **Zeroing**: CANopen expedited SDO write of 0 to the CiA 406 preset object
  **0x6003:00**: send `601: 23 03 60 00 00 00 00 00`, expect `581: 60 03 60 00 ...`.
  A `581: 80 03 60 00 <abort LE>` is an SDO abort (code tells you why).
- **Persistence**: store parameters with the standard signature `"save"`
  (0x65766173) to **0x1010:01**: send `601: 23 10 10 01 73 61 76 65`.
- Changing the broadcast period would be an SDO write to the TPDO1 event timer
  (0x1800:05), untested.

This is exactly what `shared_components/encoder_can/encoder_can.c` implements
(`encoder_can_zero()`): SDO preset + store first, falling back to the CAN2.0B
0x04/0x06 sequence only if SDO gets no response (i.e. a non-CANopen encoder is
fitted some day). The web UI "CAN bus monitor" panel (Rudder zero card,
`/api/can/stats`) shows every CAN ID heard with its latest payload plus TX
ok/fail counters, and the Zero Rudder button reports per-step results,
including SDO abort codes.

## Gotchas learned the hard way

- The CAN2.0B manual's 0x04 set-mode ack echoes the written mode byte instead
  of returning a 0x00 status (relevant only to the fallback path).
- The CAN2.0B manual's minimum auto-return period is 50 ms; 20 ms was being
  commanded historically. Irrelevant for the CANopen variant (its 20 ms TPDO
  timer is fine) but the Kconfig minimum is now 50 ms anyway.
- `json_error_resp()` in the web UI used to send HTTP 200, so every failure
  looked like success in the browser ("Zeroed!" while nothing happened).
