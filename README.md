# ZART03 DIY Receiver

A low-cost (≈ €10) receiver for the **Schneider Electric Harmony Pocket Remote ZART03** (3-button handheld transmitter), built on an **ESP32-C6** — no original ZARB receiver needed.

The ZART remotes use a TI **CC2520** radio (IEEE 802.15.4, 2.4 GHz). The ESP32-C6 (and ESP32-H2) has a native 802.15.4 radio, so the receiver is just one small board and a relay module.

> ⚠️ **Not a safety device.** This receiver has no stop category, no safety-rated link supervision and no rolling code (a recorded frame can be replayed). Do **not** use it for cranes, hoists, gates near people or anything that can hurt someone. Use it for lights, hobby projects and the like.

---

## Features

- Pairing like the original receiver: **hold buttons 1 + 2** on the remote
- Up to **8 remotes**, one in control at a time (another takes over after 1 s of silence)
- Button N → relay N, **multiple buttons at once** supported
- **Momentary** (default) or **toggle** mode
- Relays drop immediately on the remote's release frame, or after 300 ms without signal
- Automatic **channel survey** (802.15.4 channels 11–26)
- Settings stored in flash (NVS) — survive power loss
- Serial console with a raw frame **sniffer** for diagnostics

---

## Hardware

| Part | Notes | Approx. price |
|---|---|---|
| ESP32-C6 Super Mini | Also works: Seeed XIAO ESP32C6, ESP32-C6-DevKitC-1, ESP32-H2 boards | € 4–6 |
| 3-channel relay module | 5 V, active-low type (most common) | € 3 |
| 5 V power supply | USB or 5 V to the relay module and board | – |

> The original ESP32 / ESP-WROOM-32 **cannot** be used: its radio only does Wi-Fi and Bluetooth, not 802.15.4.

### Wiring (default pins)

| Function | GPIO |
|---|---|
| Relay 1 | 0 |
| Relay 2 | 1 |
| Relay 3 | 2 |
| Status LED | 15 (on-board) |
| Pair / erase button | 9 (BOOT button) |

All pins are configurable at the top of the sketch (`RELAY_PIN`, `LED_PIN`, `LEARN_BTN_PIN`, `RELAY_ACTIVE_LOW`, `LED_ACTIVE_LOW`).

On a **Seeed XIAO ESP32C6**, set `XIAO_RF_SWITCH = true` to enable its on-board antenna switch (and `XIAO_EXT_ANTENNA = true` when using a U.FL antenna).

---

## Building

1. Install the **Arduino-ESP32** core 3.x or newer.
2. Select your board (e.g. *ESP32C6 Dev Module*).
3. Set these options in the **Tools** menu:
   - **USB CDC On Boot: Enabled** — otherwise the serial monitor on the USB port shows nothing
   - **Zigbee mode: Disabled** — the sketch drives the radio directly
4. Open `ZART03_Receiver_C6.ino` and upload.
5. Open the serial monitor at **115200 baud**.

No extra libraries are needed.

---

## First-time setup

### 1. Find the radio channel
Send `h` and **hold any remote button** for the whole survey (~7 s). The receiver hops channels 11–26 and stores the one where the remote was found (channel 15 on the unit used for development).

### 2. Pair the remote
Send `P` (or hold BOOT 2–5 s and release), then **hold buttons 1 + 2** on the remote:

```
>>> PAIR MODE: hold buttons 1 + 2 on the remote (10 s)
>>> paired as remote 1: FE -- 00 0F 00 0D A0 DD 00 00 00 -- 07 7E 04
```

Relays 1 and 2 stay off until you release the buttons after pairing.

### 3. Use it
```
remote 1: B1 pressed (rssi -62)
remote 1: released
remote 1: B1 B2 pressed (rssi -58)
remote 1: released (signal lost)
```
`(signal lost)` means the relays dropped because frames stopped arriving (out of range), not because of a release frame.

---

## Serial commands

| Key | Action |
|---|---|
| `h` | Channel survey (hold a remote button) |
| `+` / `-` | Channel up / down manually |
| `P` | Pair mode (then hold B1 + B2) |
| `E` | Erase all paired remotes |
| `p` | Print configuration |
| `t` | Toggle mode on/off |
| `s` | Sniff frames with RSSI ≥ −70 (diagnostics) |
| `a` | Sniff all frames (diagnostics) |

## BOOT button & LED

| BOOT button | Action |
|---|---|
| Hold 2–5 s, release | Pair mode (LED steady on while holding) |
| Hold ≥ 6 s, release | Erase all remotes (LED flickers while holding) |

| LED | Meaning |
|---|---|
| Blinking | Survey or pair mode active |
| On | At least one relay energised |

---

## Protocol

Decoded by sniffing a ZART03. Each frame is a 15-byte 802.15.4 PSDU (FCS not shown), **unencrypted**:

```
idx:  0  1   2  3  4  5  6  7  8  9 10  11   12 13 14
      FE SQ  00 0F 00 0D A0 DD 00 00 00 BB   07 7E 04
         │                              │
         │                              └─ button bitmask
         └─ sequence counter
```

| Field | Description |
|---|---|
| `FE` | Constant frame marker |
| `SQ` | Sequence counter — restarts at `01` on every press, wraps `FF → 00` |
| `BB` | Button bitmask: `01` = B1, `02` = B2, `04` = B3, `03` = B1+B2 (used for pairing), `00` = released |
| other bytes | Constant per remote — stored as the remote's ID/signature |

Behaviour:
- While a button is held the remote repeats the frame continuously.
- After release it sends 3–4 frames with `BB = 00`.
- Frames are simplex (remote → receiver only); the remote itself is never paired or changed.

Other ZART models (ZART04/06/08/10) likely use the same format with more bits in the button field — untested. Set `NUM_RELAYS` (max 8) accordingly.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Serial monitor shows nothing | Tools → **USB CDC On Boot: Enabled**, re-upload, press RESET |
| Compile error `redefinition of BOOT_PIN` | Use the current sketch (constant renamed to `LEARN_BTN_PIN`) |
| Survey finds 0 frames | Remote switch on "I" (top LED lights green when pressing), hold the button for the full survey |
| Pairing times out | Hold B1 + B2 close to the receiver; check with `a` that frames show `btn 03` |
| Relays inverted | Change `RELAY_ACTIVE_LOW` |
| LED inverted | Change `LED_ACTIVE_LOW` |
| Low RSSI / short range | Tiny chip antennas (Super Mini) limit range; orientation and your hand around the remote matter. A board with a U.FL antenna (e.g. XIAO ESP32C6) helps |

---

## License

Released under the [MIT License](LICENSE).

---

## Disclaimer

This is an independent hobby project, not affiliated with or endorsed by Schneider Electric. "Harmony" and "ZART" are product names of Schneider Electric. Use at your own risk.
