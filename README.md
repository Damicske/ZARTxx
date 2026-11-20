# ZART DIY Receiver

A low-cost (≈ €10) receiver for the **Schneider Electric Harmony Pocket Remote** handheld transmitters **ZART03 / ZART04 / ZART06 / ZART08 / ZART10**, built on an **ESP32-C6** — no original ZARB receiver needed.

The ZART remotes use a TI **CC2520** radio (IEEE 802.15.4, 2.4 GHz). The ESP32-C6 (and ESP32-H2) has a native 802.15.4 radio, so the receiver is just one small board and a relay module.

> ⚠️ **Not a safety device.** This receiver has no stop category, no safety-rated link supervision and no rolling code (a recorded frame can be replayed). Do **not** use it for cranes, hoists, gates near people or anything that can hurt someone. Use it for lights, hobby projects and the like.

---

## Supported remotes

| Remote | Buttons | `NUM_RELAYS` | Status |
|---|---|---|---|
| ZART03 | 3 | 3 | ✅ Tested |
| ZART04 | 4 | 4 | Expected to work (same frame format) |
| ZART06 | 6 | 6 | Expected to work |
| ZART08 | 8 | 8 | Expected to work |
| ZART10 | 10 | 10 | Expected to work — buttons 9/10 assumed in the high button byte (see [Protocol](#protocol)) |

`NUM_RELAYS` sets how many relay outputs are driven (1–10). Buttons without a relay are ignored. One receiver works with any mix of remotes; set `NUM_RELAYS` to the largest model you use.

---

## Features

- Pairing like the original receiver: **hold buttons 1 + 2** on the remote
- Up to **8 remotes**, one in control at a time (another takes over after 1 s of silence)
- Up to **10 relays**: button N → relay N, **multiple buttons at once** supported
- Optional **"any button" relay** (like relay 11 of a ZARB11WSP): on while any relay is on
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
| Relay module (1–10 channels) | 5 V, active-low type (most common); one channel per remote button | € 3–8 |
| 5 V power supply | USB or 5 V to the relay module and board | – |

> The original ESP32 / ESP-WROOM-32 **cannot** be used: its radio only does Wi-Fi and Bluetooth, not 802.15.4.

### Wiring (default pins)

| Function | GPIO |
|---|---|
| Relay 1 … 8 | 0, 1, 2, 3, 4, 5, 6, 7 |
| Relay 9, 10 | 18, 19 |
| "Any" relay (optional) | off by default (`ANY_RELAY_PIN = -1`), e.g. 20 |
| Status LED | 15 (on-board) |
| Pair / erase button | 9 (BOOT button) |

Only the first `NUM_RELAYS` pins are used. All pins are configurable at the top of the sketch (`RELAY_PIN`, `ANY_RELAY_PIN`, `LED_PIN`, `LEARN_BTN_PIN`, `RELAY_ACTIVE_LOW`, `LED_ACTIVE_LOW`). Avoid the strapping pins 8, 9 and 15 and the USB pins 12/13 for relays.

On a **Seeed XIAO ESP32C6**, set `XIAO_RF_SWITCH = true` to enable its on-board antenna switch (and `XIAO_EXT_ANTENNA = true` when using a U.FL antenna). That switch uses GPIO3 and GPIO14, so move relay 4 (GPIO3) to another pin.

---

## Building

1. Install the **Arduino-ESP32** core 3.x or newer.
2. Select your board (e.g. *ESP32C6 Dev Module*).
3. Set these options in the **Tools** menu:
   - **USB CDC On Boot: Enabled** — otherwise the serial monitor on the USB port shows nothing
   - **Zigbee mode: Disabled** — the sketch drives the radio directly
4. Open `ZART03_Receiver_C6.ino`, set `NUM_RELAYS` for your remote, and upload.
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
>>> paired as remote 1: FE -- 00 0F 00 0D A0 DD 00 00 -- -- 07 7E 04
```

Relays 1 and 2 stay off until you release the buttons after pairing. Repeat for each additional remote (up to 8).

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

Sniff is off by default and is switched off by pair mode. While sniff is on, the short event lines are not printed.

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
idx:  0  1   2  3  4  5  6  7  8  9   10 11   12 13 14
      FE SQ  00 0F 00 0D A0 DD 00 00  BH BL   07 7E 04
         │                            └─┬─┘
         │                              └─ button bitmask (16 bit)
         └─ sequence counter
```

| Field | Description |
|---|---|
| `FE` | Constant frame marker |
| `SQ` | Sequence counter — restarts at `01` on every press, wraps `FF → 00` |
| `BL` | Buttons 1–8: `01` = B1, `02` = B2, `04` = B3, … `80` = B8 · `03` = B1+B2 (pairing) · `00` = released |
| `BH` | Buttons 9–10 (**assumed**): `01` = B9, `02` = B10 — always `00` on a ZART03 |
| other bytes | Constant per remote — stored as the remote's ID/signature |

Behaviour:
- While a button is held the remote repeats the frame continuously.
- After release it sends 3–4 frames with buttons = `0000`.
- Frames are simplex (remote → receiver only); the remote itself is never paired or changed.

**Verified on a ZART03 only.** If you have a ZART08 or ZART10, please sniff it (`a`) and open an issue with a few frames per button — especially B9 and B10 — so the format can be confirmed. If they turn out to use a different byte, only `IDX_BTN_HI` / `IDX_BTN_LO` in the sketch need to change.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Serial monitor shows nothing | Tools → **USB CDC On Boot: Enabled**, re-upload, press RESET |
| Compile error `redefinition of BOOT_PIN` | Use the current sketch (constant renamed to `LEARN_BTN_PIN`) |
| Survey finds 0 frames | Remote switch on "I" (top LED lights green when pressing), hold the button for the full survey |
| Pairing times out | Hold B1 + B2 close to the receiver; check with `a` that frames show `btn 0003` |
| A button does nothing | Check `NUM_RELAYS` covers that button and the relay pin is wired |
| Relays inverted | Change `RELAY_ACTIVE_LOW` |
| LED inverted | Change `LED_ACTIVE_LOW` |
| Low RSSI / short range | Tiny chip antennas (Super Mini) limit range; orientation and your hand around the remote matter. A board with a U.FL antenna (e.g. XIAO ESP32C6) helps |

---

## License

Released under the [MIT License](LICENSE).

---

## Disclaimer

This is an independent hobby project, not affiliated with or endorsed by Schneider Electric. "Harmony" and "ZART" are product names of Schneider Electric. Use at your own risk.
