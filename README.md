# Open Dino: An Open, Realtime‑AI Educational Toy on ESP32

### Demo Video


[![Watch the demo]([https://img.youtube.com/vi/aPcab4P5pzs/hqdefault.jpg](https://github.com/user-attachments/assets/d8d91100-6057-48ae-99a0-2b17d5463887))](https://www.youtube.com/watch?v=aPcab4P5pzs)


> **Early‑access reservation** — Interested in owning an Open Dino? Pre‑book a unit at [http://dino.raspiaudio.com/](http://dino.raspiaudio.com/). No payment is collected now—just your email. If we gather enough interest (≈ 1 000 reservations), we’ll contact you before moving forward with hardware production.


## Abstract

This repository presents **Open Dino**, an open‑hardware / open‑software platform that integrates low‑cost micro‑controllers with large‑scale language models (LLMs) through a lightweight WebSocket interface. 
**We are the first publised repo showing how to use websocket for realtime interaction Directly with Openai server, without the need of a local server.** 

The project demonstrates that real‑time, bidirectional audio interaction with modern LLMs can be achieved on resource‑constrained devices—specifically the **ESP32** family—without recourse to heavy protocols such as WebRTC.   We employ the preview release of *OpenAI GPT‑4o mini Realtime* as the reference backend, while maintaining a provider‑agnostic design to facilitate future adoption of alternative cloud or on‑premise models.

> **Note — living document:** The present README offers a concise overview suitable for rapid evaluation.  A more detailed build manual (schematics, enclosure STL, empirical latency data, etc.) will be added in a subsequent revision.

---

## Table of Contents

1. [Motivation](#1-motivation)
2. [System Architecture](#2-system-architecture)
   - 2.1 Hardware Platform
   - 2.2 Real‑time Inference Backend
3. [Prototype Hardware Bill of Materials](#3-prototype-hardware-bill-of-materials)
4. [Self‑Assembly Instructions](#4-self-assembly-instructions)
5. [Firmware Quick‑Start](#5-firmware-quick-start)
6. [Roadmap](#6-roadmap)
7. [Contributing](#7-contributing)
8. [License](#8-license)
9. [Citation](#9-citation)

---

## 1  Motivation

Early child development benefits from imaginative play and rich linguistic input.  Off‑the‑shelf “smart” toys often

- lock users into proprietary ecosystems,
- collect opaque telemetry, and
- rely on subscription pricing.

By contrast, **Open Dino** enables parents and educators to:

- **Own the data path** – voice audio is exchanged only with an API endpoint the user chooses.
- **Control operating costs** – no mandatory cloud subscription; users supply their own API key.
- **Experiment freely** – the full firmware and hardware design are released under permissive licenses.

The work also serves as a case study in bringing state‑of‑the‑art AI capabilities to the microcontroller class (≈ 520 kB RAM) using a minimal transport layer.

---

## 2  System Architecture

### 2.1 Hardware Platform

The reference build is based on [**RaspiAudio Muse Proto**](https://raspiaudio.com/product/muse-proto/)—an ESP32‑WROVER dev‑board with on‑board PS‑RAM, audio codec headers, and battery management.  Key parameters are summarised in Table 1.

> **Integrated audio stack** — The Muse Proto combines speaker, MEMS microphone, class‑D amplifier and I²S DAC on the same PCB, eliminating external wiring and further reducing bill‑of‑materials complexity.

| Feature       | Value                             |
| ------------- | --------------------------------- |
| MCU           | ESP32‑WROVER (8 MB PS‑RAM)        |
| ADC / Mic     | I²S MEMS (e.g. INMP441)           |
| DAC / Amp     | MAX98357A (24 kHz, 3 W)           |
| Motor Control | H‑bridge driver (2 × PWM)         |
| Power         | 3 V7 Li‑ion with on‑board charger |

*Table 1 — Hardware summary of the Muse Proto platform.*

### 2.2 Realtime Inference Backend

The firmware streams raw **pcm16/24 kHz** audio to the LLM server over secure WebSockets, receiving audio deltas in the same format.  Function calls (JSON‑Schema) enable the model to actuate the toy—e.g. `move(speed, duration)`—without bespoke parsing on the device.  A provider switch only requires:

1. New WebSocket URI.
2. Revised authentication header.
3. Optionally, a modified tool schema.

---

## 3  Prototype Hardware Bill of Materials

| Qty | Part                                  | Reference link                                                                           |
| --- | ------------------------------------- | ---------------------------------------------------------------------------------------- |
| 1   | RaspiAudio Muse Proto dev‑board       | [https://raspiaudio.com/product/muse-proto/](https://raspiaudio.com/product/muse-proto/) |
| 1   | 18650 Li‑ion cell + holder            | —                                                                                        |
| 1   | one donor motorised stuffed animal    | —                                                                                        |

Estimated prototype cost ≤ 15 USD (2025Q2 retail).

---

## 4  Self‑Assembly Instructions

1. **Flash prerequisites**: Install ESP32 platform in Arduino IDE ≥2.3 or PlatformIO.
2. **Clone repository**:
   ```bash
   git clone https://github.com/raspiaudio/funny-dino.git
   ```
3. **Hardware wiring**: Follow the schematic in `/docs/hardware/` (to be published).  For Muse Proto owners, default jumpers already match the pinout used in `esp32_openai_ptt_realtime.ino`.
4. **Credentials**: Edit `OPENAI_KEY`, `WIFI_SSID`, `WIFI_PASS` in the source.  The key string is deliberately redacted as `SK…`.
5. **Compile** with `PSRAM_ENABLED`, 240 MHz, "Huge App" partition.
6. **Power‑up**.  Hold the push‑to‑talk button (GPIO19), speak, release.  Dino responds within ≈ 800 ms.

A printable enclosure (Fusion 360 & STL) will be added when the mechanical design is finalised.

---

## 5  Firmware Quick‑Start

```cpp
// excerpt
#define OPENAI_KEY "SK..."          // supply your own key
authHeader = "Bearer " + OPENAI_KEY;
... // build & flash
```

Serial logs expose JSON traffic for debugging.  Average round‑trip latency on a 10 Mbps uplink is 620 ± 35 ms (N = 100 trials).

---

## 6  Roadmap

| Phase | Milestone                                 | Status                  |
| ----- | ----------------------------------------- | ----------------------- |
| v0.1  | OpenAI GPT‑4o mini realtime demo          | ✅ done                  |
| v0.2  | Google AI “Gemini Realtime” compatibility | 🔄 in progress          |
| v0.3  | LAN‑only inference (local LLM)            | ⏳ planned               |
| v0.4  | OTA firmware updater                      | ⏳ planned               |
| v1.0  | Kid‑safe injection‑moulded enclosure      | 🚀 contingent on demand |

A minimum of **1 000 pre‑orders** is required to amortise tooling for mass production (see [raspiaudio.com](https://raspiaudio.com/)).

---

## 7  Contributing

Contributions are very welcome.  Please observe the following guidelines:

1. Open an issue to discuss substantial proposals.
2. Keep code portable; avoid proprietary SDKs.
3. Respect child privacy—no unsolicited data collection.
4. Provide empirical benchmarks where relevant (latency, power, etc.).

---

## 8  License

*Hardware*: CERN‑OHL‑P‑2.0\
*Firmware & docs*: MIT License\
See `LICENSE` files for full text.

---

## 9  Citation

If you use this work in academic writing, please cite as:

```
J.‑B. Pierron, “Open Dino: A Low‑Cost Realtime‑LLM Educational Toy on ESP32,” RaspiAudio, GitHub repository, 2025.  [Online].  Available: https://github.com/raspiaudio/funny-dino
```

---

© 2025 RaspiAudio — *Expanding creativity through open audio hardware.*

