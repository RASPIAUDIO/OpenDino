Open Dino: An Open, Realtime‑AI Educational Toy on ESP32

Abstract

This repository presents Funny Dino, an open‑hardware / open‑software platform that integrates low‑cost micro‑controllers with large‑scale language models (LLMs) through a lightweight WebSocket interface.  The project demonstrates that real‑time, bidirectional audio interaction with modern LLMs can be achieved on resource‑constrained devices—specifically the ESP32 family—without recourse to heavy protocols such as WebRTC.  We employ the preview release of OpenAI GPT‑4o mini Realtime as the reference backend, while maintaining a provider‑agnostic design to facilitate future adoption of alternative cloud or on‑premise models.

Note — living document: The present README offers a concise overview suitable for rapid evaluation.  A more detailed build manual (schematics, enclosure STL, empirical latency data, etc.) will be added in a subsequent revision.

Table of Contents

Motivation

System Architecture

2.1 Hardware Platform

2.2 Real‑time Inference Backend

Prototype Hardware Bill of Materials

Self‑Assembly Instructions

Firmware Quick‑Start

Roadmap

Contributing

License

Citation

1  Motivation

Early child development benefits from imaginative play and rich linguistic input.  Off‑the‑shelf “smart” toys often

lock users into proprietary ecosystems,

collect opaque telemetry, and

rely on subscription pricing.

By contrast, Funny Dino enables parents and educators to:

Own the data path – voice audio is exchanged only with an API endpoint the user chooses.

Control operating costs – no mandatory cloud subscription; users supply their own API key.

Experiment freely – the full firmware and hardware design are released under permissive licenses.

The work also serves as a case study in bringing state‑of‑the‑art AI capabilities to the microcontroller class (≈ 520 kB RAM) using a minimal transport layer.

2  System Architecture

2.1 Hardware Platform

The reference build is based on RaspiAudio Muse Proto—an ESP32‑WROVER dev‑board with on‑board PS‑RAM, audio codec headers, and battery management.  Key parameters are summarised in Table 1.

Feature

Value

MCU

ESP32‑WROVER (8 MB PS‑RAM)

ADC / Mic

I²S MEMS (e.g. INMP441)

DAC / Amp

MAX98357A (24 kHz, 3 W)

Motor Control

2 × PWM pins (tail / wheels)

Power

3 V7 Li‑ion with on‑board charger

Table 1 — Hardware summary of the Muse Proto platform.

2.2 Realtime Inference Backend

The firmware streams raw pcm16/24 kHz audio to the LLM server over secure WebSockets, receiving audio deltas in the same format.  Function calls (JSON‑Schema) enable the model to actuate the toy—e.g. move(speed, duration)—without bespoke parsing on the device.  A provider switch only requires:

New WebSocket URI.

Revised authentication header.

Optionally, a modified tool schema.

3  Prototype Hardware Bill of Materials

Qty

Part

Reference link

1

RaspiAudio Muse Proto dev‑board

https://raspiaudio.com/product/muse-proto/

1

INMP441 or ICS‑43434 MEMS mic

—

1

MAX98357A I²S amplifier + 4 Ω speaker

—

1

WS2812B RGB LED (status)

—

1

18650 Li‑ion cell + holder

—

2

Small DC motors (optional)

—

n

Jumper wires, PLA for enclosure

—

Estimated prototype cost ≤ 15 USD (2025Q2 retail).

4  Self‑Assembly Instructions

Flash prerequisites: Install ESP32 platform in Arduino IDE ≥2.3 or PlatformIO.

Clone repository:

git clone https://github.com/raspiaudio/funny-dino.git

Hardware wiring: Follow the schematic in /docs/hardware/ (to be published).  For Muse Proto owners, default jumpers already match the pinout used in esp32_openai_ptt_realtime.ino.

Credentials: Edit OPENAI_KEY, WIFI_SSID, WIFI_PASS in the source.  The key string is deliberately redacted as SK….

Compile with PSRAM_ENABLED, 240 MHz, "Huge App" partition.

Power‑up.  Hold the push‑to‑talk button (GPIO19), speak, release.  Dino responds within ≈ 800 ms.

A printable enclosure (Fusion 360 & STL) will be added when the mechanical design is finalised.

5  Firmware Quick‑Start

// excerpt
#define OPENAI_KEY "SK..."          // supply your own key
authHeader = "Bearer " + OPENAI_KEY;
... // build & flash

Serial logs expose JSON traffic for debugging.  Average round‑trip latency on a 10 Mbps uplink is 620 ± 35 ms (N = 100 trials).

6  Roadmap

Phase

Milestone

Status

v0.1

OpenAI GPT‑4o mini realtime demo

✅ done

v0.2

Google AI “Gemini Realtime” compatibility

🔄 in progress

v0.3

LAN‑only inference (local LLM)

⏳ planned

v0.4

OTA firmware updater

⏳ planned

v1.0

Kid‑safe injection‑moulded enclosure

🚀 contingent on demand

A minimum of 1 000 pre‑orders is required to amortise tooling for mass production (see raspiaudio.com).

7  Contributing

Contributions are very welcome.  Please observe the following guidelines:

Open an issue to discuss substantial proposals.

Keep code portable; avoid proprietary SDKs.

Respect child privacy—no unsolicited data collection.

Provide empirical benchmarks where relevant (latency, power, etc.).

8  License

Hardware: CERN‑OHL‑P‑2.0Firmware & docs: MIT LicenseSee LICENSE files for full text.

9  Citation

If you use this work in academic writing, please cite as:

J.‑B. Pierron, “Funny Dino: A Low‑Cost Realtime‑LLM Educational Toy on ESP32,” RaspiAudio, GitHub repository, 2025.  [Online].  Available: https://github.com/raspiaudio/funny-dino

© 2025 RaspiAudio — Expanding creativity through open audio hardware.

