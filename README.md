# CAN → MQTT Gateway for the DTU FF Boat

This repository contains the firmware for an **ESP32‑based gateway** that decodes CAN frames coming from the boat and republishes the decoded telemetry to an MQTT broker. It merges two original programs – a CAN decoder and a random‑data MQTT publisher – into a single, production‑ready sketch.

---

## 📂 Directory structure

| Path                                                                                                               | Purpose                                                                                                                                                             |
| ------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **src/main.cpp**                                                                                                   | Main application that **decodes CAN** frames using the ESP32 TWAI (CAN 2.0B) peripheral and **publishes** the parsed data to MQTT.                                  |
| **include/batteries\_gps\_motor\_2.h**                                                                             | *Auto‑generated header* produced from the projectʼs `.dbc` file. It declares the C structs representing every CAN message and exposes *pack/unpack/decode* helpers. |
| **src/batteries\_gps\_motor\_2.c**                                                                                 | It contains the logic for decoding raw CAN bytes into physical values. |
| **platformio.ini**                                                                                                 | PlatformIO build configuration for the ESP32 target (framework = Arduino).                                                                                          |

---

## 🛠️ How it works

1. **Startup**

   * Connects to Wi‑Fi using the credentials in `main.cpp`.
   * Connects/reconnects to the MQTT broker (`mqtt_server`, `mqttPort`).
   * Initialises the TWAI driver on **GPIO 5 (TX)** and **GPIO 4 (RX)** at **500 kbit/s**.
2. **Runtime loop**

   * Listens non‑stop for CAN frames.
   * For each recognised CAN ID, unpacks the payload with the generated `*_unpack` and `*_decode` helpers.
   * Publishes the resulting engineering units to MQTT with `publishFloat()` / `publishInt()`.
3. **MQTT topics**

   * All topics are rooted at **`boat/telemetry/`** (see full list below).
   * Messages are published with the *retain* flag so the broker keeps the last known value.

---

## 🌐 MQTT topic map

| Group               | Topic                                                                                                      | Payload                                        |
| ------------------- | ---------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| **Motor**           | `boat/telemetry/motor/speed`                                                                               | RPM (float, 2 dec.)                            |
|                     | `boat/telemetry/motor/temperature`                                                                         | °C (float, 2 dec.)                             |
|                     | `boat/telemetry/motor/voltage`                                                                             | V (float, 2 dec.)                              |
|                     | `boat/telemetry/motor/current`                                                                             | A (float, 2 dec.)                              |
| **GPS**             | `boat/telemetry/gps/latitude`                                                                              | ° (float, 6 dec.)                              |
|                     | `boat/telemetry/gps/longitude`                                                                             | ° (float, 6 dec.)                              |
|                     | `boat/telemetry/gps/altitude`                                                                              | m (float, 1 dec.)                              |
| **Battery n (1‑4)** | `boat/telemetry/battery/n/voltage`<br>`…/current`<br>`…/temperature`<br>`…/soc`<br>`…/status`<br>`…/alarm` | Various physical units according to the field. |

*Total published topics: **31***.

---

## 🔧 Configuration

| Parameter             | Location   | Default                               | Notes                                     |
| --------------------- | ---------- | ------------------------------------- | ----------------------------------------- |
| Wi‑Fi SSID/PWD        | `main.cpp` | `DTU_FF_Boat_Team` / `DTUBoatFF2425#` | Change to match your network.             |
| MQTT broker IP / port | `main.cpp` | `192.168.50.10` / `1883`              | Supports any broker that speaks MQTT 3.1. |
| CAN pins              | `main.cpp` | TX = GPIO 5, RX = GPIO 4              | Adapt for your transceiver wiring.        |

---

## License

Distributed under the MIT License – see `LICENSE` for details.
