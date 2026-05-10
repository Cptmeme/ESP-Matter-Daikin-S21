# Daikin S21 — Matter over Thread

[![License: GPL v2](https://img.shields.io/badge/License-GPL_v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)
[![Release](https://img.shields.io/github/v/release/cptmeme/ESP-Matter-Daikin-S21)](../../releases/latest)
[![Downloads](https://img.shields.io/github/downloads/cptmeme/ESP-Matter-Daikin-S21/total)](../../releases)
[![Stars](https://img.shields.io/github/stars/cptmeme/ESP-Matter-Daikin-S21?style=social)](../../stargazers)

> **⚠️ Disclaimer.** Opening your air conditioner exposes you to mains voltage and can damage the unit. Always disconnect mains power at the breaker before opening the indoor unit. Incorrect wiring on the S21 bus can damage the AC's controller board or the ESP32. You assume all responsibility for any damage, data loss, or device failure. This project is not affiliated with Daikin Industries, the Connectivity Standards Alliance, Espressif Systems, or RevK / the Faikin/Faikout project.

Open source Matter firmware for Daikin split air conditioners with an S21 port. Adds your AC to Apple Home, Google Home, Alexa, and Home Assistant — no Daikin app, no cloud, no subscription, no BRP module. **Most modern Daikin split units have an S21 connector inside the indoor unit waiting to be used.**

> This release is the **Room Air Conditioner variant** — Matter device type `0x0072`, with **OnOff, Thermostat, Fan Control, and Temperature Measurement** clusters bundled into a single tile. Power, mode (Heat / Cool / Auto / Off), setpoint, current temperature, and fan speed all surface as native Matter attributes.
>
> **Powerful mode** is exposed on a second endpoint as an OnOff plug-in unit. In Apple Home and Google Home it appears as an "Outlet" tile next to the AC tile — rename it to "Powerful" in your home app.
>
> **Quiet mode is intentionally not exposed.** Matter has no native concept that maps to Daikin's Quiet (which limits both compressor and fan), so adding it as a separate endpoint would be misleading. Use the AC's remote for Quiet, or contribute a PR if you want to plumb it.

> ⚠️ Uses ESP-Matter SDK test credentials (vendor `0xFFF1`, not VID/PID-certified). Functional for personal use; not suitable for resale as a certified Matter product.

---

## Contents

- [Quick Start](#quick-start)
- [Features](#features)
- [Compatible Hubs](#compatible-hubs)
- [Hardware](#hardware)
- [Flashing](#flashing)
- [Commissioning](#commissioning)
- [Factory Reset](#factory-reset)
- [Known Issues / Roadmap](#known-issues--roadmap)
- [Building from Source](#building-from-source)
- [License](#license)
- [Matter and Thread Certification](#matter-and-thread-certification)
- [Contributing](#contributing)
- [Why?](#why)

---

## Quick Start

**Have a [Thread Border Router](#compatible-hubs), an ESP32-C6, an S21 interface board (the included PCB or a Faikout), and a Daikin unit with an S21 port? Flash and go.**

1. [Download the latest release](../../releases/latest) — grab the `daikin-s21-matter-vX.Y.Z.bin` file from the assets.
2. Connect the interface board to your Daikin's S21 port — see [Hardware](#hardware).
3. Flash with `esptool` or ESPConnect — see [Flashing](#flashing).
4. [Commission](#commissioning) with your smart home app.

---

## Features

- ✅ Matter over Thread — uses the ESP32-C6's 802.15.4 radio, no WiFi required
- ✅ Compatible with Apple Home, Google Home, Alexa, Home Assistant
- ✅ Multi-fabric — commission to multiple hubs simultaneously
- ✅ Native Room Air Conditioner tile (single accessory in your home app)
- ✅ Power, Mode (Heat/Cool/Auto/Off), Setpoint, Current Temperature
- ✅ Fan speed (Low/Med/High/Auto + percent slider)
- ✅ Powerful mode as a separate switchable endpoint
- ✅ Two-way state sync — adjusting on the Daikin remote updates your home app within ~2 seconds
- ✅ Factory reset via long press on the onboard button

> **Fan speed visibility note:** Apple Home and Google Home currently *do not render* the fan slider on their AC tile, even when the firmware correctly advertises it. Home Assistant shows it. This is a controller-side UI limitation, not a firmware bug — the FanControl cluster is correctly registered and writable from any Matter controller that chooses to surface it (e.g. via Apple Shortcuts).

---

## Compatible Hubs

> **Requires a Thread Border Router.** A Thread Border Router is required for any Matter-over-Thread device. See the table below for tested and supported options.

| Thread Border Router | Ecosystem | Tested |
|----------------------|-----------|--------|
| Apple HomePod mini / HomePod (2nd gen) | Apple Home | Untested |
| Apple TV 4K (3rd gen) | Apple Home | Untested |
| Google Nest Hub (2nd gen), Nest Wifi Pro | Google Home | Untested |
| Amazon Echo (4th gen) / Echo Hub | Alexa | Untested |
| Home Assistant SkyConnect / Yellow / Connect ZBT-1 | Home Assistant | Untested |
| Aeotec / SmartThings Station / Hub v3 | SmartThings | Untested |

> Apple iPhones (15 Pro and later) include a Thread radio but, as far as I'm aware, Apple has not enabled them as Thread Border Routers — they act as commissioners only. You still need a HomePod or Apple TV in your network to actually run the Thread mesh.

> **SmartThings note.** SmartThings recognizes Matter Room Air Conditioner devices and exposes power, mode, setpoint, and current temp. It does **not** surface the secondary "Powerful" endpoint (a known SmartThings limitation: it only renders endpoint compositions matching its predefined device profiles). Use Apple Home, Google Home, or Home Assistant if you need Powerful.

---

## Hardware

### Recommended path: use a known-good interface board

You need *something* between the ESP32-C6 and the Daikin's S21 port — the S21 bus is 5V logic, the ESP32 is 3.3V, and the timing/biasing is fussy enough that throwing a random level shifter at it does not reliably work (I learned this the hard way). Two options that are known to work:

1. **The reference PCB included in this repo** — see the [`Custom PCB esp32c6/`](../Custom%20PCB%20esp32c6/) folder. KiCad project + BOM. Built around an ESP32-C6-MINI-1, a BSS138 for level shifting, USB-C for power/flashing, and a JST EH 5-way for the S21 connection.
2. **A RevK [ESP32-Faikout](https://codeberg.org/RevK/ESP32-Faikout) board.** This firmware's S21 implementation is derived from RevK's [Faikin](https://codeberg.org/RevK/Faikin) project and the Faikout hardware will work with it (you'll need to reflash with this firmware instead of Faikin's). Faikout boards are sold by RevK and on Amazon UK — check the Faikout repo for current availability.

If you want to roll your own interface, use the included PCB as a reference rather than starting from scratch.

### S21 connector pinout

Pinout, colors, and connector types are documented authoritatively in [Faikin's wiring wiki](https://codeberg.org/RevK/ESP32-Faikout/wiki/Wiring). Reproduced for convenience:

| Pin | Daikin color | Generic color | Function |
|-----|--------------|---------------|----------|
| 1 | Brown | Orange | 5V (not always connected; insufficient current for the board) |
| 2 | Red | Blue | TX from aircon (5V logic) |
| 3 | Orange | Yellow | RX to aircon (5V logic; most models accept 3.3V) |
| 4 | Yellow | Red | 12-14V (the actual power source — both Faikout and the included PCB run from this pin) |
| 5 | Blue | Black | GND |

> **Pin direction is from the aircon's perspective.** Pin 2 is the aircon's TX (data flowing *out* of the AC, *into* the ESP32). Pin 3 is the aircon's RX (data flowing *into* the AC, *from* the ESP32). The included PCB / Faikout handles the crossover correctly.

> **Power comes from pin 4, not pin 1.** Pin 1's 5V rail does not supply enough current to run the ESP32-C6 + interface logic. The included PCB uses a [TI TPS562246](https://www.ti.com/product/TPS562246) synchronous step-down (buck) converter to take the 12-14V from pin 4 down to 3.3V for the ESP32. Faikout uses a similar wide-input regulator and accepts anywhere from 4V to 36V on pin 4. **Do not try to power the board from pin 1.**

**Two physical connector variants exist on the Daikin side:**
- **Type A** — JST EH (2.5 mm pitch), older models
- **Type B** — JST PAP (2.0 mm pitch), newer models

The boards above use JST EH male (5-way). For Type B units you need either a Type-A-to-Type-B cable or a different connector on the board. Faikin's wiring wiki links to per-type cable drawings.

### Where to find the S21 port on your AC

- [Faikin's confirmed-working models list](https://codeberg.org/RevK/ESP32-Faikout/wiki/List-of-confirmed-working-air-con-units) — start here. If your model is on this list it has S21.
- [Faikin's FTXM wiring teardown](https://codeberg.org/RevK/ESP32-Faikout/wiki/FTXMxxW2VMA-Wiring) — step-by-step photos for the FTXM series, the most common Daikin split.
- For other models the S21 port is generally on the indoor unit's main controller PCB, accessible after removing the front cover and electrical-bay cover.

### ESP32-C6 GPIO assignments (firmware side)

| ESP32-C6 GPIO | Function |
|---------------|----------|
| GPIO 20 | S21 RX (input) — receives data from the AC |
| GPIO 21 | S21 TX (output) — transmits to the AC |
| GPIO 23 | Factory-reset / commissioning button → GND (active low, internal pull-up) |

> **GPIO TX/RX naming is from the ESP32's perspective.** GPIO21 is an output, GPIO20 is an input. This is the opposite of the S21 connector's pin labels (which are from the AC's perspective). The included PCB and Faikout both wire the crossover correctly — you don't need to think about it if you're using either board.

### Factory-reset button

A long press on the GPIO23 button triggers a Matter factory reset (clears all fabrics and re-enters BLE commissioning mode). A short press opens a new commissioning window if no fabric is paired.

---

## Flashing

> ⚠️ **Disconnect mains power at the breaker before opening the indoor unit.** The S21 port itself is logic-level and isolated from mains, but everything around it inside the cabinet is not. If you're not comfortable working inside an AC indoor unit, have a qualified HVAC technician install the interface board for you and just plug into the S21 connector once it's accessible.

### Flash with esptool

Requires [esptool](https://github.com/espressif/esptool) installed. On macOS/Linux: `pip install esptool`. On Windows: install via pip, or download the standalone binary from the releases page.

Throughout these commands, replace:
- `<PORT>` with your serial port (`/dev/cu.usbserial-XXXX` on macOS, `/dev/ttyUSB0` on Linux, `COM3` on Windows)
- `<VERSION>` with the firmware release version (e.g. `v1.0.0`)

**Identify the chip and read the MAC address:**

```bash
esptool.py --chip esp32c6 --port <PORT> --baud 460800 chip_id
```

**Flash the firmware:**

```bash
esptool.py --chip esp32c6 --port <PORT> --baud 460800 \
  write_flash 0x0 daikin-s21-matter-<VERSION>.bin
```

**Erase flash (if you want to fully reset before re-flashing):**

```bash
esptool.py --chip esp32c6 --port <PORT> --baud 460800 erase_flash
```

After flashing, power-cycle the ESP32. The device boots into BLE commissioning mode and is ready to be added to your smart home ecosystem.

### Flash with ESPConnect

ESPConnect is a browser-based ESP32 flashing tool built on Web Serial. No installation required — works in Chrome, Edge, or any Chromium-based browser on macOS, Windows, and Linux.

1. Connect the ESP32-C6 to your computer via USB.
2. Open [ESPConnect](https://thelastoutpostworkshop.github.io/microcontroller_devkit/espconnect/) in your browser.
3. Click **Connect** and select your USB serial port.
4. Click **Flash Tools** → **Flash Firmware**.
5. Select the latest release `.bin` from your downloads.
6. Set flash offset to `0x0`, check **Erase entire flash before writing**, click **Flash**.
7. Power-cycle the ESP32 to boot the new firmware.

### Verify the firmware is running

After flashing, open a serial monitor at 115200 baud:

- **macOS / Linux CLI:** `screen /dev/cu.usbserial-XXXX 115200` or `idf.py -p /dev/cu.usbserial-XXXX monitor` if you have ESP-IDF installed
- **Windows:** [PuTTY](https://putty.org/) configured for Serial at 115200 baud
- **VS Code:** the Serial Monitor extension

You should see boot logs followed by:

```
I (xxxx) app_main: Room Air Conditioner created with endpoint_id 1
I (xxxx) app_main: Powerful plug-in unit created with endpoint_id 2
I (xxxx) app_main: Commissioning window opened
```

This confirms Matter is initialized and the device is advertising for commissioning.

---

## Commissioning

- Commission devices ONE AT A TIME.
- Power off or move away other un-commissioned devices while setting up each one.
- All devices share the same pairing code, so the controller may get confused if multiple are advertising simultaneously.
- Once commissioned, each device gets a unique identity on your network.

### Apple Home

Scan this QR code with the Home app, or enter the setup code manually:

![Matter Setup QR Code](docs/images/matter_qrcode_20202021_3840.png)

1. Open the **Home** app.
2. Tap **+** → **Add Accessory**.
3. Scan the QR code above, or tap **More Options** to enter the setup code manually.
4. Press **Add Anyway** when prompted (this is expected — see [Matter and Thread Certification](#matter-and-thread-certification)).
5. Name the device and place it in a room. You'll see two tiles: the AC and the "Outlet" (Powerful). Rename the Outlet to "Powerful" or similar.

```
Setup code: 3497-011-2332
```

> ⚠️ This is the ESP-Matter SDK test setup code, used by all devices running this firmware. Once a device is commissioned to your Matter fabric, the setup code is no longer used for authentication — your Matter ecosystem manages credentials going forward. Multiple uncommissioned devices broadcasting the same setup code is why the commissioning instructions specify one device at a time.

> **Apple Home creates two fabrics during pairing.** This is normal. Apple Inc. (vendor `0x1349`) is the local Home Hub admin; Apple Keychain (vendor `0x1384`) is Apple's iCloud-based admin that lets all your Apple devices control the unit. Both fabrics are Apple's — you have not been auto-shared with a third party.

<details>
<summary>Google Home / Alexa / Home Assistant commissioning</summary>

Google Home, Alexa, and Home Assistant all support Matter commissioning via their respective apps and integrations. Use setup code `3497-011-2332` or the QR code above when prompted.

**Home Assistant specifically renders the fan slider** for this device, which Apple Home and Google Home currently omit on their AC tiles.

</details>

---

## Factory Reset

Hold the onboard reset button (GPIO23 → GND) for **several seconds**. The device will clear all fabrics and re-enter BLE commissioning mode. Make sure to also remove the device from whatever ecosystems you've added it to — controllers cache credentials and will refuse re-pairing otherwise.

---

## Known Issues / Roadmap

- [ ] **Quiet mode** — not currently exposed to Matter. Could be added as a third endpoint (similar to Powerful), but Matter has no native concept that cleanly represents it. PRs welcome.
- [ ] **Fan slider in Apple Home / Google Home** — both controllers' AC tiles omit the fan slider. Workaround would be exposing fan as a separate `Fan` device type (`0x002B`) endpoint, at the cost of a third tile in those apps. Not implemented yet.
- [ ] **Outside temperature** — the S21 protocol exposes outside-coil temperature; not currently mirrored to a Matter cluster. Could be added as a second TemperatureMeasurement endpoint.
- [ ] **Swing / louver position** — the S21 protocol supports it; no Matter wiring yet.
- [ ] **Energy reporting** — Matter 1.3 added Power and Energy clusters, supported in some controllers (HA). The Daikin S21 protocol does not expose energy data, so this would require external metering.
- [ ] **Configurable fan-speed mapping** — currently Matter Low/Med/High maps to S21 speeds 1/3/5. A Kconfig option for finer granularity (e.g. Low=2, High=4) could be useful for some users.

---

## Building from Source

<details>
<summary>Build instructions</summary>

Throughout these commands, replace:
- `<VERSION>` with the firmware release version (e.g. `v1.0.0`)

**Requirements:**
- ESP-IDF v5.3.2 or newer
- ESP-Matter (latest)
- macOS or Linux

```bash
git clone https://github.com/cptmeme/ESP-Matter-Daikin-S21.git
cd ESP-Matter-Daikin-S21/Thermostat_Daikin

# Source from wherever you installed esp-idf and esp-matter.
source ~/esp/esp-idf/export.sh
source ~/esp/esp-matter/export.sh

idf.py set-target esp32c6
idf.py build
```

Merge bin files for distribution:

```bash
esptool.py --chip esp32c6 merge_bin \
  -o daikin-s21-matter-<VERSION>.bin \
  --flash_mode dio \
  --flash_size 4MB \
  0x0     build/bootloader/bootloader.bin \
  0x10000 build/partition_table/partition-table.bin \
  0x17000 build/ota_data_initial.bin \
  0x20000 build/thermostat.bin
```

To customize the device name shown in your home app, edit [`main/CHIPProjectConfig.h`](main/CHIPProjectConfig.h) before building:

```c
#define CHIP_DEVICE_CONFIG_DEVICE_VENDOR_NAME   "Daikin"
#define CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_NAME  "Daikin A/C"
#define CHIP_DEVICE_CONFIG_DEFAULT_NODE_LABEL   "Daikin Thermostat"
```

</details>

---

## License

GPL v2 — see [LICENSE](../LICENSE)

The S21 protocol implementation is derived from the [Faikin](https://github.com/revk/ESP32-Faikin) project by Adrian Kennard, which is licensed under GPL v2 — that's why this firmware is also GPL v2. Combined with portions of the [ESP-Matter](https://github.com/espressif/esp-matter) thermostat example by Espressif Systems (Apache 2.0, which is GPL-compatible in this direction).

---

## Matter and Thread Certification

This firmware is **not** Matter Certified or Thread Certified. It implements the Matter protocol via Espressif's [esp-matter](https://github.com/espressif/esp-matter) SDK and Thread via OpenThread, but has not been submitted to the Connectivity Standards Alliance (CSA) or Thread Group for certification. Certification requires CSA membership ($7,000+/year) and Authorized Test Lab evaluation, which are out of scope for an open-source community project.

Devices flashed with this firmware will appear in Matter ecosystems as **uncertified Matter devices**. Most ecosystems show a one-time warning during commissioning ("Add Anyway" in Apple Home); this is normal and expected.

This firmware uses the publicly available ESP-Matter test VID/PID and setup code, which is acceptable for personal use and development but cannot be used in commercial certified products.

"Matter" is a trademark of the Connectivity Standards Alliance. "Thread" is a trademark of the Thread Group. "Daikin" is a trademark of Daikin Industries, Ltd. This project is not affiliated with, endorsed by, or sponsored by any of these organizations.

---

## Contributing

Contributions are welcome. This is a solo-maintained project, so response times will vary, but every issue and PR gets read.

### Reporting bugs

Open a [GitHub issue](../../issues/new) with:

- **Hardware:** ESP32-C6 board variant, S21 interface design (optocoupler / FET / which PCB), and Daikin model number
- **Ecosystem:** Apple Home, Google Home, Alexa, Home Assistant, etc., and the Thread Border Router(s) on your network
- **What happened:** What you expected, what actually occurred, and steps to reproduce
- **Logs:** Serial monitor output at 115200 baud, especially anything around the failure point

If your device commissions but the AC never updates state, the most likely cause is the S21 interface — wrong wiring, wrong connector type (Type A vs B), or a custom level-shifter circuit that doesn't bias the line correctly. Verify by trying with the [included PCB](../Custom%20PCB%20esp32c6/) or a [Faikout](https://codeberg.org/RevK/ESP32-Faikout) board, both of which are known-good.

Firmware filename convention: `daikin-s21-matter-{version}.bin`

---

## Why?

Daikin makes excellent air conditioners and a famously poor cloud experience. The official path to remote control is the BRP-series WiFi adapter and the Daikin app, which:

- Costs €100+ for a module that's a glorified ESP32 + their cloud
- Routes every command through Daikin's servers
- Has no native Matter, HomeKit, or local API
- Stops working the moment Daikin retires the cloud product (and they have, repeatedly)
- Doesn't extend to multiple ecosystems — it's the Daikin app or nothing

Meanwhile, most modern Daikin split units ship with an **S21 service port** sitting unused inside the indoor unit, exposing the full feature set of the AC over a documented(-ish) serial protocol. RevK's [Faikin](https://codeberg.org/RevK/Faikin) project did the hard work of reverse-engineering the protocol and shipping the [Faikout](https://codeberg.org/RevK/ESP32-Faikout) hardware. This project bolts a Matter front-end onto that work — turning the same hardware into a fully local, cloud-free, multi-ecosystem Matter accessory that works directly with Apple Home, Google Home, Alexa, and Home Assistant. No app to install, no servers to depend on, no recurring fee.

### Why Matter over Thread?

Matter over Thread offers lower power consumption, lower latency, and a mesh that doesn't depend on your WiFi. Thread devices use any compatible Thread Border Router (HomePod mini, Apple TV 4K, iPhone 15 Pro+, Nest Hub 2nd gen, HA SkyConnect/Yellow) and continue to function even if your WiFi is down or congested. The ESP32-C6 has both a WiFi 6 radio and an 802.15.4 Thread radio — this firmware uses the Thread radio so the WiFi stack can stay quiet.

### Why Matter, and not Faikin / ESPHome / Tasmota?

Faikin is excellent and deserves credit — it's the source of this project's S21 implementation and the entire reason the protocol is documented at all. But Faikin (and ESPHome, Tasmota) all assume a central controller: Home Assistant, an MQTT broker, or a custom dashboard. They're great if that's your setup. They're a problem if it isn't.

This firmware speaks Matter directly. There's no middleware:

- **Apple Home user without Home Assistant?** It just works. Commission directly from the Home app.
- **Google Home or Alexa user?** Same — direct Matter commissioning, no third-party hub.
- **Home Assistant user?** Also works — commission via HA's Matter integration. Bonus: HA renders the fan slider that Apple/Google omit.
- **Multi-ecosystem household?** Multi-fabric commissioning lets the same device live in Apple Home, HA, and Google Home simultaneously.

Matter is an industry-standard protocol — your AC works with the ecosystems you have today and remains compatible with new Matter ecosystems as they appear.

---

## About

This firmware exists because I have a Daikin AC and refused to install yet another vendor app on my phone. RevK's Faikin/Faikout project had already done the hard part — reverse-engineering S21 and shipping working hardware — but only spoke MQTT and HTTP. With ESP-IDF and esp-matter, it wasn't a huge leap to put a native Matter front-end on the same hardware so the unit shows up directly in Apple Home alongside everything else.

Along the way I learned more than I wanted to about: which level shifters do and don't work on the S21 bus, how Apple Home's Room AC tile chooses to omit the fan slider entirely, what `data_model: Cannot set bounds because of val type mismatch: expected: 7, min: 9, max: 9` means deep inside esp-matter, and why two fabrics get created during a single Apple Home pairing (spoiler: vendor `0x1384` is "Apple Keychain", not Samsung).

If this firmware saved you the same headaches — or at least a €100 BRP module — consider leaving a ⭐ on the repo. It helps other Daikin owners find this project and signals there's a working alternative to the official ecosystem.
