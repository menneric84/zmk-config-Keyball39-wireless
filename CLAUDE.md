# Keyball39 Wireless ZMK Config

## What this is

ZMK firmware configuration for a Keyball39 split keyboard with a PMW3610 trackball sensor on the right half. Runs on nice!nano v2 boards (nRF52840) with SSD1306 OLED displays.

## Architecture

- **Firmware framework:** ZMK (v0.2) via Zephyr RTOS
- **Trackball driver:** in-repo, at `drivers/input/pmw3610.c`. This repo doubles as a Zephyr module (`zephyr/module.yml` at the root), which ZMK's build workflow picks up automatically via `-DZMK_EXTRA_MODULES`. There is no external driver dependency.
- **Board:** nice_nano_v2 (both halves)
- **Shield definition:** `config/boards/shields/keyball_nano/` — custom shield with physical layout, matrix transform, GPIO pin mappings, I2C/OLED config, and SPI trackball wiring
- **Build system:** West (Zephyr's meta-tool); manifest in `config/west.yml`

## Key files

- `config/keyball39.keymap` — keymap with 7 layers: DEFAULT (QWERTY), NUM, SYM (actually BT/nav), FUN, MOUSE, SCROLL, SNIPE
- `config/keyball39.conf` — shared config (BT params, display, power)
- `config/boards/shields/keyball_nano/keyball39_right.conf` — right-half config (PMW3610 sensor settings: CPI, orientation, scroll, automouse)
- `config/boards/shields/keyball_nano/keyball39_right.overlay` — right-half devicetree overlay (SPI trackball wiring)
- `config/boards/shields/keyball_nano/keyball39_left.overlay` — left-half devicetree overlay (column GPIOs)
- `config/boards/shields/keyball_nano/keyball39.dtsi` — shared devicetree (matrix, physical layout, OLED, I2C)
- `build.yaml` — GitHub Actions build matrix (left, right, settings_reset)
- `drivers/input/pmw3610.c` — trackball driver
- `Kconfig`, `CMakeLists.txt`, `zephyr/module.yml`, `dts/bindings/input/` — module plumbing that makes the repo itself a Zephyr module

## Trackball driver notes

The sensor shares a power rail with the BLE radio, so SPI transactions get
corrupted by radio bursts as a matter of course. The driver treats that as
normal and recovers rather than failing:

- Chip-select is released on every exit path. Leaving CS asserted after a
  failed transfer is what strands the sensor mid-transaction, and that state
  survives a button reset -- only power loss clears it.
- Init retries indefinitely with backoff instead of failing permanently.
- A motion delta larger than `CONFIG_PMW3610_MAX_DELTA` is discarded. A
  corrupted burst decodes as a valid 12-bit delta, so the only thing marking
  it as garbage is its size; unfiltered, it shows up as the cursor jumping to
  a screen corner.
- A desync is repaired at runtime, triggered either by consecutive transfer
  failures or by a background health poll. Recovery holds CS high to
  resynchronize the sensor bus before re-running init.

Speed is tuned via `CONFIG_PMW3610_CPI` in hardware steps of 200. There is
deliberately no software divisor: dividing counts per report discards fine
movement with no remainder carried over.

## Build

Firmware builds automatically via GitHub Actions on push/PR. The workflow calls `zmkfirmware/zmk/.github/workflows/build-user-config.yml@v0.2`. Build artifacts (UF2 files) are downloadable from the Actions tab.

To flash: put the nice!nano into bootloader mode (double-tap reset), then copy the `.uf2` file to the mounted drive.

## Conventions

- Devicetree files use `.dtsi` (shared includes) and `.overlay` (per-half overrides)
- Config files use Kconfig syntax (`CONFIG_*=y/n/value`)
- Keymap uses ZMK's devicetree-based format with `#define` layer indices at the top
- The right half has ZMK Studio enabled (`CONFIG_ZMK_STUDIO=y`) via the `studio-rpc-usb-uart` snippet
