# Keyball39 Wireless ZMK Config

## What this is

ZMK firmware configuration for a Keyball39 split keyboard with a PMW3610 trackball sensor on the right half. Runs on nice!nano v2 boards (nRF52840) with SSD1306 OLED displays.

## Architecture

- **Firmware framework:** ZMK (v0.2) via Zephyr RTOS
- **Trackball driver:** `zmk-pmw3610-driver` from kumamuk-git (external West module)
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

## Build

Firmware builds automatically via GitHub Actions on push/PR. The workflow calls `zmkfirmware/zmk/.github/workflows/build-user-config.yml@v0.2`. Build artifacts (UF2 files) are downloadable from the Actions tab.

To flash: put the nice!nano into bootloader mode (double-tap reset), then copy the `.uf2` file to the mounted drive.

## Conventions

- Devicetree files use `.dtsi` (shared includes) and `.overlay` (per-half overrides)
- Config files use Kconfig syntax (`CONFIG_*=y/n/value`)
- Keymap uses ZMK's devicetree-based format with `#define` layer indices at the top
- The right half has ZMK Studio enabled (`CONFIG_ZMK_STUDIO=y`) via the `studio-rpc-usb-uart` snippet
