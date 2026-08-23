# This repository doubles as a Zephyr module so the PMW3610 trackball driver
# can live alongside the keymap that uses it. ZMK's build-user-config workflow
# adds the repo via -DZMK_EXTRA_MODULES when zephyr/module.yml exists.

if(CONFIG_PMW3610)
  zephyr_library()
  zephyr_library_sources(drivers/input/pmw3610.c)
  zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()
