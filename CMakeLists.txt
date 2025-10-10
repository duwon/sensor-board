cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sensor_board_fw)

target_sources(app PRIVATE
  src/main.c
  src/gpio_if.c
  src/dip_switch.c
  src/ltc3337.c
  src/sensors.c
  src/ble_adv.c
)
