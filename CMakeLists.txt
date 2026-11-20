cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_gc_wireless)

target_sources(app PRIVATE src/main.c src/radio.c)
target_sources_ifdef(CONFIG_APP_ROLE_PTX app PRIVATE src/ptx.c src/joybus.c)
target_sources_ifdef(CONFIG_APP_ROLE_PRX app PRIVATE src/prx.c src/joybus_target.c)