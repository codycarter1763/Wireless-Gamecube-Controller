# Unit tests for the pure logic in src/gc_logic.c (no radio, no Joybus).
#
#   west build -p -b native_sim/native/64 tests/unit -t run
#   (or: west twister -T tests/unit -p native_sim/native/64)
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gc_logic_tests)

set(APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_include_directories(app PRIVATE ${APP_SRC} src)
target_sources(app PRIVATE
	${APP_SRC}/gc_logic.c
	src/test_input.c
	src/test_rate.c
	src/test_link.c
	src/test_seq.c
	src/test_ota.c
)
