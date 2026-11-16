# The main component depends on all components by default, so no REQUIRES needed.
idf_component_register(SRCS "gc_link.c" "joybus.c" "main_controller.c" "main_receiver.c"
                       INCLUDE_DIRS ".")