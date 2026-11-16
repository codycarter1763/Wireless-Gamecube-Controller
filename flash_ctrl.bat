@echo off
idf.py -B build_ctrl -D SDKCONFIG=sdkconfig.controller -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.controller.defaults" %*