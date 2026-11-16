@echo off
idf.py -B build_rx -D SDKCONFIG=sdkconfig.receiver -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.receiver.defaults" %*