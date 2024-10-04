################################################################################
# MQTT component based on the Mosquitto implementation                         #
################################################################################

override INCLUDEPATHS += $(SDK_DIR)/app/bluetooth/common_host/mqtt

override C_SRC += $(SDK_DIR)/app/bluetooth/common_host/mqtt/mqtt.c

ifeq ($(OS),win)
# Make can't deal with spaces in paths, add mosquitto path separately.
override CFLAGS += -I"$(MOSQUITTO_DIR)/devel"
endif

ifeq ($(OS),posix)
override LDFLAGS += -lmosquitto -lm
else
override LDFLAGS += "${MOSQUITTO_DIR}/devel/mosquitto.lib"
endif
