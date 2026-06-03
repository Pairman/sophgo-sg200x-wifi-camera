SHELL := /bin/bash

MIDDLEWARE_DIR ?= /build/middleware
KERNEL_DIR ?= /build/kernel/build/licheervnano-e
CHIP_ARCH ?= CV181X
CROSS_COMPILE ?= /host-tools/gcc/riscv64-linux-x86_64/bin/riscv64-unknown-linux-gnu-
SYSROOT ?= /host-tools/gcc/riscv64-linux-x86_64/sysroot
DESTDIR ?=
PREFIX ?= /usr

CC := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
STRIP ?= $(CROSS_COMPILE)strip
PKG_CONFIG ?= pkg-config

chip_lc := $(shell echo $(CHIP_ARCH) | tr A-Z a-z)
MW_INC := $(MIDDLEWARE_DIR)/include
MW_LIB := $(MIDDLEWARE_DIR)/lib
MW_3RD_LIB := $(MIDDLEWARE_DIR)/lib/3rd
KERNEL_INC ?= $(KERNEL_DIR)/riscv/usr/include
MEDIA_SERVER_DIR := $(MIDDLEWARE_DIR)/sample/test_mmf/media_server-1.0.x

COMMON_CFLAGS := \
	-Os -g -std=gnu11 -Wall -Wextra -ffunction-sections -fdata-sections \
	-mcpu=c906fdv -march=rv64imafdcv0p7xthead -mcmodel=medany -mabi=lp64d \
	-DARCH_$(CHIP_ARCH) -D__CV181X__ -D__SOC_MARS__ \
	-DSENSOR0_TYPE=SONY_IMX327_MIPI_2M_30FPS_12BIT \
	-DSENSOR1_TYPE=SONY_IMX327_MIPI_2M_30FPS_12BIT \
	-I$(MW_INC) \
	-I$(MIDDLEWARE_DIR)/modules/isp/include/$(chip_lc) \
	-I$(MW_INC)/isp/$(chip_lc) \
	-I$(MIDDLEWARE_DIR)/sample/common \
	-I$(KERNEL_INC) \
	-I$(MW_INC)/linux \
	-I$(MIDDLEWARE_DIR)/component/isp/common \
	-I$(MEDIA_SERVER_DIR)/media-server/libmov/include \
	-I$(MEDIA_SERVER_DIR)/media-server/libflv/include \
	-I$(MIDDLEWARE_DIR)/sample/audio/include \
	-I$(MIDDLEWARE_DIR)/sample/audio/aac_sample

DEPFLAGS := -MMD
CFLAGS += $(COMMON_CFLAGS)
LDFLAGS += -Wl,--gc-sections -Wl,--as-needed -L$(MW_LIB) -L$(MW_3RD_LIB) \
	-Wl,-rpath=/mnt/system/usr/lib -Wl,-rpath=/mnt/system/usr/lib/3rd

PKG_CONFIG_PATH := $(MIDDLEWARE_DIR)/pkgconfig
MW_LIBS = $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) $(PKG_CONFIG) --libs --define-variable=mw_dir=$(MIDDLEWARE_DIR) cvi_common cvi_sample)
MEDIA_LIBS = $(MEDIA_SERVER_DIR)/release.linux/libmov.a $(MEDIA_SERVER_DIR)/release.linux/libflv.a

COMMON_LIBS = $(MW_LIBS) -lvdec -lcvi_ispd2 -ltinyalsa -lsys -lpthread -lm -lini -ldl -lstdc++

TARGETS := camera_frame camera_stream camera_clip camera_server
OBJS := $(TARGETS:=.o)
DEPS := $(OBJS:.o=.d)

.PHONY: all clean install
all: $(TARGETS)

%.o: %.c
	$(CC) $(DEPFLAGS) $(CFLAGS) -o $@ -c $<

camera_frame: camera_frame.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(COMMON_LIBS)

camera_stream: camera_stream.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(MEDIA_LIBS) -laacenc2 -laaccomm2 -laacsbrenc2 $(COMMON_LIBS)

camera_clip: camera_clip.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(MEDIA_SERVER_DIR)/release.linux/libmov.a -lm -ldl -lpthread -lstdc++

camera_server: camera_server.o
	$(CXX) -o $@ $^ $(LDFLAGS) -lpthread -lm -ldl -lstdc++

install: all
	install -d $(DESTDIR)$(PREFIX)/bin
	install -d $(DESTDIR)$(PREFIX)/libexec/wifi-camera
	install -d $(DESTDIR)$(PREFIX)/lib/systemd/system
	install -d $(DESTDIR)/var/lib/wifi-camera/recordings
	install -d $(DESTDIR)/var/log/wifi-camera
	install -m 0755 camera_frame $(DESTDIR)$(PREFIX)/bin/camera-frame
	install -m 0755 camera_stream $(DESTDIR)$(PREFIX)/bin/camera-stream
	install -m 0755 camera_server $(DESTDIR)$(PREFIX)/bin/camera-server
	install -m 0755 camera_clip $(DESTDIR)$(PREFIX)/libexec/wifi-camera/camera_clip
	install -m 0644 camera-server.service $(DESTDIR)$(PREFIX)/lib/systemd/system/camera-server.service
	$(STRIP) --strip-unneeded \
		$(DESTDIR)$(PREFIX)/bin/camera-frame \
		$(DESTDIR)$(PREFIX)/bin/camera-stream \
		$(DESTDIR)$(PREFIX)/bin/camera-server \
		$(DESTDIR)$(PREFIX)/libexec/wifi-camera/camera_clip

clean:
	rm -f $(TARGETS) $(OBJS) $(DEPS)

-include $(DEPS)
