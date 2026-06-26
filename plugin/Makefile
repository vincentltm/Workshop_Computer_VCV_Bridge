RACK_DIR ?= dep/Rack-SDK

# BridgeProtocol.hpp lives in ../card/ (one level up from plugin/)
FLAGS  += -Isrc -I../card

SOURCES += src/plugin.cpp
SOURCES += src/VCVBridge.cpp

DISTRIBUTABLES += res plugin.json

include $(RACK_DIR)/arch.mk

CXXFLAGS += -std=c++17

ifdef ARCH_WIN
    LDFLAGS += -lws2_32
endif

include $(RACK_DIR)/plugin.mk

# ── Dev install (macOS arm64) ──────────────────────────────────────────────────
INSTALLED_PLUGIN_DIR = $(HOME)/Library/Application Support/Rack2/plugins-mac-arm64/MTMVCVBridge
install-dev: all
	mkdir -p "$(INSTALLED_PLUGIN_DIR)"
	cp plugin.dylib "$(INSTALLED_PLUGIN_DIR)/"
	cp plugin.json  "$(INSTALLED_PLUGIN_DIR)/"
	rm -rf "$(INSTALLED_PLUGIN_DIR)/res"
	cp -r res "$(INSTALLED_PLUGIN_DIR)/"
	@echo "Installed to $(INSTALLED_PLUGIN_DIR)"
