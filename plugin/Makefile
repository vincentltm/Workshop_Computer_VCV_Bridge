RACK_DIR ?= dep/Rack-SDK

FLAGS  += -Isrc -Isrc/../../../card   # BridgeProtocol.hpp shared with card
CFLAGS +=

SOURCES += src/plugin.cpp
SOURCES += src/VCVBridge.cpp

DISTRIBUTABLES += res plugin.json

include $(RACK_DIR)/arch.mk

# C++17 for convenience
CXXFLAGS += -std=c++17

# Windows: link ws2_32 (not strictly needed for serial but avoids linker errors)
ifdef ARCH_WIN
    LDFLAGS += -lws2_32
endif

include $(RACK_DIR)/plugin.mk

# ── Dev install (macOS Arm64) ─────────────────────────────────────────────────
INSTALLED_PLUGIN_DIR = $(HOME)/Library/Application Support/Rack2/plugins-mac-arm64/MTMVCVBridge
install-dev: all
	mkdir -p "$(INSTALLED_PLUGIN_DIR)"
	cp plugin.dylib "$(INSTALLED_PLUGIN_DIR)/"
	cp plugin.json  "$(INSTALLED_PLUGIN_DIR)/"
	rm -rf "$(INSTALLED_PLUGIN_DIR)/res"
	cp -r res "$(INSTALLED_PLUGIN_DIR)/"
	@echo "Installed to $(INSTALLED_PLUGIN_DIR)"
