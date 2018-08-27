ifndef TARGET
TARGET=expfr5969
endif

CONTIKI_PROJECT = button-test button-test.hex

all: $(CONTIKI_PROJECT)
CONTIKI = ../..
CONTIKI_WITH_RIME = 1

# Enable 20-bit instruction support
#ifeq ($(TARGET),expfr5969)
#  TARGET_MEMORY_MODEL = large
#endif

include $(CONTIKI)/Makefile.include
