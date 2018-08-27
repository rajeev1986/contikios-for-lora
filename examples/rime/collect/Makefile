ifndef TARGET
TARGET=expfr5969
endif

CONTIKI_PROJECT = example-collect example-collect.hex

all: $(CONTIKI_PROJECT)
CONTIKI = ../../..
CONTIKI_WITH_RIME = 1

CFLAGS+= -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
