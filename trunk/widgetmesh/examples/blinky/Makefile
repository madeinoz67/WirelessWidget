CONTIKI_PROJECT = blinky
CONTIKI = ../../os
WIDGETMESH_PATH = ../../widgetmesh
ROOT_DIR = ../..


all: $(CONTIKI_PROJECT)

CONTIKIDIRS += .
SOURCEDIRS += .
PROJECT_OBJECTFILES += ${addprefix $(OBJECTDIR)/,${call oname, $(CONTIKI_SOURCEFILES)}}

include $(CONTIKI)/Makefile.include
