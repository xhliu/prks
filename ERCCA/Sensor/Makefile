COMPONENT=SensorC

UTILDIR = $(TOSROOT)/../Programming/TinyOS/Utils

CFLAGS += -I$(UTILDIR)# \
#		  -I../fastspi
#CFLAGS += -DFAST_SMCLK
CFLAGS += -DCC2420_DEF_RFPOWER=31        #neteye 3, motelab 23, Indriya 11
CFLAGS += -DTOSH_DATA_LENGTH=114
#CFLAGS += -DCC2420_DEF_CHANNEL=26
include $(MAKERULES)
