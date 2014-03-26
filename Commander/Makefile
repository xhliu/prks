COMPONENT=CommanderC

UTILDIR = $(HOME)/Dropbox/Programming/TinyOS

PFLAGS +=	-I$(UTILDIR)/Utils \
#			-I$(UTILDIR)/fix \

# CC2420X defaults
CFLAGS += -DCC2420X_DEF_RFPOWER=31        #neteye 3, motelab 23, Indriya 11
CFLAGS += -DCC2420X_DEF_CHANNEL=26

CFLAGS += -DTOSH_DATA_LENGTH=114

include $(MAKERULES)
