COMPONENT=TestiMACC

MAINDIR = ../iMAC_TDMA
UTILDIR = $(HOME)/Dropbox/Programming/TinyOS

CFLAGS += -DTOSH_DATA_LENGTH=114

# -----------------------------------------------------------------------------
# basic CSMA
# -----------------------------------------------------------------------------
PFLAGS +=	-I$(UTILDIR)/Utils \
			-I$(MAINDIR) \
			-I$(MAINDIR)/util \
			-I$(MAINDIR)/async \
			-I$(MAINDIR)/../router \
			-I$(MAINDIR)/../router/async \
			-I$(TOSDIR)/lib/net

CFLAGS += -DCC2420_DEF_RFPOWER=3        #neteye 3, motelab 23, Indriya 11
CFLAGS += -DCC2420_DEF_CHANNEL=26

# -----------------------------------------------------------------------------
# RTS-CTS & C-MAC: based on CC2420
# -----------------------------------------------------------------------------
#PROTOCOLDIR = ../CMAC
##PROTOCOLDIR = ../RTSCTS
#PFLAGS +=	-I$(UTILDIR)/Utils \
#			-I$(PROTOCOLDIR) \
#			-I$(PROTOCOLDIR)/beaconing \
#			-I$(PROTOCOLDIR)/controller \
#			-I$(PROTOCOLDIR)/forwarder \
#			-I$(PROTOCOLDIR)/linkestimator \
#			-I$(PROTOCOLDIR)/signalmap \
#			-I$(PROTOCOLDIR)/cc2420 \
#			-I$(MAINDIR) \
#			-I$(MAINDIR)/util \
#			-I$(MAINDIR)/../router \
#			-I$(TOSDIR)/lib/net
#			
#CFLAGS += -DCC2420_DEF_RFPOWER=3        #neteye 3, motelab 23, Indriya 11
#CFLAGS += -DCC2420_DEF_CHANNEL=26
##enable snooping
#CFLAGS+=-DCC2420_NO_ADDRESS_RECOGNITION

# -----------------------------------------------------------------------------
# PRKS based on CC2420X
# -----------------------------------------------------------------------------
#PFLAGS +=	-I$(UTILDIR)/Utils \
#			-I$(MAINDIR) \
#			-I$(MAINDIR)/util \
#			-I$(MAINDIR)/async \
#			-I$(MAINDIR)/cc2420x \
#			-I$(MAINDIR)/ftsp \
#			-I$(MAINDIR)/cc2420x/ftsp \
#			-I$(MAINDIR)/signalmap \
#			-I$(MAINDIR)/forwarder \
#			-I$(MAINDIR)/controller \
#			-I$(MAINDIR)/linkestimator \
#			-I$(MAINDIR)/../router \
#			-I$(MAINDIR)/../router/async \
#			-I$(TOSDIR)/lib/net

##CC2420X defaults
#CFLAGS += -DCC2420X_DEF_RFPOWER=3        #neteye 3, motelab 23, Indriya 11
#CFLAGS += -DCC2420X_DEF_CHANNEL=26
#CFLAGS += -DSOFTWAREACK_TIMEOUT=8000

# -----------------------------------------------------------------------------
# extra
# -----------------------------------------------------------------------------
# enable h/w ack
#CFLAGS += -DCC2420_HW_ACKNOWLEDGEMENTS

# ftsp beacon period not controlled here, but in forwarder
#CFLAGS += -DTIMESYNC_RATE=2	#10

# mig
#BUILD_EXTRA_DEPS   += IndriyaSerial.class
#CLEAN_EXTRA            = *.class .java

#IndriyaSerial.class: $(wildcard *.java) IndriyaSerialMsg.java
#	javac -source 1.4 -target 1.4 *.java

#IndriyaSerialMsg.java: TestiMAC.h
#	mig java -target=$(PLATFORM) $(CFLAGS) -java-classname=IndriyaSerialMsg TestiMAC.h imac_indriya_serial -o $@

include $(MAKERULES)
