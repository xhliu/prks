configuration CC2420ShareSpiC {
	provides {
		interface Resource;
		interface ChipSpiResource;
	}
}
implementation {
	components CC2420ShareSpiP;
	Resource = CC2420ShareSpiP;
	ChipSpiResource = CC2420ShareSpiP;
	
	components new CC2420SpiC() as Spi;
	CC2420ShareSpiP.SpiResource -> Spi;
	
	components UartLogC;
	CC2420ShareSpiP.UartLog -> UartLogC;
}
