  async command uint8_t SpiByte.write( uint8_t tx ) {
    uint8_t byte;
    // we are in spi mode which is configured to have turned off interrupts
    //call Usart.disableRxIntr();
    call Usart.tx( tx );
    while( !call Usart.isRxIntrPending() );
    call Usart.clrRxIntr();
    byte = call Usart.rx();
    //call Usart.enableRxIntr();
    return byte;
  }

  async command void Usart.tx(uint8_t data) {
    U0TXBUF = data;
  }

  async command uint8_t Usart.rx() {
    return U0RXBUF;
  }

  async command bool Usart.isRxIntrPending(){
    if (IFG1 & URXIFG0){
      return TRUE;
    }
    return FALSE;
  }

  async command void Usart.clrRxIntr() {
    IFG1 &= ~URXIFG0;
  }
// if the TX buffer is empty and all of the bits have been shifted out
  async command bool Usart.isTxEmpty(){
    if (U0TCTL & TXEPT) {
      return TRUE;
    }
    return FALSE;
  }
  
inline int8_t readRssiFast() {
   int8_t rssi;
   P4OUT &= ~0x04;      // clear CSN, CS low
   // write address 0x53  (0x40 for register read, 0x13 for RSSI register address)
   // XL: msp430 tx buffer
   U0TXBUF = 0x53;
   // wait until data has moved from UxTXBUF to the TX shift register
   // and UxTXBUF is ready for new data. It does not indicate RX/TX completion.
   // XL: first byte sent
   while (!(IFG1 & UTXIFG0))
	 ;
   U0TXBUF = 0;
   // XL: second byte sent
   while (!(IFG1 & UTXIFG0))
	 ;
   U0TXBUF = 0;
   // XL: third byte sent
   while (!(IFG1 & UTXIFG0))
	 ;
   // XL: rx buffer ready
   while (!(U0TCTL & TXEPT))
	 ;
   // XL: msp430 rx buffer	 
   rssi = U0RXBUF;
   P4OUT |= 0x04;      // CS high
   return rssi;
}
