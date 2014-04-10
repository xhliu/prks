/* *
 * @ author: Xiaohui Liu (whulxh@gmail.com) 
 * @ updated: 7/24/2012 
 * @ description: LUR cache based on LruCtpMsgCacheC
 */

generic configuration AsyncCacheC(uint8_t CACHE_SIZE) {
    provides interface AsyncCache<message_t*> as Cache;
    
    uses {
	#if !defined(DEFAULT_MAC) && !defined(RTSCTS) && !defined(CMAC)
     	interface AsyncPacket as SubPacket;
    #else
    	interface Packet as SubPacket;
    #endif
    }
}
implementation {
    components MainC, new AsyncCacheP(CACHE_SIZE) as ImplP;
//    components CtpP;
//    Cache = CacheP;
//    CacheP.CtpPacket -> CtpP;
//    MainC.SoftwareInit -> CacheP;
	Cache = ImplP;
	MainC.SoftwareInit -> ImplP;
	
	//components IMACForwarderC;
//	components RouterP;
	// not -> 
//	ImplP.SubPacket -> IMACForwarderC;
	ImplP.SubPacket = SubPacket;
}
