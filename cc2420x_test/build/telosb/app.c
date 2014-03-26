#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;
#line 310
static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 340
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 372
static __inline int32_t __nesc_hton_int32(void * target, int32_t value)  ;
#line 431
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 42 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memcpy(void *arg_0x40306e68, const void *arg_0x40304030, size_t arg_0x403041c8);

extern void *memset(void *arg_0x40304e90, int arg_0x40309010, size_t arg_0x403091a8);
#line 65
extern void *memset(void *arg_0x40315d30, int arg_0x40315e88, size_t arg_0x40314030);
# 62 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x403353b8);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x40338a70);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 164 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned char ME1 __asm ("__""ME1");
#line 183
extern volatile unsigned char ME2 __asm ("__""ME2");
#line 195
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 267
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");

extern volatile unsigned char P1IFG __asm ("__""P1IFG");



extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");

extern volatile unsigned char P2IFG __asm ("__""P2IFG");



extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");










extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");




extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");










extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");




extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 382
extern volatile unsigned char U0CTL __asm ("__""U0CTL");

extern volatile unsigned char U0TCTL __asm ("__""U0TCTL");



extern volatile unsigned char U0MCTL __asm ("__""U0MCTL");

extern volatile unsigned char U0BR0 __asm ("__""U0BR0");

extern volatile unsigned char U0BR1 __asm ("__""U0BR1");

extern const volatile unsigned char U0RXBUF __asm ("__""U0RXBUF");
#line 439
extern volatile unsigned char U1CTL __asm ("__""U1CTL");

extern volatile unsigned char U1TCTL __asm ("__""U1TCTL");



extern volatile unsigned char U1MCTL __asm ("__""U1MCTL");

extern volatile unsigned char U1BR0 __asm ("__""U1BR0");

extern volatile unsigned char U1BR1 __asm ("__""U1BR1");

extern const volatile unsigned char U1RXBUF __asm ("__""U1RXBUF");
#line 595
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");





extern volatile unsigned int TAR __asm ("__""TAR");

extern volatile unsigned int TACCR0 __asm ("__""TACCR0");
#line 718
extern volatile unsigned int TBCTL __asm ("__""TBCTL");

extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");

extern volatile unsigned int TBCCTL1 __asm ("__""TBCCTL1");

extern volatile unsigned int TBCCTL2 __asm ("__""TBCCTL2");









extern volatile unsigned int TBR __asm ("__""TBR");
#line 849
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 1021
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
# 343 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 378
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 8 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4251 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4252 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 47 "RadioCountToLeds.h"
#line 44
typedef nx_struct radio_count_msg {
  nx_uint16_t counter;
  nx_uint8_t payload[100];
} __attribute__((packed)) radio_count_msg_t;

enum __nesc_unnamed4253 {
  AM_RADIO_COUNT_MSG = 6, 
  AM_IMAC_LE = AM_RADIO_COUNT_MSG + 1
};
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4254 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4255 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4256 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4257 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4258 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4259 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4260 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4261 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/Leds.h"
enum __nesc_unnamed4262 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4263 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 114 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4264 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4265 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4266 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4267 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4268 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4269 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/platform_message.h"
#line 56
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 61
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 65
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[114];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 32 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLog.h"
#line 7
typedef nx_struct __nesc_unnamed4270 {
  nx_uint8_t type;

  nx_uint8_t nodeId;

  nx_uint8_t sourceId;


  nx_uint16_t seq;



  nx_uint8_t last_hop;


  nx_uint16_t last_hop_ntw_seq;

  nx_uint16_t last_hop_seq;

  nx_uint16_t local_ntw_seq;

  nx_uint16_t local_seq;

  nx_uint32_t timestamp;
  nx_uint32_t seqno;
} __attribute__((packed)) logMsg;






#line 35
typedef struct __nesc_unnamed4271 {
  message_t msg;

  uint8_t aggr_counts;
} queue_entry_t;

enum __nesc_unnamed4272 {



  AM_SERIAL_MSG = 100, 





  UART_QUEUE_LEN = 8, 


  AGGR_COUNTS = 114 / sizeof(logMsg )
};
# 31 "../iMAC_TDMA/cc2420x/CC2420XDriverLayer.h"
#line 28
typedef nx_struct cc2420x_header_t {

  nxle_uint8_t length;
} __attribute__((packed)) cc2420x_header_t;
#line 43
#line 33
typedef struct cc2420x_metadata_t {

  uint8_t lqi;
  union  {

    uint8_t power;
    uint8_t rssi;
  } ;

  uint16_t rssi_idle;
} cc2420x_metadata_t;

enum cc2420X_timing_enums {
  CC2420X_SYMBOL_TIME = 16, 
  IDLE_2_RX_ON_TIME = 12 * CC2420X_SYMBOL_TIME, 
  PD_2_IDLE_TIME = 860, 
  STROBE_TO_TX_ON_TIME = 12 * CC2420X_SYMBOL_TIME, 






  TX_SFD_DELAY = STROBE_TO_TX_ON_TIME + 10 * CC2420X_SYMBOL_TIME - 25, 

  RX_SFD_DELAY = 0, 


  RSSI_VALID_WAIT = 256
};

enum cc2420X_reg_access_enums {
  CC2420X_CMD_REGISTER_MASK = 0x3f, 
  CC2420X_CMD_REGISTER_READ = 0x40, 
  CC2420X_CMD_REGISTER_WRITE = 0x00, 
  CC2420X_CMD_TXRAM_WRITE = 0x80
};
#line 84
#line 71
typedef union cc2420X_status {
  uint16_t value;
  struct  {
    unsigned reserved0 : 1;
    unsigned rssi_valid : 1;
    unsigned lock : 1;
    unsigned tx_active : 1;

    unsigned enc_busy : 1;
    unsigned tx_underflow : 1;
    unsigned xosc16m_stable : 1;
    unsigned reserved7 : 1;
  } ;
} cc2420X_status_t;
#line 97
#line 86
typedef union cc2420X_iocfg0 {
  uint16_t value;
  struct __nesc_unnamed4273 {
    unsigned fifop_thr : 7;
    unsigned cca_polarity : 1;
    unsigned sfd_polarity : 1;
    unsigned fifop_polarity : 1;
    unsigned fifo_polarity : 1;
    unsigned bcn_accept : 1;
    unsigned reserved : 4;
  } f;
} cc2420X_iocfg0_t;


static const cc2420X_iocfg0_t cc2420X_iocfg0_default = { .f.fifop_thr = 64, .f.cca_polarity = 0, .f.sfd_polarity = 0, .f.fifop_polarity = 0, .f.fifo_polarity = 0, .f.bcn_accept = 0, .f.reserved = 0 };









#line 102
typedef union cc2420X_iocfg1 {
  uint16_t value;
  struct __nesc_unnamed4274 {
    unsigned ccamux : 5;
    unsigned sfdmux : 5;
    unsigned hssd_src : 3;
    unsigned reserved : 3;
  } f;
} cc2420X_iocfg1_t;
#line 124
#line 114
typedef union cc2420X_fsctrl {
  uint16_t value;
  struct __nesc_unnamed4275 {
    unsigned freq : 10;
    unsigned lock_status : 1;
    unsigned lock_length : 1;
    unsigned cal_running : 1;
    unsigned cal_done : 1;
    unsigned lock_thr : 2;
  } f;
} cc2420X_fsctrl_t;

static const cc2420X_fsctrl_t cc2420X_fsctrl_default = { .f.lock_thr = 1, .f.freq = 357, .f.lock_status = 0, .f.lock_length = 0, .f.cal_running = 0, .f.cal_done = 0 };
#line 141
#line 128
typedef union cc2420X_mdmctrl0 {
  uint16_t value;
  struct __nesc_unnamed4276 {
    unsigned preamble_length : 4;
    unsigned autoack : 1;
    unsigned autocrc : 1;
    unsigned cca_mode : 2;
    unsigned cca_hyst : 3;
    unsigned adr_decode : 1;
    unsigned pan_coordinator : 1;
    unsigned reserved_frame_mode : 1;
    unsigned reserved : 2;
  } f;
} cc2420X_mdmctrl0_t;

static const cc2420X_mdmctrl0_t cc2420X_mdmctrl0_default = { .f.preamble_length = 2, .f.autocrc = 1, .f.cca_mode = 3, .f.cca_hyst = 2, .f.adr_decode = 1 };
#line 156
#line 145
typedef union cc2420X_txctrl {
  uint16_t value;
  struct __nesc_unnamed4277 {
    unsigned pa_level : 5;
    unsigned reserved : 1;
    unsigned pa_current : 3;
    unsigned txmix_current : 2;
    unsigned txmix_caparray : 2;
    unsigned tx_turnaround : 1;
    unsigned txmixbuf_cur : 2;
  } f;
} cc2420X_txctrl_t;

static const cc2420X_txctrl_t cc2420X_txctrl_default = { .f.pa_level = 31, .f.reserved = 1, .f.pa_current = 3, .f.tx_turnaround = 1, .f.txmixbuf_cur = 2 };










enum __nesc_unnamed4278 {
  CC2420X_TX_PWR_MASK = 0x1f, 
  CC2420X_CHANNEL_MASK = 0x1f
};

enum cc2420X_config_reg_enums {
  CC2420X_SNOP = 0x00, 
  CC2420X_SXOSCON = 0x01, 
  CC2420X_STXCAL = 0x02, 
  CC2420X_SRXON = 0x03, 
  CC2420X_STXON = 0x04, 
  CC2420X_STXONCCA = 0x05, 
  CC2420X_SRFOFF = 0x06, 
  CC2420X_SXOSCOFF = 0x07, 
  CC2420X_SFLUSHRX = 0x08, 
  CC2420X_SFLUSHTX = 0x09, 
  CC2420X_SACK = 0x0a, 
  CC2420X_SACKPEND = 0x0b, 
  CC2420X_SRXDEC = 0x0c, 
  CC2420X_STXENC = 0x0d, 
  CC2420X_SAES = 0x0e, 
  CC2420X_MAIN = 0x10, 
  CC2420X_MDMCTRL0 = 0x11, 
  CC2420X_MDMCTRL1 = 0x12, 
  CC2420X_RSSI = 0x13, 
  CC2420X_SYNCWORD = 0x14, 
  CC2420X_TXCTRL = 0x15, 
  CC2420X_RXCTRL0 = 0x16, 
  CC2420X_RXCTRL1 = 0x17, 
  CC2420X_FSCTRL = 0x18, 
  CC2420X_SECCTRL0 = 0x19, 
  CC2420X_SECCTRL1 = 0x1a, 
  CC2420X_BATTMON = 0x1b, 
  CC2420X_IOCFG0 = 0x1c, 
  CC2420X_IOCFG1 = 0x1d, 
  CC2420X_MANFIDL = 0x1e, 
  CC2420X_MANFIDH = 0x1f, 
  CC2420X_FSMTC = 0x20, 
  CC2420X_MANAND = 0x21, 
  CC2420X_MANOR = 0x22, 
  CC2420X_AGCCTRL = 0x23, 
  CC2420X_AGCTST0 = 0x24, 
  CC2420X_AGCTST1 = 0x25, 
  CC2420X_AGCTST2 = 0x26, 
  CC2420X_FSTST0 = 0x27, 
  CC2420X_FSTST1 = 0x28, 
  CC2420X_FSTST2 = 0x29, 
  CC2420X_FSTST3 = 0x2a, 
  CC2420X_RXBPFTST = 0x2b, 
  CC2420X_FSMSTATE = 0x2c, 
  CC2420X_ADCTST = 0x2d, 
  CC2420X_DACTST = 0x2e, 
  CC2420X_TOPTST = 0x2f, 
  CC2420X_TXFIFO = 0x3e, 
  CC2420X_RXFIFO = 0x3f
};

enum cc2420X_ram_addr_enums {
  CC2420X_RAM_TXFIFO = 0x000, 
  CC2420X_RAM_TXFIFO_END = 0x7f, 
  CC2420X_RAM_RXFIFO = 0x080, 
  CC2420X_RAM_KEY0 = 0x100, 
  CC2420X_RAM_RXNONCE = 0x110, 
  CC2420X_RAM_SABUF = 0x120, 
  CC2420X_RAM_KEY1 = 0x130, 
  CC2420X_RAM_TXNONCE = 0x140, 
  CC2420X_RAM_CBCSTATE = 0x150, 
  CC2420X_RAM_IEEEADR = 0x160, 
  CC2420X_RAM_PANID = 0x168, 
  CC2420X_RAM_SHORTADR = 0x16a
};
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/RadioConfig.h"
typedef TMicro TRadio;
typedef uint16_t tradio_size;
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/IeeeEui64.h"
enum __nesc_unnamed4279 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 51
typedef struct __nesc_unnamed4280 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4281 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4282 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4283 {
  IEEE154_MIN_HDR_SZ = 6
};
#line 86
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3, 
  IEEE154_TYPE_MASK = 7
};

enum ieee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3, 
  IEEE154_ADDR_MASK = 3
};
# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.h"
#line 40
typedef nx_struct ieee154_simple_header_t {

  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;
} __attribute__((packed)) ieee154_simple_header_t;
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/TinyosNetworkLayer.h"
#line 42
typedef nx_struct network_header_t {

  nxle_uint8_t network;
} __attribute__((packed)) network_header_t;
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeStampingLayer.h"
#line 38
typedef struct timestamp_metadata_t {

  uint32_t timestamp;
} timestamp_metadata_t;
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/MetadataFlagsLayer.h"
#line 38
typedef struct flags_metadata_t {


  uint8_t flags;
} flags_metadata_t;
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/ActiveMessageLayer.h"
#line 40
typedef nx_struct activemessage_header_t {

  nx_am_id_t type;
} __attribute__((packed)) activemessage_header_t;
# 4 "../iMAC_TDMA/cc2420x/FastCC2420Transceiver.h"
enum __nesc_unnamed4284 {
  RSSI_OFFSET = 45
};
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/LowPowerListeningLayer.h"
#line 38
typedef struct lpl_metadata_t {

  uint16_t sleepint;
} lpl_metadata_t;
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/PacketLinkLayer.h"
#line 38
typedef struct link_metadata_t {

  uint16_t maxRetries;
  uint16_t retryDelay;
} link_metadata_t;
# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XRadio.h"
#line 37
typedef nx_struct cc2420xpacket_header_t {

  cc2420x_header_t cc2420x;
  ieee154_simple_header_t ieee154;

  network_header_t network;


  activemessage_header_t am;
} __attribute__((packed)) 
cc2420xpacket_header_t;




#line 49
typedef nx_struct cc2420xpacket_footer_t {
} __attribute__((packed)) 

cc2420xpacket_footer_t;
#line 65
#line 54
typedef struct cc2420xpacket_metadata_t {







  timestamp_metadata_t timestamp;
  flags_metadata_t flags;
  cc2420x_metadata_t cc2420x;
} cc2420xpacket_metadata_t;
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/TimeSyncMessageLayer.h"
typedef nx_int32_t timesync_relative_t;


typedef nx_uint32_t timesync_absolute_t;









#line 50
typedef nx_struct timesync_footer_t {

  nx_am_id_t type;
  nx_union timestamp_t {

    timesync_relative_t relative;
    timesync_absolute_t absolute;
  } __attribute__((packed)) timestamp;
} __attribute__((packed)) timesync_footer_t;
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4285 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4286 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4287 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4288 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4289 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4290 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4291 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4292 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4293 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4294 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4295 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

msp430_uart_union_config_t msp430_uart_default_config = { 
{ 
.utxe = 1, 
.urxe = 1, 
.ubr = UBR_1MHZ_57600, 
.umctl = UMCTL_1MHZ_57600, 
.ssel = 0x02, 
.pena = 0, 
.pev = 0, 
.spb = 0, 
.clen = 1, 
.listen = 0, 
.mm = 0, 
.ckpl = 0, 
.urxse = 0, 
.urxeie = 1, 
.urxwie = 0, 
.utxe = 1, 
.urxe = 1 } };
#line 248
#line 240
typedef struct __nesc_unnamed4296 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4297 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4298 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4299 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4300 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4301 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4302 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounter.h"
#line 38
typedef struct T2ghz {
  uint8_t dummy;
} T2ghz;
# 91 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
typedef TMilli RadioCountToLedsC__MilliTimer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4303 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef uint8_t FastCC2420TransceiverP__PacketRSSI__value_type;
typedef uint16_t FastCC2420TransceiverP__PacketRSSIIdle__value_type;
typedef TRadio FastCC2420TransceiverP__PacketTimeStampRadio__precision_tag;
typedef uint32_t FastCC2420TransceiverP__PacketTimeStampRadio__size_type;
typedef TMicro CC2420XDriverLayerP__BusyWait__precision_tag;
typedef uint16_t CC2420XDriverLayerP__BusyWait__size_type;
typedef uint8_t CC2420XDriverLayerP__PacketRSSI__value_type;
typedef TRadio CC2420XDriverLayerP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420XDriverLayerP__PacketTimeStamp__size_type;
typedef TRadio CC2420XDriverLayerP__LocalTime__precision_tag;
typedef uint8_t CC2420XDriverLayerP__PacketTransmitPower__value_type;
typedef uint16_t CC2420XDriverLayerP__PacketRSSIIdle__value_type;
typedef uint8_t CC2420XDriverLayerP__PacketTimeSyncOffset__value_type;
typedef uint8_t CC2420XDriverLayerP__PacketLinkQuality__value_type;
typedef TMicro /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag;
typedef uint16_t /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type;
enum /*HplCC2420XC.SpiC*/Msp430Spi0C__0____nesc_unnamed4304 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4305 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*HplCC2420XC.AlarmC.Msp430Timer*/Msp430TimerMicroC__0____nesc_unnamed4306 {
  Msp430TimerMicroC__0__ALARM_ID = 0U
};
typedef TMicro /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMicro Msp430HybridAlarmCounterP__CounterMicro__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__CounterMicro__size_type;
typedef T2ghz Msp430HybridAlarmCounterP__Counter2ghz__precision_tag;
typedef uint32_t Msp430HybridAlarmCounterP__Counter2ghz__size_type;
typedef T32khz Msp430HybridAlarmCounterP__Alarm32khz__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__Alarm32khz__size_type;
typedef T2ghz Msp430HybridAlarmCounterP__Alarm2ghz__precision_tag;
typedef uint32_t Msp430HybridAlarmCounterP__Alarm2ghz__size_type;
typedef TMicro Msp430HybridAlarmCounterP__AlarmMicro__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__AlarmMicro__size_type;
typedef T32khz Msp430HybridAlarmCounterP__Counter32khz__precision_tag;
typedef uint16_t Msp430HybridAlarmCounterP__Counter32khz__size_type;
enum /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4307 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__frequency_tag;
typedef /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__frequency_tag /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__precision_tag;
typedef uint16_t /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__size_type;
enum /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Timer*/Msp430TimerMicroC__1____nesc_unnamed4308 {
  Msp430TimerMicroC__1__ALARM_ID = 1U
};
typedef TMicro /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__frequency_tag;
typedef /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__frequency_tag /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__precision_tag;
typedef uint16_t /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type;
typedef TMicro /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type;
typedef T2ghz /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_precision_tag;
typedef uint32_t /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type;
typedef uint32_t /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_precision_tag /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__size_type;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_precision_tag /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__precision_tag;
typedef /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__size_type;
typedef TMicro /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
typedef TMilli UartLogP__LocalTime__precision_tag;
enum /*PlatformSerialC.UartC*/Msp430Uart1C__0____nesc_unnamed4309 {
  Msp430Uart1C__0__CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0____nesc_unnamed4310 {
  Msp430Usart1C__0__CLIENT_ID = 0U
};
enum SerialAMQueueP____nesc_unnamed4311 {
  SerialAMQueueP__NUM_CLIENTS = 1U
};
typedef TRadio /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__precision_tag;
typedef tradio_size /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type;
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 45
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4068d788);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4068d788);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 58
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 68
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 42
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );

static bool /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled(void );
#line 58
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405fbb40);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405fbb40);
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void RadioCountToLedsC__AMControl__startDone(error_t error);
#line 138
static void RadioCountToLedsC__AMControl__stopDone(error_t error);
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Boot.nc"
static void RadioCountToLedsC__Boot__booted(void );
# 13 "../iMAC_TDMA/async/FastSend.nc"
static void RadioCountToLedsC__AMSend__sendDone(message_t *msg, error_t error);
# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void RadioCountToLedsC__MilliTimer__fired(void );
# 11 "../iMAC_TDMA/async/FastReceive.nc"
static message_t *RadioCountToLedsC__Receive__receive(message_t *msg, void *payload, uint8_t len);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Leds.nc"
static void LedsP__Leds__led1Toggle(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 99
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 99
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 92
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );
#line 99
static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 99
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409989f0);
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409989f0, 
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
uint32_t dt);
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t *FastCC2420TransceiverP__SubReceive__receive(message_t *msg);
#line 46
static bool FastCC2420TransceiverP__SubReceive__header(message_t *msg);
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t FastCC2420TransceiverP__SplitControl__start(void );
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void FastCC2420TransceiverP__SubSend__ready(void );
#line 56
static void FastCC2420TransceiverP__SubSend__sendDone(error_t error);
# 67 "../iMAC_TDMA/async/FastPacket.nc"
static uint8_t FastCC2420TransceiverP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


FastCC2420TransceiverP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t FastCC2420TransceiverP__Packet__maxPayloadLength(void );
#line 83
static void FastCC2420TransceiverP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static void FastCC2420TransceiverP__RadioState__done(void );
# 11 "../iMAC_TDMA/async/FastReceive.nc"
static message_t *FastCC2420TransceiverP__Snoop__default__receive(
# 41 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fdf08, 
# 11 "../iMAC_TDMA/async/FastReceive.nc"
message_t *msg, void *payload, uint8_t len);
# 11 "../iMAC_TDMA/async/FastSend.nc"
static error_t FastCC2420TransceiverP__Send__send(
# 39 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fede8, 
# 11 "../iMAC_TDMA/async/FastSend.nc"
am_addr_t dest, message_t *msg, uint8_t len);

static void FastCC2420TransceiverP__Send__default__sendDone(
# 39 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fede8, 
# 13 "../iMAC_TDMA/async/FastSend.nc"
message_t *msg, error_t error);
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t FastCC2420TransceiverP__RadioPacket__payloadLength(message_t *msg);









static uint8_t FastCC2420TransceiverP__RadioPacket__maxPayloadLength(void );
#line 54
static void FastCC2420TransceiverP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t FastCC2420TransceiverP__RadioPacket__headerLength(message_t *msg);
# 35 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
static uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__maxPayloadLength(void );
#line 29
static uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__headerLength(message_t *msg);
#line 41
static uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__metadataLength(message_t *msg);






static uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__headerPreloadLength(void );





static bool FastCC2420TransceiverP__CC2420XDriverConfig__requiresRssiCca(message_t *msg);
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
static void FastCC2420TransceiverP__SoftwareAckConfig__reportChannelError(void );
#line 80
static void FastCC2420TransceiverP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);
#line 55
static bool FastCC2420TransceiverP__SoftwareAckConfig__requiresAckWait(message_t *msg);






static bool FastCC2420TransceiverP__SoftwareAckConfig__isAckPacket(message_t *msg);






static bool FastCC2420TransceiverP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);
#line 43
static uint16_t FastCC2420TransceiverP__SoftwareAckConfig__getAckTimeout(void );





static void FastCC2420TransceiverP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack);
#line 75
static bool FastCC2420TransceiverP__SoftwareAckConfig__requiresAckReply(message_t *msg);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t FastCC2420TransceiverP__Init__init(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void FastCC2420TransceiverP__stopTask__runTask(void );
# 11 "../iMAC_TDMA/async/FastReceive.nc"
static message_t *FastCC2420TransceiverP__Receive__default__receive(
# 40 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fd848, 
# 11 "../iMAC_TDMA/async/FastReceive.nc"
message_t *msg, void *payload, uint8_t len);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static void FastCC2420TransceiverP__PacketTimeStampRadio__set(
#line 73
message_t * msg, 




FastCC2420TransceiverP__PacketTimeStampRadio__size_type value);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void FastCC2420TransceiverP__startTask__runTask(void );
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void FastCC2420TransceiverP__Resource__granted(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void FastCC2420TransceiverP__PacketFlag__clear(
# 48 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
uint8_t arg_0x40a0ed28, 
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
message_t *msg);
#line 40
static bool FastCC2420TransceiverP__PacketFlag__get(
# 48 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
uint8_t arg_0x40a0ed28, 
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
message_t *msg);









static void FastCC2420TransceiverP__PacketFlag__set(
# 48 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
uint8_t arg_0x40a0ed28, 
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
message_t *msg);
# 57 "../iMAC_TDMA/async/FastAMPacket.nc"
static am_addr_t FastCC2420TransceiverP__AMPacket__address(void );









static am_addr_t FastCC2420TransceiverP__AMPacket__destination(
#line 63
message_t * amsg);
#line 110
static void FastCC2420TransceiverP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void FastCC2420TransceiverP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t FastCC2420TransceiverP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void FastCC2420TransceiverP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool FastCC2420TransceiverP__AMPacket__isForMe(
#line 122
message_t * amsg);
#line 176
static void FastCC2420TransceiverP__AMPacket__setGroup(
#line 173
message_t * amsg, 


am_group_t grp);







static am_group_t FastCC2420TransceiverP__AMPacket__localGroup(void );
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static void CC2420XDriverLayerP__PacketRSSI__set(message_t *msg, CC2420XDriverLayerP__PacketRSSI__value_type value);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t CC2420XDriverLayerP__SoftwareInit__init(void );
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void CC2420XDriverLayerP__RadioAlarm__fired(void );
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static error_t CC2420XDriverLayerP__RadioState__turnOn(void );
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static CC2420XDriverLayerP__PacketTransmitPower__value_type CC2420XDriverLayerP__PacketTransmitPower__get(message_t *msg);
#line 40
static bool CC2420XDriverLayerP__PacketTransmitPower__isSet(message_t *msg);
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t CC2420XDriverLayerP__RadioPacket__payloadLength(message_t *msg);









static uint8_t CC2420XDriverLayerP__RadioPacket__maxPayloadLength(void );
#line 54
static void CC2420XDriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t CC2420XDriverLayerP__RadioPacket__headerLength(message_t *msg);
#line 65
static uint8_t CC2420XDriverLayerP__RadioPacket__metadataLength(message_t *msg);
# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void CC2420XDriverLayerP__FifopInterrupt__fired(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420XDriverLayerP__releaseSpi__runTask(void );
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static CC2420XDriverLayerP__PacketTimeSyncOffset__value_type CC2420XDriverLayerP__PacketTimeSyncOffset__get(message_t *msg);
#line 40
static bool CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420XDriverLayerP__SpiResource__granted(void );
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketField.nc"
static void CC2420XDriverLayerP__PacketLinkQuality__set(message_t *msg, CC2420XDriverLayerP__PacketLinkQuality__value_type value);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t CC2420XDriverLayerP__RadioSend__send(message_t *msg);
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static void CC2420XDriverLayerP__SfdCapture__captured(uint16_t time);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void CC2420XDriverLayerP__Tasklet__run(void );
# 66 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/BusyWait.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type dt);
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void TaskletC__Tasklet__schedule(void );
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bff6b0);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bff6b0);
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfd838, 
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfcab8);
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bdac30);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bdac30);
# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bdac30);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bdac30);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bdac30);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(uint8_t data);
#line 74
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(uint8_t data);
#line 68
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );









static bool HplMsp430Usart0P__Usart__isTxIntrPending(void );
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108, 
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf988);
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce1690);
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce1690);
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 7 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );


static void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
static void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 66
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable(void );
#line 53
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable(void );
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 72
static void HplMsp430InterruptP__Port14__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port26__clear(void );
#line 72
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port17__clear(void );
#line 72
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port21__clear(void );
#line 72
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port12__clear(void );
#line 72
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port24__clear(void );
#line 72
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port15__clear(void );
#line 72
static void HplMsp430InterruptP__Port15__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port27__clear(void );
#line 72
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port10__clear(void );
#line 47
static void HplMsp430InterruptP__Port10__disable(void );




static void HplMsp430InterruptP__Port22__clear(void );
#line 72
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port13__clear(void );
#line 72
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port25__clear(void );
#line 72
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port16__clear(void );
#line 72
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port20__clear(void );
#line 72
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port11__clear(void );
#line 72
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 52
static void HplMsp430InterruptP__Port23__clear(void );
#line 72
static void HplMsp430InterruptP__Port23__default__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(void );
#line 103
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 66
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(/*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);






static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void Msp430HybridAlarmCounterP__CounterMicro__overflow(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430HybridAlarmCounterP__McuPowerOverride__lowestState(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static Msp430HybridAlarmCounterP__Counter2ghz__size_type Msp430HybridAlarmCounterP__Counter2ghz__get(void );






static bool Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void Msp430HybridAlarmCounterP__Alarm32khz__fired(void );
#line 78
static void Msp430HybridAlarmCounterP__Alarm2ghz__default__fired(void );
#line 78
static void Msp430HybridAlarmCounterP__AlarmMicro__fired(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void Msp430HybridAlarmCounterP__Counter32khz__overflow(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(/*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type t0, /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__size_type dt);
#line 88
static bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow(void );
#line 64
static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/LocalTime.nc"
static uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void UartLogP__uartSendTask__runTask(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t UartLogP__Init__init(void );
# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void UartLogP__UartSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void UartLogP__UartControl__startDone(error_t error);
#line 138
static void UartLogP__UartControl__stopDone(error_t error);
# 6 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLog.nc"
static error_t UartLogP__UartLog__logEntry(__nesc_nxbase_nx_uint8_t type, __nesc_nxbase_nx_uint8_t sourceId, __nesc_nxbase_nx_uint16_t seq, __nesc_nxbase_nx_uint32_t timestamp);
#line 4
static error_t UartLogP__UartLog__logByteStream(nx_uint8_t *log_p, uint8_t len);
# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(
# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40f01068, 
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 106
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
#line 94
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40f01a28, 
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(
#line 74
message_t * amsg);
#line 103
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 147
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(
#line 143
message_t * amsg);
#line 162
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t SerialP__SplitControl__start(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SerialP__stopDoneTask__runTask(void );
#line 75
static void SerialP__RunTx__runTask(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t SerialP__Init__init(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flushDone(void );
#line 49
static void SerialP__SerialFlush__default__flush(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SerialP__startDoneTask__runTask(void );
# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
static void SerialP__SerialFrameComm__dataReceived(uint8_t data);





static void SerialP__SerialFrameComm__putDone(void );
#line 85
static void SerialP__SerialFrameComm__delimiterReceived(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SerialP__defaultSerialFlushTask__runTask(void );
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP__SendBytePacket__completeSend(void );
#line 62
static error_t SerialP__SendBytePacket__startSend(uint8_t first_byte);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc2760, 
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc2760, 
# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc2120, 
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc12a0, 
# 31 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc12a0);
# 23 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc12a0, 
# 23 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
static void HdlcTranslateC__UartStream__receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC__UartStream__receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC__UartStream__sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );
#line 79
static void HdlcTranslateC__SerialFrameComm__resetReceive(void );
#line 65
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x410024f8);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x410024f8);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000a88);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len);
#line 79
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41005aa0);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41005aa0);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41005aa0);
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40ffcb30, 
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40ffcb30);
# 143 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P__Usart__enableUartRx(void );
#line 123
static void HplMsp430Usart1P__Usart__enableUart(void );
#line 97
static void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart1P__Usart__setUmctl(uint8_t umctl);
#line 133
static void HplMsp430Usart1P__Usart__enableUartTx(void );
#line 148
static void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 182
static void HplMsp430Usart1P__Usart__enableIntr(void );
#line 207
static void HplMsp430Usart1P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart1P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart1P__Usart__disableUart(void );
#line 174
static void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 158
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 138
static void HplMsp430Usart1P__Usart__disableUartTx(void );
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P__AsyncStdControl__start(void );









static error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108, 
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108);
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce1690);
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void TelosSerialP__Resource__granted(void );
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/StdControl.nc"
static error_t TelosSerialP__StdControl__start(void );









static error_t TelosSerialP__StdControl__stop(void );
# 31 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );







static uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
am_id_t arg_0x41120490, 
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x411219d8, 
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x411219d8, 
# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 75
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 55 "ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(message_t *msg);
#line 156
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);
#line 69
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(message_t *msg);
#line 125
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);
#line 88
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);
#line 63
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(message_t *msg);
#line 99
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(message_t *msg);
#line 94
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);
#line 162
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(message_t *msg);
#line 146
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);
#line 136
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);
#line 105
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack);
#line 168
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 141
static uint16_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(message_t *msg);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__Init__init(void );
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
#line 62
static bool /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b2e18);
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b2e18);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b1a58);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b23f0);
# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b23f0);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b23f0);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x411cbe38);
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x411cbe38, 
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
tradio_size timeout);




static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x411cbe38);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static bool /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x411cbe38);
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static tradio_size /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x411cbe38);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run(void );
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__ready(void );
#line 56
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__sendDone(error_t error);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__receive(message_t *msg);
#line 46
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__header(message_t *msg);
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__fired(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__send(message_t *msg);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketAcknowledgements.nc"
static error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__requestAck(
#line 53
message_t * msg);
#line 85
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__wasAcked(
#line 80
message_t * msg);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4312 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 76
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 97
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 112
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 127
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 160
static inline void Msp430ClockP__startTimerB(void );
#line 172
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 201
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 226
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4068d788);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4068d788);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t x);
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 135
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 110
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 130
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 175
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 192
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );




static inline bool /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled(void );









static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 117
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405fbb40);
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4313 {

  SchedulerBasicP__NUM_TASKS = 18U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 6 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLog.nc"
static error_t RadioCountToLedsC__UartLog__logEntry(__nesc_nxbase_nx_uint8_t type, __nesc_nxbase_nx_uint8_t sourceId, __nesc_nxbase_nx_uint16_t seq, __nesc_nxbase_nx_uint32_t timestamp);
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t RadioCountToLedsC__AMControl__start(void );
# 11 "../iMAC_TDMA/async/FastSend.nc"
static error_t RadioCountToLedsC__AMSend__send(am_addr_t dest, message_t *msg, uint8_t len);
# 115 "../iMAC_TDMA/async/FastPacket.nc"
static 
#line 112
void * 


RadioCountToLedsC__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void RadioCountToLedsC__MilliTimer__startPeriodic(uint32_t dt);
# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Leds.nc"
static void RadioCountToLedsC__Leds__led1Toggle(void );
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static bool RadioCountToLedsC__CCA__get(void );
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketAcknowledgements.nc"
static error_t RadioCountToLedsC__Acks__requestAck(
#line 53
message_t * msg);
#line 85
static bool RadioCountToLedsC__Acks__wasAcked(
#line 80
message_t * msg);
# 78 "RadioCountToLedsC.nc"
message_t RadioCountToLedsC__packet;

bool RadioCountToLedsC__locked;
uint16_t RadioCountToLedsC__counter = 0;
#line 105
static inline void RadioCountToLedsC__Boot__booted(void );



static inline void RadioCountToLedsC__AMControl__startDone(error_t err);








static inline void RadioCountToLedsC__AMControl__stopDone(error_t err);



static inline void RadioCountToLedsC__MilliTimer__fired(void );
#line 152
static inline message_t *RadioCountToLedsC__Receive__receive(message_t *msg, void *payload, uint8_t len);
#line 164
static inline void RadioCountToLedsC__AMSend__sendDone(message_t *msg, error_t error);
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );

static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );





static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 99
static inline void LedsP__Leds__led1Toggle(void );
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void );
#line 65
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );



static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void );
#line 67
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );



static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4314 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4315 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4316 {
#line 74
  AlarmToTimerC__0__fired = 0U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409989f0);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4317 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4318 {

  VirtualizeTimerC__0__NUM_TIMERS = 1U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4319 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 100
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void FastCC2420TransceiverP__SplitControl__startDone(error_t error);
#line 138
static void FastCC2420TransceiverP__SplitControl__stopDone(error_t error);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t FastCC2420TransceiverP__SubSend__send(message_t *msg);
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void FastCC2420TransceiverP__TimeStampFlag__set(message_t *msg);
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static error_t FastCC2420TransceiverP__RadioState__turnOn(void );
# 11 "../iMAC_TDMA/async/FastReceive.nc"
static message_t *FastCC2420TransceiverP__Snoop__receive(
# 41 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fdf08, 
# 11 "../iMAC_TDMA/async/FastReceive.nc"
message_t *msg, void *payload, uint8_t len);
# 13 "../iMAC_TDMA/async/FastSend.nc"
static void FastCC2420TransceiverP__Send__sendDone(
# 39 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fede8, 
# 13 "../iMAC_TDMA/async/FastSend.nc"
message_t *msg, error_t error);
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t FastCC2420TransceiverP__ActiveMessageAddress__amAddress(void );




static am_group_t FastCC2420TransceiverP__ActiveMessageAddress__amGroup(void );
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static uint8_t FastCC2420TransceiverP__SubPacket__payloadLength(message_t *msg);









static uint8_t FastCC2420TransceiverP__SubPacket__maxPayloadLength(void );
#line 54
static void FastCC2420TransceiverP__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t FastCC2420TransceiverP__SubPacket__headerLength(message_t *msg);
#line 65
static uint8_t FastCC2420TransceiverP__SubPacket__metadataLength(message_t *msg);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
static bool FastCC2420TransceiverP__Ieee154PacketLayer__isAckFrame(message_t *msg);
#line 156
static void FastCC2420TransceiverP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);
#line 69
static void FastCC2420TransceiverP__Ieee154PacketLayer__createDataFrame(message_t *msg);
#line 125
static void FastCC2420TransceiverP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);
#line 88
static void FastCC2420TransceiverP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);
#line 63
static bool FastCC2420TransceiverP__Ieee154PacketLayer__isDataFrame(message_t *msg);
#line 94
static bool FastCC2420TransceiverP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);
#line 162
static bool FastCC2420TransceiverP__Ieee154PacketLayer__requiresAckWait(message_t *msg);
#line 146
static void FastCC2420TransceiverP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);
#line 136
static void FastCC2420TransceiverP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);
#line 105
static void FastCC2420TransceiverP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack);
#line 168
static bool FastCC2420TransceiverP__Ieee154PacketLayer__requiresAckReply(message_t *msg);
#line 141
static uint16_t FastCC2420TransceiverP__Ieee154PacketLayer__getDestAddr(message_t *msg);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t FastCC2420TransceiverP__stopTask__postTask(void );
# 11 "../iMAC_TDMA/async/FastReceive.nc"
static message_t *FastCC2420TransceiverP__Receive__receive(
# 40 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
am_id_t arg_0x409fd848, 
# 11 "../iMAC_TDMA/async/FastReceive.nc"
message_t *msg, void *payload, uint8_t len);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t FastCC2420TransceiverP__startTask__postTask(void );
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t FastCC2420TransceiverP__Resource__immediateRequest(void );
#line 88
static error_t FastCC2420TransceiverP__Resource__request(void );
# 106 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
enum FastCC2420TransceiverP____nesc_unnamed4320 {
#line 106
  FastCC2420TransceiverP__startTask = 2U
};
#line 106
typedef int FastCC2420TransceiverP____nesc_sillytask_startTask[FastCC2420TransceiverP__startTask];


enum FastCC2420TransceiverP____nesc_unnamed4321 {
#line 109
  FastCC2420TransceiverP__stopTask = 3U
};
#line 109
typedef int FastCC2420TransceiverP____nesc_sillytask_stopTask[FastCC2420TransceiverP__stopTask];
#line 73
uint8_t FastCC2420TransceiverP__seqno;
message_t *FastCC2420TransceiverP__p_msg;

static inline error_t FastCC2420TransceiverP__Init__init(void );





static inline void FastCC2420TransceiverP__Resource__granted(void );



static activemessage_header_t *FastCC2420TransceiverP__getHeader(message_t *msg);




static inline network_header_t *FastCC2420TransceiverP__getNetworkHeader(message_t *msg);
#line 104
bool FastCC2420TransceiverP__is_on = TRUE;

static inline void FastCC2420TransceiverP__startTask__runTask(void );


static inline void FastCC2420TransceiverP__stopTask__runTask(void );




static inline void FastCC2420TransceiverP__RadioState__done(void );









static error_t FastCC2420TransceiverP__SplitControl__start(void );
#line 139
bool FastCC2420TransceiverP__pending = FALSE;
static inline error_t FastCC2420TransceiverP__Send__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len);
#line 171
static inline void FastCC2420TransceiverP__SubSend__ready(void );

static void FastCC2420TransceiverP__SubSend__sendDone(error_t error);
#line 196
static inline void FastCC2420TransceiverP__Send__default__sendDone(am_id_t id, message_t *msg, error_t error);





static inline bool FastCC2420TransceiverP__SubReceive__header(message_t *msg);



static inline message_t *FastCC2420TransceiverP__SubReceive__receive(message_t *msg);
#line 221
static inline message_t *FastCC2420TransceiverP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);


static inline message_t *FastCC2420TransceiverP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);






static __inline am_addr_t FastCC2420TransceiverP__AMPacket__address(void );



static __inline am_group_t FastCC2420TransceiverP__AMPacket__localGroup(void );



static __inline bool FastCC2420TransceiverP__AMPacket__isForMe(message_t *msg);




static __inline am_addr_t FastCC2420TransceiverP__AMPacket__destination(message_t *msg);




static __inline void FastCC2420TransceiverP__AMPacket__setDestination(message_t *msg, am_addr_t addr);









static __inline void FastCC2420TransceiverP__AMPacket__setSource(message_t *msg, am_addr_t addr);




static __inline am_id_t FastCC2420TransceiverP__AMPacket__type(message_t *msg);



static __inline void FastCC2420TransceiverP__AMPacket__setType(message_t *msg, am_id_t type);








static __inline void FastCC2420TransceiverP__AMPacket__setGroup(message_t *msg, am_group_t grp);
#line 289
static inline uint8_t FastCC2420TransceiverP__RadioPacket__headerLength(message_t *msg);




static inline uint8_t FastCC2420TransceiverP__RadioPacket__payloadLength(message_t *msg);




static inline void FastCC2420TransceiverP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);




static uint8_t FastCC2420TransceiverP__RadioPacket__maxPayloadLength(void );










static flags_metadata_t *FastCC2420TransceiverP__getMeta(message_t *msg);
#line 332
static inline uint8_t FastCC2420TransceiverP__Packet__payloadLength(message_t *msg);



static inline void FastCC2420TransceiverP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t FastCC2420TransceiverP__Packet__maxPayloadLength(void );



static void *FastCC2420TransceiverP__Packet__getPayload(message_t *msg, uint8_t len);









static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__headerLength(message_t *msg);



static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__maxPayloadLength(void );



static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__metadataLength(message_t *msg);



static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__headerPreloadLength(void );




static inline bool FastCC2420TransceiverP__CC2420XDriverConfig__requiresRssiCca(message_t *msg);








static inline bool FastCC2420TransceiverP__PacketFlag__get(uint8_t bit, message_t *msg);



static inline void FastCC2420TransceiverP__PacketFlag__set(uint8_t bit, message_t *msg);



static inline void FastCC2420TransceiverP__PacketFlag__clear(uint8_t bit, message_t *msg);
#line 402
static timestamp_metadata_t *FastCC2420TransceiverP__getMetaTimestamp(message_t *msg);
#line 418
static inline void FastCC2420TransceiverP__PacketTimeStampRadio__set(message_t *msg, uint32_t value);
#line 440
static inline bool FastCC2420TransceiverP__SoftwareAckConfig__requiresAckWait(message_t *msg);




static inline bool FastCC2420TransceiverP__SoftwareAckConfig__isAckPacket(message_t *msg);




static inline bool FastCC2420TransceiverP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack);




static inline void FastCC2420TransceiverP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack);




static inline bool FastCC2420TransceiverP__SoftwareAckConfig__requiresAckReply(message_t *msg);




static inline void FastCC2420TransceiverP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack);









static inline uint16_t FastCC2420TransceiverP__SoftwareAckConfig__getAckTimeout(void );




static inline void FastCC2420TransceiverP__SoftwareAckConfig__reportChannelError(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__FIFO__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__FIFO__get(void );
# 66 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/BusyWait.nc"
static void CC2420XDriverLayerP__BusyWait__wait(CC2420XDriverLayerP__BusyWait__size_type dt);
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void CC2420XDriverLayerP__RSSIFlag__set(message_t *msg);
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__FIFOP__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__FIFOP__get(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static void CC2420XDriverLayerP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420XDriverLayerP__PacketTimeStamp__size_type value);
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__RSTN__makeOutput(void );
#line 40
static void CC2420XDriverLayerP__RSTN__set(void );
static void CC2420XDriverLayerP__RSTN__clr(void );
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void CC2420XDriverLayerP__RadioAlarm__wait(tradio_size timeout);
#line 45
static bool CC2420XDriverLayerP__RadioAlarm__isFree(void );
#line 65
static tradio_size CC2420XDriverLayerP__RadioAlarm__getNow(void );
# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
static void CC2420XDriverLayerP__RadioState__done(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/LocalTime.nc"
static uint32_t CC2420XDriverLayerP__LocalTime__get(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420XDriverLayerP__FifopInterrupt__disable(void );
# 35 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
static uint8_t CC2420XDriverLayerP__Config__maxPayloadLength(void );
#line 29
static uint8_t CC2420XDriverLayerP__Config__headerLength(message_t *msg);
#line 41
static uint8_t CC2420XDriverLayerP__Config__metadataLength(message_t *msg);






static uint8_t CC2420XDriverLayerP__Config__headerPreloadLength(void );





static bool CC2420XDriverLayerP__Config__requiresRssiCca(message_t *msg);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t *CC2420XDriverLayerP__RadioReceive__receive(message_t *msg);
#line 46
static bool CC2420XDriverLayerP__RadioReceive__header(message_t *msg);
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__CSN__makeOutput(void );
#line 40
static void CC2420XDriverLayerP__CSN__set(void );
static void CC2420XDriverLayerP__CSN__clr(void );




static void CC2420XDriverLayerP__VREN__makeOutput(void );
#line 40
static void CC2420XDriverLayerP__VREN__set(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420XDriverLayerP__releaseSpi__postTask(void );
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420XDriverLayerP__SpiResource__release(void );
#line 97
static error_t CC2420XDriverLayerP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420XDriverLayerP__SpiResource__request(void );
#line 128
static bool CC2420XDriverLayerP__SpiResource__isOwner(void );
# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420XDriverLayerP__CCA__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__CCA__get(void );
static void CC2420XDriverLayerP__SFD__makeInput(void );
#line 43
static bool CC2420XDriverLayerP__SFD__get(void );
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void CC2420XDriverLayerP__RadioSend__ready(void );
#line 56
static void CC2420XDriverLayerP__RadioSend__sendDone(error_t error);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
static void CC2420XDriverLayerP__FastSpiByte__splitWrite(uint8_t data);
#line 74
static uint8_t CC2420XDriverLayerP__FastSpiByte__splitReadWrite(uint8_t data);
#line 68
static uint8_t CC2420XDriverLayerP__FastSpiByte__splitRead(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static error_t CC2420XDriverLayerP__SfdCapture__captureFallingEdge(void );
#line 66
static void CC2420XDriverLayerP__SfdCapture__disable(void );
#line 53
static error_t CC2420XDriverLayerP__SfdCapture__captureRisingEdge(void );
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static bool CC2420XDriverLayerP__TimeSyncFlag__get(message_t *msg);
#line 55
static void CC2420XDriverLayerP__TransmitPowerFlag__clear(message_t *msg);
#line 40
static bool CC2420XDriverLayerP__TransmitPowerFlag__get(message_t *msg);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void CC2420XDriverLayerP__Tasklet__schedule(void );
# 1230 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
enum CC2420XDriverLayerP____nesc_unnamed4322 {
#line 1230
  CC2420XDriverLayerP__releaseSpi = 4U
};
#line 1230
typedef int CC2420XDriverLayerP____nesc_sillytask_releaseSpi[CC2420XDriverLayerP__releaseSpi];
#line 102
static inline cc2420x_header_t *CC2420XDriverLayerP__getHeader(message_t *msg);




static inline void *CC2420XDriverLayerP__getPayload(message_t *msg);




static cc2420x_metadata_t *CC2420XDriverLayerP__getMeta(message_t *msg);






enum CC2420XDriverLayerP____nesc_unnamed4323 {

  CC2420XDriverLayerP__STATE_VR_ON = 0, 
  CC2420XDriverLayerP__STATE_PD = 1, 
  CC2420XDriverLayerP__STATE_PD_2_IDLE = 2, 
  CC2420XDriverLayerP__STATE_IDLE = 3, 
  CC2420XDriverLayerP__STATE_IDLE_2_RX_ON = 4, 
  CC2420XDriverLayerP__STATE_RX_ON = 5, 
  CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON = 6, 
  CC2420XDriverLayerP__STATE_IDLE_2_TX_ON = 7, 
  CC2420XDriverLayerP__STATE_TX_ON = 8, 
  CC2420XDriverLayerP__STATE_RX_DOWNLOAD = 9
};
uint8_t CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_VR_ON;

enum CC2420XDriverLayerP____nesc_unnamed4324 {

  CC2420XDriverLayerP__CMD_NONE = 0, 
  CC2420XDriverLayerP__CMD_TURNOFF = 1, 
  CC2420XDriverLayerP__CMD_STANDBY = 2, 
  CC2420XDriverLayerP__CMD_TURNON = 3, 
  CC2420XDriverLayerP__CMD_TRANSMIT = 4, 
  CC2420XDriverLayerP__CMD_RECEIVE = 5, 
  CC2420XDriverLayerP__CMD_CCA = 6, 
  CC2420XDriverLayerP__CMD_CHANNEL = 7, 
  CC2420XDriverLayerP__CMD_SIGNAL_DONE = 8, 
  CC2420XDriverLayerP__CMD_DOWNLOAD = 9
};
uint8_t CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
#line 162
bool CC2420XDriverLayerP__rxSfd = 0;
bool CC2420XDriverLayerP__txEnd = 0;

uint8_t CC2420XDriverLayerP__txPower;
uint8_t CC2420XDriverLayerP__channel;

message_t *CC2420XDriverLayerP__rxMsg;



message_t CC2420XDriverLayerP__rxMsgBuffer;

uint16_t CC2420XDriverLayerP__capturedTime;


static __inline void CC2420XDriverLayerP__enableReceiveSfd(void );




static __inline cc2420X_status_t CC2420XDriverLayerP__strobe(uint8_t reg);

uint8_t CC2420XDriverLayerP__sfd_status;
#line 202
static inline void CC2420XDriverLayerP__RadioAlarm__fired(void );
#line 244
static __inline cc2420X_status_t CC2420XDriverLayerP__strobe(uint8_t reg);
#line 266
static __inline cc2420X_status_t CC2420XDriverLayerP__writeRegister(uint8_t reg, uint16_t value);
#line 285
static __inline cc2420X_status_t CC2420XDriverLayerP__writeTxFifo(uint8_t *data, uint8_t length);
#line 311
static __inline uint8_t CC2420XDriverLayerP__waitForRxFifo(void );
#line 327
static __inline cc2420X_status_t CC2420XDriverLayerP__readLengthFromRxFifo(uint8_t *lengthPtr);
#line 367
static __inline void CC2420XDriverLayerP__readPayloadFromRxFifo(uint8_t *data, uint8_t length);
#line 382
static __inline void CC2420XDriverLayerP__readRssiFromRxFifo(uint8_t *rssiPtr);









static __inline void CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(uint8_t *crcOkAndLqiPtr);










static __inline cc2420X_status_t CC2420XDriverLayerP__flushRxFifo(void );
#line 424
static inline error_t CC2420XDriverLayerP__SoftwareInit__init(void );
#line 476
static __inline void CC2420XDriverLayerP__resetRadio(void );
#line 501
static inline void CC2420XDriverLayerP__initRadio(void );
#line 514
static inline void CC2420XDriverLayerP__SpiResource__granted(void );
#line 531
static bool CC2420XDriverLayerP__isSpiAcquired(void );
#line 555
static __inline void CC2420XDriverLayerP__setChannel(void );
#line 594
static __inline void CC2420XDriverLayerP__changeChannel(void );
#line 614
static __inline void CC2420XDriverLayerP__changeState(void );
#line 707
static inline error_t CC2420XDriverLayerP__RadioState__turnOn(void );
#line 733
static error_t CC2420XDriverLayerP__RadioSend__send(message_t *msg);
#line 932
static __inline void CC2420XDriverLayerP__enableReceiveSfd(void );
#line 1003
static __inline void CC2420XDriverLayerP__downloadMessage(void );
#line 1166
static inline void CC2420XDriverLayerP__SfdCapture__captured(uint16_t time);
#line 1212
static inline void CC2420XDriverLayerP__FifopInterrupt__fired(void );
#line 1230
static inline void CC2420XDriverLayerP__releaseSpi__runTask(void );






static inline void CC2420XDriverLayerP__Tasklet__run(void );
#line 1382
static inline uint8_t CC2420XDriverLayerP__RadioPacket__headerLength(message_t *msg);




static uint8_t CC2420XDriverLayerP__RadioPacket__payloadLength(message_t *msg);




static void CC2420XDriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length);








static inline uint8_t CC2420XDriverLayerP__RadioPacket__maxPayloadLength(void );






static inline uint8_t CC2420XDriverLayerP__RadioPacket__metadataLength(message_t *msg);
#line 1420
static inline bool CC2420XDriverLayerP__PacketTransmitPower__isSet(message_t *msg);




static inline uint8_t CC2420XDriverLayerP__PacketTransmitPower__get(message_t *msg);
#line 1458
static inline void CC2420XDriverLayerP__PacketRSSI__set(message_t *msg, uint8_t value);
#line 1494
static inline bool CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg);




static inline uint8_t CC2420XDriverLayerP__PacketTimeSyncOffset__get(message_t *msg);
#line 1533
static inline void CC2420XDriverLayerP__PacketLinkQuality__set(message_t *msg, uint8_t value);
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void );
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/BusyWaitCounterC.nc"
enum /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0____nesc_unnamed4325 {

  BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE = (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type )1 << (8 * sizeof(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type ) - 1)
};

static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt);
#line 83
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
#line 64
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void TaskletC__Tasklet__run(void );
# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
uint8_t TaskletC__state;

static inline void TaskletC__doit(void );
#line 112
static void TaskletC__Tasklet__schedule(void );
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfd838, 
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfcab8);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );









static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isTxIntrPending(void );
#line 224
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void );
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bfc068);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40bdac30);
#line 102
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4326 {
#line 102
  Msp430SpiNoDmaP__0__signalDone_task = 5U
};
#line 102
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 91
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4327 {
  Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos;
uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client;

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );


static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id);



static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id);
#line 146
static __inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(uint8_t data);



static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead(void );




static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(uint8_t data);
#line 172
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 227
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 244
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0012)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0010)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 316
static inline bool HplMsp430Usart0P__Usart__isTxIntrPending(void );
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void );



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108, 
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf988);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4328 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce1690);
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce1690);
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4329 {
#line 75
  ArbiterP__0__grantedTask = 6U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4330 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4331 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4332 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 93
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 111
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static inline bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
static inline void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
static inline void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput(void );
# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );
static inline void /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );





static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 92
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 61
static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__disable(void );
# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__fired(void );
# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 72
static void HplMsp430InterruptP__Port26__fired(void );
#line 72
static void HplMsp430InterruptP__Port17__fired(void );
#line 72
static void HplMsp430InterruptP__Port21__fired(void );
#line 72
static void HplMsp430InterruptP__Port12__fired(void );
#line 72
static void HplMsp430InterruptP__Port24__fired(void );
#line 72
static void HplMsp430InterruptP__Port15__fired(void );
#line 72
static void HplMsp430InterruptP__Port27__fired(void );
#line 72
static void HplMsp430InterruptP__Port10__fired(void );
#line 72
static void HplMsp430InterruptP__Port22__fired(void );
#line 72
static void HplMsp430InterruptP__Port13__fired(void );
#line 72
static void HplMsp430InterruptP__Port25__fired(void );
#line 72
static void HplMsp430InterruptP__Port16__fired(void );
#line 72
static void HplMsp430InterruptP__Port20__fired(void );
#line 72
static void HplMsp430InterruptP__Port11__fired(void );
#line 72
static void HplMsp430InterruptP__Port23__fired(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0008)))  ;
#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );
static inline void HplMsp430InterruptP__Port14__default__fired(void );
static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );








static inline void HplMsp430InterruptP__Port10__disable(void );







static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );
#line 169
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0002)))  ;
#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 206
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );






static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(uint16_t dt);




static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 104
static inline uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(void );









static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static Msp430HybridAlarmCounterP__CounterMicro__size_type Msp430HybridAlarmCounterP__CounterMicro__get(void );
#line 82
static void Msp430HybridAlarmCounterP__Counter2ghz__overflow(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void Msp430HybridAlarmCounterP__Alarm2ghz__fired(void );
#line 103
static void Msp430HybridAlarmCounterP__AlarmMicro__startAt(Msp430HybridAlarmCounterP__AlarmMicro__size_type t0, Msp430HybridAlarmCounterP__AlarmMicro__size_type dt);
#line 88
static bool Msp430HybridAlarmCounterP__AlarmMicro__isRunning(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static Msp430HybridAlarmCounterP__Counter32khz__size_type Msp430HybridAlarmCounterP__Counter32khz__get(void );






static bool Msp430HybridAlarmCounterP__Counter32khz__isOverflowPending(void );
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
uint32_t Msp430HybridAlarmCounterP__fireTime;


static __inline uint16_t Msp430HybridAlarmCounterP__now32khz(void );



static __inline uint16_t Msp430HybridAlarmCounterP__nowMicro(void );




static __inline void Msp430HybridAlarmCounterP__now(uint16_t *t32khz, uint16_t *tMicro, uint32_t *t2ghz);
#line 86
static __inline uint32_t Msp430HybridAlarmCounterP__now2ghz(void );
#line 101
static inline uint32_t Msp430HybridAlarmCounterP__Counter2ghz__get(void );








static inline bool Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending(void );
#line 124
static inline void Msp430HybridAlarmCounterP__Counter32khz__overflow(void );



static inline void Msp430HybridAlarmCounterP__CounterMicro__overflow(void );
#line 163
static inline void Msp430HybridAlarmCounterP__Alarm32khz__fired(void );
#line 176
static inline void Msp430HybridAlarmCounterP__AlarmMicro__fired(void );
#line 191
static inline void Msp430HybridAlarmCounterP__Alarm2ghz__default__fired(void );
#line 260
static inline mcu_power_t Msp430HybridAlarmCounterP__McuPowerOverride__lowestState(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void );
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void );
# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void );
#line 114
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void );
# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time);

static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void );
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void );

static bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__areEventsEnabled(void );
#line 58
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void );
#line 44
static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void );
# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void );





static inline bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning(void );




static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get(void );






static bool /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__overflow(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper;

enum /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1____nesc_unnamed4333 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 11, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type ) - /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type ) + 11, 



  TransformCounterC__1__OVERFLOW_MASK = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type )2 << (/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get(void );
#line 133
static inline void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow(void );
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void );
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );




static inline void /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t UartLogP__uartSendTask__postTask(void );
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t UartLogP__UartSend__send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t UartLogP__UartControl__start(void );
# 44 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
enum UartLogP____nesc_unnamed4334 {
#line 44
  UartLogP__uartSendTask = 7U
};
#line 44
typedef int UartLogP____nesc_sillytask_uartSendTask[UartLogP__uartSendTask];
#line 25
queue_entry_t UartLogP__uartQueueBufs[UART_QUEUE_LEN];
queue_entry_t *UartLogP__uartQueue[UART_QUEUE_LEN];


uint8_t UartLogP__uartIn;
#line 29
uint8_t UartLogP__uartOut;
bool UartLogP__uartBusy;
#line 30
bool UartLogP__uartFull;




uint8_t UartLogP__offset;

uint32_t UartLogP__seqno;








static inline void UartLogP__dropBlink(void );



static inline void UartLogP__failBlink(void );




uint8_t UartLogP__init_cnt = 0;
static inline error_t UartLogP__Init__init(void );









static inline void UartLogP__UartControl__startDone(error_t err);
#line 85
static inline void UartLogP__UartControl__stopDone(error_t err);


static __inline nx_uint8_t *UartLogP__getPayload(message_t *msg);



static inline error_t UartLogP__UartLog__logByteStream(nx_uint8_t *log_p, uint8_t len);
#line 149
static inline void UartLogP__uartSendTask__runTask(void );
#line 173
static void UartLogP__UartSend__sendDone(message_t *msg, error_t error);
#line 220
static error_t UartLogP__UartLog__logEntry(__nesc_nxbase_nx_uint8_t type, __nesc_nxbase_nx_uint8_t sourceId, __nesc_nxbase_nx_uint16_t seq, __nesc_nxbase_nx_uint32_t timestamp);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(
# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40f01068, 
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40f01a28, 
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg);







static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 101
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void );
#line 148
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg);









static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr);
#line 172
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type);
# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void SerialP__SplitControl__startDone(error_t error);
#line 138
static void SerialP__SplitControl__stopDone(error_t error);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP__stopDoneTask__postTask(void );
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/StdControl.nc"
static error_t SerialP__SerialControl__start(void );









static error_t SerialP__SerialControl__stop(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP__RunTx__postTask(void );
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFlush.nc"
static void SerialP__SerialFlush__flush(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP__startDoneTask__postTask(void );
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP__SerialFrameComm__putDelimiter(void );
#line 79
static void SerialP__SerialFrameComm__resetReceive(void );
#line 65
static error_t SerialP__SerialFrameComm__putData(uint8_t data);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP__defaultSerialFlushTask__postTask(void );
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP__SendBytePacket__nextByte(void );









static void SerialP__SendBytePacket__sendCompleted(error_t error);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP__ReceiveBytePacket__startPacket(void );






static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data);










static void SerialP__ReceiveBytePacket__endPacket(error_t result);
# 191 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
enum SerialP____nesc_unnamed4335 {
#line 191
  SerialP__RunTx = 8U
};
#line 191
typedef int SerialP____nesc_sillytask_RunTx[SerialP__RunTx];
#line 322
enum SerialP____nesc_unnamed4336 {
#line 322
  SerialP__startDoneTask = 9U
};
#line 322
typedef int SerialP____nesc_sillytask_startDoneTask[SerialP__startDoneTask];









enum SerialP____nesc_unnamed4337 {
#line 332
  SerialP__stopDoneTask = 10U
};
#line 332
typedef int SerialP____nesc_sillytask_stopDoneTask[SerialP__stopDoneTask];








enum SerialP____nesc_unnamed4338 {
#line 341
  SerialP__defaultSerialFlushTask = 11U
};
#line 341
typedef int SerialP____nesc_sillytask_defaultSerialFlushTask[SerialP__defaultSerialFlushTask];
#line 81
enum SerialP____nesc_unnamed4339 {
  SerialP__RX_DATA_BUFFER_SIZE = 2, 
  SerialP__TX_DATA_BUFFER_SIZE = 4, 
  SerialP__SERIAL_MTU = 255, 
  SerialP__SERIAL_VERSION = 1, 
  SerialP__ACK_QUEUE_SIZE = 5
};

enum SerialP____nesc_unnamed4340 {
  SerialP__RXSTATE_NOSYNC, 
  SerialP__RXSTATE_PROTO, 
  SerialP__RXSTATE_TOKEN, 
  SerialP__RXSTATE_INFO, 
  SerialP__RXSTATE_INACTIVE
};

enum SerialP____nesc_unnamed4341 {
  SerialP__TXSTATE_IDLE, 
  SerialP__TXSTATE_PROTO, 
  SerialP__TXSTATE_SEQNO, 
  SerialP__TXSTATE_INFO, 
  SerialP__TXSTATE_FCS1, 
  SerialP__TXSTATE_FCS2, 
  SerialP__TXSTATE_ENDFLAG, 
  SerialP__TXSTATE_ENDWAIT, 
  SerialP__TXSTATE_FINISH, 
  SerialP__TXSTATE_ERROR, 
  SerialP__TXSTATE_INACTIVE
};





#line 111
typedef enum SerialP____nesc_unnamed4342 {
  SerialP__BUFFER_AVAILABLE, 
  SerialP__BUFFER_FILLING, 
  SerialP__BUFFER_COMPLETE
} SerialP__tx_data_buffer_states_t;

enum SerialP____nesc_unnamed4343 {
  SerialP__TX_ACK_INDEX = 0, 
  SerialP__TX_DATA_INDEX = 1, 
  SerialP__TX_BUFFER_COUNT = 2
};






#line 124
typedef struct SerialP____nesc_unnamed4344 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__RX_DATA_BUFFER_SIZE + 1];
} SerialP__rx_buf_t;




#line 130
typedef struct SerialP____nesc_unnamed4345 {
  uint8_t state;
  uint8_t buf;
} SerialP__tx_buf_t;





#line 135
typedef struct SerialP____nesc_unnamed4346 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP__ACK_QUEUE_SIZE + 1];
} SerialP__ack_queue_t;



SerialP__rx_buf_t SerialP__rxBuf;
SerialP__tx_buf_t SerialP__txBuf[SerialP__TX_BUFFER_COUNT];



uint8_t SerialP__rxState;
uint8_t SerialP__rxByteCnt;
uint8_t SerialP__rxProto;
uint8_t SerialP__rxSeqno;
uint16_t SerialP__rxCRC;



uint8_t SerialP__txState;
uint8_t SerialP__txByteCnt;
uint8_t SerialP__txProto;
uint8_t SerialP__txSeqno;
uint16_t SerialP__txCRC;
uint8_t SerialP__txPending;
uint8_t SerialP__txIndex;


SerialP__ack_queue_t SerialP__ackQ;

bool SerialP__offPending = FALSE;



static __inline void SerialP__txInit(void );
static __inline void SerialP__rxInit(void );
static __inline void SerialP__ackInit(void );

static __inline bool SerialP__ack_queue_is_full(void );
static __inline bool SerialP__ack_queue_is_empty(void );
static __inline void SerialP__ack_queue_push(uint8_t token);
static __inline uint8_t SerialP__ack_queue_top(void );
static inline uint8_t SerialP__ack_queue_pop(void );




static __inline void SerialP__rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP__rx_buffer_top(void );
static __inline uint8_t SerialP__rx_buffer_pop(void );
static __inline uint16_t SerialP__rx_current_crc(void );

static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP__MaybeScheduleTx(void );




static __inline void SerialP__txInit(void );
#line 207
static __inline void SerialP__rxInit(void );








static __inline void SerialP__ackInit(void );



static inline error_t SerialP__Init__init(void );
#line 234
static __inline bool SerialP__ack_queue_is_full(void );









static __inline bool SerialP__ack_queue_is_empty(void );





static __inline void SerialP__ack_queue_push(uint8_t token);









static __inline uint8_t SerialP__ack_queue_top(void );









static inline uint8_t SerialP__ack_queue_pop(void );
#line 297
static __inline void SerialP__rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP__rx_buffer_top(void );



static __inline uint8_t SerialP__rx_buffer_pop(void );





static __inline uint16_t SerialP__rx_current_crc(void );










static inline void SerialP__startDoneTask__runTask(void );









static inline void SerialP__stopDoneTask__runTask(void );



static inline void SerialP__SerialFlush__flushDone(void );




static inline void SerialP__defaultSerialFlushTask__runTask(void );


static inline void SerialP__SerialFlush__default__flush(void );



static error_t SerialP__SplitControl__start(void );








static void SerialP__testOff(void );
#line 394
static inline void SerialP__SerialFrameComm__delimiterReceived(void );


static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data);



static inline bool SerialP__valid_rx_proto(uint8_t proto);










static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data);
#line 518
static void SerialP__MaybeScheduleTx(void );










static inline error_t SerialP__SendBytePacket__completeSend(void );








static inline error_t SerialP__SendBytePacket__startSend(uint8_t b);
#line 559
static inline void SerialP__RunTx__runTask(void );
#line 668
static inline void SerialP__SerialFrameComm__putDone(void );
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void );
# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc2760, 
# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc2120, 
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc12a0, 
# 31 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc12a0);
# 23 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40fc12a0, 
# 23 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void );
#line 62
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte);
# 158 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4347 {
#line 158
  SerialDispatcherP__0__signalSendDone = 12U
};
#line 158
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone];
#line 275
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4348 {
#line 275
  SerialDispatcherP__0__receiveTask = 13U
};
#line 275
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask];
#line 66
#line 62
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4349 {
  SerialDispatcherP__0__SEND_STATE_IDLE = 0, 
  SerialDispatcherP__0__SEND_STATE_BEGIN = 1, 
  SerialDispatcherP__0__SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4350 {
  SerialDispatcherP__0__RECV_STATE_IDLE = 0, 
  SerialDispatcherP__0__RECV_STATE_BEGIN = 1, 
  SerialDispatcherP__0__RECV_STATE_DATA = 2
};






#line 74
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0____nesc_unnamed4351 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len);
#line 158
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void );
#line 178
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void );
#line 194
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void );
#line 244
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b);
#line 275
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void );
#line 296
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result);
#line 358
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC__UartStream__send(
#line 44
uint8_t * buf, 



uint16_t len);
# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data);





static void HdlcTranslateC__SerialFrameComm__putDone(void );
#line 85
static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void );
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/HdlcTranslateC.nc"
#line 56
typedef struct HdlcTranslateC____nesc_unnamed4352 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC__HdlcState;


HdlcTranslateC__HdlcState HdlcTranslateC__state = { 0, 0 };
uint8_t HdlcTranslateC__txTemp;
uint8_t HdlcTranslateC__m_data;


static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void );





static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data);
#line 98
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void );







static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data);
#line 118
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error);
#line 132
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000a88);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset);
#line 179
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void );


static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void );
#line 224
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data);
#line 128
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void );
#line 174
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config);
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41002c58, 
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
uint8_t * buf, 



uint16_t len, error_t error);
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41000030);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x41005aa0);
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len;
#line 59
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len;
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;
#line 60
uint8_t * /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos;
#line 61
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner;

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id);
#line 77
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id);







static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id);






static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id);








static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id);
#line 134
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data);
#line 147
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len);
#line 162
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id);
#line 208
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void );

static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id);

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id);
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte);
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error);
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__UCLK__selectIOFunc(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P__Interrupts__txDone(void );
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P__URXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__URXD__selectModuleFunc(void );






static void HplMsp430Usart1P__UTXD__selectIOFunc(void );
#line 92
static void HplMsp430Usart1P__UTXD__selectModuleFunc(void );






static void HplMsp430Usart1P__SOMI__selectIOFunc(void );
#line 99
static void HplMsp430Usart1P__SIMO__selectIOFunc(void );
# 87 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P__ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P__U1TCTL __asm ("0x0079");
static volatile uint8_t HplMsp430Usart1P__U1RCTL __asm ("0x007A");
static volatile uint8_t HplMsp430Usart1P__U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0006)))  ;




void sig_UART1TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0004)))  ;



static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void );



static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void );
#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset);
#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void );







static void HplMsp430Usart1P__Usart__disableUart(void );








static inline void HplMsp430Usart1P__Usart__enableUartTx(void );




static inline void HplMsp430Usart1P__Usart__disableUartTx(void );





static inline void HplMsp430Usart1P__Usart__enableUartRx(void );




static inline void HplMsp430Usart1P__Usart__disableUartRx(void );
#line 251
static void HplMsp430Usart1P__Usart__disableSpi(void );
#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config);









static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config);
#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void );
#line 359
static inline void HplMsp430Usart1P__Usart__disableIntr(void );
#line 377
static inline void HplMsp430Usart1P__Usart__enableIntr(void );






static inline void HplMsp430Usart1P__Usart__tx(uint8_t data);
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108, 
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(
# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40caf108);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4353 {
#line 49
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce1690);
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40cdfa58);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void );
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ce2bd0);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4354 {
#line 75
  ArbiterP__1__grantedTask = 14U
};
#line 75
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4355 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4356 {
#line 68
  ArbiterP__1__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1____nesc_unnamed4357 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
#line 93
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 133
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void );










static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id);
#line 190
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void );
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void );
# 74 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t TelosSerialP__Resource__release(void );
#line 97
static error_t TelosSerialP__Resource__immediateRequest(void );
# 36 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/TelosSerialP.nc"
enum TelosSerialP____nesc_unnamed4358 {

  TelosSerialP__UBR_4MHZ_4800 = 0x0369, TelosSerialP__UMCTL_4MHZ_4800 = 0xfb, 
  TelosSerialP__UBR_4MHZ_9600 = 0x01b4, TelosSerialP__UMCTL_4MHZ_9600 = 0xdf, 
  TelosSerialP__UBR_4MHZ_57600 = 0x0048, TelosSerialP__UMCTL_4MHZ_57600 = 0xfb, 
  TelosSerialP__UBR_4MHZ_115200 = 0x0024, TelosSerialP__UMCTL_4MHZ_115200 = 0x4a, 

  TelosSerialP__UBR_3_7MHZ_115200 = 0x0020, TelosSerialP__UMCTL_3_7MHZ_115200 = 0x00
};


msp430_uart_union_config_t TelosSerialP__msp430_uart_telos_config = { { .ubr = TelosSerialP__UBR_4MHZ_115200, .umctl = TelosSerialP__UBR_4MHZ_115200, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP__StdControl__start(void );


static inline error_t TelosSerialP__StdControl__stop(void );



static inline void TelosSerialP__Resource__granted(void );

static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void );
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen);
# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 162
static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 158
message_t * amsg, 



am_id_t t);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline error_t /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
am_id_t arg_0x41120490, 
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x411219d8, 
# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 74
message_t * msg);
#line 94
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 74
message_t * amsg);
#line 147
static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 143
message_t * amsg);
# 126 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4359 {
#line 126
  AMQueueImplP__0__CancelTask = 15U
};
#line 126
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 169
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4360 {
#line 169
  AMQueueImplP__0__errorTask = 16U
};
#line 169
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 57
#line 55
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4361 {
  message_t * msg;
} /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 90
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 163
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 189
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 65 "ActiveMessageAddressC.nc"
am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 93
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 106
static inline am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress(void );
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length);
#line 43
static uint8_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(message_t *msg);
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
enum /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0____nesc_unnamed4362 {

  Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_MASK = (((IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE)
   | (1 << IEEE154_FCF_INTRAPAN))
   | (IEEE154_ADDR_MASK << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_MASK << IEEE154_FCF_SRC_ADDR_MODE), 

  Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_VALUE = (((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE)
   | (1 << IEEE154_FCF_INTRAPAN))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE))
   | (IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE), 

  Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_PRESERVE = (1 << IEEE154_FCF_ACK_REQ)
   | (1 << IEEE154_FCF_FRAME_PENDING), 

  Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_LENGTH = 3, 
  Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_MASK = IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE, 
  Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE = IEEE154_TYPE_ACK << IEEE154_FCF_FRAME_TYPE
};

static inline ieee154_simple_header_t */*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(message_t *msg);
#line 92
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(message_t *msg);




static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(message_t *msg);






static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(message_t *msg);










static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack);








static inline bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack);







static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(message_t *msg);




static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack);
#line 163
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn);









static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan);




static uint16_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(message_t *msg);




static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr);









static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr);




static inline bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(message_t *msg);






static inline bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(message_t *msg);
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2____nesc_unnamed4363 {
#line 49
  FcfsResourceQueueC__2__NO_ENTRY = 0xFF
};
uint8_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__resQ[1U];
uint8_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qHead = /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
uint8_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qTail = /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

static inline error_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__Init__init(void );








static inline bool /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);
#line 82
static inline error_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b2e18);
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b2e18);
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b1a58);
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id);
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x411b23f0);
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void );
# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
enum /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4364 {
#line 69
  SimpleArbiterP__0__grantedTask = 17U
};
#line 69
typedef int /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_sillytask_grantedTask[/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask];
#line 62
enum /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4365 {
#line 62
  SimpleArbiterP__0__RES_IDLE = 0, SimpleArbiterP__0__RES_GRANTING = 1, SimpleArbiterP__0__RES_BUSY = 2
};
#line 63
enum /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0____nesc_unnamed4366 {
#line 63
  SimpleArbiterP__0__NO_RES = 0xFF
};
uint8_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE;
uint8_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__NO_RES;
uint8_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId;



static inline error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id);
#line 84
static inline error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 155
static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );









static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id);
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__fired(
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t arg_0x411cbe38);
# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__getNow(void );
#line 66
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__start(/*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type dt);






static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__stop(void );
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__schedule(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
uint8_t /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state;
enum /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0____nesc_unnamed4367 {

  RadioAlarmP__0__STATE_READY = 0, 
  RadioAlarmP__0__STATE_WAIT = 1, 
  RadioAlarmP__0__STATE_FIRED = 2
};

uint8_t /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__alarm;

static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired(void );










static __inline tradio_size /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(uint8_t id);




static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run(void );








static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(uint8_t id);



static __inline bool /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(uint8_t id);




static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(uint8_t id, tradio_size timeout);








static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(uint8_t id);
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__send(message_t *msg);
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__wait(tradio_size timeout);




static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__cancel(void );
#line 45
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__isFree(void );
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__clear(message_t *msg);
#line 40
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__get(message_t *msg);









static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__set(message_t *msg);
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__reportChannelError(void );
#line 80
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__createAckPacket(message_t *data, message_t *ack);
#line 55
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__requiresAckWait(message_t *msg);






static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__isAckPacket(message_t *msg);






static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__verifyAckPacket(message_t *data, message_t *ack);
#line 43
static uint16_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__getAckTimeout(void );





static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__setAckRequired(message_t *msg, bool ack);
#line 75
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__requiresAckReply(message_t *msg);
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
static message_t */*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioReceive__receive(message_t *msg);
#line 46
static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioReceive__header(message_t *msg);
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__ready(void );
#line 56
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__sendDone(error_t error);
# 32 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
uint8_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state;
enum /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0____nesc_unnamed4368 {

  SoftwareAckLayerC__0__STATE_READY = 0, 
  SoftwareAckLayerC__0__STATE_DATA_SEND = 1, 
  SoftwareAckLayerC__0__STATE_ACK_WAIT = 2, 
  SoftwareAckLayerC__0__STATE_ACK_SEND = 3
};

message_t */*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__txMsg;
message_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__ackMsg;

static inline void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__ready(void );





static inline error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__send(message_t *msg);
#line 69
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__sendDone(error_t error);
#line 96
static inline void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__fired(void );









static inline bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__header(message_t *msg);
#line 119
static inline message_t */*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__receive(message_t *msg);
#line 153
static inline error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__requestAck(message_t *msg);
#line 167
static inline bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__wasAcked(message_t *msg);
# 397 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 137 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4068d788){
#line 39
  switch (arg_0x4068d788) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4068d788);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__overflow(void ){
#line 82
  /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 82
}
#line 82
# 133 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static inline void /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper++;
    if ((/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper & /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void Msp430HybridAlarmCounterP__Counter2ghz__overflow(void ){
#line 82
  /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__overflow();
#line 82
}
#line 82
# 124 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__Counter32khz__overflow(void )
#line 124
{
  Msp430HybridAlarmCounterP__Counter2ghz__overflow();
}

# 208 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow(void )
#line 208
{
}

# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Counter__overflow();
#line 82
  Msp430HybridAlarmCounterP__Counter32khz__overflow();
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 114 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow(void )
{
}

# 196 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 58
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4369 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents(void )
{
  * (volatile uint16_t * )354U &= ~0x0010;
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )352U & 1U;
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents(void )
{
  * (volatile uint16_t * )354U |= 0x0010;
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )354U &= ~0x0001;
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )370U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__get() + x;
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4370 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 130
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 57
}
#line 57
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 41
}
#line 41
# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (0) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
        {
          uint16_t t0;
          uint16_t t1 = * (volatile uint16_t * )400U;

#line 72
          do {
#line 72
              t0 = t1;
#line 72
              t1 = * (volatile uint16_t * )400U;
            }
          while (
#line 72
          t0 != t1);
          {
            unsigned int __nesc_temp = 
#line 73
            t1;

            {
#line 73
              __nesc_atomic_end(__nesc_atomic); 
#line 73
              return __nesc_temp;
            }
          }
        }
#line 76
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__clearPendingInterrupt();
    /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__enableEvents();
  }
}

# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void Msp430HybridAlarmCounterP__AlarmMicro__startAt(Msp430HybridAlarmCounterP__AlarmMicro__size_type t0, Msp430HybridAlarmCounterP__AlarmMicro__size_type dt){
#line 103
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static Msp430HybridAlarmCounterP__CounterMicro__size_type Msp430HybridAlarmCounterP__CounterMicro__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static __inline uint16_t Msp430HybridAlarmCounterP__nowMicro(void )
#line 57
{
  return Msp430HybridAlarmCounterP__CounterMicro__get();
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static Msp430HybridAlarmCounterP__Counter32khz__size_type Msp430HybridAlarmCounterP__Counter32khz__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static __inline uint16_t Msp430HybridAlarmCounterP__now32khz(void )
#line 53
{
  return Msp430HybridAlarmCounterP__Counter32khz__get();
}






static __inline void Msp430HybridAlarmCounterP__now(uint16_t *t32khz, uint16_t *tMicro, uint32_t *t2ghz)
#line 62
{
  uint16_t eMicro;

  /* atomic removed: atomic calls only */
#line 65
  {

    *t32khz = Msp430HybridAlarmCounterP__now32khz();


    *tMicro = Msp430HybridAlarmCounterP__nowMicro();


    while (*t32khz == Msp430HybridAlarmCounterP__now32khz()) ;
  }


  eMicro = Msp430HybridAlarmCounterP__nowMicro() - *tMicro;


  *t2ghz = (uint32_t )*t32khz << 16;


  *t2ghz -= (uint32_t )eMicro << 11;
}

#line 163
static inline void Msp430HybridAlarmCounterP__Alarm32khz__fired(void )
#line 163
{
  uint16_t tMicro;
#line 164
  uint16_t t32khz;
  uint32_t t2ghz;
#line 165
  uint32_t dt;


  Msp430HybridAlarmCounterP__now(&t32khz, &tMicro, &t2ghz);


  dt = Msp430HybridAlarmCounterP__fireTime - t2ghz;

  Msp430HybridAlarmCounterP__AlarmMicro__startAt(tMicro, dt >> 11);
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired(void ){
#line 78
  Msp430HybridAlarmCounterP__Alarm32khz__fired();
#line 78
}
#line 78
# 135 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents(void )
{
  * (volatile uint16_t * )356U &= ~0x0010;
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired(void )
{
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430TimerControl__disableEvents();
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Alarm__fired();
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Compare__fired();
#line 45
}
#line 45
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4371 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void )
{
}

# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__CounterMicro__overflow(void )
#line 128
{
}

# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 82
  Msp430HybridAlarmCounterP__CounterMicro__overflow();
#line 82
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow();
#line 82
}
#line 82
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 114 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 48
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Timer__overflow();
#line 48
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__schedule(void ){
#line 59
  TaskletC__Tasklet__schedule();
#line 59
}
#line 59
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state == /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_WAIT) {
      /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_FIRED;
      }
  }
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__schedule();
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 78
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__fired();
#line 78
}
#line 78
# 135 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4372 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420XDriverLayerP__releaseSpi__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420XDriverLayerP__releaseSpi);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 171 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__SubSend__ready(void )
#line 171
{
}

# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__ready(void ){
#line 63
  FastCC2420TransceiverP__SubSend__ready();
#line 63
}
#line 63
# 44 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__ready(void )
{
  if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state == /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY) {
    /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__ready();
    }
}

# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void CC2420XDriverLayerP__RadioSend__ready(void ){
#line 63
  /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__ready();
#line 63
}
#line 63
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t FastCC2420TransceiverP__stopTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(FastCC2420TransceiverP__stopTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
inline static error_t FastCC2420TransceiverP__startTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(FastCC2420TransceiverP__startTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 114 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__RadioState__done(void )
#line 114
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      if (FastCC2420TransceiverP__is_on) {
          FastCC2420TransceiverP__startTask__postTask();
        }
      else 
#line 118
        {
          FastCC2420TransceiverP__stopTask__postTask();
        }
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
inline static void CC2420XDriverLayerP__RadioState__done(void ){
#line 69
  FastCC2420TransceiverP__RadioState__done();
#line 69
}
#line 69
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 48
}
#line 48
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CSN__set(void ){
#line 40
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 40
}
#line 40
# 386 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  return U0RXBUF;
}

# 231 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx(void ){
#line 231
  unsigned char __nesc_result;
#line 231

#line 231
  __nesc_result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return __nesc_result;
#line 231
}
#line 231
# 330 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & 0x40) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char __nesc_result;
#line 192

#line 192
  __nesc_result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return __nesc_result;
#line 192
}
#line 192
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead(void )
#line 150
{
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
}

# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
inline static uint8_t CC2420XDriverLayerP__FastSpiByte__splitRead(void ){
#line 68
  unsigned char __nesc_result;
#line 68

#line 68
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitRead();
#line 68

#line 68
  return __nesc_result;
#line 68
}
#line 68
# 382 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  HplMsp430Usart0P__U0TXBUF = data;
}

# 224 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 146 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static __inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(uint8_t data)
#line 146
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(data);
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
inline static void CC2420XDriverLayerP__FastSpiByte__splitWrite(uint8_t data){
#line 62
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitWrite(data);
#line 62
}
#line 62
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 53
}
#line 53
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CSN__clr(void ){
#line 41
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 244 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__strobe(uint8_t reg)
{
  cc2420X_status_t status;

  for (; 0; ) ;
  for (; 0; ) ;

  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();

  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_WRITE | reg);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();

  CC2420XDriverLayerP__CSN__set();
  return status;
}

# 316 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isTxIntrPending(void )
#line 316
{
  if (HplMsp430Usart0P__IFG1 & 0x80) {
      return TRUE;
    }
  return FALSE;
}

# 187 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isTxIntrPending(void ){
#line 187
  unsigned char __nesc_result;
#line 187

#line 187
  __nesc_result = HplMsp430Usart0P__Usart__isTxIntrPending();
#line 187

#line 187
  return __nesc_result;
#line 187
}
#line 187
# 155 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static __inline uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(uint8_t data)
#line 155
{
  uint8_t b;

  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  b = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isTxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(data);

  return b;
}

# 74 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/FastSpiByte.nc"
inline static uint8_t CC2420XDriverLayerP__FastSpiByte__splitReadWrite(uint8_t data){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__FastSpiByte__splitReadWrite(data);
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 266 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__writeRegister(uint8_t reg, uint16_t value)
{
  cc2420X_status_t status;

  for (; 0; ) ;
  for (; 0; ) ;

  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();

  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_WRITE | reg);
  CC2420XDriverLayerP__FastSpiByte__splitReadWrite(value >> 8);
  CC2420XDriverLayerP__FastSpiByte__splitReadWrite(value & 0xff);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();

  CC2420XDriverLayerP__CSN__set();
  return status;
}

#line 555
static __inline void CC2420XDriverLayerP__setChannel(void )
{
  cc2420X_fsctrl_t fsctrl;

  fsctrl = cc2420X_fsctrl_default;
  fsctrl.f.freq = 357 + 5 * (CC2420XDriverLayerP__channel - 11);

  CC2420XDriverLayerP__strobe(CC2420_SRFOFF);
  CC2420XDriverLayerP__writeRegister(CC2420X_FSCTRL, fsctrl.value);
  CC2420XDriverLayerP__strobe(CC2420_SRXON);
}

#line 594
static __inline void CC2420XDriverLayerP__changeChannel(void )
{
  for (; 0; ) ;
  for (; 0; ) ;

  if (CC2420XDriverLayerP__isSpiAcquired()) 
    {
      CC2420XDriverLayerP__setChannel();

      if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON) {
          CC2420XDriverLayerP__RadioAlarm__wait(IDLE_2_RX_ON_TIME);
          CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE_2_RX_ON;
        }
      else {
        CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
        }
    }
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 48
}
#line 48
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__RSTN__set(void ){
#line 40
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 40
}
#line 40
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 53
}
#line 53
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__RSTN__clr(void ){
#line 41
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 41
}
#line 41
# 476 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__resetRadio(void )
#line 476
{

  cc2420X_iocfg0_t iocfg0;
  cc2420X_mdmctrl0_t mdmctrl0;


  CC2420XDriverLayerP__RSTN__clr();
  CC2420XDriverLayerP__RSTN__set();


  iocfg0 = cc2420X_iocfg0_default;
  iocfg0.f.fifop_thr = 127;
  CC2420XDriverLayerP__writeRegister(CC2420X_IOCFG0, iocfg0.value);


  mdmctrl0 = cc2420X_mdmctrl0_default;
  mdmctrl0.f.reserved_frame_mode = 1;
  mdmctrl0.f.adr_decode = 0;

  CC2420XDriverLayerP__writeRegister(CC2420X_MDMCTRL0, mdmctrl0.value);

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD;
}


static inline void CC2420XDriverLayerP__initRadio(void )
{
  CC2420XDriverLayerP__resetRadio();



  CC2420XDriverLayerP__txPower = 0;
  CC2420XDriverLayerP__channel = 26 & CC2420X_CHANNEL_MASK;
}

# 66 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static void CC2420XDriverLayerP__SfdCapture__disable(void ){
#line 66
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable();
#line 66
}
#line 66
# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static __inline bool /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(uint8_t id)
{
  return /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state == /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_READY;
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static bool CC2420XDriverLayerP__RadioAlarm__isFree(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(0U);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 614 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__changeState(void )
{


  if ((
#line 617
  CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY || CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNON)
   && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_PD && CC2420XDriverLayerP__isSpiAcquired() && CC2420XDriverLayerP__RadioAlarm__isFree()) 
    {

      CC2420XDriverLayerP__strobe(CC2420X_SXOSCON);

      CC2420XDriverLayerP__RadioAlarm__wait(PD_2_IDLE_TIME);
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD_2_IDLE;
    }
  else {
#line 626
    if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNON && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__isSpiAcquired() && CC2420XDriverLayerP__RadioAlarm__isFree()) 
      {

        CC2420XDriverLayerP__setChannel();


        CC2420XDriverLayerP__strobe(CC2420X_SRXON);
        CC2420XDriverLayerP__RadioAlarm__wait(IDLE_2_RX_ON_TIME);
        CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE_2_RX_ON;
      }
    else {
      if ((
#line 636
      CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNOFF || CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY)
       && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON && CC2420XDriverLayerP__isSpiAcquired()) 
        {

          CC2420XDriverLayerP__SfdCapture__disable();
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 641
            CC2420XDriverLayerP__sfd_status = 0;
#line 641
            __nesc_atomic_end(__nesc_atomic); }


          CC2420XDriverLayerP__strobe(CC2420X_SRFOFF);
          CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE;
        }
      }
    }
#line 648
  if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_TURNOFF && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__isSpiAcquired()) 
    {

      CC2420XDriverLayerP__strobe(CC2420X_SXOSCOFF);


      CC2420XDriverLayerP__initRadio();
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_PD;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
    }
  else {
#line 658
    if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE) {
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
      }
    }
}

# 297 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 297
{
  uint8_t *base = target;

#line 299
  base[0] = value;
  return value;
}

# 224 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline message_t *FastCC2420TransceiverP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 224
{
  return msg;
}

# 11 "../iMAC_TDMA/async/FastReceive.nc"
inline static message_t *FastCC2420TransceiverP__Snoop__receive(am_id_t arg_0x409fdf08, message_t *msg, void *payload, uint8_t len){
#line 11
  nx_struct message_t *__nesc_result;
#line 11

#line 11
    __nesc_result = FastCC2420TransceiverP__Snoop__default__receive(arg_0x409fdf08, msg, payload, len);
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle();
#line 58
}
#line 58
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle();
}

# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 42
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle();
#line 42
}
#line 42
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 99
{
  LedsP__Led1__toggle();
  ;
#line 101
  ;
}

# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void RadioCountToLedsC__Leds__led1Toggle(void ){
#line 83
  LedsP__Leds__led1Toggle();
#line 83
}
#line 83
# 6 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLog.nc"
inline static error_t RadioCountToLedsC__UartLog__logEntry(__nesc_nxbase_nx_uint8_t type, __nesc_nxbase_nx_uint8_t sourceId, __nesc_nxbase_nx_uint16_t seq, __nesc_nxbase_nx_uint32_t timestamp){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = UartLogP__UartLog__logEntry(type, sourceId, seq, timestamp);
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 152 "RadioCountToLedsC.nc"
static inline message_t *RadioCountToLedsC__Receive__receive(message_t *msg, void *payload, uint8_t len)
#line 152
{
  RadioCountToLedsC__UartLog__logEntry(255, len, 153, sizeof(radio_count_msg_t ));
  ;
  if (len != sizeof(radio_count_msg_t )) {
#line 155
      return msg;
    }
  else 
#line 156
    {
      radio_count_msg_t *rcm = (radio_count_msg_t *)payload;

#line 158
      RadioCountToLedsC__Leds__led1Toggle();
      return msg;
    }
}

# 221 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline message_t *FastCC2420TransceiverP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 221
{
  return msg;
}

# 11 "../iMAC_TDMA/async/FastReceive.nc"
inline static message_t *FastCC2420TransceiverP__Receive__receive(am_id_t arg_0x409fd848, message_t *msg, void *payload, uint8_t len){
#line 11
  nx_struct message_t *__nesc_result;
#line 11

#line 11
  switch (arg_0x409fd848) {
#line 11
    case AM_RADIO_COUNT_MSG:
#line 11
      __nesc_result = RadioCountToLedsC__Receive__receive(msg, payload, len);
#line 11
      break;
#line 11
    default:
#line 11
      __nesc_result = FastCC2420TransceiverP__Receive__default__receive(arg_0x409fd848, msg, payload, len);
#line 11
      break;
#line 11
    }
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 106 "ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__amAddress(void )
#line 106
{




  return TOS_NODE_ID;
}

#line 72
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 72
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t FastCC2420TransceiverP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 231 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline am_addr_t FastCC2420TransceiverP__AMPacket__address(void )
#line 231
{
  return FastCC2420TransceiverP__ActiveMessageAddress__amAddress();
}

# 141 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static uint16_t FastCC2420TransceiverP__Ieee154PacketLayer__getDestAddr(message_t *msg){
#line 141
  unsigned int __nesc_result;
#line 141

#line 141
  __nesc_result = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg);
#line 141

#line 141
  return __nesc_result;
#line 141
}
#line 141
# 244 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline am_addr_t FastCC2420TransceiverP__AMPacket__destination(message_t *msg)
#line 244
{

  return FastCC2420TransceiverP__Ieee154PacketLayer__getDestAddr(msg);
}

#line 239
static __inline bool FastCC2420TransceiverP__AMPacket__isForMe(message_t *msg)
#line 239
{
  am_addr_t addr = FastCC2420TransceiverP__AMPacket__destination(msg);

#line 241
  return addr == FastCC2420TransceiverP__AMPacket__address() || addr == AM_BROADCAST_ADDR;
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t FastCC2420TransceiverP__SubPacket__payloadLength(message_t *msg){
#line 49
  unsigned char __nesc_result;
#line 49

#line 49
  __nesc_result = CC2420XDriverLayerP__RadioPacket__payloadLength(msg);
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 294 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint8_t FastCC2420TransceiverP__RadioPacket__payloadLength(message_t *msg)
#line 294
{

  return FastCC2420TransceiverP__SubPacket__payloadLength(msg) - sizeof(activemessage_header_t ) - sizeof(network_header_t ) - sizeof(ieee154_simple_header_t );
}

#line 332
static inline uint8_t FastCC2420TransceiverP__Packet__payloadLength(message_t *msg)
#line 332
{
  return FastCC2420TransceiverP__RadioPacket__payloadLength(msg);
}

# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

# 264 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline am_id_t FastCC2420TransceiverP__AMPacket__type(message_t *msg)
#line 264
{
  return __nesc_ntoh_uint8(FastCC2420TransceiverP__getHeader(msg)->type.nxdata);
}

#line 206
static inline message_t *FastCC2420TransceiverP__SubReceive__receive(message_t *msg)
#line 206
{
  am_id_t id = FastCC2420TransceiverP__AMPacket__type(msg);
  uint8_t len = FastCC2420TransceiverP__Packet__payloadLength(msg);

  void *payload = FastCC2420TransceiverP__Packet__getPayload(msg, len);


  if (FastCC2420TransceiverP__AMPacket__isForMe(msg)) {
      return FastCC2420TransceiverP__Receive__receive(id, msg, payload, len);
    }
  else 
#line 215
    {

      return FastCC2420TransceiverP__Snoop__receive(id, msg, payload, len);
    }
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static message_t */*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioReceive__receive(message_t *msg){
#line 53
  nx_struct message_t *__nesc_result;
#line 53

#line 53
  __nesc_result = FastCC2420TransceiverP__SubReceive__receive(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__send(message_t *msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = CC2420XDriverLayerP__RadioSend__send(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 292 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 292
{
  const uint8_t *base = source;

#line 294
  return base[0];
}

#line 327
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 327
{
  uint8_t *base = target;

#line 329
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 354 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__headerLength(message_t *msg)
#line 354
{
  return (unsigned short )& ((message_t *)0)->data - sizeof(cc2420xpacket_header_t );
}

# 29 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__headerLength(message_t *msg){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = FastCC2420TransceiverP__CC2420XDriverConfig__headerLength(msg);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 1382 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline uint8_t CC2420XDriverLayerP__RadioPacket__headerLength(message_t *msg)
{
  return CC2420XDriverLayerP__Config__headerLength(msg) + sizeof(cc2420x_header_t );
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = CC2420XDriverLayerP__RadioPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 77 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline ieee154_simple_header_t */*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(message_t *msg)
{
  return (void *)msg + /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__headerLength(msg);
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  CC2420XDriverLayerP__RadioPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 115 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack)
{
  ieee154_simple_header_t *header = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(ack);

#line 118
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__SubPacket__setPayloadLength(ack, /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_LENGTH);

  __nesc_hton_leuint16(header->fcf.nxdata, /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE);
  __nesc_hton_leuint8(header->dsn.nxdata, __nesc_ntoh_leuint8(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(data)->dsn.nxdata));
}

# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__createAckReply(message_t *data, message_t *ack){
#line 88
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createAckReply(data, ack);
#line 88
}
#line 88
# 465 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__SoftwareAckConfig__createAckPacket(message_t *data, message_t *ack)
{
  FastCC2420TransceiverP__Ieee154PacketLayer__createAckReply(data, ack);
}

# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__createAckPacket(message_t *data, message_t *ack){
#line 80
  FastCC2420TransceiverP__SoftwareAckConfig__createAckPacket(data, ack);
#line 80
}
#line 80
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 205 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(message_t *msg)
{
  return /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(msg)
   && /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(msg)
   && /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg) == /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__ActiveMessageAddress__amAddress();
}

# 168 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool FastCC2420TransceiverP__Ieee154PacketLayer__requiresAckReply(message_t *msg){
#line 168
  unsigned char __nesc_result;
#line 168

#line 168
  __nesc_result = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckReply(msg);
#line 168

#line 168
  return __nesc_result;
#line 168
}
#line 168
# 460 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__SoftwareAckConfig__requiresAckReply(message_t *msg)
{
  return FastCC2420TransceiverP__Ieee154PacketLayer__requiresAckReply(msg);
}

# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__requiresAckReply(message_t *msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = FastCC2420TransceiverP__SoftwareAckConfig__requiresAckReply(msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__sendDone(error_t error){
#line 56
  FastCC2420TransceiverP__SubSend__sendDone(error);
#line 56
}
#line 56
# 384 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__PacketFlag__set(uint8_t bit, message_t *msg)
#line 384
{
  FastCC2420TransceiverP__getMeta(msg)->flags |= 1 << bit;
}

# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__set(message_t *msg){
#line 50
  FastCC2420TransceiverP__PacketFlag__set(5U, msg);
#line 50
}
#line 50
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__stop(void ){
#line 73
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 108 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(uint8_t id)
{
  for (; 0; ) ;
  for (; 0; ) ;

  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__stop();
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_READY;
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__cancel(void ){
#line 55
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__cancel(1U);
#line 55
}
#line 55
# 322 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 322
{
  const uint8_t *base = source;

#line 324
  return ((uint16_t )base[1] << 8) | base[0];
}

# 124 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack)
{
  ieee154_simple_header_t *header = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(ack);

  return __nesc_ntoh_leuint8(header->dsn.nxdata) == __nesc_ntoh_leuint8(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(data)->dsn.nxdata)
   && (__nesc_ntoh_leuint16(header->fcf.nxdata) & /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_MASK) == /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE;
}

# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool FastCC2420TransceiverP__Ieee154PacketLayer__verifyAckReply(message_t *data, message_t *ack){
#line 94
  unsigned char __nesc_result;
#line 94

#line 94
  __nesc_result = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__verifyAckReply(data, ack);
#line 94

#line 94
  return __nesc_result;
#line 94
}
#line 94
# 450 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__SoftwareAckConfig__verifyAckPacket(message_t *data, message_t *ack)
{
  return FastCC2420TransceiverP__Ieee154PacketLayer__verifyAckReply(data, ack);
}

# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__verifyAckPacket(message_t *data, message_t *ack){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = FastCC2420TransceiverP__SoftwareAckConfig__verifyAckPacket(data, ack);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool FastCC2420TransceiverP__Ieee154PacketLayer__isAckFrame(message_t *msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 445 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__SoftwareAckConfig__isAckPacket(message_t *msg)
{
  return FastCC2420TransceiverP__Ieee154PacketLayer__isAckFrame(msg);
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__isAckPacket(message_t *msg){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = FastCC2420TransceiverP__SoftwareAckConfig__isAckPacket(msg);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 119 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline message_t */*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__receive(message_t *msg)
{
  for (; 0; ) ;

  if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__isAckPacket(msg)) 
    {
      if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state == /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_ACK_WAIT && /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__verifyAckPacket(/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__txMsg, msg)) 
        {
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__cancel();
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__set(/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__txMsg);

          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY;
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__sendDone(SUCCESS);
        }

      return msg;
    }

  if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state == /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY && /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__requiresAckReply(msg)) 
    {
      /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__createAckPacket(msg, &/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__ackMsg);


      if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__send(&/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__ackMsg) == SUCCESS) {
        /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_ACK_SEND;
        }
      else {
#line 145
        for (; 0; ) ;
        }
    }
  return /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioReceive__receive(msg);
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static message_t *CC2420XDriverLayerP__RadioReceive__receive(message_t *msg){
#line 53
  nx_struct message_t *__nesc_result;
#line 53

#line 53
  __nesc_result = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__receive(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void FastCC2420TransceiverP__TimeStampFlag__set(message_t *msg){
#line 50
  FastCC2420TransceiverP__PacketFlag__set(0U, msg);
#line 50
}
#line 50
# 418 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__PacketTimeStampRadio__set(message_t *msg, uint32_t value)
#line 418
{
  FastCC2420TransceiverP__TimeStampFlag__set(msg);
  FastCC2420TransceiverP__getMetaTimestamp(msg)->timestamp = value;
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420XDriverLayerP__PacketTimeStamp__set(message_t * msg, CC2420XDriverLayerP__PacketTimeStamp__size_type value){
#line 78
  FastCC2420TransceiverP__PacketTimeStampRadio__set(msg, value);
#line 78
}
#line 78
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void )
{
  return /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get();
}

# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/LocalTime.nc"
inline static uint32_t CC2420XDriverLayerP__LocalTime__get(void ){
#line 61
  unsigned long __nesc_result;
#line 61

#line 61
  __nesc_result = /*LocalTimeHybridMicroC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline uint16_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(void )
{
  return /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
}

# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__getNow(void ){
#line 109
  unsigned int __nesc_result;
#line 109

#line 109
  __nesc_result = /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static __inline tradio_size /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(uint8_t id)
{
  return /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__getNow();
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static tradio_size CC2420XDriverLayerP__RadioAlarm__getNow(void ){
#line 65
  unsigned int __nesc_result;
#line 65

#line 65
  __nesc_result = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__getNow(0U);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 1533 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__PacketLinkQuality__set(message_t *msg, uint8_t value)
{
  CC2420XDriverLayerP__getMeta(msg)->lqi = value;
}

# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void CC2420XDriverLayerP__RSSIFlag__set(message_t *msg){
#line 50
  FastCC2420TransceiverP__PacketFlag__set(2U, msg);
#line 50
}
#line 50
# 388 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__PacketFlag__clear(uint8_t bit, message_t *msg)
#line 388
{
  FastCC2420TransceiverP__getMeta(msg)->flags &= ~(1 << bit);
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void CC2420XDriverLayerP__TransmitPowerFlag__clear(message_t *msg){
#line 55
  FastCC2420TransceiverP__PacketFlag__clear(1U, msg);
#line 55
}
#line 55
# 1458 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__PacketRSSI__set(message_t *msg, uint8_t value)
{

  CC2420XDriverLayerP__TransmitPowerFlag__clear(msg);

  CC2420XDriverLayerP__RSSIFlag__set(msg);
  CC2420XDriverLayerP__getMeta(msg)->rssi = value;
}

# 202 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__SubReceive__header(message_t *msg)
#line 202
{
  return TRUE;
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioReceive__header(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = FastCC2420TransceiverP__SubReceive__header(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 106 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__header(message_t *msg)
{

  if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__isAckPacket(msg)) {
    return /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state == /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_ACK_WAIT;
    }




  return /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioReceive__header(msg);
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioReceive.nc"
inline static bool CC2420XDriverLayerP__RadioReceive__header(message_t *msg){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubReceive__header(msg);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 61
{
  return /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420XDriverLayerP__SfdCapture__captureRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 403 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__flushRxFifo(void )
#line 403
{



  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();


  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_READ | CC2420X_RXFIFO);
  CC2420XDriverLayerP__FastSpiByte__splitRead();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);

  CC2420XDriverLayerP__FastSpiByte__splitRead();
  CC2420XDriverLayerP__CSN__set();

  CC2420XDriverLayerP__strobe(CC2420X_SFLUSHRX);
  return CC2420XDriverLayerP__strobe(CC2420X_SFLUSHRX);
}

#line 932
static __inline void CC2420XDriverLayerP__enableReceiveSfd(void )
#line 932
{


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 935
    {

      CC2420XDriverLayerP__strobe(CC2420_SRFOFF);



      CC2420XDriverLayerP__flushRxFifo();


      CC2420XDriverLayerP__SfdCapture__captureRisingEdge();
      CC2420XDriverLayerP__sfd_status = 2;




      CC2420XDriverLayerP__strobe(CC2420_SRXON);
    }
#line 951
    __nesc_atomic_end(__nesc_atomic); }
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__SFD__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__FIFO__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__FIFOP__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 392 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline void CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(uint8_t *crcOkAndLqiPtr)
{

  for (; 0; ) ;

  *crcOkAndLqiPtr = CC2420XDriverLayerP__FastSpiByte__splitRead();


  CC2420XDriverLayerP__CSN__set();
}

#line 311
static __inline uint8_t CC2420XDriverLayerP__waitForRxFifo(void )
#line 311
{

  if (CC2420XDriverLayerP__FIFO__get() == 1) {

      return 1;
    }
  else 
#line 316
    {



      uint16_t timeout = CC2420XDriverLayerP__RadioAlarm__getNow() + 4 * CC2420X_SYMBOL_TIME;

      while (CC2420XDriverLayerP__FIFO__get() == 0 && timeout - CC2420XDriverLayerP__RadioAlarm__getNow() < 0x7fff) ;
      return CC2420XDriverLayerP__FIFO__get();
    }
}

#line 382
static __inline void CC2420XDriverLayerP__readRssiFromRxFifo(uint8_t *rssiPtr)
{

  for (; 0; ) ;

  *rssiPtr = CC2420XDriverLayerP__FastSpiByte__splitRead();
  CC2420XDriverLayerP__waitForRxFifo();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);
}

#line 367
static __inline void CC2420XDriverLayerP__readPayloadFromRxFifo(uint8_t *data, uint8_t length)
{
  uint8_t idx;


  for (; 0; ) ;


  for (idx = 0; idx < length; idx++) {
      data[idx] = CC2420XDriverLayerP__FastSpiByte__splitRead();
      CC2420XDriverLayerP__waitForRxFifo();
      CC2420XDriverLayerP__FastSpiByte__splitWrite(0);
    }
}

#line 102
static inline cc2420x_header_t *CC2420XDriverLayerP__getHeader(message_t *msg)
{
  return (void *)msg + CC2420XDriverLayerP__Config__headerLength(msg);
}

# 358 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__maxPayloadLength(void )
#line 358
{
  return sizeof(cc2420xpacket_header_t ) + 114;
}

# 35 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__maxPayloadLength(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = FastCC2420TransceiverP__CC2420XDriverConfig__maxPayloadLength();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 1401 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline uint8_t CC2420XDriverLayerP__RadioPacket__maxPayloadLength(void )
{
  for (; 0; ) ;

  return CC2420XDriverLayerP__Config__maxPayloadLength() - sizeof(cc2420x_header_t );
}

#line 327
static __inline cc2420X_status_t CC2420XDriverLayerP__readLengthFromRxFifo(uint8_t *lengthPtr)
{
  cc2420X_status_t status;

  for (; 0; ) ;
  for (; 0; ) ;


  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();




  CC2420XDriverLayerP__waitForRxFifo();
  for (; 0; ) ;


  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_READ | CC2420X_RXFIFO);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);

  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;
  for (; 0; ) ;

  *lengthPtr = CC2420XDriverLayerP__FastSpiByte__splitRead();



  CC2420XDriverLayerP__waitForRxFifo();
  CC2420XDriverLayerP__FastSpiByte__splitWrite(0);


  return status;
}

#line 107
static inline void *CC2420XDriverLayerP__getPayload(message_t *msg)
{
  return (void *)msg;
}

#line 1003
static __inline void CC2420XDriverLayerP__downloadMessage(void )
{
  uint8_t length;
  uint16_t crc = 1;
  uint8_t *data;
  uint8_t rssi;
  uint8_t crc_ok_lqi;
  uint16_t sfdTime;

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_DOWNLOAD;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1014
    sfdTime = CC2420XDriverLayerP__capturedTime;
#line 1014
    __nesc_atomic_end(__nesc_atomic); }


  data = CC2420XDriverLayerP__getPayload(CC2420XDriverLayerP__rxMsg) + sizeof(cc2420x_header_t );


  CC2420XDriverLayerP__readLengthFromRxFifo(&length);

  if (length < 3 || length > CC2420XDriverLayerP__RadioPacket__maxPayloadLength() + 2) {






      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;

      CC2420XDriverLayerP__enableReceiveSfd();



      return;
    }


  for (; 0; ) ;

  __nesc_hton_leuint8(CC2420XDriverLayerP__getHeader(CC2420XDriverLayerP__rxMsg)->length.nxdata, length);


  length -= 2;

  {

    CC2420XDriverLayerP__readPayloadFromRxFifo(data, length);


    CC2420XDriverLayerP__readRssiFromRxFifo(&rssi);
    CC2420XDriverLayerP__readCrcOkAndLqiFromRxFifo(&crc_ok_lqi);
  }



  if ((CC2420XDriverLayerP__FIFOP__get() == 1 || CC2420XDriverLayerP__FIFO__get() == 1) || CC2420XDriverLayerP__SFD__get() == 1) {
      for (; 0; ) ;


      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;

      for (; 0; ) ;
      CC2420XDriverLayerP__enableReceiveSfd();
      return;
    }

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
  CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;


  for (; 0; ) ;
  CC2420XDriverLayerP__enableReceiveSfd();


  if (!CC2420XDriverLayerP__RadioReceive__header(CC2420XDriverLayerP__rxMsg)) {
    return;
    }


  CC2420XDriverLayerP__PacketRSSI__set(CC2420XDriverLayerP__rxMsg, rssi);









  CC2420XDriverLayerP__PacketLinkQuality__set(CC2420XDriverLayerP__rxMsg, crc_ok_lqi & 0x7f);
  crc = crc_ok_lqi > 0x7f ? 0 : 1;
#line 1125
  if (crc == 0) {
      uint32_t time32;
      uint16_t time;

#line 1128
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1128
        {
          time = CC2420XDriverLayerP__RadioAlarm__getNow();
          time32 = CC2420XDriverLayerP__LocalTime__get();
        }
#line 1131
        __nesc_atomic_end(__nesc_atomic); }

      time -= sfdTime;
      time32 -= time;

      CC2420XDriverLayerP__PacketTimeStamp__set(CC2420XDriverLayerP__rxMsg, time32);
#line 1157
      CC2420XDriverLayerP__rxMsg = CC2420XDriverLayerP__RadioReceive__receive(CC2420XDriverLayerP__rxMsg);
    }
}

# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static void CC2420XDriverLayerP__RadioSend__sendDone(error_t error){
#line 56
  /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__sendDone(error);
#line 56
}
#line 56
# 1237 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__Tasklet__run(void )
{
#line 1263
  if (CC2420XDriverLayerP__txEnd) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1265
        if (CC2420XDriverLayerP__isSpiAcquired()) 
          {
            cc2420X_status_t status;

            CC2420XDriverLayerP__txEnd = FALSE;

            for (; 0; ) ;
            for (; 0; ) ;


            CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
            CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;



            status = CC2420XDriverLayerP__flushRxFifo();
            CC2420XDriverLayerP__enableReceiveSfd();
#line 1302
            if (status.tx_underflow == 1) {
                for (; 0; ) ;

                CC2420XDriverLayerP__strobe(CC2420X_SFLUSHTX);
                CC2420XDriverLayerP__RadioSend__sendDone(FAIL);
              }
            else 
#line 1307
              {
                CC2420XDriverLayerP__RadioSend__sendDone(SUCCESS);
              }
          }
        else {
          for (; 0; ) ;
          }
#line 1313
        __nesc_atomic_end(__nesc_atomic); }
    }
  if (CC2420XDriverLayerP__rxSfd) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 1317
        if (CC2420XDriverLayerP__isSpiAcquired()) 
          {
            CC2420XDriverLayerP__rxSfd = FALSE;

            for (; 0; ) ;
            for (; 0; ) ;

            CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_DOWNLOAD;
          }
        else {
          for (; 0; ) ;
          }
#line 1328
        __nesc_atomic_end(__nesc_atomic); }
    }

  if (CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE) 
    {
      if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_DOWNLOAD) {
          for (; 0; ) ;
          CC2420XDriverLayerP__downloadMessage();
        }
      else {
#line 1337
        if (CC2420XDriverLayerP__CMD_TURNOFF <= CC2420XDriverLayerP__cmd && CC2420XDriverLayerP__cmd <= CC2420XDriverLayerP__CMD_TURNON) {
          CC2420XDriverLayerP__changeState();
          }
        else {
#line 1339
          if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_CHANNEL) {
            CC2420XDriverLayerP__changeChannel();
            }
          }
        }
#line 1342
      if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_SIGNAL_DONE) 
        {
          CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_NONE;
          CC2420XDriverLayerP__RadioState__done();
        }
    }

  if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_NONE && CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON && !CC2420XDriverLayerP__rxSfd && !CC2420XDriverLayerP__txEnd) {
    CC2420XDriverLayerP__RadioSend__ready();
    }
  if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_NONE) {
    CC2420XDriverLayerP__releaseSpi__postTask();
    }
}

# 480 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__SoftwareAckConfig__reportChannelError(void )
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__reportChannelError(void ){
#line 86
  FastCC2420TransceiverP__SoftwareAckConfig__reportChannelError();
#line 86
}
#line 86
# 96 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__fired(void )
{
  for (; 0; ) ;

  /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__reportChannelError();

  /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY;
  /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__sendDone(SUCCESS);
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void CC2420XDriverLayerP__Tasklet__schedule(void ){
#line 59
  TaskletC__Tasklet__schedule();
#line 59
}
#line 59
# 202 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__RadioAlarm__fired(void )
{
  if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_PD_2_IDLE) {
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_IDLE;
      if (CC2420XDriverLayerP__cmd == CC2420XDriverLayerP__CMD_STANDBY) {
        CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;
        }
    }
  else {
#line 209
    if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_IDLE_2_RX_ON) {
        CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_RX_ON;
        CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_SIGNAL_DONE;

        for (; 0; ) ;
        CC2420XDriverLayerP__enableReceiveSfd();
      }
    else {
      for (; 0; ) ;
      }
    }
  CC2420XDriverLayerP__Tasklet__schedule();
}

# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(uint8_t id)
{
}

# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__fired(uint8_t arg_0x411cbe38){
#line 60
  switch (arg_0x411cbe38) {
#line 60
    case 0U:
#line 60
      CC2420XDriverLayerP__RadioAlarm__fired();
#line 60
      break;
#line 60
    case 1U:
#line 60
      /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__fired();
#line 60
      break;
#line 60
    default:
#line 60
      /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__default__fired(arg_0x411cbe38);
#line 60
      break;
#line 60
    }
#line 60
}
#line 60
# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run(void )
{
  if (/*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state == /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_FIRED) 
    {
      /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_READY;
      /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__fired(/*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__alarm);
    }
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/Tasklet.nc"
inline static void TaskletC__Tasklet__run(void ){
#line 48
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Tasklet__run();
#line 48
  CC2420XDriverLayerP__Tasklet__run();
#line 48
}
#line 48
# 74 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
static inline void TaskletC__doit(void )
{
  for (; ; ) 
    {
      TaskletC__Tasklet__run();

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          if (TaskletC__state == 1) 
            {
              TaskletC__state = 0;
              {
#line 85
                __nesc_atomic_end(__nesc_atomic); 
#line 85
                return;
              }
            }
          for (; 0; ) ;
          TaskletC__state = 1;
        }
#line 90
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 135 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 58
}
#line 58
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4373 {
#line 57
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 110
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 55
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 55
}
#line 55
# 130 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 57
}
#line 57
# 380 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__PacketFlag__get(uint8_t bit, message_t *msg)
#line 380
{
  return FastCC2420TransceiverP__getMeta(msg)->flags & (1 << bit);
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__get(message_t *msg){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = FastCC2420TransceiverP__PacketFlag__get(5U, msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 167 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__wasAcked(message_t *msg)
{
  return /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__get(msg);
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketAcknowledgements.nc"
inline static bool RadioCountToLedsC__Acks__wasAcked(message_t * msg){
#line 85
  unsigned char __nesc_result;
#line 85

#line 85
  __nesc_result = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__wasAcked(msg);
#line 85

#line 85
  return __nesc_result;
#line 85
}
#line 85
# 164 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__AMSend__sendDone(message_t *msg, error_t error)
#line 164
{
  if (&RadioCountToLedsC__packet == msg) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 166
        RadioCountToLedsC__locked = FALSE;
#line 166
        __nesc_atomic_end(__nesc_atomic); }
      RadioCountToLedsC__UartLog__logEntry(255, error, 167, RadioCountToLedsC__Acks__wasAcked(msg));
    }
}

# 196 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__Send__default__sendDone(am_id_t id, message_t *msg, error_t error)
#line 196
{
}

# 13 "../iMAC_TDMA/async/FastSend.nc"
inline static void FastCC2420TransceiverP__Send__sendDone(am_id_t arg_0x409fede8, message_t *msg, error_t error){
#line 13
  switch (arg_0x409fede8) {
#line 13
    case AM_RADIO_COUNT_MSG:
#line 13
      RadioCountToLedsC__AMSend__sendDone(msg, error);
#line 13
      break;
#line 13
    default:
#line 13
      FastCC2420TransceiverP__Send__default__sendDone(arg_0x409fede8, msg, error);
#line 13
      break;
#line 13
    }
#line 13
}
#line 13
# 46 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static inline void UartLogP__dropBlink(void )
#line 46
{
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t UartLogP__uartSendTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(UartLogP__uartSendTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 88 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static __inline nx_uint8_t *UartLogP__getPayload(message_t *msg)
#line 88
{
  return (nx_uint8_t *)msg->data;
}

static inline error_t UartLogP__UartLog__logByteStream(nx_uint8_t *log_p, uint8_t len)
#line 92
{
  nx_uint8_t *payload;

  if (len > 114) {
      return ESIZE;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (!UartLogP__uartFull) {







          payload = (nx_uint8_t *)UartLogP__getPayload(& UartLogP__uartQueue[UartLogP__uartIn]->msg);

          UartLogP__offset = UartLogP__uartQueue[UartLogP__uartIn]->aggr_counts++ * sizeof(logMsg );

          memcpy(payload + UartLogP__offset, log_p, len);
#line 128
          if (UartLogP__uartQueue[UartLogP__uartIn]->aggr_counts >= AGGR_COUNTS) {


              UartLogP__uartIn = (UartLogP__uartIn + 1) % UART_QUEUE_LEN;
              if (UartLogP__uartIn == UartLogP__uartOut) {
                UartLogP__uartFull = TRUE;
                }
              if (!UartLogP__uartBusy) {
                  UartLogP__uartSendTask__postTask();
                  UartLogP__uartBusy = TRUE;
                }
            }

          {
            unsigned char __nesc_temp = 
#line 141
            SUCCESS;

            {
#line 141
              __nesc_atomic_end(__nesc_atomic); 
#line 141
              return __nesc_temp;
            }
          }
        }
      else 
#line 142
        {
          UartLogP__dropBlink();
          {
            unsigned char __nesc_temp = 
#line 144
            FAIL;

            {
#line 144
              __nesc_atomic_end(__nesc_atomic); 
#line 144
              return __nesc_temp;
            }
          }
        }
    }
#line 148
    __nesc_atomic_end(__nesc_atomic); }
}

# 362 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__metadataLength(message_t *msg)
#line 362
{
  return 0;
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__metadataLength(message_t *msg){
#line 41
  unsigned char __nesc_result;
#line 41

#line 41
  __nesc_result = FastCC2420TransceiverP__CC2420XDriverConfig__metadataLength(msg);
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 1408 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline uint8_t CC2420XDriverLayerP__RadioPacket__metadataLength(message_t *msg)
{
  return CC2420XDriverLayerP__Config__metadataLength(msg) + sizeof(cc2420x_metadata_t );
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t FastCC2420TransceiverP__SubPacket__metadataLength(message_t *msg){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = CC2420XDriverLayerP__RadioPacket__metadataLength(msg);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 177 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 172 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 172
{
#line 172
  return FALSE;
}

# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0x40bfc068){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x40bfc068) {
#line 128
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0x40bfc068);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 112 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 112
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool CC2420XDriverLayerP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 176 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 176
{
  return &msp430_spi_default_config;
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x40bfcab8){
#line 39
  union __nesc_unnamed4291 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x40bfcab8);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 120
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40cdfa58){
#line 59
  switch (arg_0x40cdfa58) {
#line 59
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x40cdfa58);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 213 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 213
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 206 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x40ce1690){
#line 61
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x40ce1690);
#line 61
}
#line 61
# 93 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 100
          FAIL;

          {
#line 100
            __nesc_atomic_end(__nesc_atomic); 
#line 100
            return __nesc_temp;
          }
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 102
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 174 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 174
{
#line 174
  return FAIL;
}

# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x40bfc068){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x40bfc068) {
#line 97
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0x40bfc068);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 104
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420XDriverLayerP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 151 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 99
}
#line 99
# 207 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~(0x80 | 0x40);
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 238 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 0x40;
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~(0x80 | 0x40);
}









static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~(0x80 | 0x40);
}

# 204 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x40ce1690){
#line 53
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x40ce1690);
#line 53
}
#line 53
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = id;
          }
        else {
#line 88
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail] = id;
          }
#line 89
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 210 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 198 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(message_t *msg)
{
  return /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(msg)
   && /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(msg)
   && /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(msg) != 0xFFFF;
}

# 162 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static bool FastCC2420TransceiverP__Ieee154PacketLayer__requiresAckWait(message_t *msg){
#line 162
  unsigned char __nesc_result;
#line 162

#line 162
  __nesc_result = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__requiresAckWait(msg);
#line 162

#line 162
  return __nesc_result;
#line 162
}
#line 162
# 440 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__SoftwareAckConfig__requiresAckWait(message_t *msg)
{
  return FastCC2420TransceiverP__Ieee154PacketLayer__requiresAckWait(msg);
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__requiresAckWait(message_t *msg){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = FastCC2420TransceiverP__SoftwareAckConfig__requiresAckWait(msg);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static bool /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__isFree(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__isFree(1U);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(uint16_t dt)
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__getNow(), dt);
}

# 66 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__start(/*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__size_type dt){
#line 66
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__start(dt);
#line 66
}
#line 66
# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarmP.nc"
static inline void /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(uint8_t id, tradio_size timeout)
{
  for (; 0; ) ;

  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__alarm = id;
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__state = /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__STATE_WAIT;
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__Alarm__start(timeout);
}

# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__wait(tradio_size timeout){
#line 50
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(1U, timeout);
#line 50
}
#line 50
# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 45
  unsigned int __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 165 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 155 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 130 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 475 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint16_t FastCC2420TransceiverP__SoftwareAckConfig__getAckTimeout(void )
{
  return (uint16_t )(800 * 1);
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static uint16_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__getAckTimeout(void ){
#line 43
  unsigned int __nesc_result;
#line 43

#line 43
  __nesc_result = FastCC2420TransceiverP__SoftwareAckConfig__getAckTimeout();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static __inline uint32_t Msp430HybridAlarmCounterP__now2ghz(void )
#line 86
{
  uint16_t t32khz;
#line 87
  uint16_t tMicro;
  uint32_t t2ghz;

  Msp430HybridAlarmCounterP__now(&t32khz, &tMicro, &t2ghz);

  return t2ghz;
}







static inline uint32_t Msp430HybridAlarmCounterP__Counter2ghz__get(void )
#line 101
{
  return Msp430HybridAlarmCounterP__now2ghz();
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = Msp430HybridAlarmCounterP__Counter2ghz__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static bool Msp430HybridAlarmCounterP__Counter32khz__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline bool Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending(void )
#line 110
{
  return Msp430HybridAlarmCounterP__Counter32khz__isOverflowPending();
}

# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static bool /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = Msp430HybridAlarmCounterP__Counter2ghz__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static bool CC2420XDriverLayerP__TransmitPowerFlag__get(message_t *msg){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = FastCC2420TransceiverP__PacketFlag__get(1U, msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 1420 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline bool CC2420XDriverLayerP__PacketTransmitPower__isSet(message_t *msg)
{
  return CC2420XDriverLayerP__TransmitPowerFlag__get(msg);
}

static inline uint8_t CC2420XDriverLayerP__PacketTransmitPower__get(message_t *msg)
{
  return CC2420XDriverLayerP__getMeta(msg)->power;
}

# 371 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline bool FastCC2420TransceiverP__CC2420XDriverConfig__requiresRssiCca(message_t *msg)
#line 371
{


  return FastCC2420TransceiverP__AMPacket__type(msg) != AM_IMAC_LE;
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static bool CC2420XDriverLayerP__Config__requiresRssiCca(message_t *msg){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = FastCC2420TransceiverP__CC2420XDriverConfig__requiresRssiCca(msg);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420XDriverLayerP__CCA__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 366 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint8_t FastCC2420TransceiverP__CC2420XDriverConfig__headerPreloadLength(void )
#line 366
{

  return 7;
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/cc2420x/CC2420XDriverConfig.nc"
inline static uint8_t CC2420XDriverLayerP__Config__headerPreloadLength(void ){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = FastCC2420TransceiverP__CC2420XDriverConfig__headerPreloadLength();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 285 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static __inline cc2420X_status_t CC2420XDriverLayerP__writeTxFifo(uint8_t *data, uint8_t length)
{
  cc2420X_status_t status;
  uint8_t idx;

  for (; 0; ) ;

  CC2420XDriverLayerP__CSN__set();
  CC2420XDriverLayerP__CSN__clr();

  CC2420XDriverLayerP__FastSpiByte__splitWrite(CC2420X_CMD_REGISTER_WRITE | CC2420X_TXFIFO);
  for (idx = 0; idx < length; idx++) 
    CC2420XDriverLayerP__FastSpiByte__splitReadWrite(data[idx]);
  status.value = CC2420XDriverLayerP__FastSpiByte__splitRead();

  CC2420XDriverLayerP__CSN__set();
  return status;
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 65
{
  return /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420XDriverLayerP__SfdCapture__captureFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static bool CC2420XDriverLayerP__TimeSyncFlag__get(message_t *msg){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = FastCC2420TransceiverP__PacketFlag__get(4U, msg);
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 1494 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline bool CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return CC2420XDriverLayerP__TimeSyncFlag__get(msg);
}

static inline uint8_t CC2420XDriverLayerP__PacketTimeSyncOffset__get(message_t *msg)
{
  return CC2420XDriverLayerP__RadioPacket__headerLength(msg) + CC2420XDriverLayerP__RadioPacket__payloadLength(msg) - sizeof(timesync_absolute_t );
}

# 347 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 347
{
  uint8_t *base = target;

#line 349
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 372
static __inline  int32_t __nesc_hton_int32(void * target, int32_t value)
#line 372
{
#line 372
  __nesc_hton_uint32(target, value);
#line 372
  return value;
}

#line 340
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 340
{
  const uint8_t *base = source;

#line 342
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static uint8_t FastCC2420TransceiverP__SubPacket__maxPayloadLength(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = CC2420XDriverLayerP__RadioPacket__maxPayloadLength();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
#line 43
inline static uint8_t FastCC2420TransceiverP__SubPacket__headerLength(message_t *msg){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = CC2420XDriverLayerP__RadioPacket__headerLength(msg);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 289 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline uint8_t FastCC2420TransceiverP__RadioPacket__headerLength(message_t *msg)
#line 289
{

  return FastCC2420TransceiverP__SubPacket__headerLength(msg) + sizeof(activemessage_header_t ) + sizeof(network_header_t ) + sizeof(ieee154_simple_header_t );
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 99
}
#line 99
# 192 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 1166 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__SfdCapture__captured(uint16_t time)
{


  for (; 0; ) ;
  for (; 0; ) ;

  CC2420XDriverLayerP__SfdCapture__disable();
  CC2420XDriverLayerP__sfd_status = 0;

  if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON) {
      CC2420XDriverLayerP__rxSfd = TRUE;
      CC2420XDriverLayerP__capturedTime = time;
    }
  else {
#line 1179
    if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_TX_ON || CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON) {
        CC2420XDriverLayerP__txEnd = TRUE;
      }
    else 
#line 1181
      {

        for (; 0; ) ;
      }
    }
#line 1208
  CC2420XDriverLayerP__Tasklet__schedule();
}

# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 61
  CC2420XDriverLayerP__SfdCapture__captured(time);
#line 61
}
#line 61
# 175 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 68
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 68
}
#line 68
# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 76
{
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__captured(time);
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4374 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 191 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__Alarm2ghz__default__fired(void )
#line 191
{
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void Msp430HybridAlarmCounterP__Alarm2ghz__fired(void ){
#line 78
  Msp430HybridAlarmCounterP__Alarm2ghz__default__fired();
#line 78
}
#line 78
# 176 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline void Msp430HybridAlarmCounterP__AlarmMicro__fired(void )
#line 176
{

  Msp430HybridAlarmCounterP__Alarm2ghz__fired();
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired(void ){
#line 78
  Msp430HybridAlarmCounterP__AlarmMicro__fired();
#line 78
}
#line 78
# 135 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 58
}
#line 58
# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired(void )
{
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__disableEvents();
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__fired();
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4375 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4376 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4377 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4378 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4379 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TAR;

#line 8
  while (TAR - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = MotePlatformC__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 160 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 148
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 112
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = (0x0200 | 0x0080) | 0x0002;
}

#line 142
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0100 | 0x0002;
}

#line 137
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 132
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 178 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TACCR0 = TAR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TACCTL0 &= ~0x0001;
      while ((TACCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TBR;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 64
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0100 | 0x0020;
  Msp430ClockP__TBCTL = 0x0200 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 127
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 226 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 10 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 36 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 82 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__Resource__granted(void )
#line 82
{
}

# 165 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id)
#line 165
{
}

# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(uint8_t arg_0x411b23f0){
#line 102
  switch (arg_0x411b23f0) {
#line 102
    case 0U:
#line 102
      FastCC2420TransceiverP__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__default__granted(arg_0x411b23f0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 171 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x411b1a58){
#line 59
    /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(arg_0x411b1a58);
#line 59
}
#line 59
# 155 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId;
      /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__granted(/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
}

# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 110
  UartLogP__UartSend__sendDone(msg, error);
#line 110
}
#line 110
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 65
{
  /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 215 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x411219d8, message_t * msg, error_t error){
#line 100
  switch (arg_0x411219d8) {
#line 100
    case 0U:
#line 100
      /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x411219d8, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 126 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 169
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 169
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x41120490, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(arg_0x41120490, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(message_t * msg)
#line 60
{
  return (serial_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 122
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(message_t *msg)
#line 122
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

#line 124
  return __nesc_ntoh_uint8(header->length.nxdata);
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 78
  unsigned int __nesc_result;
#line 78

#line 78
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(amsg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 172 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(message_t *amsg)
#line 172
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 174
  return __nesc_ntoh_uint8(header->type.nxdata);
}

# 147 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 147
  unsigned char __nesc_result;
#line 147

#line 147
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(amsg);
#line 147

#line 147
  return __nesc_result;
#line 147
}
#line 147
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 174
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 174
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 182
      nextErr = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 310 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 310
{
  const uint8_t *base = source;

#line 312
  return ((uint16_t )base[0] << 8) | base[1];
}

# 131 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength(void )
#line 131
{
  return 114;
}

# 538 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__startSend(uint8_t b)
#line 538
{
  bool not_busy = FALSE;

#line 540
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 540
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE) 
        {
          unsigned char __nesc_temp = 
#line 542
          EOFF;

          {
#line 542
            __nesc_atomic_end(__nesc_atomic); 
#line 542
            return __nesc_temp;
          }
        }
    }
#line 545
    __nesc_atomic_end(__nesc_atomic); }
#line 544
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 544
    {
      if (SerialP__txBuf[SerialP__TX_DATA_INDEX].state == SerialP__BUFFER_AVAILABLE) {
          SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_FILLING;
          SerialP__txBuf[SerialP__TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 550
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP__MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(uint8_t first_byte){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = SerialP__SendBytePacket__startSend(first_byte);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__dataLinkLength(message_t *msg, uint8_t upperLen)
#line 54
{
  return upperLen + sizeof(serial_header_t );
}

# 361 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 362
{
  return 0;
}

# 23 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(uart_id_t arg_0x40fc12a0, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x40fc12a0) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP__Info__dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__dataLinkLength(arg_0x40fc12a0, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 51 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__offset(void )
#line 51
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 358 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(uart_id_t id)
#line 358
{
  return 0;
}

# 15 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(uart_id_t arg_0x40fc12a0){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x40fc12a0) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP__Info__offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__offset(arg_0x40fc12a0);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 111 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(uint8_t id, message_t *msg, uint8_t len)
#line 111
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 119
            ESIZE;

            {
#line 119
              __nesc_atomic_end(__nesc_atomic); 
#line 119
              return __nesc_temp;
            }
          }
        }
#line 122
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
      return FAIL;
    }
}

# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__RunTx__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__RunTx);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/TelosSerialP.nc"
static inline void TelosSerialP__Resource__granted(void )
#line 56
{
}

# 218 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(uint8_t id)
#line 218
{
}

# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(uint8_t arg_0x41005aa0){
#line 102
  switch (arg_0x41005aa0) {
#line 102
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 102
      TelosSerialP__Resource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__default__granted(arg_0x41005aa0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 101 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(uint8_t id)
#line 101
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__granted(id);
}

# 202 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x40ce2bd0){
#line 102
  switch (arg_0x40ce2bd0) {
#line 102
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 102
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__granted(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x40ce2bd0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 216 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x40cdfa58){
#line 59
  switch (arg_0x40cdfa58) {
#line 59
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 59
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x40cdfa58);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 190 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
}

# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP__Msp430UartConfigure__getConfig(void )
#line 58
{
  return &TelosSerialP__msp430_uart_telos_config;
}

# 214 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(uint8_t id)
#line 214
{
  return &msp430_uart_default_config;
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(uint8_t arg_0x41000a88){
#line 39
  union __nesc_unnamed4295 *__nesc_result;
#line 39

#line 39
  switch (arg_0x41000a88) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 39
      __nesc_result = TelosSerialP__Msp430UartConfigure__getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__default__getConfig(arg_0x41000a88);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 359 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableIntr(void )
#line 359
{
  HplMsp430Usart1P__IE2 &= ~(0x20 | 0x10);
}

#line 347
static inline void HplMsp430Usart1P__Usart__clrIntr(void )
#line 347
{
  HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
}

#line 159
static inline void HplMsp430Usart1P__Usart__resetUsart(bool reset)
#line 159
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 163
    U1CTL &= ~0x01;
    }
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 6;
}

# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectModuleFunc();
#line 92
}
#line 92
# 220 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartTx(void )
#line 220
{
  HplMsp430Usart1P__UTXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x20;
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 7);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectIOFunc();
#line 99
}
#line 99
# 236 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartRx(void )
#line 236
{
  HplMsp430Usart1P__ME2 &= ~0x10;
  HplMsp430Usart1P__URXD__selectIOFunc();
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 7;
}

# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__URXD__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP__23__IO__selectModuleFunc();
#line 92
}
#line 92
# 231 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableUartRx(void )
#line 231
{
  HplMsp430Usart1P__URXD__selectModuleFunc();
  HplMsp430Usart1P__ME2 |= 0x10;
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 6);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP__22__IO__selectIOFunc();
#line 99
}
#line 99
# 225 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__disableUartTx(void )
#line 225
{
  HplMsp430Usart1P__ME2 &= ~0x20;
  HplMsp430Usart1P__UTXD__selectIOFunc();
}

#line 203
static inline void HplMsp430Usart1P__Usart__enableUart(void )
#line 203
{
  /* atomic removed: atomic calls only */
#line 204
  {
    HplMsp430Usart1P__UTXD__selectModuleFunc();
    HplMsp430Usart1P__URXD__selectModuleFunc();
  }
  HplMsp430Usart1P__ME2 |= 0x20 | 0x10;
}

#line 151
static inline void HplMsp430Usart1P__Usart__setUmctl(uint8_t control)
#line 151
{
  U1MCTL = control;
}

#line 140
static inline void HplMsp430Usart1P__Usart__setUbr(uint16_t control)
#line 140
{
  /* atomic removed: atomic calls only */
#line 141
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 283
static inline void HplMsp430Usart1P__configUart(msp430_uart_union_config_t *config)
#line 283
{

  U1CTL = (config->uartRegisters.uctl & ~0x04) | 0x01;
  HplMsp430Usart1P__U1TCTL = config->uartRegisters.utctl;
  HplMsp430Usart1P__U1RCTL = config->uartRegisters.urctl;

  HplMsp430Usart1P__Usart__setUbr(config->uartRegisters.ubr);
  HplMsp430Usart1P__Usart__setUmctl(config->uartRegisters.umctl);
}

static inline void HplMsp430Usart1P__Usart__setModeUart(msp430_uart_union_config_t *config)
#line 293
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
    {
      HplMsp430Usart1P__Usart__resetUsart(TRUE);
      HplMsp430Usart1P__Usart__disableSpi();
      HplMsp430Usart1P__configUart(config);
      if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 1) {
          HplMsp430Usart1P__Usart__enableUart();
        }
      else {
#line 301
        if (config->uartConfig.utxe == 0 && config->uartConfig.urxe == 1) {
            HplMsp430Usart1P__Usart__disableUartTx();
            HplMsp430Usart1P__Usart__enableUartRx();
          }
        else {
#line 304
          if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 0) {
              HplMsp430Usart1P__Usart__disableUartRx();
              HplMsp430Usart1P__Usart__enableUartTx();
            }
          else 
#line 307
            {
              HplMsp430Usart1P__Usart__disableUart();
            }
          }
        }
#line 310
      HplMsp430Usart1P__Usart__resetUsart(FALSE);
      HplMsp430Usart1P__Usart__clrIntr();
      HplMsp430Usart1P__Usart__disableIntr();
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 174 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P__Usart__setModeUart(config);
#line 174
}
#line 174
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 1);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP__33__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 2);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP__34__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )51U &= ~(0x01 << 3);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP__35__IO__selectIOFunc();
#line 99
}
#line 99
# 377 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__enableIntr(void )
#line 377
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 378
    {
      HplMsp430Usart1P__IFG2 &= ~(0x20 | 0x10);
      HplMsp430Usart1P__IE2 |= 0x20 | 0x10;
    }
#line 381
    __nesc_atomic_end(__nesc_atomic); }
}

# 182 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr(void ){
#line 182
  HplMsp430Usart1P__Usart__enableIntr();
#line 182
}
#line 182
# 189 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 110 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(am_id_t arg_0x40f01068, message_t * msg, error_t error){
#line 110
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x40f01068, msg, error);
#line 110
}
#line 110
# 101 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(message_t *msg, error_t result)
#line 101
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, result);
}

# 376 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 376
{
  return;
}

# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(uart_id_t arg_0x40fc2760, message_t * msg, error_t error){
#line 100
  switch (arg_0x40fc2760) {
#line 100
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 100
      /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__default__sendDone(arg_0x40fc2760, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 158 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask(void )
#line 158
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 162
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError;
#line 162
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendCancelled) {
#line 164
    error = ECANCEL;
    }
#line 165
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Send__sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer, error);
}

#line 212
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(uint8_t which)
#line 212
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 0;
    }
}

# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 109
{
  return msg;
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(am_id_t arg_0x40f01a28, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__default__receive(arg_0x40f01a28, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 113
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Receive__receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__type(msg), msg, msg->data, len);
}

# 371 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 373
{
  return msg;
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(uart_id_t arg_0x40fc2120, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x40fc2120) {
#line 78
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 78
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubReceive__receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__default__receive(arg_0x40fc2120, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP__Info__upperLength(message_t *msg, uint8_t dataLinkLen)
#line 57
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 365 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 366
{
  return 0;
}

# 31 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(uart_id_t arg_0x40fc12a0, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x40fc12a0) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP__Info__upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__default__upperLength(arg_0x40fc12a0, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 275 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask(void )
#line 275
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 280
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich;
    }
#line 285
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__Receive__receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 289
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = FALSE;
    }
#line 293
    __nesc_atomic_end(__nesc_atomic); }
}

# 85 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static inline void UartLogP__UartControl__stopDone(error_t err)
#line 85
{
}

# 138 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__stopDone(error_t error){
#line 138
  UartLogP__UartControl__stopDone(error);
#line 138
}
#line 138
# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__stop(void )
#line 109
{
  HplMsp430Usart1P__Usart__disableSpi();
  HplMsp430Usart1P__Usart__disableUart();
  return SUCCESS;
}

# 105 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 84 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 84
{
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 62
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 62
}
#line 62
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 79
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__PowerDownCleanup__cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__stop();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart(void ){
#line 128
  HplMsp430Usart1P__Usart__disableUart();
#line 128
}
#line 128
#line 179
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr(void ){
#line 179
  HplMsp430Usart1P__Usart__disableIntr();
#line 179
}
#line 179
#line 97
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart1P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 92 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__resetUsart(TRUE);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__disableUart();
}

# 218 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x40cdfa58){
#line 65
  switch (arg_0x40cdfa58) {
#line 65
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 65
      /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__unconfigure(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x40cdfa58);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 72
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 111 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Queue__dequeue();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 213 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(uint8_t id)
#line 213
{
#line 213
  return FAIL;
}

# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(uint8_t arg_0x41000030){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x41000030) {
#line 120
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__release(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__release(arg_0x41000030);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 210 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(uint8_t id)
#line 210
{
#line 210
  return FALSE;
}

# 128 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(uint8_t arg_0x41000030){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x41000030) {
#line 128
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__isOwner(arg_0x41000030);
#line 128
      break;
#line 128
    }
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 77 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(uint8_t id)
#line 77
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 80
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf || /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
    return EBUSY;
    }
#line 82
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__release(id);
}

# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__release(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__stop(void )
#line 52
{
  TelosSerialP__Resource__release();
  return SUCCESS;
}

# 105 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = TelosSerialP__StdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 336 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__flushDone(void )
#line 336
{
  SerialP__SerialControl__stop();
  SerialP__SplitControl__stopDone(SUCCESS);
}

static inline void SerialP__defaultSerialFlushTask__runTask(void )
#line 341
{
  SerialP__SerialFlush__flushDone();
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__defaultSerialFlushTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__defaultSerialFlushTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 344 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFlush__default__flush(void )
#line 344
{
  SerialP__defaultSerialFlushTask__postTask();
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFlush.nc"
inline static void SerialP__SerialFlush__flush(void ){
#line 49
  SerialP__SerialFlush__default__flush();
#line 49
}
#line 49
# 332 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__stopDoneTask__runTask(void )
#line 332
{
  SerialP__SerialFlush__flush();
}

# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static error_t UartLogP__UartControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = SerialP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 66 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static inline void UartLogP__UartControl__startDone(error_t err)
#line 66
{
  uint8_t i;

#line 68
  if (err != SUCCESS) {
    UartLogP__UartControl__start();
    }
  else 
#line 70
    {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 72
        {
          for (i = 0; i < UART_QUEUE_LEN; i++) {
              UartLogP__uartQueue[i] = &UartLogP__uartQueueBufs[i];
              UartLogP__uartQueue[i]->aggr_counts = 0;
            }
          UartLogP__uartIn = UartLogP__uartOut = 0;
          UartLogP__uartBusy = FALSE;
          UartLogP__uartFull = FALSE;
        }
#line 80
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void SerialP__SplitControl__startDone(error_t error){
#line 113
  UartLogP__UartControl__startDone(error);
#line 113
}
#line 113
# 133 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__default_owner_id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId;
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 105 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P__AsyncStdControl__start(void )
#line 105
{
  return SUCCESS;
}

# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = HplMsp430Usart1P__AsyncStdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 74 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested(void )
#line 74
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__AsyncStdControl__start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__release();
}

# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP__0__ResourceDefaultOwner__immediateRequested();
#line 81
}
#line 81
# 206 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(uint8_t arg_0x40ce1690){
#line 61
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__default__immediateRequested(arg_0x40ce1690);
#line 61
}
#line 61
# 93 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceRequested__immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_IMM_GRANTING;
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 100
          FAIL;

          {
#line 100
            __nesc_atomic_end(__nesc_atomic); 
#line 100
            return __nesc_temp;
          }
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
#line 102
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED;
#line 107
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 212 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 212
{
#line 212
  return FAIL;
}

# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(uint8_t arg_0x41000030){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x41000030) {
#line 97
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__default__immediateRequest(arg_0x41000030);
#line 97
      break;
#line 97
    }
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(uint8_t id)
#line 65
{
  return /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__immediateRequest(id);
}

# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP__Resource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__Resource__immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/TelosSerialP.nc"
static inline error_t TelosSerialP__StdControl__start(void )
#line 49
{
  return TelosSerialP__Resource__immediateRequest();
}

# 95 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/StdControl.nc"
inline static error_t SerialP__SerialControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = TelosSerialP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 322 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__startDoneTask__runTask(void )
#line 322
{
  SerialP__SerialControl__start();
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 324
    {
      SerialP__txState = SerialP__TXSTATE_IDLE;
      SerialP__rxState = SerialP__RXSTATE_NOSYNC;
    }
#line 327
    __nesc_atomic_end(__nesc_atomic); }
  SerialP__SplitControl__startDone(SUCCESS);
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__startDoneTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__startDoneTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putDelimiter(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = HdlcTranslateC__SerialFrameComm__putDelimiter();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 194 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error_t error)
#line 194
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 195
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendError = error;
#line 195
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__postTask();
}

# 91 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP__SendBytePacket__sendCompleted(error_t error){
#line 91
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__sendCompleted(error);
#line 91
}
#line 91
# 244 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_empty(void )
#line 244
{
  bool ret;

#line 246
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    ret = SerialP__ackQ.writePtr == SerialP__ackQ.readPtr;
#line 246
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP__ack_queue_top(void )
#line 260
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 262
  {
    if (!SerialP__ack_queue_is_empty()) {
        tmp = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP__ack_queue_pop(void )
#line 270
{
  uint8_t retval = 0;

  /* atomic removed: atomic calls only */
#line 272
  {
    if (SerialP__ackQ.writePtr != SerialP__ackQ.readPtr) {
        retval = SerialP__ackQ.buf[SerialP__ackQ.readPtr];
        if (++ SerialP__ackQ.readPtr > SerialP__ACK_QUEUE_SIZE) {
#line 275
          SerialP__ackQ.readPtr = 0;
          }
      }
  }
#line 278
  return retval;
}

#line 559
static inline void SerialP__RunTx__runTask(void )
#line 559
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 576
    {
      SerialP__txPending = 0;
      idle = SerialP__txState == SerialP__TXSTATE_IDLE;
      done = SerialP__txState == SerialP__TXSTATE_FINISH;
      fail = SerialP__txState == SerialP__TXSTATE_ERROR;
      if (done || fail) {
          SerialP__txState = SerialP__TXSTATE_IDLE;
          SerialP__txBuf[SerialP__txIndex].state = SerialP__BUFFER_AVAILABLE;
        }
    }
#line 585
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 589
        {
          SerialP__txSeqno++;
          if (SerialP__txProto == SERIAL_PROTO_ACK) {
              SerialP__ack_queue_pop();
            }
          else {
              result = done ? SUCCESS : FAIL;
              send_completed = TRUE;
            }
        }
#line 598
        __nesc_atomic_end(__nesc_atomic); }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 605
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 605
        goInactive = SerialP__offPending;
#line 605
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 607
            SerialP__txState = SerialP__TXSTATE_INACTIVE;
#line 607
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 613
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 613
            {
              myAckState = SerialP__txBuf[SerialP__TX_ACK_INDEX].state;
              myDataState = SerialP__txBuf[SerialP__TX_DATA_INDEX].state;
            }
#line 616
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP__ack_queue_is_empty() && myAckState == SerialP__BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 618
                {
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].state = SerialP__BUFFER_COMPLETE;
                  SerialP__txBuf[SerialP__TX_ACK_INDEX].buf = SerialP__ack_queue_top();

                  SerialP__txProto = SERIAL_PROTO_ACK;
                  SerialP__txIndex = SerialP__TX_ACK_INDEX;
                  start_it = TRUE;
                }
#line 625
                __nesc_atomic_end(__nesc_atomic); }
            }
          else {
#line 627
            if (myDataState == SerialP__BUFFER_FILLING || myDataState == SerialP__BUFFER_COMPLETE) {
                { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 628
                  {
                    SerialP__txProto = SERIAL_PROTO_PACKET_NOACK;
                    SerialP__txIndex = SerialP__TX_DATA_INDEX;
                    start_it = TRUE;
                  }
#line 632
                  __nesc_atomic_end(__nesc_atomic); }
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP__SendBytePacket__sendCompleted(result);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 646
    {
      if (SerialP__txState == SerialP__TXSTATE_INACTIVE) {
          SerialP__testOff();
          {
#line 649
            __nesc_atomic_end(__nesc_atomic); 
#line 649
            return;
          }
        }
    }
#line 652
    __nesc_atomic_end(__nesc_atomic); }
  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 655
        {
          SerialP__txCRC = 0;
          SerialP__txByteCnt = 0;
          SerialP__txState = SerialP__TXSTATE_PROTO;
        }
#line 659
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP__SerialFrameComm__putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 661
            SerialP__txState = SerialP__TXSTATE_ERROR;
#line 661
            __nesc_atomic_end(__nesc_atomic); }
          SerialP__MaybeScheduleTx();
        }
    }
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP__stopDoneTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(SerialP__stopDoneTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC__UartStream__send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 50 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static inline void UartLogP__failBlink(void )
#line 50
{
}

# 286 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

# 127 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 127
{
  __nesc_hton_uint8(/*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg)->length.nxdata, len);
}

# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 177 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(message_t *amsg, am_id_t type)
#line 177
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 179
  __nesc_hton_uint8(header->type.nxdata, type);
}

# 162 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 162
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setType(amsg, t);
#line 162
}
#line 162
# 315 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 315
{
  uint8_t *base = target;

#line 317
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 158 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 158
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 160
  __nesc_hton_uint16(header->dest.nxdata, addr);
}

# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static void /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 103
  /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__setDestination(amsg, addr);
#line 103
}
#line 103
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline error_t /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 100);
  return /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static error_t UartLogP__UartSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*UartLogC.SerialSender.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 149 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static inline void UartLogP__uartSendTask__runTask(void )
#line 149
{
  uint8_t len;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 152
    {

      if (UartLogP__uartIn == UartLogP__uartOut && !UartLogP__uartFull) {
          UartLogP__uartBusy = FALSE;
          {
#line 156
            __nesc_atomic_end(__nesc_atomic); 
#line 156
            return;
          }
        }


      len = UartLogP__uartQueue[UartLogP__uartOut]->aggr_counts * sizeof(logMsg );
      if (SUCCESS == UartLogP__UartSend__send(AM_BROADCAST_ADDR, & UartLogP__uartQueue[UartLogP__uartOut]->msg, len)) {


          ;
        }
      else 
#line 166
        {
          UartLogP__failBlink();
          UartLogP__uartSendTask__postTask();
        }
    }
#line 170
    __nesc_atomic_end(__nesc_atomic); }
}

# 175 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 175
{
#line 175
  return FAIL;
}

# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x40bfc068){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x40bfc068) {
#line 120
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x40bfc068);
#line 120
      break;
#line 120
    }
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 116 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 116
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 120 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420XDriverLayerP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420XC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 514 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__SpiResource__granted(void )
{

  CC2420XDriverLayerP__CSN__makeOutput();
  CC2420XDriverLayerP__CSN__set();

  if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_VR_ON) 
    {
      CC2420XDriverLayerP__initRadio();

      CC2420XDriverLayerP__SpiResource__release();
    }
  else {

    CC2420XDriverLayerP__Tasklet__schedule();
    }
}

# 180 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 180
{
}

# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x40bdac30){
#line 102
  switch (arg_0x40bdac30) {
#line 102
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 102
      CC2420XDriverLayerP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x40bdac30);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 130 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 130
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 202 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x40ce2bd0){
#line 102
  switch (arg_0x40ce2bd0) {
#line 102
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x40ce2bd0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 190 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead;

#line 72
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 124 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 124
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 218 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x40cdfa58){
#line 65
  switch (arg_0x40cdfa58) {
#line 65
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 65
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x40cdfa58);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 99
}
#line 99
# 208 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 208
{
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 251 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 251
{
}

# 82 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x40bfd838, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x40bfd838, txBuf, rxBuf, len, error);
#line 82
}
#line 82
# 244 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 244
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 227
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 227
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 228
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 228
    __nesc_atomic_end(__nesc_atomic); }
}

# 1230 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__releaseSpi__runTask(void )
{

  CC2420XDriverLayerP__SpiResource__release();
}

# 118 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__AMControl__stopDone(error_t err)
#line 118
{
}

# 138 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void FastCC2420TransceiverP__SplitControl__stopDone(error_t error){
#line 138
  RadioCountToLedsC__AMControl__stopDone(error);
#line 138
}
#line 138
# 109 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__stopTask__runTask(void )
#line 109
{
  FastCC2420TransceiverP__SplitControl__stopDone(SUCCESS);
}

# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static error_t RadioCountToLedsC__AMControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = FastCC2420TransceiverP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 144 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void RadioCountToLedsC__MilliTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 64
}
#line 64
# 109 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__AMControl__startDone(error_t err)
#line 109
{
  if (err == SUCCESS) {
      RadioCountToLedsC__MilliTimer__startPeriodic(50);
    }
  else {
      RadioCountToLedsC__AMControl__start();
    }
}

# 113 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void FastCC2420TransceiverP__SplitControl__startDone(error_t error){
#line 113
  RadioCountToLedsC__AMControl__startDone(error);
#line 113
}
#line 113
# 106 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__startTask__runTask(void )
#line 106
{
  FastCC2420TransceiverP__SplitControl__startDone(SUCCESS);
}

# 707 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline error_t CC2420XDriverLayerP__RadioState__turnOn(void )
{
  if (CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE || (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_PD && !CC2420XDriverLayerP__RadioAlarm__isFree())) {
    return EBUSY;
    }
  else {
#line 711
    if (CC2420XDriverLayerP__state == CC2420XDriverLayerP__STATE_RX_ON) {
      return EALREADY;
      }
    }








  CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_TURNON;
  CC2420XDriverLayerP__Tasklet__schedule();

  return SUCCESS;
}

# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioState.nc"
inline static error_t FastCC2420TransceiverP__RadioState__turnOn(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = CC2420XDriverLayerP__RadioState__turnOn();
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 93
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 129 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/PacketFlag.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__clear(message_t *msg){
#line 55
  FastCC2420TransceiverP__PacketFlag__clear(5U, msg);
#line 55
}
#line 55
# 50 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__send(message_t *msg)
{
  error_t error;

  if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state == /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY) 
    {
      if ((error = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__send(msg)) == SUCCESS) 
        {
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__AckReceivedFlag__clear(msg);
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_DATA_SEND;
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__txMsg = msg;
        }
    }
  else {
    error = EBUSY;
    }
  return error;
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioSend.nc"
inline static error_t FastCC2420TransceiverP__SubSend__send(message_t *msg){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__send(msg);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 268 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline void FastCC2420TransceiverP__AMPacket__setType(message_t *msg, am_id_t type)
#line 268
{
  __nesc_hton_uint8(FastCC2420TransceiverP__getHeader(msg)->type.nxdata, type);
}

#line 91
static inline network_header_t *FastCC2420TransceiverP__getNetworkHeader(message_t *msg)
#line 91
{

  return (void *)msg + FastCC2420TransceiverP__SubPacket__headerLength(msg) + sizeof(ieee154_simple_header_t );
}

# 193 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr)
{
  __nesc_hton_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->src.nxdata, addr);
}

# 156 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__setSrcAddr(message_t *msg, uint16_t addr){
#line 156
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setSrcAddr(msg, addr);
#line 156
}
#line 156
# 259 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline void FastCC2420TransceiverP__AMPacket__setSource(message_t *msg, am_addr_t addr)
#line 259
{

  FastCC2420TransceiverP__Ieee154PacketLayer__setSrcAddr(msg, addr);
}

# 183 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr)
{
  __nesc_hton_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dest.nxdata, addr);
}

# 146 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__setDestAddr(message_t *msg, uint16_t addr){
#line 146
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestAddr(msg, addr);
#line 146
}
#line 146
# 249 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline void FastCC2420TransceiverP__AMPacket__setDestination(message_t *msg, am_addr_t addr)
#line 249
{

  FastCC2420TransceiverP__Ieee154PacketLayer__setDestAddr(msg, addr);
}

# 93 "ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 93
{
  am_group_t myGroup;

#line 95
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 95
    myGroup = ActiveMessageAddressC__group;
#line 95
    __nesc_atomic_end(__nesc_atomic); }
  return myGroup;
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t FastCC2420TransceiverP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 235 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline am_group_t FastCC2420TransceiverP__AMPacket__localGroup(void )
#line 235
{
  return FastCC2420TransceiverP__ActiveMessageAddress__amGroup();
}

# 173 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan)
{
  __nesc_hton_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->destpan.nxdata, pan);
}

# 136 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__setDestPan(message_t *msg, uint16_t pan){
#line 136
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDestPan(msg, pan);
#line 136
}
#line 136
# 277 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static __inline void FastCC2420TransceiverP__AMPacket__setGroup(message_t *msg, am_group_t grp)
#line 277
{

  FastCC2420TransceiverP__Ieee154PacketLayer__setDestPan(msg, grp);
}

# 163 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn)
{
  __nesc_hton_leuint8(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dsn.nxdata, dsn);
}

# 125 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__setDSN(message_t *msg, uint8_t dsn){
#line 125
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setDSN(msg, dsn);
#line 125
}
#line 125
# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(message_t *msg)
{

  __nesc_hton_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata, (__nesc_ntoh_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_PRESERVE)
   | /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_VALUE);
}

# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__createDataFrame(message_t *msg){
#line 69
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__createDataFrame(msg);
#line 69
}
#line 69
#line 63
inline static bool FastCC2420TransceiverP__Ieee154PacketLayer__isDataFrame(message_t *msg){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(msg);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioPacket.nc"
inline static void FastCC2420TransceiverP__SubPacket__setPayloadLength(message_t *msg, uint8_t length){
#line 54
  CC2420XDriverLayerP__RadioPacket__setPayloadLength(msg, length);
#line 54
}
#line 54
# 299 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
#line 299
{

  FastCC2420TransceiverP__SubPacket__setPayloadLength(msg, length + sizeof(activemessage_header_t ) + sizeof(network_header_t ) + sizeof(ieee154_simple_header_t ));
}

#line 336
static inline void FastCC2420TransceiverP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 336
{
  FastCC2420TransceiverP__RadioPacket__setPayloadLength(msg, len);
}

static inline uint8_t FastCC2420TransceiverP__Packet__maxPayloadLength(void )
#line 340
{
  return FastCC2420TransceiverP__RadioPacket__maxPayloadLength();
}

#line 140
static inline error_t FastCC2420TransceiverP__Send__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len)
#line 140
{
  error_t ret;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 143
    {
      if (FastCC2420TransceiverP__pending) {
          {
            unsigned char __nesc_temp = 
#line 145
            EBUSY;

            {
#line 145
              __nesc_atomic_end(__nesc_atomic); 
#line 145
              return __nesc_temp;
            }
          }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 148
  if (len > FastCC2420TransceiverP__Packet__maxPayloadLength()) {
    return ESIZE;
    }
#line 150
  FastCC2420TransceiverP__Packet__setPayloadLength(msg, len);


  if (!FastCC2420TransceiverP__Ieee154PacketLayer__isDataFrame(msg)) {
    FastCC2420TransceiverP__Ieee154PacketLayer__createDataFrame(msg);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    FastCC2420TransceiverP__Ieee154PacketLayer__setDSN(msg, FastCC2420TransceiverP__seqno);
#line 156
    __nesc_atomic_end(__nesc_atomic); }
  FastCC2420TransceiverP__AMPacket__setGroup(msg, FastCC2420TransceiverP__AMPacket__localGroup());
  FastCC2420TransceiverP__AMPacket__setDestination(msg, addr);
  FastCC2420TransceiverP__AMPacket__setSource(msg, FastCC2420TransceiverP__AMPacket__address());
  __nesc_hton_leuint8(FastCC2420TransceiverP__getNetworkHeader(msg)->network.nxdata, 0x3f);
  FastCC2420TransceiverP__AMPacket__setType(msg, id);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 163
    FastCC2420TransceiverP__p_msg = msg;
#line 163
    __nesc_atomic_end(__nesc_atomic); }
  ret = FastCC2420TransceiverP__SubSend__send(msg);
  if (SUCCESS == ret) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 166
        FastCC2420TransceiverP__pending = TRUE;
#line 166
        __nesc_atomic_end(__nesc_atomic); }
    }
  return ret;
}

# 11 "../iMAC_TDMA/async/FastSend.nc"
inline static error_t RadioCountToLedsC__AMSend__send(am_addr_t dest, message_t *msg, uint8_t len){
#line 11
  unsigned char __nesc_result;
#line 11

#line 11
  __nesc_result = FastCC2420TransceiverP__Send__send(AM_RADIO_COUNT_MSG, dest, msg, len);
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 137 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static inline void /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack)
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;

#line 139
  if (ack) {
    (__nesc_temp42 = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) | (1 << IEEE154_FCF_ACK_REQ)));
    }
  else {
#line 142
    (__nesc_temp43 = /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) & ~ (uint16_t )(1 << IEEE154_FCF_ACK_REQ)));
    }
}

# 105 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayer.nc"
inline static void FastCC2420TransceiverP__Ieee154PacketLayer__setAckRequired(message_t *msg, bool ack){
#line 105
  /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__setAckRequired(msg, ack);
#line 105
}
#line 105
# 455 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline void FastCC2420TransceiverP__SoftwareAckConfig__setAckRequired(message_t *msg, bool ack)
{
  FastCC2420TransceiverP__Ieee154PacketLayer__setAckRequired(msg, ack);
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/SoftwareAckConfig.nc"
inline static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__setAckRequired(message_t *msg, bool ack){
#line 49
  FastCC2420TransceiverP__SoftwareAckConfig__setAckRequired(msg, ack);
#line 49
}
#line 49
# 153 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static inline error_t /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__requestAck(message_t *msg)
{
  /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__setAckRequired(msg, TRUE);

  return SUCCESS;
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/PacketAcknowledgements.nc"
inline static error_t RadioCountToLedsC__Acks__requestAck(message_t * msg){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__PacketAcknowledgements__requestAck(msg);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 115 "../iMAC_TDMA/async/FastPacket.nc"
inline static void * RadioCountToLedsC__Packet__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = FastCC2420TransceiverP__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 43 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool RadioCountToLedsC__CCA__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 122 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__MilliTimer__fired(void )
#line 122
{
  bool locked_;

#line 124
  RadioCountToLedsC__counter++;
  ;
  RadioCountToLedsC__UartLog__logEntry(255, RadioCountToLedsC__CCA__get(), 126, RadioCountToLedsC__counter);

  if (TOS_NODE_ID >= 0) {
    return;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    locked_ = RadioCountToLedsC__locked;
#line 131
    __nesc_atomic_end(__nesc_atomic); }
  if (locked_) {
      return;
    }
  else {
      radio_count_msg_t *rcm = (radio_count_msg_t *)RadioCountToLedsC__Packet__getPayload(&RadioCountToLedsC__packet, sizeof(radio_count_msg_t ));

#line 137
      if (rcm == (void *)0) {
          return;
        }

      __nesc_hton_uint16(rcm->counter.nxdata, RadioCountToLedsC__counter);

      RadioCountToLedsC__Acks__requestAck(&RadioCountToLedsC__packet);

      if (RadioCountToLedsC__AMSend__send(28, &RadioCountToLedsC__packet, sizeof(radio_count_msg_t )) == SUCCESS) {
          ;
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 147
            RadioCountToLedsC__locked = TRUE;
#line 147
            __nesc_atomic_end(__nesc_atomic); }
        }
    }
}

# 204 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x409989f0){
#line 83
  switch (arg_0x409989f0) {
#line 83
    case 0U:
#line 83
      RadioCountToLedsC__MilliTimer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x409989f0);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 139 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 91 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 93
        __nesc_atomic_end(__nesc_atomic); 
#line 93
        return __nesc_temp;
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 116 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 116
  unsigned long __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
# 74 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4380 {
#line 57
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__compareControl();
}

# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__resQ[id] != /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__NO_ENTRY || /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(id)) {
        if (/*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qHead == /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qHead = id;
          }
        else {
#line 88
          /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__resQ[/*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qTail] = id;
          }
#line 89
        /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 167 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(uint8_t arg_0x411b2e18){
#line 53
    /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(arg_0x411b2e18);
#line 53
}
#line 53
# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id)
#line 71
{
  /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 73
  {
    if (/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
        /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
        /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__reqResId = id;
        /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
        {
          unsigned char __nesc_temp = 
#line 78
          SUCCESS;

#line 78
          return __nesc_temp;
        }
      }
#line 80
    {
      unsigned char __nesc_temp = 
#line 80
      /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Queue__enqueue(id);

#line 80
      return __nesc_temp;
    }
  }
}

# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t FastCC2420TransceiverP__Resource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__request(0U);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 169 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 169
{
}

# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x411b2e18){
#line 61
    /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x411b2e18);
#line 61
}
#line 61
# 84 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 84
{
  /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceRequested__immediateRequested(/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 86
  {
    if (/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__state == /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
        /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__state = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__RES_BUSY;
        /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId = id;
        /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__resId);
        {
          unsigned char __nesc_temp = 
#line 91
          SUCCESS;

#line 91
          return __nesc_temp;
        }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      FAIL;

#line 93
      return __nesc_temp;
    }
  }
}

# 97 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t FastCC2420TransceiverP__Resource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__Resource__immediateRequest(0U);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 76 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static inline error_t FastCC2420TransceiverP__Init__init(void )
#line 76
{
  FastCC2420TransceiverP__seqno = TOS_NODE_ID;
  if (FastCC2420TransceiverP__Resource__immediateRequest() != SUCCESS) {
    FastCC2420TransceiverP__Resource__request();
    }
#line 80
  return SUCCESS;
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4381 {
#line 57
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 173 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 173
{
#line 173
  return FAIL;
}

# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x40bfc068){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x40bfc068) {
#line 88
    case /*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 88
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x40bfc068);
#line 88
      break;
#line 88
    }
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 108 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 108
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420XDriverLayerP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*HplCC2420XC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt)
{
  /* atomic removed: atomic calls only */
  {


    /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type t0 = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get();

    if (dt > /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE) 
      {
        dt -= /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
        while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
        t0 += dt;
        dt = /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE;
      }

    while (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get() - t0 <= dt) ;
  }
}

# 66 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/BusyWait.nc"
inline static void CC2420XDriverLayerP__BusyWait__wait(CC2420XDriverLayerP__BusyWait__size_type dt){
#line 66
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(dt);
#line 66
}
#line 66
# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )29U |= 0x01 << 5;
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 48
}
#line 48
# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 40 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__VREN__set(void ){
#line 40
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 40
}
#line 40
# 102 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 102
{
#line 102
  P1IFG &= ~(1 << 0);
}

# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port10__clear();
#line 52
}
#line 52
# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 94
{
#line 94
  P1IE &= ~(1 << 0);
}

# 47 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port10__disable();
#line 47
}
#line 47
# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420XDriverLayerP__FifopInterrupt__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 0);
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__makeInput();
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__FIFOP__makeInput(void ){
#line 44
  /*HplCC2420XC.FIFOPM*/Msp430GpioC__6__GeneralIO__makeInput();
#line 44
}
#line 44
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 3);
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.FIFOM*/Msp430GpioC__5__HplGeneralIO__makeInput();
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__FIFO__makeInput(void ){
#line 44
  /*HplCC2420XC.FIFOM*/Msp430GpioC__5__GeneralIO__makeInput();
#line 44
}
#line 44
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__SFD__makeInput(void ){
#line 44
  /*HplCC2420XC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 44
}
#line 44
# 61 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 78
}
#line 78
# 52 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420XC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 44 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__CCA__makeInput(void ){
#line 44
  /*HplCC2420XC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 44
}
#line 44
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__RSTN__makeOutput(void ){
#line 46
  /*HplCC2420XC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 46 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420XDriverLayerP__VREN__makeOutput(void ){
#line 46
  /*HplCC2420XC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 46
}
#line 46
# 424 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline error_t CC2420XDriverLayerP__SoftwareInit__init(void )
{





  CC2420XDriverLayerP__CSN__makeOutput();
  CC2420XDriverLayerP__VREN__makeOutput();
  CC2420XDriverLayerP__RSTN__makeOutput();
  CC2420XDriverLayerP__CCA__makeInput();
  CC2420XDriverLayerP__SFD__makeInput();
  CC2420XDriverLayerP__FIFO__makeInput();
  CC2420XDriverLayerP__FIFOP__makeInput();

  CC2420XDriverLayerP__FifopInterrupt__disable();
  CC2420XDriverLayerP__SfdCapture__disable();
  CC2420XDriverLayerP__sfd_status = 0;


  CC2420XDriverLayerP__CSN__set();


  CC2420XDriverLayerP__VREN__set();
  CC2420XDriverLayerP__BusyWait__wait(600);


  CC2420XDriverLayerP__RSTN__clr();
  CC2420XDriverLayerP__RSTN__set();

  CC2420XDriverLayerP__rxMsg = &CC2420XDriverLayerP__rxMsgBuffer;

  CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_VR_ON;
#line 473
  return CC2420XDriverLayerP__SpiResource__request();
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 56 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static inline error_t UartLogP__Init__init(void )
#line 56
{
  UartLogP__UartControl__start();
  /* atomic removed: atomic calls only */
#line 58
  UartLogP__seqno = 0;
  UartLogP__init_cnt++;


  return SUCCESS;
}

# 216 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static __inline void SerialP__ackInit(void )
#line 216
{
  SerialP__ackQ.writePtr = SerialP__ackQ.readPtr = 0;
}

#line 207
static __inline void SerialP__rxInit(void )
#line 207
{
  SerialP__rxBuf.writePtr = SerialP__rxBuf.readPtr = 0;
  SerialP__rxState = SerialP__RXSTATE_INACTIVE;
  SerialP__rxByteCnt = 0;
  SerialP__rxProto = 0;
  SerialP__rxSeqno = 0;
  SerialP__rxCRC = 0;
}

#line 195
static __inline void SerialP__txInit(void )
#line 195
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 197
  for (i = 0; i < SerialP__TX_BUFFER_COUNT; i++) SerialP__txBuf[i].state = SerialP__BUFFER_AVAILABLE;
  SerialP__txState = SerialP__TXSTATE_INACTIVE;
  SerialP__txByteCnt = 0;
  SerialP__txProto = 0;
  SerialP__txSeqno = 0;
  SerialP__txCRC = 0;
  SerialP__txPending = FALSE;
  SerialP__txIndex = 0;
}

#line 220
static inline error_t SerialP__Init__init(void )
#line 220
{

  SerialP__txInit();
  SerialP__rxInit();
  SerialP__ackInit();

  return SUCCESS;
}

# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

#line 55
static inline error_t /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__Init__init(void )
#line 55
{
  memset(/*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__resQ, /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__NO_ENTRY, sizeof /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__resQ);
  return SUCCESS;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*FastCC2420TransceiverC.SendResourceC.Queue*/FcfsResourceQueueC__2__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, SerialP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UartLogP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420XDriverLayerP__SoftwareInit__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, FastCC2420TransceiverP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 105 "RadioCountToLedsC.nc"
static inline void RadioCountToLedsC__Boot__booted(void )
#line 105
{
  RadioCountToLedsC__AMControl__start();
}

# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  RadioCountToLedsC__Boot__booted();
#line 60
}
#line 60
# 391 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

#line 379
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 60 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 60
{
  return MSP430_POWER_LPM3;
}

# 140 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled(void )
{
  return * (volatile uint16_t * )390U & 0x0010;
}

# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__areEventsEnabled(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__areEventsEnabled();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline bool /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning(void )
{
  return /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Msp430TimerControl__areEventsEnabled();
}

# 88 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static bool Msp430HybridAlarmCounterP__AlarmMicro__isRunning(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430HybridAlarmCounterC.AlarmMicro16C.Msp430Alarm*/Msp430AlarmC__3__Alarm__isRunning();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 260 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430HybridAlarmCounterP.nc"
static inline mcu_power_t Msp430HybridAlarmCounterP__McuPowerOverride__lowestState(void )
#line 260
{
  if (Msp430HybridAlarmCounterP__AlarmMicro__isRunning()) {
    return MSP430_POWER_LPM0;
    }
  else {
#line 264
    return MSP430_POWER_LPM3;
    }
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430HybridAlarmCounterP__McuPowerOverride__lowestState();
#line 62
  __nesc_result = mcombine(__nesc_result, Msp430ClockP__McuPowerOverride__lowestState());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 79
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 82
  TBCCTL0 & 0x0010 || 
  TBCCTL1 & 0x0010) || 
  TBCCTL2 & 0x0010) && (
  TBCTL & 0x0300) == 0x0200) || (
  ME1 & (0x80 | 0x40) && U0TCTL & 0x20)) || (
  ME2 & (0x20 | 0x10) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 104
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 105
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

static inline void McuSleepC__computePowerState(void )
#line 117
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 122
{
  uint16_t temp;

#line 124
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 98 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 349 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~0x40;
}

# 177 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 182 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 182
{

  uint8_t end;
  uint8_t tmp;

  /* atomic removed: atomic calls only */
#line 187
  {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);

    end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos + /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SPI_ATOMIC_SIZE;
    if (end > /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
      end = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len;
      }
    while (++/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < end) {
        while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
        tmp = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();
        if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = tmp;
          }
#line 199
        /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
      }
  }
}

#line 231
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 231
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 238
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x40caf108, uint8_t data){
#line 54
  switch (arg_0x40caf108) {
#line 54
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x40caf108, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 66 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0x40caf988){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0x40caf988);
#line 39
}
#line 39
# 59 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 249 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 249
{
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x40caf108){
#line 49
  switch (arg_0x40caf108) {
#line 49
    case /*HplCC2420XC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x40caf108);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 1212 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static inline void CC2420XDriverLayerP__FifopInterrupt__fired(void )
{
}

# 68 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 68
  CC2420XDriverLayerP__FifopInterrupt__fired();
#line 68
}
#line 68
# 77 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__Interrupt__fired();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 72
  /*HplCC2420XC.FifopInterruptC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 72
}
#line 72
# 103 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 103
{
#line 103
  P1IFG &= ~(1 << 1);
}

#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 79
{
#line 79
  HplMsp430InterruptP__Port11__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 72
  HplMsp430InterruptP__Port11__default__fired();
#line 72
}
#line 72
# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 104
{
#line 104
  P1IFG &= ~(1 << 2);
}

#line 80
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 80
{
#line 80
  HplMsp430InterruptP__Port12__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 72
  HplMsp430InterruptP__Port12__default__fired();
#line 72
}
#line 72
# 105 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 105
{
#line 105
  P1IFG &= ~(1 << 3);
}

#line 81
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 81
{
#line 81
  HplMsp430InterruptP__Port13__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 72
  HplMsp430InterruptP__Port13__default__fired();
#line 72
}
#line 72
# 106 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 106
{
#line 106
  P1IFG &= ~(1 << 4);
}

#line 82
static inline void HplMsp430InterruptP__Port14__default__fired(void )
#line 82
{
#line 82
  HplMsp430InterruptP__Port14__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 72
  HplMsp430InterruptP__Port14__default__fired();
#line 72
}
#line 72
# 107 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 107
{
#line 107
  P1IFG &= ~(1 << 5);
}

#line 83
static inline void HplMsp430InterruptP__Port15__default__fired(void )
#line 83
{
#line 83
  HplMsp430InterruptP__Port15__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 72
  HplMsp430InterruptP__Port15__default__fired();
#line 72
}
#line 72
# 108 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 108
{
#line 108
  P1IFG &= ~(1 << 6);
}

#line 84
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 84
{
#line 84
  HplMsp430InterruptP__Port16__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 72
  HplMsp430InterruptP__Port16__default__fired();
#line 72
}
#line 72
# 109 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 109
{
#line 109
  P1IFG &= ~(1 << 7);
}

#line 85
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 85
{
#line 85
  HplMsp430InterruptP__Port17__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 72
  HplMsp430InterruptP__Port17__default__fired();
#line 72
}
#line 72
# 206 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 206
{
#line 206
  P2IFG &= ~(1 << 0);
}

#line 182
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 182
{
#line 182
  HplMsp430InterruptP__Port20__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 72
  HplMsp430InterruptP__Port20__default__fired();
#line 72
}
#line 72
# 207 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 207
{
#line 207
  P2IFG &= ~(1 << 1);
}

#line 183
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 183
{
#line 183
  HplMsp430InterruptP__Port21__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 72
  HplMsp430InterruptP__Port21__default__fired();
#line 72
}
#line 72
# 208 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 208
{
#line 208
  P2IFG &= ~(1 << 2);
}

#line 184
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 184
{
#line 184
  HplMsp430InterruptP__Port22__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 72
  HplMsp430InterruptP__Port22__default__fired();
#line 72
}
#line 72
# 209 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 209
{
#line 209
  P2IFG &= ~(1 << 3);
}

#line 185
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 185
{
#line 185
  HplMsp430InterruptP__Port23__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 72
  HplMsp430InterruptP__Port23__default__fired();
#line 72
}
#line 72
# 210 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 210
{
#line 210
  P2IFG &= ~(1 << 4);
}

#line 186
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 186
{
#line 186
  HplMsp430InterruptP__Port24__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 72
  HplMsp430InterruptP__Port24__default__fired();
#line 72
}
#line 72
# 211 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 211
{
#line 211
  P2IFG &= ~(1 << 5);
}

#line 187
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 187
{
#line 187
  HplMsp430InterruptP__Port25__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 72
  HplMsp430InterruptP__Port25__default__fired();
#line 72
}
#line 72
# 212 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 212
{
#line 212
  P2IFG &= ~(1 << 6);
}

#line 188
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 188
{
#line 188
  HplMsp430InterruptP__Port26__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 72
  HplMsp430InterruptP__Port26__default__fired();
#line 72
}
#line 72
# 213 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 213
{
#line 213
  P2IFG &= ~(1 << 7);
}

#line 189
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 189
{
#line 189
  HplMsp430InterruptP__Port27__clear();
}

# 72 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 72
  HplMsp430InterruptP__Port27__default__fired();
#line 72
}
#line 72
# 98 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 397 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__dataReceived(uint8_t data)
#line 397
{
  SerialP__rx_state_machine(FALSE, data);
}

# 94 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__dataReceived(uint8_t data){
#line 94
  SerialP__SerialFrameComm__dataReceived(data);
#line 94
}
#line 94
# 394 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__delimiterReceived(void )
#line 394
{
  SerialP__rx_state_machine(TRUE, 0);
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__delimiterReceived(void ){
#line 85
  SerialP__SerialFrameComm__delimiterReceived();
#line 85
}
#line 85
# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receivedByte(uint8_t data)
#line 73
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC__SerialFrameComm__delimiterReceived();
      return;
    }
  else {
#line 85
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC__state.receiveEscape = 1;
        return;
      }
    else {
#line 90
      if (HdlcTranslateC__state.receiveEscape) {

          HdlcTranslateC__state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 95
  HdlcTranslateC__SerialFrameComm__dataReceived(data);
}

# 221 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(uint8_t id, uint8_t byte)
#line 221
{
}

# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(uint8_t arg_0x41002c58, uint8_t byte){
#line 79
  switch (arg_0x41002c58) {
#line 79
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 79
      HdlcTranslateC__UartStream__receivedByte(byte);
#line 79
      break;
#line 79
    default:
#line 79
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receivedByte(arg_0x41002c58, byte);
#line 79
      break;
#line 79
    }
#line 79
}
#line 79
# 132 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__UartStream__receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 132
{
}

# 222 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 222
{
}

# 99 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(uint8_t arg_0x41002c58, uint8_t * buf, uint16_t len, error_t error){
#line 99
  switch (arg_0x41002c58) {
#line 99
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 99
      HdlcTranslateC__UartStream__receiveDone(buf, len, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__receiveDone(arg_0x41002c58, buf, len, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 134 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(uint8_t id, uint8_t data)
#line 134
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf) {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos++] = data;
      if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_pos >= /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len) {
          uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf;

#line 139
          /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_buf = (void *)0;
          /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receiveDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_rx_len, SUCCESS);
        }
    }
  else 
#line 142
    {
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__receivedByte(id, data);
    }
}

# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(uint8_t arg_0x40caf108, uint8_t data){
#line 54
  switch (arg_0x40caf108) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__rxDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID, data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__rxDone(arg_0x40caf108, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 401 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline bool SerialP__valid_rx_proto(uint8_t proto)
#line 401
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 203 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer(void )
#line 203
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked = 1;
    }
}

#line 199
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked(void )
#line 199
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufOneLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.bufZeroLocked;
}

#line 226
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket(void )
#line 226
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 228
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP__ReceiveBytePacket__startPacket(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__startPacket();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 311 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP__rx_current_crc(void )
#line 311
{
  uint16_t crc;
  uint8_t tmp = SerialP__rxBuf.writePtr;

#line 314
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP__rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP__RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP__rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__endPacket(error_t result){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(result);
#line 80
}
#line 80
# 221 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap(void )
#line 221
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which];
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 234 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static __inline bool SerialP__ack_queue_is_full(void )
#line 234
{
  uint8_t tmp;
#line 235
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 236
  {
    tmp = SerialP__ackQ.writePtr;
    tmp2 = SerialP__ackQ.readPtr;
  }
  if (++tmp > SerialP__ACK_QUEUE_SIZE) {
#line 240
    tmp = 0;
    }
#line 241
  return tmp == tmp2;
}







static __inline void SerialP__ack_queue_push(uint8_t token)
#line 250
{
  if (!SerialP__ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 252
      {
        SerialP__ackQ.buf[SerialP__ackQ.writePtr] = token;
        if (++ SerialP__ackQ.writePtr > SerialP__ACK_QUEUE_SIZE) {
#line 254
          SerialP__ackQ.writePtr = 0;
          }
      }
#line 256
      SerialP__MaybeScheduleTx();
    }
}

# 67 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC__SerialFrameComm__resetReceive(void )
#line 67
{
  HdlcTranslateC__state.receiveEscape = 0;
}

# 79 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP__SerialFrameComm__resetReceive(void ){
#line 79
  HdlcTranslateC__SerialFrameComm__resetReceive();
#line 79
}
#line 79
# 244 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(uint8_t b)
#line 244
{
  /* atomic removed: atomic calls only */
#line 245
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__PacketInfo__offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE: 
          default: 
#line 266
            ;
      }
  }
}

# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP__ReceiveBytePacket__byteReceived(uint8_t data){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__byteReceived(data);
#line 69
}
#line 69
# 301 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP__rx_buffer_top(void )
#line 301
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 303
  return tmp;
}

#line 305
static __inline uint8_t SerialP__rx_buffer_pop(void )
#line 305
{
  uint8_t tmp = SerialP__rxBuf.buf[SerialP__rxBuf.readPtr];

#line 307
  if (++ SerialP__rxBuf.readPtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 307
    SerialP__rxBuf.readPtr = 0;
    }
#line 308
  return tmp;
}

#line 297
static __inline void SerialP__rx_buffer_push(uint8_t data)
#line 297
{
  SerialP__rxBuf.buf[SerialP__rxBuf.writePtr] = data;
  if (++ SerialP__rxBuf.writePtr > SerialP__RX_DATA_BUFFER_SIZE) {
#line 299
    SerialP__rxBuf.writePtr = 0;
    }
}

# 220 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(uint8_t id, uint8_t *buf, uint16_t len, error_t error)
#line 220
{
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(uint8_t arg_0x41002c58, uint8_t * buf, uint16_t len, error_t error){
#line 57
  switch (arg_0x41002c58) {
#line 57
    case /*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID:
#line 57
      HdlcTranslateC__UartStream__sendDone(buf, len, error);
#line 57
      break;
#line 57
    default:
#line 57
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__default__sendDone(arg_0x41002c58, buf, len, error);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 384 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P__Usart__tx(uint8_t data)
#line 384
{
  HplMsp430Usart1P__U1TXBUF = data;
}

# 224 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart1P__Usart__tx(data);
#line 224
}
#line 224
# 162 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(uint8_t id)
#line 162
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner != id) {
      uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 165
      /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, FAIL);
    }
  else {
#line 168
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos < /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len) {
        /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
      }
    else {
        uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf;

#line 173
        /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = (void *)0;
        /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__sendDone(id, buf, /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len, SUCCESS);
      }
    }
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(uint8_t arg_0x40caf108){
#line 49
  switch (arg_0x40caf108) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C__0__CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP__0__UsartInterrupts__txDone(/*PlatformSerialC.UartC*/Msp430Uart1C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__default__txDone(arg_0x40caf108);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__Interrupts__txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__ArbiterInfo__userId());
    }
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP__1__RawInterrupts__txDone();
#line 49
}
#line 49
# 65 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP__SerialFrameComm__putData(uint8_t data){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = HdlcTranslateC__SerialFrameComm__putData(data);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 529 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline error_t SerialP__SendBytePacket__completeSend(void )
#line 529
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 531
  {
    SerialP__txBuf[SerialP__TX_DATA_INDEX].state = SerialP__BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 71 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = SerialP__SendBytePacket__completeSend();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 178 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte(void )
#line 178
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 181
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP__SendBytePacket__nextByte(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__SendBytePacket__nextByte();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 668 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static inline void SerialP__SerialFrameComm__putDone(void )
#line 668
{
  {
    error_t txResult = SUCCESS;

    /* atomic removed: atomic calls only */
#line 671
    {
      switch (SerialP__txState) {

          case SerialP__TXSTATE_PROTO: 

            txResult = SerialP__SerialFrameComm__putData(SerialP__txProto);

          SerialP__txState = SerialP__TXSTATE_INFO;



          SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txProto);
          break;

          case SerialP__TXSTATE_SEQNO: 
            txResult = SerialP__SerialFrameComm__putData(SerialP__txSeqno);
          SerialP__txState = SerialP__TXSTATE_INFO;
          SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txSeqno);
          break;

          case SerialP__TXSTATE_INFO: 
            {
              txResult = SerialP__SerialFrameComm__putData(SerialP__txBuf[SerialP__txIndex].buf);
              SerialP__txCRC = crcByte(SerialP__txCRC, SerialP__txBuf[SerialP__txIndex].buf);
              ++SerialP__txByteCnt;

              if (SerialP__txIndex == SerialP__TX_DATA_INDEX) {
                  uint8_t nextByte;

#line 699
                  nextByte = SerialP__SendBytePacket__nextByte();
                  if (SerialP__txBuf[SerialP__txIndex].state == SerialP__BUFFER_COMPLETE || SerialP__txByteCnt >= SerialP__SERIAL_MTU) {
                      SerialP__txState = SerialP__TXSTATE_FCS1;
                    }
                  else {
                      SerialP__txBuf[SerialP__txIndex].buf = nextByte;
                    }
                }
              else {
                  SerialP__txState = SerialP__TXSTATE_FCS1;
                }
            }
          break;

          case SerialP__TXSTATE_FCS1: 
            txResult = SerialP__SerialFrameComm__putData(SerialP__txCRC & 0xff);
          SerialP__txState = SerialP__TXSTATE_FCS2;
          break;

          case SerialP__TXSTATE_FCS2: 
            txResult = SerialP__SerialFrameComm__putData((SerialP__txCRC >> 8) & 0xff);
          SerialP__txState = SerialP__TXSTATE_ENDFLAG;
          break;

          case SerialP__TXSTATE_ENDFLAG: 
            txResult = SerialP__SerialFrameComm__putDelimiter();
          SerialP__txState = SerialP__TXSTATE_ENDWAIT;
          break;

          case SerialP__TXSTATE_ENDWAIT: 
            SerialP__txState = SerialP__TXSTATE_FINISH;
          case SerialP__TXSTATE_FINISH: 
            SerialP__MaybeScheduleTx();
          break;
          case SerialP__TXSTATE_ERROR: 
            default: 
              txResult = FAIL;
          break;
        }

      if (txResult != SUCCESS) {
          SerialP__txState = SerialP__TXSTATE_ERROR;
          SerialP__MaybeScheduleTx();
        }
    }
  }
}

# 100 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC__SerialFrameComm__putDone(void ){
#line 100
  SerialP__SerialFrameComm__putDone();
#line 100
}
#line 100
# 411 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 48 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*Msp430HybridAlarmCounterC.Alarm32khz16C.Msp430Alarm*/Msp430AlarmC__2__Msp430Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 180 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

# 170 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 107 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 62 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 69
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 72
        do {
#line 72
            t0 = t1;
#line 72
            t1 = * (volatile uint16_t * )368U;
          }
        while (
#line 72
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 73
          t1;

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )368U;
    }
}

# 180 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000A)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x001A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4068d788){
#line 39
  switch (arg_0x4068d788) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4068d788);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 112 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/TaskletC.nc"
static void TaskletC__Tasklet__schedule(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      if (TaskletC__state != 0) 
        {
          TaskletC__state |= 0x80;
          {
#line 119
            __nesc_atomic_end(__nesc_atomic); 
#line 119
            return;
          }
        }
      TaskletC__state = 1;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }

  TaskletC__doit();
}

# 56 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

#line 57
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 49 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 49
{
  /* atomic removed: atomic calls only */
#line 50
  {
    /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 173 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static void FastCC2420TransceiverP__SubSend__sendDone(error_t error)
#line 173
{
  message_t *p_msg_;

#line 175
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 175
    {
      FastCC2420TransceiverP__pending = FALSE;
      p_msg_ = FastCC2420TransceiverP__p_msg;
    }
#line 178
    __nesc_atomic_end(__nesc_atomic); }

  FastCC2420TransceiverP__Send__sendDone(FastCC2420TransceiverP__AMPacket__type(p_msg_), p_msg_, error);
}

# 220 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static error_t UartLogP__UartLog__logEntry(__nesc_nxbase_nx_uint8_t type, __nesc_nxbase_nx_uint8_t sourceId, __nesc_nxbase_nx_uint16_t seq, __nesc_nxbase_nx_uint32_t timestamp)
#line 220
{



  logMsg log_msg;
  logMsg *log_msg_p = &log_msg;

#line 226
  __nesc_hton_uint8(log_msg_p->type.nxdata, type);
  __nesc_hton_uint8(log_msg_p->nodeId.nxdata, TOS_NODE_ID);
  __nesc_hton_uint8(log_msg_p->sourceId.nxdata, sourceId);
  __nesc_hton_uint16(log_msg_p->seq.nxdata, seq);
  __nesc_hton_uint8(log_msg_p->last_hop.nxdata, 0xFF);
  __nesc_hton_uint16(log_msg_p->last_hop_ntw_seq.nxdata, 0xFFFF);
  __nesc_hton_uint16(log_msg_p->last_hop_seq.nxdata, 0xFFFF);
  __nesc_hton_uint16(log_msg_p->local_ntw_seq.nxdata, 0xFFFF);
  __nesc_hton_uint16(log_msg_p->local_seq.nxdata, 0xFFFF);

  __nesc_hton_uint32(log_msg_p->timestamp.nxdata, timestamp);


  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 239
    __nesc_hton_uint32(log_msg_p->seqno.nxdata, UartLogP__seqno++);
#line 239
    __nesc_atomic_end(__nesc_atomic); }
  return UartLogP__UartLog__logByteStream((nx_uint8_t *)log_msg_p, sizeof(logMsg ));
}

# 315 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static flags_metadata_t *FastCC2420TransceiverP__getMeta(message_t *msg)
#line 315
{
  return (void *)msg + sizeof(message_t ) - FastCC2420TransceiverP__SubPacket__metadataLength(msg) - sizeof(flags_metadata_t );
}

#line 86
static activemessage_header_t *FastCC2420TransceiverP__getHeader(message_t *msg)
#line 86
{

  return (void *)msg + FastCC2420TransceiverP__SubPacket__headerLength(msg) + sizeof(ieee154_simple_header_t ) + sizeof(network_header_t );
}

# 531 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static bool CC2420XDriverLayerP__isSpiAcquired(void )
{
  if (CC2420XDriverLayerP__SpiResource__isOwner()) {
    return TRUE;
    }
  if (CC2420XDriverLayerP__SpiResource__immediateRequest() == SUCCESS) 
    {
      CC2420XDriverLayerP__CSN__makeOutput();
      CC2420XDriverLayerP__CSN__set();

      return TRUE;
    }

  CC2420XDriverLayerP__SpiResource__request();
  return FALSE;
}

# 133 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 134
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 138
                SUCCESS;

                {
#line 138
                  __nesc_atomic_end(__nesc_atomic); 
#line 138
                  return __nesc_temp;
                }
              }
            }
          else {
#line 140
            if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
                /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
                /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 143
                  SUCCESS;

                  {
#line 143
                    __nesc_atomic_end(__nesc_atomic); 
#line 143
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 149
    __nesc_atomic_end(__nesc_atomic); }
#line 147
  return FAIL;
}

# 265 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 63
{
#line 63
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 63
    * (volatile uint8_t * )30U |= 0x01 << 2;
#line 63
    __nesc_atomic_end(__nesc_atomic); }
}

# 77 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
        }
      else {
#line 84
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId == id) {
            {
              unsigned char __nesc_temp = 
#line 85
              SUCCESS;

              {
#line 85
                __nesc_atomic_end(__nesc_atomic); 
#line 85
                return __nesc_temp;
              }
            }
          }
        else 
#line 87
          {
            unsigned char __nesc_temp = 
#line 87
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

            {
#line 87
              __nesc_atomic_end(__nesc_atomic); 
#line 87
              return __nesc_temp;
            }
          }
        }
    }
#line 91
    __nesc_atomic_end(__nesc_atomic); }
#line 89
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 69 "../iMAC_TDMA/cc2420x/SoftwareAckLayerC.nc"
static void /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__SubSend__sendDone(error_t error)
{
  if (/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state == /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_ACK_SEND) 
    {

      for (; 0; ) ;

      /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY;
    }
  else 
    {
      for (; 0; ) ;
      for (; 0; ) ;

      if (error == SUCCESS && /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__requiresAckWait(/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__txMsg) && /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__isFree()) 
        {
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioAlarm__wait(/*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__Config__getAckTimeout());
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_ACK_WAIT;
        }
      else 
        {
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__state = /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__STATE_READY;
          /*FastCC2420TransceiverC.AckLayer*/SoftwareAckLayerC__0__RadioSend__sendDone(error);
        }
    }
}

# 132 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getAckRequired(message_t *msg)
{
  return __nesc_ntoh_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ) ? TRUE : FALSE;
}

#line 92
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isDataFrame(message_t *msg)
{
  return (__nesc_ntoh_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_MASK) == /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_DATA_FRAME_VALUE;
}

#line 178
static uint16_t /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__getDestAddr(message_t *msg)
{
  return __nesc_ntoh_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->dest.nxdata);
}

# 81 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static void /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      uint16_t now = /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
      uint16_t elapsed = now - t0;

#line 87
      if (elapsed >= dt) 
        {
          /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
        }
      else 
        {
          uint16_t remaining = dt - elapsed;

#line 94
          if (remaining <= 2) {
            /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
            }
          else {
#line 97
            /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
            }
        }
#line 99
      /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
      /*HplCC2420XC.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
    }
#line 101
    __nesc_atomic_end(__nesc_atomic); }
}

# 104 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/layers/Ieee154PacketLayerP.nc"
static bool /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__Ieee154PacketLayer__isAckFrame(message_t *msg)
{
  return (__nesc_ntoh_leuint16(/*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__getHeader(msg)->fcf.nxdata) & /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_MASK) == /*FastCC2420TransceiverC.Ieee154PacketLayerC.Ieee154PacketLayerP*/Ieee154PacketLayerP__0__IEEE154_ACK_FRAME_VALUE;
}

# 112 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static cc2420x_metadata_t *CC2420XDriverLayerP__getMeta(message_t *msg)
{
  return (void *)msg + sizeof(message_t ) - CC2420XDriverLayerP__RadioPacket__metadataLength(msg);
}

# 80 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformCounterC.nc"
static /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__Counter__get(void )
{
  /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type rv = 0;

  /* atomic removed: atomic calls only */
#line 84
  {
    /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__upper_count_type high = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__m_upper;
    /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__from_size_type low = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get();

#line 87
    if (/*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
      {






        high++;
        low = /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__CounterFrom__get();
      }
    {
      /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type high_to = high;
      /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__to_size_type low_to = low >> /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 101
      rv = (high_to << /*LocalTimeHybridMicroC.TransformCounterC*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
    }
  }
  return rv;
}

# 402 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static timestamp_metadata_t *FastCC2420TransceiverP__getMetaTimestamp(message_t *msg)
#line 402
{
  return (void *)msg + sizeof(message_t ) - FastCC2420TransceiverP__SubPacket__metadataLength(msg) - sizeof(flags_metadata_t ) - sizeof(timestamp_metadata_t );
}

# 1392 "../iMAC_TDMA/cc2420x/CC2420XDriverLayerP.nc"
static void CC2420XDriverLayerP__RadioPacket__setPayloadLength(message_t *msg, uint8_t length)
{
  for (; 0; ) ;
  for (; 0; ) ;


  __nesc_hton_leuint8(CC2420XDriverLayerP__getHeader(msg)->length.nxdata, length + 2);
}

#line 733
static error_t CC2420XDriverLayerP__RadioSend__send(message_t *msg)
{
  uint16_t time;
  uint8_t p;
  uint8_t length;
  uint8_t *data;
  uint8_t header;
  uint32_t time32;
  void *timesync;
  timesync_relative_t timesync_relative;
  uint32_t sfdTime;




  if ((((CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE || (CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_RX_ON)) || !CC2420XDriverLayerP__isSpiAcquired()) || CC2420XDriverLayerP__rxSfd) || CC2420XDriverLayerP__txEnd) {
    return EBUSY;
    }
  p = (CC2420XDriverLayerP__PacketTransmitPower__isSet(msg) ? 
  CC2420XDriverLayerP__PacketTransmitPower__get(msg) : 3) & CC2420X_TX_PWR_MASK;

  if (p != CC2420XDriverLayerP__txPower) 
    {
      cc2420X_txctrl_t txctrl = cc2420X_txctrl_default;

      CC2420XDriverLayerP__txPower = p;

      txctrl.f.pa_level = CC2420XDriverLayerP__txPower;
      CC2420XDriverLayerP__writeRegister(CC2420X_TXCTRL, txctrl.value);
    }

  if (CC2420XDriverLayerP__Config__requiresRssiCca(msg) && !CC2420XDriverLayerP__CCA__get()) {
    return EBUSY;
    }
  data = CC2420XDriverLayerP__getPayload(msg);
  length = __nesc_ntoh_leuint8(CC2420XDriverLayerP__getHeader(msg)->length.nxdata);



  header = CC2420XDriverLayerP__Config__headerPreloadLength();
  if (header > length) {
    header = length;
    }
  length -= header;


  CC2420XDriverLayerP__writeTxFifo(data, header);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 781
    {


      if ((CC2420XDriverLayerP__cmd != CC2420XDriverLayerP__CMD_NONE || (CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_IDLE && CC2420XDriverLayerP__state != CC2420XDriverLayerP__STATE_RX_ON)) || CC2420XDriverLayerP__rxSfd) 


        {

          CC2420XDriverLayerP__strobe(CC2420X_SFLUSHTX);


          {
            unsigned char __nesc_temp = 
#line 792
            EBUSY;

            {
#line 792
              __nesc_atomic_end(__nesc_atomic); 
#line 792
              return __nesc_temp;
            }
          }
        }


      CC2420XDriverLayerP__strobe(CC2420X_STXON);



      time = CC2420XDriverLayerP__RadioAlarm__getNow();

      CC2420XDriverLayerP__cmd = CC2420XDriverLayerP__CMD_TRANSMIT;
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_TX_ON;


      CC2420XDriverLayerP__SfdCapture__captureFallingEdge();
      CC2420XDriverLayerP__sfd_status = 1;
    }
#line 810
    __nesc_atomic_end(__nesc_atomic); }







  timesync = CC2420XDriverLayerP__PacketTimeSyncOffset__isSet(msg) ? (void *)msg + CC2420XDriverLayerP__PacketTimeSyncOffset__get(msg) : 0;

  if (timesync == 0) {

      if (length > 0) {
        CC2420XDriverLayerP__writeTxFifo(data + header, length - 1);
        }
#line 824
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON;
    }
  else 
#line 825
    {


      CC2420XDriverLayerP__writeTxFifo(data + header, length - sizeof timesync_relative - 1);
    }



  sfdTime = time;



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 837
    {
      time = CC2420XDriverLayerP__RadioAlarm__getNow();
      time32 = CC2420XDriverLayerP__LocalTime__get();
    }
#line 840
    __nesc_atomic_end(__nesc_atomic); }


  time -= sfdTime;
  time32 -= time;


  time32 += TX_SFD_DELAY;

  CC2420XDriverLayerP__PacketTimeStamp__set(msg, time32);

  if (timesync != 0) {

      __nesc_hton_int32(timesync_relative.nxdata, __nesc_ntoh_uint32((* (timesync_absolute_t *)timesync).nxdata) - time32);



      CC2420XDriverLayerP__writeTxFifo((uint8_t *)&timesync_relative, sizeof timesync_relative);
      CC2420XDriverLayerP__state = CC2420XDriverLayerP__STATE_BUSY_TX_2_RX_ON;
    }
#line 878
  return SUCCESS;
}

#line 1387
static uint8_t CC2420XDriverLayerP__RadioPacket__payloadLength(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420XDriverLayerP__getHeader(msg)->length.nxdata) - 2;
}

# 344 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static void *FastCC2420TransceiverP__Packet__getPayload(message_t *msg, uint8_t len)
#line 344
{
  if (len > FastCC2420TransceiverP__RadioPacket__maxPayloadLength()) {
    return (void *)0;
    }
  return (void *)msg + FastCC2420TransceiverP__RadioPacket__headerLength(msg);
}

#line 304
static uint8_t FastCC2420TransceiverP__RadioPacket__maxPayloadLength(void )
#line 304
{

  return FastCC2420TransceiverP__SubPacket__maxPayloadLength() - sizeof(activemessage_header_t ) - sizeof(network_header_t ) - sizeof(ieee154_simple_header_t );
}

# 50 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/rfxlink/util/RadioAlarm.nc"
static void CC2420XDriverLayerP__RadioAlarm__wait(tradio_size timeout){
#line 50
  /*FastCC2420TransceiverC.RadioAlarmC.RadioAlarmP*/RadioAlarmP__0__RadioAlarm__wait(0U, timeout);
#line 50
}
#line 50
# 69 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/GpioCaptureC.nc"
static void /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Capture__disable(void )
#line 69
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 70
    {
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
      /*HplCC2420XC.GpioCaptureC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
}

# 57 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 14 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 172 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosa/chips/cc2420x/tmicro/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 134 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 175
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x405fbb40){
#line 75
  switch (arg_0x405fbb40) {
#line 75
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 75
      break;
#line 75
    case FastCC2420TransceiverP__startTask:
#line 75
      FastCC2420TransceiverP__startTask__runTask();
#line 75
      break;
#line 75
    case FastCC2420TransceiverP__stopTask:
#line 75
      FastCC2420TransceiverP__stopTask__runTask();
#line 75
      break;
#line 75
    case CC2420XDriverLayerP__releaseSpi:
#line 75
      CC2420XDriverLayerP__releaseSpi__runTask();
#line 75
      break;
#line 75
    case /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task:
#line 75
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    case UartLogP__uartSendTask:
#line 75
      UartLogP__uartSendTask__runTask();
#line 75
      break;
#line 75
    case SerialP__RunTx:
#line 75
      SerialP__RunTx__runTask();
#line 75
      break;
#line 75
    case SerialP__startDoneTask:
#line 75
      SerialP__startDoneTask__runTask();
#line 75
      break;
#line 75
    case SerialP__stopDoneTask:
#line 75
      SerialP__stopDoneTask__runTask();
#line 75
      break;
#line 75
    case SerialP__defaultSerialFlushTask:
#line 75
      SerialP__defaultSerialFlushTask__runTask();
#line 75
      break;
#line 75
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone:
#line 75
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__signalSendDone__runTask();
#line 75
      break;
#line 75
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask:
#line 75
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask:
#line 75
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 75
      break;
#line 75
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 75
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 75
      break;
#line 75
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 75
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 75
      break;
#line 75
    case /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask:
#line 75
      /*FastCC2420TransceiverC.SendResourceC.Arbiter*/SimpleArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x405fbb40);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 173 "/home/xiaohui/Dropbox/Programming/TinyOS/Utils/UartLogP.nc"
static void UartLogP__UartSend__sendDone(message_t *msg, error_t error)
#line 173
{

  if (error != SUCCESS) {
    UartLogP__failBlink();
    }
  else 
#line 177
    {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
        {
          if (msg == & UartLogP__uartQueue[UartLogP__uartOut]->msg) {


              UartLogP__uartQueue[UartLogP__uartOut]->aggr_counts = 0;
              UartLogP__uartOut = (UartLogP__uartOut + 1) % UART_QUEUE_LEN;
              if (UartLogP__uartFull) {
                UartLogP__uartFull = FALSE;
                }
            }
        }
#line 188
        __nesc_atomic_end(__nesc_atomic); }
    }
  UartLogP__uartSendTask__postTask();
}

# 163 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 148 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialActiveMessageP.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMPacket__destination(message_t *amsg)
#line 148
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(amsg);

#line 150
  return __nesc_ntoh_uint16(header->dest.nxdata);
}

#line 68
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__AMSend__send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 70
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__getHeader(msg);

  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__Packet__maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_uint16(header->dest.nxdata, dest);





  __nesc_hton_uint8(header->type.nxdata, id);
  __nesc_hton_uint8(header->length.nxdata, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP__0__SubSend__send(msg, len);
}

# 518 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static void SerialP__MaybeScheduleTx(void )
#line 518
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 519
    {
      if (SerialP__txPending == 0) {
          if (SerialP__RunTx__postTask() == SUCCESS) {
              SerialP__txPending = 1;
            }
        }
    }
#line 525
    __nesc_atomic_end(__nesc_atomic); }
}

# 85 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP__0__ResourceConfigure__configure(uint8_t id)
#line 85
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP__0__Msp430UartConfigure__getConfig(id);

#line 87
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__enableIntr();
}

# 251 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P__Usart__disableSpi(void )
#line 251
{
  /* atomic removed: atomic calls only */
#line 252
  {
    HplMsp430Usart1P__ME2 &= ~0x10;
    HplMsp430Usart1P__SIMO__selectIOFunc();
    HplMsp430Usart1P__SOMI__selectIOFunc();
    HplMsp430Usart1P__UCLK__selectIOFunc();
  }
}

#line 211
static void HplMsp430Usart1P__Usart__disableUart(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430Usart1P__ME2 &= ~(0x20 | 0x10);
    HplMsp430Usart1P__UTXD__selectIOFunc();
    HplMsp430Usart1P__URXD__selectIOFunc();
  }
}

# 177 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__Resource__isOwner(uint8_t id)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 178
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId == id && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) {
          unsigned char __nesc_temp = 
#line 179
          TRUE;

          {
#line 179
            __nesc_atomic_end(__nesc_atomic); 
#line 179
            return __nesc_temp;
          }
        }
      else 
#line 180
        {
          unsigned char __nesc_temp = 
#line 180
          FALSE;

          {
#line 180
            __nesc_atomic_end(__nesc_atomic); 
#line 180
            return __nesc_temp;
          }
        }
    }
#line 183
    __nesc_atomic_end(__nesc_atomic); }
}

# 348 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static error_t SerialP__SplitControl__start(void )
#line 348
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP__txState != SerialP__TXSTATE_INACTIVE && SerialP__rxState != SerialP__RXSTATE_INACTIVE) 
        {
          unsigned char __nesc_temp = 
#line 351
          EALREADY;

          {
#line 351
            __nesc_atomic_end(__nesc_atomic); 
#line 351
            return __nesc_temp;
          }
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
#line 353
  SerialP__startDoneTask__postTask();
  return SUCCESS;
}

static void SerialP__testOff(void )
#line 357
{
  bool turnOff = FALSE;

  /* atomic removed: atomic calls only */
#line 359
  {
    if (SerialP__txState == SerialP__TXSTATE_INACTIVE && 
    SerialP__rxState == SerialP__RXSTATE_INACTIVE) {
        turnOff = TRUE;
      }
  }
  if (turnOff) {
      SerialP__stopDoneTask__postTask();
      /* atomic removed: atomic calls only */
#line 367
      SerialP__offPending = FALSE;
    }
  else {
      /* atomic removed: atomic calls only */
#line 370
      SerialP__offPending = TRUE;
    }
}

# 98 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC__SerialFrameComm__putDelimiter(void )
#line 98
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      HdlcTranslateC__state.sendEscape = 0;
      HdlcTranslateC__m_data = HDLC_FLAG_BYTE;
    }
#line 102
    __nesc_atomic_end(__nesc_atomic); }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

# 147 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/Msp430UartP.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP__0__UartStream__send(uint8_t id, uint8_t *buf, uint16_t len)
#line 147
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP__0__UsartResource__isOwner(id) == FALSE) {
    return FAIL;
    }
#line 150
  if (len == 0) {
    return FAIL;
    }
  else {
#line 152
    if (/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf) {
      return EBUSY;
      }
    }
#line 154
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_buf = buf;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_len = len;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos = 0;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__current_owner = id;
  /*Msp430Uart1P.UartP*/Msp430UartP__0__Usart__tx(buf[/*Msp430Uart1P.UartP*/Msp430UartP__0__m_tx_pos++]);
  return SUCCESS;
}

# 111 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            }
          else {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 127
            SUCCESS;

            {
#line 127
              __nesc_atomic_end(__nesc_atomic); 
#line 127
              return __nesc_temp;
            }
          }
        }
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
#line 130
  return FAIL;
}

# 247 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~0x40;
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 124 "../iMAC_TDMA/cc2420x/FastCC2420TransceiverP.nc"
static error_t FastCC2420TransceiverP__SplitControl__start(void )
#line 124
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 125
    FastCC2420TransceiverP__is_on = TRUE;
#line 125
    __nesc_atomic_end(__nesc_atomic); }
  return FastCC2420TransceiverP__RadioState__turnOn();
}

# 73 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 147 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0012)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 153 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0010)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 64 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0008)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 68
      HplMsp430InterruptP__Port10__fired();
#line 68
      return;
    }
#line 69
  if (n & (1 << 1)) {
#line 69
      HplMsp430InterruptP__Port11__fired();
#line 69
      return;
    }
#line 70
  if (n & (1 << 2)) {
#line 70
      HplMsp430InterruptP__Port12__fired();
#line 70
      return;
    }
#line 71
  if (n & (1 << 3)) {
#line 71
      HplMsp430InterruptP__Port13__fired();
#line 71
      return;
    }
#line 72
  if (n & (1 << 4)) {
#line 72
      HplMsp430InterruptP__Port14__fired();
#line 72
      return;
    }
#line 73
  if (n & (1 << 5)) {
#line 73
      HplMsp430InterruptP__Port15__fired();
#line 73
      return;
    }
#line 74
  if (n & (1 << 6)) {
#line 74
      HplMsp430InterruptP__Port16__fired();
#line 74
      return;
    }
#line 75
  if (n & (1 << 7)) {
#line 75
      HplMsp430InterruptP__Port17__fired();
#line 75
      return;
    }
}

#line 169
__attribute((wakeup)) __attribute((interrupt(0x0002)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 173
      HplMsp430InterruptP__Port20__fired();
#line 173
      return;
    }
#line 174
  if (n & (1 << 1)) {
#line 174
      HplMsp430InterruptP__Port21__fired();
#line 174
      return;
    }
#line 175
  if (n & (1 << 2)) {
#line 175
      HplMsp430InterruptP__Port22__fired();
#line 175
      return;
    }
#line 176
  if (n & (1 << 3)) {
#line 176
      HplMsp430InterruptP__Port23__fired();
#line 176
      return;
    }
#line 177
  if (n & (1 << 4)) {
#line 177
      HplMsp430InterruptP__Port24__fired();
#line 177
      return;
    }
#line 178
  if (n & (1 << 5)) {
#line 178
      HplMsp430InterruptP__Port25__fired();
#line 178
      return;
    }
#line 179
  if (n & (1 << 6)) {
#line 179
      HplMsp430InterruptP__Port26__fired();
#line 179
      return;
    }
#line 180
  if (n & (1 << 7)) {
#line 180
      HplMsp430InterruptP__Port27__fired();
#line 180
      return;
    }
}

# 96 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0006)))  void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P__Interrupts__rxDone(temp);
}

# 153 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}

# 412 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialP.nc"
static void SerialP__rx_state_machine(bool isDelimeter, uint8_t data)
#line 412
{

  switch (SerialP__rxState) {

      case SerialP__RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP__rxInit();
            SerialP__rxState = SerialP__RXSTATE_PROTO;
          }
      break;

      case SerialP__RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP__rxCRC = crcByte(SerialP__rxCRC, data);
            SerialP__rxState = SerialP__RXSTATE_TOKEN;
            SerialP__rxProto = data;
            if (!SerialP__valid_rx_proto(SerialP__rxProto)) {
              goto nosync;
              }
            if (SerialP__rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP__ReceiveBytePacket__startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP__RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP__rxSeqno = data;
            SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rxSeqno);
            SerialP__rxState = SerialP__RXSTATE_INFO;
          }
      break;

      case SerialP__RXSTATE_INFO: 
        if (SerialP__rxByteCnt < SerialP__SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP__rxByteCnt >= 2) {
                    if (SerialP__rx_current_crc() == SerialP__rxCRC) {
                        SerialP__ReceiveBytePacket__endPacket(SUCCESS);
                        SerialP__ack_queue_push(SerialP__rxSeqno);
                        SerialP__rxInit();
                        SerialP__SerialFrameComm__resetReceive();
                        if (SerialP__offPending) {
                            SerialP__rxState = SerialP__RXSTATE_INACTIVE;
                            SerialP__testOff();
                          }
                        goto done;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP__rxByteCnt >= 2) {
                    SerialP__ReceiveBytePacket__byteReceived(SerialP__rx_buffer_top());
                    SerialP__rxCRC = crcByte(SerialP__rxCRC, SerialP__rx_buffer_pop());
                  }
                SerialP__rx_buffer_push(data);
                SerialP__rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP__rxInit();
  SerialP__SerialFrameComm__resetReceive();
  SerialP__ReceiveBytePacket__endPacket(FAIL);
  if (SerialP__offPending) {
      SerialP__rxState = SerialP__RXSTATE_INACTIVE;
      SerialP__testOff();
    }
  else {
    if (isDelimeter) {
        SerialP__rxState = SerialP__RXSTATE_PROTO;
      }
    }
  done: ;
}

# 91 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 91
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 296 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__ReceiveBytePacket__endPacket(error_t result)
#line 296
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 298
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__RECV_STATE_IDLE;
      }
    else 
#line 308
      {

        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__unlockBuffer(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveState.which);
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP__0__receiveTask__postTask();
    }
}

# 166 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__state != /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP__1__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 101 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0004)))  void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P__Interrupts__txDone();
}

# 118 "/home/xiaohui/Dropbox/tinyos-2.1.2/tos/lib/serial/HdlcTranslateC.nc"
static void HdlcTranslateC__UartStream__sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 119
{
  /* atomic removed: atomic calls only */
#line 120
  {
    if (HdlcTranslateC__state.sendEscape) {
        HdlcTranslateC__state.sendEscape = 0;
        HdlcTranslateC__m_data = HdlcTranslateC__txTemp;
        HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
      }
    else {
        HdlcTranslateC__SerialFrameComm__putDone();
      }
  }
}

#line 106
static error_t HdlcTranslateC__SerialFrameComm__putData(uint8_t data)
#line 106
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC__state.sendEscape = 1;
      HdlcTranslateC__txTemp = data ^ 0x20;
      HdlcTranslateC__m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC__m_data = data;
    }
  return HdlcTranslateC__UartStream__send(&HdlcTranslateC__m_data, 1);
}

