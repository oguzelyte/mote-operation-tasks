#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 149 "/usr/bin/../lib/gcc/msp430/4.5.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 211
typedef unsigned int size_t;
#line 323
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
# 38 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/stdint.h" 3
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
#line 310
static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;
#line 340
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;








static __inline uint32_t __nesc_ntoh_leuint32(const void * source)  ;






static __inline uint32_t __nesc_hton_leuint32(void * target, uint32_t value)  ;
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
# 48 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/types.h" 3
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
# 40 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/string.h" 3
extern void *memset(void *arg_0x402a64f8, int arg_0x402a6650, size_t arg_0x402a67e8);
#line 61
extern void *memset(void *arg_0x402b1358, int arg_0x402b14b0, size_t arg_0x402b1648);
# 59 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/stdlib.h" 3
#line 55
typedef struct __nesc_unnamed4242 {

  int quot;
  int rem;
} div_t;







#line 63
typedef struct __nesc_unnamed4243 {

  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/sys/reent.h" 3
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

  void (*__cleanup)(struct _reent *arg_0x402d63b8);


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


  void (**_sig_func)(int arg_0x402d4a70);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 220
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 273
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/home/tinyos/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 29 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/platform.h"
enum PlatformInit {
  PLATFORM_INIT_FIRST = 0, 
  PLATFORM_INIT_CLOCKS = 10, 
  PLATFORM_INIT_GPIO = 20, 
  PLATFORM_INIT_WDT = 30, 
  PLATFORM_INIT_LAST = 255
};
# 40 "/home/tinyos/tinyos-2.x/tos/types/TinyError.h"
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
# 30 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/msp430.h" 3
#line 24
typedef enum msp430_cpu_e {

  MSP430_CPU_MSP430 = 0x0000, 
  MSP430_CPU_MSP430X = 0x0002, 
  MSP430_CPU_MSP430XV2 = 0x0003, 
  MSP430_CPU = 0x0003
} msp430_cpu_e;
#line 46
#line 34
typedef enum msp430_mpy_e {

  MSP430_MPY_NONE = 0x0000, 
  MSP430_MPY_TYPE_16 = 0x0010, 
  MSP430_MPY_TYPE_32 = 0x0020, 
  MSP430_MPY_TYPE_ANY = 0x0030, 
  MSP430_MPY_HAS_SE = 0x0001, 
  MSP430_MPY_HAS_DW = 0x0002, 
  MSP430_MPY_16 = MSP430_MPY_TYPE_16, 
  MSP430_MPY_16SE = MSP430_MPY_16 | MSP430_MPY_HAS_SE, 
  MSP430_MPY_32 = MSP430_MPY_TYPE_32 | MSP430_MPY_HAS_SE, 
  MSP430_MPY_32DW = MSP430_MPY_32 | MSP430_MPY_HAS_DW
} msp430_mpy_e;
# 43 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/in430.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );
# 157 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/msp430f2418.h" 3
extern volatile unsigned char IE2 __asm ("__""IE2");







extern volatile unsigned char IFG2 __asm ("__""IFG2");
#line 192
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
#line 451
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 663
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");

extern volatile unsigned char P1IFG __asm ("__""P1IFG");



extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");






extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");

extern volatile unsigned char P2IFG __asm ("__""P2IFG");

extern volatile unsigned char P2IES __asm ("__""P2IES");

extern volatile unsigned char P2IE __asm ("__""P2IE");

extern volatile unsigned char P2SEL __asm ("__""P2SEL");
#line 703
extern volatile unsigned char P3OUT __asm ("__""P3OUT");

extern volatile unsigned char P3DIR __asm ("__""P3DIR");

extern volatile unsigned char P3SEL __asm ("__""P3SEL");






extern volatile unsigned char P4OUT __asm ("__""P4OUT");

extern volatile unsigned char P4DIR __asm ("__""P4DIR");

extern volatile unsigned char P4SEL __asm ("__""P4SEL");
#line 731
extern volatile unsigned char P5OUT __asm ("__""P5OUT");

extern volatile unsigned char P5DIR __asm ("__""P5DIR");

extern volatile unsigned char P5SEL __asm ("__""P5SEL");






extern volatile unsigned char P6OUT __asm ("__""P6OUT");

extern volatile unsigned char P6DIR __asm ("__""P6DIR");

extern volatile unsigned char P6SEL __asm ("__""P6SEL");
#line 819
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");
#line 923
extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");
#line 937
extern volatile unsigned int TBR __asm ("__""TBR");

extern volatile unsigned int TBCCR0 __asm ("__""TBCCR0");
#line 1043
extern volatile unsigned char UCB0CTL0 __asm ("__""UCB0CTL0");

extern volatile unsigned char UCB0CTL1 __asm ("__""UCB0CTL1");

extern volatile unsigned char UCB0BR0 __asm ("__""UCB0BR0");

extern volatile unsigned char UCB0BR1 __asm ("__""UCB0BR1");



extern volatile unsigned char UCB0STAT __asm ("__""UCB0STAT");

extern const volatile unsigned char UCB0RXBUF __asm ("__""UCB0RXBUF");

extern volatile unsigned char UCB0TXBUF __asm ("__""UCB0TXBUF");
#line 1283
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
# 371 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/msp430hardware.h"
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
#line 426
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 441
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_DISABLED, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 29 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4251 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4252 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4253 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 4 "BlinkToRadio.h"
enum __nesc_unnamed4254 {
  AM_BLINKTORADIO = 6, 
  TIMER_PERIOD_MILLI = 1000
};




#line 9
typedef nx_struct BlinkToRadioMsg {
  nx_uint16_t nodeid;
  nx_uint16_t counter;
} __attribute__((packed)) BlinkToRadioMsg;
# 29 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/AM.h"
typedef uint8_t am_id_t;
typedef nx_uint8_t nx_am_id_t;

typedef uint32_t am_netmask_t;
typedef uint32_t am_group_t;
typedef uint32_t am_addr_t;



typedef nxle_uint32_t nx_am_netmask_t;
typedef nxle_uint32_t nx_am_group_t;
typedef nxle_uint32_t nx_am_addr_t;
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4255 {
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
#line 64
#line 51
typedef struct __nesc_unnamed4256 {

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
#line 76
#line 66
typedef struct __nesc_unnamed4257 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4258 {

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
# 32 "/home/tinyos/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4259 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 31 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.h"
enum Flags {
  FLAG_ACK_REQUEST = 1 << 0, 
  FLAG_ACK_REPLY = 1 << 1
};
#line 47
#line 41
typedef nx_struct chipcon_header_t {
  nx_uint8_t length;
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t flags;
  nx_am_id_t type;
} __attribute__((packed)) chipcon_header_t;





#line 52
typedef nx_struct chipcon_footer_t {
} __attribute__((packed)) chipcon_footer_t;
#line 71
#line 59
typedef nx_struct chipcon_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t crcAndLqi;
  nx_uint8_t ack;
  nx_uint16_t rxInterval;
  nx_uint32_t timestamp;
} __attribute__((packed)) 





chipcon_metadata_t;
# 78 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4260 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4261 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4262 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 116
#line 104
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









#line 120
typedef nx_struct serial_header {
  nx_uint16_t dest;
  nx_uint16_t src;
  nx_uint8_t length;
  nx_uint8_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 128
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 133
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/platform_message.h"
#line 32
typedef union message_header {
  chipcon_header_t radio;
  serial_header_t serial;
} message_header_t;



#line 37
typedef union message_footer {
  chipcon_footer_t radio;
} message_footer_t;



#line 41
typedef union message_metadata {
  chipcon_metadata_t radio;
} message_metadata_t;
# 19 "/home/tinyos/tinyos-2.x/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28U];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 29 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ChipconAssert.h"
enum ErrorCodes {
  ASSERT_CC_OFFSET = 100, 
  ASSERT_CC_TEST = ASSERT_CC_OFFSET + 6, 
  ASSERT_CC_HAL_SPI_REQUEST, 
  ASSERT_CC_HAL_CHIP_VERSION, 
  ASSERT_CC_HAL_TX_ALREADY, 
  ASSERT_CC_HAL_CCA_NO_RX, 
  ASSERT_CC_HAL_RSSI_NO_RX, 
  ASSERT_CC_HAL_NOT_IDLE, 
  ASSERT_CC_HAL_RX_FIFO_EMPTY, 
  ASSERT_CC_HAL_PACKET_TOO_LARGE, 
  ASSERT_CC_HAL_INVALID_LENGTH, 
  ASSERT_CC_HAL_NO_PENDING, 
  ASSERT_CC_HAL_TOO_MANY_PENDING, 
  ASSERT_CC_HAL_TX_WAS_RX, 
  ASSERT_CC_HAL_G0_HIGH, 
  ASSERT_CC_HAL_NO_TX, 
  ASSERT_CC_HAL_CALIBRATE_TX, 
  ASSERT_CC_RXTX_NULL_POINTER, 
  ASSERT_CC_RXTX_BAD_TX_POINTER, 
  ASSERT_CC_RXTX_BROADCAST, 
  ASSERT_CC_ACK_SUBSEND_FAILED, 
  ASSERT_CC_ACK_SEND_ACK_FAILED, 
  ASSERT_CC_ACK_UNEXPECTED_ERROR, 
  ASSERT_CC_ACK_SEND, 
  ASSERT_CC_ACK_PACKET, 
  ASSERT_CC_ACK_BUSY, 
  ASSERT_CC_ACK_STATE, 
  ASSERT_CC_LPL_OFF, 
  ASSERT_CC_LPL_START, 
  ASSERT_CC_LPL_STOP, 
  ASSERT_CC_LPL_SEND, 
  ASSERT_CC_AM_SENDDONE, 
  ASSERT_CC_AM_MODIFIED
};
# 29 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/ChipconRegisters.h"
enum __nesc_unnamed4263 {
#line 29
  NUM_REGISTERS = 36
};

enum ChipconConfigRegisters {
  FSCTRL1 = 0x0B, 
  FSCTRL0 = 0x0C, 
  FREQ2 = 0x0D, 
  FREQ1 = 0x0E, 
  FREQ0 = 0x0F, 
  MDMCFG4 = 0x10, 
  MDMCFG3 = 0x11, 
  MDMCFG2 = 0x12, 
  MDMCFG1 = 0x13, 
  MDMCFG0 = 0x14, 
  CHANNR = 0x0A, 
  DEVIATN = 0x15, 
  FREND1 = 0x21, 
  FREND0 = 0x22, 
  MCSM1 = 0x17, 
  MCSM0 = 0x18, 
  FOCCFG = 0x19, 
  BSCFG = 0x1A, 
  AGCCTRL2 = 0x1B, 
  AGCCTRL1 = 0x1C, 
  AGCCTRL0 = 0x1D, 
  FSCAL3 = 0x23, 
  FSCAL2 = 0x24, 
  FSCAL1 = 0x25, 
  FSCAL0 = 0x26, 
  FSTEST = 0x29, 
  TEST2 = 0x2C, 
  TEST1 = 0x2D, 
  TEST0 = 0x2E, 
  IOCFG2 = 0x00, 
  IOCFG0 = 0x02, 
  PKTCTRL1 = 0x07, 
  PKTCTRL0 = 0x08, 
  ADDR = 0x09, 
  PKTLEN = 0x06, 
  PATABLE = 0x3E
};

enum StatusRegisters {
  PARTNUM = 0x30, 
  VERSION = 0x31, 
  FREQEST = 0x32, 
  LQI = 0x33, 
  RSSI = 0x34, 
  MARCSTATE = 0x35, 
  WORTIME1 = 0x36, 
  WORTIME0 = 0x37, 
  PKTSTATUS = 0x38, 
  VCO_VC_DAC = 0x39, 
  TXBYTES = 0x3A, 
  RXBYTES = 0x3B, 
  RCCTRL1_STATUS = 0x3C, 
  RCCTRL0_STATUS = 0x3D
};

enum FifoAccess {
  TXFIFO = 0x3F, 
  RXFIFO = 0xBF, 
  TX_BURST_WRITE = 0x7F, 
  RX_BURST_READ = 0xFF
};


enum Strobes {
  SRES = 0x30, 
  SFSTXON = 0x31, 

  SXOFF = 0x32, 
  SCAL = 0x33, 

  SRX = 0x34, 
  STX = 0x35, 

  SIDLE = 0x36, 
  SWOR = 0x38, 
  SPWD = 0x39, 
  SFRX = 0x3A, 
  SFTX = 0x3B, 
  SWORRST = 0x3C, 
  SNOP = 0x3D
};
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/cc1101/ChipconRegisterValues.h"
enum __nesc_unnamed4264 {
  PREAMBLE_BYTES = 4, 
  SYNC_BYTES = 4, 
  CRC_BYTES = 2, 
  RSSI_OFFSET = 74, 
  EXPECTED_PARTNUM = 0x00, 
  EXPECTED_VERSION_CC1101 = 0x04, 
  EXPECTED_VERSION_RF1A = 0x06
};



enum OutputPower {
  POWER_DEFAULT = 0xC6, 
  POWER_MINUS_30 = 0x03, 
  POWER_MINUS_20 = 0x0F, 
  POWER_MINUS_10 = 0x27, 
  POWER_MINUS_5 = 0x67, 
  POWER_0 = 0x50, 
  POWER_5 = 0x81, 
  POWER_7 = 0xCB, 
  POWER_10 = 0xC2, 
  POWER_12 = 0xC5
};
#line 152
const uint8_t chipconRegisterValues[NUM_REGISTERS * 2] = { 
#line 177
FSCTRL1, 0x06, 
FSCTRL0, 0x00, 
FREQ2, 0x21, 
FREQ1, 0x65, 
FREQ0, 0x6A, 
MDMCFG4, 0xCA, 
MDMCFG3, 0x83, 
MDMCFG2, 0x1B, 
MDMCFG1, 0x22, 
MDMCFG0, 0xF8, 
CHANNR, 0, 
DEVIATN, 0x34, 
FREND1, 0x56, 
FREND0, 0x10, 
FOCCFG, 0x16, 
BSCFG, 0x6C, 
AGCCTRL2, 0x43, 
AGCCTRL1, 0x68, 
AGCCTRL0, 0x91, 
FSCAL3, 0xE9, 
FSCAL2, 0x2A, 
FSCAL1, 0x00, 
FSCAL0, 0x1F, 
FSTEST, 0x59, 
TEST2, 0x81, 
TEST1, 0x35, 
TEST0, 0x09, 


IOCFG2, 0x09, 


IOCFG0, 0x06, 





PKTCTRL1, 0x0C, 


PKTCTRL0, 0x45, 

ADDR, 0x00, 



PKTLEN, (uint8_t )(sizeof(chipcon_header_t ) + 28U + sizeof(chipcon_footer_t )) - 1, 


MCSM1, 0x30, 


MCSM0, 0x28, 

PATABLE, POWER_12 };
# 80 "/home/tinyos/tinyos-2.x/tos/system/crc.h"
static inline uint16_t crcByte(uint16_t crc, uint8_t b);
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/timesync/PacketTimeSync.h"
#line 33
typedef nx_struct __nesc_unnamed4265 {
  nx_am_id_t type;
  nx_uint32_t eventTime;
} __attribute__((packed)) chipcon_timesync_header_t;
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.h"
#line 29
typedef struct __nesc_unnamed4266 {
  uint32_t txCount;
  uint32_t rxCount;
  uint32_t dropCount;
  uint8_t errorCount;
} cc_hal_status_t;
# 32 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/Assert.h"
enum PredefinedAssertions {
  ASSERT_UNUSED = 0U, 
  ASSERT_RESERVED1 = 1U, 
  ASSERT_RESERVED2 = 2U, 
  ASSERT_RESERVED3 = 3U, 
  ASSERT_CANT_HAPPEN = 4U
};
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/lpl/LowPowerListening.h"
#line 33
typedef struct __nesc_unnamed4267 {
  uint32_t timeOn;
  uint32_t timeTotal;
} cc_lpl_status_t;

enum __nesc_unnamed4268 {
  CCA_SETTLING_TIME = 1, 
  LPL_PREAMBLE_OVERLAP = 5
};
typedef TMilli StackGuardWatchDogP__Timer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4269 {
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
typedef TMilli BlinkToRadioC__Timer0__precision_tag;
typedef T32khz ChipconPacketP__PacketTimeStamp__precision_tag;
typedef uint32_t ChipconPacketP__PacketTimeStamp__size_type;
typedef T32khz PacketTimeStampP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t PacketTimeStampP__PacketTimeStamp32khz__size_type;
typedef T32khz PacketTimeStampP__LocalTime32khz__precision_tag;
typedef TMilli PacketTimeStampP__LocalTimeMilli__precision_tag;
typedef TMilli PacketTimeStampP__PacketTimeStampMilli__precision_tag;
typedef uint32_t PacketTimeStampP__PacketTimeStampMilli__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef T32khz /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
typedef TMilli AckP__AckTimer__precision_tag;
typedef TMicro /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag;
typedef uint16_t /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type;
typedef TMicro HalChipconControlP__BusyWait__precision_tag;
typedef uint16_t HalChipconControlP__BusyWait__size_type;
typedef cc_hal_status_t *HalChipconControlP__Status__val_t;
typedef T32khz HalChipconControlP__LocalTime__precision_tag;
typedef TMilli HalChipconControlP__TxTimer__precision_tag;
typedef T32khz SendP__PacketTimeStamp__precision_tag;
typedef uint32_t SendP__PacketTimeStamp__size_type;
typedef TMilli SendP__BackoffTimer__precision_tag;
typedef TMilli SendP__PreambleTimer__precision_tag;
typedef uint16_t RandomMlcgC__SeedInit__parameter;
typedef T32khz ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t ReceiveP__PacketTimeStamp__size_type;
enum AMQueueP____nesc_unnamed4270 {
  AMQueueP__NUM_CLIENTS = 1U
};
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/Reboot.nc"
static void RebootC__Reboot__reboot(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t PlatformP__InitLevel__default__init(
# 32 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
uint8_t arg_0x403e0430);
# 35 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 34
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 54 "/home/tinyos/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40444448);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40444448);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 59 "/home/tinyos/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t Msp430GeneralIOP__Init__init(void );
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );
#line 38
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void );




static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );
#line 75
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void );
#line 38
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__set(void );




static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void );
#line 75
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void );
#line 38
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__set(void );




static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void );
#line 75
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput(void );
#line 63
static bool /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get(void );
#line 56
static uint8_t /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw(void );
#line 38
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__set(void );




static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr(void );
#line 68
static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void );
#line 63
static bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void );
#line 56
static uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void );
#line 68
static void /*HplMsp430GeneralIOC.P25*/HplMsp430GeneralIOP__13__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeOutput(void );






static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );
#line 68
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__makeInput(void );
#line 63
static bool /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__get(void );
#line 56
static uint8_t /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__getRaw(void );
#line 82
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );
#line 75
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeOutput(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 41
static void HplMsp430InterruptP__Port26__clear(void );
#line 41
static void HplMsp430InterruptP__Port17__clear(void );
#line 41
static void HplMsp430InterruptP__Port21__clear(void );
#line 41
static void HplMsp430InterruptP__Port12__clear(void );
#line 41
static void HplMsp430InterruptP__Port24__clear(void );
#line 36
static void HplMsp430InterruptP__Port24__disable(void );
#line 56
static void HplMsp430InterruptP__Port24__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port24__enable(void );









static void HplMsp430InterruptP__Port15__clear(void );
#line 41
static void HplMsp430InterruptP__Port27__clear(void );
#line 41
static void HplMsp430InterruptP__Port10__clear(void );
#line 41
static void HplMsp430InterruptP__Port22__clear(void );
#line 41
static void HplMsp430InterruptP__Port13__clear(void );
#line 41
static void HplMsp430InterruptP__Port25__clear(void );
#line 41
static void HplMsp430InterruptP__Port16__clear(void );
#line 41
static void HplMsp430InterruptP__Port20__clear(void );
#line 41
static void HplMsp430InterruptP__Port11__clear(void );
#line 41
static void HplMsp430InterruptP__Port23__clear(void );
# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 33
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__set(void );
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__clr(void );




static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__makeOutput(void );
#line 33
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__set(void );
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__clr(void );




static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 33
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__set(void );
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__clr(void );

static bool /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__get(void );


static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__makeOutput(void );
#line 33
static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__set(void );
static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__clr(void );


static void /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__makeInput(void );
#line 36
static bool /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__get(void );
static void /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__GeneralIO__makeInput(void );

static void /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__GeneralIO__makeOutput(void );
#line 37
static void /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__makeInput(void );
#line 36
static bool /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__get(void );


static void /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__GeneralIO__makeOutput(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__fired(void );
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__disable(void );
#line 43
static error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__enableFallingEdge(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__default__fired(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__fired(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__default__fired(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformPinsP__Init__init(void );
# 49 "/home/tinyos/tinyos-2.x/tos/interfaces/Boot.nc"
static void StackGuardWatchDogP__Boot__booted(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t StackGuardWatchDogP__Init__init(void );
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void StackGuardWatchDogP__Timer__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408d49f0);
# 81 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408d49f0);
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408d49f0, 
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408d49f0, 
# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408d49f0);
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x403b2580);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x403b2580);
# 46 "/home/tinyos/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Off(void );










static void LedsP__Leds__led1On(void );




static void LedsP__Leds__led1Off(void );
#line 83
static void LedsP__Leds__led2Off(void );
#line 123
static void LedsP__Leds__set(uint8_t val);
#line 45
static void LedsP__Leds__led0On(void );
#line 78
static void LedsP__Leds__led2On(void );
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkToRadioC__Timer0__fired(void );
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void BlinkToRadioC__AMControl__startDone(error_t error);
#line 117
static void BlinkToRadioC__AMControl__stopDone(error_t error);
# 49 "/home/tinyos/tinyos-2.x/tos/interfaces/Boot.nc"
static void BlinkToRadioC__Boot__booted(void );
# 99 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static void BlinkToRadioC__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



BlinkToRadioC__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/NetMask.nc"
static am_netmask_t ActiveMessageAddressC__NetMask__netMask(void );
# 52 "/home/tinyos/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static ChipconPacketP__PacketTimeStamp__size_type ChipconPacketP__PacketTimeStamp__timestamp(
#line 41
message_t * msg);
#line 67
static void ChipconPacketP__PacketTimeStamp__set(
#line 62
message_t * msg, 




ChipconPacketP__PacketTimeStamp__size_type value);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t ChipconPacketP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


ChipconPacketP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t ChipconPacketP__Packet__maxPayloadLength(void );
#line 83
static void ChipconPacketP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
static void ChipconPacketP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len);
#line 33
static chipcon_header_t *ChipconPacketP__ChipconPacket__getHeader(message_t *msg);


static uint8_t ChipconPacketP__ChipconPacket__getPacketLength(message_t *msg);
#line 35
static chipcon_metadata_t *ChipconPacketP__ChipconPacket__getMetadata(message_t *msg);

static uint8_t ChipconPacketP__ChipconPacket__getPayloadLength(message_t *msg);
# 166 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_group_t ChipconPacketP__AMPacket__group(
#line 162
message_t * amsg);
#line 57
static am_addr_t ChipconPacketP__AMPacket__address(void );









static am_addr_t ChipconPacketP__AMPacket__destination(
#line 63
message_t * amsg);
#line 110
static void ChipconPacketP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void ChipconPacketP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t ChipconPacketP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void ChipconPacketP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool ChipconPacketP__AMPacket__isForMe(
#line 122
message_t * amsg);
#line 184
static am_group_t ChipconPacketP__AMPacket__localGroup(void );
# 83 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t ChipconActiveMessageP__SplitControl__start(void );
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void ChipconActiveMessageP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ChipconActiveMessageP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void ChipconActiveMessageP__started__runTask(void );
#line 64
static void ChipconActiveMessageP__stopped__runTask(void );
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t ChipconActiveMessageP__AMSend__send(
# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
am_id_t arg_0x409d7a98, 
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ChipconActiveMessageP__Snoop__default__receive(
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
am_id_t arg_0x40a03b08, 
# 60 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 67
static 
#line 63
message_t * 



ChipconActiveMessageP__Receive__default__receive(
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
am_id_t arg_0x40a03470, 
# 60 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 53 "/home/tinyos/tinyos-2.x/tos/interfaces/Crc.nc"
static uint16_t CrcC__Crc__seededCrc16(uint16_t startCrc, 
#line 49
void * buf, 



uint8_t len);
#line 42
static uint16_t CrcC__Crc__crc16(
#line 38
void * buf, 



uint8_t len);
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 53
static /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
# 50 "/home/tinyos/tinyos-2.x/tos/lib/timer/LocalTime.nc"
static uint32_t /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void AckP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



AckP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t AckP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void AckP__AckTimer__fired(void );
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t AckP__StdControl__start(void );
# 55 "/home/tinyos/tinyos-2.x/tos/lib/timer/BusyWait.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type dt);
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HalChipconControlP__rxDone__runTask(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void HalChipconControlP__G0Interrupt__fired(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HalChipconControlP__txDone__runTask(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HalChipconControlP__Init__init(void );
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
static void HalChipconControlP__SpiResource__granted(void );
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HalChipconControlP__TxTimer__fired(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HalChipconControlP__taskSetAddress__runTask(void );
# 137 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static error_t HalChipconControlP__HalChipconControl__read(uint8_t *buffer);
#line 31
static void HalChipconControlP__HalChipconControl__init(void );





static void HalChipconControlP__HalChipconControl__on(void );
#line 59
static error_t HalChipconControlP__HalChipconControl__tx(void );
#line 128
static void HalChipconControlP__HalChipconControl__write(uint8_t *buffer, uint8_t length);
#line 42
static void HalChipconControlP__HalChipconControl__off(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t HplChipconSpiB0P__Init__init(void );
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpi.nc"
static void HplChipconSpiB0P__HplChipconSpi__writeRegister(uint8_t reg, uint8_t value);





static void HplChipconSpiB0P__HplChipconSpi__read(uint8_t addr, uint8_t *buffer, uint8_t len);
#line 30
static uint8_t HplChipconSpiB0P__HplChipconSpi__strobe(uint8_t cmd);
#line 52
static void HplChipconSpiB0P__HplChipconSpi__write(uint8_t addr, uint8_t *buffer, uint8_t len);
#line 35
static uint8_t HplChipconSpiB0P__HplChipconSpi__readRegister(uint8_t reg);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void DedicatedResourceC__grant__runTask(void );
# 110 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t DedicatedResourceC__Resource__release(void );
#line 87
static error_t DedicatedResourceC__Resource__immediateRequest(void );
#line 118
static bool DedicatedResourceC__Resource__isOwner(void );
# 74 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static void SendP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
#line 67
static void SendP__HalChipconControl__txStart(uint32_t timestamp);
#line 81
static void SendP__HalChipconControl__rxWaiting(uint32_t timestamp);
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendP__BackoffTimer__fired(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t SendP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 101
static uint8_t SendP__Send__maxPayloadLength(void );
#line 78
static error_t SendP__Send__cancel(
#line 74
message_t * msg);
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendP__PreambleTimer__fired(void );
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t SendP__StdControl__start(void );
# 30 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendNotify.nc"
static void SendP__SendNotify__default__sending(message_t *msg);
# 41 "/home/tinyos/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 35
static uint32_t RandomMlcgC__Random__rand32(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 74 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static void ReceiveP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
#line 67
static void ReceiveP__HalChipconControl__txStart(uint32_t timestamp);
#line 81
static void ReceiveP__HalChipconControl__rxWaiting(uint32_t timestamp);
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t PowerP__StdControl__start(void );
# 74 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static void PowerP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
#line 67
static void PowerP__HalChipconControl__txStart(uint32_t timestamp);
#line 81
static void PowerP__HalChipconControl__rxWaiting(uint32_t timestamp);
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void SendReceiveP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



SendReceiveP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t SendReceiveP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
#line 78
static error_t SendReceiveP__Send__cancel(
#line 74
message_t * msg);
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t SendReceiveP__StdControl__start(void );
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(
# 40 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40c080a0, 
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(
# 38 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40c096a8, 
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(
# 38 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40c096a8, 
# 85 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );
#line 64
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/Reboot.nc"
static void AssertRebootP__Reboot__reboot(void );
# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/AssertRebootP.nc"
static inline void doAssert(bool condition, uint16_t errorCode)  ;






static void doAssertNot(bool condition, uint16_t errorCode)  ;






static void doAssertSuccess(error_t error, uint16_t errorCode)  ;






static void doAssertEquals(uint32_t a, uint32_t b, uint16_t errorCode)  ;
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/RebootC.nc"
static inline void RebootC__Reboot__reboot(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__InitLevel__init(
# 32 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
uint8_t arg_0x403e0430);



static inline error_t PlatformP__Init__init(void );








static inline error_t PlatformP__InitLevel__default__init(uint8_t level);
# 32 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 49 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4271 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 81
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 102
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 117
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 132
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 165
static inline void Msp430ClockP__startTimerB(void );
#line 177
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 206
static inline int Msp430ClockP__busyCalibrateDco(void );
#line 233
static inline void Msp430ClockP__busyWait(uint16_t delay);






static inline error_t Msp430ClockP__Init__init(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40444448);
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 51 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x40444448);
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0032)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0030)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x003A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0038)))  ;
# 54 "/home/tinyos/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/McuSleepC.nc"
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
#line 122
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GeneralIOP.nc"
static inline error_t Msp430GeneralIOP__Init__init(void );
# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );
#line 51
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void );
#line 51
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void );
#line 51
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__set(void );
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput(void );
#line 54
static inline uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P25*/HplMsp430GeneralIOP__13__IO__makeInput(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );
#line 54
static inline uint8_t /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );
#line 58
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );
# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 61
static void HplMsp430InterruptP__Port26__fired(void );
#line 61
static void HplMsp430InterruptP__Port17__fired(void );
#line 61
static void HplMsp430InterruptP__Port21__fired(void );
#line 61
static void HplMsp430InterruptP__Port12__fired(void );
#line 61
static void HplMsp430InterruptP__Port24__fired(void );
#line 61
static void HplMsp430InterruptP__Port15__fired(void );
#line 61
static void HplMsp430InterruptP__Port27__fired(void );
#line 61
static void HplMsp430InterruptP__Port10__fired(void );
#line 61
static void HplMsp430InterruptP__Port22__fired(void );
#line 61
static void HplMsp430InterruptP__Port13__fired(void );
#line 61
static void HplMsp430InterruptP__Port25__fired(void );
#line 61
static void HplMsp430InterruptP__Port16__fired(void );
#line 61
static void HplMsp430InterruptP__Port20__fired(void );
#line 61
static void HplMsp430InterruptP__Port11__fired(void );
#line 61
static void HplMsp430InterruptP__Port23__fired(void );
# 55 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0024)))  ;
#line 93
static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );
#line 160
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0026)))  ;
#line 185
static inline void HplMsp430InterruptP__Port24__enable(void );







static inline void HplMsp430InterruptP__Port24__disable(void );



static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
#line 237
static inline void HplMsp430InterruptP__Port24__edge(bool l2h);
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 38
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__makeOutput(void );
#line 38
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__set(void );




static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__clr(void );
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__set(void );
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__clr(void );




static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__makeOutput(void );
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 38
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__set(void );




static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__set(void );
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__clr(void );




static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__makeOutput(void );
#line 63
static bool /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__get(void );
#line 38
static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__set(void );




static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__clr(void );
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__set(void );
static inline void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__clr(void );

static inline bool /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__get(void );


static inline void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__makeOutput(void );
# 68 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__HplGeneralIO__makeInput(void );
#line 63
static bool /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__HplGeneralIO__get(void );
# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__get(void );
static inline void /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__makeInput(void );
# 68 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__HplGeneralIO__makeInput(void );
# 44 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__GeneralIO__makeInput(void );
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__HplGeneralIO__makeOutput(void );
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__GeneralIO__makeOutput(void );
# 68 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__HplGeneralIO__makeInput(void );
#line 63
static bool /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__HplGeneralIO__get(void );
# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__get(void );
static inline void /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__makeInput(void );
# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__HplGeneralIO__makeOutput(void );
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__GeneralIO__makeOutput(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__clear(void );
#line 36
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__disable(void );
#line 56
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__enable(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__enable(bool rising);
#line 54
static inline error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__enableFallingEdge(void );



static error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__disable(void );







static inline void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__default__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__clear(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__default__fired(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformPinsP__DefaultInit__init(void );
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformPinsP.nc"
static inline error_t PlatformPinsP__Init__init(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void StackGuardWatchDogP__Timer__startPeriodic(uint32_t dt);
# 47 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/StackGuardWatchDogP.nc"
enum StackGuardWatchDogP____nesc_unnamed4272 {
  StackGuardWatchDogP__ASSERT_STACK_OVERFLOW = 1
};









register void *StackGuardWatchDogP____stackptr __asm ("r1");




extern uint8_t _end  ;





uint16_t *StackGuardWatchDogP__stackGuard;






static inline void StackGuardWatchDogP__stackProtectionInit(void );
#line 104
static inline void StackGuardWatchDogP__stackCheck(void );
#line 119
static inline void StackGuardWatchDogP__resetWatchdog(void );



static inline error_t StackGuardWatchDogP__Init__init(void );






static inline void StackGuardWatchDogP__Timer__fired(void );




static inline void StackGuardWatchDogP__Boot__booted(void );
# 30 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4273 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4274 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4275 {
#line 63
  AlarmToTimerC__0__fired = 0U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408d49f0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4276 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4277 {

  VirtualizeTimerC__0__NUM_TIMERS = 6U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4278 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 47 "/home/tinyos/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/home/tinyos/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/home/tinyos/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/home/tinyos/tinyos-2.x/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x403b2580);
# 59 "/home/tinyos/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4279 {

  SchedulerBasicP__NUM_TASKS = 10U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 86
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 159
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 33
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );




static void LedsP__Led1__makeOutput(void );
#line 33
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );




static void LedsP__Led2__makeOutput(void );
#line 33
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 66
static inline void LedsP__Leds__led0On(void );




static inline void LedsP__Leds__led0Off(void );









static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );









static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );
#line 128
static inline void LedsP__Leds__set(uint8_t val);
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkToRadioC__Timer0__startPeriodic(uint32_t dt);
# 83 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t BlinkToRadioC__AMControl__start(void );
# 115 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
static 
#line 112
void * 


BlinkToRadioC__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t BlinkToRadioC__AMSend__send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 123 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
static void BlinkToRadioC__Leds__set(uint8_t val);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t BlinkToRadioC__AMPacket__destination(
#line 63
message_t * amsg);
# 16 "BlinkToRadioC.nc"
bool BlinkToRadioC__busy = FALSE;
message_t BlinkToRadioC__pkt;
uint16_t BlinkToRadioC__counter = 0;

static inline void BlinkToRadioC__Boot__booted(void );



static inline void BlinkToRadioC__AMSend__sendDone(message_t *msg, error_t error);





static inline void BlinkToRadioC__AMControl__startDone(error_t err);








static inline void BlinkToRadioC__AMControl__stopDone(error_t err);


static inline message_t *BlinkToRadioC__Receive__receive(message_t *msg, void *payload, uint8_t len);







static inline void BlinkToRadioC__Timer0__fired(void );
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = 0x00000000UL | 1;
am_addr_t ActiveMessageAddressC__group = 0x00000000UL;
am_addr_t ActiveMessageAddressC__netmask = 0xFFFF0000UL;

static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 58
static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );








static am_netmask_t ActiveMessageAddressC__NetMask__netMask(void );
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ChipconPacketP__ActiveMessageAddress__amAddress(void );




static am_group_t ChipconPacketP__ActiveMessageAddress__amGroup(void );
# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/NetMask.nc"
static am_netmask_t ChipconPacketP__NetMask__netMask(void );
# 49 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline am_addr_t ChipconPacketP__getLocalBroadcastAddress(void );



static inline chipcon_header_t *ChipconPacketP__ChipconPacket__getHeader(message_t *msg);







static inline chipcon_metadata_t *ChipconPacketP__ChipconPacket__getMetadata(message_t *msg);



static uint8_t ChipconPacketP__ChipconPacket__getPacketLength(message_t *msg);




static inline uint8_t ChipconPacketP__ChipconPacket__getPayloadLength(message_t *msg);




static void ChipconPacketP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len);










static inline uint8_t ChipconPacketP__Packet__payloadLength(message_t *msg);



static inline void ChipconPacketP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t ChipconPacketP__Packet__maxPayloadLength(void );



static void *ChipconPacketP__Packet__getPayload(message_t *msg, uint8_t len);







static inline am_addr_t ChipconPacketP__AMPacket__address(void );



static inline am_group_t ChipconPacketP__AMPacket__localGroup(void );



static am_addr_t ChipconPacketP__AMPacket__destination(message_t *amsg);









static void ChipconPacketP__AMPacket__setDestination(message_t *amsg, am_addr_t addr);
#line 139
static inline void ChipconPacketP__AMPacket__setSource(message_t *amsg, am_addr_t addr);




static inline bool ChipconPacketP__AMPacket__isForMe(message_t *amsg);










static inline am_id_t ChipconPacketP__AMPacket__type(message_t *amsg);




static void ChipconPacketP__AMPacket__setType(message_t *amsg, am_id_t type);




static inline am_group_t ChipconPacketP__AMPacket__group(message_t *amsg);
#line 214
static inline uint32_t ChipconPacketP__PacketTimeStamp__timestamp(message_t *msg);









static void ChipconPacketP__PacketTimeStamp__set(message_t *msg, uint32_t value);
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void ChipconActiveMessageP__SplitControl__startDone(error_t error);
#line 117
static void ChipconActiveMessageP__SplitControl__stopDone(error_t error);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t ChipconActiveMessageP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t ChipconActiveMessageP__started__postTask(void );
# 99 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static void ChipconActiveMessageP__AMSend__sendDone(
# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
am_id_t arg_0x409d7a98, 
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ChipconActiveMessageP__Snoop__receive(
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
am_id_t arg_0x40a03b08, 
# 60 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t ChipconActiveMessageP__SubControl__start(void );
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ChipconActiveMessageP__Receive__receive(
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
am_id_t arg_0x40a03470, 
# 60 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/home/tinyos/tinyos-2.x/tos/interfaces/Crc.nc"
static uint16_t ChipconActiveMessageP__Crc__crc16(
#line 38
void * buf, 



uint8_t len);
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t ChipconActiveMessageP__AMPacket__address(void );
#line 110
static void ChipconActiveMessageP__AMPacket__setSource(
#line 106
message_t * amsg, 



am_addr_t addr);
#line 92
static void ChipconActiveMessageP__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t ChipconActiveMessageP__AMPacket__type(
#line 132
message_t * amsg);
#line 151
static void ChipconActiveMessageP__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
#line 125
static bool ChipconActiveMessageP__AMPacket__isForMe(
#line 122
message_t * amsg);
# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
enum ChipconActiveMessageP____nesc_unnamed4280 {
#line 53
  ChipconActiveMessageP__started = 2U
};
#line 53
typedef int ChipconActiveMessageP____nesc_sillytask_started[ChipconActiveMessageP__started];



enum ChipconActiveMessageP____nesc_unnamed4281 {
#line 57
  ChipconActiveMessageP__stopped = 3U
};
#line 57
typedef int ChipconActiveMessageP____nesc_sillytask_stopped[ChipconActiveMessageP__stopped];
#line 50
bool ChipconActiveMessageP__txBusy;
uint16_t ChipconActiveMessageP__crc;

static inline void ChipconActiveMessageP__started__runTask(void );



static inline void ChipconActiveMessageP__stopped__runTask(void );



static error_t ChipconActiveMessageP__SplitControl__start(void );
#line 113
static inline uint16_t ChipconActiveMessageP__crcMessage(message_t *msg);
#line 125
static error_t ChipconActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len);
#line 160
static inline void ChipconActiveMessageP__SubSend__sendDone(message_t *msg, error_t result);
#line 176
static message_t *ChipconActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 191
static inline message_t *ChipconActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *ChipconActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len);
# 50 "/home/tinyos/tinyos-2.x/tos/system/CrcC.nc"
static inline uint16_t CrcC__Crc__crc16(void *buf, uint8_t len);
#line 63
static uint16_t CrcC__Crc__seededCrc16(uint16_t startCrc, void *buf, uint8_t len);
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 56 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__1____nesc_unnamed4282 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) + 0, 



  TransformCounterC__1__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 122
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void );
# 42 "/home/tinyos/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void );




static inline void /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t AckP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 78
static error_t AckP__SubSend__cancel(
#line 74
message_t * msg);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t AckP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


AckP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void AckP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t AckP__SubControl__start(void );
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



AckP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
static void AckP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len);
#line 33
static chipcon_header_t *AckP__ChipconPacket__getHeader(message_t *msg);

static chipcon_metadata_t *AckP__ChipconPacket__getMetadata(message_t *msg);

static uint8_t AckP__ChipconPacket__getPayloadLength(message_t *msg);
# 81 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool AckP__AckTimer__isRunning(void );
#line 62
static void AckP__AckTimer__startOneShot(uint32_t dt);




static void AckP__AckTimer__stop(void );
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t AckP__AMPacket__address(void );









static am_addr_t AckP__AMPacket__destination(
#line 63
message_t * amsg);
# 73 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
message_t AckP__rxBuffer;
message_t *AckP__buffer = &AckP__rxBuffer;



message_t AckP__ackBuffer;
message_t *AckP__ackMessage = &AckP__ackBuffer;
message_t *AckP__txMessage;
message_t *AckP__rxMessage;

bool AckP__sending;
bool AckP__subSending;
bool AckP__acking;

static inline error_t AckP__StdControl__start(void );
#line 105
static inline void AckP__setAck(message_t *msg, bool ack);
#line 123
static bool AckP__needsAck(message_t *msg);







static inline bool AckP__isAck(message_t *msg);







static inline bool AckP__matchAck(message_t *msg);
#line 151
static inline error_t AckP__sendAck(message_t *msg);
#line 207
static void AckP__sendDone(error_t error);









static void AckP__SubSend__sendDone(message_t *msg, error_t error);
#line 263
static inline error_t AckP__Send__send(message_t *msg, uint8_t len);
#line 297
static inline message_t *AckP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 327
static inline void AckP__AckTimer__fired(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void );
# 47 "/home/tinyos/tinyos-2.x/tos/lib/timer/BusyWaitCounterC.nc"
enum /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0____nesc_unnamed4283 {

  BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE = (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type )1 << (8 * sizeof(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type ) - 1)
};

static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt);
#line 72
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 38 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void );
#line 53
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 55 "/home/tinyos/tinyos-2.x/tos/lib/timer/BusyWait.nc"
static void HalChipconControlP__BusyWait__wait(HalChipconControlP__BusyWait__size_type dt);
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HalChipconControlP__rxDone__postTask(void );
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void HalChipconControlP__SO__makeInput(void );
#line 36
static bool HalChipconControlP__SO__get(void );
#line 36
static bool HalChipconControlP__CSn__get(void );


static void HalChipconControlP__CSn__makeOutput(void );
#line 33
static void HalChipconControlP__CSn__set(void );
static void HalChipconControlP__CSn__clr(void );


static void HalChipconControlP__G2__makeInput(void );
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t HalChipconControlP__G0Interrupt__disable(void );
#line 43
static error_t HalChipconControlP__G0Interrupt__enableFallingEdge(void );
# 50 "/home/tinyos/tinyos-2.x/tos/lib/timer/LocalTime.nc"
static uint32_t HalChipconControlP__LocalTime__get(void );
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HalChipconControlP__txDone__postTask(void );
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t HalChipconControlP__ActiveMessageAddress__amAddress(void );
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void HalChipconControlP__G0__makeInput(void );
#line 36
static bool HalChipconControlP__G0__get(void );
# 110 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t HalChipconControlP__SpiResource__release(void );
#line 87
static error_t HalChipconControlP__SpiResource__immediateRequest(void );
#line 118
static bool HalChipconControlP__SpiResource__isOwner(void );
# 81 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool HalChipconControlP__TxTimer__isRunning(void );
#line 62
static void HalChipconControlP__TxTimer__startOneShot(uint32_t dt);




static void HalChipconControlP__TxTimer__stop(void );
# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void HalChipconControlP__Clock__makeOutput(void );
#line 39
static void HalChipconControlP__SI__makeOutput(void );
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpi.nc"
static void HalChipconControlP__HplChipconSpi__writeRegister(uint8_t reg, uint8_t value);





static void HalChipconControlP__HplChipconSpi__read(uint8_t addr, uint8_t *buffer, uint8_t len);
#line 30
static uint8_t HalChipconControlP__HplChipconSpi__strobe(uint8_t cmd);
#line 52
static void HalChipconControlP__HplChipconSpi__write(uint8_t addr, uint8_t *buffer, uint8_t len);
#line 35
static uint8_t HalChipconControlP__HplChipconSpi__readRegister(uint8_t reg);
# 74 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static void HalChipconControlP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
#line 67
static void HalChipconControlP__HalChipconControl__txStart(uint32_t timestamp);
#line 81
static void HalChipconControlP__HalChipconControl__rxWaiting(uint32_t timestamp);
# 257 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
enum HalChipconControlP____nesc_unnamed4284 {
#line 257
  HalChipconControlP__taskSetAddress = 4U
};
#line 257
typedef int HalChipconControlP____nesc_sillytask_taskSetAddress[HalChipconControlP__taskSetAddress];
#line 269
enum HalChipconControlP____nesc_unnamed4285 {
#line 269
  HalChipconControlP__txDone = 5U
};
#line 269
typedef int HalChipconControlP____nesc_sillytask_txDone[HalChipconControlP__txDone];
#line 324
enum HalChipconControlP____nesc_unnamed4286 {
#line 324
  HalChipconControlP__rxDone = 6U
};
#line 324
typedef int HalChipconControlP____nesc_sillytask_rxDone[HalChipconControlP__rxDone];
#line 64
enum HalChipconControlP__Timings {
  HalChipconControlP__CALIBRATION_TIME = 721, 
  HalChipconControlP__OFF_TO_IDLE_TIME = 300, 
  HalChipconControlP__IDLE_TO_RXTX_TIME = 89, 
  HalChipconControlP__RX_TO_TX_TIME = 10
};








enum HalChipconControlP__States {
  HalChipconControlP__STATE_IDLE, 
  HalChipconControlP__STATE_RX, 
  HalChipconControlP__STATE_TX, 
  HalChipconControlP__STATE_FSTXON, 
  HalChipconControlP__STATE_CALIBRATE, 
  HalChipconControlP__STATE_SETTLING, 
  HalChipconControlP__STATE_RXFIFO_OVERFLOW, 
  HalChipconControlP__STATE_TXFIFO_UNDERFLOW, 
  HalChipconControlP__STATE_OFF
};

enum HalChipconControlP____nesc_unnamed4287 {
  HalChipconControlP__FIFO_SIZE = 64
};


bool HalChipconControlP__transmitting = FALSE;


uint8_t HalChipconControlP__pending = 0;


uint32_t HalChipconControlP__rxTimeStamp;
uint32_t HalChipconControlP__txTimeStamp;


cc_hal_status_t HalChipconControlP__status;





static inline uint8_t HalChipconControlP__strobe(uint8_t cmd);







static uint8_t HalChipconControlP__getChipState(void );
#line 136
static inline void HalChipconControlP__waitForChipReady(void );







static uint8_t HalChipconControlP__readRegister(uint8_t reg);
#line 157
static inline void HalChipconControlP__writeRegister(uint8_t reg, uint8_t value);







static inline void HalChipconControlP__reset(void );
#line 192
static inline void HalChipconControlP__checkChipVersion(void );
#line 207
static inline void HalChipconControlP__calibrate(void );







static inline void HalChipconControlP__configure(void );
#line 229
static inline void HalChipconControlP__listen(void );







static inline void HalChipconControlP__waitForRssiValid(void );










static void HalChipconControlP__setAddress(void );








static inline void HalChipconControlP__taskSetAddress__runTask(void );
#line 269
static inline void HalChipconControlP__txDone__runTask(void );
#line 324
static inline void HalChipconControlP__rxDone__runTask(void );
#line 407
static inline void HalChipconControlP__G0Interrupt__fired(void );
#line 426
static inline error_t HalChipconControlP__Init__init(void );








static inline void HalChipconControlP__HalChipconControl__init(void );
#line 477
static void HalChipconControlP__HalChipconControl__on(void );
#line 493
static inline void HalChipconControlP__HalChipconControl__off(void );
#line 525
static error_t HalChipconControlP__HalChipconControl__tx(void );
#line 626
static inline void HalChipconControlP__HalChipconControl__write(uint8_t *buffer, uint8_t length);
#line 656
static inline error_t HalChipconControlP__HalChipconControl__read(uint8_t *buffer);
#line 791
static inline void HalChipconControlP__TxTimer__fired(void );
#line 814
static inline void HalChipconControlP__SpiResource__granted(void );
# 82 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplChipconSpiB0P__UCLK__selectModuleFunc(void );
#line 82
static void HplChipconSpiB0P__SOMI__selectModuleFunc(void );
#line 82
static void HplChipconSpiB0P__SIMO__selectModuleFunc(void );
# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static inline error_t HplChipconSpiB0P__Init__init(void );
#line 80
static uint8_t HplChipconSpiB0P__HplChipconSpi__strobe(uint8_t strobe);








static uint8_t HplChipconSpiB0P__HplChipconSpi__readRegister(uint8_t reg);










static void HplChipconSpiB0P__HplChipconSpi__writeRegister(uint8_t reg, uint8_t value);







static inline void HplChipconSpiB0P__HplChipconSpi__read(uint8_t addr, uint8_t *buffer, uint8_t len);
#line 137
static inline void HplChipconSpiB0P__HplChipconSpi__write(uint8_t addr, uint8_t *buffer, uint8_t len);
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
static void DedicatedResourceC__Resource__granted(void );
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/DedicatedResourceC.nc"
enum DedicatedResourceC____nesc_unnamed4288 {
#line 37
  DedicatedResourceC__grant = 7U
};
#line 37
typedef int DedicatedResourceC____nesc_sillytask_grant[DedicatedResourceC__grant];
#line 35
bool DedicatedResourceC__owned;

static inline void DedicatedResourceC__grant__runTask(void );
#line 51
static inline error_t DedicatedResourceC__Resource__immediateRequest(void );








static inline error_t DedicatedResourceC__Resource__release(void );









static inline bool DedicatedResourceC__Resource__isOwner(void );
# 59 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static error_t SendP__HalChipconControl__tx(void );
#line 128
static void SendP__HalChipconControl__write(uint8_t *buffer, uint8_t length);
# 52 "/home/tinyos/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static SendP__PacketTimeStamp__size_type SendP__PacketTimeStamp__timestamp(
#line 41
message_t * msg);
#line 67
static void SendP__PacketTimeStamp__set(
#line 62
message_t * msg, 




SendP__PacketTimeStamp__size_type value);
# 81 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool SendP__BackoffTimer__isRunning(void );
#line 62
static void SendP__BackoffTimer__startOneShot(uint32_t dt);




static void SendP__BackoffTimer__stop(void );
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void SendP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/home/tinyos/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t SendP__Random__rand16(void );
# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void SendP__PreambleTimer__startOneShot(uint32_t dt);
# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
static void SendP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len);
#line 33
static chipcon_header_t *SendP__ChipconPacket__getHeader(message_t *msg);


static uint8_t SendP__ChipconPacket__getPacketLength(message_t *msg);
#line 35
static chipcon_metadata_t *SendP__ChipconPacket__getMetadata(message_t *msg);
# 30 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendNotify.nc"
static void SendP__SendNotify__sending(message_t *msg);
# 59 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
enum SendP__Backoff {
  SendP__CCA_BACKOFF = CCA_SETTLING_TIME, 
  SendP__END_OF_PACKET_BACKOFF = ((PREAMBLE_BYTES + SYNC_BYTES + (uint8_t )(sizeof(chipcon_header_t ) + 28U + sizeof(chipcon_footer_t ))) * 8UL * 1024 + 512) / (38400UL / 2), 
  SendP__INITIAL_RANDOM_BACKOFF = 4, 
  SendP__MAX_RANDOM_BACKOFF = 16
};

message_t *SendP__message;
bool SendP__busy;
uint16_t SendP__backoffLimit;


int16_t SendP__packetDurationAdjustment;





static void SendP__backoff(uint16_t fixed);
#line 89
static void SendP__writeData(void );





static void SendP__attemptTx(void );
#line 131
static inline error_t SendP__sendAck(void );
#line 154
static inline void SendP__BackoffTimer__fired(void );




static inline void SendP__PreambleTimer__fired(void );







static inline bool SendP__isAck(message_t *msg);




static inline error_t SendP__Send__send(message_t *msg, uint8_t len);
#line 200
static inline void SendP__HalChipconControl__txStart(uint32_t timestamp);
#line 214
static void SendP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
#line 235
static inline error_t SendP__Send__cancel(message_t *msg);










static inline uint8_t SendP__Send__maxPayloadLength(void );
#line 263
static inline void SendP__HalChipconControl__rxWaiting(uint32_t timestamp);





static inline error_t SendP__StdControl__start(void );
#line 283
static inline void SendP__SendNotify__default__sending(message_t *msg);
# 41 "/home/tinyos/tinyos-2.x/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 58
static uint32_t RandomMlcgC__Random__rand32(void );
#line 78
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void ReceiveP__PacketTimeStamp__set(
#line 62
message_t * msg, 




ReceiveP__PacketTimeStamp__size_type value);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t ReceiveP__Packet__payloadLength(
#line 63
message_t * msg);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



ReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
static chipcon_header_t *ReceiveP__ChipconPacket__getHeader(message_t *msg);


static uint8_t ReceiveP__ChipconPacket__getPacketLength(message_t *msg);
#line 35
static chipcon_metadata_t *ReceiveP__ChipconPacket__getMetadata(message_t *msg);
# 137 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static error_t ReceiveP__HalChipconControl__read(uint8_t *buffer);
# 45 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/ReceiveP.nc"
message_t ReceiveP__rxBuffer;
message_t *ReceiveP__message = &ReceiveP__rxBuffer;

static inline void ReceiveP__HalChipconControl__rxWaiting(uint32_t timestamp);
#line 78
static inline void ReceiveP__HalChipconControl__txStart(uint32_t timestamp);
static inline void ReceiveP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
static void PowerP__HalChipconControl__on(void );
# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/PowerP.nc"
bool PowerP__on = FALSE;






static inline error_t PowerP__StdControl__start(void );
#line 62
static inline void PowerP__HalChipconControl__txStart(uint32_t timestamp);
static inline void PowerP__HalChipconControl__txDone(uint32_t timestamp, error_t error);
static inline void PowerP__HalChipconControl__rxWaiting(uint32_t timestamp);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t SendReceiveP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 78
static error_t SendReceiveP__SubSend__cancel(
#line 74
message_t * msg);
#line 89
static void SendReceiveP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t SendReceiveP__SubControl__start(void );
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



SendReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendReceiveP.nc"
bool SendReceiveP__on = FALSE;
message_t *SendReceiveP__rxBuffer;





static inline error_t SendReceiveP__StdControl__start(void );
#line 90
static error_t SendReceiveP__Send__send(message_t *msg, uint8_t len);
#line 106
static inline void SendReceiveP__SubSend__sendDone(message_t *msg, error_t error);






static inline message_t *SendReceiveP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 125
static inline error_t SendReceiveP__Send__cancel(message_t *msg);
# 99 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/home/tinyos/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err);
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(
# 40 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40c080a0, 
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(
# 38 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40c096a8, 
# 85 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(
#line 63
message_t * msg);
#line 83
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void );
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(
#line 132
message_t * amsg);
# 118 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4289 {
#line 118
  AMQueueImplP__0__CancelTask = 8U
};
#line 118
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask];
#line 161
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4290 {
#line 161
  AMQueueImplP__0__errorTask = 9U
};
#line 161
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask];
#line 49
#line 47
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP__0____nesc_unnamed4291 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[1 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void );
#line 82
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 118
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void );
#line 155
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void );
#line 181
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err);
#line 207
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err);
# 390 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 185 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void )
{
}

# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 71
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow();
#line 71
}
#line 71
# 53 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x40444448){
#line 28
  switch (arg_0x40444448) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x40444448);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4292 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4293 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4294 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 103 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 47 "/home/tinyos/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 166 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
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

# 47 "/home/tinyos/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__1__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
}
#line 71
# 53 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
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

# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4295 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 86 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
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

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 70 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 43 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 119 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 154 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 70 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4296 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4297 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4298 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4299 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4300 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4301 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 120 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__default__fired();
#line 57
}
#line 57
# 93 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 93
{
#line 93
  P1IFG &= ~(1 << 0);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port10__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt10*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__default__fired();
#line 57
}
#line 57
# 94 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 94
{
#line 94
  P1IFG &= ~(1 << 1);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port11__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt11*/Msp430InterruptC__1__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__default__fired();
#line 57
}
#line 57
# 95 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 95
{
#line 95
  P1IFG &= ~(1 << 2);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port12__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt12*/Msp430InterruptC__2__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__default__fired();
#line 57
}
#line 57
# 96 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 96
{
#line 96
  P1IFG &= ~(1 << 3);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port13__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt13*/Msp430InterruptC__3__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__default__fired();
#line 57
}
#line 57
# 97 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 97
{
#line 97
  P1IFG &= ~(1 << 4);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port14__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt14*/Msp430InterruptC__4__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__default__fired();
#line 57
}
#line 57
# 98 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 98
{
#line 98
  P1IFG &= ~(1 << 5);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port15__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt15*/Msp430InterruptC__5__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__default__fired();
#line 57
}
#line 57
# 99 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 99
{
#line 99
  P1IFG &= ~(1 << 6);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port16__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt16*/Msp430InterruptC__6__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__default__fired();
#line 57
}
#line 57
# 100 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 100
{
#line 100
  P1IFG &= ~(1 << 7);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port17__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt17*/Msp430InterruptC__7__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__default__fired();
#line 57
}
#line 57
# 197 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 197
{
#line 197
  P2IFG &= ~(1 << 0);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port20__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt20*/Msp430InterruptC__8__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__default__fired();
#line 57
}
#line 57
# 198 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 198
{
#line 198
  P2IFG &= ~(1 << 1);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port21__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt21*/Msp430InterruptC__9__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__default__fired();
#line 57
}
#line 57
# 199 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 199
{
#line 199
  P2IFG &= ~(1 << 2);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port22__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt22*/Msp430InterruptC__10__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__default__fired();
#line 57
}
#line 57
# 200 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 200
{
#line 200
  P2IFG &= ~(1 << 3);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port23__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt23*/Msp430InterruptC__11__HplInterrupt__fired();
#line 61
}
#line 61
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t HalChipconControlP__rxDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(HalChipconControlP__rxDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t HalChipconControlP__txDone__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(HalChipconControlP__txDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 42 "/home/tinyos/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get(void )
{
  return /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__get();
}

# 50 "/home/tinyos/tinyos-2.x/tos/lib/timer/LocalTime.nc"
inline static uint32_t HalChipconControlP__LocalTime__get(void ){
#line 50
  unsigned long __nesc_result;
#line 50

#line 50
  __nesc_result = /*LocalTime32khzC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__get();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 407 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__G0Interrupt__fired(void )
#line 407
{


  uint32_t time = HalChipconControlP__LocalTime__get();

#line 411
  HalChipconControlP__pending++;
  if (HalChipconControlP__transmitting) {
      HalChipconControlP__txTimeStamp = time;
      HalChipconControlP__txDone__postTask();
    }
  else 
#line 415
    {
      HalChipconControlP__rxTimeStamp = time;
      HalChipconControlP__rxDone__postTask();
    }
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__fired(void ){
#line 57
  HalChipconControlP__G0Interrupt__fired();
#line 57
}
#line 57
# 201 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 201
{
#line 201
  P2IFG &= ~(1 << 4);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port24__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__fired();
#line 61
}
#line 61
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53







inline static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__default__fired();
#line 57
}
#line 57
# 202 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 202
{
#line 202
  P2IFG &= ~(1 << 5);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port25__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt25*/Msp430InterruptC__13__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__default__fired();
#line 57
}
#line 57
# 203 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 203
{
#line 203
  P2IFG &= ~(1 << 6);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port26__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt26*/Msp430InterruptC__14__HplInterrupt__fired();
#line 61
}
#line 61
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__default__fired();
#line 57
}
#line 57
# 204 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 204
{
#line 204
  P2IFG &= ~(1 << 7);
}

# 41 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port27__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__fired(void )
#line 66
{
  /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__clear();
  /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__Interrupt__fired();
}

# 61 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  /*Msp430GeneralIOC.GpioInterrupt27*/Msp430InterruptC__15__HplInterrupt__fired();
#line 61
}
#line 61
# 113 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/home/tinyos/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 58 "/home/tinyos/tinyos-2.x/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 77 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/StackGuardWatchDogP.nc"
static inline void StackGuardWatchDogP__stackProtectionInit(void )
#line 77
{
  uint16_t *address;


  uint16_t p = (uint16_t )&_end;

#line 82
  StackGuardWatchDogP__stackGuard = (uint16_t *)(p + (p & 1));


  for (address = StackGuardWatchDogP__stackGuard; address < (uint16_t *)StackGuardWatchDogP____stackptr; address++) {
      *address = 0x5aa5;
    }


  *StackGuardWatchDogP__stackGuard = 0xcf55;
}

#line 119
static inline void StackGuardWatchDogP__resetWatchdog(void )
#line 119
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 120
    WDTCTL = 0x5A00 + 0x0008 + 0x0004;
#line 120
    __nesc_atomic_end(__nesc_atomic); }
}

static inline error_t StackGuardWatchDogP__Init__init(void )
#line 123
{

  StackGuardWatchDogP__resetWatchdog();
  StackGuardWatchDogP__stackProtectionInit();
  return SUCCESS;
}

# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GeneralIOP.nc"
static inline error_t Msp430GeneralIOP__Init__init(void )
#line 37
{
  /* atomic removed: atomic calls only */
#line 38
  {

    P1SEL = 0;
    P1DIR = 0;
    P1OUT = 0;
    P1IE = 0;



    P2SEL = 0;
    P2DIR = 0;
    P2OUT = 0;
    P2IE = 0;



    P3SEL = 0;
    P3DIR = 0;
    P3OUT = 0;



    P4SEL = 0;
    P4DIR = 0;
    P4OUT = 0;



    P5SEL = 0;
    P5DIR = 0;
    P5OUT = 0;



    P6SEL = 0;
    P6DIR = 0;
    P6OUT = 0;
  }


  return SUCCESS;
}

# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformPinsP__DefaultInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = Msp430GeneralIOP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformPinsP.nc"
static inline error_t PlatformPinsP__Init__init(void )
#line 36
{
  PlatformPinsP__DefaultInit__init();


  P4DIR = 0xB0;
  P4OUT = 0xB0;

  return SUCCESS;
}

# 165 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 153
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 117
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 147
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 102 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = (0x0200 | 0x0002) | 0;
}

#line 142
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 81 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & (((0x08 | 0x04) | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 137
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 183 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TAR;
    }

  return dco_curr - dco_prev;
}




static inline int Msp430ClockP__busyCalibrateDco(void )
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
  return calib;
}

#line 65
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;



  BCSCTL1 = (0x80 | 0x08) | 0x04;

  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 132
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 29 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 29
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 29
}
#line 29
# 233 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__busyWait(uint16_t delay)
#line 233
{
  uint16_t i;

#line 235
  for (i = 0; i < delay; i++) {
       __asm volatile ("nop\n");}
}


static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {

    int previous = 0;
    int current = 0;



    Msp430ClockP__busyWait(10000);

    do {
        Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
        previous = current;
        current = Msp430ClockP__busyCalibrateDco();
      }
    while (
#line 262
    current != previous);

    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 52 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )41U &= ~(0x01 << 1);
}

# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__clr(void ){
#line 43
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr();
#line 43
}
#line 43
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__clr(void )
#line 41
{
#line 41
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__clr();
}

# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 34
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__clr();
#line 34
}
#line 34
# 52 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )41U &= ~(0x01 << 0);
}

# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__clr(void ){
#line 43
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr();
#line 43
}
#line 43
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__clr(void )
#line 41
{
#line 41
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__clr();
}

# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 34
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__clr();
#line 34
}
#line 34
# 52 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )33U &= ~(0x01 << 7);
}

# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 43
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr();
#line 43
}
#line 43
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__clr(void )
#line 41
{
#line 41
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 34
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__clr();
#line 34
}
#line 34
# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 58
  * (volatile uint8_t * )42U |= 0x01 << 1;
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 75
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput();
#line 75
}
#line 75
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 46
{
#line 46
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 39
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 39
}
#line 39
# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 58
  * (volatile uint8_t * )42U |= 0x01 << 0;
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__makeOutput(void ){
#line 75
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput();
#line 75
}
#line 75
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__makeOutput(void )
#line 46
{
#line 46
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__makeOutput();
}

# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 39
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__makeOutput();
#line 39
}
#line 39
# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 58
  * (volatile uint8_t * )34U |= 0x01 << 7;
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 75
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput();
#line 75
}
#line 75
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 46
{
#line 46
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 39
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 39
}
#line 39
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 49
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__clr();
    LedsP__Led1__clr();
    LedsP__Led2__clr();
  }
  return SUCCESS;
}

# 60 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 60
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 82 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplChipconSpiB0P__UCLK__selectModuleFunc(void ){
#line 82
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 82
}
#line 82
# 60 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 60
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 82 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplChipconSpiB0P__SOMI__selectModuleFunc(void ){
#line 82
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 82
}
#line 82
# 60 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 60
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 82 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplChipconSpiB0P__SIMO__selectModuleFunc(void ){
#line 82
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 82
}
#line 82
# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static inline error_t HplChipconSpiB0P__Init__init(void )
#line 51
{

  UCB0CTL1 |= 0x01;


  UCB0CTL0 = 0x01 + 0x08 + 0x20 + 0x80;


  UCB0CTL1 = 0x80 + 0x01;



  UCB0BR0 = 2;
  UCB0BR1 = 0;


  UCB0CTL1 &= ~0x01;


  IE2 &= ~(0x08 | 0x04);


  HplChipconSpiB0P__SIMO__selectModuleFunc();
  HplChipconSpiB0P__SOMI__selectModuleFunc();
  HplChipconSpiB0P__UCLK__selectModuleFunc();

  return SUCCESS;
}

# 45 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
static inline error_t PlatformP__InitLevel__default__init(uint8_t level)
#line 45
{
#line 45
  return SUCCESS;
}

# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__InitLevel__init(uint8_t arg_0x403e0430){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  switch (arg_0x403e0430) {
#line 51
    case PLATFORM_INIT_CLOCKS:
#line 51
      __nesc_result = Msp430ClockP__Init__init();
#line 51
      break;
#line 51
    case PLATFORM_INIT_GPIO:
#line 51
      __nesc_result = PlatformPinsP__Init__init();
#line 51
      break;
#line 51
    case PLATFORM_INIT_GPIO + 1:
#line 51
      __nesc_result = HplChipconSpiB0P__Init__init();
#line 51
      __nesc_result = ecombine(__nesc_result, LedsP__Init__init());
#line 51
      break;
#line 51
    case PLATFORM_INIT_WDT:
#line 51
      __nesc_result = StackGuardWatchDogP__Init__init();
#line 51
      break;
#line 51
    default:
#line 51
      __nesc_result = PlatformP__InitLevel__default__init(arg_0x403e0430);
#line 51
      break;
#line 51
    }
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 36
{
  uint16_t level;

#line 38
  for (level = 0; level < 256; level++) {
      PlatformP__InitLevel__init(level);
    }

  return SUCCESS;
}

# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 54 "/home/tinyos/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 24 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__AMSend__sendDone(message_t *msg, error_t error)
#line 24
{
  if (&BlinkToRadioC__pkt == msg) {
      BlinkToRadioC__busy = FALSE;
    }
}

# 99 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(message_t * msg, error_t error){
#line 99
  BlinkToRadioC__AMSend__sendDone(msg, error);
#line 99
}
#line 99
# 57 "/home/tinyos/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(message_t *m, error_t err)
#line 57
{
  /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__sendDone(m, err);
}

# 207 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(uint8_t arg_0x40c096a8, message_t * msg, error_t error){
#line 89
  switch (arg_0x40c096a8) {
#line 89
    case 0U:
#line 89
      /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__default__sendDone(arg_0x40c096a8, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 118 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask(void )
#line 118
{
  uint8_t i;
#line 119
  uint8_t j;
#line 119
  uint8_t mask;
#line 119
  uint8_t last;
  message_t *msg;

#line 121
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 161
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask(void )
#line 161
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg, FAIL);
}

# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(am_id_t arg_0x40c080a0, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = ChipconActiveMessageP__AMSend__send(arg_0x40c080a0, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 70 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline uint8_t ChipconPacketP__ChipconPacket__getPayloadLength(message_t *msg)
#line 70
{

  return ChipconPacketP__ChipconPacket__getPacketLength(msg) - (sizeof(chipcon_header_t ) + sizeof(chipcon_footer_t ));
}

#line 86
static inline uint8_t ChipconPacketP__Packet__payloadLength(message_t *msg)
#line 86
{
  return ChipconPacketP__ChipconPacket__getPayloadLength(msg);
}

# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = ChipconPacketP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(message_t * amsg){
#line 67
  unsigned long __nesc_result;
#line 67

#line 67
  __nesc_result = ChipconPacketP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline chipcon_header_t *ChipconPacketP__ChipconPacket__getHeader(message_t *msg)
#line 53
{
  return (chipcon_header_t *)(msg->data - sizeof(chipcon_header_t ));
}

#line 155
static inline am_id_t ChipconPacketP__AMPacket__type(message_t *amsg)
#line 155
{
  chipcon_header_t *header = ChipconPacketP__ChipconPacket__getHeader(amsg);

#line 157
  return __nesc_ntoh_uint8(header->type.nxdata);
}

# 136 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = ChipconPacketP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 57 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 70
    /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
    }
}

#line 166
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend(void )
#line 166
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__payloadLength(nextMsg);

#line 174
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__postTask();
        }
    }
}

# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/NetMask.nc"
inline static am_netmask_t ChipconPacketP__NetMask__netMask(void ){
#line 33
  unsigned long __nesc_result;
#line 33

#line 33
  __nesc_result = ActiveMessageAddressC__NetMask__netMask();
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 55 "/home/tinyos/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t ChipconPacketP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned long __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 49 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline am_addr_t ChipconPacketP__getLocalBroadcastAddress(void )
#line 49
{
  return ChipconPacketP__ActiveMessageAddress__amGroup() | ~ChipconPacketP__NetMask__netMask();
}

# 151 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void ChipconActiveMessageP__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  ChipconPacketP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void ChipconActiveMessageP__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  ChipconPacketP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 363 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_leuint32(void * target, uint32_t value)
#line 363
{
  uint8_t *base = target;

#line 365
  base[0] = value;
  base[1] = value >> 8;
  base[2] = value >> 16;
  base[3] = value >> 24;
  return value;
}

# 139 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline void ChipconPacketP__AMPacket__setSource(message_t *amsg, am_addr_t addr)
#line 139
{
  chipcon_header_t *header = ChipconPacketP__ChipconPacket__getHeader(amsg);

#line 141
  __nesc_hton_leuint32(header->src.nxdata, addr);
}

# 110 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void ChipconActiveMessageP__AMPacket__setSource(message_t * amsg, am_addr_t addr){
#line 110
  ChipconPacketP__AMPacket__setSource(amsg, addr);
#line 110
}
#line 110
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t ChipconPacketP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned long __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 106 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline am_addr_t ChipconPacketP__AMPacket__address(void )
#line 106
{
  return ChipconPacketP__ActiveMessageAddress__amAddress();
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t ChipconActiveMessageP__AMPacket__address(void ){
#line 57
  unsigned long __nesc_result;
#line 57

#line 57
  __nesc_result = ChipconPacketP__AMPacket__address();
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57
# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t AckP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = SendReceiveP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static void AckP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len){
#line 38
  ChipconPacketP__ChipconPacket__setPayloadLength(msg, len);
#line 38
}
#line 38
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/RebootC.nc"
static inline void RebootC__Reboot__reboot(void )
#line 35
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 36
    WDTCTL = 0;
#line 36
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/Reboot.nc"
inline static void AssertRebootP__Reboot__reboot(void ){
#line 34
  RebootC__Reboot__reboot();
#line 34
}
#line 34
# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/AssertRebootP.nc"
static inline  void doAssert(bool condition, uint16_t errorCode)
#line 39
{
  if (!condition) {
#line 40
    AssertRebootP__Reboot__reboot();
    }
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

# 61 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline chipcon_metadata_t *ChipconPacketP__ChipconPacket__getMetadata(message_t *msg)
#line 61
{
  return (chipcon_metadata_t *)msg->metadata;
}

# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static chipcon_metadata_t *AckP__ChipconPacket__getMetadata(message_t *msg){
#line 35
  nx_struct chipcon_metadata_t *__nesc_result;
#line 35

#line 35
  __nesc_result = ChipconPacketP__ChipconPacket__getMetadata(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
#line 33
inline static chipcon_header_t *AckP__ChipconPacket__getHeader(message_t *msg){
#line 33
  nx_struct chipcon_header_t *__nesc_result;
#line 33

#line 33
  __nesc_result = ChipconPacketP__ChipconPacket__getHeader(msg);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 105 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static inline void AckP__setAck(message_t *msg, bool ack)
#line 105
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 106
  chipcon_header_t *header = AckP__ChipconPacket__getHeader(msg);
  chipcon_metadata_t *metadata = AckP__ChipconPacket__getMetadata(msg);

#line 108
  if (ack) {

      (__nesc_temp44 = header->flags.nxdata, __nesc_hton_uint8(__nesc_temp44, __nesc_ntoh_uint8(__nesc_temp44) | FLAG_ACK_REQUEST));
    }
  else 
#line 111
    {

      (__nesc_temp45 = header->flags.nxdata, __nesc_hton_uint8(__nesc_temp45, __nesc_ntoh_uint8(__nesc_temp45) & ~FLAG_ACK_REQUEST));
    }


  __nesc_hton_uint8(metadata->ack.nxdata, TRUE);
}

#line 263
static inline error_t AckP__Send__send(message_t *msg, uint8_t len)
#line 263
{
  chipcon_metadata_t *metadata = AckP__ChipconPacket__getMetadata(msg);

  if (AckP__sending) {
#line 266
    return EBUSY;
    }
#line 267
  AckP__sending = TRUE;
  AckP__subSending = FALSE;

  if (! __nesc_ntoh_uint8(metadata->ack.nxdata)) {

      AckP__setAck(msg, TRUE);
    }

  doAssert(AckP__txMessage == (void *)0, ASSERT_CC_ACK_STATE);
  AckP__txMessage = msg;

  if (AckP__acking) {

      AckP__ChipconPacket__setPayloadLength(AckP__txMessage, len);
      return SUCCESS;
    }
  else 
#line 282
    {


      error_t error = AckP__SubSend__send(msg, len);

#line 286
      doAssertSuccess(error, ASSERT_CC_ACK_SEND);
      AckP__subSending = TRUE;
      return SUCCESS;
    }
}

# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t ChipconActiveMessageP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = AckP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 59 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static error_t SendP__HalChipconControl__tx(void ){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = HalChipconControlP__HalChipconControl__tx();
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 131 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline error_t SendP__sendAck(void )
#line 131
{
  error_t error;



  do {
      error = SendP__HalChipconControl__tx();
    }
  while (
#line 138
  error == ERETRY);

  if (error == SUCCESS) {
      SendP__writeData();
    }



  if (error == FAIL || error == EBUSY) {
      return FAIL;
    }


  return error;
}

# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static chipcon_header_t *SendP__ChipconPacket__getHeader(message_t *msg){
#line 33
  nx_struct chipcon_header_t *__nesc_result;
#line 33

#line 33
  __nesc_result = ChipconPacketP__ChipconPacket__getHeader(msg);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 167 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline bool SendP__isAck(message_t *msg)
#line 167
{
  chipcon_header_t *header = SendP__ChipconPacket__getHeader(msg);

#line 169
  return __nesc_ntoh_uint8(header->flags.nxdata) & FLAG_ACK_REPLY;
}

# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static void SendP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len){
#line 38
  ChipconPacketP__ChipconPacket__setPayloadLength(msg, len);
#line 38
}
#line 38
# 246 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline uint8_t SendP__Send__maxPayloadLength(void )
#line 246
{

  return 28U;
}

#line 172
static inline error_t SendP__Send__send(message_t *msg, uint8_t len)
#line 172
{
  error_t error;

  if (SendP__busy) {
#line 175
    return EBUSY;
    }
#line 176
  if (len > SendP__Send__maxPayloadLength()) {
#line 176
    return ESIZE;
    }
  SendP__message = msg;
  SendP__ChipconPacket__setPayloadLength(SendP__message, len);

  if (SendP__isAck(msg)) {

      error = SendP__sendAck();
    }
  else 
#line 184
    {

      SendP__backoffLimit = SendP__INITIAL_RANDOM_BACKOFF;
      SendP__attemptTx();


      error = SUCCESS;
    }

  if (error == SUCCESS) {
      SendP__busy = TRUE;
    }

  return error;
}

# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t SendReceiveP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = SendP__Send__send(msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 54 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw(void )
#line 54
{
#line 54
  return * (volatile uint8_t * )40U & (0x01 << 3);
}

#line 55
static inline bool /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get(void )
#line 55
{
#line 55
  return /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__getRaw() != 0;
}

# 63 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__get(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__get();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__get(void )
#line 43
{
#line 43
  return /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__get();
}

# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static bool HalChipconControlP__CSn__get(void ){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__get();
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 54 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void )
#line 54
{
#line 54
  return * (volatile uint8_t * )40U & (0x01 << 4);
}

#line 55
static inline bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void )
#line 55
{
#line 55
  return /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw() != 0;
}

# 63 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__HplGeneralIO__get(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__get(void )
#line 43
{
#line 43
  return /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__HplGeneralIO__get();
}

# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static bool HalChipconControlP__G0__get(void ){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__get();
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 237 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__waitForRssiValid(void )
#line 237
{



  uint8_t valid = (1 << 6) | (1 << 4);

#line 242
  while (!(HalChipconControlP__readRegister(PKTSTATUS) & valid) && HalChipconControlP__getChipState() == HalChipconControlP__STATE_RX) ;
}

# 51 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (0) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )368U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )368U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )368U;
    }
}

# 34 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
#line 34
  unsigned int __nesc_result;
#line 34

#line 34
  __nesc_result = /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get();
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 38 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__1__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 158 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 81 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static bool HalChipconControlP__TxTimer__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(3U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long __nesc_result;
#line 53

#line 53
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 75 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 98 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 148 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void HalChipconControlP__TxTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 62
}
#line 62
# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__clr(void ){
#line 43
  /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr();
#line 43
}
#line 43
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__clr(void )
#line 41
{
#line 41
  /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__clr();
}

# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__CSn__clr(void ){
#line 34
  /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__clr();
#line 34
}
#line 34
# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__set(void ){
#line 38
  /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__set();
#line 38
}
#line 38
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__set(void )
#line 40
{
#line 40
  /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__set();
}

# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__CSn__set(void ){
#line 33
  /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__set();
#line 33
}
#line 33
# 137 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static inline void HplChipconSpiB0P__HplChipconSpi__write(uint8_t addr, uint8_t *buffer, uint8_t len)
#line 137
{
  uint8_t i;

#line 139
  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = addr;
  for (i = 0; i < len; i++) {
      while (!(IFG2 & 0x08)) ;
      UCB0TXBUF = buffer[i];
    }

  while (UCB0STAT & 0x01) ;
}

# 52 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpi.nc"
inline static void HalChipconControlP__HplChipconSpi__write(uint8_t addr, uint8_t *buffer, uint8_t len){
#line 52
  HplChipconSpiB0P__HplChipconSpi__write(addr, buffer, len);
#line 52
}
#line 52
# 62 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/PowerP.nc"
static inline void PowerP__HalChipconControl__txStart(uint32_t timestamp)
#line 62
{
}

# 78 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/ReceiveP.nc"
static inline void ReceiveP__HalChipconControl__txStart(uint32_t timestamp)
#line 78
{
}

# 283 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline void SendP__SendNotify__default__sending(message_t *msg)
#line 283
{
}

# 30 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendNotify.nc"
inline static void SendP__SendNotify__sending(message_t *msg){
#line 30
  SendP__SendNotify__default__sending(msg);
#line 30
}
#line 30
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void SendP__PacketTimeStamp__set(message_t * msg, SendP__PacketTimeStamp__size_type value){
#line 67
  ChipconPacketP__PacketTimeStamp__set(msg, value);
#line 67
}
#line 67
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static uint8_t SendP__ChipconPacket__getPacketLength(message_t *msg){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = ChipconPacketP__ChipconPacket__getPacketLength(msg);
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 200 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline void SendP__HalChipconControl__txStart(uint32_t timestamp)
#line 200
{






  uint8_t bytes = PREAMBLE_BYTES + SYNC_BYTES + CRC_BYTES + SendP__ChipconPacket__getPacketLength(SendP__message);
  uint16_t bits = bytes * 8;
  uint32_t duration = (bits * 1024UL * 32 + 512) / (38400UL / 2) + SendP__packetDurationAdjustment;

#line 210
  SendP__PacketTimeStamp__set(SendP__message, timestamp + duration);
  SendP__SendNotify__sending(SendP__message);
}

# 67 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static void HalChipconControlP__HalChipconControl__txStart(uint32_t timestamp){
#line 67
  SendP__HalChipconControl__txStart(timestamp);
#line 67
  ReceiveP__HalChipconControl__txStart(timestamp);
#line 67
  PowerP__HalChipconControl__txStart(timestamp);
#line 67
}
#line 67
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpi.nc"
inline static void HalChipconControlP__HplChipconSpi__writeRegister(uint8_t reg, uint8_t value){
#line 40
  HplChipconSpiB0P__HplChipconSpi__writeRegister(reg, value);
#line 40
}
#line 40
# 626 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__HalChipconControl__write(uint8_t *buffer, uint8_t length)
#line 626
{
  doAssert(length < HalChipconControlP__FIFO_SIZE, ASSERT_CC_HAL_PACKET_TOO_LARGE);
  doAssert(length == buffer[0] + 1, ASSERT_CC_HAL_INVALID_LENGTH);


  HalChipconControlP__HplChipconSpi__writeRegister(TXFIFO, buffer[0]);


  HalChipconControlP__HalChipconControl__txStart(HalChipconControlP__LocalTime__get());


  HalChipconControlP__HplChipconSpi__write(TX_BURST_WRITE, buffer + 1, length - 1);


  HalChipconControlP__CSn__set();
  HalChipconControlP__CSn__clr();



  ;
  HalChipconControlP__TxTimer__startOneShot(3 * 8 * (uint8_t )(sizeof(chipcon_header_t ) + 28U + sizeof(chipcon_footer_t )) * 1024UL / (38400UL / 2));
}

# 128 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static void SendP__HalChipconControl__write(uint8_t *buffer, uint8_t length){
#line 128
  HalChipconControlP__HalChipconControl__write(buffer, length);
#line 128
}
#line 128
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

# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static chipcon_metadata_t *SendP__ChipconPacket__getMetadata(message_t *msg){
#line 35
  nx_struct chipcon_metadata_t *__nesc_result;
#line 35

#line 35
  __nesc_result = ChipconPacketP__ChipconPacket__getMetadata(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendP__PreambleTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(5U, dt);
#line 62
}
#line 62
# 78 "/home/tinyos/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 78
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 41 "/home/tinyos/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t SendP__Random__rand16(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendP__BackoffTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(4U, dt);
#line 62
}
#line 62
# 340 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 340
{
  const uint8_t *base = source;

#line 342
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 214 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline uint32_t ChipconPacketP__PacketTimeStamp__timestamp(message_t *msg)
#line 214
{
  chipcon_metadata_t *metadata = ChipconPacketP__ChipconPacket__getMetadata(msg);

#line 216
  return __nesc_ntoh_uint32(metadata->timestamp.nxdata);
}

# 52 "/home/tinyos/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static SendP__PacketTimeStamp__size_type SendP__PacketTimeStamp__timestamp(message_t * msg){
#line 52
  unsigned long __nesc_result;
#line 52

#line 52
  __nesc_result = ChipconPacketP__PacketTimeStamp__timestamp(msg);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t AckP__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = ChipconPacketP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 110 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline am_group_t ChipconPacketP__AMPacket__localGroup(void )
#line 110
{
  return ChipconPacketP__ActiveMessageAddress__amGroup();
}

# 356 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_ntoh_leuint32(const void * source)
#line 356
{
  const uint8_t *base = source;

#line 358
  return ((((uint32_t )base[3] << 24) | (
  (uint32_t )base[2] << 16)) | (
  (uint32_t )base[1] << 8)) | base[0];
}

# 165 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline am_group_t ChipconPacketP__AMPacket__group(message_t *amsg)
#line 165
{
  chipcon_header_t *header = ChipconPacketP__ChipconPacket__getHeader(amsg);

#line 167
  return __nesc_ntoh_leuint32(header->dest.nxdata) & ChipconPacketP__NetMask__netMask();
}

#line 144
static inline bool ChipconPacketP__AMPacket__isForMe(message_t *amsg)
#line 144
{






  return ChipconPacketP__AMPacket__group(amsg) == ChipconPacketP__AMPacket__localGroup() && (
  ChipconPacketP__AMPacket__destination(amsg) == ChipconPacketP__AMPacket__address() || ChipconPacketP__AMPacket__destination(amsg) == 0xFFFFFFFFUL);
}

# 125 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static bool ChipconActiveMessageP__AMPacket__isForMe(message_t * amsg){
#line 125
  unsigned char __nesc_result;
#line 125

#line 125
  __nesc_result = ChipconPacketP__AMPacket__isForMe(amsg);
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 310 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 310
{
  const uint8_t *base = source;

#line 312
  return ((uint16_t )base[0] << 8) | base[1];
}

# 101 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 101
{
  LedsP__Led2__clr();
  ;
#line 103
  ;
}

# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__set(void )
#line 51
{
  /* atomic removed: atomic calls only */
#line 51
  * (volatile uint8_t * )41U |= 0x01 << 1;
}

# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 38
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__set();
#line 38
}
#line 38
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__set(void )
#line 40
{
#line 40
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__set();
}

# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 33
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__set();
#line 33
}
#line 33
# 96 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 96
{
  LedsP__Led2__set();
  ;
#line 98
  ;
}

#line 86
static inline void LedsP__Leds__led1Off(void )
#line 86
{
  LedsP__Led1__clr();
  ;
#line 88
  ;
}

# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__set(void )
#line 51
{
  /* atomic removed: atomic calls only */
#line 51
  * (volatile uint8_t * )41U |= 0x01 << 0;
}

# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__set(void ){
#line 38
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__set();
#line 38
}
#line 38
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__set(void )
#line 40
{
#line 40
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__set();
}

# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 33
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__set();
#line 33
}
#line 33
# 81 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 81
{
  LedsP__Led1__set();
  ;
#line 83
  ;
}

#line 71
static inline void LedsP__Leds__led0Off(void )
#line 71
{
  LedsP__Led0__clr();
  ;
#line 73
  ;
}

# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void )
#line 51
{
  /* atomic removed: atomic calls only */
#line 51
  * (volatile uint8_t * )33U |= 0x01 << 7;
}

# 38 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 38
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set();
#line 38
}
#line 38
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__set(void )
#line 40
{
#line 40
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__set();
}

# 33 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 33
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__set();
#line 33
}
#line 33
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led0On(void )
#line 66
{
  LedsP__Led0__set();
  ;
#line 68
  ;
}

#line 128
static inline void LedsP__Leds__set(uint8_t val)
#line 128
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (val & LEDS_LED0) {
          LedsP__Leds__led0On();
        }
      else {
          LedsP__Leds__led0Off();
        }
      if (val & LEDS_LED1) {
          LedsP__Leds__led1On();
        }
      else {
          LedsP__Leds__led1Off();
        }
      if (val & LEDS_LED2) {
          LedsP__Leds__led2On();
        }
      else {
          LedsP__Leds__led2Off();
        }
    }
#line 148
    __nesc_atomic_end(__nesc_atomic); }
}

# 123 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkToRadioC__Leds__set(uint8_t val){
#line 123
  LedsP__Leds__set(val);
#line 123
}
#line 123
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t BlinkToRadioC__AMPacket__destination(message_t * amsg){
#line 67
  unsigned long __nesc_result;
#line 67

#line 67
  __nesc_result = ChipconPacketP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 42 "BlinkToRadioC.nc"
static inline message_t *BlinkToRadioC__Receive__receive(message_t *msg, void *payload, uint8_t len)
#line 42
{
  if (len == sizeof(BlinkToRadioMsg ) && BlinkToRadioC__AMPacket__destination(msg) == TOS_NODE_ID) {
      BlinkToRadioMsg *btrpkt = (BlinkToRadioMsg *)payload;

#line 45
      BlinkToRadioC__Leds__set(__nesc_ntoh_uint16(btrpkt->counter.nxdata) * 2);
    }
  return msg;
}

# 191 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static inline message_t *ChipconActiveMessageP__Receive__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 191
{
  return msg;
}

# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * ChipconActiveMessageP__Receive__receive(am_id_t arg_0x40a03470, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x40a03470) {
#line 67
    case 6:
#line 67
      __nesc_result = BlinkToRadioC__Receive__receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = ChipconActiveMessageP__Receive__default__receive(arg_0x40a03470, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 195 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static inline message_t *ChipconActiveMessageP__Snoop__default__receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 195
{
  return msg;
}

# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * ChipconActiveMessageP__Snoop__receive(am_id_t arg_0x40a03b08, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
    __nesc_result = ChipconActiveMessageP__Snoop__default__receive(arg_0x40a03b08, msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 115 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void * AckP__Packet__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = ChipconPacketP__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 94 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline uint8_t ChipconPacketP__Packet__maxPayloadLength(void )
#line 94
{
  return 28U;
}

# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void AckP__AckTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(2U, dt);
#line 62
}
#line 62
# 181 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current, msg, err);
    }
  else {
      ;
    }
}

# 99 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static void ChipconActiveMessageP__AMSend__sendDone(am_id_t arg_0x409d7a98, message_t * msg, error_t error){
#line 99
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__sendDone(arg_0x409d7a98, msg, error);
#line 99
}
#line 99
# 50 "/home/tinyos/tinyos-2.x/tos/system/CrcC.nc"
static inline uint16_t CrcC__Crc__crc16(void *buf, uint8_t len)
#line 50
{
  return CrcC__Crc__seededCrc16(0, buf, len);
}

# 42 "/home/tinyos/tinyos-2.x/tos/interfaces/Crc.nc"
inline static uint16_t ChipconActiveMessageP__Crc__crc16(void * buf, uint8_t len){
#line 42
  unsigned int __nesc_result;
#line 42

#line 42
  __nesc_result = CrcC__Crc__crc16(buf, len);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 113 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static inline uint16_t ChipconActiveMessageP__crcMessage(message_t *msg)
#line 113
{





  uint8_t len = sizeof(message_header_t ) + 28U + sizeof(message_footer_t );


  return ChipconActiveMessageP__Crc__crc16(msg, len);
}

#line 160
static inline void ChipconActiveMessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 160
{
  ;








  doAssertEquals(ChipconActiveMessageP__crc, ChipconActiveMessageP__crcMessage(msg), ASSERT_CC_AM_MODIFIED);

  ChipconActiveMessageP__txBusy = FALSE;
  ChipconActiveMessageP__AMSend__sendDone(ChipconActiveMessageP__AMPacket__type(msg), msg, result);
}

# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static void AckP__Send__sendDone(message_t * msg, error_t error){
#line 89
  ChipconActiveMessageP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 80 "/home/tinyos/tinyos-2.x/tos/system/crc.h"
static inline uint16_t crcByte(uint16_t crc, uint8_t b)
#line 80
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 814 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__SpiResource__granted(void )
#line 814
{
}

# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void DedicatedResourceC__Resource__granted(void ){
#line 92
  HalChipconControlP__SpiResource__granted();
#line 92
}
#line 92
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/DedicatedResourceC.nc"
static inline void DedicatedResourceC__grant__runTask(void )
#line 37
{
  DedicatedResourceC__Resource__granted();
}

# 64 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/PowerP.nc"
static inline void PowerP__HalChipconControl__rxWaiting(uint32_t timestamp)
#line 64
{
}

# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t ReceiveP__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = ChipconPacketP__Packet__payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * AckP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = ChipconActiveMessageP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static uint8_t AckP__ChipconPacket__getPayloadLength(message_t *msg){
#line 37
  unsigned char __nesc_result;
#line 37

#line 37
  __nesc_result = ChipconPacketP__ChipconPacket__getPayloadLength(msg);
#line 37

#line 37
  return __nesc_result;
#line 37
}
#line 37
# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static void SendReceiveP__Send__sendDone(message_t * msg, error_t error){
#line 89
  AckP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 106 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendReceiveP.nc"
static inline void SendReceiveP__SubSend__sendDone(message_t *msg, error_t error)
#line 106
{
  SendReceiveP__Send__sendDone(msg, error);
}

# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static void SendP__Send__sendDone(message_t * msg, error_t error){
#line 89
  SendReceiveP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 153 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void SendP__BackoffTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(4U);
#line 67
}
#line 67
#line 81
inline static bool SendP__BackoffTimer__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(4U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 235 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline error_t SendP__Send__cancel(message_t *msg)
#line 235
{
  if (msg == SendP__message && SendP__BackoffTimer__isRunning()) {
      SendP__BackoffTimer__stop();
      SendP__busy = FALSE;
      SendP__Send__sendDone(SendP__message, ECANCEL);
      return SUCCESS;
    }
  else 
#line 241
    {
      return FAIL;
    }
}

# 78 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t SendReceiveP__SubSend__cancel(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = SendP__Send__cancel(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 125 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendReceiveP.nc"
static inline error_t SendReceiveP__Send__cancel(message_t *msg)
#line 125
{
  return SendReceiveP__SubSend__cancel(msg);
}

# 78 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t AckP__SubSend__cancel(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = SendReceiveP__Send__cancel(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 151 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static inline error_t AckP__sendAck(message_t *msg)
#line 151
{
  unsigned char *__nesc_temp47;
  unsigned char *__nesc_temp46;
#line 152
  error_t error;



  chipcon_header_t *rxHeader = AckP__ChipconPacket__getHeader(msg);
  chipcon_header_t *ackHeader = AckP__ChipconPacket__getHeader(AckP__ackMessage);

#line 158
  *ackHeader = *rxHeader;
  __nesc_hton_leuint32(ackHeader->src.nxdata, __nesc_ntoh_leuint32(rxHeader->dest.nxdata));
  __nesc_hton_leuint32(ackHeader->dest.nxdata, __nesc_ntoh_leuint32(rxHeader->src.nxdata));
  (__nesc_temp46 = ackHeader->flags.nxdata, __nesc_hton_uint8(__nesc_temp46, __nesc_ntoh_uint8(__nesc_temp46) & ~FLAG_ACK_REQUEST));
  (__nesc_temp47 = ackHeader->flags.nxdata, __nesc_hton_uint8(__nesc_temp47, __nesc_ntoh_uint8(__nesc_temp47) | FLAG_ACK_REPLY));



  error = AckP__SubSend__send(AckP__ackMessage, 0);
  if (error == SUCCESS) {
      return SUCCESS;
    }
  else {
#line 169
    if (error == EBUSY) {

        if (AckP__SubSend__cancel(AckP__txMessage) == SUCCESS) {
            AckP__subSending = FALSE;
            ;
#line 173
            ;




            error = AckP__SubSend__send(AckP__ackMessage, 0);
            if (error == SUCCESS) {
                ;
#line 180
                ;
              }
            else 
#line 181
              {

                error_t resendError;

#line 184
                ;
#line 184
                ;


                resendError = AckP__SubSend__send(AckP__txMessage, AckP__ChipconPacket__getPayloadLength(AckP__txMessage));
                doAssertSuccess(resendError, ASSERT_CC_ACK_SEND);
                AckP__subSending = TRUE;
              }

            return error;
          }
        else 
#line 193
          {

            ;
#line 195
            ;
            return FAIL;
          }
      }
    else 
#line 198
      {
        ;
#line 199
        ;
        return error;
      }
    }
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static am_addr_t AckP__AMPacket__address(void ){
#line 57
  unsigned long __nesc_result;
#line 57

#line 57
  __nesc_result = ChipconPacketP__AMPacket__address();
#line 57

#line 57
  return __nesc_result;
#line 57
}
#line 57










inline static am_addr_t AckP__AMPacket__destination(message_t * amsg){
#line 67
  unsigned long __nesc_result;
#line 67

#line 67
  __nesc_result = ChipconPacketP__AMPacket__destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void AckP__AckTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 67
}
#line 67
#line 81
inline static bool AckP__AckTimer__isRunning(void ){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(2U);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 139 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static inline bool AckP__matchAck(message_t *msg)
#line 139
{
  chipcon_header_t *ackHeader = AckP__ChipconPacket__getHeader(msg);
  chipcon_header_t *txHeader = AckP__ChipconPacket__getHeader(AckP__txMessage);

#line 142
  doAssert(__nesc_ntoh_uint8(ackHeader->flags.nxdata) & FLAG_ACK_REPLY, ASSERT_CC_ACK_PACKET);
  return __nesc_ntoh_leuint32(
  ackHeader->dest.nxdata) == __nesc_ntoh_leuint32(txHeader->src.nxdata) && __nesc_ntoh_leuint32(
  ackHeader->src.nxdata) == __nesc_ntoh_leuint32(txHeader->dest.nxdata);
}

#line 131
static inline bool AckP__isAck(message_t *msg)
#line 131
{
  chipcon_header_t *header = AckP__ChipconPacket__getHeader(msg);

#line 133
  return __nesc_ntoh_uint8(header->flags.nxdata) & FLAG_ACK_REPLY;
}

#line 297
static inline message_t *AckP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 297
{
  if (AckP__isAck(msg)) {
      if (AckP__sending && AckP__matchAck(msg)) {
          if (AckP__AckTimer__isRunning()) {
              AckP__AckTimer__stop();
              __nesc_hton_uint8(AckP__ChipconPacket__getMetadata(AckP__txMessage)->ack.nxdata, TRUE);
              AckP__sendDone(SUCCESS);
            }
          else 
#line 304
            {

              ;
#line 306
              ;
            }
        }

      return msg;
    }
  else 
#line 311
    {
      if (AckP__needsAck(msg) && AckP__AMPacket__destination(msg) == AckP__AMPacket__address()) {
          if (AckP__sendAck(msg) == SUCCESS) {


              AckP__acking = TRUE;
              AckP__rxMessage = msg;
              return AckP__buffer;
            }
        }


      return AckP__Receive__receive(msg, payload, len);
    }
}

# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * SendReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = AckP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 113 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendReceiveP.nc"
static inline message_t *SendReceiveP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 113
{




  SendReceiveP__rxBuffer = SendReceiveP__Receive__receive(msg, payload, len);
  return SendReceiveP__rxBuffer;
}

# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  __nesc_result = SendReceiveP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/home/tinyos/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void ReceiveP__PacketTimeStamp__set(message_t * msg, ReceiveP__PacketTimeStamp__size_type value){
#line 67
  ChipconPacketP__PacketTimeStamp__set(msg, value);
#line 67
}
#line 67
# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static uint8_t ReceiveP__ChipconPacket__getPacketLength(message_t *msg){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = ChipconPacketP__ChipconPacket__getPacketLength(msg);
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 108 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static inline void HplChipconSpiB0P__HplChipconSpi__read(uint8_t addr, uint8_t *buffer, uint8_t len)
#line 108
{
  uint8_t i;

  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = addr;
  while (UCB0STAT & 0x01) ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      UCB0TXBUF = 0;
      IFG2 &= ~0x04;

      while (!(IFG2 & 0x04)) ;
    }
#line 120
    __nesc_atomic_end(__nesc_atomic); }


  for (i = 0; i < len - 1; i++) {



      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
        {
          UCB0TXBUF = 0;
          buffer[i] = UCB0RXBUF;
          while (!(IFG2 & 0x04)) ;
        }
#line 131
        __nesc_atomic_end(__nesc_atomic); }
    }

  buffer[len - 1] = UCB0RXBUF;
}

# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpi.nc"
inline static void HalChipconControlP__HplChipconSpi__read(uint8_t addr, uint8_t *buffer, uint8_t len){
#line 46
  HplChipconSpiB0P__HplChipconSpi__read(addr, buffer, len);
#line 46
}
#line 46
#line 30
inline static uint8_t HalChipconControlP__HplChipconSpi__strobe(uint8_t cmd){
#line 30
  unsigned char __nesc_result;
#line 30

#line 30
  __nesc_result = HplChipconSpiB0P__HplChipconSpi__strobe(cmd);
#line 30

#line 30
  return __nesc_result;
#line 30
}
#line 30
# 111 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline uint8_t HalChipconControlP__strobe(uint8_t cmd)
#line 111
{
  return HalChipconControlP__HplChipconSpi__strobe(cmd);
}

#line 229
static inline void HalChipconControlP__listen(void )
#line 229
{
  HalChipconControlP__strobe(SRX);
}

# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpi.nc"
inline static uint8_t HalChipconControlP__HplChipconSpi__readRegister(uint8_t reg){
#line 35
  unsigned char __nesc_result;
#line 35

#line 35
  __nesc_result = HplChipconSpiB0P__HplChipconSpi__readRegister(reg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
# 656 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline error_t HalChipconControlP__HalChipconControl__read(uint8_t *buffer)
#line 656
{
  uint8_t length;

  uint8_t state = HalChipconControlP__getChipState();
  uint8_t bytesInFifo = HalChipconControlP__readRegister(RXBYTES);
  uint8_t overflow = bytesInFifo & (1 << 7);

#line 662
  doAssert(state == HalChipconControlP__STATE_IDLE || state == HalChipconControlP__STATE_CALIBRATE, ASSERT_CC_HAL_NOT_IDLE);


  length = HalChipconControlP__HplChipconSpi__readRegister(RXFIFO);


  HalChipconControlP__listen();

  doAssertNot(overflow, ASSERT_CANT_HAPPEN);






  if ((length == 0 || length + 2 > bytesInFifo) || length + 1 > (uint8_t )(sizeof(chipcon_header_t ) + 28U + sizeof(chipcon_footer_t ))) {

      ;
#line 679
      ;
      HalChipconControlP__CSn__set();
      HalChipconControlP__CSn__clr();
      HalChipconControlP__strobe(SIDLE);
      HalChipconControlP__strobe(SFRX);
      HalChipconControlP__listen();
      return FAIL;
    }


  buffer[0] = length;
  HalChipconControlP__HplChipconSpi__read(RX_BURST_READ, &buffer[1], length + 2);


  HalChipconControlP__CSn__set();
  HalChipconControlP__CSn__clr();
  return SUCCESS;
}

# 137 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static error_t ReceiveP__HalChipconControl__read(uint8_t *buffer){
#line 137
  unsigned char __nesc_result;
#line 137

#line 137
  __nesc_result = HalChipconControlP__HalChipconControl__read(buffer);
#line 137

#line 137
  return __nesc_result;
#line 137
}
#line 137
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacket.nc"
inline static chipcon_metadata_t *ReceiveP__ChipconPacket__getMetadata(message_t *msg){
#line 35
  nx_struct chipcon_metadata_t *__nesc_result;
#line 35

#line 35
  __nesc_result = ChipconPacketP__ChipconPacket__getMetadata(msg);
#line 35

#line 35
  return __nesc_result;
#line 35
}
#line 35
#line 33
inline static chipcon_header_t *ReceiveP__ChipconPacket__getHeader(message_t *msg){
#line 33
  nx_struct chipcon_header_t *__nesc_result;
#line 33

#line 33
  __nesc_result = ChipconPacketP__ChipconPacket__getHeader(msg);
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/ReceiveP.nc"
static inline void ReceiveP__HalChipconControl__rxWaiting(uint32_t timestamp)
#line 48
{
  uint8_t *buffer = (uint8_t *)ReceiveP__ChipconPacket__getHeader(ReceiveP__message);
  chipcon_metadata_t *metadata = ReceiveP__ChipconPacket__getMetadata(ReceiveP__message);
  uint8_t length;


  error_t error = ReceiveP__HalChipconControl__read(buffer);

#line 55
  if (error == SUCCESS) {



      length = ReceiveP__ChipconPacket__getPacketLength(ReceiveP__message);
      __nesc_hton_uint8(metadata->crcAndLqi.nxdata, buffer[length + 1]);
      __nesc_hton_uint8(metadata->rssi.nxdata, buffer[length]);
      ReceiveP__PacketTimeStamp__set(ReceiveP__message, timestamp);

      ;

      ReceiveP__message = ReceiveP__Receive__receive(ReceiveP__message, ReceiveP__message->data, ReceiveP__Packet__payloadLength(ReceiveP__message));
      doAssert(ReceiveP__message != (void *)0, ASSERT_CC_RXTX_NULL_POINTER);




      memset(ReceiveP__message, 0, sizeof(message_t ));
    }
  else 
#line 73
    {
    }
}

# 263 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline void SendP__HalChipconControl__rxWaiting(uint32_t timestamp)
#line 263
{
  if (SendP__BackoffTimer__isRunning()) {
      SendP__backoff(SendP__END_OF_PACKET_BACKOFF);
    }
}

# 81 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static void HalChipconControlP__HalChipconControl__rxWaiting(uint32_t timestamp){
#line 81
  SendP__HalChipconControl__rxWaiting(timestamp);
#line 81
  ReceiveP__HalChipconControl__rxWaiting(timestamp);
#line 81
  PowerP__HalChipconControl__rxWaiting(timestamp);
#line 81
}
#line 81
# 324 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__rxDone__runTask(void )
#line 324
{
  uint8_t state = HalChipconControlP__getChipState();
  uint8_t bytesInFifo;
  bool overflow;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 329
    {
      doAssert(HalChipconControlP__pending > 0, ASSERT_CC_HAL_NO_PENDING);







      if (state == HalChipconControlP__STATE_RX) {
          HalChipconControlP__pending--;

          if (HalChipconControlP__pending > 0) {



              HalChipconControlP__rxDone__postTask();
            }
          {
            __nesc_atomic_end(__nesc_atomic); 
#line 348
            return;
          }
        }


      HalChipconControlP__pending = 0;
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }


  if (state == HalChipconControlP__STATE_OFF) {
      ;
      return;
    }



  bytesInFifo = HalChipconControlP__readRegister(RXBYTES);
  overflow = bytesInFifo & (1 << 7);

  if (overflow) {

      ;
      HalChipconControlP__strobe(SFRX);
    }



  if ((overflow || bytesInFifo == 0) || state == HalChipconControlP__STATE_RX) {


      ;
      HalChipconControlP__status.dropCount++;
      if ((state == HalChipconControlP__STATE_IDLE || state == HalChipconControlP__STATE_CALIBRATE) || state == HalChipconControlP__STATE_RXFIFO_OVERFLOW) {
#line 380
        HalChipconControlP__listen();
        }
    }
  else 
#line 381
    {

      uint32_t timestamp;

      doAssert(state == HalChipconControlP__STATE_IDLE || state == HalChipconControlP__STATE_CALIBRATE, ASSERT_CC_HAL_NOT_IDLE);
      HalChipconControlP__status.rxCount++;


      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 389
        timestamp = HalChipconControlP__rxTimeStamp;
#line 389
        __nesc_atomic_end(__nesc_atomic); }
      HalChipconControlP__HalChipconControl__rxWaiting(timestamp);
    }
}

# 63 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/PowerP.nc"
static inline void PowerP__HalChipconControl__txDone(uint32_t timestamp, error_t error)
#line 63
{
}

# 79 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/ReceiveP.nc"
static inline void ReceiveP__HalChipconControl__txDone(uint32_t timestamp, error_t error)
#line 79
{
}

# 74 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static void HalChipconControlP__HalChipconControl__txDone(uint32_t timestamp, error_t error){
#line 74
  SendP__HalChipconControl__txDone(timestamp, error);
#line 74
  ReceiveP__HalChipconControl__txDone(timestamp, error);
#line 74
  PowerP__HalChipconControl__txDone(timestamp, error);
#line 74
}
#line 74
# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void HalChipconControlP__TxTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(3U);
#line 67
}
#line 67
# 269 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__txDone__runTask(void )
#line 269
{
  uint8_t state;
  uint32_t timestamp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 273
    {
      doAssertEquals(HalChipconControlP__pending, 1, ASSERT_CC_HAL_TOO_MANY_PENDING);
      HalChipconControlP__pending = 0;
      HalChipconControlP__transmitting = FALSE;
      timestamp = HalChipconControlP__txTimeStamp;
      HalChipconControlP__TxTimer__stop();
    }
#line 279
    __nesc_atomic_end(__nesc_atomic); }

  ;








  if (HalChipconControlP__readRegister(RXBYTES) != 0) {
      ;
#line 291
      ;
      HalChipconControlP__strobe(SIDLE);
      HalChipconControlP__strobe(SFRX);
      ;
#line 294
      ;
    }

  state = HalChipconControlP__getChipState();
  if (state == HalChipconControlP__STATE_OFF) {
      HalChipconControlP__HalChipconControl__txDone(timestamp, EOFF);
    }
  else {
#line 300
    if (state == HalChipconControlP__STATE_TXFIFO_UNDERFLOW) {
        ;
#line 301
        ;


        HalChipconControlP__strobe(SFTX);


        HalChipconControlP__listen();
        HalChipconControlP__HalChipconControl__txDone(timestamp, ERETRY);
      }
    else {
#line 309
      if (state == HalChipconControlP__STATE_IDLE || state == HalChipconControlP__STATE_CALIBRATE) {

          HalChipconControlP__status.txCount++;
          HalChipconControlP__listen();
          HalChipconControlP__HalChipconControl__txDone(timestamp, SUCCESS);
        }
      else 
#line 314
        {


          doAssertEquals(state, HalChipconControlP__STATE_IDLE, ASSERT_CC_HAL_NOT_IDLE);
        }
      }
    }
}

#line 257
static inline void HalChipconControlP__taskSetAddress__runTask(void )
#line 257
{
  HalChipconControlP__setAddress();
}

# 70 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/DedicatedResourceC.nc"
static inline bool DedicatedResourceC__Resource__isOwner(void )
#line 70
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 71
    {
      unsigned char __nesc_temp = 
#line 71
      DedicatedResourceC__owned;

      {
#line 71
        __nesc_atomic_end(__nesc_atomic); 
#line 71
        return __nesc_temp;
      }
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
}

# 118 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool HalChipconControlP__SpiResource__isOwner(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = DedicatedResourceC__Resource__isOwner();
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 157 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__writeRegister(uint8_t reg, uint8_t value)
#line 157
{
  HalChipconControlP__HplChipconSpi__writeRegister(reg, value);
}

# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t HalChipconControlP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned long __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 39 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__AMControl__stopDone(error_t err)
#line 39
{
}

# 117 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void ChipconActiveMessageP__SplitControl__stopDone(error_t error){
#line 117
  BlinkToRadioC__AMControl__stopDone(error);
#line 117
}
#line 117
# 57 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static inline void ChipconActiveMessageP__stopped__runTask(void )
#line 57
{
  ChipconActiveMessageP__SplitControl__stopDone(SUCCESS);
}

# 83 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t BlinkToRadioC__AMControl__start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = ChipconActiveMessageP__SplitControl__start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void BlinkToRadioC__Timer0__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(1U, dt);
#line 53
}
#line 53
# 30 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__AMControl__startDone(error_t err)
#line 30
{
  if (err == SUCCESS) {
      BlinkToRadioC__Timer0__startPeriodic(TIMER_PERIOD_MILLI);
    }
  else {
      BlinkToRadioC__AMControl__start();
    }
}

# 92 "/home/tinyos/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void ChipconActiveMessageP__SplitControl__startDone(error_t error){
#line 92
  BlinkToRadioC__AMControl__startDone(error);
#line 92
}
#line 92
# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static inline void ChipconActiveMessageP__started__runTask(void )
#line 53
{
  ChipconActiveMessageP__SplitControl__startDone(SUCCESS);
}

# 269 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline error_t SendP__StdControl__start(void )
#line 269
{
  return SUCCESS;
}

# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControl.nc"
inline static void PowerP__HalChipconControl__on(void ){
#line 37
  HalChipconControlP__HalChipconControl__on();
#line 37
}
#line 37
# 40 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/PowerP.nc"
static inline error_t PowerP__StdControl__start(void )
#line 40
{
  if (!PowerP__on) {
      PowerP__HalChipconControl__on();
      PowerP__on = TRUE;
    }

  return SUCCESS;
}

# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t SendReceiveP__SubControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = PowerP__StdControl__start();
#line 74
  __nesc_result = ecombine(__nesc_result, SendP__StdControl__start());
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendReceiveP.nc"
static inline error_t SendReceiveP__StdControl__start(void )
#line 53
{
  if (!SendReceiveP__on) {
#line 54
    SendReceiveP__SubControl__start();
    }
#line 55
  SendReceiveP__on = TRUE;
  return SUCCESS;
}

# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t AckP__SubControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = SendReceiveP__StdControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 87 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static inline error_t AckP__StdControl__start(void )
#line 87
{
  return AckP__SubControl__start();
}

# 74 "/home/tinyos/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t ChipconActiveMessageP__SubControl__start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = AckP__StdControl__start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/DedicatedResourceC.nc"
static inline error_t DedicatedResourceC__Resource__immediateRequest(void )
#line 51
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    {
      if (DedicatedResourceC__owned) {
          unsigned char __nesc_temp = 
#line 53
          FAIL;

          {
#line 53
            __nesc_atomic_end(__nesc_atomic); 
#line 53
            return __nesc_temp;
          }
        }
#line 54
      DedicatedResourceC__owned = TRUE;
    }
#line 55
    __nesc_atomic_end(__nesc_atomic); }

  return SUCCESS;
}

# 87 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t HalChipconControlP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char __nesc_result;
#line 87

#line 87
  __nesc_result = DedicatedResourceC__Resource__immediateRequest();
#line 87

#line 87
  return __nesc_result;
#line 87
}
#line 87
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t ChipconActiveMessageP__started__postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(ChipconActiveMessageP__started);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 92 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 118 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 60 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 89 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

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
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 104 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/StackGuardWatchDogP.nc"
static inline void StackGuardWatchDogP__stackCheck(void )
#line 104
{
  if (*StackGuardWatchDogP__stackGuard != 0xcf55) {



      doAssert(FALSE, StackGuardWatchDogP__ASSERT_STACK_OVERFLOW);
    }
}

#line 130
static inline void StackGuardWatchDogP__Timer__fired(void )
#line 130
{
  StackGuardWatchDogP__stackCheck();
  StackGuardWatchDogP__resetWatchdog();
}

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

# 90 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static inline void ChipconPacketP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 90
{
  ChipconPacketP__ChipconPacket__setPayloadLength(msg, len);
}

# 83 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  ChipconPacketP__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 82 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Packet__setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current >= 1) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMPacket__destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__AMSend__send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__current = 1;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 151 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
inline static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(message_t * amsg, am_id_t t){
#line 151
  ChipconPacketP__AMPacket__setType(amsg, t);
#line 151
}
#line 151
#line 92
inline static void /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(message_t * amsg, am_addr_t addr){
#line 92
  ChipconPacketP__AMPacket__setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/home/tinyos/tinyos-2.x/tos/system/AMQueueEntryP.nc"
static inline error_t /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setDestination(msg, dest);
  /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMPacket__setType(msg, 6);
  return /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__Send__send(msg, len);
}

# 69 "/home/tinyos/tinyos-2.x/tos/interfaces/AMSend.nc"
inline static error_t BlinkToRadioC__AMSend__send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*BlinkToRadioAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP__0__AMSend__send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 115 "/home/tinyos/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void * BlinkToRadioC__Packet__getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = ChipconPacketP__Packet__getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 50 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__Timer0__fired(void )
#line 50
{
  BlinkToRadioC__counter++;

  if (!BlinkToRadioC__busy) {
      BlinkToRadioMsg *btrpkt = (BlinkToRadioMsg *)
      BlinkToRadioC__Packet__getPayload(&BlinkToRadioC__pkt, sizeof(BlinkToRadioMsg ));

#line 56
      __nesc_hton_uint16(btrpkt->nodeid.nxdata, TOS_NODE_ID);
      __nesc_hton_uint16(btrpkt->counter.nxdata, BlinkToRadioC__counter);
      if (BlinkToRadioC__AMSend__send(2, &BlinkToRadioC__pkt, sizeof(BlinkToRadioMsg )) == SUCCESS) {
          BlinkToRadioC__busy = TRUE;
        }
    }
}

# 327 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static inline void AckP__AckTimer__fired(void )
#line 327
{


  __nesc_hton_uint8(AckP__ChipconPacket__getMetadata(AckP__txMessage)->ack.nxdata, FALSE);
  AckP__sendDone(SUCCESS);
}

# 791 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__TxTimer__fired(void )
#line 791
{
  ;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 794
    {
      if (HalChipconControlP__pending > 0) {
          {
            __nesc_atomic_end(__nesc_atomic); 
#line 797
            return;
          }
        }
      ;

      ;
#line 802
      ;

      doAssert(HalChipconControlP__transmitting, ASSERT_CC_HAL_NO_TX);
      HalChipconControlP__status.errorCount++;
      HalChipconControlP__transmitting = FALSE;
      HalChipconControlP__pending = 0;
      if (HalChipconControlP__getChipState() == HalChipconControlP__STATE_TX) {
#line 808
        HalChipconControlP__listen();
        }
#line 809
      HalChipconControlP__HalChipconControl__txDone(HalChipconControlP__LocalTime__get(), FAIL);
    }
#line 810
    __nesc_atomic_end(__nesc_atomic); }
}

# 154 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static inline void SendP__BackoffTimer__fired(void )
#line 154
{

  SendP__attemptTx();
}

static inline void SendP__PreambleTimer__fired(void )
#line 159
{

  SendP__writeData();
}

# 193 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x408d49f0){
#line 72
  switch (arg_0x408d49f0) {
#line 72
    case 0U:
#line 72
      StackGuardWatchDogP__Timer__fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      BlinkToRadioC__Timer0__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      AckP__AckTimer__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      HalChipconControlP__TxTimer__fired();
#line 72
      break;
#line 72
    case 4U:
#line 72
      SendP__BackoffTimer__fired();
#line 72
      break;
#line 72
    case 5U:
#line 72
      SendP__PreambleTimer__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x408d49f0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 128 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/home/tinyos/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 63 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 46 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4302 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
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

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 36 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 185 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__enable(void )
#line 185
{
#line 185
  P2IE |= 1 << 4;
}

# 31 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port24__enable();
#line 31
}
#line 31
# 237 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__edge(bool l2h)
#line 237
{
  /* atomic removed: atomic calls only */
#line 238
  {
    if (l2h) {
#line 239
      P2IES &= ~(1 << 4);
      }
    else {
#line 240
      P2IES |= 1 << 4;
      }
  }
}

# 56 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port24__edge(low_to_high);
#line 56
}
#line 56
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__disable();
    /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__edge(rising);
    /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__enable(FALSE);
}

# 43 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t HalChipconControlP__G0Interrupt__enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__enableFallingEdge();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 60 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/DedicatedResourceC.nc"
static inline error_t DedicatedResourceC__Resource__release(void )
#line 60
{
  error_t error;

  /* atomic removed: atomic calls only */
#line 62
  {
    error = DedicatedResourceC__owned ? SUCCESS : FAIL;
    DedicatedResourceC__owned = FALSE;
  }

  return error;
}

# 110 "/home/tinyos/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t HalChipconControlP__SpiResource__release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = DedicatedResourceC__Resource__release();
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 493 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__HalChipconControl__off(void )
#line 493
{

  HalChipconControlP__strobe(SIDLE);
  HalChipconControlP__strobe(SPWD);
  HalChipconControlP__CSn__set();


  HalChipconControlP__SpiResource__release();
}

# 55 "/home/tinyos/tinyos-2.x/tos/lib/timer/BusyWait.nc"
inline static void HalChipconControlP__BusyWait__wait(HalChipconControlP__BusyWait__size_type dt){
#line 55
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(dt);
#line 55
}
#line 55
# 207 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__calibrate(void )
#line 207
{
  HalChipconControlP__strobe(SCAL);
  HalChipconControlP__BusyWait__wait(HalChipconControlP__CALIBRATION_TIME);
}




static inline void HalChipconControlP__configure(void )
#line 215
{
  uint8_t i;

#line 217
  for (i = 0; i < NUM_REGISTERS * 2; i += 2) {
      uint8_t reg = chipconRegisterValues[i];
      uint8_t value = chipconRegisterValues[i + 1];

#line 220
      HalChipconControlP__HplChipconSpi__writeRegister(reg, value);
    }
}

#line 192
static inline void HalChipconControlP__checkChipVersion(void )
#line 192
{
  uint8_t partnum = HalChipconControlP__readRegister(PARTNUM);
  uint8_t version = HalChipconControlP__readRegister(VERSION);

#line 195
  doAssert(partnum == EXPECTED_PARTNUM, ASSERT_CC_HAL_CHIP_VERSION);
}

# 54 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__getRaw(void )
#line 54
{
#line 54
  return * (volatile uint8_t * )24U & (0x01 << 2);
}

#line 55
static inline bool /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__get(void )
#line 55
{
#line 55
  return /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__getRaw() != 0;
}

# 63 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__HplGeneralIO__get(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__get();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 43 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__get(void )
#line 43
{
#line 43
  return /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__HplGeneralIO__get();
}

# 36 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static bool HalChipconControlP__SO__get(void ){
#line 36
  unsigned char __nesc_result;
#line 36

#line 36
  __nesc_result = /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__get();
#line 36

#line 36
  return __nesc_result;
#line 36
}
#line 36
# 136 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__waitForChipReady(void )
#line 136
{

  while (HalChipconControlP__SO__get()) ;
}

#line 165
static inline void HalChipconControlP__reset(void )
#line 165
{
  /* atomic removed: atomic calls only */
#line 166
  doAssertNot(HalChipconControlP__transmitting, ASSERT_CC_HAL_NOT_IDLE);
#line 178
  HalChipconControlP__CSn__set();
  HalChipconControlP__CSn__clr();
  HalChipconControlP__CSn__set();
  HalChipconControlP__BusyWait__wait(40);
  HalChipconControlP__CSn__clr();
  HalChipconControlP__waitForChipReady();

  HalChipconControlP__strobe(SRES);
  HalChipconControlP__waitForChipReady();
}

# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeOutput(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 58
  * (volatile uint8_t * )26U |= 0x01 << 3;
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__HplGeneralIO__makeOutput(void ){
#line 75
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__makeOutput();
#line 75
}
#line 75
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__GeneralIO__makeOutput(void )
#line 46
{
#line 46
  /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__HplGeneralIO__makeOutput();
}

# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__Clock__makeOutput(void ){
#line 39
  /*Msp430GeneralIOC.GeneralIO33*/Msp430GpioC__19__GeneralIO__makeOutput();
#line 39
}
#line 39
# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeOutput(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 58
  * (volatile uint8_t * )26U |= 0x01 << 1;
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__HplGeneralIO__makeOutput(void ){
#line 75
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__makeOutput();
#line 75
}
#line 75
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__GeneralIO__makeOutput(void )
#line 46
{
#line 46
  /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__HplGeneralIO__makeOutput();
}

# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__SI__makeOutput(void ){
#line 39
  /*Msp430GeneralIOC.GeneralIO31*/Msp430GpioC__17__GeneralIO__makeOutput();
#line 39
}
#line 39
# 56 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__makeInput(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )26U &= ~(0x01 << 2);
}

# 68 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__HplGeneralIO__makeInput(void ){
#line 68
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__makeInput();
#line 68
}
#line 68
# 44 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__makeInput(void )
#line 44
{
#line 44
  /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__HplGeneralIO__makeInput();
}

# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__SO__makeInput(void ){
#line 37
  /*Msp430GeneralIOC.GeneralIO32*/Msp430GpioC__18__GeneralIO__makeInput();
#line 37
}
#line 37
# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 58
  * (volatile uint8_t * )42U |= 0x01 << 3;
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__makeOutput(void ){
#line 75
  /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__makeOutput();
#line 75
}
#line 75
# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__makeOutput(void )
#line 46
{
#line 46
  /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__HplGeneralIO__makeOutput();
}

# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__CSn__makeOutput(void ){
#line 39
  /*Msp430GeneralIOC.GeneralIO23*/Msp430GpioC__11__GeneralIO__makeOutput();
#line 39
}
#line 39
# 56 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P25*/HplMsp430GeneralIOP__13__IO__makeInput(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )42U &= ~(0x01 << 5);
}

# 68 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__HplGeneralIO__makeInput(void ){
#line 68
  /*HplMsp430GeneralIOC.P25*/HplMsp430GeneralIOP__13__IO__makeInput();
#line 68
}
#line 68
# 44 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__GeneralIO__makeInput(void )
#line 44
{
#line 44
  /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__HplGeneralIO__makeInput();
}

# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__G2__makeInput(void ){
#line 37
  /*Msp430GeneralIOC.GeneralIO25*/Msp430GpioC__13__GeneralIO__makeInput();
#line 37
}
#line 37
# 56 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )42U &= ~(0x01 << 4);
}

# 68 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__HplGeneralIO__makeInput(void ){
#line 68
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput();
#line 68
}
#line 68
# 44 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__makeInput(void )
#line 44
{
#line 44
  /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__HplGeneralIO__makeInput();
}

# 37 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void HalChipconControlP__G0__makeInput(void ){
#line 37
  /*Msp430GeneralIOC.GeneralIO24*/Msp430GpioC__12__GeneralIO__makeInput();
#line 37
}
#line 37
# 50 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t HalChipconControlP__G0Interrupt__disable(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__disable();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 435 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static inline void HalChipconControlP__HalChipconControl__init(void )
#line 435
{
  /* atomic removed: atomic calls only */
#line 436
  HalChipconControlP__transmitting = FALSE;



  HalChipconControlP__G0Interrupt__disable();
  HalChipconControlP__G0__makeInput();
  HalChipconControlP__G2__makeInput();
  HalChipconControlP__CSn__makeOutput();
  HalChipconControlP__CSn__set();

  HalChipconControlP__SO__makeInput();
  HalChipconControlP__SI__makeOutput();
  HalChipconControlP__Clock__makeOutput();
  ;


  HalChipconControlP__HalChipconControl__on();
  ;
  HalChipconControlP__reset();
  ;
  HalChipconControlP__checkChipVersion();

  HalChipconControlP__configure();

  HalChipconControlP__setAddress();

  HalChipconControlP__calibrate();



  HalChipconControlP__HalChipconControl__off();



  HalChipconControlP__G0Interrupt__enableFallingEdge();
}

#line 426
static inline error_t HalChipconControlP__Init__init(void )
#line 426
{
  HalChipconControlP__HalChipconControl__init();
  ;
  return SUCCESS;
}

# 44 "/home/tinyos/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = RandomMlcgC__Init__init();
#line 51
  __nesc_result = ecombine(__nesc_result, HalChipconControlP__Init__init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 193 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__disable(void )
#line 193
{
#line 193
  P2IE &= ~(1 << 4);
}

# 36 "/home/tinyos/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port24__disable();
#line 36
}
#line 36
# 20 "BlinkToRadioC.nc"
static inline void BlinkToRadioC__Boot__booted(void )
#line 20
{
  BlinkToRadioC__AMControl__start();
}

# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void StackGuardWatchDogP__Timer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 53
}
#line 53
# 135 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/StackGuardWatchDogP.nc"
static inline void StackGuardWatchDogP__Boot__booted(void )
#line 135
{
  StackGuardWatchDogP__resetWatchdog();
  StackGuardWatchDogP__Timer__startPeriodic(500);
}

# 49 "/home/tinyos/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  StackGuardWatchDogP__Boot__booted();
#line 49
  BlinkToRadioC__Boot__booted();
#line 49
}
#line 49
# 384 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 61 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 61
{
  return MSP430_POWER_LPM3;
}

# 54 "/home/tinyos/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM4;



  if (((
#line 69
  TACCTL0 & 0x0010 || 
  TACCTL1 & 0x0010) || 
  TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) {
#line 99
    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 109
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 110
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 372 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 372
{
  return m1 < m2 ? m1 : m2;
}

# 122 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 122
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 127
{
  uint16_t temp;

#line 129
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/home/tinyos/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 72
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

#line 138
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
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 61 "/home/tinyos/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 404 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 407
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

# 11 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0032)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0030)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x003A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x40444448){
#line 28
  switch (arg_0x40444448) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x40444448);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
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
#line 121
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

# 69 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 14 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0038)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 55 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0024)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 59
      HplMsp430InterruptP__Port10__fired();
#line 59
      return;
    }
#line 60
  if (n & (1 << 1)) {
#line 60
      HplMsp430InterruptP__Port11__fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 2)) {
#line 61
      HplMsp430InterruptP__Port12__fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 3)) {
#line 62
      HplMsp430InterruptP__Port13__fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 4)) {
#line 63
      HplMsp430InterruptP__Port14__fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 5)) {
#line 64
      HplMsp430InterruptP__Port15__fired();
#line 64
      return;
    }
#line 65
  if (n & (1 << 6)) {
#line 65
      HplMsp430InterruptP__Port16__fired();
#line 65
      return;
    }
#line 66
  if (n & (1 << 7)) {
#line 66
      HplMsp430InterruptP__Port17__fired();
#line 66
      return;
    }
}

#line 160
__attribute((wakeup)) __attribute((interrupt(0x0026)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 164
      HplMsp430InterruptP__Port20__fired();
#line 164
      return;
    }
#line 165
  if (n & (1 << 1)) {
#line 165
      HplMsp430InterruptP__Port21__fired();
#line 165
      return;
    }
#line 166
  if (n & (1 << 2)) {
#line 166
      HplMsp430InterruptP__Port22__fired();
#line 166
      return;
    }
#line 167
  if (n & (1 << 3)) {
#line 167
      HplMsp430InterruptP__Port23__fired();
#line 167
      return;
    }
#line 168
  if (n & (1 << 4)) {
#line 168
      HplMsp430InterruptP__Port24__fired();
#line 168
      return;
    }
#line 169
  if (n & (1 << 5)) {
#line 169
      HplMsp430InterruptP__Port25__fired();
#line 169
      return;
    }
#line 170
  if (n & (1 << 6)) {
#line 170
      HplMsp430InterruptP__Port26__fired();
#line 170
      return;
    }
#line 171
  if (n & (1 << 7)) {
#line 171
      HplMsp430InterruptP__Port27__fired();
#line 171
      return;
    }
}

# 69 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 52 "/home/tinyos/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 177 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 123 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x403b2580){
#line 64
  switch (arg_0x403b2580) {
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 64
      break;
#line 64
    case ChipconActiveMessageP__started:
#line 64
      ChipconActiveMessageP__started__runTask();
#line 64
      break;
#line 64
    case ChipconActiveMessageP__stopped:
#line 64
      ChipconActiveMessageP__stopped__runTask();
#line 64
      break;
#line 64
    case HalChipconControlP__taskSetAddress:
#line 64
      HalChipconControlP__taskSetAddress__runTask();
#line 64
      break;
#line 64
    case HalChipconControlP__txDone:
#line 64
      HalChipconControlP__txDone__runTask();
#line 64
      break;
#line 64
    case HalChipconControlP__rxDone:
#line 64
      HalChipconControlP__rxDone__runTask();
#line 64
      break;
#line 64
    case DedicatedResourceC__grant:
#line 64
      DedicatedResourceC__grant__runTask();
#line 64
      break;
#line 64
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask:
#line 64
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__CancelTask__runTask();
#line 64
      break;
#line 64
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask:
#line 64
      /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__errorTask__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x403b2580);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 155 "/home/tinyos/tinyos-2.x/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP__0__Send__sendDone(last, msg, err);
}

# 114 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static am_addr_t ChipconPacketP__AMPacket__destination(message_t *amsg)
#line 114
{
  chipcon_header_t *header = ChipconPacketP__ChipconPacket__getHeader(amsg);

  if (__nesc_ntoh_leuint32(header->dest.nxdata) == ChipconPacketP__getLocalBroadcastAddress()) {
      return 0xFFFFFFFFUL;
    }
  else 
#line 119
    {
      return __nesc_ntoh_leuint32(header->dest.nxdata);
    }
}

# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ActiveMessageAddressC.nc"
static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      unsigned long __nesc_temp = 
#line 59
      ActiveMessageAddressC__group;

      {
#line 59
        __nesc_atomic_end(__nesc_atomic); 
#line 59
        return __nesc_temp;
      }
    }
#line 61
    __nesc_atomic_end(__nesc_atomic); }
}




static am_netmask_t ActiveMessageAddressC__NetMask__netMask(void )
#line 67
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 68
    {
      unsigned long __nesc_temp = 
#line 68
      ActiveMessageAddressC__netmask;

      {
#line 68
        __nesc_atomic_end(__nesc_atomic); 
#line 68
        return __nesc_temp;
      }
    }
#line 70
    __nesc_atomic_end(__nesc_atomic); }
}

# 65 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static uint8_t ChipconPacketP__ChipconPacket__getPacketLength(message_t *msg)
#line 65
{

  return __nesc_ntoh_uint8(ChipconPacketP__ChipconPacket__getHeader(msg)->length.nxdata) + 1;
}

# 125 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static error_t ChipconActiveMessageP__AMSend__send(am_id_t id, am_addr_t addr, message_t *msg, uint8_t len)
#line 125
{
  error_t error;

#line 127
  if (ChipconActiveMessageP__txBusy) {
#line 127
    return EBUSY;
    }
  ChipconActiveMessageP__AMPacket__setType(msg, id);
  ChipconActiveMessageP__AMPacket__setDestination(msg, addr);
  ChipconActiveMessageP__AMPacket__setSource(msg, ChipconActiveMessageP__AMPacket__address());



  ;

  error = ChipconActiveMessageP__SubSend__send(msg, len);
  if (error == SUCCESS) {
      ChipconActiveMessageP__txBusy = TRUE;
      ChipconActiveMessageP__crc = ChipconActiveMessageP__crcMessage(msg);
    }

  ;

  return error;
}

# 160 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static void ChipconPacketP__AMPacket__setType(message_t *amsg, am_id_t type)
#line 160
{
  chipcon_header_t *header = ChipconPacketP__ChipconPacket__getHeader(amsg);

#line 162
  __nesc_hton_uint8(header->type.nxdata, type);
}

#line 124
static void ChipconPacketP__AMPacket__setDestination(message_t *amsg, am_addr_t addr)
#line 124
{
  chipcon_header_t *header = ChipconPacketP__ChipconPacket__getHeader(amsg);

  if (addr == 0xFFFFFFFFUL) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 128
        __nesc_hton_leuint32(header->dest.nxdata, ChipconPacketP__getLocalBroadcastAddress());
#line 128
        __nesc_atomic_end(__nesc_atomic); }
    }
  else 
#line 129
    {
      __nesc_hton_leuint32(header->dest.nxdata, addr);
    }
}

# 45 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 45
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    {
      unsigned long __nesc_temp = 
#line 46
      ActiveMessageAddressC__group | ActiveMessageAddressC__addr;

      {
#line 46
        __nesc_atomic_end(__nesc_atomic); 
#line 46
        return __nesc_temp;
      }
    }
#line 48
    __nesc_atomic_end(__nesc_atomic); }
}

# 75 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static void ChipconPacketP__ChipconPacket__setPayloadLength(message_t *msg, uint8_t len)
#line 75
{
  __nesc_hton_uint8(ChipconPacketP__ChipconPacket__getHeader(msg)->length.nxdata, sizeof(chipcon_header_t ) + len + sizeof(chipcon_footer_t ) - 1);
}

# 90 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendReceiveP.nc"
static error_t SendReceiveP__Send__send(message_t *msg, uint8_t len)
#line 90
{






  doAssert(msg != SendReceiveP__rxBuffer, ASSERT_CC_RXTX_BAD_TX_POINTER);

  if (!SendReceiveP__on) {
#line 99
    return EOFF;
    }
#line 100
  return SendReceiveP__SubSend__send(msg, len);
}

# 525 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static error_t HalChipconControlP__HalChipconControl__tx(void )
#line 525
{
  uint8_t state = HalChipconControlP__getChipState();

#line 527
  if (state == HalChipconControlP__STATE_OFF) {
#line 527
    return EOFF;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 529
    doAssertNot(HalChipconControlP__transmitting, ASSERT_CC_HAL_TX_ALREADY);
#line 529
    __nesc_atomic_end(__nesc_atomic); }




  if (((state == HalChipconControlP__STATE_IDLE || state == HalChipconControlP__STATE_CALIBRATE) || state == HalChipconControlP__STATE_SETTLING) || state == HalChipconControlP__STATE_RXFIFO_OVERFLOW) {

      return ERETRY;
    }
  else 
#line 537
    {
      doAssertEquals(state, HalChipconControlP__STATE_RX, ASSERT_CC_HAL_CCA_NO_RX);
    }
#line 553
  if (HalChipconControlP__G0__get()) {
      return EBUSY;
    }





  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 561
    {

      if (HalChipconControlP__pending > 0) {
          {
            unsigned char __nesc_temp = 
#line 564
            EBUSY;

            {
#line 564
              __nesc_atomic_end(__nesc_atomic); 
#line 564
              return __nesc_temp;
            }
          }
        }

      HalChipconControlP__waitForRssiValid();



      HalChipconControlP__strobe(STX);


      HalChipconControlP__BusyWait__wait(HalChipconControlP__RX_TO_TX_TIME);









      switch (HalChipconControlP__getChipState()) {
          case HalChipconControlP__STATE_RX: 

            {
              unsigned char __nesc_temp = 
#line 589
              FAIL;

              {
#line 589
                __nesc_atomic_end(__nesc_atomic); 
#line 589
                return __nesc_temp;
              }
            }
#line 591
          case HalChipconControlP__STATE_IDLE: 
            case HalChipconControlP__STATE_CALIBRATE: 
              case HalChipconControlP__STATE_RXFIFO_OVERFLOW: 

                {
                  unsigned char __nesc_temp = 
#line 595
                  EBUSY;

                  {
#line 595
                    __nesc_atomic_end(__nesc_atomic); 
#line 595
                    return __nesc_temp;
                  }
                }
#line 597
          case HalChipconControlP__STATE_TX: 
            case HalChipconControlP__STATE_SETTLING: 
              HalChipconControlP__transmitting = TRUE;
          break;

          default: 


            doAssertEquals(state, HalChipconControlP__STATE_IDLE, ASSERT_CC_HAL_NO_TX);
          {
            unsigned char __nesc_temp = 
#line 606
            FAIL;

            {
#line 606
              __nesc_atomic_end(__nesc_atomic); 
#line 606
              return __nesc_temp;
            }
          }
        }
      if (HalChipconControlP__TxTimer__isRunning()) {
        }
      else {

          HalChipconControlP__TxTimer__startOneShot(60 * 1024UL);
        }
    }
#line 616
    __nesc_atomic_end(__nesc_atomic); }

  return SUCCESS;
}

#line 119
static uint8_t HalChipconControlP__getChipState(void )
#line 119
{
  uint8_t statusByte;

  if (HalChipconControlP__CSn__get()) {

      return HalChipconControlP__STATE_OFF;
    }


  statusByte = HalChipconControlP__strobe(SNOP);
  return (statusByte >> 4) & 0x07;
}

# 80 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static uint8_t HplChipconSpiB0P__HplChipconSpi__strobe(uint8_t strobe)
#line 80
{
  uint8_t status;

#line 82
  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = strobe;
  while (UCB0STAT & 0x01) ;
  status = UCB0RXBUF;
  return status;
}

# 46 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/AssertRebootP.nc"
static  void doAssertNot(bool condition, uint16_t errorCode)
#line 46
{
  doAssert(!condition, errorCode);
}











static  void doAssertEquals(uint32_t a, uint32_t b, uint16_t errorCode)
#line 60
{
  doAssert(a == b, errorCode);
}

# 144 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static uint8_t HalChipconControlP__readRegister(uint8_t reg)
#line 144
{

  if (reg >= 0x30 && reg <= 0x3E) {
#line 146
    reg |= 0x40;
    }

  reg |= 0x80;

  return HalChipconControlP__HplChipconSpi__readRegister(reg);
}

# 89 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static uint8_t HplChipconSpiB0P__HplChipconSpi__readRegister(uint8_t reg)
#line 89
{
  uint8_t value;

#line 91
  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = reg;
  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = 0;
  while (UCB0STAT & 0x01) ;
  value = UCB0RXBUF;
  return value;
}

# 52 "/home/tinyos/tinyos-2.x/tos/lib/timer/BusyWaitCounterC.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt)
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

# 133 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 89 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static void SendP__writeData(void )
#line 89
{
  uint8_t *data = (uint8_t *)SendP__ChipconPacket__getHeader(SendP__message);
  uint8_t length = SendP__ChipconPacket__getPacketLength(SendP__message);

#line 92
  SendP__HalChipconControl__write(data, length);
}

# 100 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/spi/HplChipconSpiB0P.nc"
static void HplChipconSpiB0P__HplChipconSpi__writeRegister(uint8_t reg, uint8_t value)
#line 100
{
  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = reg;
  while (!(IFG2 & 0x08)) ;
  UCB0TXBUF = value;
  while (UCB0STAT & 0x01) ;
}

# 224 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static void ChipconPacketP__PacketTimeStamp__set(message_t *msg, uint32_t value)
#line 224
{
  chipcon_metadata_t *metadata = ChipconPacketP__ChipconPacket__getMetadata(msg);

#line 226
  __nesc_hton_uint32(metadata->timestamp.nxdata, value);
}

# 51 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__set(void )
#line 51
{
#line 51
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 51
    * (volatile uint8_t * )41U |= 0x01 << 3;
#line 51
    __nesc_atomic_end(__nesc_atomic); }
}

#line 52
static void /*HplMsp430GeneralIOC.P23*/HplMsp430GeneralIOP__11__IO__clr(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )41U &= ~(0x01 << 3);
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 95 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static void SendP__attemptTx(void )
#line 95
{
  error_t error;




  doAssert(__nesc_ntoh_leuint32(SendP__ChipconPacket__getHeader(SendP__message)->dest.nxdata) != 0xFFFFFFFFUL, ASSERT_CC_RXTX_BROADCAST);


  error = SendP__HalChipconControl__tx();
  if (error == SUCCESS) {
      uint16_t interval = __nesc_ntoh_uint16(SendP__ChipconPacket__getMetadata(SendP__message)->rxInterval.nxdata);

#line 107
      if (interval == 0) {

          SendP__writeData();
        }
      else 
#line 110
        {

          SendP__PreambleTimer__startOneShot(interval + LPL_PREAMBLE_OVERLAP);


          __nesc_hton_uint16(SendP__ChipconPacket__getMetadata(SendP__message)->rxInterval.nxdata, 0);
        }
    }
  else {
#line 117
    if ((error == ERETRY || error == FAIL) || error == EBUSY) {
        SendP__backoff(SendP__CCA_BACKOFF);
      }
    else {
#line 119
      if (error == EOFF) {
          ;
          SendP__HalChipconControl__txDone(0, EOFF);
        }
      else 
#line 122
        {
          doAssertSuccess(error, ASSERT_CANT_HAPPEN);
        }
      }
    }
}

#line 77
static void SendP__backoff(uint16_t fixed)
#line 77
{
  uint16_t delay = fixed + SendP__Random__rand16() % SendP__backoffLimit;

#line 79
  SendP__BackoffTimer__startOneShot(delay);


  SendP__backoffLimit *= 2;
  if (SendP__backoffLimit > SendP__MAX_RANDOM_BACKOFF) {
#line 83
    SendP__backoffLimit = SendP__MAX_RANDOM_BACKOFF;
    }
}

# 58 "/home/tinyos/tinyos-2.x/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 58
{
  uint32_t mlcg;
#line 59
  uint32_t p;
#line 59
  uint32_t q;
  uint64_t tmpseed;

#line 61
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMlcgC__seed = mlcg;
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

# 214 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/rxtx/SendP.nc"
static void SendP__HalChipconControl__txDone(uint32_t timestamp, error_t error)
#line 214
{
  if (error == ERETRY) {

      SendP__attemptTx();
    }
  else 
#line 218
    {
      if (error == SUCCESS) {

          uint32_t estimate = SendP__PacketTimeStamp__timestamp(SendP__message) - SendP__packetDurationAdjustment;

#line 222
          SendP__packetDurationAdjustment = timestamp - estimate;
        }



      SendP__busy = FALSE;
      SendP__Send__sendDone(SendP__message, error);
    }
}

# 217 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static void AckP__SubSend__sendDone(message_t *msg, error_t error)
#line 217
{
  if (error == ECANCEL) {


      return;
    }

  if (msg == AckP__ackMessage) {

      uint8_t payloadLength;

#line 227
      doAssert(AckP__acking, ASSERT_CC_ACK_STATE);
      AckP__acking = FALSE;


      payloadLength = AckP__Packet__payloadLength(AckP__rxMessage);
      AckP__buffer = AckP__Receive__receive(AckP__rxMessage, AckP__Packet__getPayload(AckP__rxMessage, payloadLength), payloadLength);



      if (AckP__sending && !AckP__subSending) {
          error = AckP__SubSend__send(AckP__txMessage, AckP__ChipconPacket__getPayloadLength(AckP__txMessage));
          doAssertSuccess(error, ASSERT_CC_ACK_SEND);
          AckP__subSending = TRUE;
        }
    }
  else 
#line 241
    {

      doAssert(msg == AckP__txMessage, ASSERT_CC_ACK_STATE);
      if (error == SUCCESS) {


          if (AckP__needsAck(msg)) {
              AckP__AckTimer__startOneShot((PREAMBLE_BYTES + SYNC_BYTES + sizeof(chipcon_header_t )) * 8 * 1024UL / (38400UL / 2) + 20);
            }
          else 
#line 249
            {

              __nesc_hton_uint8(AckP__ChipconPacket__getMetadata(AckP__txMessage)->ack.nxdata, FALSE);
              AckP__sendDone(SUCCESS);
            }
        }
      else 
#line 254
        {

          ;
#line 256
          ;
          __nesc_hton_uint8(AckP__ChipconPacket__getMetadata(AckP__txMessage)->ack.nxdata, FALSE);
          AckP__sendDone(error);
        }
    }
}

# 176 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static message_t *ChipconActiveMessageP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 176
{



  ;
  if (ChipconActiveMessageP__AMPacket__isForMe(msg)) {
      return ChipconActiveMessageP__Receive__receive(ChipconActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
  else 
#line 183
    {
      return ChipconActiveMessageP__Snoop__receive(ChipconActiveMessageP__AMPacket__type(msg), msg, payload, len);
    }
}

# 136 "/home/tinyos/tinyos-2.x/tos/interfaces/AMPacket.nc"
static am_id_t ChipconActiveMessageP__AMPacket__type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = ChipconPacketP__AMPacket__type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 98 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/packet/ChipconPacketP.nc"
static void *ChipconPacketP__Packet__getPayload(message_t *msg, uint8_t len)
#line 98
{
  if (len > ChipconPacketP__Packet__maxPayloadLength()) {
      return (void *)0;
    }
  else 
#line 101
    {
      return (void * )msg->data;
    }
}

# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/AssertRebootP.nc"
static  void doAssertSuccess(error_t error, uint16_t errorCode)
#line 53
{
  doAssert(error == SUCCESS, errorCode);
}

# 123 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/ack/AckP.nc"
static bool AckP__needsAck(message_t *msg)
#line 123
{
  chipcon_header_t *header = AckP__ChipconPacket__getHeader(msg);

#line 125
  return AckP__AMPacket__destination(msg) != 0xFFFFFFFFUL && __nesc_ntoh_uint8(header->flags.nxdata) & FLAG_ACK_REQUEST;
}

#line 207
static void AckP__sendDone(error_t error)
#line 207
{
  message_t *tmp;

#line 209
  doAssert(AckP__txMessage != (void *)0, ASSERT_CC_ACK_STATE);
  tmp = AckP__txMessage;
  AckP__txMessage = (void *)0;
  AckP__sending = FALSE;
  AckP__Send__sendDone(tmp, error);
  __nesc_hton_uint8(AckP__ChipconPacket__getMetadata(tmp)->ack.nxdata, FALSE);
}

# 63 "/home/tinyos/tinyos-2.x/tos/system/CrcC.nc"
static uint16_t CrcC__Crc__seededCrc16(uint16_t startCrc, void *buf, uint8_t len)
#line 63
{
  uint8_t *tmp = (uint8_t *)buf;
  uint16_t crc;

#line 66
  for (crc = startCrc; len > 0; len--) {
      crc = crcByte(crc, * tmp++);
    }
  return crc;
}

# 248 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static void HalChipconControlP__setAddress(void )
#line 248
{



  if (HalChipconControlP__SpiResource__isOwner()) {
      HalChipconControlP__writeRegister(ADDR, HalChipconControlP__ActiveMessageAddress__amAddress() & 0xFF);
    }
}

# 143 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 61 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/am/ChipconActiveMessageP.nc"
static error_t ChipconActiveMessageP__SplitControl__start(void )
#line 61
{
  error_t error = ChipconActiveMessageP__SubControl__start();

#line 63
  ;
  if (error == SUCCESS) {
#line 64
    ChipconActiveMessageP__started__postTask();
    }
#line 65
  return error;
}

# 477 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/ccpacket/hal/HalChipconControlP.nc"
static void HalChipconControlP__HalChipconControl__on(void )
#line 477
{

  error_t error = HalChipconControlP__SpiResource__immediateRequest();

#line 480
  doAssertSuccess(error, ASSERT_CC_HAL_SPI_REQUEST);


  HalChipconControlP__CSn__clr();
  HalChipconControlP__waitForChipReady();


  HalChipconControlP__listen();
}

# 62 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

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
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 136 "/home/tinyos/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static error_t /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__disable();
    /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__clear();
  }
  return SUCCESS;
}

