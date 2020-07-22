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
# 431 "/usr/lib/ncc/nesc_nx.h"
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
# 192 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/msp430f2418.h" 3
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
enum __nesc_unnamed4251 {
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
typedef struct __nesc_unnamed4252 {

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
typedef struct __nesc_unnamed4253 {

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
typedef struct __nesc_unnamed4254 {

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
# 29 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4255 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4256 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4257 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 32 "/home/tinyos/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4258 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
typedef TMilli StackGuardWatchDogP__Timer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4259 {
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
typedef TMilli BlinkC__Timer0__precision_tag;
typedef TMilli BlinkC__Timer1__precision_tag;
typedef TMilli BlinkC__Timer2__precision_tag;
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/Reboot.nc"
static void RebootC__Reboot__reboot(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t PlatformP__InitLevel__default__init(
# 32 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
uint8_t arg_0x403e3788);
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
uint8_t arg_0x4043db30);
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4043db30);
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
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__toggle(void );
#line 75
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );
#line 43
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );




static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle(void );
#line 75
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void );
#line 43
static void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void );




static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle(void );
#line 75
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void );
#line 43
static void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void );
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
#line 41
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
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__toggle(void );



static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 34
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__clr(void );
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__toggle(void );



static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__makeOutput(void );
#line 34
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__clr(void );
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__toggle(void );



static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 34
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__clr(void );
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
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__default__fired(void );
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
uint8_t arg_0x408b79f0);
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 37 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408b79f0, 
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);
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
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC__Timer0__fired(void );
# 49 "/home/tinyos/tinyos-2.x/tos/interfaces/Boot.nc"
static void BlinkC__Boot__booted(void );
# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC__Timer1__fired(void );
#line 72
static void BlinkC__Timer2__fired(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
#line 72
static void LedsP__Leds__led1Toggle(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
# 34 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/Reboot.nc"
static void AssertRebootP__Reboot__reboot(void );
# 39 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/AssertRebootP.nc"
static inline void doAssert(bool condition, uint16_t errorCode)  ;
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/lib/assert/RebootC.nc"
static inline void RebootC__Reboot__reboot(void );
# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__InitLevel__init(
# 32 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
uint8_t arg_0x403e3788);



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

enum Msp430ClockP____nesc_unnamed4260 {

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
uint8_t arg_0x4043db30);
# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 115 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4043db30);
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
# 52 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );
#line 52
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__makeOutput(void );
#line 52
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle(void );




static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__makeOutput(void );
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
#line 197
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__toggle(void );
#line 75
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 43
static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__clr(void );
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__toggle(void );



static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__toggle(void );
#line 75
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__makeOutput(void );
#line 43
static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__clr(void );
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__clr(void );
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__toggle(void );



static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__makeOutput(void );
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__toggle(void );
#line 75
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 43
static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 41 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__clr(void );
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__toggle(void );



static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__makeOutput(void );
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
# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__fired(void );
# 66 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__HplInterrupt__fired(void );




static inline void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__default__fired(void );
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
enum StackGuardWatchDogP____nesc_unnamed4261 {
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

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4262 {

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

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4263 {

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
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4264 {
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


static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






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
uint8_t arg_0x408b79f0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4265 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4266 {

  VirtualizeTimerC__0__NUM_TIMERS = 4U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4267 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);
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
enum SchedulerBasicP____nesc_unnamed4268 {

  SchedulerBasicP__NUM_TASKS = 2U, 
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




static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC__Timer0__startPeriodic(uint32_t dt);
#line 53
static void BlinkC__Timer1__startPeriodic(uint32_t dt);
# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
static void BlinkC__Leds__led0Toggle(void );
#line 72
static void BlinkC__Leds__led1Toggle(void );
#line 89
static void BlinkC__Leds__led2Toggle(void );
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
static void BlinkC__Timer2__startPeriodic(uint32_t dt);
# 49 "BlinkC.nc"
static inline void BlinkC__Boot__booted(void );






static inline void BlinkC__Timer0__fired(void );





static inline void BlinkC__Timer1__fired(void );





static inline void BlinkC__Timer2__fired(void );
# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );



static void LedsP__Led0__makeOutput(void );
#line 34
static void LedsP__Led0__clr(void );
static void LedsP__Led1__toggle(void );



static void LedsP__Led1__makeOutput(void );
#line 34
static void LedsP__Led1__clr(void );
static void LedsP__Led2__toggle(void );



static void LedsP__Led2__makeOutput(void );
#line 34
static void LedsP__Led2__clr(void );
# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 76
static inline void LedsP__Leds__led0Toggle(void );
#line 91
static inline void LedsP__Leds__led1Toggle(void );
#line 106
static inline void LedsP__Leds__led2Toggle(void );
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

# 37 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
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
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4043db30){
#line 28
  switch (arg_0x4043db30) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4043db30);
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
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4269 {
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
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4270 {
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
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4271 {
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

# 71 "/home/tinyos/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
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
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4272 {
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
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4273 {
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
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4274 {
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
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4275 {
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
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4276 {
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
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4277 {
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
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4278 {
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
# 71 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__default__fired(void )
#line 71
{
}

# 57 "/home/tinyos/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__fired(void ){
#line 57
  /*Msp430GeneralIOC.GpioInterrupt24*/Msp430InterruptC__12__Interrupt__default__fired();
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

# 45 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/PlatformP.nc"
static inline error_t PlatformP__InitLevel__default__init(uint8_t level)
#line 45
{
#line 45
  return SUCCESS;
}

# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__InitLevel__init(uint8_t arg_0x403e3788){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  switch (arg_0x403e3788) {
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
      __nesc_result = LedsP__Init__init();
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
      __nesc_result = PlatformP__InitLevel__default__init(arg_0x403e3788);
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

# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__toggle(void )
#line 53
{
#line 53
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    * (volatile uint8_t * )33U ^= 0x01 << 7;
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__toggle(void ){
#line 48
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__toggle();
#line 48
}
#line 48
# 42 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__toggle(void )
#line 42
{
#line 42
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__HplGeneralIO__toggle();
}

# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 35
  /*Msp430GeneralIOC.GeneralIO17*/Msp430GpioC__7__GeneralIO__toggle();
#line 35
}
#line 35
# 76 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 76
{
  LedsP__Led0__toggle();
  ;
#line 78
  ;
}

# 56 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC__Leds__led0Toggle(void ){
#line 56
  LedsP__Leds__led0Toggle();
#line 56
}
#line 56
# 56 "BlinkC.nc"
static inline void BlinkC__Timer0__fired(void )
{
  ;
  BlinkC__Leds__led0Toggle();
}

# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle(void )
#line 53
{
#line 53
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    * (volatile uint8_t * )41U ^= 0x01 << 0;
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__toggle(void ){
#line 48
  /*HplMsp430GeneralIOC.P20*/HplMsp430GeneralIOP__8__IO__toggle();
#line 48
}
#line 48
# 42 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__toggle(void )
#line 42
{
#line 42
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__HplGeneralIO__toggle();
}

# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 35
  /*Msp430GeneralIOC.GeneralIO20*/Msp430GpioC__8__GeneralIO__toggle();
#line 35
}
#line 35
# 91 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 91
{
  LedsP__Led1__toggle();
  ;
#line 93
  ;
}

# 72 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 62 "BlinkC.nc"
static inline void BlinkC__Timer1__fired(void )
{
  ;
  BlinkC__Leds__led1Toggle();
}

# 53 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle(void )
#line 53
{
#line 53
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 53
    * (volatile uint8_t * )41U ^= 0x01 << 1;
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__toggle(void ){
#line 48
  /*HplMsp430GeneralIOC.P21*/HplMsp430GeneralIOP__9__IO__toggle();
#line 48
}
#line 48
# 42 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__toggle(void )
#line 42
{
#line 42
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__HplGeneralIO__toggle();
}

# 35 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 35
  /*Msp430GeneralIOC.GeneralIO21*/Msp430GpioC__9__GeneralIO__toggle();
#line 35
}
#line 35
# 106 "/home/tinyos/tinyos-2.x/tos/platforms/gnode/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 106
{
  LedsP__Led2__toggle();
  ;
#line 108
  ;
}

# 89 "/home/tinyos/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void BlinkC__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 68 "BlinkC.nc"
static inline void BlinkC__Timer2__fired(void )
{
  ;
  BlinkC__Leds__led2Toggle();
}

# 193 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x408b79f0){
#line 72
  switch (arg_0x408b79f0) {
#line 72
    case 0U:
#line 72
      StackGuardWatchDogP__Timer__fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      BlinkC__Timer0__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      BlinkC__Timer1__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      BlinkC__Timer2__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x408b79f0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
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
# 46 "/home/tinyos/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4279 {
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

# 51 "/home/tinyos/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 53 "/home/tinyos/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void BlinkC__Timer2__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(3U, dt);
#line 53
}
#line 53
inline static void BlinkC__Timer1__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(2U, dt);
#line 53
}
#line 53
inline static void BlinkC__Timer0__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(1U, dt);
#line 53
}
#line 53
# 49 "BlinkC.nc"
static inline void BlinkC__Boot__booted(void )
{
  BlinkC__Timer0__startPeriodic(2000);
  BlinkC__Timer1__startPeriodic(2000);
  BlinkC__Timer2__startPeriodic(2000);
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
  BlinkC__Boot__booted();
#line 49
}
#line 49
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
# 133 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 164 "/home/tinyos/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/home/tinyos/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x403b2580){
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
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4043db30){
#line 28
  switch (arg_0x4043db30) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4043db30);
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

# 89 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
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

#line 62
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

# 63 "/home/tinyos/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 143 "/home/tinyos/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

