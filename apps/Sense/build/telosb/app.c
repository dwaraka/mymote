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
extern void *memset(void *arg_0x402aa4f8, int arg_0x402aa650, size_t arg_0x402aa7e8);
#line 61
extern void *memset(void *arg_0x402bb358, int arg_0x402bb4b0, size_t arg_0x402bb648);
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

  void (*__cleanup)(struct _reent *arg_0x402da3b8);


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


  void (**_sig_func)(int arg_0x402dea70);




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
# 25 "/opt/tinyos/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/opt/tinyos/tos/types/TinyError.h"
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
# 193 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/msp430f1611.h" 3
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 265
extern volatile unsigned char P1OUT __asm ("__""P1OUT");

extern volatile unsigned char P1DIR __asm ("__""P1DIR");





extern volatile unsigned char P1IE __asm ("__""P1IE");

extern volatile unsigned char P1SEL __asm ("__""P1SEL");




extern volatile unsigned char P2OUT __asm ("__""P2OUT");

extern volatile unsigned char P2DIR __asm ("__""P2DIR");





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
#line 593
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");
#line 697
extern volatile unsigned int TBCCTL0 __asm ("__""TBCCTL0");
#line 711
extern volatile unsigned int TBR __asm ("__""TBR");

extern volatile unsigned int TBCCR0 __asm ("__""TBCCR0");
#line 790
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");

extern volatile unsigned char BCSCTL2 __asm ("__""BCSCTL2");
#line 962
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
# 378 "/opt/tinyos/tos/chips/msp430/msp430hardware.h"
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
# 8 "/opt/tinyos/tos/platforms/telosb/hardware.h"
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
# 41 "/opt/tinyos/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4253 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4254 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4255 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4256 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 43 "/opt/tinyos/tos/types/Leds.h"
enum __nesc_unnamed4257 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4258 {
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
typedef struct __nesc_unnamed4259 {

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
typedef struct __nesc_unnamed4260 {

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
typedef struct __nesc_unnamed4261 {

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
# 59 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12.h"
#line 48
typedef struct __nesc_unnamed4262 {

  unsigned int inch : 4;
  unsigned int sref : 3;
  unsigned int ref2_5v : 1;
  unsigned int adc12ssel : 2;
  unsigned int adc12div : 3;
  unsigned int sht : 4;
  unsigned int sampcon_ssel : 2;
  unsigned int sampcon_id : 2;
  unsigned int  : 0;
} msp430adc12_channel_config_t;








#line 61
typedef struct __nesc_unnamed4263 {


  volatile unsigned 
  inch : 4, 
  sref : 3, 
  eos : 1;
} __attribute((packed))  adc12memctl_t;

enum inch_enum {


  INPUT_CHANNEL_A0 = 0, 
  INPUT_CHANNEL_A1 = 1, 
  INPUT_CHANNEL_A2 = 2, 
  INPUT_CHANNEL_A3 = 3, 
  INPUT_CHANNEL_A4 = 4, 
  INPUT_CHANNEL_A5 = 5, 
  INPUT_CHANNEL_A6 = 6, 
  INPUT_CHANNEL_A7 = 7, 
  EXTERNAL_REF_VOLTAGE_CHANNEL = 8, 
  REF_VOLTAGE_NEG_TERMINAL_CHANNEL = 9, 
  TEMPERATURE_DIODE_CHANNEL = 10, 
  SUPPLY_VOLTAGE_HALF_CHANNEL = 11, 
  INPUT_CHANNEL_NONE = 12
};

enum sref_enum {

  REFERENCE_AVcc_AVss = 0, 
  REFERENCE_VREFplus_AVss = 1, 
  REFERENCE_VeREFplus_AVss = 2, 
  REFERENCE_AVcc_VREFnegterm = 4, 
  REFERENCE_VREFplus_VREFnegterm = 5, 
  REFERENCE_VeREFplus_VREFnegterm = 6
};

enum ref2_5v_enum {

  REFVOLT_LEVEL_1_5 = 0, 
  REFVOLT_LEVEL_2_5 = 1, 
  REFVOLT_LEVEL_NONE = 0
};

enum adc12ssel_enum {

  SHT_SOURCE_ADC12OSC = 0, 
  SHT_SOURCE_ACLK = 1, 
  SHT_SOURCE_MCLK = 2, 
  SHT_SOURCE_SMCLK = 3
};

enum adc12div_enum {

  SHT_CLOCK_DIV_1 = 0, 
  SHT_CLOCK_DIV_2 = 1, 
  SHT_CLOCK_DIV_3 = 2, 
  SHT_CLOCK_DIV_4 = 3, 
  SHT_CLOCK_DIV_5 = 4, 
  SHT_CLOCK_DIV_6 = 5, 
  SHT_CLOCK_DIV_7 = 6, 
  SHT_CLOCK_DIV_8 = 7
};

enum sht_enum {

  SAMPLE_HOLD_4_CYCLES = 0, 
  SAMPLE_HOLD_8_CYCLES = 1, 
  SAMPLE_HOLD_16_CYCLES = 2, 
  SAMPLE_HOLD_32_CYCLES = 3, 
  SAMPLE_HOLD_64_CYCLES = 4, 
  SAMPLE_HOLD_96_CYCLES = 5, 
  SAMPLE_HOLD_128_CYCLES = 6, 
  SAMPLE_HOLD_192_CYCLES = 7, 
  SAMPLE_HOLD_256_CYCLES = 8, 
  SAMPLE_HOLD_384_CYCLES = 9, 
  SAMPLE_HOLD_512_CYCLES = 10, 
  SAMPLE_HOLD_768_CYCLES = 11, 
  SAMPLE_HOLD_1024_CYCLES = 12
};

enum sampcon_ssel_enum {

  SAMPCON_SOURCE_TACLK = 0, 
  SAMPCON_SOURCE_ACLK = 1, 
  SAMPCON_SOURCE_SMCLK = 2, 
  SAMPCON_SOURCE_INCLK = 3
};

enum sampcon_id_enum {

  SAMPCON_CLOCK_DIV_1 = 0, 
  SAMPCON_CLOCK_DIV_2 = 1, 
  SAMPCON_CLOCK_DIV_4 = 2, 
  SAMPCON_CLOCK_DIV_8 = 3
};
#line 199
#line 186
typedef struct __nesc_unnamed4264 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
  volatile unsigned int  : 0;
} __attribute((packed))  adc12ctl0_t;
#line 212
#line 201
typedef struct __nesc_unnamed4265 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
  volatile unsigned int  : 0;
} __attribute((packed))  adc12ctl1_t;
# 33 "/opt/tinyos/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef uint16_t SenseC__Read__val_t;
typedef TMilli SenseC__Timer__precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4266 {
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
typedef uint16_t AdcP__Read__val_t;
typedef uint16_t AdcP__ReadNow__val_t;
typedef const msp430adc12_channel_config_t *AdcP__Config__adc_config_t;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOffTimer__precision_tag;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOffSettleTimer__precision_tag;
typedef TMilli Msp430RefVoltGeneratorP__SwitchOnTimer__precision_tag;
typedef const msp430adc12_channel_config_t *Msp430RefVoltArbiterImplP__Config__adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0____nesc_unnamed4267 {
  Msp430Adc12ClientAutoRVGC__0__ID = 0U
};
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0____nesc_unnamed4268 {
  AdcReadClientC__0__CLIENT = 0U
};
typedef TMilli AdcStreamP__Alarm__precision_tag;
typedef uint32_t AdcStreamP__Alarm__size_type;
typedef const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__adc_config_t;
typedef uint16_t AdcStreamP__ReadStream__val_t;
enum /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4269 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMilli /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_precision_tag /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__size_type;
typedef uint16_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t;
typedef /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1____nesc_unnamed4270 {
  Msp430Adc12ClientAutoRVGC__1__ID = 1U
};
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0____nesc_unnamed4271 {
  AdcReadStreamClientC__0__RSCLIENT = 0U
};
typedef const msp430adc12_channel_config_t *Msp430InternalVoltageP__AdcConfigure__adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2____nesc_unnamed4272 {
  Msp430Adc12ClientAutoRVGC__2__ID = 2U
};
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__adc_config_t;
typedef const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__adc_config_t;
enum /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0____nesc_unnamed4273 {
  AdcReadNowClientC__0__CLIENT = 1U
};
# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
static void SenseC__Boot__booted(void );
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
static void SenseC__Read__readDone(error_t result, SenseC__Read__val_t val);
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void SenseC__Timer__fired(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 62
static error_t MotePlatformC__Init__init(void );
# 46 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
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
# 62 "/opt/tinyos/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4064a620);
# 52 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void );


static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource);
#line 54
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void );
#line 50
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode);





static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider);
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4064a620);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );



static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(msp430_compare_control_t control);
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t time);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );



static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(msp430_compare_control_t control);
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t time);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 47
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/opt/tinyos/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405964a0);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405964a0);
# 57 "/opt/tinyos/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 61 "/opt/tinyos/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Off(void );










static void LedsP__Leds__led1On(void );




static void LedsP__Leds__led1Off(void );
#line 94
static void LedsP__Leds__led2Off(void );
#line 56
static void LedsP__Leds__led0On(void );
#line 89
static void LedsP__Leds__led2On(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
#line 78
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void );
#line 99
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void );
# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 136 "/opt/tinyos/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 83
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408dd030);
# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408dd030);
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408dd030, 
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408dd030, 
# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408dd030);
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void AdcP__SubResourceReadNow__granted(
# 46 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094c010);
# 55 "/opt/tinyos/tos/interfaces/Read.nc"
static error_t AdcP__Read__read(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40934950);
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
static void AdcP__Read__default__readDone(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40934950, 
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
# 66 "/opt/tinyos/tos/interfaces/ReadNow.nc"
static void AdcP__ReadNow__default__readDone(
# 39 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40950968, 
# 66 "/opt/tinyos/tos/interfaces/ReadNow.nc"
error_t result, AdcP__ReadNow__val_t val);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void AdcP__ResourceReadNow__default__granted(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094f930);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static AdcP__Config__adc_config_t AdcP__Config__default__getConfiguration(
# 48 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094cc90);
# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__default__getData(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4095fe40);
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__default__configureSingle(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4095fe40, 
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcP__SingleChannel__multipleDataReady(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4095fe40, 
# 227 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t AdcP__SingleChannel__singleDataReady(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4095fe40, 
# 206 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__default__release(
# 44 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094e4b0);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__default__request(
# 44 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094e4b0);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void AdcP__ResourceRead__granted(
# 44 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094e4b0);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void AdcP__readDone__runTask(void );
# 107 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP__MultiChannel__default__dataReady(
# 42 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40984108, 
# 107 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 112 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA1__fired(void );
# 49 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__default__memOverflow(
# 43 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409849f8);
# 54 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(
# 43 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409849f8);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t Msp430Adc12ImplP__Init__init(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP__TimerA__overflow(void );
# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP__SingleChannel__getData(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0);
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0, 
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0, 
# 227 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0, 
# 138 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0, 
# 206 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA0__fired(void );
# 63 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void );
#line 82
static adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t idx);
#line 106
static void HplAdc12P__HplAdc12__resetIFGs(void );
#line 118
static bool HplAdc12P__HplAdc12__isBusy(void );
#line 75
static void HplAdc12P__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void HplAdc12P__HplAdc12__startConversion(void );
#line 51
static void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t HplAdc12P__HplAdc12__getMem(uint8_t idx);





static void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask);
#line 123
static void HplAdc12P__HplAdc12__stopConversion(void );
#line 57
static void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void );
# 79 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void );
# 53 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(
# 52 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a5d948);
# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(
# 56 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a5b5c0);
# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(
# 56 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a5b5c0);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(
# 51 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a4be98);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(
# 51 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a4be98);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(
# 51 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a4be98);
# 98 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );
# 112 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
static void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv);
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void );
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void );
#line 130
static error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__stop(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__fired(void );
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void );
#line 130
static error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void );
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__default__getConfiguration(
# 43 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa8ab0);
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error);
#line 138
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa9010);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa9010);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__AdcResource__granted(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa9010);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__release(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aaa4c0);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__request(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aaa4c0);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__ClientResource__default__granted(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aaa4c0);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void Msp430RefVoltArbiterImplP__switchOff__runTask(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error);
#line 138
static void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void AdcStreamP__bufferDone__runTask(void );
#line 75
static void AdcStreamP__readStreamDone__runTask(void );
#line 75
static void AdcStreamP__readStreamFail__runTask(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void AdcStreamP__Alarm__fired(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t AdcStreamP__Init__init(void );
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__default__getConfiguration(
# 53 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af4878);
# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__default__getData(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18);
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__default__configureSingle(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18, 
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 227
static uint16_t * AdcStreamP__SingleChannel__multipleDataReady(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18, 
# 227 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 138
static error_t AdcStreamP__SingleChannel__default__configureMultiple(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18, 
# 138 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
#line 206
static error_t AdcStreamP__SingleChannel__singleDataReady(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18, 
# 206 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 78 "/opt/tinyos/tos/interfaces/ReadStream.nc"
static error_t AdcStreamP__ReadStream__read(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40afaf28, 
# 78 "/opt/tinyos/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);





static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 103
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 78
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(
# 26 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4d6c8, 
# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(
# 26 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4d6c8, 
# 102 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
#line 89
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(
# 24 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4fbb0, 
# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(
# 24 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4fbb0, 
# 102 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(
# 27 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4a030);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(
# 27 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4a030);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void );
#line 58
static Msp430InternalVoltageP__AdcConfigure__adc_config_t Msp430InternalVoltageP__AdcConfigure__getConfiguration(void );
#line 58
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration(void );
# 55 "/opt/tinyos/tos/interfaces/Read.nc"
static error_t SenseC__Read__read(void );
# 61 "/opt/tinyos/tos/interfaces/Leds.nc"
static void SenseC__Leds__led0Off(void );










static void SenseC__Leds__led1On(void );




static void SenseC__Leds__led1Off(void );
#line 94
static void SenseC__Leds__led2Off(void );
#line 56
static void SenseC__Leds__led0On(void );
#line 89
static void SenseC__Leds__led2On(void );
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void SenseC__Timer__startPeriodic(uint32_t dt);
# 61 "SenseC.nc"
static inline void SenseC__Boot__booted(void );



static inline void SenseC__Timer__fired(void );




static void SenseC__Read__readDone(error_t result, uint16_t data);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 62
static error_t PlatformP__MoteClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 10 "/opt/tinyos/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/opt/tinyos/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 43 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 40
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4274 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 79
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 100
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 163
static inline void Msp430ClockP__startTimerB(void );
#line 175
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 204
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 229
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4064a620);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 91 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode);









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void );









static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider);




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4064a620);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 62 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 81
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 100
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x);
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x);
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 100
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x);
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t x);
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
#line 130
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000C)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000A)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x001A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0018)))  ;
# 62 "/opt/tinyos/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/opt/tinyos/tos/chips/msp430/McuSleepC.nc"
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
#line 112
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/opt/tinyos/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/opt/tinyos/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x405964a0);
# 76 "/opt/tinyos/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4275 {

  SchedulerBasicP__NUM_TASKS = 8U, 
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
# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );




static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );




static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
static void LedsP__Led2__clr(void );
# 56 "/opt/tinyos/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 74
static inline void LedsP__Leds__led0On(void );




static inline void LedsP__Leds__led0Off(void );









static inline void LedsP__Leds__led1On(void );




static inline void LedsP__Leds__led1Off(void );









static inline void LedsP__Leds__led2On(void );




static inline void LedsP__Leds__led2Off(void );
# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void );
#line 61
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );




static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4276 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4277 {

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
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 74 "/opt/tinyos/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4278 {
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
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 136 "/opt/tinyos/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 129
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x408dd030);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4279 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4280 {

  VirtualizeTimerC__0__NUM_TIMERS = 4U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4281 {

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




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/opt/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
static void AdcP__Read__readDone(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40934950, 
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
error_t result, AdcP__Read__val_t val);
# 66 "/opt/tinyos/tos/interfaces/ReadNow.nc"
static void AdcP__ReadNow__readDone(
# 39 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x40950968, 
# 66 "/opt/tinyos/tos/interfaces/ReadNow.nc"
error_t result, AdcP__ReadNow__val_t val);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void AdcP__ResourceReadNow__granted(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094f930);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static AdcP__Config__adc_config_t AdcP__Config__getConfiguration(
# 48 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094cc90);
# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__getData(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4095fe40);
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcP__SingleChannel__configureSingle(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4095fe40, 
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__release(
# 44 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094e4b0);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t AdcP__ResourceRead__request(
# 44 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
uint8_t arg_0x4094e4b0);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t AdcP__readDone__postTask(void );
# 136 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
enum AdcP____nesc_unnamed4282 {
#line 136
  AdcP__readDone = 2U
};
#line 136
typedef int AdcP____nesc_sillytask_readDone[AdcP__readDone];
#line 54
enum AdcP____nesc_unnamed4283 {
  AdcP__STATE_READ, 
  AdcP__STATE_READNOW, 
  AdcP__STATE_READNOW_INVALID_CONFIG
};


uint8_t AdcP__state;
uint8_t AdcP__owner;
uint16_t AdcP__value;

static error_t AdcP__configure(uint8_t client);









static inline error_t AdcP__Read__read(uint8_t client);




static void AdcP__ResourceRead__granted(uint8_t client);
#line 98
static void AdcP__SubResourceReadNow__granted(uint8_t nowClient);
#line 136
static inline void AdcP__readDone__runTask(void );





static error_t AdcP__SingleChannel__singleDataReady(uint8_t client, uint16_t data);
#line 161
static inline uint16_t *AdcP__SingleChannel__multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples);





static inline error_t AdcP__ResourceRead__default__request(uint8_t client);

static inline error_t AdcP__ResourceRead__default__release(uint8_t client);

static inline void AdcP__Read__default__readDone(uint8_t client, error_t result, uint16_t val);




static inline void AdcP__ResourceReadNow__default__granted(uint8_t nowClient);
static inline void AdcP__ReadNow__default__readDone(uint8_t client, error_t result, uint16_t val);

static inline error_t AdcP__SingleChannel__default__getData(uint8_t client);




const msp430adc12_channel_config_t AdcP__defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
AdcP__Config__default__getConfiguration(uint8_t client);



static inline error_t AdcP__SingleChannel__default__configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config);
# 107 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
static void Msp430Adc12ImplP__MultiChannel__dataReady(
# 42 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x40984108, 
# 107 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
uint16_t *buffer, uint16_t numSamples);
# 63 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430Adc12ImplP__HplAdc12__getCtl0(void );
#line 82
static adc12memctl_t Msp430Adc12ImplP__HplAdc12__getMCtl(uint8_t idx);
#line 106
static void Msp430Adc12ImplP__HplAdc12__resetIFGs(void );
#line 75
static void Msp430Adc12ImplP__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl);
#line 128
static void Msp430Adc12ImplP__HplAdc12__startConversion(void );
#line 51
static void Msp430Adc12ImplP__HplAdc12__setCtl0(adc12ctl0_t control0);
#line 89
static uint16_t Msp430Adc12ImplP__HplAdc12__getMem(uint8_t idx);





static void Msp430Adc12ImplP__HplAdc12__setIEFlags(uint16_t mask);
#line 123
static void Msp430Adc12ImplP__HplAdc12__stopConversion(void );
#line 57
static void Msp430Adc12ImplP__HplAdc12__setCtl1(adc12ctl1_t control1);
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port64__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port64__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port64__selectModuleFunc(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA1__setEvent(uint16_t time);
# 46 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP__ControlA0__setControl(msp430_compare_control_t control);
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port62__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port62__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port62__selectModuleFunc(void );
# 49 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__memOverflow(
# 43 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409849f8);
# 54 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
static void Msp430Adc12ImplP__Overflow__conversionTimeOverflow(
# 43 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409849f8);
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port67__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port67__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port67__selectModuleFunc(void );
#line 78
static void Msp430Adc12ImplP__Port60__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port60__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port60__selectModuleFunc(void );
#line 78
static void Msp430Adc12ImplP__Port65__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port65__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port65__selectModuleFunc(void );
# 52 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void Msp430Adc12ImplP__TimerA__clear(void );


static void Msp430Adc12ImplP__TimerA__setClockSource(uint16_t clockSource);
#line 54
static void Msp430Adc12ImplP__TimerA__disableEvents(void );
#line 50
static void Msp430Adc12ImplP__TimerA__setMode(int mode);





static void Msp430Adc12ImplP__TimerA__setInputDivider(uint16_t inputDivider);
# 98 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
static uint8_t Msp430Adc12ImplP__ADCArbiterInfo__userId(void );
# 46 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void Msp430Adc12ImplP__ControlA1__setControl(msp430_compare_control_t control);
# 227 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static uint16_t * Msp430Adc12ImplP__SingleChannel__multipleDataReady(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0, 
# 227 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t * buffer, uint16_t numSamples);
#line 206
static error_t Msp430Adc12ImplP__SingleChannel__singleDataReady(
# 41 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
uint8_t arg_0x409853f0, 
# 206 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
uint16_t data);
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port63__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port63__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port63__selectModuleFunc(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void Msp430Adc12ImplP__CompareA0__setEvent(uint16_t time);
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void Msp430Adc12ImplP__Port61__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port61__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port61__selectModuleFunc(void );
#line 78
static void Msp430Adc12ImplP__Port66__makeInput(void );
#line 99
static void Msp430Adc12ImplP__Port66__selectIOFunc(void );
#line 92
static void Msp430Adc12ImplP__Port66__selectModuleFunc(void );
# 71 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
enum Msp430Adc12ImplP____nesc_unnamed4284 {
  Msp430Adc12ImplP__SINGLE_DATA = 1, 
  Msp430Adc12ImplP__SINGLE_DATA_REPEAT = 2, 
  Msp430Adc12ImplP__MULTIPLE_DATA = 4, 
  Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT = 8, 
  Msp430Adc12ImplP__MULTI_CHANNEL = 16, 
  Msp430Adc12ImplP__CONVERSION_MODE_MASK = 0x1F, 

  Msp430Adc12ImplP__ADC_BUSY = 32, 
  Msp430Adc12ImplP__USE_TIMERA = 64, 
  Msp430Adc12ImplP__ADC_OVERFLOW = 128
};

uint8_t Msp430Adc12ImplP__state;

uint16_t Msp430Adc12ImplP__resultBufferLength;
uint16_t * Msp430Adc12ImplP__resultBufferStart;
uint16_t Msp430Adc12ImplP__resultBufferIndex;
uint8_t Msp430Adc12ImplP__numChannels;
uint8_t Msp430Adc12ImplP__clientID;

static inline error_t Msp430Adc12ImplP__Init__init(void );
#line 109
static inline void Msp430Adc12ImplP__prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON);
#line 127
static inline void Msp430Adc12ImplP__startTimerA(void );
#line 148
static inline void Msp430Adc12ImplP__configureAdcPin(uint8_t inch);
#line 165
static void Msp430Adc12ImplP__resetAdcPin(uint8_t inch);
#line 182
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config);
#line 277
static inline error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies);
#line 400
static error_t Msp430Adc12ImplP__SingleChannel__getData(uint8_t id);
#line 509
static void Msp430Adc12ImplP__stopConversion(void );
#line 546
static inline void Msp430Adc12ImplP__TimerA__overflow(void );
static inline void Msp430Adc12ImplP__CompareA0__fired(void );
static inline void Msp430Adc12ImplP__CompareA1__fired(void );

static inline void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv);
#line 649
static inline error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(uint8_t id, uint16_t data);




static inline uint16_t *Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples);




static inline void Msp430Adc12ImplP__MultiChannel__default__dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples);

static inline void Msp430Adc12ImplP__Overflow__default__memOverflow(uint8_t id);
static inline void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(uint8_t id);
# 112 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
static void HplAdc12P__HplAdc12__conversionDone(uint16_t iv);
# 51 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static volatile uint16_t HplAdc12P__ADC12CTL0 __asm ("0x01A0");
static volatile uint16_t HplAdc12P__ADC12CTL1 __asm ("0x01A2");
static volatile uint16_t HplAdc12P__ADC12IFG __asm ("0x01A4");
static volatile uint16_t HplAdc12P__ADC12IE __asm ("0x01A6");
static volatile uint16_t HplAdc12P__ADC12IV __asm ("0x01A8");

static inline adc12ctl0_t HplAdc12P__int2adc12ctl0(uint16_t x)  ;

static inline uint16_t HplAdc12P__adc12ctl0cast2int(adc12ctl0_t x)  ;
static inline uint16_t HplAdc12P__adc12ctl1cast2int(adc12ctl1_t x)  ;
static inline uint8_t HplAdc12P__adc12memctl2int(adc12memctl_t x)  ;
static inline adc12memctl_t HplAdc12P__int2adc12memctl(uint8_t x)  ;

static inline void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0);



static inline void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1);



static inline adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void );







static inline void HplAdc12P__HplAdc12__setMCtl(uint8_t i, adc12memctl_t memCtl);



static inline adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t i);



static inline uint16_t HplAdc12P__HplAdc12__getMem(uint8_t i);



static inline void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask);


static inline void HplAdc12P__HplAdc12__resetIFGs(void );




static inline void HplAdc12P__HplAdc12__startConversion(void );




static void HplAdc12P__HplAdc12__stopConversion(void );
#line 118
static inline bool HplAdc12P__HplAdc12__isBusy(void );

void sig_ADC12_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x000E)))  ;
# 49 "/opt/tinyos/tos/system/RoundRobinResourceQueueC.nc"
enum /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0____nesc_unnamed4285 {
  RoundRobinResourceQueueC__0__NO_ENTRY = 0xFF, 
  RoundRobinResourceQueueC__0__SIZE = 3U ? (3U - 1) / 8 + 1 : 0
};

uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__SIZE];
uint8_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last = 0;

static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(uint8_t id);



static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void );




static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void );








static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void );
#line 97
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id);
# 53 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(
# 52 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a5d948);
# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(
# 56 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a5b5c0);
# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(
# 56 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a5b5c0);
# 79 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(
# 51 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x40a4be98);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void );
# 69 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4286 {
#line 69
  SimpleArbiterP__0__grantedTask = 3U
};
#line 69
typedef int /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_sillytask_grantedTask[/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask];
#line 62
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4287 {
#line 62
  SimpleArbiterP__0__RES_IDLE = 0, SimpleArbiterP__0__RES_GRANTING = 1, SimpleArbiterP__0__RES_BUSY = 2
};
#line 63
enum /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0____nesc_unnamed4288 {
#line 63
  SimpleArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId;



static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id);
#line 97
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id);
#line 137
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void );
#line 155
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void );









static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id);



static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 63 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
static adc12ctl0_t Msp430RefVoltGeneratorP__HplAdc12__getCtl0(void );
#line 118
static bool Msp430RefVoltGeneratorP__HplAdc12__isBusy(void );
#line 51
static void Msp430RefVoltGeneratorP__HplAdc12__setCtl0(adc12ctl0_t control0);
# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOffTimer__stop(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(error_t error);
#line 138
static void Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(error_t error);
# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
static bool Msp430RefVoltGeneratorP__SwitchOffSettleTimer__isRunning(void );
#line 73
static void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__stop(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(error_t error);
#line 138
static void Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(error_t error);
# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(uint32_t dt);




static void Msp430RefVoltGeneratorP__SwitchOnTimer__stop(void );
# 66 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
#line 53
typedef enum Msp430RefVoltGeneratorP____nesc_unnamed4289 {

  Msp430RefVoltGeneratorP__GENERATOR_OFF = 0, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE = 1, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE = 2, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING = 3, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING = 4, 

  Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING = 5, 
  Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING = 6
} 
Msp430RefVoltGeneratorP__state_t;

Msp430RefVoltGeneratorP__state_t Msp430RefVoltGeneratorP__m_state;


static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level);
static error_t Msp430RefVoltGeneratorP__switchOff(void );
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result);
static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result);
static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState);
static error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState);


static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void );



static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__stop(void );



static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState);
#line 147
static error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState);
#line 173
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result);






static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result);







static inline void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void );
#line 205
static inline void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void );
#line 233
static inline void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__fired(void );


static inline void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv);



static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level);
#line 259
static error_t Msp430RefVoltGeneratorP__switchOff(void );
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__getConfiguration(
# 43 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa8ab0);
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__start(void );
#line 130
static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__stop(void );
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__release(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa9010);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t Msp430RefVoltArbiterImplP__AdcResource__request(
# 40 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aa9010);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void Msp430RefVoltArbiterImplP__ClientResource__granted(
# 38 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
uint8_t arg_0x40aaa4c0);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t Msp430RefVoltArbiterImplP__switchOff__postTask(void );
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__start(void );
#line 130
static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop(void );
# 52 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
enum Msp430RefVoltArbiterImplP____nesc_unnamed4290 {
#line 52
  Msp430RefVoltArbiterImplP__switchOff = 4U
};
#line 52
typedef int Msp430RefVoltArbiterImplP____nesc_sillytask_switchOff[Msp430RefVoltArbiterImplP__switchOff];
#line 46
enum Msp430RefVoltArbiterImplP____nesc_unnamed4291 {
  Msp430RefVoltArbiterImplP__NO_OWNER = 0xFF
};
uint8_t Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
bool Msp430RefVoltArbiterImplP__ref2_5v;



static inline error_t Msp430RefVoltArbiterImplP__ClientResource__request(uint8_t client);
#line 71
static void Msp430RefVoltArbiterImplP__AdcResource__granted(uint8_t client);
#line 103
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error);








static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error);








static error_t Msp430RefVoltArbiterImplP__ClientResource__release(uint8_t client);
#line 141
static inline void Msp430RefVoltArbiterImplP__switchOff__runTask(void );
#line 157
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error);



static inline void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error);








static inline void Msp430RefVoltArbiterImplP__ClientResource__default__granted(uint8_t client);
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(uint8_t client);








static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(uint8_t client);
const msp430adc12_channel_config_t Msp430RefVoltArbiterImplP__defaultConfig = { INPUT_CHANNEL_NONE, 0, 0, 0, 0, 0, 0, 0 };
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP__Config__default__getConfiguration(uint8_t client);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration(void );
# 47 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void );
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t AdcStreamP__bufferDone__postTask(void );
#line 67
static error_t AdcStreamP__readStreamDone__postTask(void );
#line 67
static error_t AdcStreamP__readStreamFail__postTask(void );
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static AdcStreamP__Alarm__size_type AdcStreamP__Alarm__getNow(void );
#line 103
static void AdcStreamP__Alarm__startAt(AdcStreamP__Alarm__size_type t0, AdcStreamP__Alarm__size_type dt);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__getConfiguration(
# 53 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af4878);
# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__getData(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18);
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
static error_t AdcStreamP__SingleChannel__configureSingle(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18, 
# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config);
#line 138
static error_t AdcStreamP__SingleChannel__configureMultiple(
# 52 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40af5a18, 
# 138 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies);
# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
static void AdcStreamP__ReadStream__bufferDone(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40afaf28, 
# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
AdcStreamP__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void AdcStreamP__ReadStream__readDone(
# 49 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
uint8_t arg_0x40afaf28, 
# 102 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 119 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
enum AdcStreamP____nesc_unnamed4292 {
#line 119
  AdcStreamP__readStreamDone = 5U
};
#line 119
typedef int AdcStreamP____nesc_sillytask_readStreamDone[AdcStreamP__readStreamDone];
#line 135
enum AdcStreamP____nesc_unnamed4293 {
#line 135
  AdcStreamP__readStreamFail = 6U
};
#line 135
typedef int AdcStreamP____nesc_sillytask_readStreamFail[AdcStreamP__readStreamFail];
#line 156
enum AdcStreamP____nesc_unnamed4294 {
#line 156
  AdcStreamP__bufferDone = 7U
};
#line 156
typedef int AdcStreamP____nesc_sillytask_bufferDone[AdcStreamP__bufferDone];
#line 58
enum AdcStreamP____nesc_unnamed4295 {
  AdcStreamP__NSTREAM = 1U
};




uint8_t AdcStreamP__client = AdcStreamP__NSTREAM;


struct AdcStreamP__list_entry_t {
  uint16_t count;
  struct AdcStreamP__list_entry_t * next;
};
struct AdcStreamP__list_entry_t *AdcStreamP__bufferQueue[AdcStreamP__NSTREAM];
struct AdcStreamP__list_entry_t * *AdcStreamP__bufferQueueEnd[AdcStreamP__NSTREAM];
uint16_t * AdcStreamP__lastBuffer;
#line 74
uint16_t AdcStreamP__lastCount;

uint16_t AdcStreamP__count;
uint16_t * AdcStreamP__buffer;
uint16_t * AdcStreamP__pos;
uint32_t AdcStreamP__now;
#line 79
uint32_t AdcStreamP__period;
bool AdcStreamP__periodModified;


static inline error_t AdcStreamP__Init__init(void );








static inline void AdcStreamP__sampleSingle(void );



static inline error_t AdcStreamP__postBuffer(uint8_t c, uint16_t *buf, uint16_t n);
#line 119
static inline void AdcStreamP__readStreamDone__runTask(void );
#line 135
static inline void AdcStreamP__readStreamFail__runTask(void );
#line 156
static inline void AdcStreamP__bufferDone__runTask(void );
#line 168
static inline void AdcStreamP__nextAlarm(void );




static inline void AdcStreamP__Alarm__fired(void );



static error_t AdcStreamP__nextBuffer(bool startNextAlarm);
#line 206
static void AdcStreamP__nextMultiple(uint8_t c);
#line 221
static error_t AdcStreamP__ReadStream__read(uint8_t c, uint32_t usPeriod);
#line 242
static error_t AdcStreamP__SingleChannel__singleDataReady(uint8_t streamClient, uint16_t data);
#line 281
static uint16_t *AdcStreamP__SingleChannel__multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length);
#line 304
const msp430adc12_channel_config_t AdcStreamP__defaultConfig = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };

static inline const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__default__getConfiguration(uint8_t c);



static inline error_t AdcStreamP__SingleChannel__default__configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies);



static inline error_t AdcStreamP__SingleChannel__default__getData(uint8_t c);



static inline error_t AdcStreamP__SingleChannel__default__configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config);
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 44
static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 70 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 103
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get(void );
# 77 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0;
/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt;

enum /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1____nesc_unnamed4296 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 5, 
  TransformAlarmC__1__MAX_DELAY = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type )1 << /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 107
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm(void );
#line 147
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type dt);
#line 162
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 177
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 78 "/opt/tinyos/tos/interfaces/ReadStream.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(
# 26 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4d6c8, 
# 78 "/opt/tinyos/tos/interfaces/ReadStream.nc"
uint32_t usPeriod);










static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(
# 24 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4fbb0, 
# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, 
#line 86
/*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, 



uint16_t count);
#line 102
static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(
# 24 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4fbb0, 
# 102 "/opt/tinyos/tos/interfaces/ReadStream.nc"
error_t result, uint32_t usActualPeriod);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(
# 27 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
uint8_t arg_0x40b4a030);



uint32_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[1U];
#line 48
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);




static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(uint8_t client, error_t result, uint32_t actualPeriod);





static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(uint8_t client);







static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(uint8_t client);
#line 79
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count);



static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(uint8_t client, error_t result, uint32_t actualPeriod);
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration(void );
# 47 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void );
# 39 "/opt/tinyos/tos/chips/msp430/sensors/Msp430InternalVoltageP.nc"
const msp430adc12_channel_config_t Msp430InternalVoltageP__config = { 
.inch = SUPPLY_VOLTAGE_HALF_CHANNEL, 
.sref = REFERENCE_VREFplus_AVss, 
.ref2_5v = REFVOLT_LEVEL_1_5, 
.adc12ssel = SHT_SOURCE_ACLK, 
.adc12div = SHT_CLOCK_DIV_1, 
.sht = SAMPLE_HOLD_4_CYCLES, 
.sampcon_ssel = SAMPCON_SOURCE_SMCLK, 
.sampcon_id = SAMPCON_CLOCK_DIV_1 };


static inline const msp430adc12_channel_config_t *Msp430InternalVoltageP__AdcConfigure__getConfiguration(void );
# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__getConfiguration(void );
# 47 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration(void );
# 397 "/opt/tinyos/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 196 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 546 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__TimerA__overflow(void )
#line 546
{
}

# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  Msp430Adc12ImplP__TimerA__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4064a620){
#line 39
  switch (arg_0x4064a620) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4064a620);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4297 {
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

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

# 547 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__CompareA0__fired(void )
#line 547
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  Msp430Adc12ImplP__CompareA0__fired();
#line 45
}
#line 45
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4298 {
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

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

# 548 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__CompareA1__fired(void )
#line 548
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  Msp430Adc12ImplP__CompareA1__fired();
#line 45
}
#line 45
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4299 {
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

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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

# 114 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 58 "/opt/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 177 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

#line 177
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__overflow();
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
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

# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
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
# 137 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
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
# 81 "/opt/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
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

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4300 {
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

# 97 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
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

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
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
# 49 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
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
# 81 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 46 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 54 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 71 "/opt/tinyos/tos/lib/timer/Counter.nc"
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
# 130 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 57
}
#line 57
# 95 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 165 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 43 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
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
# 81 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
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

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 192 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4301 {
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

# 324 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__getData(uint8_t c)
{
  return FAIL;
}

# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__getData(uint8_t arg_0x40af5a18){
#line 189
  unsigned char __nesc_result;
#line 189

#line 189
  switch (arg_0x40af5a18) {
#line 189
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 189
      break;
#line 189
    default:
#line 189
      __nesc_result = AdcStreamP__SingleChannel__default__getData(arg_0x40af5a18);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return __nesc_result;
#line 189
}
#line 189
# 92 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__sampleSingle(void )
#line 92
{
  AdcStreamP__SingleChannel__getData(AdcStreamP__client);
}

#line 173
static inline void AdcStreamP__Alarm__fired(void )
#line 173
{
  AdcStreamP__sampleSingle();
}

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 78
  AdcStreamP__Alarm__fired();
#line 78
}
#line 78
# 162 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 78
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4302 {
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

# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 7);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 7;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 6);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 6;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 5);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 5;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 4);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 4;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 3);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 3;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 2);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 2;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 1);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 1;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectModuleFunc();
#line 92
}
#line 92
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )54U &= ~(0x01 << 0);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__makeInput();
#line 78
}
#line 78
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )55U |= 0x01 << 0;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectModuleFunc();
#line 92
}
#line 92
# 148 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__configureAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP__Port60__selectModuleFunc();
#line 153
      Msp430Adc12ImplP__Port60__makeInput();
#line 153
      break;
      case 1: Msp430Adc12ImplP__Port61__selectModuleFunc();
#line 154
      Msp430Adc12ImplP__Port61__makeInput();
#line 154
      break;
      case 2: Msp430Adc12ImplP__Port62__selectModuleFunc();
#line 155
      Msp430Adc12ImplP__Port62__makeInput();
#line 155
      break;
      case 3: Msp430Adc12ImplP__Port63__selectModuleFunc();
#line 156
      Msp430Adc12ImplP__Port63__makeInput();
#line 156
      break;
      case 4: Msp430Adc12ImplP__Port64__selectModuleFunc();
#line 157
      Msp430Adc12ImplP__Port64__makeInput();
#line 157
      break;
      case 5: Msp430Adc12ImplP__Port65__selectModuleFunc();
#line 158
      Msp430Adc12ImplP__Port65__makeInput();
#line 158
      break;
      case 6: Msp430Adc12ImplP__Port66__selectModuleFunc();
#line 159
      Msp430Adc12ImplP__Port66__makeInput();
#line 159
      break;
      case 7: Msp430Adc12ImplP__Port67__selectModuleFunc();
#line 160
      Msp430Adc12ImplP__Port67__makeInput();
#line 160
      break;
    }
}

# 100 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__startConversion(void )
#line 100
{
  HplAdc12P__ADC12CTL0 |= 0x010;
  HplAdc12P__ADC12CTL0 |= 0x001 + 0x002;
}

# 128 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__startConversion(void ){
#line 128
  HplAdc12P__HplAdc12__startConversion();
#line 128
}
#line 128
# 50 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setMode(int mode){
#line 50
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(mode);
#line 50
}
#line 50
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4303 {
#line 57
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 100
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t x)
{
  * (volatile uint16_t * )356U = /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__CC2int(x);
}

# 46 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP__ControlA1__setControl(msp430_compare_control_t control){
#line 46
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__setControl(control);
#line 46
}
#line 46
# 127 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__startTimerA(void )
{

  msp430_compare_control_t ccSetSHI = { 
  .ccifg = 0, .cov = 0, .out = 1, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };
  msp430_compare_control_t ccRSOutmod = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 7, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP__ControlA1__setControl(ccResetSHI);
  Msp430Adc12ImplP__ControlA1__setControl(ccSetSHI);

  Msp430Adc12ImplP__ControlA1__setControl(ccRSOutmod);
  Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_UP_MODE);
}

# 130 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 57
}
#line 57
# 95 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 155 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 41
}
#line 41
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
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
# 165 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 43 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
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
# 81 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 103
  /*WireAdcStreamP.Alarm.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 192 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4304 {
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

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4305 {
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

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4306 {
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

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4307 {
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

# 131 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/opt/tinyos/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 48 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 40
}
#line 40
# 48 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 40
}
#line 40
# 48 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 40
}
#line 40
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/opt/tinyos/tos/system/LedsP.nc"
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

# 62 "/opt/tinyos/tos/interfaces/Init.nc"
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
# 36 "/opt/tinyos/tos/platforms/telosb/hardware.h"
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

# 11 "/opt/tinyos/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
  __nop();
#line 12
  __nop();
}

# 89 "/opt/tinyos/tos/platforms/telosb/hardware.h"
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

# 27 "/opt/tinyos/tos/platforms/telosb/MotePlatformC.nc"
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

# 62 "/opt/tinyos/tos/interfaces/Init.nc"
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
# 163 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 151
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 115
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 145
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 100 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0200 | 0x0002;
}

#line 140
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 79 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~0x02;
}

#line 135
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 181 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
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

#line 67
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TACTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 130
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 40 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 40
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 40
}
#line 40
# 229 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
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

# 62 "/opt/tinyos/tos/interfaces/Init.nc"
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
# 10 "/opt/tinyos/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/opt/tinyos/tos/interfaces/Init.nc"
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
# 36 "/opt/tinyos/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 65 "/opt/tinyos/tos/interfaces/Scheduler.nc"
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
# 79 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
}

# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(uint8_t arg_0x40b4fbb0, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__val_t * buf, uint16_t count){
#line 89
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__bufferDone(arg_0x40b4fbb0, result, buf, count);
#line 89
}
#line 89
# 48 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(uint8_t client, error_t result, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__val_t *buf, uint16_t count)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__bufferDone(client, result, buf, count);
}

# 89 "/opt/tinyos/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP__ReadStream__bufferDone(uint8_t arg_0x40afaf28, error_t result, AdcStreamP__ReadStream__val_t * buf, uint16_t count){
#line 89
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__bufferDone(arg_0x40afaf28, result, buf, count);
#line 89
}
#line 89
# 156 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__bufferDone__runTask(void )
#line 156
{
  uint16_t *b;
#line 157
  uint16_t c;

#line 158
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      b = AdcStreamP__lastBuffer;
      c = AdcStreamP__lastCount;
      AdcStreamP__lastBuffer = (void *)0;
    }
#line 163
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__ReadStream__bufferDone(AdcStreamP__client, SUCCESS, b, c);
}

# 83 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
}

# 102 "/opt/tinyos/tos/interfaces/ReadStream.nc"
inline static void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(uint8_t arg_0x40b4fbb0, error_t result, uint32_t usActualPeriod){
#line 102
    /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__default__readDone(arg_0x40b4fbb0, result, usActualPeriod);
#line 102
}
#line 102
# 67 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
static inline error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(uint8_t client)
#line 67
{
#line 67
  return FAIL;
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(uint8_t arg_0x40b4a030){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x40b4a030) {
#line 120
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 120
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__default__release(arg_0x40b4a030);
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
# 53 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(uint8_t client, error_t result, uint32_t actualPeriod)
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__release(client);
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__ReadStream__readDone(client, result, actualPeriod);
}

# 102 "/opt/tinyos/tos/interfaces/ReadStream.nc"
inline static void AdcStreamP__ReadStream__readDone(uint8_t arg_0x40afaf28, error_t result, uint32_t usActualPeriod){
#line 102
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__readDone(arg_0x40afaf28, result, usActualPeriod);
#line 102
}
#line 102
# 135 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__readStreamFail__runTask(void )
#line 135
{

  struct AdcStreamP__list_entry_t *entry;
  uint8_t c = AdcStreamP__client;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 140
    entry = AdcStreamP__bufferQueue[c];
#line 140
    __nesc_atomic_end(__nesc_atomic); }
  for (; entry; entry = entry->next) {
      uint16_t tmp_count __attribute((unused))  = entry->count;

#line 143
      AdcStreamP__ReadStream__bufferDone(c, FAIL, (uint16_t * )entry, entry->count);
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP__bufferQueue[c] = (void *)0;
      AdcStreamP__bufferQueueEnd[c] = &AdcStreamP__bufferQueue[c];
    }
#line 150
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__client = AdcStreamP__NSTREAM;
  AdcStreamP__ReadStream__readDone(c, FAIL, 0);
}

# 180 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__release(uint8_t client)
#line 180
{
#line 180
  return FAIL;
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP__AdcResource__release(uint8_t arg_0x40aa9010){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x40aa9010) {
#line 120
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 120
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 120
      break;
#line 120
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 120
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 120
      break;
#line 120
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 120
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = Msp430RefVoltArbiterImplP__AdcResource__default__release(arg_0x40aa9010);
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
# 66 "/opt/tinyos/tos/system/RoundRobinResourceQueueC.nc"
static inline bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty(void )
#line 66
{
  int i;

  /* atomic removed: atomic calls only */
#line 68
  {
    for (i = 0; i < sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ; i++) 
      if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[i] > 0) {
          unsigned char __nesc_temp = 
#line 70
          FALSE;

#line 70
          return __nesc_temp;
        }
#line 71
    {
      unsigned char __nesc_temp = 
#line 71
      TRUE;

#line 71
      return __nesc_temp;
    }
  }
}

# 53 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 57 "/opt/tinyos/tos/system/RoundRobinResourceQueueC.nc"
static inline void /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(uint8_t id)
#line 57
{
  /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] &= ~(1 << id % 8);
}

#line 79
static inline resource_client_id_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue(void )
#line 79
{
  int i;

  /* atomic removed: atomic calls only */
#line 81
  {
    for (i = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last + 1; ; i++) {
        if (i == 3U) {
          i = 0;
          }
#line 85
        if (/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(i)) {
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__clearEntry(i);
            /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last = i;
            {
              unsigned char __nesc_temp = 
#line 88
              i;

#line 88
              return __nesc_temp;
            }
          }
#line 90
        if (i == /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__last) {
          break;
          }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__NO_ENTRY;

#line 93
      return __nesc_temp;
    }
  }
}

# 70 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 173 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 173
{
}

# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x40a5b5c0){
#line 65
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x40a5b5c0);
#line 65
}
#line 65
# 119 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__readStreamDone__runTask(void )
#line 119
{
  uint8_t c = AdcStreamP__client;
  uint32_t actualPeriod = AdcStreamP__period;

#line 122
  if (AdcStreamP__periodModified) {
    actualPeriod = AdcStreamP__period - AdcStreamP__period % 1000;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      AdcStreamP__bufferQueue[c] = (void *)0;
      AdcStreamP__bufferQueueEnd[c] = &AdcStreamP__bufferQueue[c];
    }
#line 129
    __nesc_atomic_end(__nesc_atomic); }

  AdcStreamP__client = AdcStreamP__NSTREAM;
  AdcStreamP__ReadStream__readDone(c, SUCCESS, actualPeriod);
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t Msp430RefVoltArbiterImplP__switchOff__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(Msp430RefVoltArbiterImplP__switchOff);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 87 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__stop(void )
#line 87
{
  return Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING);
}

# 130 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop(void ){
#line 130
  unsigned char __nesc_result;
#line 130

#line 130
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_1_5V__stop();
#line 130

#line 130
  return __nesc_result;
#line 130
}
#line 130
# 91 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__stop(void )
#line 91
{
  return Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING);
}

# 130 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__stop(void ){
#line 130
  unsigned char __nesc_result;
#line 130

#line 130
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_2_5V__stop();
#line 130

#line 130
  return __nesc_result;
#line 130
}
#line 130
# 141 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__switchOff__runTask(void )
{
  error_t stopped;

  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {
      if (Msp430RefVoltArbiterImplP__ref2_5v) {
        stopped = Msp430RefVoltArbiterImplP__RefVolt_2_5V__stop();
        }
      else {
#line 149
        stopped = Msp430RefVoltArbiterImplP__RefVolt_1_5V__stop();
        }
#line 150
      if (stopped == SUCCESS) {
        Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
        }
      else {
#line 153
        Msp430RefVoltArbiterImplP__switchOff__postTask();
        }
    }
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
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
# 118 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline bool HplAdc12P__HplAdc12__isBusy(void )
#line 118
{
#line 118
  return HplAdc12P__ADC12CTL1 & 0x0001;
}

# 118 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static bool Msp430RefVoltGeneratorP__HplAdc12__isBusy(void ){
#line 118
  unsigned char __nesc_result;
#line 118

#line 118
  __nesc_result = HplAdc12P__HplAdc12__isBusy();
#line 118

#line 118
  return __nesc_result;
#line 118
}
#line 118
# 57 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  adc12ctl0_t HplAdc12P__int2adc12ctl0(uint16_t x)
#line 57
{
#line 57
  union __nesc_unnamed4308 {
#line 57
    uint16_t f;
#line 57
    adc12ctl0_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

#line 72
static inline adc12ctl0_t HplAdc12P__HplAdc12__getCtl0(void )
#line 72
{
  return HplAdc12P__int2adc12ctl0(HplAdc12P__ADC12CTL0);
}

# 63 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12ctl0_t Msp430RefVoltGeneratorP__HplAdc12__getCtl0(void ){
#line 63
  struct __nesc_unnamed4264 __nesc_result;
#line 63

#line 63
  __nesc_result = HplAdc12P__HplAdc12__getCtl0();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 59 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint16_t HplAdc12P__adc12ctl0cast2int(adc12ctl0_t x)
#line 59
{
#line 59
  union __nesc_unnamed4309 {
#line 59
    adc12ctl0_t f;
#line 59
    uint16_t t;
  } 
#line 59
  c = { .f = x };

#line 59
  return c.t;
}



static inline void HplAdc12P__HplAdc12__setCtl0(adc12ctl0_t control0)
#line 64
{
  HplAdc12P__ADC12CTL0 = HplAdc12P__adc12ctl0cast2int(control0);
}

# 51 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430RefVoltGeneratorP__HplAdc12__setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P__HplAdc12__setCtl0(control0);
#line 51
}
#line 51
# 164 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 78 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOnTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 78
}
#line 78
# 50 "/opt/tinyos/tos/chips/msp430/sensors/Msp430InternalVoltageP.nc"
static inline const msp430adc12_channel_config_t *Msp430InternalVoltageP__AdcConfigure__getConfiguration(void )
{
  return &Msp430InternalVoltageP__config;
}

# 186 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline const msp430adc12_channel_config_t *
AdcP__Config__default__getConfiguration(uint8_t client)
{
  return &AdcP__defaultConfig;
}

# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
inline static AdcP__Config__adc_config_t AdcP__Config__getConfiguration(uint8_t arg_0x4094cc90){
#line 58
  struct __nesc_unnamed4262 const *__nesc_result;
#line 58

#line 58
  switch (arg_0x4094cc90) {
#line 58
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 58
      __nesc_result = Msp430InternalVoltageP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT:
#line 58
      __nesc_result = Msp430InternalVoltageP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = AdcP__Config__default__getConfiguration(arg_0x4094cc90);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 191 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__SingleChannel__default__configureSingle(uint8_t client, 
const msp430adc12_channel_config_t *config)
#line 192
{
#line 192
  return FAIL;
}

# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP__SingleChannel__configureSingle(uint8_t arg_0x4095fe40, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  switch (arg_0x4095fe40) {
#line 84
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID, config);
#line 84
      break;
#line 84
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      __nesc_result = AdcP__SingleChannel__default__configureSingle(arg_0x4095fe40, config);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 177 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__ResourceReadNow__default__granted(uint8_t nowClient)
#line 177
{
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void AdcP__ResourceReadNow__granted(uint8_t arg_0x4094f930){
#line 102
    AdcP__ResourceReadNow__default__granted(arg_0x4094f930);
#line 102
}
#line 102
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get(void ){
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
# 86 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get();
}

# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static AdcStreamP__Alarm__size_type AdcStreamP__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 328 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__configureSingle(uint8_t c, 
const msp430adc12_channel_config_t *config)
#line 329
{
#line 329
  return FAIL;
}

# 84 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__configureSingle(uint8_t arg_0x40af5a18, const msp430adc12_channel_config_t * config){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  switch (arg_0x40af5a18) {
#line 84
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 84
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureSingle(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID, config);
#line 84
      break;
#line 84
    default:
#line 84
      __nesc_result = AdcStreamP__SingleChannel__default__configureSingle(arg_0x40af5a18, config);
#line 84
      break;
#line 84
    }
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 314 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline const msp430adc12_channel_config_t *AdcStreamP__AdcConfigure__default__getConfiguration(uint8_t c)
{
  return &AdcStreamP__defaultConfig;
}

# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
inline static AdcStreamP__AdcConfigure__adc_config_t AdcStreamP__AdcConfigure__getConfiguration(uint8_t arg_0x40af4878){
#line 58
  struct __nesc_unnamed4262 const *__nesc_result;
#line 58

#line 58
  switch (arg_0x40af4878) {
#line 58
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 58
      __nesc_result = Msp430InternalVoltageP__AdcConfigure__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = AdcStreamP__AdcConfigure__default__getConfiguration(arg_0x40af4878);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__readStreamDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__readStreamDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 147 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type t0, /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type dt)
{
  /* atomic removed: atomic calls only */
  {
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 = t0;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt = dt;
    /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm();
  }
}

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void AdcStreamP__Alarm__startAt(AdcStreamP__Alarm__size_type t0, AdcStreamP__Alarm__size_type dt){
#line 103
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 168 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline void AdcStreamP__nextAlarm(void )
#line 168
{
  AdcStreamP__Alarm__startAt(AdcStreamP__now, AdcStreamP__period);
  AdcStreamP__now += AdcStreamP__period;
}

# 155 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )372U = x;
}

# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP__CompareA1__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__setEvent(time);
#line 41
}
#line 41
# 155 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )370U = x;
}

# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void Msp430Adc12ImplP__CompareA0__setEvent(uint16_t time){
#line 41
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__setEvent(time);
#line 41
}
#line 41
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4310 {
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

#line 100
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t x)
{
  * (volatile uint16_t * )354U = /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__CC2int(x);
}

# 46 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void Msp430Adc12ImplP__ControlA0__setControl(msp430_compare_control_t control){
#line 46
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__setControl(control);
#line 46
}
#line 46
# 121 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(uint16_t inputDivider)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0040 | 0x0080)) | ((inputDivider << 6) & (0x0040 | 0x0080));
}

# 56 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setInputDivider(uint16_t inputDivider){
#line 56
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setInputDivider(inputDivider);
#line 56
}
#line 56
# 116 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(uint16_t clockSource)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(256U | 512U)) | ((clockSource << 8) & (256U | 512U));
}

# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__setClockSource(uint16_t clockSource){
#line 55
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setClockSource(clockSource);
#line 55
}
#line 55
# 111 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents(void )
{
  * (volatile uint16_t * )352U &= ~2U;
}

# 54 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__disableEvents(void ){
#line 54
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__disableEvents();
#line 54
}
#line 54
# 101 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear(void )
{
  * (volatile uint16_t * )352U |= 4U;
}

# 52 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void Msp430Adc12ImplP__TimerA__clear(void ){
#line 52
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__clear();
#line 52
}
#line 52
# 109 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__prepareTimerA(uint16_t interval, uint16_t csSAMPCON, uint16_t cdSAMPCON)
{

  msp430_compare_control_t ccResetSHI = { 
  .ccifg = 0, .cov = 0, .out = 0, .cci = 0, .ccie = 0, 
  .outmod = 0, .cap = 0, .clld = 0, .scs = 0, .ccis = 0, .cm = 0 };

  Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_STOP_MODE);
  Msp430Adc12ImplP__TimerA__clear();
  Msp430Adc12ImplP__TimerA__disableEvents();
  Msp430Adc12ImplP__TimerA__setClockSource(csSAMPCON);
  Msp430Adc12ImplP__TimerA__setInputDivider(cdSAMPCON);
  Msp430Adc12ImplP__ControlA0__setControl(ccResetSHI);
  Msp430Adc12ImplP__CompareA0__setEvent(interval - 1);
  Msp430Adc12ImplP__CompareA1__setEvent((interval - 1) / 2);
}

# 92 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__setIEFlags(uint16_t mask)
#line 92
{
#line 92
  HplAdc12P__ADC12IE = mask;
}

# 95 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setIEFlags(uint16_t mask){
#line 95
  HplAdc12P__HplAdc12__setIEFlags(mask);
#line 95
}
#line 95
# 61 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint8_t HplAdc12P__adc12memctl2int(adc12memctl_t x)
#line 61
{
#line 61
  union __nesc_unnamed4311 {
#line 61
    adc12memctl_t f;
#line 61
    uint8_t t;
  } 
#line 61
  c = { .f = x };

#line 61
  return c.t;
}

#line 80
static inline void HplAdc12P__HplAdc12__setMCtl(uint8_t i, adc12memctl_t memCtl)
#line 80
{
  ((volatile char *)0x0080)[i] = HplAdc12P__adc12memctl2int(memCtl);
}

# 75 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setMCtl(uint8_t idx, adc12memctl_t memControl){
#line 75
  HplAdc12P__HplAdc12__setMCtl(idx, memControl);
#line 75
}
#line 75
# 60 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  uint16_t HplAdc12P__adc12ctl1cast2int(adc12ctl1_t x)
#line 60
{
#line 60
  union __nesc_unnamed4312 {
#line 60
    adc12ctl1_t f;
#line 60
    uint16_t t;
  } 
#line 60
  c = { .f = x };

#line 60
  return c.t;
}






static inline void HplAdc12P__HplAdc12__setCtl1(adc12ctl1_t control1)
#line 68
{
  HplAdc12P__ADC12CTL1 = HplAdc12P__adc12ctl1cast2int(control1);
}

# 57 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__setCtl1(adc12ctl1_t control1){
#line 57
  HplAdc12P__HplAdc12__setCtl1(control1);
#line 57
}
#line 57
#line 51
inline static void Msp430Adc12ImplP__HplAdc12__setCtl0(adc12ctl0_t control0){
#line 51
  HplAdc12P__HplAdc12__setCtl0(control0);
#line 51
}
#line 51
#line 63
inline static adc12ctl0_t Msp430Adc12ImplP__HplAdc12__getCtl0(void ){
#line 63
  struct __nesc_unnamed4264 __nesc_result;
#line 63

#line 63
  __nesc_result = HplAdc12P__HplAdc12__getCtl0();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 98 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t Msp430Adc12ImplP__ADCArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 277 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__SingleChannel__configureMultiple(uint8_t id, 
const msp430adc12_channel_config_t *config, 
uint16_t *buf, uint16_t length, uint16_t jiffies)
{
  error_t result = ERESERVE;





  if ((((!config || !buf) || !length) || jiffies == 1) || jiffies == 2) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 290
    {
      if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 292
          EBUSY;

          {
#line 292
            __nesc_atomic_end(__nesc_atomic); 
#line 292
            return __nesc_temp;
          }
        }
#line 293
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = length > 16 ? 3 : 1, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = jiffies == 0 ? 0 : 1, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 0 };

          uint16_t i;
#line 309
          uint16_t mask = 1;
          adc12ctl0_t ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();

#line 311
          ctl0.msc = jiffies == 0 ? 1 : 0;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP__state = Msp430Adc12ImplP__MULTIPLE_DATA;
          Msp430Adc12ImplP__resultBufferStart = (void *)0;
          Msp430Adc12ImplP__resultBufferLength = length;
          Msp430Adc12ImplP__resultBufferStart = buf;
          Msp430Adc12ImplP__resultBufferIndex = 0;
          Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
          Msp430Adc12ImplP__HplAdc12__setCtl1(ctl1);
          for (i = 0; i < length - 1 && i < 15; i++) 
            Msp430Adc12ImplP__HplAdc12__setMCtl(i, memctl);
          memctl.eos = 1;
          Msp430Adc12ImplP__HplAdc12__setMCtl(i, memctl);
          Msp430Adc12ImplP__HplAdc12__setIEFlags(mask << i);

          if (jiffies) {
              Msp430Adc12ImplP__state |= Msp430Adc12ImplP__USE_TIMERA;
              Msp430Adc12ImplP__prepareTimerA(jiffies, config->sampcon_ssel, config->sampcon_id);
            }
          result = SUCCESS;
        }
    }
#line 334
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 318 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__SingleChannel__default__configureMultiple(uint8_t c, 
const msp430adc12_channel_config_t *config, uint16_t b[], 
uint16_t numSamples, uint16_t jiffies)
{
  return FAIL;
}

# 138 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcStreamP__SingleChannel__configureMultiple(uint8_t arg_0x40af5a18, const msp430adc12_channel_config_t * config, uint16_t * buffer, uint16_t numSamples, uint16_t jiffies){
#line 138
  unsigned char __nesc_result;
#line 138

#line 138
  switch (arg_0x40af5a18) {
#line 138
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT:
#line 138
      __nesc_result = Msp430Adc12ImplP__SingleChannel__configureMultiple(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    default:
#line 138
      __nesc_result = AdcStreamP__SingleChannel__default__configureMultiple(arg_0x40af5a18, config, buffer, numSamples, jiffies);
#line 138
      break;
#line 138
    }
#line 138

#line 138
  return __nesc_result;
#line 138
}
#line 138
# 96 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__postBuffer(uint8_t c, uint16_t *buf, uint16_t n)
{
  if (n < sizeof(struct AdcStreamP__list_entry_t )) {
    return ESIZE;
    }
#line 100
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP__list_entry_t * newEntry = (struct AdcStreamP__list_entry_t * )buf;

      if (!AdcStreamP__bufferQueueEnd[c]) 
        {
          unsigned char __nesc_temp = 
#line 105
          FAIL;

          {
#line 105
            __nesc_atomic_end(__nesc_atomic); 
#line 105
            return __nesc_temp;
          }
        }
#line 107
      newEntry->count = n;
      newEntry->next = (void *)0;
      *AdcStreamP__bufferQueueEnd[c] = newEntry;
      AdcStreamP__bufferQueueEnd[c] = & newEntry->next;
    }
#line 111
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__readStreamFail__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__readStreamFail);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 180 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__SingleChannel__default__getData(uint8_t client)
{
  return EINVAL;
}

# 189 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t AdcP__SingleChannel__getData(uint8_t arg_0x4095fe40){
#line 189
  unsigned char __nesc_result;
#line 189

#line 189
  switch (arg_0x4095fe40) {
#line 189
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 189
      break;
#line 189
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT:
#line 189
      __nesc_result = Msp430Adc12ImplP__SingleChannel__getData(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 189
      break;
#line 189
    default:
#line 189
      __nesc_result = AdcP__SingleChannel__default__getData(arg_0x4095fe40);
#line 189
      break;
#line 189
    }
#line 189

#line 189
  return __nesc_result;
#line 189
}
#line 189
# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 6);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__clr(void ){
#line 41
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr();
#line 41
}
#line 41
# 104 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2On(void )
#line 104
{
  LedsP__Led2__clr();
  ;
#line 106
  ;
}

# 89 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void SenseC__Leds__led2On(void ){
#line 89
  LedsP__Leds__led2On();
#line 89
}
#line 89
# 109 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 109
{
  LedsP__Led2__set();
  ;
#line 111
  ;
}

# 94 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void SenseC__Leds__led2Off(void ){
#line 94
  LedsP__Leds__led2Off();
#line 94
}
#line 94
# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 5);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__clr(void ){
#line 41
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr();
#line 41
}
#line 41
# 89 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1On(void )
#line 89
{
  LedsP__Led1__clr();
  ;
#line 91
  ;
}

# 72 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void SenseC__Leds__led1On(void ){
#line 72
  LedsP__Leds__led1On();
#line 72
}
#line 72
# 94 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Off(void )
#line 94
{
  LedsP__Led1__set();
  ;
#line 96
  ;
}

# 77 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void SenseC__Leds__led1Off(void ){
#line 77
  LedsP__Leds__led1Off();
#line 77
}
#line 77
# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )49U &= ~(0x01 << 4);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void )
#line 49
{
#line 49
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__clr(void ){
#line 41
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr();
#line 41
}
#line 41
# 74 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0On(void )
#line 74
{
  LedsP__Led0__clr();
  ;
#line 76
  ;
}

# 56 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void SenseC__Leds__led0On(void ){
#line 56
  LedsP__Leds__led0On();
#line 56
}
#line 56
# 79 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Off(void )
#line 79
{
  LedsP__Led0__set();
  ;
#line 81
  ;
}

# 61 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void SenseC__Leds__led0Off(void ){
#line 61
  LedsP__Leds__led0Off();
#line 61
}
#line 61
# 165 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(uint8_t id)
#line 165
{
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(uint8_t arg_0x40a4be98){
#line 102
  switch (arg_0x40a4be98) {
#line 102
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 102
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 102
      break;
#line 102
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 102
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 102
      break;
#line 102
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 102
      Msp430RefVoltArbiterImplP__AdcResource__granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__default__granted(arg_0x40a4be98);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 171 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 171
{
}

# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40a5b5c0){
#line 59
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__default__configure(arg_0x40a5b5c0);
#line 59
}
#line 59
# 155 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId;
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__configure(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__granted(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
}

# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
inline static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4262 const *__nesc_result;
#line 58

#line 58
  __nesc_result = Msp430InternalVoltageP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration(void )
{
  return /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfUp__getConfiguration();
}

# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
inline static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4262 const *__nesc_result;
#line 58

#line 58
  __nesc_result = Msp430InternalVoltageP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration(void )
{
  return /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfUp__getConfiguration();
}

# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
inline static /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__adc_config_t /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__getConfiguration(void ){
#line 58
  struct __nesc_unnamed4262 const *__nesc_result;
#line 58

#line 58
  __nesc_result = Msp430InternalVoltageP__AdcConfigure__getConfiguration();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 47 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ConfAlertC.nc"
static inline const msp430adc12_channel_config_t */*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration(void )
{
  return /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfUp__getConfiguration();
}

# 182 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline const msp430adc12_channel_config_t *
Msp430RefVoltArbiterImplP__Config__default__getConfiguration(uint8_t client)
{
  return &Msp430RefVoltArbiterImplP__defaultConfig;
}

# 58 "/opt/tinyos/tos/interfaces/AdcConfigure.nc"
inline static Msp430RefVoltArbiterImplP__Config__adc_config_t Msp430RefVoltArbiterImplP__Config__getConfiguration(uint8_t arg_0x40aa8ab0){
#line 58
  struct __nesc_unnamed4262 const *__nesc_result;
#line 58

#line 58
  switch (arg_0x40aa8ab0) {
#line 58
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 58
      __nesc_result = /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__0__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 58
      __nesc_result = /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__1__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 58
      __nesc_result = /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient.Msp430Adc12ConfAlertC*/Msp430Adc12ConfAlertC__2__ConfSub__getConfiguration();
#line 58
      break;
#line 58
    default:
#line 58
      __nesc_result = Msp430RefVoltArbiterImplP__Config__default__getConfiguration(arg_0x40aa8ab0);
#line 58
      break;
#line 58
    }
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 167 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static inline void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 167
{
}

# 53 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(uint8_t arg_0x40a5d948){
#line 53
    /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__default__requested(arg_0x40a5d948);
#line 53
}
#line 53
# 97 "/opt/tinyos/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(resource_client_id_t id)
#line 97
{
  /* atomic removed: atomic calls only */
#line 98
  {
    if (!/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(id)) {
        /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] |= 1 << id % 8;
        {
          unsigned char __nesc_temp = 
#line 101
          SUCCESS;

#line 101
          return __nesc_temp;
        }
      }
#line 103
    {
      unsigned char __nesc_temp = 
#line 103
      EBUSY;

#line 103
      return __nesc_temp;
    }
  }
}

# 79 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 79 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_1_5V__start(void )
#line 79
{
  return Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE);
}

# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_1_5V__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_1_5V__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 169 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static bool Msp430RefVoltGeneratorP__SwitchOffSettleTimer__isRunning(void ){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(3U);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
#line 78
inline static void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(3U);
#line 78
}
#line 78
inline static void Msp430RefVoltGeneratorP__SwitchOffTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 78
}
#line 78
# 83 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline error_t Msp430RefVoltGeneratorP__RefVolt_2_5V__start(void )
#line 83
{
  return Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE);
}

# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t Msp430RefVoltArbiterImplP__RefVolt_2_5V__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = Msp430RefVoltGeneratorP__RefVolt_2_5V__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 172 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__Read__default__readDone(uint8_t client, error_t result, uint16_t val)
#line 172
{
}

# 63 "/opt/tinyos/tos/interfaces/Read.nc"
inline static void AdcP__Read__readDone(uint8_t arg_0x40934950, error_t result, AdcP__Read__val_t val){
#line 63
  switch (arg_0x40934950) {
#line 63
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 63
      SenseC__Read__readDone(result, val);
#line 63
      break;
#line 63
    default:
#line 63
      AdcP__Read__default__readDone(arg_0x40934950, result, val);
#line 63
      break;
#line 63
    }
#line 63
}
#line 63
# 170 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__ResourceRead__default__release(uint8_t client)
#line 170
{
#line 170
  return FAIL;
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t AdcP__ResourceRead__release(uint8_t arg_0x4094e4b0){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x4094e4b0) {
#line 120
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 120
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__release(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = AdcP__ResourceRead__default__release(arg_0x4094e4b0);
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
# 136 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__readDone__runTask(void )
{
  AdcP__ResourceRead__release(AdcP__owner);
  AdcP__Read__readDone(AdcP__owner, SUCCESS, AdcP__value);
}

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 58 "/opt/tinyos/tos/lib/timer/AlarmToTimerC.nc"
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

# 129 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 129
}
#line 129
# 65 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 71 "/opt/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 78 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 78
}
#line 78
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
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
# 86 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
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
# 96 "/opt/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 136 "/opt/tinyos/tos/lib/timer/Timer.nc"
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
# 100 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
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

# 171 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__AdcResource__default__request(uint8_t client)
{
  return FAIL;
}

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t Msp430RefVoltArbiterImplP__AdcResource__request(uint8_t arg_0x40aa9010){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x40aa9010) {
#line 88
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 88
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 88
      break;
#line 88
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 88
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID);
#line 88
      break;
#line 88
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 88
      __nesc_result = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = Msp430RefVoltArbiterImplP__AdcResource__default__request(arg_0x40aa9010);
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
# 54 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline error_t Msp430RefVoltArbiterImplP__ClientResource__request(uint8_t client)
{
  return Msp430RefVoltArbiterImplP__AdcResource__request(client);
}

# 168 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__ResourceRead__default__request(uint8_t client)
#line 168
{
#line 168
  return FAIL;
}

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t AdcP__ResourceRead__request(uint8_t arg_0x4094e4b0){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x4094e4b0) {
#line 88
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT:
#line 88
      __nesc_result = Msp430RefVoltArbiterImplP__ClientResource__request(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = AdcP__ResourceRead__default__request(arg_0x4094e4b0);
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
# 75 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline error_t AdcP__Read__read(uint8_t client)
{
  return AdcP__ResourceRead__request(client);
}

# 55 "/opt/tinyos/tos/interfaces/Read.nc"
inline static error_t SenseC__Read__read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = AdcP__Read__read(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 65 "SenseC.nc"
static inline void SenseC__Timer__fired(void )
{
  SenseC__Read__read();
}

# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(error_t error){
#line 113
  Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error);
#line 113
}
#line 113
# 78 "/opt/tinyos/tos/interfaces/ReadStream.nc"
inline static error_t /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(uint8_t arg_0x40b4d6c8, uint32_t usPeriod){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = AdcStreamP__ReadStream__read(arg_0x40b4d6c8, usPeriod);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 59 "/opt/tinyos/tos/system/ArbitratedReadStreamC.nc"
static inline void /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(uint8_t client)
#line 59
{
  /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Service__read(client, /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__period[client]);
}

# 170 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__ClientResource__default__granted(uint8_t client)
#line 170
{
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void Msp430RefVoltArbiterImplP__ClientResource__granted(uint8_t arg_0x40aaa4c0){
#line 102
  switch (arg_0x40aaa4c0) {
#line 102
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 102
      AdcP__ResourceRead__granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT);
#line 102
      break;
#line 102
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 102
      /*WireAdcStreamP.ArbitrateReadStream*/ArbitratedReadStreamC__0__Resource__granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT);
#line 102
      break;
#line 102
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 102
      AdcP__SubResourceReadNow__granted(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT);
#line 102
      break;
#line 102
    default:
#line 102
      Msp430RefVoltArbiterImplP__ClientResource__default__granted(arg_0x40aaa4c0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 103 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {


      Msp430RefVoltArbiterImplP__ClientResource__granted(Msp430RefVoltArbiterImplP__syncOwner);
    }
}

# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(error_t error){
#line 113
  Msp430RefVoltArbiterImplP__RefVolt_1_5V__startDone(error);
#line 113
}
#line 113
# 188 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__SwitchOnTimer__fired(void )
#line 188
{
  switch (Msp430RefVoltGeneratorP__m_state) {
      case Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING: 
        Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE;
      Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(SUCCESS);
      break;

      case Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING: 
        Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE;
      Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(SUCCESS);
      break;

      default: 
        return;
    }
}

# 159 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(2U, dt);
#line 73
}
#line 73
inline static void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 73
}
#line 73
# 161 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error_t error)
{
}

# 138 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(error_t error){
#line 138
  Msp430RefVoltArbiterImplP__RefVolt_2_5V__stopDone(error);
#line 138
}
#line 138
# 157 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static inline void Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error_t error)
{
}

# 138 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(error_t error){
#line 138
  Msp430RefVoltArbiterImplP__RefVolt_1_5V__stopDone(error);
#line 138
}
#line 138
# 205 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__SwitchOffTimer__fired(void )
#line 205
{
  switch (Msp430RefVoltGeneratorP__m_state) {
      case Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING: 
        if (Msp430RefVoltGeneratorP__switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
            Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(SUCCESS);
          }
        else {
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      break;

      case Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING: 
        if (Msp430RefVoltGeneratorP__switchOff() == SUCCESS) {
            Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
            Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(SUCCESS);

            Msp430RefVoltGeneratorP__SwitchOffSettleTimer__startOneShot(2048);
          }
        else 
#line 223
          {
            Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
          }
      break;

      default: 
        break;
    }
}

static inline void Msp430RefVoltGeneratorP__SwitchOffSettleTimer__fired(void )
#line 233
{
}

# 204 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x408dd030){
#line 83
  switch (arg_0x408dd030) {
#line 83
    case 0U:
#line 83
      SenseC__Timer__fired();
#line 83
      break;
#line 83
    case 1U:
#line 83
      Msp430RefVoltGeneratorP__SwitchOnTimer__fired();
#line 83
      break;
#line 83
    case 2U:
#line 83
      Msp430RefVoltGeneratorP__SwitchOffTimer__fired();
#line 83
      break;
#line 83
    case 3U:
#line 83
      Msp430RefVoltGeneratorP__SwitchOffSettleTimer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x408dd030);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 139 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 83
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 83
}
#line 83
# 91 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
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

# 116 "/opt/tinyos/tos/lib/timer/Alarm.nc"
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
# 74 "/opt/tinyos/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 69 "/opt/tinyos/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4313 {
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

# 47 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 95 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline void HplAdc12P__HplAdc12__resetIFGs(void )
#line 95
{
  HplAdc12P__ADC12IV = 0;
  HplAdc12P__ADC12IFG = 0;
}

# 106 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void Msp430Adc12ImplP__HplAdc12__resetIFGs(void ){
#line 106
  HplAdc12P__HplAdc12__resetIFGs();
#line 106
}
#line 106
#line 123
inline static void Msp430Adc12ImplP__HplAdc12__stopConversion(void ){
#line 123
  HplAdc12P__HplAdc12__stopConversion();
#line 123
}
#line 123
# 92 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__Init__init(void )
{
  adc12ctl0_t ctl0;

  /* atomic removed: atomic calls only */
#line 96
  {

    Msp430Adc12ImplP__HplAdc12__stopConversion();

    Msp430Adc12ImplP__HplAdc12__resetIFGs();
    ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();
    ctl0.adc12tovie = 1;
    ctl0.adc12ovie = 1;
    Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
  }
  return SUCCESS;
}

# 61 "/opt/tinyos/tos/system/RoundRobinResourceQueueC.nc"
static inline error_t /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init(void )
#line 61
{
  memset(/*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ, 0, sizeof /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ);
  return SUCCESS;
}

# 83 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static inline error_t AdcStreamP__Init__init(void )
#line 83
{
  uint8_t i;

  for (i = 0; i != AdcStreamP__NSTREAM; i++) 
    AdcStreamP__bufferQueueEnd[i] = &AdcStreamP__bufferQueue[i];

  return SUCCESS;
}

# 62 "/opt/tinyos/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = AdcStreamP__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, Msp430Adc12ImplP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 154 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void SenseC__Timer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 64
}
#line 64
# 61 "SenseC.nc"
static inline void SenseC__Boot__booted(void )
#line 61
{
  SenseC__Timer__startPeriodic(500);
}

# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  SenseC__Boot__booted();
#line 60
}
#line 60
# 391 "/opt/tinyos/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 63 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 63
{
  return MSP430_POWER_LPM3;
}

# 62 "/opt/tinyos/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/opt/tinyos/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM4;



  if (((
#line 77
  TACCTL0 & 0x0010 || 
  TACCTL1 & 0x0010) || 
  TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) {








    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 99
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 100
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 379 "/opt/tinyos/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 112 "/opt/tinyos/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 112
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 117
{
  uint16_t temp;

#line 119
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/opt/tinyos/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
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

# 72 "/opt/tinyos/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 161 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline uint16_t *AdcP__SingleChannel__multipleDataReady(uint8_t client, 
uint16_t *buf, uint16_t numSamples)
{

  return 0;
}

# 654 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline uint16_t *Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(uint8_t id, 
uint16_t *buf, uint16_t numSamples)
{
  return 0;
}

# 227 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static uint16_t * Msp430Adc12ImplP__SingleChannel__multipleDataReady(uint8_t arg_0x409853f0, uint16_t * buffer, uint16_t numSamples){
#line 227
  unsigned int *__nesc_result;
#line 227

#line 227
  switch (arg_0x409853f0) {
#line 227
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 227
      __nesc_result = AdcP__SingleChannel__multipleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 227
      __nesc_result = AdcStreamP__SingleChannel__multipleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 227
      __nesc_result = AdcP__SingleChannel__multipleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT, buffer, numSamples);
#line 227
      break;
#line 227
    default:
#line 227
      __nesc_result = Msp430Adc12ImplP__SingleChannel__default__multipleDataReady(arg_0x409853f0, buffer, numSamples);
#line 227
      break;
#line 227
    }
#line 227

#line 227
  return __nesc_result;
#line 227
}
#line 227
# 88 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline uint16_t HplAdc12P__HplAdc12__getMem(uint8_t i)
#line 88
{
  return ((volatile int *)0x0140)[i];
}

# 89 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static uint16_t Msp430Adc12ImplP__HplAdc12__getMem(uint8_t idx){
#line 89
  unsigned int __nesc_result;
#line 89

#line 89
  __nesc_result = HplAdc12P__HplAdc12__getMem(idx);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 62 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static inline  adc12memctl_t HplAdc12P__int2adc12memctl(uint8_t x)
#line 62
{
#line 62
  union __nesc_unnamed4314 {
#line 62
    uint8_t f;
#line 62
    adc12memctl_t t;
  } 
#line 62
  c = { .f = x };

#line 62
  return c.t;
}

#line 84
static inline adc12memctl_t HplAdc12P__HplAdc12__getMCtl(uint8_t i)
#line 84
{
  return HplAdc12P__int2adc12memctl(((volatile char *)0x0080)[i]);
}

# 82 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static adc12memctl_t Msp430Adc12ImplP__HplAdc12__getMCtl(uint8_t idx){
#line 82
  struct __nesc_unnamed4263 __nesc_result;
#line 82

#line 82
  __nesc_result = HplAdc12P__HplAdc12__getMCtl(idx);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 660 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__MultiChannel__default__dataReady(uint8_t id, uint16_t *buffer, uint16_t numSamples)
#line 660
{
}

# 107 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12MultiChannel.nc"
inline static void Msp430Adc12ImplP__MultiChannel__dataReady(uint8_t arg_0x40984108, uint16_t *buffer, uint16_t numSamples){
#line 107
    Msp430Adc12ImplP__MultiChannel__default__dataReady(arg_0x40984108, buffer, numSamples);
#line 107
}
#line 107
# 649 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline error_t Msp430Adc12ImplP__SingleChannel__default__singleDataReady(uint8_t id, uint16_t data)
{
  return FAIL;
}

# 206 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12SingleChannel.nc"
inline static error_t Msp430Adc12ImplP__SingleChannel__singleDataReady(uint8_t arg_0x409853f0, uint16_t data){
#line 206
  unsigned char __nesc_result;
#line 206

#line 206
  switch (arg_0x409853f0) {
#line 206
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__0__ID:
#line 206
      __nesc_result = AdcP__SingleChannel__singleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadClientC*/AdcReadClientC__0__CLIENT, data);
#line 206
      break;
#line 206
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__1__ID:
#line 206
      __nesc_result = AdcStreamP__SingleChannel__singleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadStreamClientC*/AdcReadStreamClientC__0__RSCLIENT, data);
#line 206
      break;
#line 206
    case /*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC.Msp430AdcClient*/Msp430Adc12ClientAutoRVGC__2__ID:
#line 206
      __nesc_result = AdcP__SingleChannel__singleDataReady(/*SenseAppC.Sensor.DemoSensor.Msp430InternalVoltageC.AdcReadNowClientC*/AdcReadNowClientC__0__CLIENT, data);
#line 206
      break;
#line 206
    default:
#line 206
      __nesc_result = Msp430Adc12ImplP__SingleChannel__default__singleDataReady(arg_0x409853f0, data);
#line 206
      break;
#line 206
    }
#line 206

#line 206
  return __nesc_result;
#line 206
}
#line 206
# 663 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(uint8_t id)
#line 663
{
}

# 54 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP__Overflow__conversionTimeOverflow(uint8_t arg_0x409849f8){
#line 54
    Msp430Adc12ImplP__Overflow__default__conversionTimeOverflow(arg_0x409849f8);
#line 54
}
#line 54
# 662 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__Overflow__default__memOverflow(uint8_t id)
#line 662
{
}

# 49 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12Overflow.nc"
inline static void Msp430Adc12ImplP__Overflow__memOverflow(uint8_t arg_0x409849f8){
#line 49
    Msp430Adc12ImplP__Overflow__default__memOverflow(arg_0x409849f8);
#line 49
}
#line 49
# 550 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static inline void Msp430Adc12ImplP__HplAdc12__conversionDone(uint16_t iv)
{
  bool overflow = FALSE;
  uint16_t *resultBuffer;

  if (iv <= 4) {
      if (iv == 2) {
        Msp430Adc12ImplP__Overflow__memOverflow(Msp430Adc12ImplP__clientID);
        }
      else {
#line 559
        Msp430Adc12ImplP__Overflow__conversionTimeOverflow(Msp430Adc12ImplP__clientID);
        }
      if (! Msp430Adc12ImplP__HplAdc12__getCtl0().msc) {
        overflow = TRUE;
        }
    }
#line 564
  switch (Msp430Adc12ImplP__state & Msp430Adc12ImplP__CONVERSION_MODE_MASK) 
    {
      case Msp430Adc12ImplP__SINGLE_DATA: 
        Msp430Adc12ImplP__stopConversion();
      Msp430Adc12ImplP__SingleChannel__singleDataReady(Msp430Adc12ImplP__clientID, Msp430Adc12ImplP__HplAdc12__getMem(0));
      break;
      case Msp430Adc12ImplP__SINGLE_DATA_REPEAT: 
        {
          error_t repeatContinue;

#line 573
          repeatContinue = Msp430Adc12ImplP__SingleChannel__singleDataReady(Msp430Adc12ImplP__clientID, 
          Msp430Adc12ImplP__HplAdc12__getMem(0));
          if (repeatContinue != SUCCESS) {
            Msp430Adc12ImplP__stopConversion();
            }
#line 577
          break;
        }

      case Msp430Adc12ImplP__MULTI_CHANNEL: 
        {
          uint16_t i = 0;
#line 582
          uint16_t k;

#line 583
          resultBuffer = Msp430Adc12ImplP__resultBufferStart + Msp430Adc12ImplP__resultBufferIndex;
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 586
          ++i < Msp430Adc12ImplP__numChannels);
          Msp430Adc12ImplP__resultBufferIndex += Msp430Adc12ImplP__numChannels;
          if (overflow || Msp430Adc12ImplP__resultBufferLength == Msp430Adc12ImplP__resultBufferIndex) {
              Msp430Adc12ImplP__stopConversion();
              resultBuffer -= Msp430Adc12ImplP__resultBufferIndex;
              k = Msp430Adc12ImplP__resultBufferIndex - Msp430Adc12ImplP__numChannels;
              Msp430Adc12ImplP__resultBufferIndex = 0;
              Msp430Adc12ImplP__MultiChannel__dataReady(Msp430Adc12ImplP__clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP__resultBufferLength);
            }
        }
      break;
      case Msp430Adc12ImplP__MULTIPLE_DATA: 
        {
          uint16_t i = 0;
#line 600
          uint16_t length;
#line 600
          uint16_t k;

#line 601
          resultBuffer = Msp430Adc12ImplP__resultBufferStart + Msp430Adc12ImplP__resultBufferIndex;
          if (Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex > 16) {
            length = 16;
            }
          else {
#line 605
            length = Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex;
            }
#line 606
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 608
          ++i < length);
          Msp430Adc12ImplP__resultBufferIndex += length;
          if (overflow || Msp430Adc12ImplP__resultBufferLength == Msp430Adc12ImplP__resultBufferIndex) {
              Msp430Adc12ImplP__stopConversion();
              resultBuffer -= Msp430Adc12ImplP__resultBufferIndex;
              k = Msp430Adc12ImplP__resultBufferIndex - length;
              Msp430Adc12ImplP__resultBufferIndex = 0;
              Msp430Adc12ImplP__SingleChannel__multipleDataReady(Msp430Adc12ImplP__clientID, resultBuffer, 
              overflow ? k : Msp430Adc12ImplP__resultBufferLength);
            }
          else {
#line 617
            if (Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex > 15) {
              return;
              }
            else 
#line 619
              {

                adc12memctl_t memctl = Msp430Adc12ImplP__HplAdc12__getMCtl(0);

#line 622
                memctl.eos = 1;
                Msp430Adc12ImplP__HplAdc12__setMCtl(Msp430Adc12ImplP__resultBufferLength - Msp430Adc12ImplP__resultBufferIndex, memctl);
              }
            }
        }
#line 626
      break;
      case Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT: 
        {
          uint8_t i = 0;

#line 630
          resultBuffer = Msp430Adc12ImplP__resultBufferStart;
          do {
              * resultBuffer++ = Msp430Adc12ImplP__HplAdc12__getMem(i);
            }
          while (
#line 633
          ++i < Msp430Adc12ImplP__resultBufferLength);

          Msp430Adc12ImplP__resultBufferStart = Msp430Adc12ImplP__SingleChannel__multipleDataReady(Msp430Adc12ImplP__clientID, 
          resultBuffer - Msp430Adc12ImplP__resultBufferLength, 
          overflow ? 0 : Msp430Adc12ImplP__resultBufferLength);
          if (!Msp430Adc12ImplP__resultBufferStart) {
            Msp430Adc12ImplP__stopConversion();
            }
#line 640
          break;
        }

      default: 
        Msp430Adc12ImplP__stopConversion();
      break;
    }
}

# 236 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static inline void Msp430RefVoltGeneratorP__HplAdc12__conversionDone(uint16_t iv)
#line 236
{
}

# 112 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12.nc"
inline static void HplAdc12P__HplAdc12__conversionDone(uint16_t iv){
#line 112
  Msp430RefVoltGeneratorP__HplAdc12__conversionDone(iv);
#line 112
  Msp430Adc12ImplP__HplAdc12__conversionDone(iv);
#line 112
}
#line 112
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 0);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port60__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P60*/HplMsp430GeneralIOP__40__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 1);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port61__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P61*/HplMsp430GeneralIOP__41__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 2);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port62__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P62*/HplMsp430GeneralIOP__42__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 3);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port63__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P63*/HplMsp430GeneralIOP__43__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 4);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port64__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P64*/HplMsp430GeneralIOP__44__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 5);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port65__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P65*/HplMsp430GeneralIOP__45__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 6);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port66__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P66*/HplMsp430GeneralIOP__46__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )55U &= ~(0x01 << 7);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void Msp430Adc12ImplP__Port67__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P67*/HplMsp430GeneralIOP__47__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t AdcP__readDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcP__readDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 178 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static inline void AdcP__ReadNow__default__readDone(uint8_t client, error_t result, uint16_t val)
#line 178
{
}

# 66 "/opt/tinyos/tos/interfaces/ReadNow.nc"
inline static void AdcP__ReadNow__readDone(uint8_t arg_0x40950968, error_t result, AdcP__ReadNow__val_t val){
#line 66
    AdcP__ReadNow__default__readDone(arg_0x40950968, result, val);
#line 66
}
#line 66
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t AdcStreamP__bufferDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(AdcStreamP__bufferDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 411 "/opt/tinyos/tos/chips/msp430/msp430hardware.h"
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

# 11 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x000C)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 180 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
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

#line 180
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

# 12 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
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

# 146 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4064a620){
#line 39
  switch (arg_0x4064a620) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4064a620);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 170 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
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

# 107 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
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

# 80 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
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

# 62 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
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

#line 73
          return __nesc_temp;
        }
      }
    }
  else 
#line 76
    {
      return * (volatile uint16_t * )400U;
    }
}

# 400 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP__SingleChannel__getData(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 402
    {
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__MULTIPLE_DATA_REPEAT && !Msp430Adc12ImplP__resultBufferStart) 
            {
              unsigned char __nesc_temp = 
#line 405
              EINVAL;

              {
#line 405
                __nesc_atomic_end(__nesc_atomic); 
#line 405
                return __nesc_temp;
              }
            }
#line 406
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
            {
              unsigned char __nesc_temp = 
#line 407
              EBUSY;

              {
#line 407
                __nesc_atomic_end(__nesc_atomic); 
#line 407
                return __nesc_temp;
              }
            }
#line 408
          Msp430Adc12ImplP__state |= Msp430Adc12ImplP__ADC_BUSY;
          Msp430Adc12ImplP__clientID = id;
          Msp430Adc12ImplP__configureAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(0).inch);
          Msp430Adc12ImplP__HplAdc12__startConversion();
          if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__USE_TIMERA) {
            Msp430Adc12ImplP__startTimerA();
            }
#line 414
          {
            unsigned char __nesc_temp = 
#line 414
            SUCCESS;

            {
#line 414
              __nesc_atomic_end(__nesc_atomic); 
#line 414
              return __nesc_temp;
            }
          }
        }
    }
#line 418
    __nesc_atomic_end(__nesc_atomic); }
#line 417
  return FAIL;
}

# 137 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static uint8_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ArbiterInfo__userId(void )
#line 137
{
  /* atomic removed: atomic calls only */
#line 138
  {
    if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state != /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 140
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;

#line 140
        return __nesc_temp;
      }
#line 141
    {
      unsigned char __nesc_temp = 
#line 141
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId;

#line 141
      return __nesc_temp;
    }
  }
}

# 91 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__setMode(int mode)
{
  * (volatile uint16_t * )352U = (* (volatile uint16_t * )352U & ~(0x0020 | 0x0010)) | ((mode << 4) & (0x0020 | 0x0010));
}

# 107 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type now = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__Counter__get();
#line 109
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type expires;
#line 109
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 = now + /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt = remaining - /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_t0 += /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt;
      /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type )now << 5, 
  (/*WireAdcStreamP.Alarm.Transform*/TransformAlarmC__1__from_size_type )remaining << 5);
}

# 14 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0018)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/opt/tinyos/tos/system/RealMainP.nc"
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

# 175 "/opt/tinyos/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos/tos/platforms/telosb/MotePlatformC.nc"
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

# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 4;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 5;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

#line 56
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )49U |= 0x01 << 6;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 134 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
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

# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x405964a0){
#line 75
  switch (arg_0x405964a0) {
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
    case AdcP__readDone:
#line 75
      AdcP__readDone__runTask();
#line 75
      break;
#line 75
    case /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask:
#line 75
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    case Msp430RefVoltArbiterImplP__switchOff:
#line 75
      Msp430RefVoltArbiterImplP__switchOff__runTask();
#line 75
      break;
#line 75
    case AdcStreamP__readStreamDone:
#line 75
      AdcStreamP__readStreamDone__runTask();
#line 75
      break;
#line 75
    case AdcStreamP__readStreamFail:
#line 75
      AdcStreamP__readStreamFail__runTask();
#line 75
      break;
#line 75
    case AdcStreamP__bufferDone:
#line 75
      AdcStreamP__bufferDone__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x405964a0);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 121 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static error_t Msp430RefVoltArbiterImplP__ClientResource__release(uint8_t client)
{
  error_t error;

#line 124
  if (Msp430RefVoltArbiterImplP__syncOwner == client) {
    Msp430RefVoltArbiterImplP__switchOff__postTask();
    }
#line 126
  error = Msp430RefVoltArbiterImplP__AdcResource__release(client);
#line 138
  return error;
}

# 97 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_BUSY && /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId == id) {
          if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__isEmpty() == FALSE) {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__dequeue();
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
            }
          else {
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__NO_RES;
              /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceConfigure__unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 75 "/opt/tinyos/tos/system/RoundRobinResourceQueueC.nc"
static bool /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__RoundRobinQueue__isEnqueued(resource_client_id_t id)
#line 75
{
  return /*Msp430Adc12P.Arbiter.Queue*/RoundRobinResourceQueueC__0__resQ[id / 8] & (1 << id % 8);
}

# 147 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__stop(Msp430RefVoltGeneratorP__state_t nextState)
#line 147
{
  error_t result;

  if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__GENERATOR_OFF) {
    result = EALREADY;
    }
  else {
#line 152
    if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE) {
        result = SUCCESS;
        Msp430RefVoltGeneratorP__m_state = nextState;
        Msp430RefVoltGeneratorP__SwitchOffTimer__startOneShot(20);
      }
    else {
#line 156
      if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING) {
          if ((result = Msp430RefVoltGeneratorP__switchOff()) == SUCCESS) {

              Msp430RefVoltGeneratorP__state_t oldState = Msp430RefVoltGeneratorP__m_state;

#line 160
              Msp430RefVoltGeneratorP__SwitchOnTimer__stop();
              Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__GENERATOR_OFF;
              Msp430RefVoltGeneratorP__signalStartDone(oldState, FAIL);
              Msp430RefVoltGeneratorP__signalStopDone(nextState, SUCCESS);
            }
        }
      else {
#line 165
        if (Msp430RefVoltGeneratorP__m_state == nextState) {
          result = SUCCESS;
          }
        else {
#line 168
          result = EBUSY;
          }
        }
      }
    }
#line 170
  return result;
}

# 144 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 259 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__switchOff(void )
#line 259
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 260
    {
      if (Msp430RefVoltGeneratorP__HplAdc12__isBusy()) {
          {
            unsigned char __nesc_temp = 
#line 262
            EBUSY;

            {
#line 262
              __nesc_atomic_end(__nesc_atomic); 
#line 262
              return __nesc_temp;
            }
          }
        }
      else 
#line 264
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP__HplAdc12__getCtl0();

#line 266
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          ctl0.refon = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 270
            SUCCESS;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
}

#line 173
static void Msp430RefVoltGeneratorP__signalStartDone(Msp430RefVoltGeneratorP__state_t state, error_t result)
#line 173
{
  if (state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING) {
    Msp430RefVoltGeneratorP__RefVolt_1_5V__startDone(result);
    }
  else {
#line 177
    Msp430RefVoltGeneratorP__RefVolt_2_5V__startDone(result);
    }
}

# 98 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static void AdcP__SubResourceReadNow__granted(uint8_t nowClient)
{
  if (AdcP__configure(nowClient) == SUCCESS) {
    AdcP__state = AdcP__STATE_READNOW;
    }
  else {
#line 103
    AdcP__state = AdcP__STATE_READNOW_INVALID_CONFIG;
    }
#line 104
  AdcP__ResourceReadNow__granted(nowClient);
}

#line 65
static error_t AdcP__configure(uint8_t client)
{
  error_t result = EINVAL;
  const msp430adc12_channel_config_t * config;

#line 69
  config = AdcP__Config__getConfiguration(client);
  if (config->inch != INPUT_CHANNEL_NONE) {
    result = AdcP__SingleChannel__configureSingle(client, config);
    }
#line 72
  return result;
}

# 182 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static error_t Msp430Adc12ImplP__SingleChannel__configureSingle(uint8_t id, 
const msp430adc12_channel_config_t *config)
{
  error_t result = ERESERVE;

  if (!config) {
    return EINVAL;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 190
    {
      if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__ADC_BUSY) 
        {
          unsigned char __nesc_temp = 
#line 192
          EBUSY;

          {
#line 192
            __nesc_atomic_end(__nesc_atomic); 
#line 192
            return __nesc_temp;
          }
        }
#line 193
      if (Msp430Adc12ImplP__ADCArbiterInfo__userId() == id) {
          adc12ctl1_t ctl1 = { 
          .adc12busy = 0, 
          .conseq = 0, 
          .adc12ssel = config->adc12ssel, 
          .adc12div = config->adc12div, 
          .issh = 0, 
          .shp = 1, 
          .shs = 0, 
          .cstartadd = 0 };

          adc12memctl_t memctl = { 
          .inch = config->inch, 
          .sref = config->sref, 
          .eos = 1 };

          adc12ctl0_t ctl0 = Msp430Adc12ImplP__HplAdc12__getCtl0();

#line 210
          ctl0.msc = 1;
          ctl0.sht0 = config->sht;
          ctl0.sht1 = config->sht;

          Msp430Adc12ImplP__state = Msp430Adc12ImplP__SINGLE_DATA;
          Msp430Adc12ImplP__HplAdc12__setCtl0(ctl0);
          Msp430Adc12ImplP__HplAdc12__setCtl1(ctl1);
          Msp430Adc12ImplP__HplAdc12__setMCtl(0, memctl);
          Msp430Adc12ImplP__HplAdc12__setIEFlags(0x01);
          result = SUCCESS;
        }
    }
#line 221
    __nesc_atomic_end(__nesc_atomic); }
  return result;
}

# 221 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__ReadStream__read(uint8_t c, uint32_t usPeriod)
{
  if (usPeriod & 0xFFFF0000) {

      AdcStreamP__period = usPeriod / 1000;
      AdcStreamP__periodModified = TRUE;
      AdcStreamP__client = c;
      AdcStreamP__now = AdcStreamP__Alarm__getNow();
      AdcStreamP__SingleChannel__configureSingle(c, AdcStreamP__AdcConfigure__getConfiguration(c));
      if (AdcStreamP__nextBuffer(FALSE) == SUCCESS) {
        AdcStreamP__sampleSingle();
        }
    }
  else 
#line 232
    {
      AdcStreamP__period = usPeriod;
      AdcStreamP__periodModified = FALSE;
      AdcStreamP__client = c;
      AdcStreamP__nextMultiple(c);
    }
  return SUCCESS;
}

#line 177
static error_t AdcStreamP__nextBuffer(bool startNextAlarm)
#line 177
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      struct AdcStreamP__list_entry_t *entry = AdcStreamP__bufferQueue[AdcStreamP__client];

      if (!entry) 
        {

          AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
          AdcStreamP__readStreamDone__postTask();
          {
            unsigned char __nesc_temp = 
#line 187
            FAIL;

            {
#line 187
              __nesc_atomic_end(__nesc_atomic); 
#line 187
              return __nesc_temp;
            }
          }
        }
      else 
#line 190
        {
          uint16_t tmp_count;

#line 192
          AdcStreamP__bufferQueue[AdcStreamP__client] = entry->next;
          if (!AdcStreamP__bufferQueue[AdcStreamP__client]) {
            AdcStreamP__bufferQueueEnd[AdcStreamP__client] = &AdcStreamP__bufferQueue[AdcStreamP__client];
            }
#line 195
          AdcStreamP__pos = AdcStreamP__buffer = (void *)0;
          AdcStreamP__count = entry->count;
          tmp_count = AdcStreamP__count;
          AdcStreamP__pos = AdcStreamP__buffer = (uint16_t * )entry;
          if (startNextAlarm) {
            AdcStreamP__nextAlarm();
            }
#line 201
          {
            unsigned char __nesc_temp = 
#line 201
            SUCCESS;

            {
#line 201
              __nesc_atomic_end(__nesc_atomic); 
#line 201
              return __nesc_temp;
            }
          }
        }
    }
#line 205
    __nesc_atomic_end(__nesc_atomic); }
}

#line 206
static void AdcStreamP__nextMultiple(uint8_t c)
{
  if (AdcStreamP__nextBuffer(FALSE) == SUCCESS) {
      msp430adc12_channel_config_t config = *AdcStreamP__AdcConfigure__getConfiguration(c);

#line 210
      config.sampcon_ssel = SAMPCON_SOURCE_SMCLK;
      config.sampcon_id = SAMPCON_CLOCK_DIV_1;
      if (AdcStreamP__SingleChannel__configureMultiple(c, &config, AdcStreamP__pos, AdcStreamP__count, AdcStreamP__period) == SUCCESS) {
        AdcStreamP__SingleChannel__getData(c);
        }
      else 
#line 214
        {
          AdcStreamP__postBuffer(c, AdcStreamP__pos, AdcStreamP__count);
          AdcStreamP__readStreamFail__postTask();
        }
    }
}

# 80 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static void AdcP__ResourceRead__granted(uint8_t client)
{

  error_t result = AdcP__configure(client);

#line 84
  if (result == SUCCESS) {
      AdcP__state = AdcP__STATE_READ;
      result = AdcP__SingleChannel__getData(client);
    }
  else 
#line 87
    {
      AdcP__ResourceRead__release(client);
      AdcP__Read__readDone(client, result, 0);
    }
}

# 70 "SenseC.nc"
static void SenseC__Read__readDone(error_t result, uint16_t data)
{
  if (result == SUCCESS) {
      if (data & 0x0004) {
        SenseC__Leds__led2On();
        }
      else {
#line 76
        SenseC__Leds__led2Off();
        }
#line 77
      if (data & 0x0002) {
        SenseC__Leds__led1On();
        }
      else {
#line 80
        SenseC__Leds__led1Off();
        }
#line 81
      if (data & 0x0001) {
        SenseC__Leds__led0On();
        }
      else {
#line 84
        SenseC__Leds__led0Off();
        }
    }
}

# 112 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP__RefVolt_2_5V__startDone(error_t error)
{
  if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {


      Msp430RefVoltArbiterImplP__ClientResource__granted(Msp430RefVoltArbiterImplP__syncOwner);
    }
}

# 180 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static void Msp430RefVoltGeneratorP__signalStopDone(Msp430RefVoltGeneratorP__state_t state, error_t result)
#line 180
{
  if (state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING) {
    Msp430RefVoltGeneratorP__RefVolt_1_5V__stopDone(result);
    }
  else {
#line 184
    Msp430RefVoltGeneratorP__RefVolt_2_5V__stopDone(result);
    }
}

# 71 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltArbiterImplP.nc"
static void Msp430RefVoltArbiterImplP__AdcResource__granted(uint8_t client)
{
  const msp430adc12_channel_config_t *settings = Msp430RefVoltArbiterImplP__Config__getConfiguration(client);

#line 74
  if (settings->sref == REFERENCE_VREFplus_AVss || 
  settings->sref == REFERENCE_VREFplus_VREFnegterm) {
      error_t started;

#line 77
      if (Msp430RefVoltArbiterImplP__syncOwner != Msp430RefVoltArbiterImplP__NO_OWNER) {



          Msp430RefVoltArbiterImplP__AdcResource__release(client);
          Msp430RefVoltArbiterImplP__AdcResource__request(client);
          return;
        }
      Msp430RefVoltArbiterImplP__syncOwner = client;
      if (settings->ref2_5v == REFVOLT_LEVEL_1_5) {
          Msp430RefVoltArbiterImplP__ref2_5v = FALSE;
          started = Msp430RefVoltArbiterImplP__RefVolt_1_5V__start();
        }
      else {
          Msp430RefVoltArbiterImplP__ref2_5v = TRUE;
          started = Msp430RefVoltArbiterImplP__RefVolt_2_5V__start();
        }
      if (started != SUCCESS) {
          Msp430RefVoltArbiterImplP__syncOwner = Msp430RefVoltArbiterImplP__NO_OWNER;
          Msp430RefVoltArbiterImplP__AdcResource__release(client);
          Msp430RefVoltArbiterImplP__AdcResource__request(client);
        }
    }
  else {
#line 100
    Msp430RefVoltArbiterImplP__ClientResource__granted(client);
    }
}

# 71 "/opt/tinyos/tos/system/SimpleArbiterP.nc"
static error_t /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Resource__request(uint8_t id)
#line 71
{
  /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__ResourceRequested__requested(/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state == /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_IDLE) {
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__state = /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__RES_GRANTING;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__reqResId = id;
          /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__grantedTask__postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*Msp430Adc12P.Arbiter.Arbiter*/SimpleArbiterP__0__Queue__enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 95 "/opt/tinyos/tos/chips/msp430/adc12/Msp430RefVoltGeneratorP.nc"
static error_t Msp430RefVoltGeneratorP__start(Msp430RefVoltGeneratorP__state_t targetState)
#line 95
{
  error_t result;

  if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_STABLE) {
      if (targetState == Msp430RefVoltGeneratorP__m_state) {
          result = EALREADY;
        }
      else {
#line 101
        if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {
            if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE) {
                Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING;
                Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(17);
              }
            else 
#line 105
              {
                Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING;
                Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(70);
              }
          }
        }
    }
  else {
#line 110
    if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__GENERATOR_OFF) {
        if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {
            if (targetState == Msp430RefVoltGeneratorP__REFERENCE_1_5V_STABLE && Msp430RefVoltGeneratorP__SwitchOffSettleTimer__isRunning()) {
                Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(70);
              }
            else {
              Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(17);
              }
#line 117
            Msp430RefVoltGeneratorP__SwitchOffSettleTimer__stop();
            Msp430RefVoltGeneratorP__m_state = targetState + 2;
          }
      }
    else {
#line 120
      if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING || Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_2_5V_OFF_PENDING) {
          if ((result = Msp430RefVoltGeneratorP__switchOn(targetState)) == SUCCESS) {

              Msp430RefVoltGeneratorP__state_t oldState = Msp430RefVoltGeneratorP__m_state;

#line 124
              Msp430RefVoltGeneratorP__SwitchOffTimer__stop();
              Msp430RefVoltGeneratorP__signalStopDone(oldState, FAIL);
              if (targetState == Msp430RefVoltGeneratorP__m_state - 4) {
                  Msp430RefVoltGeneratorP__m_state = targetState;
                  Msp430RefVoltGeneratorP__signalStartDone(targetState, SUCCESS);
                }
              else {
#line 130
                if (Msp430RefVoltGeneratorP__m_state == Msp430RefVoltGeneratorP__REFERENCE_1_5V_OFF_PENDING) {
                    Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_2_5V_ON_PENDING;
                    Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(17);
                  }
                else {
                    Msp430RefVoltGeneratorP__m_state = Msp430RefVoltGeneratorP__REFERENCE_1_5V_ON_PENDING;
                    Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(70);
                  }
                }
            }
        }
      else {
#line 139
        if (Msp430RefVoltGeneratorP__m_state == targetState + 2) {
          result = SUCCESS;
          }
        else {
#line 142
          result = EBUSY;
          }
        }
      }
    }
#line 144
  return result;
}

#line 240
static error_t Msp430RefVoltGeneratorP__switchOn(uint8_t level)
#line 240
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 241
    {
      if (Msp430RefVoltGeneratorP__HplAdc12__isBusy()) {
          {
            unsigned char __nesc_temp = 
#line 243
            EBUSY;

            {
#line 243
              __nesc_atomic_end(__nesc_atomic); 
#line 243
              return __nesc_temp;
            }
          }
        }
      else 
#line 245
        {
          adc12ctl0_t ctl0 = Msp430RefVoltGeneratorP__HplAdc12__getCtl0();

#line 247
          ctl0.enc = 0;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          ctl0.refon = 1;


          ctl0.r2_5v = level - 1;
          Msp430RefVoltGeneratorP__HplAdc12__setCtl0(ctl0);
          {
            unsigned char __nesc_temp = 
#line 254
            SUCCESS;

            {
#line 254
              __nesc_atomic_end(__nesc_atomic); 
#line 254
              return __nesc_temp;
            }
          }
        }
    }
#line 258
    __nesc_atomic_end(__nesc_atomic); }
}

# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void Msp430RefVoltGeneratorP__SwitchOnTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 73
}
#line 73
# 73 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
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

# 147 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
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

# 105 "/opt/tinyos/tos/chips/msp430/adc12/HplAdc12P.nc"
static void HplAdc12P__HplAdc12__stopConversion(void )
#line 105
{

  uint16_t ctl1 = HplAdc12P__ADC12CTL1;

#line 108
  HplAdc12P__ADC12CTL1 &= ~(0x0002 | 0x0004);
  HplAdc12P__ADC12CTL0 &= ~(0x001 + 0x002);
  HplAdc12P__ADC12CTL0 &= ~0x010;
  HplAdc12P__ADC12CTL1 |= ctl1 & (0x0002 | 0x0004);
}







__attribute((wakeup)) __attribute((interrupt(0x000E)))  void sig_ADC12_VECTOR(void )
#line 120
{
  HplAdc12P__HplAdc12__conversionDone(HplAdc12P__ADC12IV);
}

# 509 "/opt/tinyos/tos/chips/msp430/adc12/Msp430Adc12ImplP.nc"
static void Msp430Adc12ImplP__stopConversion(void )
{
  uint8_t i;

  if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__USE_TIMERA) {
    Msp430Adc12ImplP__TimerA__setMode(MSP430TIMER_STOP_MODE);
    }
  Msp430Adc12ImplP__resetAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(0).inch);
  if (Msp430Adc12ImplP__state & Msp430Adc12ImplP__MULTI_CHANNEL) {
      for (i = 1; i < Msp430Adc12ImplP__numChannels; i++) 
        Msp430Adc12ImplP__resetAdcPin(Msp430Adc12ImplP__HplAdc12__getMCtl(i).inch);
    }
  /* atomic removed: atomic calls only */
#line 521
  {
    Msp430Adc12ImplP__HplAdc12__stopConversion();
    Msp430Adc12ImplP__HplAdc12__resetIFGs();
    Msp430Adc12ImplP__state &= ~Msp430Adc12ImplP__ADC_BUSY;
  }
}

#line 165
static void Msp430Adc12ImplP__resetAdcPin(uint8_t inch)
{

  switch (inch) 
    {
      case 0: Msp430Adc12ImplP__Port60__selectIOFunc();
#line 170
      break;
      case 1: Msp430Adc12ImplP__Port61__selectIOFunc();
#line 171
      break;
      case 2: Msp430Adc12ImplP__Port62__selectIOFunc();
#line 172
      break;
      case 3: Msp430Adc12ImplP__Port63__selectIOFunc();
#line 173
      break;
      case 4: Msp430Adc12ImplP__Port64__selectIOFunc();
#line 174
      break;
      case 5: Msp430Adc12ImplP__Port65__selectIOFunc();
#line 175
      break;
      case 6: Msp430Adc12ImplP__Port66__selectIOFunc();
#line 176
      break;
      case 7: Msp430Adc12ImplP__Port67__selectIOFunc();
#line 177
      break;
    }
}

# 142 "/opt/tinyos/tos/chips/msp430/adc12/AdcP.nc"
static error_t AdcP__SingleChannel__singleDataReady(uint8_t client, uint16_t data)
{
  switch (AdcP__state) 
    {
      case AdcP__STATE_READ: 
        AdcP__owner = client;
      AdcP__value = data;
      AdcP__readDone__postTask();
      break;
      case AdcP__STATE_READNOW: 
        AdcP__ReadNow__readDone(client, SUCCESS, data);
      break;
      default: 

        break;
    }
  return SUCCESS;
}

# 242 "/opt/tinyos/tos/chips/msp430/adc12/AdcStreamP.nc"
static error_t AdcStreamP__SingleChannel__singleDataReady(uint8_t streamClient, uint16_t data)
{
  if (AdcStreamP__client == AdcStreamP__NSTREAM) {
    return FAIL;
    }
  if (AdcStreamP__count == 0) 
    {
      AdcStreamP__now = AdcStreamP__Alarm__getNow();
      AdcStreamP__nextBuffer(TRUE);
    }
  else 
    {
      * AdcStreamP__pos++ = data;
      if (AdcStreamP__pos == AdcStreamP__buffer + AdcStreamP__count) 
        {
          /* atomic removed: atomic calls only */
          {
            if (AdcStreamP__lastBuffer) 
              {

                AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
                AdcStreamP__readStreamFail__postTask();
                {
                  unsigned char __nesc_temp = 
#line 264
                  FAIL;

#line 264
                  return __nesc_temp;
                }
              }
            else {
                AdcStreamP__lastCount = AdcStreamP__count;
                AdcStreamP__lastBuffer = AdcStreamP__buffer;
              }
          }
          AdcStreamP__bufferDone__postTask();
          AdcStreamP__nextBuffer(TRUE);
        }
      else {
        AdcStreamP__nextAlarm();
        }
    }
#line 278
  return FAIL;
}

static uint16_t *AdcStreamP__SingleChannel__multipleDataReady(uint8_t streamClient, 
uint16_t *buf, uint16_t length)
{
  /* atomic removed: atomic calls only */
  {
    if (AdcStreamP__lastBuffer) 
      {

        AdcStreamP__bufferQueueEnd[AdcStreamP__client] = (void *)0;
        AdcStreamP__readStreamFail__postTask();
        {
          unsigned int *__nesc_temp = 
#line 291
          0;

#line 291
          return __nesc_temp;
        }
      }
    else {
        AdcStreamP__lastBuffer = AdcStreamP__buffer;
        AdcStreamP__lastCount = AdcStreamP__pos - AdcStreamP__buffer;
      }
  }
  AdcStreamP__bufferDone__postTask();
  AdcStreamP__nextMultiple(streamClient);
  return 0;
}

