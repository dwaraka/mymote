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





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 303
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 340
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
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
# 37 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/string.h" 3
extern int memcmp(const void *arg_0x402abcd8, const void *arg_0x402abe70, size_t arg_0x402aa030);
extern void *memcpy(void *arg_0x402aa4d8, const void *arg_0x402aa670, size_t arg_0x402aa808);
extern void *memmove(void *arg_0x402aacd8, const void *arg_0x402aae70, size_t arg_0x402af030);
extern void *memset(void *arg_0x402af4f8, int arg_0x402af650, size_t arg_0x402af7e8);








extern int strncmp(const char *arg_0x402b0ee0, const char *arg_0x402b5088, size_t arg_0x402b5220);
#line 61
extern void *memset(void *arg_0x402ba358, int arg_0x402ba4b0, size_t arg_0x402ba648);
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
#line 95
void *malloc(size_t size);
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

  void (*__cleanup)(struct _reent *arg_0x402df3b8);


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


  void (**_sig_func)(int arg_0x402dda70);




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
uint16_t TOS_NODE_ID = 1;






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

extern volatile unsigned char P1IFG __asm ("__""P1IFG");

extern volatile unsigned char P1IES __asm ("__""P1IES");

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
#line 380
extern volatile unsigned char U0CTL __asm ("__""U0CTL");





extern volatile unsigned char U0MCTL __asm ("__""U0MCTL");

extern volatile unsigned char U0BR0 __asm ("__""U0BR0");

extern volatile unsigned char U0BR1 __asm ("__""U0BR1");

extern const volatile unsigned char U0RXBUF __asm ("__""U0RXBUF");
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
# 52 "/opt/tinyos/tos/types/Storage.h"
typedef uint8_t volume_id_t;
typedef uint32_t storage_addr_t;
typedef uint32_t storage_len_t;
typedef uint32_t storage_cookie_t;

enum __nesc_unnamed4253 {
  SEEK_BEGINNING = 0
};
# 40 "/opt/tinyos/tos/chips/stm25p/Stm25p.h"
typedef storage_addr_t stm25p_addr_t;
typedef storage_len_t stm25p_len_t;

enum __nesc_unnamed4254 {
  STM25P_NUM_SECTORS = 16, 
  STM25P_SECTOR_SIZE_LOG2 = 16, 
  STM25P_SECTOR_SIZE = 1L << STM25P_SECTOR_SIZE_LOG2, 
  STM25P_SECTOR_MASK = 0xffff, 
  STM25P_PAGE_SIZE_LOG2 = 8, 
  STM25P_PAGE_SIZE = 1 << STM25P_PAGE_SIZE_LOG2, 
  STM25P_PAGE_MASK = STM25P_PAGE_SIZE - 1, 
  STM25P_INVALID_ADDRESS = 0xffffffff
};




#line 54
typedef struct stm25p_volume_info_t {
  uint8_t base;
  uint8_t size;
} stm25p_volume_info_t;
# 35 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/6lowpan.h"
enum __nesc_unnamed4255 {
  LOWMSG_MESH_LEN = 5, 
  LOWMSG_BCAST_LEN = 2, 
  LOWMSG_FRAG1_LEN = 4, 
  LOWMSG_FRAGN_LEN = 5
};

enum __nesc_unnamed4256 {
  INET_MTU = 1280, 
  LIB6LOWPAN_MAX_LEN = 100, 
  LOWPAN_LINK_MTU = 109, 




  FRAG_EXPIRE_TIME = 4096
};




enum __nesc_unnamed4257 {
  LOWPAN_NALP_PATTERN = 0x0, 
  LOWPAN_MESH_PATTERN = 0x2, 
  LOWPAN_FRAG1_PATTERN = 0x18, 
  LOWPAN_FRAGN_PATTERN = 0x1c, 
  LOWPAN_BCAST_PATTERN = 0x50, 
  LOWPAN_IPV6_PATTERN = 0x41
};

enum __nesc_unnamed4258 {
  LOWPAN_MESH_V_MASK = 0x20, 
  LOWPAN_MESH_F_MASK = 0x10, 
  LOWPAN_MESH_HOPS_MASK = 0x0f
};




enum __nesc_unnamed4259 {
  LOWPAN_DISPATCH_BYTE_MASK = 0xe0, 
  LOWPAN_DISPATCH_BYTE_VAL = 0x60, 

  LOWPAN_IPHC_TF_MASK = 0x18, 
  LOWPAN_IPHC_TF_NONE = 0x18, 
  LOWPAN_IPHC_TF_ECN_DSCP = 0x10, 
  LOWPAN_IPHC_TF_ECN_FL = 0x08, 
  LOWPAN_IPHC_TF_ECN_DSCP_FL = 0x00, 

  LOWPAN_IPHC_NH_MASK = 0x04, 
  LOWPAN_IPHC_NH_INLINE = 0, 

  LOWPAN_IPHC_HLIM_MASK = 0x03, 
  LOWPAN_IPHC_HLIM_NONE = 0x00, 
  LOWPAN_IPHC_HLIM_1 = 0x01, 
  LOWPAN_IPHC_HLIM_64 = 0x02, 
  LOWPAN_IPHC_HLIM_255 = 0x03, 

  LOWPAN_IPHC_CID_MASK = 0x80, 
  LOWPAN_IPHC_CID_PRESENT = 0x80, 

  LOWPAN_IPHC_SAM_SHIFT = 4, 
  LOWPAN_IPHC_M = 0x08, 
  LOWPAN_IPHC_DAM_SHIFT = 0, 

  LOWPAN_IPHC_AC_CONTEXT = 0x04, 
  LOWPAN_IPHC_AM_MASK = 0x3, 
  LOWPAN_IPHC_AM_128 = 0x0, 
  LOWPAN_IPHC_AM_64 = 0x1, 
  LOWPAN_IPHC_AM_16 = 0x2, 
  LOWPAN_IPHC_AM_0 = 0x3, 

  LOWPAN_IPHC_AM_M = 0x08, 
  LOWPAN_IPHC_AM_M_128 = 0x0, 
  LOWPAN_IPHC_AM_M_48 = 0x1, 
  LOWPAN_IPHC_AM_M_32 = 0x2, 
  LOWPAN_IPHC_AM_M_8 = 0x3
};




enum __nesc_unnamed4260 {
  LOWPAN_NHC_IPV6_MASK = 0xf0, 
  LOWPAN_NHC_IPV6_PATTERN = 0xe0, 

  LOWPAN_NHC_EID_SHIFT = 0x1, 
  LOWPAN_NHC_EID_MASK = 0xe, 
  LOWPAN_NHC_EID_HOP = 0x0 << LOWPAN_NHC_EID_SHIFT, 
  LOWPAN_NHC_EID_ROUTING = 0x1 << LOWPAN_NHC_EID_SHIFT, 
  LOWPAN_NHC_EID_FRAG = 0x2 << LOWPAN_NHC_EID_SHIFT, 
  LOWPAN_NHC_EID_DEST = 0x3 << LOWPAN_NHC_EID_SHIFT, 
  LOWPAN_NHC_EID_MOBILE = 0x4 << LOWPAN_NHC_EID_SHIFT, 
  LOWPAN_NHC_EID_IPV6 = 0x7 << LOWPAN_NHC_EID_SHIFT, 

  LOWPAN_NHC_NH = 0x1, 

  LOWPAN_NHC_UDP_MASK = 0xf8, 
  LOWPAN_NHC_UDP_PATTERN = 0xf0, 

  LOWPAN_NHC_UDP_CKSUM = 0x4, 

  LOWPAN_NHC_UDP_PORT_MASK = 0x3, 
  LOWPAN_NHC_UDP_PORT_FULL = 0x0, 
  LOWPAN_NHC_UDP_PORT_SRC_FULL = 0x1, 
  LOWPAN_NHC_UDP_PORT_DST_FULL = 0x2, 
  LOWPAN_NHC_UDP_PORT_SHORT = 0x3
};
# 23 "/opt/tinyos/support/sdk/c/coap/list.h"
struct coap_linkedlistnode {
  struct coap_linkedlistnode *next;
  void *data;





  void (*delete)(void *arg_0x405c37e0);
};

typedef struct coap_linkedlistnode coap_list_t;
# 28 "/opt/tinyos/support/sdk/c/coap/str.h"
#line 25
typedef struct __nesc_unnamed4261 {
  size_t length;
  unsigned char *s;
} str;
# 29 "/opt/tinyos/support/sdk/c/coap/uri.h"
#line 25
typedef struct __nesc_unnamed4262 {
  str na;
  str path;
  str query;
} coap_uri_t;
#line 46
coap_uri_t *coap_new_uri(const unsigned char *uri, unsigned int length);
# 122 "/opt/tinyos/support/sdk/c/coap/pdu.h"
typedef unsigned short coap_tid_t;
#line 141
#line 135
typedef struct __nesc_unnamed4263 {
  unsigned int optcnt : 4;
  unsigned int type : 2;
  unsigned int version : 2;
  unsigned int code : 8;
  unsigned short id;
} coap_hdr_t;
#line 171
#line 159
typedef union __nesc_unnamed4264 {
  struct __nesc_unnamed4265 {
    unsigned int length : 4;
    unsigned int delta : 4;
  } 
  sval;
  struct __nesc_unnamed4266 {
    unsigned int flag : 4;
    unsigned int delta : 4;
    unsigned int length : 8;
  } 
  lval;
} coap_opt_t;
#line 215
#line 204
typedef struct __nesc_unnamed4267 {
  unsigned short key;
  unsigned int length;
} 







coap_option;
#line 234
#line 229
typedef struct __nesc_unnamed4268 {
  coap_hdr_t *hdr;
  unsigned short length;
  coap_list_t *options;
  unsigned char *data;
} coap_pdu_t;









coap_pdu_t *coap_new_pdu();
void coap_delete_pdu(coap_pdu_t *arg_0x405d9110);










int coap_add_option(coap_pdu_t *pdu, unsigned char type, unsigned int len, const unsigned char *data);
coap_opt_t *coap_check_option(coap_pdu_t *pdu, unsigned char type);
#line 273
int coap_add_data(coap_pdu_t *pdu, unsigned int len, const unsigned char *data);






int coap_get_data(coap_pdu_t *pdu, unsigned int *len, unsigned char **data);








int coap_get_request_uri(coap_pdu_t *pdu, coap_uri_t *result);
# 51 "tinyos_coap_resources.h"
#line 42
typedef struct key_uri {

  uint8_t key;
  char uri[5];
  uint8_t urilen;
  uint8_t mediatype;
  uint8_t writable : 1;
  uint8_t splitphase : 1;
  uint8_t immediately : 1;
} key_uri_t;




enum __nesc_unnamed4269 {
#line 70
  KEY_LED, 







  COAP_NO_SUCH_RESOURCE = 0xff
};
#line 92
#line 81
typedef nx_struct val_all {
  unsigned char __nesc_filler0[1];


  nx_uint16_t temp;
  unsigned char __nesc_filler1[1];

  nx_uint16_t hum;
  unsigned char __nesc_filler2[1];

  nx_uint16_t volt;
} __attribute__((packed)) val_all_t;






#line 94
typedef nx_struct config_t {

  nx_uint8_t version;
  nx_uint8_t EUI64[8];
  nx_uint8_t KEY128[16];
} __attribute__((packed)) config_t;

key_uri_t uri_key_map[1] = { 
#line 119
{ KEY_LED, "l", sizeof "l", 
42, 1, 1, 1 } };
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4270 {
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
typedef struct __nesc_unnamed4271 {

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
typedef struct __nesc_unnamed4272 {

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
typedef struct __nesc_unnamed4273 {

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
# 43 "/opt/tinyos/tos/types/Leds.h"
enum __nesc_unnamed4274 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos/tos/chips/cc2520/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler3[1];


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



  nx_uint16_t maxRetries;
  nx_uint16_t retryDelay;
} __attribute__((packed)) 
cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4275 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 112 + MAC_FOOTER_SIZE, 

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


enum __nesc_unnamed4276 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/opt/tinyos/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4277 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4278 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/opt/tinyos/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4279 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4280 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4281 {
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
# 59 "/opt/tinyos/tos/platforms/telosa/platform_message.h"
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
# 19 "/opt/tinyos/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[112];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 27 "/opt/tinyos/tos/lib/net/blip/IPDispatch.h"
enum __nesc_unnamed4282 {
  N_RECONSTRUCTIONS = 3, 
  N_CONCURRENT_SENDS = 3, 
  N_FRAGMENTS = 12
};

struct send_info {
  void *upper_data;
  uint8_t link_fragments;
  uint8_t link_transmissions;
  uint8_t link_fragment_attempts;
  bool failed;
  uint8_t _refcount;
};

struct send_entry {
  struct send_info *info;
  message_t *msg;
};
# 24 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/nwbyte.h"
uint32_t ntohl(uint32_t i);
# 40 "/opt/tinyos/tos/types/IeeeEui64.h"
enum __nesc_unnamed4283 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 47 "/opt/tinyos/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 51
typedef struct __nesc_unnamed4284 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4285 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4286 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4287 {
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
# 24 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/nwbyte.h"
uint32_t ntohl(uint32_t i);
# 7 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/iovec.h"
struct ip_iovec {
  uint8_t *iov_base;
  size_t iov_len;
  struct ip_iovec *iov_next;
};

int iov_read(struct ip_iovec *iov, int offset, int len, uint8_t *buf);
int iov_len(struct ip_iovec *iov);
# 42 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/ip.h"
struct in6_addr {

  union __nesc_unnamed4288 {

    uint8_t u6_addr8[16];
    uint16_t u6_addr16[8];
    uint32_t u6_addr32[4];
  } in6_u;
};




struct sockaddr_in6 {
  uint16_t sin6_port;
  struct in6_addr sin6_addr;
};









struct ip6_hdr {
  union __nesc_unnamed4289 {
    struct ip6_hdrctl {
      uint32_t ip6_un1_flow;
      uint16_t ip6_un1_plen;
      uint8_t ip6_un1_nxt;
      uint8_t ip6_un1_hlim;
    } ip6_un1;
    uint8_t ip6_un2_vfc;
  } ip6_ctlun;
  struct in6_addr ip6_src;
  struct in6_addr ip6_dst;
} __attribute((packed)) ;
#line 110
struct ip6_ext {
  uint8_t ip6e_nxt;
  uint8_t ip6e_len;
};

struct tlv_hdr {
  uint8_t type;
  uint8_t len;
};



enum __nesc_unnamed4290 {
  IANA_ICMP = 58, 
  IANA_UDP = 17, 
  IANA_TCP = 6, 




  IPV6_HOP = 0, 
  IPV6_IPV6 = 41, 
  IPV6_ROUTING = 43, 
  IPV6_FRAG = 44, 
  IPV6_AUTH = 51, 
  IPV6_SEC = 50, 
  IPV6_NONEXT = 59, 
  IPV6_DEST = 60, 
  IPV6_MOBILITY = 135, 

  IPV6_TLV_PAD1 = 0, 
  IPV6_TLV_PADN = 1
};




struct in6_iid {
  uint8_t data[8];
};




struct icmp6_hdr {
  uint8_t type;
  uint8_t code;
  uint16_t cksum;
};

enum __nesc_unnamed4291 {
  ICMP_TYPE_ECHO_DEST_UNREACH = 1, 
  ICMP_TYPE_ECHO_PKT_TOO_BIG = 2, 
  ICMP_TYPE_ECHO_TIME_EXCEEDED = 3, 
  ICMP_TYPE_ECHO_PARAM_PROBLEM = 4, 
  ICMP_TYPE_ECHO_REQUEST = 128, 
  ICMP_TYPE_ECHO_REPLY = 129, 
  ICMP_TYPE_ROUTER_SOL = 133, 
  ICMP_TYPE_ROUTER_ADV = 134, 
  ICMP_TYPE_NEIGHBOR_SOL = 135, 
  ICMP_TYPE_NEIGHBOR_ADV = 136, 
  ICMP_TYPE_RPL_CONTROL = 155, 
  ICMP_NEIGHBOR_HOPLIMIT = 255, 

  ICMP_CODE_HOPLIMIT_EXCEEDED = 0, 
  ICMP_CODE_ASSEMBLY_EXCEEDED = 1
};




struct udp_hdr {
  uint16_t srcport;
  uint16_t dstport;
  uint16_t len;
  uint16_t chksum;
};




enum __nesc_unnamed4292 {
  TCP_FLAG_FIN = 0x1, 
  TCP_FLAG_SYN = 0x2, 
  TCP_FLAG_RST = 0x4, 
  TCP_FLAG_PSH = 0x8, 
  TCP_FLAG_ACK = 0x10, 
  TCP_FLAG_URG = 0x20, 
  TCP_FLAG_ECE = 0x40, 
  TCP_FLAG_CWR = 0x80
};

struct tcp_hdr {
  uint16_t srcport;
  uint16_t dstport;
  uint32_t seqno;
  uint32_t ackno;
  uint8_t offset;
  uint8_t flags;
  uint16_t window;
  uint16_t chksum;
  uint16_t urgent;
};







struct ip6_metadata {
  ieee154_addr_t sender;

  uint8_t lqi;
  uint8_t rssi;
};
#line 244
struct ip6_packet {
  struct ip_iovec *ip6_data;
  struct ip6_hdr ip6_hdr;
};






void inet_pton6(char *addr, struct in6_addr *dest);
# 38 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.h"
uint8_t *ip_memcpy(uint8_t *dst0, const uint8_t *src0, uint16_t len);


uint16_t ieee154_hashaddr(ieee154_addr_t *addr);




struct packed_lowmsg {
  uint8_t headers;
  uint8_t len;
  uint8_t *data;
};

struct lowpan_reconstruct {
  uint16_t r_tag;
  uint16_t r_source_key;
  uint16_t r_size;
  uint8_t *r_buf;
  uint16_t r_bytes_rcvd;

  uint8_t r_timeout;
  uint16_t *r_app_len;
  uint8_t *r_transport_header;
  struct ip6_metadata r_meta;
};

struct lowpan_ctx {
  uint16_t tag;
  uint16_t offset;
};


enum __nesc_unnamed4293 {
  LOWMSG_MESH_HDR = 1 << 0, 
  LOWMSG_BCAST_HDR = 1 << 1, 
  LOWMSG_FRAG1_HDR = 1 << 2, 
  LOWMSG_FRAGN_HDR = 1 << 3, 
  LOWMSG_NALP = 1 << 4, 
  LOWMSG_IPNH_HDR = 1 << 5, 
  LOWMSG_IPV6 = 1 << 6
};
#line 183
enum __nesc_unnamed4294 {
  T_FAILED1 = 0, 
  T_FAILED2 = 1, 
  T_UNUSED = 2, 
  T_ACTIVE = 3, 
  T_ZOMBIE = 4
};
# 40 "/usr/bin/../lib/gcc/msp430/4.5.3/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 102
typedef __gnuc_va_list va_list;
# 50 "/usr/bin/../lib/gcc/msp430/4.5.3/../../../../msp430/include/stdio.h" 3
int __attribute((format(printf, 3, 4))) snprintf(char *buf, size_t size, const char *fmt, ...);
# 26 "/opt/tinyos/support/sdk/c/coap/net.h"
typedef uint16_t ssize_t;
#line 38
struct coap_listnode {
  struct coap_listnode *next;




  unsigned char retransmit_cnt;

  struct sockaddr_in6 remote;

  coap_pdu_t *pdu;
};

typedef struct coap_listnode coap_queue_t;


int coap_insert_node(coap_queue_t **queue, coap_queue_t *node, 
int (*order)(coap_queue_t *arg_0x408c4e60, coap_queue_t *node));


int coap_delete_node(coap_queue_t *node);





coap_queue_t *coap_new_node();
#line 76
#line 67
typedef struct __nesc_unnamed4295 {
  coap_list_t *resources, *subscriptions;
  coap_queue_t *sendqueue, *recvqueue;



  int reqtoken;
  void (*msg_handler)(void *arg_0x408c1d98, coap_queue_t *arg_0x408de010, void *arg_0x408de188);
  coap_queue_t *splitphasequeue;
} coap_context_t;

typedef void (*coap_message_handler_t)(coap_context_t *arg_0x408dee48, coap_queue_t *arg_0x408dd068, void *arg_0x408dd1e0);





void coap_register_message_handler(coap_context_t *context, coap_message_handler_t handler);
#line 141
coap_queue_t *coap_find_transaction(coap_queue_t *queue, coap_tid_t id);


void coap_dispatch(coap_context_t *context);




int order_transaction_id(coap_queue_t *lhs, coap_queue_t *rhs);
# 6 "/opt/tinyos/tos/lib/net/blip/iprouting.h"
enum __nesc_unnamed4296 {
  ROUTE_INVAL_KEY = -1, 
  ROUTE_TABLE_SZ = 20
};

enum __nesc_unnamed4297 {
  ROUTE_IFACE_ALL = 0, 
  ROUTE_IFACE_154 = 1, 
  ROUTE_IFACE_PPP = 2
};

enum __nesc_unnamed4298 {
  ROUTE_DROP_NOROUTE, 
  ROUTE_DROP_HLIM
};

typedef int route_key_t;

struct route_entry {
  int valid : 1;
  route_key_t key;
  struct in6_addr prefix;
  uint8_t prefixlen;
  struct in6_addr next_hop;
  uint8_t ifindex;
};
# 41 "/opt/tinyos/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4299 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4300 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4301 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4302 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 12 "/opt/tinyos/tos/platforms/epic/chips/ds2411/DallasId48.h"
enum __nesc_unnamed4303 {
  DALLASID48_SERIAL_LENGTH = 6, 
  DALLASID48_DATA_LENGTH = 8
};








#line 17
typedef union dallasid48_serial_t {
  uint8_t data[DALLASID48_DATA_LENGTH];
  struct  {
    uint8_t family_code;
    uint8_t serial[DALLASID48_SERIAL_LENGTH];
    uint8_t crc;
  } ;
} dallasid48_serial_t;




static inline bool dallasid48checkCrc(const dallasid48_serial_t *id);
# 29 "/opt/tinyos/tos/platforms/epic/chips/ds2411/PlatformIeeeEui64.h"
enum __nesc_unnamed4304 {
  IEEE_EUI64_COMPANY_ID_0 = 0x00, 
  IEEE_EUI64_COMPANY_ID_1 = 0x12, 
  IEEE_EUI64_COMPANY_ID_2 = 0x6d, 
  IEEE_EUI64_SERIAL_ID_0 = 'E', 
  IEEE_EUI64_SERIAL_ID_1 = 'P'
};
# 56 "/opt/tinyos/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4305 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4306 {
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
typedef struct __nesc_unnamed4307 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4308 {
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
typedef struct __nesc_unnamed4309 {
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
typedef struct __nesc_unnamed4310 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4311 {
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
typedef enum __nesc_unnamed4312 {

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
typedef struct __nesc_unnamed4313 {
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
typedef struct __nesc_unnamed4314 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4315 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4316 {
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
typedef struct __nesc_unnamed4317 {
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
typedef struct __nesc_unnamed4318 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4319 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4320 {
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

enum __nesc_unnamed4321 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4322 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 33 "/opt/tinyos/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 59 "/opt/tinyos/tos/lib/net/blip/BlipStatistics.h"
#line 40
typedef nx_struct __nesc_unnamed4323 {
  nx_uint16_t sent;
  nx_uint16_t forwarded;
  nx_uint8_t rx_drop;
  nx_uint8_t tx_drop;
  nx_uint8_t fw_drop;
  nx_uint8_t rx_total;
  nx_uint8_t encfail;
} __attribute__((packed)) 
#line 59
ip_statistics_t;







#line 62
typedef nx_struct __nesc_unnamed4324 {
  nx_uint8_t hop_limit;
  nx_uint16_t parent;
  nx_uint16_t parent_metric;
  nx_uint16_t parent_etx;
} __attribute__((packed)) route_statistics_t;










#line 69
typedef nx_struct __nesc_unnamed4325 {
  nx_uint8_t sol_rx;
  nx_uint8_t sol_tx;
  nx_uint8_t adv_rx;
  nx_uint8_t adv_tx;
  nx_uint8_t echo_rx;
  nx_uint8_t echo_tx;
  nx_uint8_t unk_rx;
  nx_uint16_t rx;
} __attribute__((packed)) icmp_statistics_t;






#line 81
typedef nx_struct __nesc_unnamed4326 {
  nx_uint16_t sent;
  nx_uint16_t rcvd;
  nx_uint16_t cksum;
} __attribute__((packed)) udp_statistics_t;
# 36 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/in_cksum.h"
uint16_t msg_cksum(const struct ip6_hdr *iph, 
struct ip_iovec *data, 
uint8_t nxt_hdr);
# 36 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/ip_malloc.h"
typedef uint16_t bndrt_t;

void ip_malloc_init();
void *ip_malloc(uint16_t sz);
void ip_free(void *ptr);
# 31 "/opt/tinyos/tos/lib/net/blip/table.h"
#line 27
typedef struct __nesc_unnamed4327 {
  void *data;
  uint16_t elt_len;
  uint16_t n_elts;
} table_t;

void table_init(table_t *table, void *data, uint16_t elt_len, uint16_t n_elts);

void table_map(table_t *table, void (*fn)(void *arg_0x40e99668));
# 43 "/opt/tinyos/tos/chips/cc2520/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 45
typedef nx_struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} __attribute__((packed)) timesync_footer_t;
# 25 "/opt/tinyos/tos/lib/net/blip/icmp/icmp6.h"
enum __nesc_unnamed4328 {
  ICMP_EXT_TYPE_PREFIX = 3, 
  ICMP_EXT_TYPE_BEACON = 17
};


enum __nesc_unnamed4329 {

  TRICKLE_JITTER = 10240, 

  TRICKLE_PERIOD = 4096, 


  TRICKLE_MAX = TRICKLE_PERIOD << 5
};
#line 60
#line 54
typedef nx_struct icmp6_echo_hdr {
  nx_uint8_t type;
  nx_uint8_t code;
  nx_uint16_t cksum;
  nx_uint16_t ident;
  nx_uint16_t seqno;
} __attribute__((packed)) icmp_echo_hdr_t;
#line 72
#line 62
typedef nx_struct radv {
  nx_uint8_t type;
  nx_uint8_t code;
  nx_uint16_t cksum;
  nx_uint8_t hlim;
  nx_uint8_t flags;
  nx_uint16_t lifetime;
  nx_uint32_t reachable_time;
  nx_uint32_t retrans_time;
  nx_uint8_t options[0];
} __attribute__((packed)) radv_t;






#line 74
typedef nx_struct rsol {
  nx_uint8_t type;
  nx_uint8_t code;
  nx_uint16_t cksum;
  nx_uint32_t reserved;
} __attribute__((packed)) rsol_t;










#line 81
typedef nx_struct rpfx {
  nx_uint8_t type;
  nx_uint8_t length;
  nx_uint8_t pfx_len;
  nx_uint8_t flags;
  nx_uint32_t valid_lifetime;
  nx_uint32_t preferred_lifetime;
  nx_uint32_t reserved;
  nx_uint8_t prefix[16];
} __attribute__((packed)) pfx_t;







#line 92
typedef nx_struct __nesc_unnamed4330 {
  nx_uint8_t type;
  nx_uint8_t length;
  nx_uint16_t metric;
  nx_uint16_t seqno;
  nx_uint8_t pad[2];
} __attribute__((packed)) rqual_t;

struct icmp_stats {
  uint16_t seq;
  uint8_t ttl;
  uint32_t rtt;
};
# 103 "/opt/tinyos/tos/lib/net/rpl/RPL.h"
enum __nesc_unnamed4331 {
  RPL_DODAG_METRIC_CONTAINER_TYPE = 2, 
  RPL_DST_PREFIX_TYPE = 3, 
  RPL_DODAG_CONFIG_TYPE = 4, 
  RPL_TARGET_TYPE = 5, 
  RPL_TRANSIT_INFORMATION_TYPE = 6, 
  RPL_MOP_No_Downward = 0, 
  RPL_MOP_No_Storing = 1, 
  RPL_MOP_Storing_No_Multicast = 2, 
  RPL_MOP_Storing_With_Multicast = 3, 

  RPL_DIO_TYPE_METRIC = 2, 
  RPL_DIO_TYPE_ROUTING = 3, 
  RPL_DIO_TYPE_DODAG = 4, 
  RPL_DIO_TYPE_PREFIX = 8, 

  RPL_ROUTE_METRIC_ETX = 7, 

  RPLOF_OCP_OF0 = 0, 
  RPLOF_OCP_MRHOF = 1, 
  RPLOF_OPTION_SOLICITATION = 7
};

enum __nesc_unnamed4332 {
  RPL_IFACE = ROUTE_IFACE_154, 
  RPL_HBH_RANK_TYPE = 0x6b
};

struct icmpv6_header_t {
  uint8_t type;
  uint8_t code;
  nx_uint16_t checksum;
} __attribute((packed)) ;

struct dis_base_t {
  struct icmpv6_header_t icmpv6;
  nx_uint16_t reserved;
};

struct rpl_instance_id {

  uint8_t id;
} __attribute((packed)) ;

struct transit_info_option_t {
  uint8_t type;
  uint8_t option_length;
  uint8_t path_sequence;
  uint8_t path_control;
  uint32_t path_lifetime;
  struct in6_addr parent_address;
};

struct target_option_t {
  uint8_t type;
  uint8_t option_length;
  uint8_t reserved;
  uint8_t prefix_length;
  struct in6_addr target_prefix;
};

struct dao_base_t {
  struct icmpv6_header_t icmpv6;
  struct rpl_instance_id instance_id;
  uint16_t k_bit : 1;
  uint16_t d_bit : 1;
  uint16_t flags : 6;
  uint16_t reserved : 8;
  uint8_t DAOsequence;
  struct in6_addr dodagID;
  struct target_option_t target_option;
  struct transit_info_option_t transit_info_option;
} __attribute((packed)) ;

struct dio_base_t {
  struct icmpv6_header_t icmpv6;
  struct rpl_instance_id instance_id;
  nx_uint8_t version;
  nx_uint16_t dagRank;
  uint8_t flags;
  uint8_t dtsn;
  nx_uint16_t reserved;
  struct in6_addr dodagID;
} __attribute((packed)) ;

struct dio_body_t {
  uint8_t type;
  uint8_t container_len;
};


struct dio_dodag_config_t {
  nx_uint8_t type;
  nx_uint8_t length;
  uint8_t flags : 4;
  uint8_t A : 1;
  uint8_t PCS : 3;
  nx_uint8_t DIOIntDoubl;
  nx_uint8_t DIOIntMin;
  nx_uint8_t DIORedun;
  nx_uint16_t MaxRankInc;
  nx_uint16_t MinHopRankInc;
  nx_uint16_t ocp;
  nx_uint8_t reserved;
  nx_uint8_t default_lifetime;
  nx_uint16_t lifetime_unit;
};

struct dio_metric_header_t {
  uint8_t routing_obj_type;
  uint8_t reserved : 2;
  uint8_t R_flag : 1;
  uint8_t G_flag : 1;
  uint8_t A_flag : 2;
  uint8_t O_flag : 1;
  uint8_t C_flag : 1;
  nx_uint16_t object_len;
};

struct dio_etx_t {
  nx_uint16_t etx;
};

struct dio_latency_t {
  float latency;
};

struct dio_prefix_t {
  uint8_t type;
  nx_uint16_t suboption_len;
  uint8_t reserved : 3;
  uint8_t preference : 2;
  uint8_t reserved2 : 3;
  nx_uint32_t lifetime;
  uint8_t prefix_len;
  struct in6_addr prefix;
};

struct rpl_route {
  uint8_t next_header;
  uint8_t hdr_ext_len;
  uint8_t routing_type;
  uint8_t segments_left;
  uint8_t compr : 4;
  uint8_t pad : 4;
  uint8_t reserved;
  uint16_t reserved1;
  struct in6_addr addr[10];
};


uint16_t ROOT_RANK = 1;
enum __nesc_unnamed4333 {
  BASE_RANK = 0, 
  INFINITE_RANK = 0xFFFF, 
  RPL_DEFAULT_INSTANCE = 0, 
  NUMBER_OF_PARENTS = 10, 
  DIS_INTERVAL = 3 * 1024U, 
  DEFAULT_LIFETIME = 1024L * 60 * 20
};


enum __nesc_unnamed4334 {
  ICMPV6_TYPE = 58
};

enum __nesc_unnamed4335 {
  ICMPV6_CODE_DIS = 0x00, 
  ICMPV6_CODE_DIO = 0x01, 
  ICMPV6_CODE_DAO = 0x02
};

enum __nesc_unnamed4336 {
  DIO_BASE_FLAG_GRD = 0, 
  DIO_BASE_FLAG_DA_TRIGGER = 1, 
  DIO_BASE_FLAG_DA_SUPPORT = 2, 
  DIO_BASE_FLAG_PREF_5 = 5, 
  DIO_BASE_FLAG_PREF_6 = 6, 
  DIO_BASE_FLAG_PREF_7 = 7
};

enum __nesc_unnamed4337 {
  DIO_BASE_OPT_PAD1 = 0, 
  DIO_BASE_OPT_PADN = 1, 
  DIO_BASE_OPT_DAG_METRIC = 2, 
  DIO_BASE_OPT_DST_PREFIX = 3, 
  DIO_BASE_OPT_DAG_TIMER_CONFIG = 4
};






#line 294
typedef struct __nesc_unnamed4338 {
  struct in6_addr next_hop;
  uint8_t *data;
} rpl_data_packet_t;





#line 299
typedef struct __nesc_unnamed4339 {
  struct ip6_hdr iphdr;
  uint8_t retries;
  rpl_data_packet_t packet;
} queue_entry_t;





#line 305
typedef struct __nesc_unnamed4340 {
  struct ip6_packet s_pkt;
  struct dao_base_t dao_base;
  struct ip_iovec v[1];
} dao_entry_t;
#line 322
#line 311
typedef struct __nesc_unnamed4341 {
  struct in6_addr nodeID;
  uint8_t interfaceID;
  uint8_t DAOsequence;

  uint32_t DAOlifetime;
  uint8_t routeTag;
  uint8_t RRlength;
  uint8_t prefixLength;
  struct in6_addr prefix;
  uint8_t *RRStack;
} dao_table_entry;






#line 324
typedef struct __nesc_unnamed4342 {
  struct in6_addr nodeID;
  uint16_t successTx;
  uint16_t totalTx;
  uint16_t etx;
} parentTableEntryDAO;




#line 331
typedef struct __nesc_unnamed4343 {
  route_key_t key;
  uint32_t lifetime;
} downwards_table_t;


nx_struct nx_ip6_ext {
  nx_uint8_t ip6e_nxt;
  nx_uint8_t ip6e_len;
} __attribute__((packed));









#line 343
typedef nx_struct __nesc_unnamed4344 {
  nx_struct nx_ip6_ext ip6_ext_outer;
  nx_struct nx_ip6_ext ip6_ext_inner;
  nx_uint8_t bitflag;

  nx_uint8_t instance_id;
  nx_uint16_t senderRank;
} __attribute__((packed)) __attribute((packed))  rpl_data_hdr_t;
#line 372
#line 363
typedef struct __nesc_unnamed4345 {
  struct in6_addr parentIP;
  uint16_t rank;


  uint16_t etx;
  uint16_t etx_hop;

  bool valid;
} parent_t;

struct dio_dest_prefix_t {
  uint8_t type;
  uint16_t length;
  uint8_t *data;
};
# 61 "/opt/tinyos/tos/lib/net/coap/tinyos_net.h"
int coap_read(coap_context_t *ctx, 
struct sockaddr_in6 *src, void *buf, 
uint16_t bytes_read, struct ip6_metadata *meta);
# 31 "/opt/tinyos/support/sdk/c/coap/subscribe.h"
typedef unsigned long coap_key_t;
#line 67
#line 36
typedef struct __nesc_unnamed4346 {
  coap_uri_t *uri;
  str *name;
  unsigned char mediatype;
  unsigned int dirty : 1;
  unsigned int writable : 1;
  unsigned int splitphase : 1;
  unsigned int immediately;



  unsigned char etag[4];

  unsigned int maxage;
#line 66
  int (*data)(coap_uri_t *uri, unsigned short *tid, unsigned char *mediatype, unsigned int offset, unsigned char *buf, unsigned int *buflen, int *finished, unsigned int method);
} coap_resource_t;









#line 69
typedef struct __nesc_unnamed4347 {
  coap_key_t resource;




  struct sockaddr_in6 subscriber;
  str token;
} coap_subscription_t;
#line 93
coap_key_t coap_add_resource(coap_context_t *context, coap_resource_t *arg_0x415c1010);
#line 161
coap_resource_t *coap_get_resource(coap_context_t *ctx, coap_uri_t *uri);
# 42 "/opt/tinyos/support/sdk/c/coap/encode.h"
extern int coap_fls(unsigned int i);
#line 55
unsigned int coap_decode_var_bytes(unsigned char *buf, unsigned int len);
typedef struct in6_iid IPForwardingEngineP__Pool__t;
typedef TMicro OneWireMasterC__BusyWait__precision_tag;
typedef uint16_t OneWireMasterC__BusyWait__size_type;
typedef TMicro /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag;
typedef uint16_t /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__precision_tag /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__precision_tag;
typedef /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type;
typedef TMicro /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4348 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4349 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4350 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4351 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4352 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4353 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
typedef struct send_info IPDispatchP__SendInfoPool__t;
typedef ip_statistics_t IPDispatchP__BlipStatistics__stat_str;
typedef struct send_entry *IPDispatchP__SendQueue__t;
typedef TMilli IPDispatchP__ExpireTimer__precision_tag;
typedef message_t IPDispatchP__FragPool__t;
typedef struct send_entry IPDispatchP__SendEntryPool__t;
typedef T32khz CC2420TransmitP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420TransmitP__PacketTimeStamp__size_type;
typedef T32khz CC2420TransmitP__BackoffTimer__precision_tag;
typedef uint32_t CC2420TransmitP__BackoffTimer__size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4354 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420ReceiveP__PacketTimeStamp__size_type;
typedef T32khz CC2420PacketP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStamp32khz__size_type;
typedef T32khz CC2420PacketP__LocalTime32khz__precision_tag;
typedef TMilli CC2420PacketP__LocalTimeMilli__precision_tag;
typedef TMilli CC2420PacketP__PacketTimeStampMilli__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStampMilli__size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4355 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4356 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
enum CC2420TinyosNetworkC____nesc_unnamed4357 {
  CC2420TinyosNetworkC__TINYOS_N_NETWORKS = 0U
};
typedef TMilli PacketLinkP__DelayTimer__precision_tag;
typedef message_t /*IPDispatchC.FragPool*/PoolC__0__pool_t;
typedef /*IPDispatchC.FragPool*/PoolC__0__pool_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t;
typedef /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__t;
typedef struct send_entry /*IPDispatchC.SendEntryPool*/PoolC__1__pool_t;
typedef /*IPDispatchC.SendEntryPool*/PoolC__1__pool_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t;
typedef /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__t;
typedef struct send_entry */*IPDispatchC.QueueC*/QueueC__0__queue_t;
typedef /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__t;
typedef struct send_info /*IPDispatchC.SendInfoPool*/PoolC__2__pool_t;
typedef /*IPDispatchC.SendInfoPool*/PoolC__2__pool_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t;
typedef /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__t;
typedef struct in6_iid /*IPStackC.FwdAddrPoolC*/PoolC__3__pool_t;
typedef /*IPStackC.FwdAddrPoolC*/PoolC__3__pool_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t;
typedef /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__t;
typedef TMilli /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__precision_tag;
typedef TMilli /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__precision_tag;
typedef TMilli /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IncreaseVersionTimer__precision_tag;
typedef dao_entry_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__t;
typedef TMilli /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__precision_tag;
typedef dao_entry_t */*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__t;
typedef TMilli /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__precision_tag;
typedef TMilli /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__precision_tag;
typedef dao_entry_t */*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t;
typedef /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__t;
typedef dao_entry_t /*RPLDAORoutingEngineC.SendPoolP*/PoolC__4__pool_t;
typedef /*RPLDAORoutingEngineC.SendPoolP*/PoolC__4__pool_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t;
typedef /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__t;
typedef udp_statistics_t UdpP__BlipStatistics__stat_str;
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
uint8_t arg_0x4064e4b0);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4064e4b0);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
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
#line 68
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 42
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 44
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
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
#line 47
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );










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
uint8_t arg_0x40599108);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40599108);
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
#line 100
static void LedsP__Leds__led2Toggle(void );
#line 117
static uint8_t LedsP__Leds__get(void );
#line 77
static void LedsP__Leds__led1Off(void );
#line 94
static void LedsP__Leds__led2Off(void );
#line 134
static void LedsP__Leds__set(uint8_t val);
#line 56
static void LedsP__Leds__led0On(void );
#line 89
static void LedsP__Leds__led2On(void );
# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void );
#line 53
static void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr(void );
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




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );




static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 73
static bool /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get(void );
#line 66
static uint8_t /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );




static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void );


static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );

static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void );


static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void );


static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
static void CoapBlipP__Boot__booted(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void CoapBlipP__RadioControl__startDone(error_t error);
#line 138
static void CoapBlipP__RadioControl__stopDone(error_t error);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t CoapBlipP__Init__init(void );
# 39 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
static coap_tid_t LibCoapAdapterP__LibCoapServer__send(coap_context_t *ctx, 
struct sockaddr_in6 *dst, 
coap_pdu_t *pdu, 
int free_pdu);




static error_t LibCoapAdapterP__LibCoapServer__bind(uint16_t port);
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
static void LibCoapAdapterP__UDPServer__recvfrom(struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip6_metadata *meta);
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void IPProtocolsP__SubIP__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
#line 17
static error_t IPProtocolsP__IP__send(
# 9 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
uint8_t arg_0x40905ed0, 
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_packet *msg);





static void IPProtocolsP__IP__default__recv(
# 9 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
uint8_t arg_0x40905ed0, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
static bool IPForwardingEngineP__ForwardingEvents__default__approve(
# 22 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40925e40, 
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 13
static bool IPForwardingEngineP__ForwardingEvents__default__initiate(
# 22 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40925e40, 
# 13 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 39
static void IPForwardingEngineP__ForwardingEvents__default__linkResult(
# 22 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40925e40, 
# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
struct in6_addr *dest, struct send_info *info);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
static error_t IPForwardingEngineP__IPForward__default__send(
# 28 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40922068, 
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
struct in6_addr *next_hop, 
struct ip6_packet *msg, 
void *data);







static void IPForwardingEngineP__IPForward__recv(
# 28 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40922068, 
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta);
#line 22
static void IPForwardingEngineP__IPForward__sendDone(
# 28 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40922068, 
# 22 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
struct send_info *status);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void IPForwardingEngineP__defaultRouteAddedTask__runTask(void );
# 43 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTableEvents.nc"
static void IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteAdded(void );






static void IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteRemoved(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void IPForwardingEngineP__IPRaw__default__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void IPForwardingEngineP__IPAddress__changed(bool valid);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
static struct route_entry *IPForwardingEngineP__ForwardingTable__lookupRoute(const uint8_t *prefix, int prefix_len_bits);
#line 16
static error_t IPForwardingEngineP__ForwardingTable__delRoute(route_key_t key);
#line 10
static route_key_t IPForwardingEngineP__ForwardingTable__addRoute(const uint8_t *prefix, int prefix_len_bits, 
struct in6_addr *next_hop, uint8_t ifindex);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t IPForwardingEngineP__IP__send(struct ip6_packet *msg);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
static error_t IPNeighborDiscoveryP__IPForward__send(struct in6_addr *next_hop, 
struct ip6_packet *msg, 
void *data);
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
static void IPNeighborDiscoveryP__IPLower__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta);
#line 22
static void IPNeighborDiscoveryP__IPLower__sendDone(struct send_info *status);
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void IPNeighborDiscoveryP__IPAddress__changed(bool valid);
# 11 "/opt/tinyos/tos/lib/net/blip/interfaces/NeighborDiscovery.nc"
static error_t IPNeighborDiscoveryP__NeighborDiscovery__resolveAddress(struct in6_addr *addr, ieee154_addr_t *link_addr);




static int IPNeighborDiscoveryP__NeighborDiscovery__matchContext(struct in6_addr *addr, uint8_t *ctx);
static int IPNeighborDiscoveryP__NeighborDiscovery__getContext(uint8_t context, struct in6_addr *ctx);
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool IPAddressP__IPAddress__getLLAddr(struct in6_addr *addr);
#line 44
static bool IPAddressP__IPAddress__isLocalAddress(struct in6_addr *addr);
#line 34
static bool IPAddressP__IPAddress__getGlobalAddr(struct in6_addr *addr);




static bool IPAddressP__IPAddress__setSource(struct ip6_hdr *hdr);
#line 52
static error_t IPAddressP__IPAddress__setAddress(struct in6_addr *addr);
#line 50
static bool IPAddressP__IPAddress__isLLAddress(struct in6_addr *addr);
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
static void Ieee154AddressP__CC2420Config__syncDone(error_t error);
# 5 "/opt/tinyos/tos/lib/net/blip/interfaces/Ieee154Address.nc"
static ieee154_panid_t Ieee154AddressP__Ieee154Address__getPanId(void );

static ieee154_laddr_t Ieee154AddressP__Ieee154Address__getExtAddr(void );
#line 6
static ieee154_saddr_t Ieee154AddressP__Ieee154Address__getShortAddr(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t Ieee154AddressP__Init__init(void );
# 12 "/opt/tinyos/tos/platforms/epic/chips/ds2411/ReadId48.nc"
static error_t Ds2411P__ReadId48__read(uint8_t *id);
# 10 "/opt/tinyos/tos/platforms/epic/chips/ds2411/OneWireStream.nc"
static error_t OneWireMasterC__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len);
# 66 "/opt/tinyos/tos/lib/timer/BusyWait.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__size_type dt);
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__get(void );
# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeInput(void );
#line 43
static bool /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__get(void );


static void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 41
static void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__clr(void );
# 48 "/opt/tinyos/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void );
# 93 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
static bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 117
static bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );
#line 112
static bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );
#line 66
static ieee_eui64_t CC2420ControlP__CC2420Config__getExtAddr(void );




static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 54
static error_t CC2420ControlP__CC2420Config__sync(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__default__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 102
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420ControlP__CC2420Power__stopVReg(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 88
static error_t CC2420ControlP__Resource__request(void );
# 68 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 73
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 64
static /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 103
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 66
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__GeneralIO__get(void );
#line 43
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__makeInput(void );
#line 43
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
#line 40
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__clr(void );
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 54 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 66
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 52 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 47
static void HplMsp430InterruptP__Port14__disable(void );
#line 67
static void HplMsp430InterruptP__Port14__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port14__enable(void );









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
#line 67
static void HplMsp430InterruptP__Port10__edge(bool low_to_high);
#line 42
static void HplMsp430InterruptP__Port10__enable(void );









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
#line 72
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 61 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 61 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 54
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 82 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 75
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 62 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 91 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 82 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 51 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 71 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__abortRelease(void );







static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint16_t arg_0x40c03a38, 
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 48 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c02200, 
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c02200, 
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
uint16_t data);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c04558);
# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c04558);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c04558);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c04558);
# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c04558);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c029b8);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 71 "/opt/tinyos/tos/interfaces/State.nc"
static uint8_t StateImplP__State__getState(
# 67 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x40c67030);
# 56 "/opt/tinyos/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x40c67030);
# 66 "/opt/tinyos/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x40c67030, 
# 66 "/opt/tinyos/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x40c67030);
# 45 "/opt/tinyos/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x40c67030, 
# 45 "/opt/tinyos/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t arg_0x40c67030, 
# 51 "/opt/tinyos/tos/interfaces/State.nc"
uint8_t reqState);
# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(
# 76 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9b068);
# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(
# 76 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9b068);
# 70 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(
# 79 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a1b8, 
# 59 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 82
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(
# 79 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a1b8, 
# 75 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(
# 82 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c99418);
# 45 "/opt/tinyos/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(
# 75 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40ca05e0);
# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(
# 75 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40ca05e0);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(
# 75 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40ca05e0);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(
# 75 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40ca05e0);
# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(
# 75 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40ca05e0);
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );
# 180 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 197
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 177
static void HplMsp430Usart0P__Usart__disableRxIntr(void );
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
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40d6f0c0, 
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40d6f0c0);
# 39 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40d6f940);
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 79 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 53 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da9010);
# 61 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da9010);
# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da8430);
# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da8430);
# 56 "/opt/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40daa520);
# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40daa520);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40daa520);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40daa520);
# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40daa520);
# 90 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 7 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 55 "/opt/tinyos/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/opt/tinyos/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 100 "/opt/tinyos/tos/interfaces/Leds.nc"
static void NoLedsC__Leds__led2Toggle(void );
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t IPDispatchP__SplitControl__start(void );
# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
static void IPDispatchP__Boot__booted(void );
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/BlipStatistics.nc"
static void IPDispatchP__BlipStatistics__clear(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void IPDispatchP__RadioControl__startDone(error_t error);
#line 138
static void IPDispatchP__RadioControl__stopDone(error_t error);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
static error_t IPDispatchP__IPLower__send(struct ieee154_frame_addr *next_hop, 
struct ip6_packet *msg, 
void *data);
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void IPDispatchP__ExpireTimer__fired(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t IPDispatchP__Init__init(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void IPDispatchP__sendTask__runTask(void );
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void IPDispatchP__Ieee154Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



IPDispatchP__Ieee154Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t CC2420CsmaP__SplitControl__start(void );
#line 130
static error_t CC2420CsmaP__SplitControl__stop(void );
# 95 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__default__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(message_t * msg);
#line 81
static void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 73 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Transmit.nc"
static void CC2420CsmaP__CC2420Transmit__sendDone(message_t * p_msg, error_t error);
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t CC2420CsmaP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 76 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
static void CC2420CsmaP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420CsmaP__CC2420Power__startVRegDone(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420CsmaP__Resource__granted(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__sendDone_task__runTask(void );
#line 75
static void CC2420CsmaP__stopDone_task__runTask(void );
#line 75
static void CC2420CsmaP__startDone_task__runTask(void );
# 66 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);
# 61 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__BackoffTimer__fired(void );
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t * message);
# 51 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
# 24 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP__StdControl__start(void );









static error_t CC2420TransmitP__StdControl__stop(void );
# 91 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP__receiveDone_task__runTask(void );
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 68 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );









static error_t CC2420ReceiveP__StdControl__stop(void );
# 64 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Packet.nc"
static int8_t CC2420PacketP__CC2420Packet__getRssi(message_t *p_msg);










static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t * p_msg);
#line 72
static uint8_t CC2420PacketP__CC2420Packet__getLqi(message_t *p_msg);
# 70 "/opt/tinyos/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(
#line 66
message_t * msg);
#line 78
static void CC2420PacketP__PacketTimeStamp32khz__set(
#line 73
message_t * msg, 




CC2420PacketP__PacketTimeStamp32khz__size_type value);
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420PacketP__CC2420PacketBody__getMetadata(message_t * msg);
# 58 "/opt/tinyos/tos/chips/cc2520/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP__PacketTimeSyncOffset__get(
#line 53
message_t * msg);
#line 50
static bool CC2420PacketP__PacketTimeSyncOffset__isSet(
#line 46
message_t * msg);
# 59 "/opt/tinyos/tos/interfaces/PacketAcknowledgements.nc"
static error_t CC2420PacketP__Acks__requestAck(
#line 53
message_t * msg);
#line 85
static bool CC2420PacketP__Acks__wasAcked(
#line 80
message_t * msg);
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 64
static /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 116
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
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
#line 136
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x412069f0);
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x412069f0);
# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x412069f0);
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x412069f0, 
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x412069f0, 
# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 48 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x412069f0);
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 52 "/opt/tinyos/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 46
static uint32_t RandomMlcgC__Random__rand32(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void UniqueSendP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
#line 75
static error_t UniqueSendP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t UniqueSendP__Init__init(void );
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t UniqueReceiveP__Init__init(void );
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__DuplicateReceive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__SubReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP__grantTask__runTask(void );
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__default__sendDone(
#line 96
message_t * msg, 



error_t error);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__release(
# 46 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x412b4e60);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__default__granted(
# 46 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x412b4e60);
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__BareSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 125
static 
#line 123
void * 

CC2420TinyosNetworkP__BareSend__getPayload(
#line 122
message_t * msg, 


uint8_t len);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__ActiveReceive__default__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 65 "/opt/tinyos/tos/interfaces/Packet.nc"
static void CC2420TinyosNetworkP__BarePacket__clear(
#line 62
message_t * msg);
#line 78
static uint8_t CC2420TinyosNetworkP__BarePacket__payloadLength(
#line 74
message_t * msg);
#line 106
static uint8_t CC2420TinyosNetworkP__BarePacket__maxPayloadLength(void );
#line 94
static void CC2420TinyosNetworkP__BarePacket__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );
# 53 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );
#line 70
static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void PacketLinkP__SubSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void PacketLinkP__send__runTask(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void PacketLinkP__DelayTimer__fired(void );
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t PacketLinkP__Send__send(
#line 67
message_t * msg, 







uint8_t len);
# 65 "/opt/tinyos/tos/interfaces/PacketLink.nc"
static uint16_t PacketLinkP__PacketLink__getRetryDelay(
#line 62
message_t * msg);
#line 46
static void PacketLinkP__PacketLink__setRetries(
#line 42
message_t * msg, 



uint16_t maxRetries);
#line 59
static uint16_t PacketLinkP__PacketLink__getRetries(
#line 56
message_t * msg);
#line 53
static void PacketLinkP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay);
#line 71
static bool PacketLinkP__PacketLink__wasDelivered(
#line 68
message_t * msg);
# 8 "/opt/tinyos/tos/lib/net/blip/interfaces/ReadLqi.nc"
static uint8_t CC2420ReadLqiC__ReadLqi__readRssi(message_t *msg);
#line 6
static uint8_t CC2420ReadLqiC__ReadLqi__readLqi(message_t *msg);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
/*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__t * 


/*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get(void );
#line 89
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(
#line 85
/*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__t * newVal);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init(void );
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__t * 


/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get(void );
#line 89
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(
#line 85
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__t * newVal);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init(void );
# 73 "/opt/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
/*IPDispatchC.QueueC*/QueueC__0__Queue__t  

/*IPDispatchC.QueueC*/QueueC__0__Queue__head(void );
#line 90
static error_t /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(
#line 86
/*IPDispatchC.QueueC*/QueueC__0__Queue__t  newVal);
#line 65
static uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize(void );
#line 81
static 
#line 79
/*IPDispatchC.QueueC*/QueueC__0__Queue__t  

/*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 50
static bool /*IPDispatchC.QueueC*/QueueC__0__Queue__empty(void );







static uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__size(void );
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__t * 


/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get(void );
#line 89
static error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(
#line 85
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__t * newVal);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init(void );
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t IPStackControlP__SplitControl__start(void );








static void IPStackControlP__SubSplitControl__startDone(error_t error);
#line 138
static void IPStackControlP__SubSplitControl__stopDone(error_t error);
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void IPStackControlP__IPAddress__changed(bool valid);
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t IPStackControlP__StdControl__default__start(void );
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t ICMPCoreP__ICMP_IP__send(
# 50 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
uint8_t arg_0x4134ee70, 
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_packet *msg);





static void ICMPCoreP__ICMP_IP__default__recv(
# 50 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
uint8_t arg_0x4134ee70, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void ICMPCoreP__IPAddress__changed(bool valid);
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void ICMPCoreP__IP__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 14 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
static int IPPacketC__IPPacket__findHeader(struct ip_iovec *payload, 
uint8_t first_type, uint8_t *search_type);

static int IPPacketC__IPPacket__findTLV(struct ip_iovec *header, 
int ext_offset, 
uint8_t type);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__t * 


/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__get(void );
#line 89
static error_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__put(
#line 85
/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__t * newVal);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Init__init(void );
# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
static void NoDhcpC__Boot__booted(void );
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void NoDhcpC__IPAddress__changed(bool valid);
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
static bool RPLRankP__ForwardingEvents__approve(struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 13
static bool RPLRankP__ForwardingEvents__initiate(struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 39
static void RPLRankP__ForwardingEvents__linkResult(struct in6_addr *dest, struct send_info *info);
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void RPLRankP__IPAddress__changed(bool valid);
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void RPLRankP__IP_DIO__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 98 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
static uint8_t RPLRankP__RPLRankInfo__hasParent(void );
#line 84
static uint16_t RPLRankP__RPLRankInfo__getRank(struct in6_addr *node);
#line 101
static uint16_t RPLRankP__RPLRankInfo__getEtx(void );
#line 99
static bool RPLRankP__RPLRankInfo__isLeaf(void );
#line 94
static void RPLRankP__RPLRankInfo__inconsistencyDetected(void );
#line 113
static error_t RPLRankP__RPLRankInfo__getDefaultRoute(struct in6_addr *next_hop);
# 2 "/opt/tinyos/tos/lib/net/rpl/RPLParentTable.nc"
static parent_t *RPLRankP__RPLParentTable__get(uint8_t parent_index);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void RPLRankP__newParentSearch__runTask(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t RPLRankP__StdControl__start(void );









static error_t RPLRankP__StdControl__stop(void );
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t RPLRankP__IP_DIO_Filter__send(struct ip6_packet *msg);





static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__runTask(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__fired(void );
#line 83
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__fired(void );
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__runTask(void );
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__changed(bool valid);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__runTask(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__runTask(void );
#line 75
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__runTask(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IncreaseVersionTimer__fired(void );
# 52 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
static uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getMOP(void );
#line 45
static uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getInstanceID(void );







static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDTSN(uint8_t dtsn);
#line 42
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__hasDODAG(void );
#line 56
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__inconsistency(void );
#line 44
static uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getRank(void );









static uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDTSN(void );
#line 49
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDODAGConfig(uint8_t DIOIntDouble, uint8_t DIOIntMin, 
uint8_t DIORedun, uint8_t MaxRankInc, uint8_t MinHopRankInc);
#line 43
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDefaultRoute(struct in6_addr *next_hop);
#line 41
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__start(void );









static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__stop(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__runTask(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__fired(void );
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__changed(bool valid);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__runTask(void );
# 42 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngine.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__newParent(void );
#line 41
static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__getStoreState(void );
#line 40
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__startDAO(void );
# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__fired(void );
#line 83
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__fired(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__start(void );









static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__stop(void );
# 73 "/opt/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__t  

/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__head(void );
#line 90
static error_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__enqueue(
#line 86
/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__t  newVal);
#line 65
static uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__maxSize(void );
#line 81
static 
#line 79
/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__t  

/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__dequeue(void );
#line 50
static bool /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__empty(void );







static uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__size(void );
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__t * 


/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__get(void );
#line 89
static error_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__put(
#line 85
/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__t * newVal);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Init__init(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
#line 17
static error_t /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__send(
# 35 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
uint8_t arg_0x415698a8, 
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_packet *msg);





static void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__default__recv(
# 35 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
uint8_t arg_0x415698a8, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 20 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
static bool RPLOF0P__RPLOF__recomputeRoutes(void );
#line 4
static bool RPLOF0P__RPLOF__OCP(uint16_t ocp);


static bool RPLOF0P__RPLOF__objectSupported(uint16_t objectType);






static uint16_t RPLOF0P__RPLOF__getRank(void );
static void RPLOF0P__RPLOF__resetRank(void );

static bool RPLOF0P__RPLOF__recalcualateRank(void );




static void RPLOF0P__RPLOF__setMinHopRankIncrease(uint16_t val);
#line 9
static uint16_t RPLOF0P__RPLOF__getObjectValue(void );

static struct in6_addr *RPLOF0P__RPLOF__getParent(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
#line 17
static error_t /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__send(
# 35 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
uint8_t arg_0x415698a8, 
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_packet *msg);





static void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__default__recv(
# 35 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
uint8_t arg_0x415698a8, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 52 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
static void CoapUdpServerP__LibCoapServer__read(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta);
# 48 "/opt/tinyos/tos/interfaces/CoAPServer.nc"
static error_t CoapUdpServerP__CoAPServer__registerResource(char uri[5], 
unsigned int uri_length, 
unsigned char mediatype, 
unsigned int writable, 
unsigned int splitphase, 
unsigned int immediately);
#line 38
static error_t CoapUdpServerP__CoAPServer__bind(uint16_t port);




static error_t CoapUdpServerP__CoAPServer__registerWellknownCore(void );
# 35 "/opt/tinyos/tos/interfaces/WriteResource.nc"
static void CoapUdpServerP__WriteResource__putDone(
# 67 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
uint8_t arg_0x415b4978, 
# 35 "/opt/tinyos/tos/interfaces/WriteResource.nc"
error_t result, coap_tid_t id, uint8_t asyn_message);
#line 34
static int CoapUdpServerP__WriteResource__default__put(
# 67 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
uint8_t arg_0x415b4978, 
# 34 "/opt/tinyos/tos/interfaces/WriteResource.nc"
uint8_t *val, size_t buflen, coap_tid_t id);
# 33 "/opt/tinyos/tos/interfaces/ReadResource.nc"
static int CoapUdpServerP__ReadResource__default__get(
# 66 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
uint8_t arg_0x415b4110, 
# 33 "/opt/tinyos/tos/interfaces/ReadResource.nc"
coap_tid_t id);
static void CoapUdpServerP__ReadResource__getDone(
# 66 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
uint8_t arg_0x415b4110, 
# 34 "/opt/tinyos/tos/interfaces/ReadResource.nc"
error_t result, coap_tid_t id, uint8_t asyn_message, uint8_t *val, size_t buflen);
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t CoapUdpServerP__Init__init(void );
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/BlipStatistics.nc"
static void UdpP__BlipStatistics__clear(void );
# 62 "/opt/tinyos/tos/interfaces/Init.nc"
static error_t UdpP__Init__init(void );
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void UdpP__IPAddress__changed(bool valid);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
static error_t UdpP__UDP__sendto(
# 8 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x41652e90, 
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 12
static error_t UdpP__UDP__bind(
# 8 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x41652e90, 
# 12 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
uint16_t port);








static error_t UdpP__UDP__sendtov(
# 8 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x41652e90, 
# 21 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *dest, 
struct ip_iovec *iov);






static void UdpP__UDP__default__recvfrom(
# 8 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x41652e90, 
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip6_metadata *meta);
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void UdpP__IP__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 34 "/opt/tinyos/tos/interfaces/WriteResource.nc"
static int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__put(uint8_t *val, size_t buflen, coap_tid_t id);
# 33 "/opt/tinyos/tos/interfaces/ReadResource.nc"
static int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__get(coap_tid_t id);
# 75 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__runTask(void );
#line 75
static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__runTask(void );
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

enum Msp430ClockP____nesc_unnamed4358 {

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
uint8_t arg_0x4064e4b0);
# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 62 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void );
#line 126
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4064e4b0);
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


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



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
# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void );
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );
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
uint8_t arg_0x40599108);
# 76 "/opt/tinyos/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/opt/tinyos/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4359 {

  SchedulerBasicP__NUM_TASKS = 25U, 
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
# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static bool LedsP__Led0__get(void );


static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );
static void LedsP__Led0__clr(void );

static bool LedsP__Led1__get(void );


static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );
static void LedsP__Led1__clr(void );
static void LedsP__Led2__toggle(void );
static bool LedsP__Led2__get(void );


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




static inline void LedsP__Leds__led2Toggle(void );




static inline uint8_t LedsP__Leds__get(void );
#line 136
static inline void LedsP__Leds__set(uint8_t val);
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 57
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void );

static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput(void );

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
#line 59
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 56
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 56
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void );
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 73
static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__clr(void );

static inline bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 73
static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__clr(void );

static inline bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 58 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 85
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 73
static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );




static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__clr(void );
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 48 "/opt/tinyos/tos/interfaces/CoAPServer.nc"
static error_t CoapBlipP__CoAPServer__registerResource(char uri[5], 
unsigned int uri_length, 
unsigned char mediatype, 
unsigned int writable, 
unsigned int splitphase, 
unsigned int immediately);
#line 38
static error_t CoapBlipP__CoAPServer__bind(uint16_t port);




static error_t CoapBlipP__CoAPServer__registerWellknownCore(void );
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t CoapBlipP__RadioControl__start(void );
# 67 "CoapBlipP.nc"
static inline error_t CoapBlipP__Init__init(void );



static inline void CoapBlipP__Boot__booted(void );
#line 104
static inline void CoapBlipP__RadioControl__startDone(error_t e);



static inline void CoapBlipP__RadioControl__stopDone(error_t e);
# 52 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
static void LibCoapAdapterP__LibCoapServer__read(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
static error_t LibCoapAdapterP__UDPServer__sendto(struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 12
static error_t LibCoapAdapterP__UDPServer__bind(uint16_t port);
# 39 "/opt/tinyos/tos/lib/net/coap/LibCoapAdapterP.nc"
static inline void LibCoapAdapterP__libcoap_server_read(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta);



static inline void LibCoapAdapterP__UDPServer__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta);
#line 63
coap_tid_t coap_send_impl(coap_context_t *context, 
struct sockaddr_in6 *dst, 
coap_pdu_t *pdu, 
int free_pdu)   ;
#line 82
static inline coap_tid_t LibCoapAdapterP__LibCoapServer__send(coap_context_t *context, 
struct sockaddr_in6 *dst, 
coap_pdu_t *pdu, 
int free_pdu);




static inline error_t LibCoapAdapterP__LibCoapServer__bind(uint16_t port);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t IPProtocolsP__SubIP__send(struct ip6_packet *msg);
# 14 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
static int IPProtocolsP__IPPacket__findHeader(struct ip_iovec *payload, 
uint8_t first_type, uint8_t *search_type);
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void IPProtocolsP__IP__recv(
# 9 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
uint8_t arg_0x40905ed0, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 18 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
static inline void IPProtocolsP__SubIP__recv(struct ip6_hdr *iph, 
void *payload, 
size_t len, 
struct ip6_metadata *meta);
#line 48
static error_t IPProtocolsP__IP__send(uint8_t nxt_hdr, struct ip6_packet *msg);







static inline void IPProtocolsP__IP__default__recv(uint8_t nxt_hdr, struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
static bool IPForwardingEngineP__ForwardingEvents__approve(
# 22 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40925e40, 
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 13
static bool IPForwardingEngineP__ForwardingEvents__initiate(
# 22 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40925e40, 
# 13 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 39
static void IPForwardingEngineP__ForwardingEvents__linkResult(
# 22 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40925e40, 
# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
struct in6_addr *dest, struct send_info *info);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
static error_t IPForwardingEngineP__IPForward__send(
# 28 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
uint8_t arg_0x40922068, 
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
struct in6_addr *next_hop, 
struct ip6_packet *msg, 
void *data);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
IPForwardingEngineP__Pool__t * 


IPForwardingEngineP__Pool__get(void );
#line 89
static error_t IPForwardingEngineP__Pool__put(
#line 85
IPForwardingEngineP__Pool__t * newVal);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t IPForwardingEngineP__defaultRouteAddedTask__postTask(void );
# 43 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTableEvents.nc"
static void IPForwardingEngineP__ForwardingTableEvents__defaultRouteAdded(void );






static void IPForwardingEngineP__ForwardingTableEvents__defaultRouteRemoved(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void IPForwardingEngineP__IPRaw__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 44 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool IPForwardingEngineP__IPAddress__isLocalAddress(struct in6_addr *addr);





static bool IPForwardingEngineP__IPAddress__isLLAddress(struct in6_addr *addr);
# 14 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
static int IPForwardingEngineP__IPPacket__findHeader(struct ip_iovec *payload, 
uint8_t first_type, uint8_t *search_type);
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void IPForwardingEngineP__IP__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 96 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
enum IPForwardingEngineP____nesc_unnamed4360 {
#line 96
  IPForwardingEngineP__defaultRouteAddedTask = 0U
};
#line 96
typedef int IPForwardingEngineP____nesc_sillytask_defaultRouteAddedTask[IPForwardingEngineP__defaultRouteAddedTask];
#line 49
struct route_entry IPForwardingEngineP__routing_table[ROUTE_TABLE_SZ];

route_key_t IPForwardingEngineP__last_key = 1;





static inline int IPForwardingEngineP__alloc_key(void );
#line 69
static inline struct route_entry *IPForwardingEngineP__alloc_entry(int pfxlen);
#line 96
static inline void IPForwardingEngineP__defaultRouteAddedTask__runTask(void );



static route_key_t IPForwardingEngineP__ForwardingTable__addRoute(const uint8_t *prefix, 
int prefix_len_bits, 
struct in6_addr *next_hop, 
uint8_t ifindex);
#line 130
static error_t IPForwardingEngineP__ForwardingTable__delRoute(route_key_t key);
#line 154
static struct route_entry *IPForwardingEngineP__ForwardingTable__lookupRoute(const uint8_t *prefix, 
int prefix_len_bits);
#line 184
static error_t IPForwardingEngineP__do_send(uint8_t ifindex, struct in6_addr *next, struct ip6_packet *pkt);










static inline error_t IPForwardingEngineP__IP__send(struct ip6_packet *pkt);
#line 248
static inline void IPForwardingEngineP__IPForward__recv(uint8_t ifindex, struct ip6_hdr *iph, void *payload, 
struct ip6_metadata *meta);
#line 313
static void IPForwardingEngineP__IPForward__sendDone(uint8_t ifindex, struct send_info *status);
#line 344
static inline bool IPForwardingEngineP__ForwardingEvents__default__approve(uint8_t idx, struct ip6_packet *pkt, 
struct in6_addr *next_hop);


static inline bool IPForwardingEngineP__ForwardingEvents__default__initiate(uint8_t idx, struct ip6_packet *pkt, 
struct in6_addr *next_hop);


static inline void IPForwardingEngineP__ForwardingEvents__default__linkResult(uint8_t idx, struct in6_addr *host, 
struct send_info *info);

static inline error_t IPForwardingEngineP__IPForward__default__send(uint8_t ifindex, struct in6_addr *next_hop, 
struct ip6_packet *pkt, 
void *data);







static inline void IPForwardingEngineP__IPRaw__default__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);

static inline void IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteAdded(void );
static inline void IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteRemoved(void );

static inline void IPForwardingEngineP__IPAddress__changed(bool global_valid);
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
static void IPNeighborDiscoveryP__IPForward__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta);
#line 22
static void IPNeighborDiscoveryP__IPForward__sendDone(struct send_info *status);
# 5 "/opt/tinyos/tos/lib/net/blip/interfaces/Ieee154Address.nc"
static ieee154_panid_t IPNeighborDiscoveryP__Ieee154Address__getPanId(void );
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
static error_t IPNeighborDiscoveryP__IPLower__send(struct ieee154_frame_addr *next_hop, 
struct ip6_packet *msg, 
void *data);
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool IPNeighborDiscoveryP__IPAddress__getLLAddr(struct in6_addr *addr);




static bool IPNeighborDiscoveryP__IPAddress__getGlobalAddr(struct in6_addr *addr);
# 32 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static int IPNeighborDiscoveryP__NeighborDiscovery__matchContext(struct in6_addr *addr, 
uint8_t *ctx);










static inline int IPNeighborDiscoveryP__NeighborDiscovery__getContext(uint8_t context, 
struct in6_addr *ctx);
#line 58
static error_t IPNeighborDiscoveryP__NeighborDiscovery__resolveAddress(struct in6_addr *addr, 
ieee154_addr_t *link_addr);
#line 95
static error_t IPNeighborDiscoveryP__IPForward__send(struct in6_addr *next, struct ip6_packet *msg, void *ptr);
#line 124
static inline void IPNeighborDiscoveryP__IPLower__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta);



static inline void IPNeighborDiscoveryP__IPLower__sendDone(struct send_info *status);




static inline void IPNeighborDiscoveryP__IPAddress__changed(bool global_valid);
# 5 "/opt/tinyos/tos/lib/net/blip/interfaces/Ieee154Address.nc"
static ieee154_panid_t IPAddressP__Ieee154Address__getPanId(void );

static ieee154_laddr_t IPAddressP__Ieee154Address__getExtAddr(void );
#line 6
static ieee154_saddr_t IPAddressP__Ieee154Address__getShortAddr(void );
# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static void IPAddressP__IPAddress__changed(bool valid);
# 34 "/opt/tinyos/tos/lib/net/blip/IPAddressP.nc"
bool IPAddressP__m_valid_addr = FALSE;
#line 34
bool IPAddressP__m_short_addr = FALSE;
struct in6_addr IPAddressP__m_addr;

static bool IPAddressP__IPAddress__getLLAddr(struct in6_addr *addr);
#line 60
static inline bool IPAddressP__IPAddress__getGlobalAddr(struct in6_addr *addr);




static bool IPAddressP__IPAddress__setSource(struct ip6_hdr *hdr);
#line 87
static bool IPAddressP__IPAddress__isLocalAddress(struct in6_addr *addr);
#line 128
static inline bool IPAddressP__IPAddress__isLLAddress(struct in6_addr *addr);







static inline error_t IPAddressP__IPAddress__setAddress(struct in6_addr *addr);
# 48 "/opt/tinyos/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t Ieee154AddressP__LocalIeeeEui64__getId(void );
# 12 "/opt/tinyos/tos/lib/net/blip/Ieee154AddressP.nc"
ieee154_saddr_t Ieee154AddressP__m_saddr;
ieee154_panid_t Ieee154AddressP__m_panid;

static inline error_t Ieee154AddressP__Init__init(void );





static inline ieee154_panid_t Ieee154AddressP__Ieee154Address__getPanId(void );


static inline ieee154_saddr_t Ieee154AddressP__Ieee154Address__getShortAddr(void );


static ieee154_laddr_t Ieee154AddressP__Ieee154Address__getExtAddr(void );
#line 49
static inline void Ieee154AddressP__CC2420Config__syncDone(error_t err);
# 10 "/opt/tinyos/tos/platforms/epic/chips/ds2411/OneWireStream.nc"
static error_t Ds2411P__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len);
# 20 "/opt/tinyos/tos/platforms/epic/chips/ds2411/Ds2411P.nc"
bool Ds2411P__haveId = FALSE;
dallasid48_serial_t Ds2411P__ds2411id;

static inline error_t Ds2411P__readId(void );
#line 36
static inline error_t Ds2411P__ReadId48__read(uint8_t *id);
# 66 "/opt/tinyos/tos/lib/timer/BusyWait.nc"
static void OneWireMasterC__BusyWait__wait(OneWireMasterC__BusyWait__size_type dt);
# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void OneWireMasterC__Pin__makeInput(void );
#line 43
static bool OneWireMasterC__Pin__get(void );


static void OneWireMasterC__Pin__makeOutput(void );
#line 41
static void OneWireMasterC__Pin__clr(void );
# 25 "/opt/tinyos/tos/platforms/epic/chips/ds2411/OneWireMasterC.nc"
#line 18
typedef enum OneWireMasterC____nesc_unnamed4361 {
  OneWireMasterC__DELAY_5US = 5, 
  OneWireMasterC__RESET_LOW_TIME = 560, 
  OneWireMasterC__DELAY_60US = 60, 
  OneWireMasterC__PRESENCE_DETECT_LOW_TIME = 240, 
  OneWireMasterC__PRESENCE_RESET_HIGH_TIME = 480, 
  OneWireMasterC__SLOT_TIME = 65
} OneWireMasterC__onewiretimes_t;

static inline bool OneWireMasterC__reset(void );
#line 42
static inline void OneWireMasterC__writeOne(void );






static inline void OneWireMasterC__writeZero(void );






static inline bool OneWireMasterC__readBit(void );










static inline void OneWireMasterC__writeByte(uint8_t c);
#line 80
static inline uint8_t OneWireMasterC__readByte(void );










static inline error_t OneWireMasterC__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len);
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void );
# 58 "/opt/tinyos/tos/lib/timer/BusyWaitCounterC.nc"
enum /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0____nesc_unnamed4362 {

  BusyWaitCounterC__0__HALF_MAX_SIZE_TYPE = (/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type )1 << (8 * sizeof(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type ) - 1)
};

static void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(/*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__size_type dt);
#line 83
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 49 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__get(void );
#line 64
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__makeInput(void );






static void /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 73
static bool /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__get(void );
#line 53
static void /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__clr(void );

static inline bool /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__get(void );
static inline void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeInput(void );

static inline void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 12 "/opt/tinyos/tos/platforms/epic/chips/ds2411/ReadId48.nc"
static error_t DallasId48ToIeeeEui64C__ReadId48__read(uint8_t *id);
# 8 "/opt/tinyos/tos/platforms/epic/chips/ds2411/DallasId48ToIeeeEui64C.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void );
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 48 "/opt/tinyos/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t CC2420ControlP__LocalIeeeEui64__getId(void );
# 66 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 40
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 63 "/opt/tinyos/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__TXCTRL__write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 50 "/opt/tinyos/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void );




static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void );
# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 40
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 40
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 88
static error_t CC2420ControlP__SpiResource__request(void );
#line 120
static error_t CC2420ControlP__SyncResource__release(void );
#line 88
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length);
# 61 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 53
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 125 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4363 {
#line 125
  CC2420ControlP__sync = 1U
};
#line 125
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4364 {
#line 126
  CC2420ControlP__syncDone = 2U
};
#line 126
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 90
#line 84
typedef enum CC2420ControlP____nesc_unnamed4365 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

ieee_eui64_t CC2420ControlP__m_ext_addr;

bool CC2420ControlP__m_sync_busy;


bool CC2420ControlP__autoAckEnabled;


bool CC2420ControlP__hwAutoAckDefault;


bool CC2420ControlP__addressRecognition;


bool CC2420ControlP__hwAddressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );
static inline void CC2420ControlP__writeTxctrl(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 188
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static inline error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 216
static error_t CC2420ControlP__CC2420Power__stopVReg(void );







static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 268
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 298
static inline ieee_eui64_t CC2420ControlP__CC2420Config__getExtAddr(void );



static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 323
static inline error_t CC2420ControlP__CC2420Config__sync(void );
#line 355
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 382
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );






static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );









static inline void CC2420ControlP__SyncResource__granted(void );
#line 413
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 431
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 465
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 496
static void CC2420ControlP__writeMdmctrl0(void );
#line 515
static void CC2420ControlP__writeId(void );
#line 533
static inline void CC2420ControlP__writeTxctrl(void );
#line 545
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data);
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 47
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 65
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__isOverflowPending(void );
# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__overflow(void );
# 49 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 67 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__0____nesc_unnamed4366 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type ) + 0, 



  TransformCounterC__0__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 133
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 103
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 73
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 77 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0____nesc_unnamed4367 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 0, 
  TransformAlarmC__0__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 102
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 147
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 177
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__makeInput(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__makeOutput(void );
# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__HplGeneralIO__get(void );
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__GeneralIO__get(void );
# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__GeneralIO__get(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__HplGeneralIO__makeInput(void );
#line 73
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__HplGeneralIO__get(void );
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__makeInput(void );
# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void );
#line 48
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__clr(void );
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
# 68 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 61 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 92
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 49 "/opt/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 61
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
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
# 64 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0008)))  ;
#line 79
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );

static inline void HplMsp430InterruptP__Port15__default__fired(void );
static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );
static inline void HplMsp430InterruptP__Port10__enable(void );



static inline void HplMsp430InterruptP__Port14__enable(void );



static inline void HplMsp430InterruptP__Port10__disable(void );



static inline void HplMsp430InterruptP__Port14__disable(void );



static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );








static inline void HplMsp430InterruptP__Port10__edge(bool l2h);
#line 142
static inline void HplMsp430InterruptP__Port14__edge(bool l2h);
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
# 52 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 67
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 42
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 68 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 52 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 52 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 47
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 67
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 42
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 68 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 52 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 65
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 70 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 59
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 91 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c03010, 
# 71 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 45 "/opt/tinyos/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/opt/tinyos/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 97
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420SpiP__SpiResource__request(void );
#line 128
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 102
static void CC2420SpiP__Resource__granted(
# 45 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
uint8_t arg_0x40c04558);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4368 {
#line 88
  CC2420SpiP__grant = 3U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4369 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4370 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );







static inline void CC2420SpiP__ChipSpiResource__abortRelease(void );






static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline bool CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/opt/tinyos/tos/system/StateImplP.nc"
uint8_t StateImplP__state[5U];

enum StateImplP____nesc_unnamed4371 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);









static uint8_t StateImplP__State__getState(uint8_t id);
# 82 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(
# 79 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a1b8, 
# 75 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(
# 82 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c99418);
# 180 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void );
#line 197
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(bool reset);
#line 177
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void );
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
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(
# 81 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40c9a9b0);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(
# 75 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
uint8_t arg_0x40ca05e0);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void );
# 102 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4372 {
#line 102
  Msp430SpiNoDmaP__0__signalDone_task = 4U
};
#line 102
typedef int /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task];
#line 91
enum /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0____nesc_unnamed4373 {
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



static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx);
#line 173
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id);

static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void );
#line 206
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 228
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 245
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void );




static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void );

static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 99
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 92
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart0P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart0P__Usart__clrIntr(void );



static inline void HplMsp430Usart0P__Usart__disableRxIntr(void );







static inline void HplMsp430Usart0P__Usart__disableIntr(void );



static inline void HplMsp430Usart0P__Usart__enableRxIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 90 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40d6f0c0, 
# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40d6f0c0);
# 39 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40d6f940);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 49 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4374 {
#line 49
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 53 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da9010);
# 61 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da9010);
# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da8430);
# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40da8430);
# 79 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 53
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 70
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/opt/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/opt/tinyos/tos/system/ArbiterP.nc"
uint8_t arg_0x40daa520);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/opt/tinyos/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4375 {
#line 75
  ArbiterP__0__grantedTask = 5U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4376 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4377 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4378 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 93
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 111
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 153
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
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
# 97 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 62 "/opt/tinyos/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 93
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 106
static inline am_addr_t ActiveMessageAddressC__amAddress(void );
# 63 "/opt/tinyos/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led2Toggle(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void IPDispatchP__SplitControl__startDone(error_t error);
#line 138
static void IPDispatchP__SplitControl__stopDone(error_t error);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
IPDispatchP__SendInfoPool__t * 


IPDispatchP__SendInfoPool__get(void );
#line 89
static error_t IPDispatchP__SendInfoPool__put(
#line 85
IPDispatchP__SendInfoPool__t * newVal);
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static error_t IPDispatchP__RadioControl__start(void );
#line 130
static error_t IPDispatchP__RadioControl__stop(void );
# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
static void IPDispatchP__IPLower__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta);
#line 22
static void IPDispatchP__IPLower__sendDone(struct send_info *status);
# 8 "/opt/tinyos/tos/lib/net/blip/interfaces/ReadLqi.nc"
static uint8_t IPDispatchP__ReadLqi__readRssi(message_t *msg);
#line 6
static uint8_t IPDispatchP__ReadLqi__readLqi(message_t *msg);
# 73 "/opt/tinyos/tos/interfaces/Queue.nc"
static 
#line 71
IPDispatchP__SendQueue__t  

IPDispatchP__SendQueue__head(void );
#line 90
static error_t IPDispatchP__SendQueue__enqueue(
#line 86
IPDispatchP__SendQueue__t  newVal);
#line 81
static 
#line 79
IPDispatchP__SendQueue__t  

IPDispatchP__SendQueue__dequeue(void );
#line 50
static bool IPDispatchP__SendQueue__empty(void );
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void IPDispatchP__ExpireTimer__startPeriodic(uint32_t dt);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t IPDispatchP__sendTask__postTask(void );
# 16 "/opt/tinyos/tos/lib/net/blip/interfaces/NeighborDiscovery.nc"
static int IPDispatchP__NeighborDiscovery__matchContext(struct in6_addr *addr, uint8_t *ctx);
static int IPDispatchP__NeighborDiscovery__getContext(uint8_t context, struct in6_addr *ctx);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
IPDispatchP__FragPool__t * 


IPDispatchP__FragPool__get(void );
#line 89
static error_t IPDispatchP__FragPool__put(
#line 85
IPDispatchP__FragPool__t * newVal);
# 100 "/opt/tinyos/tos/interfaces/Leds.nc"
static void IPDispatchP__Leds__led2Toggle(void );
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t IPDispatchP__Ieee154Send__send(
#line 67
message_t * msg, 







uint8_t len);
#line 125
static 
#line 123
void * 

IPDispatchP__Ieee154Send__getPayload(
#line 122
message_t * msg, 


uint8_t len);
# 65 "/opt/tinyos/tos/interfaces/Packet.nc"
static void IPDispatchP__BarePacket__clear(
#line 62
message_t * msg);
#line 78
static uint8_t IPDispatchP__BarePacket__payloadLength(
#line 74
message_t * msg);
#line 106
static uint8_t IPDispatchP__BarePacket__maxPayloadLength(void );
#line 94
static void IPDispatchP__BarePacket__setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 46 "/opt/tinyos/tos/interfaces/PacketLink.nc"
static void IPDispatchP__PacketLink__setRetries(
#line 42
message_t * msg, 



uint16_t maxRetries);
#line 59
static uint16_t IPDispatchP__PacketLink__getRetries(
#line 56
message_t * msg);
#line 53
static void IPDispatchP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay);
#line 71
static bool IPDispatchP__PacketLink__wasDelivered(
#line 68
message_t * msg);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
IPDispatchP__SendEntryPool__t * 


IPDispatchP__SendEntryPool__get(void );
#line 89
static error_t IPDispatchP__SendEntryPool__put(
#line 85
IPDispatchP__SendEntryPool__t * newVal);
# 431 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
enum IPDispatchP____nesc_unnamed4379 {
#line 431
  IPDispatchP__sendTask = 6U
};
#line 431
typedef int IPDispatchP____nesc_sillytask_sendTask[IPDispatchP__sendTask];
#line 84
static inline int IPDispatchP__lowpan_extern_read_context(struct in6_addr *addr, int context);



static inline int IPDispatchP__lowpan_extern_match_context(struct in6_addr *addr, uint8_t *ctx_id);
# 15 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/internal.h"
static int IPDispatchP__bit_range_zero_p(uint8_t *buf, int start, int end);
static __inline uint8_t *IPDispatchP__pack_tcfl(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch);
static __inline uint8_t *IPDispatchP__pack_nh(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch);
static __inline uint8_t *IPDispatchP__pack_hlim(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch);
static uint8_t *IPDispatchP__pack_address(uint8_t *buf, struct in6_addr *addr, int context_match_len, 
ieee154_addr_t *l2addr, ieee154_panid_t pan, uint8_t *flags);
static inline uint8_t *IPDispatchP__pack_multicast(uint8_t *buf, struct in6_addr *addr, uint8_t *flags);
static inline int IPDispatchP__pack_udp(uint8_t *buf, size_t cnt, struct ip6_packet *packet, int offset);
static inline int IPDispatchP__pack_ipnh(uint8_t *dest, size_t cnt, uint8_t *type, struct ip6_packet *packet, int offset);
static inline int IPDispatchP__pack_nhc_chain(uint8_t **dest, size_t cnt, struct ip6_packet *packet);
static uint8_t *IPDispatchP__pack_ieee154_header(uint8_t *buf, size_t cnt, 
struct ieee154_frame_addr *frame);
static inline uint8_t *IPDispatchP__lowpan_pack_headers(struct ip6_packet *packet, 
struct ieee154_frame_addr *frame, 
uint8_t *buf, size_t cnt);


static inline uint8_t *IPDispatchP__unpack_ieee154_hdr(uint8_t *buf, struct ieee154_frame_addr *frame);
static inline uint8_t *IPDispatchP__unpack_tcfl(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf);
static inline uint8_t *IPDispatchP__unpack_nh(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf);
static inline uint8_t *IPDispatchP__unpack_hlim(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf);
static uint8_t *IPDispatchP__unpack_address(struct in6_addr *addr, uint8_t dispatch, 
int context, uint8_t *buf, 
ieee154_addr_t *frame, ieee154_panid_t pan);
static inline uint8_t *IPDispatchP__unpack_multicast(struct in6_addr *addr, uint8_t dispatch, 
int context, uint8_t *buf);
static inline uint8_t *IPDispatchP__unpack_udp(uint8_t *dest, uint8_t *nxt_hdr, uint8_t *buf);
static inline uint8_t *IPDispatchP__unpack_ipnh(uint8_t *dest, size_t cnt, uint8_t *nxt_hdr, uint8_t *buf);
static inline uint8_t *IPDispatchP__unpack_nhc_chain(struct lowpan_reconstruct *recon, 
uint8_t **dest, size_t cnt, 
uint8_t *nxt_hdr, uint8_t *buf);
# 16 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/ieee154_header.c"
static uint8_t *IPDispatchP__pack_ieee154_header(uint8_t *buf, size_t cnt, 
struct ieee154_frame_addr *frame);
#line 44
static inline uint8_t *IPDispatchP__unpack_ieee154_hdr(uint8_t *buf, struct ieee154_frame_addr *frame);
# 4 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/ieee154_header.h"
static uint8_t *IPDispatchP__pack_ieee154_header(uint8_t *buf, size_t cnt, struct ieee154_frame_addr *frame);
static inline uint8_t *IPDispatchP__unpack_ieee154_hdr(uint8_t *buf, struct ieee154_frame_addr *frame);
# 63 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static inline int IPDispatchP__iid_eui_cmp(uint8_t *iid, uint8_t *eui);
#line 76
static int IPDispatchP__bit_range_zero_p(uint8_t *buf, int start, int end);
#line 105
static __inline uint8_t *IPDispatchP__pack_tcfl(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch);
#line 136
static __inline uint8_t *IPDispatchP__pack_nh(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch);
#line 148
static __inline uint8_t *IPDispatchP__pack_hlim(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch);
#line 169
static uint8_t *IPDispatchP__pack_address(uint8_t *buf, struct in6_addr *addr, int context_match_len, 
ieee154_addr_t *l2addr, ieee154_panid_t pan, uint8_t *flags);
#line 235
static inline uint8_t *IPDispatchP__pack_multicast(uint8_t *buf, struct in6_addr *addr, uint8_t *flags);
#line 261
static inline int IPDispatchP__pack_udp(uint8_t *buf, size_t cnt, struct ip6_packet *packet, int offset);
#line 288
static inline uint8_t IPDispatchP____ipnh_real_length(uint8_t type, struct ip_iovec *pkt, int offset);
#line 325
static inline int IPDispatchP__pack_ipnh(uint8_t *dest, size_t cnt, uint8_t *type, struct ip6_packet *packet, int offset);
#line 380
static inline int IPDispatchP__pack_nhc_chain(uint8_t **dest, size_t cnt, struct ip6_packet *packet);
#line 413
static inline uint8_t *IPDispatchP__lowpan_pack_headers(struct ip6_packet *packet, 
struct ieee154_frame_addr *frame, 
uint8_t *buf, size_t cnt);
#line 465
static inline uint8_t *IPDispatchP__unpack_tcfl(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf);
#line 499
static inline uint8_t *IPDispatchP__unpack_nh(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf);








static inline uint8_t *IPDispatchP__unpack_hlim(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf);
#line 526
static uint8_t *IPDispatchP__unpack_address(struct in6_addr *addr, uint8_t dispatch, 
int context, uint8_t *buf, 
ieee154_addr_t *frame, ieee154_panid_t pan);
#line 592
static inline uint8_t *IPDispatchP__unpack_multicast(struct in6_addr *addr, uint8_t dispatch, 
int context, uint8_t *buf);
#line 624
static inline uint8_t *IPDispatchP__unpack_udp(uint8_t *dest, uint8_t *nxt_hdr, uint8_t *buf);
#line 673
static inline uint8_t *IPDispatchP__unpack_ipnh(uint8_t *dest, size_t cnt, uint8_t *nxt_hdr, uint8_t *buf);
#line 733
static inline uint8_t *IPDispatchP__unpack_nhc_chain(struct lowpan_reconstruct *recon, 
uint8_t **dest, size_t cnt, 
uint8_t *nxt_hdr, uint8_t *buf);
#line 767
static inline uint8_t *IPDispatchP__lowpan_unpack_headers(struct lowpan_reconstruct *recon, 
struct ieee154_frame_addr *frame, 
uint8_t *buf, size_t cnt);
# 48 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t *IPDispatchP__getLowpanPayload(struct packed_lowmsg *lowmsg);
#line 68
static __inline uint16_t IPDispatchP__getHeaderBitmap(struct packed_lowmsg *lowmsg);
#line 114
static __inline uint8_t IPDispatchP__setupHeaders(struct packed_lowmsg *packed, uint16_t headers);
#line 163
static __inline uint8_t IPDispatchP__hasFrag1Header(struct packed_lowmsg *msg);


static __inline uint8_t IPDispatchP__hasFragNHeader(struct packed_lowmsg *msg);
#line 255
static __inline uint8_t IPDispatchP__getFragDgramSize(struct packed_lowmsg *msg, uint16_t *size);
#line 272
static __inline uint8_t IPDispatchP__getFragDgramTag(struct packed_lowmsg *msg, uint16_t *tag);
#line 301
static __inline uint8_t IPDispatchP__setFragDgramSize(struct packed_lowmsg *msg, uint16_t size);
#line 321
static __inline uint8_t IPDispatchP__setFragDgramTag(struct packed_lowmsg *msg, uint16_t tag);
#line 337
static __inline uint8_t IPDispatchP__setFragDgramOffset(struct packed_lowmsg *msg, uint8_t size);
# 15 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_frag.c"
static int IPDispatchP__lowpan_recon_start(struct ieee154_frame_addr *frame_addr, 
struct lowpan_reconstruct *recon, 
uint8_t *pkt, size_t len);
#line 76
static inline int IPDispatchP__lowpan_recon_add(struct lowpan_reconstruct *recon, 
uint8_t *pkt, size_t len);
#line 102
static inline int IPDispatchP__lowpan_frag_get(uint8_t *frag, size_t len, 
struct ip6_packet *packet, 
struct ieee154_frame_addr *frame, 
struct lowpan_ctx *ctx);
# 100 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
enum IPDispatchP____nesc_unnamed4380 {
  IPDispatchP__S_RUNNING, 
  IPDispatchP__S_STOPPED, 
  IPDispatchP__S_STOPPING
};
uint8_t IPDispatchP__state = IPDispatchP__S_STOPPED;
bool IPDispatchP__radioBusy;
uint8_t IPDispatchP__current_local_label = 0;
ip_statistics_t IPDispatchP__stats;
#line 121
table_t IPDispatchP__recon_cache;



struct lowpan_reconstruct IPDispatchP__recon_data[N_RECONSTRUCTIONS];







static inline void IPDispatchP__reconstruct_clear(void *ent);






static inline struct send_info *IPDispatchP__getSendInfo(void );
#line 152
static void IPDispatchP__SENDINFO_DECR(struct send_info *si);





static inline error_t IPDispatchP__SplitControl__start(void );
#line 174
static inline void IPDispatchP__RadioControl__startDone(error_t error);
#line 189
static inline void IPDispatchP__RadioControl__stopDone(error_t error);



static inline error_t IPDispatchP__Init__init(void );






static inline void IPDispatchP__Boot__booted(void );
#line 213
static void IPDispatchP__deliver(struct lowpan_reconstruct *recon);
#line 251
static inline void IPDispatchP__reconstruct_age(void *elt);
#line 276
static inline void IPDispatchP__ip_print_heap(void );










static inline void IPDispatchP__ExpireTimer__fired(void );
#line 303
static inline struct lowpan_reconstruct *IPDispatchP__get_reconstruct(uint16_t key, uint16_t tag);
#line 335
static inline message_t *IPDispatchP__Ieee154Receive__receive(message_t *msg, void *msg_payload, uint8_t len);
#line 431
static inline void IPDispatchP__sendTask__runTask(void );
#line 486
static inline error_t IPDispatchP__IPLower__send(struct ieee154_frame_addr *frame_addr, 
struct ip6_packet *msg, 
void *data);
#line 582
static inline void IPDispatchP__Ieee154Send__sendDone(message_t *msg, error_t error);
#line 658
static inline void IPDispatchP__BlipStatistics__clear(void );
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void CC2420CsmaP__SplitControl__startDone(error_t error);
#line 138
static void CC2420CsmaP__SplitControl__stopDone(error_t error);
# 95 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg);
#line 66
static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime);
# 51 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Transmit.nc"
static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca);
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void CC2420CsmaP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/opt/tinyos/tos/interfaces/Random.nc"
static uint16_t CC2420CsmaP__Random__rand16(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t CC2420CsmaP__SubControl__start(void );









static error_t CC2420CsmaP__SubControl__stop(void );
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg);
# 71 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420CsmaP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420CsmaP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420CsmaP__CC2420Power__stopVReg(void );
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP__Resource__release(void );
#line 88
static error_t CC2420CsmaP__Resource__request(void );
# 66 "/opt/tinyos/tos/interfaces/State.nc"
static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState);





static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP__sendDone_task__postTask(void );
#line 67
static error_t CC2420CsmaP__stopDone_task__postTask(void );
#line 67
static error_t CC2420CsmaP__startDone_task__postTask(void );
# 74 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
enum CC2420CsmaP____nesc_unnamed4381 {
#line 74
  CC2420CsmaP__startDone_task = 7U
};
#line 74
typedef int CC2420CsmaP____nesc_sillytask_startDone_task[CC2420CsmaP__startDone_task];
enum CC2420CsmaP____nesc_unnamed4382 {
#line 75
  CC2420CsmaP__stopDone_task = 8U
};
#line 75
typedef int CC2420CsmaP____nesc_sillytask_stopDone_task[CC2420CsmaP__stopDone_task];
enum CC2420CsmaP____nesc_unnamed4383 {
#line 76
  CC2420CsmaP__sendDone_task = 9U
};
#line 76
typedef int CC2420CsmaP____nesc_sillytask_sendDone_task[CC2420CsmaP__sendDone_task];
#line 58
enum CC2420CsmaP____nesc_unnamed4384 {
  CC2420CsmaP__S_STOPPED, 
  CC2420CsmaP__S_STARTING, 
  CC2420CsmaP__S_STARTED, 
  CC2420CsmaP__S_STOPPING, 
  CC2420CsmaP__S_TRANSMITTING
};

message_t * CC2420CsmaP__m_msg;

error_t CC2420CsmaP__sendErr = SUCCESS;


bool CC2420CsmaP__ccaOn;






static inline void CC2420CsmaP__shutdown(void );


static error_t CC2420CsmaP__SplitControl__start(void );
#line 96
static inline error_t CC2420CsmaP__SplitControl__stop(void );
#line 122
static error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len);
#line 205
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP__CC2420Power__startVRegDone(void );



static inline void CC2420CsmaP__Resource__granted(void );



static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void );




static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg);
#line 244
static inline void CC2420CsmaP__sendDone_task__runTask(void );
#line 257
static inline void CC2420CsmaP__startDone_task__runTask(void );







static inline void CC2420CsmaP__stopDone_task__runTask(void );









static inline void CC2420CsmaP__shutdown(void );
#line 288
static inline void CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(message_t *msg);


static inline void CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(message_t *msg);


static inline void CC2420CsmaP__RadioBackoff__default__requestCca(message_t *msg);
# 81 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg);
# 70 "/opt/tinyos/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP__PacketTimeStamp__clear(
#line 66
message_t * msg);
#line 78
static void CC2420TransmitP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420TransmitP__PacketTimeStamp__size_type value);
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void );
# 54 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 66
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 53
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void );
#line 66
static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt);






static void CC2420TransmitP__BackoffTimer__stop(void );
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length);
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time);
# 73 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error);
# 31 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__abortRelease(void );







static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 40
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg);
# 58 "/opt/tinyos/tos/chips/cc2520/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(
#line 53
message_t * msg);
#line 50
static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(
#line 46
message_t * msg);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 97
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420TransmitP__SpiResource__request(void );
# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 43
static bool CC2420TransmitP__CCA__get(void );
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SNOP__strobe(void );
# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 43
static bool CC2420TransmitP__SFD__get(void );
# 82 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 99 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
#line 89
typedef enum CC2420TransmitP____nesc_unnamed4385 {
  CC2420TransmitP__S_STOPPED, 
  CC2420TransmitP__S_STARTED, 
  CC2420TransmitP__S_LOAD, 
  CC2420TransmitP__S_SAMPLE_CCA, 
  CC2420TransmitP__S_BEGIN_TRANSMIT, 
  CC2420TransmitP__S_SFD, 
  CC2420TransmitP__S_EFD, 
  CC2420TransmitP__S_ACK_WAIT, 
  CC2420TransmitP__S_CANCEL
} CC2420TransmitP__cc2420_transmit_state_t;





enum CC2420TransmitP____nesc_unnamed4386 {
  CC2420TransmitP__CC2420_ABORT_PERIOD = 320
};
#line 120
message_t * CC2420TransmitP__m_msg;

bool CC2420TransmitP__m_cca;

uint8_t CC2420TransmitP__m_tx_power;

CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;

bool CC2420TransmitP__m_receiving = FALSE;

uint16_t CC2420TransmitP__m_prev_time;


bool CC2420TransmitP__sfdHigh;


bool CC2420TransmitP__abortSpiRelease;


int8_t CC2420TransmitP__totalCcaChecks;


uint16_t CC2420TransmitP__myInitialBackoff;


uint16_t CC2420TransmitP__myCongestionBackoff;



static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);

static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__attemptSend(void );
static void CC2420TransmitP__congestionBackoff(void );
static error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static void CC2420TransmitP__signalDone(error_t err);



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__StdControl__start(void );










static error_t CC2420TransmitP__StdControl__stop(void );
#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
#line 243
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP__getTime32(uint16_t captured_time);
#line 280
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 377
static inline void CC2420TransmitP__ChipSpiResource__releasing(void );
#line 389
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg);
#line 416
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 454
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 486
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP__BackoffTimer__fired(void );
#line 547
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);
#line 743
static void CC2420TransmitP__attemptSend(void );
#line 788
static void CC2420TransmitP__congestionBackoff(void );






static error_t CC2420TransmitP__acquireSpiResource(void );







static inline error_t CC2420TransmitP__releaseSpiResource(void );
#line 825
static void CC2420TransmitP__loadTXFIFO(void );
#line 850
static void CC2420TransmitP__signalDone(error_t err);
# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
# 93 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void );
#line 117
static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void );
#line 112
static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void );
#line 66
static ieee_eui64_t CC2420ReceiveP__CC2420Config__getExtAddr(void );




static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void );
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP__receiveDone_task__postTask(void );
# 70 "/opt/tinyos/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP__PacketTimeStamp__clear(
#line 66
message_t * msg);
#line 78
static void CC2420ReceiveP__PacketTimeStamp__set(
#line 73
message_t * msg, 




CC2420ReceiveP__PacketTimeStamp__size_type value);
# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFOP__get(void );
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message);
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SACK__strobe(void );
# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ReceiveP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 97
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 88
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 128
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 61 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 54
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 148 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP____nesc_unnamed4387 {
#line 148
  CC2420ReceiveP__receiveDone_task = 10U
};
#line 148
typedef int CC2420ReceiveP____nesc_sillytask_receiveDone_task[CC2420ReceiveP__receiveDone_task];
#line 89
#line 81
typedef enum CC2420ReceiveP____nesc_unnamed4388 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_DEC, 
  CC2420ReceiveP__S_RX_DEC_WAIT, 
  CC2420ReceiveP__S_RX_FCF, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4389 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP__m_timestamp_head;

uint8_t CC2420ReceiveP__m_timestamp_size;





uint8_t CC2420ReceiveP__m_missed_packets;



bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;

uint8_t CC2420ReceiveP__m_bytes_left;

message_t * CC2420ReceiveP__m_p_rx_buf;

message_t CC2420ReceiveP__m_rx_buf;
#line 137
CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;



static void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );
static inline bool CC2420ReceiveP__passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP__Init__init(void );





static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 171
static error_t CC2420ReceiveP__StdControl__stop(void );
#line 186
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);








static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 212
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );
#line 513
static inline void CC2420ReceiveP__SpiResource__granted(void );
#line 530
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 668
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP__receiveDone_task__runTask(void );
#line 709
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);






static void CC2420ReceiveP__beginReceive(void );
#line 733
static void CC2420ReceiveP__flush(void );
#line 759
static void CC2420ReceiveP__receive(void );









static void CC2420ReceiveP__waitForNextPacket(void );
#line 813
static void CC2420ReceiveP__reset_state(void );










static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg);
# 65 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static error_t CC2420PacketP__Acks__requestAck(message_t *p_msg);









static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg);





static inline int CC2420PacketP__getAddressLength(int type);








static inline uint8_t * CC2420PacketP__getNetwork(message_t * msg);
#line 111
static inline int8_t CC2420PacketP__CC2420Packet__getRssi(message_t *p_msg);



static inline uint8_t CC2420PacketP__CC2420Packet__getLqi(message_t *p_msg);



static inline uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t * p_msg);
#line 137
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);
#line 152
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg);
#line 171
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg);





static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value);
#line 210
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg);








static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg);
# 58 "/opt/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 41 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 47
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 44
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 53 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
#line 65
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 114
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 67 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__1____nesc_unnamed4390 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type ) + 5, 



  TransformCounterC__1__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 133
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 103
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
#line 73
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void );
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void );
# 77 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1____nesc_unnamed4391 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 5, 
  TransformAlarmC__1__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void );
#line 147
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt);
#line 162
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 177
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
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
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4392 {
#line 74
  AlarmToTimerC__0__fired = 11U
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
uint8_t arg_0x412069f0);
#line 71
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4393 {
#line 71
  VirtualizeTimerC__0__updateFromTimer = 12U
};
#line 71
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4394 {

  VirtualizeTimerC__0__NUM_TIMERS = 9U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4395 {

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









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 189
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(uint8_t num);
#line 204
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 58 "/opt/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 52 "/opt/tinyos/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 69
static uint32_t RandomMlcgC__Random__rand32(void );
#line 89
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t UniqueSendP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
#line 100
static void UniqueSendP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/opt/tinyos/tos/interfaces/Random.nc"
static uint16_t UniqueSendP__Random__rand16(void );
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg);
# 56 "/opt/tinyos/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void );
#line 45
static error_t UniqueSendP__State__requestState(uint8_t reqState);
# 54 "/opt/tinyos/tos/chips/cc2520/unique/UniqueSendP.nc"
uint8_t UniqueSendP__localSendId;

enum UniqueSendP____nesc_unnamed4396 {
  UniqueSendP__S_IDLE, 
  UniqueSendP__S_SENDING
};


static inline error_t UniqueSendP__Init__init(void );
#line 75
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len);
#line 104
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__Receive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP__DuplicateReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/opt/tinyos/tos/chips/cc2520/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP____nesc_unnamed4397 {
  uint16_t source;
  uint8_t dsn;
} UniqueReceiveP__receivedMessages[4];

uint8_t UniqueReceiveP__writeIndex = 0;


uint8_t UniqueReceiveP__recycleSourceElement;

enum UniqueReceiveP____nesc_unnamed4398 {
  UniqueReceiveP__INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP__Init__init(void );









static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
static inline uint16_t UniqueReceiveP__getSourceKey(message_t  *msg);


static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len);
#line 112
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 138
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
#line 165
static inline uint16_t UniqueReceiveP__getSourceKey(message_t * msg);
#line 192
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len);
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP__grantTask__postTask(void );
# 75 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Packet.nc"
static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t * p_msg);
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 53 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
static bool CC2420TinyosNetworkP__Queue__isEmpty(void );
#line 70
static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void );
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__BareReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__granted(
# 46 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x412b4e60);
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP__ActiveReceive__receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 180 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP____nesc_unnamed4399 {
#line 180
  CC2420TinyosNetworkP__grantTask = 13U
};
#line 180
typedef int CC2420TinyosNetworkP____nesc_sillytask_grantTask[CC2420TinyosNetworkP__grantTask];
#line 68
enum CC2420TinyosNetworkP____nesc_unnamed4400 {
  CC2420TinyosNetworkP__OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP__TINYOS_N_NETWORKS = 0U
};




#line 73
enum CC2420TinyosNetworkP____nesc_unnamed4401 {
  CC2420TinyosNetworkP__CLIENT_AM, 
  CC2420TinyosNetworkP__CLIENT_BARE
} CC2420TinyosNetworkP__m_busy_client;

uint8_t CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
#line 78
uint8_t CC2420TinyosNetworkP__next_owner;
#line 102
static inline void CC2420TinyosNetworkP__BarePacket__clear(message_t *msg);



static inline uint8_t CC2420TinyosNetworkP__BarePacket__payloadLength(message_t *msg);




static void CC2420TinyosNetworkP__BarePacket__setPayloadLength(message_t *msg, uint8_t len);




static inline uint8_t CC2420TinyosNetworkP__BarePacket__maxPayloadLength(void );







static inline error_t CC2420TinyosNetworkP__BareSend__send(message_t *msg, uint8_t len);
#line 138
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len);









static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 180
static inline void CC2420TinyosNetworkP__grantTask__runTask(void );
#line 229
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id);
#line 247
static inline message_t *CC2420TinyosNetworkP__ActiveReceive__default__receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP__ActiveSend__default__sendDone(message_t *msg, error_t error);


static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client);
# 49 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0____nesc_unnamed4402 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[0];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );







static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
static error_t PacketLinkP__SubSend__send(
#line 67
message_t * msg, 







uint8_t len);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t PacketLinkP__send__postTask(void );
# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void PacketLinkP__DelayTimer__startOneShot(uint32_t dt);




static void PacketLinkP__DelayTimer__stop(void );
# 100 "/opt/tinyos/tos/interfaces/Send.nc"
static void PacketLinkP__Send__sendDone(
#line 96
message_t * msg, 



error_t error);
# 71 "/opt/tinyos/tos/interfaces/State.nc"
static uint8_t PacketLinkP__SendState__getState(void );
#line 56
static void PacketLinkP__SendState__toIdle(void );
#line 45
static error_t PacketLinkP__SendState__requestState(uint8_t reqState);
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
static cc2420_metadata_t * PacketLinkP__CC2420PacketBody__getMetadata(message_t * msg);
# 59 "/opt/tinyos/tos/interfaces/PacketAcknowledgements.nc"
static error_t PacketLinkP__PacketAcknowledgements__requestAck(
#line 53
message_t * msg);
#line 85
static bool PacketLinkP__PacketAcknowledgements__wasAcked(
#line 80
message_t * msg);
# 77 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
enum PacketLinkP____nesc_unnamed4403 {
#line 77
  PacketLinkP__send = 14U
};
#line 77
typedef int PacketLinkP____nesc_sillytask_send[PacketLinkP__send];
#line 58
message_t *PacketLinkP__currentSendMsg;


uint8_t PacketLinkP__currentSendLen;


uint16_t PacketLinkP__totalRetries;





enum PacketLinkP____nesc_unnamed4404 {
  PacketLinkP__S_IDLE, 
  PacketLinkP__S_SENDING
};




static void PacketLinkP__signalDone(error_t error);









static inline void PacketLinkP__PacketLink__setRetries(message_t *msg, uint16_t maxRetries);








static inline void PacketLinkP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay);






static inline uint16_t PacketLinkP__PacketLink__getRetries(message_t *msg);






static inline uint16_t PacketLinkP__PacketLink__getRetryDelay(message_t *msg);






static inline bool PacketLinkP__PacketLink__wasDelivered(message_t *msg);
#line 130
static inline error_t PacketLinkP__Send__send(message_t *msg, uint8_t len);
#line 171
static inline void PacketLinkP__SubSend__sendDone(message_t *msg, error_t error);
#line 202
static inline void PacketLinkP__DelayTimer__fired(void );






static inline void PacketLinkP__send__runTask(void );










static void PacketLinkP__signalDone(error_t error);
# 64 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Packet.nc"
static int8_t CC2420ReadLqiC__CC2420Packet__getRssi(message_t *p_msg);







static uint8_t CC2420ReadLqiC__CC2420Packet__getLqi(message_t *p_msg);
# 12 "/opt/tinyos/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static inline uint8_t CC2420ReadLqiC__ReadLqi__readLqi(message_t *msg);



static inline uint8_t CC2420ReadLqiC__ReadLqi__readRssi(message_t *msg);
# 60 "/opt/tinyos/tos/system/PoolP.nc"
uint8_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__free;
uint8_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__index;
/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t * /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[12];
/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool[12];

static inline error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init(void );
#line 88
static inline /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t */*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get(void );
#line 103
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t *newVal);
#line 60
uint8_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free;
uint8_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index;
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t * /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[12];
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool[12];

static inline error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init(void );
#line 88
static inline /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t */*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get(void );
#line 103
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t *newVal);
# 48 "/opt/tinyos/tos/system/QueueC.nc"
/*IPDispatchC.QueueC*/QueueC__0__queue_t  /*IPDispatchC.QueueC*/QueueC__0__queue[12];
uint8_t /*IPDispatchC.QueueC*/QueueC__0__head = 0;
uint8_t /*IPDispatchC.QueueC*/QueueC__0__tail = 0;
uint8_t /*IPDispatchC.QueueC*/QueueC__0__size = 0;

static inline bool /*IPDispatchC.QueueC*/QueueC__0__Queue__empty(void );



static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__size(void );



static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize(void );



static inline /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__head(void );



static inline void /*IPDispatchC.QueueC*/QueueC__0__printQueue(void );
#line 85
static /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 97
static inline error_t /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(/*IPDispatchC.QueueC*/QueueC__0__queue_t newVal);
# 60 "/opt/tinyos/tos/system/PoolP.nc"
uint8_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free;
uint8_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index;
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t * /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[3];
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool[3];

static inline error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init(void );
#line 88
static inline /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t */*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get(void );
#line 103
static inline error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t *newVal);
# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
static void IPStackControlP__SplitControl__startDone(error_t error);
#line 138
static void IPStackControlP__SplitControl__stopDone(error_t error);
#line 104
static error_t IPStackControlP__SubSplitControl__start(void );
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool IPStackControlP__IPAddress__getGlobalAddr(struct in6_addr *addr);
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t IPStackControlP__RoutingControl__start(void );









static error_t IPStackControlP__RoutingControl__stop(void );
#line 95
static error_t IPStackControlP__StdControl__start(void );
# 12 "/opt/tinyos/tos/lib/net/blip/IPStackControlP.nc"
static inline error_t IPStackControlP__SplitControl__start(void );



static inline void IPStackControlP__SubSplitControl__startDone(error_t error);
#line 37
static inline void IPStackControlP__SubSplitControl__stopDone(error_t error);



static inline void IPStackControlP__IPAddress__changed(bool valid);






static inline error_t IPStackControlP__StdControl__default__start(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void ICMPCoreP__ICMP_IP__recv(
# 50 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
uint8_t arg_0x4134ee70, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool ICMPCoreP__IPAddress__setSource(struct ip6_hdr *hdr);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t ICMPCoreP__IP__send(struct ip6_packet *msg);
# 59 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
static inline void ICMPCoreP__IP__recv(struct ip6_hdr *iph, 
void *packet, 
size_t len, 
struct ip6_metadata *meta);
#line 105
static error_t ICMPCoreP__ICMP_IP__send(uint8_t type, struct ip6_packet *pkt);









static inline void ICMPCoreP__IPAddress__changed(bool valid);

static inline void ICMPCoreP__ICMP_IP__default__recv(uint8_t type, struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);
# 23 "/opt/tinyos/tos/lib/net/blip/IPPacketC.nc"
static int IPPacketC__IPPacket__findHeader(struct ip_iovec *payload, 
uint8_t first_type, uint8_t *search_type);
#line 59
static int IPPacketC__IPPacket__findTLV(struct ip_iovec *header, int ext_offset, uint8_t type);
# 60 "/opt/tinyos/tos/system/PoolP.nc"
uint8_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free;
uint8_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index;
/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t * /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__queue[3];
/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool[3];

static inline error_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Init__init(void );
#line 88
static inline /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t */*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__get(void );
#line 103
static error_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__put(/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t *newVal);
# 52 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static error_t NoDhcpC__IPAddress__setAddress(struct in6_addr *addr);
# 49 "/opt/tinyos/tos/lib/net/blip/dhcp/NoDhcpC.nc"
static inline void NoDhcpC__Boot__booted(void );







static inline void NoDhcpC__IPAddress__changed(bool valid);
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool RPLRankP__IPAddress__getGlobalAddr(struct in6_addr *addr);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t RPLRankP__IP_DIO__send(struct ip6_packet *msg);
# 20 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
static bool RPLRankP__RPLOF__recomputeRoutes(void );
#line 4
static bool RPLRankP__RPLOF__OCP(uint16_t ocp);


static bool RPLRankP__RPLOF__objectSupported(uint16_t objectType);






static uint16_t RPLRankP__RPLOF__getRank(void );
static void RPLRankP__RPLOF__resetRank(void );

static bool RPLRankP__RPLOF__recalcualateRank(void );




static void RPLRankP__RPLOF__setMinHopRankIncrease(uint16_t val);
#line 9
static uint16_t RPLRankP__RPLOF__getObjectValue(void );

static struct in6_addr *RPLRankP__RPLOF__getParent(void );
# 41 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
static void RPLRankP__RouteInfo__resetTrickle(void );
#line 56
static void RPLRankP__RouteInfo__inconsistency(void );
#line 49
static void RPLRankP__RouteInfo__setDODAGConfig(uint8_t DIOIntDouble, uint8_t DIOIntMin, 
uint8_t DIORedun, uint8_t MaxRankInc, uint8_t MinHopRankInc);
#line 45
static uint8_t RPLRankP__RouteInfo__getInstanceID(void );
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
static int RPLRankP__IPPacket__findTLV(struct ip_iovec *header, 
int ext_offset, 
uint8_t type);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t RPLRankP__newParentSearch__postTask(void );
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static void RPLRankP__IP_DIO_Filter__recv(struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 362 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
enum RPLRankP____nesc_unnamed4405 {
#line 362
  RPLRankP__newParentSearch = 15U
};
#line 362
typedef int RPLRankP____nesc_sillytask_newParentSearch[RPLRankP__newParentSearch];
#line 102
uint16_t RPLRankP__nodeRank = INFINITE_RANK;
uint16_t RPLRankP__minRank = INFINITE_RANK;
bool RPLRankP__leafState = FALSE;

struct in6_addr;

uint8_t RPLRankP__parentNum = 0;
uint16_t RPLRankP__VERSION = 0;

uint16_t RPLRankP__MAX_RANK_INCREASE = 1;







struct in6_addr RPLRankP__DODAGID;
struct in6_addr RPLRankP__DODAG_MAX;
uint8_t RPLRankP__METRICID;
uint16_t RPLRankP__OCP;


uint8_t RPLRankP__Prf = 0xFF;


bool RPLRankP__ignore = FALSE;
bool RPLRankP__ROOT = FALSE;
bool RPLRankP__m_running = FALSE;
parent_t RPLRankP__parentSet[20];

static void RPLRankP__resetValid(void );
static void RPLRankP__getNewRank(void );






static inline error_t RPLRankP__StdControl__start(void );
#line 158
static inline error_t RPLRankP__StdControl__stop(void );




static inline parent_t *RPLRankP__RPLParentTable__get(uint8_t i);
#line 186
static uint8_t RPLRankP__getParent(struct in6_addr *node);


static uint16_t RPLRankP__RPLRankInfo__getRank(struct in6_addr *node);
#line 216
static error_t RPLRankP__RPLRankInfo__getDefaultRoute(struct in6_addr *next);
#line 228
static inline bool RPLRankP__exceedThreshold(uint8_t indexset, uint8_t ID);









static uint8_t RPLRankP__getParent(struct in6_addr *node);
#line 265
static void RPLRankP__resetValid(void );









static void RPLRankP__RPLRankInfo__inconsistencyDetected(void );









static inline uint8_t RPLRankP__RPLRankInfo__hasParent(void );



static inline bool RPLRankP__RPLRankInfo__isLeaf(void );




static inline uint8_t RPLRankP__getPreExistingParent(struct in6_addr *node);
#line 309
static inline uint16_t RPLRankP__RPLRankInfo__getEtx(void );



static void RPLRankP__insertParent(parent_t parent);
#line 350
static void RPLRankP__evictParent(uint8_t indexset);
#line 362
static inline void RPLRankP__newParentSearch__runTask(void );






static inline void RPLRankP__evictAll(void );
#line 393
static inline bool RPLRankP__ForwardingEvents__initiate(struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 442
static inline bool RPLRankP__ForwardingEvents__approve(struct ip6_packet *pkt, 
struct in6_addr *next_hop);
#line 524
static inline void RPLRankP__ForwardingEvents__linkResult(struct in6_addr *node, 
struct send_info *info);
#line 569
static inline bool RPLRankP__compareParent(parent_t oldP, parent_t newP);



static void RPLRankP__getNewRank(void );
#line 610
static inline void RPLRankP__parseDIO(struct ip6_hdr *iph, 
uint8_t *buf, 
int len);
#line 935
static inline void RPLRankP__IP_DIO__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);
#line 959
static inline error_t RPLRankP__IP_DIO_Filter__send(struct ip6_packet *msg);



static inline void RPLRankP__IPAddress__changed(bool global_valid);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__send(struct ip6_packet *msg);
# 40 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngine.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLDAORoutingEngine__startDAO(void );
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__postTask(void );
# 136 "/opt/tinyos/tos/lib/timer/Timer.nc"
static uint32_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__getNow(void );
#line 92
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__isRunning(void );
#line 64
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__startPeriodic(uint32_t dt);
#line 78
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__stop(void );
# 46 "/opt/tinyos/tos/interfaces/Random.nc"
static uint32_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__Random__rand32(void );
# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__isRunning(void );
#line 73
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__startOneShot(uint32_t dt);




static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__stop(void );
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__postTask(void );
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getLLAddr(struct in6_addr *addr);
#line 44
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__isLocalAddress(struct in6_addr *addr);
#line 34
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getGlobalAddr(struct in6_addr *addr);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__postTask(void );
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__send(struct ip6_packet *msg);
# 98 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
static uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__hasParent(void );
#line 84
static uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(struct in6_addr *node);
#line 101
static uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getEtx(void );
#line 99
static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__isLeaf(void );
#line 94
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__inconsistencyDetected(void );
#line 113
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getDefaultRoute(struct in6_addr *next_hop);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__postTask(void );
#line 67
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__postTask(void );
# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RankControl__start(void );









static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RankControl__stop(void );
# 119 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
enum /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_unnamed4406 {
#line 119
  RPLRoutingEngineP__0__sendDIOTask = 16U
};
#line 119
typedef int /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_sillytask_sendDIOTask[/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask];
enum /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_unnamed4407 {
#line 120
  RPLRoutingEngineP__0__sendDISTask = 17U
};
#line 120
typedef int /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_sillytask_sendDISTask[/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask];
enum /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_unnamed4408 {
#line 121
  RPLRoutingEngineP__0__init = 18U
};
#line 121
typedef int /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_sillytask_init[/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init];
enum /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_unnamed4409 {
#line 122
  RPLRoutingEngineP__0__initDIO = 19U
};
#line 122
typedef int /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_sillytask_initDIO[/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO];
#line 167
enum /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_unnamed4410 {
#line 167
  RPLRoutingEngineP__0__computeRemaining = 20U
};
#line 167
typedef int /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0____nesc_sillytask_computeRemaining[/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining];
#line 71
uint32_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__tricklePeriod;
uint32_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime;
bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sentDIOFlag = FALSE;
bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_ROOT = FALSE;
bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_LEAF = FALSE;
bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running = FALSE;
bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG = FALSE;

uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__node_rank = INFINITE_RANK;
uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__LOWRANK = INFINITE_RANK;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__GROUND_STATE = 1;

uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLInstanceID = 0;
struct in6_addr /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGVersionNumber = 0;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MOP = RPL_MOP_Storing_No_Multicast;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DAG_PREF = 7;

uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__redunCounter = 0xFF;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__doubleCounter = 0;

uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntDouble = 10;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntMin = 8;
uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIORedun = 0xFF;
uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MinHopRankInc = 1;
uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MaxRankInc = 3;

uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DTSN = 2;




bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO = FALSE;

struct in6_addr;

struct in6_addr /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP;
struct in6_addr;
struct in6_addr /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR;
struct in6_addr /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO_ADDR;


static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__resetTrickleTime(void );
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__chooseAdvertiseTime(void );
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeTrickleRemaining(void );
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__nextTrickleTime(void );
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__inconsistencyDetected(void );







static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__runTask(void );
#line 161
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__runTask(void );





static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__runTask(void );



static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__runTask(void );
#line 322
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__runTask(void );
#line 355
uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__INCONSISTENCY_COUNT = 0;

static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__inconsistencyDetected(void );
#line 383
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__resetTrickleTime(void );






static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__chooseAdvertiseTime(void );










static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeTrickleRemaining(void );







static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__nextTrickleTime(void );










static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__inconsistency(void );



static inline bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__hasDODAG(void );



static inline uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getMOP(void );



static inline error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDefaultRoute(struct in6_addr *next);



static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDODAGConfig(uint8_t IntDouble, 
uint8_t IntMin, 
uint8_t Redun, 
uint8_t RankInc, 
uint8_t HopRankInc);
#line 454
static inline uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getInstanceID(void );







static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle(void );





static inline uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getRank(void );



static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDTSN(uint8_t dtsn);



static inline uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDTSN(void );
#line 500
static inline error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__start(void );









static inline error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__stop(void );






static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__fired(void );



static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IncreaseVersionTimer__fired(void );





static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__fired(void );
#line 542
static inline bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__compare_ip6_addr(struct in6_addr *node1, struct in6_addr *node2);
#line 554
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);
#line 577
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);
#line 693
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__changed(bool global_valid);
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
static 
#line 94
/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__t * 


/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__get(void );
#line 89
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(
#line 85
/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__t * newVal);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__send(struct ip6_packet *msg);
# 52 "/opt/tinyos/tos/interfaces/Random.nc"
static uint16_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__Random__rand16(void );
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__postTask(void );
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__startPeriodic(uint32_t dt);
# 90 "/opt/tinyos/tos/interfaces/Queue.nc"
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__enqueue(
#line 86
/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__t  newVal);
#line 81
static 
#line 79
/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__t  

/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__dequeue(void );
#line 58
static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__size(void );
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__getLLAddr(struct in6_addr *addr);




static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__getGlobalAddr(struct in6_addr *addr);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__postTask(void );
# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__isRunning(void );
#line 73
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__startOneShot(uint32_t dt);
#line 92
static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__isRunning(void );
#line 73
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__startOneShot(uint32_t dt);
# 52 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getMOP(void );
#line 45
static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getInstanceID(void );







static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__setDTSN(uint8_t dtsn);
#line 42
static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__hasDODAG(void );

static uint16_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getRank(void );









static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDTSN(void );
#line 43
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDefaultRoute(struct in6_addr *next_hop);
# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
static struct route_entry */*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__lookupRoute(const uint8_t *prefix, int prefix_len_bits);
#line 16
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__delRoute(route_key_t key);
#line 10
static route_key_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__addRoute(const uint8_t *prefix, int prefix_len_bits, 
struct in6_addr *next_hop, uint8_t ifindex);
# 107 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
enum /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0____nesc_unnamed4411 {
#line 107
  RPLDAORoutingEngineP__0__sendDAO = 21U
};
#line 107
typedef int /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0____nesc_sillytask_sendDAO[/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO];
#line 190
enum /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0____nesc_unnamed4412 {
#line 190
  RPLDAORoutingEngineP__0__initDAO = 22U
};
#line 190
typedef int /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0____nesc_sillytask_initDAO[/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO];
#line 68
uint32_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__dao_rate = 10000;



uint32_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__delay_dao = 256;
uint32_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__remove_time = 120 * 1024U;


uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__daoseq = 0;


uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__PATH_SEQUENCE = 0;
uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__PATH_CONTROL = 0;

downwards_table_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[ROUTE_TABLE_SZ];
uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count = 0;
bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__m_running = FALSE;

static inline bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__memcmp_rpl(uint8_t *a, uint8_t *b, uint8_t len);







static inline error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__start(void );





static inline error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__stop(void );






static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__runTask(void );
#line 156
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__startDAO(void );
#line 182
static inline bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__getStoreState(void );









static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__fired(void );
#line 209
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__runTask(void );
#line 280
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__fired(void );



static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__fired(void );
#line 304
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta);
#line 419
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__newParent(void );









static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__changed(bool global_valid);
# 48 "/opt/tinyos/tos/system/QueueC.nc"
/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t  /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue[5];
uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__head = 0;
uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__tail = 0;
uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__size = 0;

static inline bool /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__empty(void );



static inline uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__size(void );



static inline uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__maxSize(void );



static inline /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__head(void );



static inline void /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__printQueue(void );
#line 85
static inline /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__dequeue(void );
#line 97
static error_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__enqueue(/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t newVal);
# 60 "/opt/tinyos/tos/system/PoolP.nc"
uint8_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free;
uint8_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index;
/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t * /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__queue[5];
/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool[5];

static inline error_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Init__init(void );
#line 88
static /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t */*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__get(void );
#line 103
static error_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__put(/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t *newVal);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__send(struct ip6_packet *msg);





static void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__recv(
# 35 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
uint8_t arg_0x415698a8, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 39 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__recv(struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta);





static inline error_t /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__send(uint8_t code, struct ip6_packet *msg);



static inline void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__default__recv(uint8_t code, struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta);
# 2 "/opt/tinyos/tos/lib/net/rpl/RPLParentTable.nc"
static parent_t *RPLOF0P__ParentTable__get(uint8_t parent_index);
# 42 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngine.nc"
static void RPLOF0P__RPLDAO__newParent(void );
# 56 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
static void RPLOF0P__RPLRoute__inconsistency(void );
# 16 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
static error_t RPLOF0P__ForwardingTable__delRoute(route_key_t key);
#line 10
static route_key_t RPLOF0P__ForwardingTable__addRoute(const uint8_t *prefix, int prefix_len_bits, 
struct in6_addr *next_hop, uint8_t ifindex);
# 51 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
uint16_t RPLOF0P__nodeRank = INFINITE_RANK;
uint16_t RPLOF0P__minMetric = 0xFFFF;
uint16_t RPLOF0P__prevParent;


uint16_t RPLOF0P__desiredParent = 20 - 1;
uint16_t RPLOF0P__nodeEtx = 10;
bool RPLOF0P__newParent = FALSE;
uint16_t RPLOF0P__min_hop_rank_inc = 1;
route_key_t RPLOF0P__route_key = ROUTE_INVAL_KEY;


static inline bool RPLOF0P__RPLOF__OCP(uint16_t ocp);






static inline bool RPLOF0P__RPLOF__objectSupported(uint16_t objectType);







static inline void RPLOF0P__RPLOF__setMinHopRankIncrease(uint16_t val);



static inline uint16_t RPLOF0P__RPLOF__getObjectValue(void );




static struct in6_addr *RPLOF0P__RPLOF__getParent(void );





static inline uint16_t RPLOF0P__RPLOF__getRank(void );



static inline bool RPLOF0P__RPLOF__recalcualateRank(void );
#line 125
static bool RPLOF0P__RPLOF__recomputeRoutes(void );
#line 250
static inline void RPLOF0P__RPLOF__resetRank(void );
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__send(struct ip6_packet *msg);





static void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__recv(
# 35 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
uint8_t arg_0x415698a8, 
# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *hdr, void *packet, 
size_t len, struct ip6_metadata *meta);
# 39 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__recv(struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta);





static inline error_t /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__send(uint8_t code, struct ip6_packet *msg);



static inline void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__default__recv(uint8_t code, struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta);
# 39 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
static coap_tid_t CoapUdpServerP__LibCoapServer__send(coap_context_t *ctx, 
struct sockaddr_in6 *dst, 
coap_pdu_t *pdu, 
int free_pdu);




static error_t CoapUdpServerP__LibCoapServer__bind(uint16_t port);
# 52 "/opt/tinyos/tos/interfaces/Random.nc"
static uint16_t CoapUdpServerP__Random__rand16(void );
# 34 "/opt/tinyos/tos/interfaces/WriteResource.nc"
static int CoapUdpServerP__WriteResource__put(
# 67 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
uint8_t arg_0x415b4978, 
# 34 "/opt/tinyos/tos/interfaces/WriteResource.nc"
uint8_t *val, size_t buflen, coap_tid_t id);
# 33 "/opt/tinyos/tos/interfaces/ReadResource.nc"
static int CoapUdpServerP__ReadResource__get(
# 66 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
uint8_t arg_0x415b4110, 
# 33 "/opt/tinyos/tos/interfaces/ReadResource.nc"
coap_tid_t id);
# 100 "/opt/tinyos/tos/interfaces/Leds.nc"
static void CoapUdpServerP__Leds__led2Toggle(void );
# 69 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
coap_context_t *CoapUdpServerP__ctx_server;
unsigned short CoapUdpServerP__tid;



static 
#line 73
int 
CoapUdpServerP__coap_extract_node(coap_queue_t **queue, coap_queue_t *node);
#line 110
static uint8_t CoapUdpServerP__get_key(uint8_t *uri, uint8_t len);










static unsigned short CoapUdpServerP__get_new_tid(void );






static inline int CoapUdpServerP__resource_splitphase(coap_uri_t *uri, 
coap_tid_t *id, 
unsigned char *mediatype, 
unsigned int offset, 
unsigned char *buf, 
unsigned int *buflen, 
int *finished, 
unsigned int method);

static int CoapUdpServerP__coap_save_splitphase(coap_context_t *ctx, coap_queue_t *node);

static inline void CoapUdpServerP__message_handler(coap_context_t *ctx, coap_queue_t *node, void *data);

static inline error_t CoapUdpServerP__Init__init(void );
#line 153
static inline error_t CoapUdpServerP__CoAPServer__bind(uint16_t port);






static inline int CoapUdpServerP__print_link(coap_resource_t *resource, unsigned char *buf, size_t buflen);
#line 209
static inline int CoapUdpServerP__resource_wellknown(coap_uri_t *uri, 
coap_tid_t *id, 
unsigned char *mediatype, 
unsigned int offset, 
unsigned char *buf, 
unsigned int *buflen, 
int *finished, 
unsigned int method);
#line 272
static inline error_t CoapUdpServerP__CoAPServer__registerWellknownCore(void );
#line 301
static inline error_t CoapUdpServerP__CoAPServer__registerResource(char uri[5], 
unsigned int uri_length, 
unsigned char mediatype, 
unsigned int writable, 
unsigned int splitphase, 
unsigned int immediately);
#line 330
static inline int CoapUdpServerP__resource_splitphase(coap_uri_t *uri, 
coap_tid_t *id, 
unsigned char *mediatype, 
unsigned int offset, 
unsigned char *buf, 
unsigned int *buflen, 
int *finished, 
unsigned int method);
#line 349
static inline void CoapUdpServerP__LibCoapServer__read(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta);
#line 366
static coap_pdu_t *CoapUdpServerP__new_rst(coap_context_t *ctx, coap_queue_t *node, 
unsigned int code);





static coap_pdu_t *CoapUdpServerP__new_response(coap_context_t *ctx, coap_queue_t *node, 
unsigned int code);







static coap_pdu_t *CoapUdpServerP__new_asynresponse(coap_context_t *ctx, coap_queue_t *node);








static void CoapUdpServerP__add_contents(coap_pdu_t *pdu, unsigned char mediatype, 
unsigned int len, unsigned char *data);
#line 404
static inline coap_opt_t *CoapUdpServerP__coap_next_option(coap_pdu_t *pdu, coap_opt_t *opt);








static inline int CoapUdpServerP__mediatype_matches(coap_pdu_t *pdu, unsigned char mediatype);
#line 429
static inline coap_pdu_t *CoapUdpServerP__handle_get(coap_context_t *ctx, coap_queue_t *node, void *data);
#line 532
static inline int CoapUdpServerP__ReadResource__default__get(uint8_t uri_key, coap_tid_t id);




static inline void CoapUdpServerP__ReadResource__getDone(uint8_t uri_key, error_t result, 
coap_tid_t id, 
uint8_t asyn_message, 
uint8_t *val_buf, 
size_t buflen);
#line 656
static inline coap_pdu_t *CoapUdpServerP__handle_put(coap_context_t *ctx, coap_queue_t *node, void *data);
#line 743
static inline int CoapUdpServerP__WriteResource__default__put(uint8_t uri_key, uint8_t *val, size_t buflen, coap_tid_t id);




static inline void CoapUdpServerP__WriteResource__putDone(uint8_t uri_key, error_t result, 
coap_tid_t id, 
uint8_t asyn_message);
#line 815
static inline coap_pdu_t *CoapUdpServerP__handle_post(coap_context_t *ctx, coap_queue_t *node, void *data);






static inline coap_pdu_t *CoapUdpServerP__handle_delete(coap_context_t *ctx, coap_queue_t *node, void *data);






static inline void CoapUdpServerP__message_handler(coap_context_t *ctx, coap_queue_t *node, void *data);
#line 897
static int CoapUdpServerP__coap_save_splitphase(coap_context_t *ctx, 
coap_queue_t *node);
# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
static bool UdpP__IPAddress__setSource(struct ip6_hdr *hdr);
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
static void UdpP__UDP__recvfrom(
# 8 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x41652e90, 
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip6_metadata *meta);
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
static error_t UdpP__IP__send(struct ip6_packet *msg);
# 15 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
enum UdpP____nesc_unnamed4413 {
  UdpP__N_CLIENTS = 1U
};


uint16_t UdpP__local_ports[UdpP__N_CLIENTS];

enum UdpP____nesc_unnamed4414 {
  UdpP__LOCAL_PORT_START = 51024U, 
  UdpP__LOCAL_PORT_STOP = 54999U
};
uint16_t UdpP__last_localport = UdpP__LOCAL_PORT_START;

static inline uint16_t UdpP__alloc_lport(uint8_t clnt);
#line 46
static inline error_t UdpP__Init__init(void );





static inline error_t UdpP__UDP__bind(uint8_t clnt, uint16_t port);
#line 64
static inline void UdpP__IP__recv(struct ip6_hdr *iph, void *packet, size_t len, struct ip6_metadata *meta);
#line 115
static inline error_t UdpP__UDP__sendto(uint8_t clnt, struct sockaddr_in6 *dest, void *payload, 
uint16_t len);







static inline error_t UdpP__UDP__sendtov(uint8_t clnt, struct sockaddr_in6 *dest, 
struct ip_iovec *iov);
#line 168
static inline void UdpP__BlipStatistics__clear(void );
#line 180
static inline void UdpP__UDP__default__recvfrom(uint8_t clnt, struct sockaddr_in6 *from, 
void *payload, 
uint16_t len, 
struct ip6_metadata *meta);

static inline void UdpP__IPAddress__changed(bool global_valid);
# 35 "/opt/tinyos/tos/interfaces/WriteResource.nc"
static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__putDone(error_t result, coap_tid_t id, uint8_t asyn_message);
# 34 "/opt/tinyos/tos/interfaces/ReadResource.nc"
static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__getDone(error_t result, coap_tid_t id, uint8_t asyn_message, uint8_t *val, size_t buflen);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__postTask(void );
# 117 "/opt/tinyos/tos/interfaces/Leds.nc"
static uint8_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__Leds__get(void );
#line 134
static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__Leds__set(uint8_t val);
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
static error_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__postTask(void );
# 44 "/opt/tinyos/tos/lib/net/coap/CoapLedResourceP.nc"
enum /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0____nesc_unnamed4415 {
#line 44
  CoapLedResourceP__0__getLed = 23U
};
#line 44
typedef int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0____nesc_sillytask_getLed[/*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed];
#line 63
enum /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0____nesc_unnamed4416 {
#line 63
  CoapLedResourceP__0__setLedDone = 24U
};
#line 63
typedef int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0____nesc_sillytask_setLedDone[/*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone];
#line 41
bool /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock = FALSE;
coap_tid_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__temp_id;

static inline void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__runTask(void );






static inline int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__get(coap_tid_t id);
#line 63
static inline void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__runTask(void );




static inline int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__put(uint8_t *val, size_t buflen, coap_tid_t id);
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

# 83 "/opt/tinyos/tos/lib/timer/BusyWaitCounterC.nc"
static inline void /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow(void )
{
}

# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 82
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__overflow();
#line 82
}
#line 82
# 64 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
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
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4064e4b0){
#line 39
  switch (arg_0x4064e4b0) {
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
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4064e4b0);
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
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4417 {
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

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4418 {
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

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4419 {
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
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 114
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 177 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 58 "/opt/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 82
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 82
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 82
}
#line 82
# 133 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__0__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 58 "/opt/tinyos/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 177 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 82
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__overflow();
#line 82
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 82
}
#line 82
# 133 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__1__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 82 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__overflow(void ){
#line 82
  /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 82
  /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 82
}
#line 82
# 64 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__overflow();
}

# 48 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__overflow();
#line 48
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 48
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
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

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 188 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 188
{
  return CC2420ControlP__SpiResource__request();
}

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420ControlP__Resource__request();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 210 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startVRegDone(void )
#line 210
{
  CC2420CsmaP__Resource__request();
}

# 56 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  CC2420CsmaP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 48 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 40
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__set();
#line 40
}
#line 40
# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 41
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__clr();
#line 41
}
#line 41
# 431 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 431
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 803 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 803
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 61 "/opt/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 61
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 53 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
}

#line 157
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow(), dt);
}

# 66 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 66
}
#line 66
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__HplGeneralIO__get(void ){
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
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__CCA__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 498 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__BackoffTimer__fired(void )
#line 498
{
  /* atomic removed: atomic calls only */
#line 499
  {
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SAMPLE_CCA: 


          if (CC2420TransmitP__CCA__get()) {
              CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
              CC2420TransmitP__BackoffTimer__start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP__congestionBackoff();
            }
        break;

        case CC2420TransmitP__S_BEGIN_TRANSMIT: 
          case CC2420TransmitP__S_CANCEL: 
            if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
                CC2420TransmitP__attemptSend();
              }
        break;

        case CC2420TransmitP__S_ACK_WAIT: 
          CC2420TransmitP__signalDone(SUCCESS);
        break;

        case CC2420TransmitP__S_SFD: 


          CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CaptureSFD__captureRisingEdge();
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 78
  CC2420TransmitP__BackoffTimer__fired();
#line 78
  CC2420ControlP__StartupTimer__fired();
#line 78
}
#line 78
# 162 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 78
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
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
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4420 {
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

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__get(void ){
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
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__get();
}

# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__get();
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
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__isOverflowPending(void ){
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
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Msp430Timer__isOverflowPending();
}

# 71 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static bool /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__isOverflowPending();
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
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
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 103
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 103
}
#line 103
# 291 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(message_t *msg)
#line 291
{
}

# 88 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 89 "/opt/tinyos/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 89
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 52 "/opt/tinyos/tos/interfaces/Random.nc"
inline static uint16_t CC2420CsmaP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 251 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime)
#line 251
{
  CC2420TransmitP__myCongestionBackoff = backoffTime + 1;
}

# 66 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime){
#line 66
  CC2420TransmitP__RadioBackoff__setCongestionBackoff(backoffTime);
#line 66
}
#line 66
# 230 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 230
{
  CC2420CsmaP__SubBackoff__setCongestionBackoff(CC2420CsmaP__Random__rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestCongestionBackoff(msg);
}

# 88 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 45 "/opt/tinyos/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 173 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 173
{
#line 173
  return FALSE;
}

# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(uint8_t arg_0x40c9a9b0){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  switch (arg_0x40c9a9b0) {
#line 128
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 128
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 128
      break;
#line 128
    default:
#line 128
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__isOwner(arg_0x40c9a9b0);
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
# 112 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline bool /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(uint8_t id)
#line 112
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__isOwner(id);
}

# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 177 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 177
{
  return &msp430_spi_default_config;
}

# 39 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x40c99418){
#line 39
  union __nesc_unnamed4311 *__nesc_result;
#line 39

#line 39
    __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x40c99418);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 168 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 120 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 120
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__setModeSpi(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40da8430){
#line 59
  switch (arg_0x40da8430) {
#line 59
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x40da8430);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 213 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 213
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/opt/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 206 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 206
{
}

# 61 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x40da9010){
#line 61
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x40da9010);
#line 61
}
#line 61
# 93 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 93
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
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

#line 100
        return __nesc_temp;
      }
  }
#line 102
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 107
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
  return FAIL;
}

# 175 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 175
{
#line 175
  return FAIL;
}

# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x40c9a9b0){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  switch (arg_0x40c9a9b0) {
#line 97
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 97
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 97
      break;
#line 97
    default:
#line 97
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__immediateRequest(arg_0x40c9a9b0);
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
# 104 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(uint8_t id)
#line 104
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__immediateRequest(id);
}

# 97 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
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

# 151 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 97 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
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

# 7 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 99
}
#line 99
# 207 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 92
}
#line 92
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 238 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 118 "/opt/tinyos/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP__state[id] = StateImplP__S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 210 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/opt/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 64 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 88
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 89
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = id;
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

# 79 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 204 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 204
{
}

# 53 "/opt/tinyos/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x40da9010){
#line 53
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x40da9010);
#line 53
}
#line 53
# 77 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
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

#line 85
            return __nesc_temp;
          }
        }
      else 
#line 87
        {
          unsigned char __nesc_temp = 
#line 87
          /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

#line 87
          return __nesc_temp;
        }
      }
  }
#line 89
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 174 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(uint8_t id)
#line 174
{
#line 174
  return FAIL;
}

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(uint8_t arg_0x40c9a9b0){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  switch (arg_0x40c9a9b0) {
#line 88
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 88
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 88
      break;
#line 88
    default:
#line 88
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__request(arg_0x40c9a9b0);
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
# 108 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(uint8_t id)
#line 108
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__request(id);
}

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 382 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  HplMsp430Usart0P__U0TXBUF = data;
}

# 224 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 330 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & 0x40) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
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
# 341 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~0x40;
}

# 197 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
# 386 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  return U0RXBUF;
}

# 231 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
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
# 95 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__ChipSpiResource__abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP__release = FALSE;
}

# 31 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP__ChipSpiResource__abortRelease(void ){
#line 31
  CC2420SpiP__ChipSpiResource__abortRelease();
#line 31
}
#line 31
# 377 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 377
{
  if (CC2420TransmitP__abortSpiRelease) {
      CC2420TransmitP__ChipSpiResource__abortRelease();
    }
}

# 24 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 208 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 208
{
}

# 46 "/opt/tinyos/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 97 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
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
# 124 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 124
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableSpi();
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__resetUsart(FALSE);
}

# 218 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 218
{
}

# 65 "/opt/tinyos/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x40da8430){
#line 65
  switch (arg_0x40da8430) {
#line 65
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 65
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 65
      break;
#line 65
    default:
#line 65
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x40da8430);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
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
# 68 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 72
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 75
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
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
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 111 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 111
{
  /* atomic removed: atomic calls only */
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

#line 127
          return __nesc_temp;
        }
      }
  }
#line 130
  return FAIL;
}

# 176 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(uint8_t id)
#line 176
{
#line 176
  return FAIL;
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(uint8_t arg_0x40c9a9b0){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  switch (arg_0x40c9a9b0) {
#line 120
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 120
      __nesc_result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 120
      break;
#line 120
    default:
#line 120
      __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__default__release(arg_0x40c9a9b0);
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
# 116 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(uint8_t id)
#line 116
{
  return /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__release(id);
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 99
}
#line 99
# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 99
}
#line 99
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXONCCA);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP__SNOP__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SNOP);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 102 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 102
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/opt/tinyos/tos/chips/cc2520/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 65 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 65
{
  /* atomic removed: atomic calls only */
#line 65
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 92 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4421 {
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

# 55 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 55
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 55
}
#line 55
# 130 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 57
}
#line 57
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

# 322 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 322
{
  const uint8_t *base = source;

#line 324
  return ((uint16_t )base[1] << 8) | base[0];
}

#line 347
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

# 70 "/opt/tinyos/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__clear(message_t * msg){
#line 70
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 70
}
#line 70
# 195 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void )
#line 195
{
  if (CC2420ReceiveP__m_timestamp_size) {
      CC2420ReceiveP__m_timestamp_size--;
    }
}

# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd_dropped(void ){
#line 55
  CC2420ReceiveP__CC2420Receive__sfd_dropped();
#line 55
}
#line 55
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__HplGeneralIO__get(void ){
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
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 186 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time)
#line 186
{
  if (CC2420ReceiveP__m_timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__m_timestamp_head + CC2420ReceiveP__m_timestamp_size) % 
      CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 190
      CC2420ReceiveP__m_timestamp_queue[tail] = time;
      CC2420ReceiveP__m_timestamp_size++;
    }
}

# 49 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time){
#line 49
  CC2420ReceiveP__CC2420Receive__sfd(time);
#line 49
}
#line 49
# 65 "/opt/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 65
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 54 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 137 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg)
#line 137
{
  return (cc2420_header_t * )((uint8_t *)msg + (unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 65 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 73 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 102 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 73 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 73
}
#line 73
# 48 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__set();
}

# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__set();
#line 40
}
#line 40
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__clr();
#line 41
}
#line 41
# 292 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 292
{
  const uint8_t *base = source;

#line 294
  return base[0];
}

# 219 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(msg)->length.nxdata)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 58 "/opt/tinyos/tos/chips/cc2520/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(message_t * msg){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__get(msg);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

#line 303
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 303
{
#line 303
  return __nesc_ntoh_uint8(source);
}

# 152 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg)
#line 152
{
  return (cc2420_metadata_t *)msg->metadata;
}

#line 210
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.nxdata);
}

# 50 "/opt/tinyos/tos/chips/cc2520/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(message_t * msg){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = CC2420PacketP__PacketTimeSyncOffset__isSet(msg);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 177 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.nxdata, value);
}

# 78 "/opt/tinyos/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__set(message_t * msg, CC2420TransmitP__PacketTimeStamp__size_type value){
#line 78
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 78
}
#line 78
# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 259 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP__getTime32(uint16_t captured_time)
{
  uint32_t now = CC2420TransmitP__BackoffTimer__getNow();


  return now - (uint16_t )(now - captured_time);
}

#line 280
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 280
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 281
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 283
  {
    time32 = CC2420TransmitP__getTime32(time);
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__m_state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();
        CC2420TransmitP__PacketTimeStamp__set(CC2420TransmitP__m_msg, time32);
        if (CC2420TransmitP__PacketTimeSyncOffset__isSet(CC2420TransmitP__m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP__PacketTimeSyncOffset__get(CC2420TransmitP__m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP__m_msg + absOffset);

            (__nesc_temp44 = (*timesync).nxdata, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32));
            CC2420TransmitP__CSN__clr();
            CC2420TransmitP__TXFIFO_RAM__write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP__CSN__set();

            (__nesc_temp45 = (*timesync).nxdata, __nesc_hton_uint32(__nesc_temp45, __nesc_ntoh_uint32(__nesc_temp45) + time32));
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP__abortSpiRelease = TRUE;
          }
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__BackoffTimer__stop();

        if (CC2420TransmitP__SFD__get()) {
            break;
          }


        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__CaptureSFD__captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP__m_state = CC2420TransmitP__S_ACK_WAIT;
            CC2420TransmitP__BackoffTimer__start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 326
          {
            CC2420TransmitP__signalDone(SUCCESS);
          }

        if (!CC2420TransmitP__SFD__get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP__m_receiving && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();

              sfd_state = CC2420TransmitP__SFD__get();
              CC2420TransmitP__CC2420Receive__sfd(time32);
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              if (CC2420TransmitP__SFD__get()) {

                  return;
                }
            }



        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP__m_prev_time < 10) {
                CC2420TransmitP__CC2420Receive__sfd_dropped();
                if (CC2420TransmitP__m_msg) {
                  CC2420TransmitP__PacketTimeStamp__clear(CC2420TransmitP__m_msg);
                  }
              }
#line 370
            break;
          }
      }
  }
}

# 61 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 61
  CC2420TransmitP__CaptureSFD__captured(time);
#line 61
}
#line 61
# 175 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 68 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 68
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 68
}
#line 68
# 95 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 44 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 44
}
#line 44
# 76 "/opt/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 76
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 86 "/opt/tinyos/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 86
}
#line 86
# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4422 {
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
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 78
}
#line 78
# 162 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 78 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 78
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 78
}
#line 78
# 135 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 58
}
#line 58
# 70 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
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
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4423 {
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

# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static bool /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__1__Counter__isOverflowPending();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 130 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
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
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
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
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
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
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 43
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 43
}
#line 43
# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
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
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 87
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 94
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 97
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 99
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
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
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4424 {
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
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4425 {
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
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4426 {
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
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4427 {
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
# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

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
# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

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
# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

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
# 82 "/opt/tinyos/tos/lib/net/coap/LibCoapAdapterP.nc"
static inline coap_tid_t LibCoapAdapterP__LibCoapServer__send(coap_context_t *context, 
struct sockaddr_in6 *dst, 
coap_pdu_t *pdu, 
int free_pdu)
#line 85
{
  return coap_send_impl(context, dst, pdu, free_pdu);
}

# 39 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
inline static coap_tid_t CoapUdpServerP__LibCoapServer__send(coap_context_t *ctx, struct sockaddr_in6 *dst, coap_pdu_t *pdu, int free_pdu){
#line 39
  unsigned short __nesc_result;
#line 39

#line 39
  __nesc_result = LibCoapAdapterP__LibCoapServer__send(ctx, dst, pdu, free_pdu);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 537 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline void CoapUdpServerP__ReadResource__getDone(uint8_t uri_key, error_t result, 
coap_tid_t id, 
uint8_t asyn_message, 
uint8_t *val_buf, 
size_t buflen)
#line 541
{
  coap_queue_t *node;
  coap_pdu_t *pdu;
  coap_opt_t *ct;




  if (!(node = coap_find_transaction(CoapUdpServerP__ctx_server->splitphasequeue, id))) {
      ;
#line 550
      ;
      return;
    }

  if (result != SUCCESS) {


      if (!(pdu = CoapUdpServerP__new_response(CoapUdpServerP__ctx_server, node, 200))) {
        }


      memcpy(val_buf, "Sensor not found", 16);
      buflen = 16;
    }
  else 
#line 563
    {

      if (asyn_message) {
          node->pdu->hdr->id = CoapUdpServerP__get_new_tid();

          if (!(pdu = CoapUdpServerP__new_asynresponse(CoapUdpServerP__ctx_server, node))) {
            }
        }
      else {

          if (!(pdu = CoapUdpServerP__new_response(CoapUdpServerP__ctx_server, node, 80))) {
            }
        }
    }




  ct = coap_check_option(node->pdu, 1);
  if (ct) {
      coap_add_option(pdu, 1, ct->lval.flag == 15 ? ct->lval.length + 15 : ct->sval.length, (unsigned char *)& *ct + (ct->lval.flag == 15 ? 2 : 1));
    }


  if (!coap_add_data(pdu, buflen, val_buf)) {
      coap_delete_pdu(pdu);
      if (!(pdu = CoapUdpServerP__new_response(CoapUdpServerP__ctx_server, node, 200))) {
        }
    }



  ;
#line 595
  ;
  if (pdu && CoapUdpServerP__LibCoapServer__send(CoapUdpServerP__ctx_server, & node->remote, pdu, 1) == -1) {

      coap_delete_pdu(pdu);
    }





  CoapUdpServerP__coap_extract_node(& CoapUdpServerP__ctx_server->splitphasequeue, node);
  node->next = (void *)0;
  coap_delete_node(node);
}

# 34 "/opt/tinyos/tos/interfaces/ReadResource.nc"
inline static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__getDone(error_t result, coap_tid_t id, uint8_t asyn_message, uint8_t *val, size_t buflen){
#line 34
  CoapUdpServerP__ReadResource__getDone(KEY_LED, result, id, asyn_message, val, buflen);
#line 34
}
#line 34
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )48U & (0x01 << 6);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw() != 0;
}

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void )
#line 51
{
#line 51
  return /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led2__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )48U & (0x01 << 5);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw() != 0;
}

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void )
#line 51
{
#line 51
  return /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led1__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )48U & (0x01 << 4);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw() != 0;
}

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void )
#line 51
{
#line 51
  return /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led0__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 119 "/opt/tinyos/tos/system/LedsP.nc"
static inline uint8_t LedsP__Leds__get(void )
#line 119
{
  uint8_t rval;

#line 121
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    {
      rval = 0;
      if (!LedsP__Led0__get()) {
          rval |= LEDS_LED0;
        }
      if (!LedsP__Led1__get()) {
          rval |= LEDS_LED1;
        }
      if (!LedsP__Led2__get()) {
          rval |= LEDS_LED2;
        }
    }
#line 132
    __nesc_atomic_end(__nesc_atomic); }
  return rval;
}

# 117 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static uint8_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__Leds__get(void ){
#line 117
  unsigned char __nesc_result;
#line 117

#line 117
  __nesc_result = LedsP__Leds__get();
#line 117

#line 117
  return __nesc_result;
#line 117
}
#line 117
# 44 "/opt/tinyos/tos/lib/net/coap/CoapLedResourceP.nc"
static inline void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__runTask(void )
#line 44
{
  uint8_t val = /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__Leds__get();

#line 46
  /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock = FALSE;
  /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__getDone(SUCCESS, /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__temp_id, 0, 
  (uint8_t *)&val, sizeof(uint8_t ));
}

# 52 "/opt/tinyos/tos/interfaces/Random.nc"
inline static uint16_t CoapUdpServerP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t UdpP__IP__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = IPProtocolsP__IP__send(IANA_UDP, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 28 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline uint16_t UdpP__alloc_lport(uint8_t clnt)
#line 28
{
  int i;
#line 29
  int done = 0;
  uint16_t compare = (((uint16_t )UdpP__last_localport << 8) | ((uint16_t )UdpP__last_localport >> 8)) & 0xffff;

#line 31
  UdpP__last_localport = UdpP__last_localport < UdpP__LOCAL_PORT_STOP ? UdpP__last_localport + 1 : UdpP__LOCAL_PORT_START;
  while (!done) {
      done = 1;
      for (i = 0; i < UdpP__N_CLIENTS; i++) {
          if (UdpP__local_ports[i] == compare) {
              UdpP__last_localport = UdpP__last_localport < UdpP__LOCAL_PORT_STOP ? UdpP__last_localport + 1 : UdpP__LOCAL_PORT_START;
              compare = (((uint16_t )UdpP__last_localport << 8) | ((uint16_t )UdpP__last_localport >> 8)) & 0xffff;
              done = 0;
              break;
            }
        }
    }
  return UdpP__last_localport;
}

# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool UdpP__IPAddress__setSource(struct ip6_hdr *hdr){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = IPAddressP__IPAddress__setSource(hdr);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 124 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline error_t UdpP__UDP__sendtov(uint8_t clnt, struct sockaddr_in6 *dest, 
struct ip_iovec *iov)
#line 125
{
  error_t rc;
  struct ip6_packet pkt;
  struct udp_hdr udp;
  struct ip_iovec v[1];
  size_t len = iov_len(iov);


  memset((uint8_t *)& pkt.ip6_hdr, 0, sizeof  pkt.ip6_hdr);
  memset((uint8_t *)&udp, 0, sizeof udp);
  memcpy(& pkt.ip6_hdr.ip6_dst, dest->sin6_addr.in6_u.u6_addr8, 16);
  UdpP__IPAddress__setSource(& pkt.ip6_hdr);

  if (UdpP__local_ports[clnt] == 0 && (
  UdpP__local_ports[clnt] = UdpP__alloc_lport(clnt)) == 0) {
      return FAIL;
    }

  udp.srcport = UdpP__local_ports[clnt];
  udp.dstport = dest->sin6_port;
  udp.len = (((uint16_t )(len + sizeof(struct udp_hdr )) << 8) | ((uint16_t )(len + sizeof(struct udp_hdr )) >> 8)) & 0xffff;
  udp.chksum = 0;


  pkt.ip6_hdr.ip6_ctlun.ip6_un2_vfc = 0x60;
  pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IANA_UDP;
  pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = udp.len;


  v[0].iov_base = (uint8_t *)&udp;
  v[0].iov_len = sizeof(struct udp_hdr );
  v[0].iov_next = iov;
  pkt.ip6_data = &v[0];

  udp.chksum = (((uint16_t )msg_cksum(& pkt.ip6_hdr, v, IANA_UDP) << 8) | ((uint16_t )msg_cksum(& pkt.ip6_hdr, v, IANA_UDP) >> 8)) & 0xffff;

  rc = UdpP__IP__send(&pkt);
  ;
  return rc;
}

#line 115
static inline error_t UdpP__UDP__sendto(uint8_t clnt, struct sockaddr_in6 *dest, void *payload, 
uint16_t len)
#line 116
{
  struct ip_iovec v[1];

#line 118
  v[0].iov_base = payload;
  v[0].iov_len = len;
  v[0].iov_next = (void *)0;
  return UdpP__UDP__sendtov(clnt, dest, &v[0]);
}

# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t LibCoapAdapterP__UDPServer__sendto(struct sockaddr_in6 *dest, void *payload, uint16_t len){
#line 18
  unsigned char __nesc_result;
#line 18

#line 18
  __nesc_result = UdpP__UDP__sendto(0U, dest, payload, len);
#line 18

#line 18
  return __nesc_result;
#line 18
}
#line 18
# 21 "/opt/tinyos/tos/lib/net/blip/Ieee154AddressP.nc"
static inline ieee154_panid_t Ieee154AddressP__Ieee154Address__getPanId(void )
#line 21
{
  return Ieee154AddressP__m_panid;
}

# 5 "/opt/tinyos/tos/lib/net/blip/interfaces/Ieee154Address.nc"
inline static ieee154_panid_t IPAddressP__Ieee154Address__getPanId(void ){
#line 5
  unsigned int __nesc_result;
#line 5

#line 5
  __nesc_result = Ieee154AddressP__Ieee154Address__getPanId();
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 24 "/opt/tinyos/tos/lib/net/blip/Ieee154AddressP.nc"
static inline ieee154_saddr_t Ieee154AddressP__Ieee154Address__getShortAddr(void )
#line 24
{
  return Ieee154AddressP__m_saddr;
}

# 6 "/opt/tinyos/tos/lib/net/blip/interfaces/Ieee154Address.nc"
inline static ieee154_saddr_t IPAddressP__Ieee154Address__getShortAddr(void ){
#line 6
  unsigned int __nesc_result;
#line 6

#line 6
  __nesc_result = Ieee154AddressP__Ieee154Address__getShortAddr();
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6

inline static ieee154_laddr_t IPAddressP__Ieee154Address__getExtAddr(void ){
#line 7
  struct ieee_eui64 __nesc_result;
#line 7

#line 7
  __nesc_result = Ieee154AddressP__Ieee154Address__getExtAddr();
#line 7

#line 7
  return __nesc_result;
#line 7
}
#line 7
# 48 "/opt/tinyos/tos/interfaces/LocalIeeeEui64.nc"
inline static ieee_eui64_t Ieee154AddressP__LocalIeeeEui64__getId(void ){
#line 48
  struct ieee_eui64 __nesc_result;
#line 48

#line 48
  __nesc_result = DallasId48ToIeeeEui64C__LocalIeeeEui64__getId();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 29 "/opt/tinyos/tos/platforms/epic/chips/ds2411/DallasId48.h"
static inline bool dallasid48checkCrc(const dallasid48_serial_t *id)
#line 29
{
  uint8_t crc = 0;
  uint8_t idx;

#line 32
  for (idx = 0; idx < DALLASID48_DATA_LENGTH; idx++) {
      uint8_t i;

#line 34
      crc = crc ^ id->data[idx];
      for (i = 0; i < 8; i++) {
          if (crc & 0x01) {
              crc = (crc >> 1) ^ 0x8C;
            }
          else {
              crc >>= 1;
            }
        }
    }
  return crc == 0;
}

# 66 "/opt/tinyos/tos/lib/timer/BusyWait.nc"
inline static void OneWireMasterC__BusyWait__wait(OneWireMasterC__BusyWait__size_type dt){
#line 66
  /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__BusyWait__wait(dt);
#line 66
}
#line 66
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw(void )
#line 59
{
#line 59
  return * (volatile uint8_t * )40U & (0x01 << 4);
}

#line 60
static inline bool /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get(void )
#line 60
{
#line 60
  return /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__getRaw() != 0;
}

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__get(void ){
#line 73
  unsigned char __nesc_result;
#line 73

#line 73
  __nesc_result = /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__get();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__get(void )
#line 51
{
#line 51
  return /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool OneWireMasterC__Pin__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )42U &= ~(0x01 << 4);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void OneWireMasterC__Pin__makeInput(void ){
#line 44
  /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeInput();
#line 44
}
#line 44
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )42U |= 0x01 << 4;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void OneWireMasterC__Pin__makeOutput(void ){
#line 46
  /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/opt/tinyos/tos/platforms/epic/chips/ds2411/OneWireMasterC.nc"
static inline bool OneWireMasterC__readBit(void )
#line 56
{
  bool bit;

#line 58
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
  bit = OneWireMasterC__Pin__get();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__SLOT_TIME);
  return bit;
}

#line 80
static inline uint8_t OneWireMasterC__readByte(void )
#line 80
{
  uint8_t i;
#line 81
  uint8_t c = 0;

#line 82
  for (i = 0; i < 8; i++) {
      c >>= 1;
      if (OneWireMasterC__readBit()) {
          c |= 0x80;
        }
    }
  return c;
}

#line 49
static inline void OneWireMasterC__writeZero(void )
#line 49
{
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_60US);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
}

#line 42
static inline void OneWireMasterC__writeOne(void )
#line 42
{
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__SLOT_TIME);
}

#line 67
static inline void OneWireMasterC__writeByte(uint8_t c)
#line 67
{
  uint8_t j;

#line 69
  for (j = 0; j < 8; j++) {
      if (c & 0x01) {
          OneWireMasterC__writeOne();
        }
      else {
          OneWireMasterC__writeZero();
        }
      c >>= 1;
    }
}

# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )41U &= ~(0x01 << 4);
}

# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P24*/HplMsp430GeneralIOP__12__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__clr(void )
#line 49
{
#line 49
  /*Ds2411C.Gpio*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void OneWireMasterC__Pin__clr(void ){
#line 41
  /*Ds2411C.Gpio*/Msp430GpioC__4__GeneralIO__clr();
#line 41
}
#line 41
# 27 "/opt/tinyos/tos/platforms/epic/chips/ds2411/OneWireMasterC.nc"
static inline bool OneWireMasterC__reset(void )
#line 27
{
  uint16_t i;

#line 29
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__Pin__clr();
  OneWireMasterC__Pin__makeOutput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__RESET_LOW_TIME);
  OneWireMasterC__Pin__makeInput();
  OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_60US);

  for (i = 0; i < OneWireMasterC__PRESENCE_DETECT_LOW_TIME; i += OneWireMasterC__DELAY_5US, OneWireMasterC__BusyWait__wait(OneWireMasterC__DELAY_5US)) 
    if (!OneWireMasterC__Pin__get()) {
#line 37
      break;
      }
#line 38
  OneWireMasterC__BusyWait__wait(OneWireMasterC__PRESENCE_RESET_HIGH_TIME - OneWireMasterC__DELAY_60US);
  return i < OneWireMasterC__PRESENCE_DETECT_LOW_TIME;
}

#line 91
static inline error_t OneWireMasterC__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len)
#line 91
{
  error_t e = SUCCESS;

#line 93
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      if (OneWireMasterC__reset()) {
          uint8_t i;

#line 96
          OneWireMasterC__writeByte(cmd);
          for (i = 0; i < len; i++) {
              buf[i] = OneWireMasterC__readByte();
            }
        }
      else {
          e = EOFF;
        }
    }
#line 104
    __nesc_atomic_end(__nesc_atomic); }
  return e;
}

# 10 "/opt/tinyos/tos/platforms/epic/chips/ds2411/OneWireStream.nc"
inline static error_t Ds2411P__OneWire__read(uint8_t cmd, uint8_t *buf, uint8_t len){
#line 10
  unsigned char __nesc_result;
#line 10

#line 10
  __nesc_result = OneWireMasterC__OneWire__read(cmd, buf, len);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 23 "/opt/tinyos/tos/platforms/epic/chips/ds2411/Ds2411P.nc"
static inline error_t Ds2411P__readId(void )
#line 23
{
  error_t e = Ds2411P__OneWire__read(0x33, Ds2411P__ds2411id.data, DALLASID48_DATA_LENGTH);

#line 25
  if (e == SUCCESS) {
      if (dallasid48checkCrc(&Ds2411P__ds2411id)) {
          Ds2411P__haveId = TRUE;
        }
      else {
          e = EINVAL;
        }
    }
  return e;
}

static inline error_t Ds2411P__ReadId48__read(uint8_t *id)
#line 36
{
  error_t e = SUCCESS;

#line 38
  if (!Ds2411P__haveId) {
      e = Ds2411P__readId();
    }
  if (Ds2411P__haveId) {
      uint8_t i;

#line 43
      for (i = 0; i < DALLASID48_SERIAL_LENGTH; i++) {
          id[i] = Ds2411P__ds2411id.serial[i];
        }
    }
  return e;
}

# 12 "/opt/tinyos/tos/platforms/epic/chips/ds2411/ReadId48.nc"
inline static error_t DallasId48ToIeeeEui64C__ReadId48__read(uint8_t *id){
#line 12
  unsigned char __nesc_result;
#line 12

#line 12
  __nesc_result = Ds2411P__ReadId48__read(id);
#line 12

#line 12
  return __nesc_result;
#line 12
}
#line 12
# 62 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__get(void )
{




  if (0) {
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

# 45 "/opt/tinyos/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
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
# 49 "/opt/tinyos/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__size_type /*BusyWaitMicroC.BusyWaitCounterC*/BusyWaitCounterC__0__Counter__get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Msp430CounterMicroC.Counter*/Msp430CounterC__0__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
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

#line 286
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

# 454 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getInstanceID(void )
#line 454
{
  return /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLInstanceID;
}

# 45 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static uint8_t RPLRankP__RouteInfo__getInstanceID(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getInstanceID();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 393 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline bool RPLRankP__ForwardingEvents__initiate(struct ip6_packet *pkt, 
struct in6_addr *next_hop)
#line 394
{
  unsigned char *__nesc_temp49;
  unsigned char *__nesc_temp48;
#line 395
  uint16_t len;
  static struct ip_iovec v;
  static rpl_data_hdr_t data_hdr;


  return TRUE;


  if (pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt == IANA_ICMP) {
    return TRUE;
    }
  __nesc_hton_uint8(data_hdr.ip6_ext_outer.ip6e_nxt.nxdata, pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt);
  __nesc_hton_uint8(data_hdr.ip6_ext_outer.ip6e_len.nxdata, 0);


  __nesc_hton_uint8(data_hdr.ip6_ext_inner.ip6e_nxt.nxdata, RPL_HBH_RANK_TYPE);
  __nesc_hton_uint8(data_hdr.ip6_ext_inner.ip6e_len.nxdata, sizeof(rpl_data_hdr_t ) - 
  (unsigned short )& ((rpl_data_hdr_t *)0)->bitflag);
  __nesc_hton_uint8(data_hdr.bitflag.nxdata, 0);
  __nesc_hton_uint8(data_hdr.bitflag.nxdata, 0 << 7);
  (__nesc_temp48 = data_hdr.bitflag.nxdata, __nesc_hton_uint8(__nesc_temp48, __nesc_ntoh_uint8(__nesc_temp48) | (0 << 6)));
  (__nesc_temp49 = data_hdr.bitflag.nxdata, __nesc_hton_uint8(__nesc_temp49, __nesc_ntoh_uint8(__nesc_temp49) | (0 << 5)));
  __nesc_hton_uint8(data_hdr.instance_id.nxdata, RPLRankP__RouteInfo__getInstanceID());
  __nesc_hton_uint16(data_hdr.senderRank.nxdata, RPLRankP__nodeRank);
  pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IPV6_HOP;

  len = (((uint16_t )pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff;


  v.iov_base = (uint8_t *)&data_hdr;
  v.iov_len = sizeof(rpl_data_hdr_t );
  v.iov_next = pkt->ip6_data;


  pkt->ip6_data = &v;
  len = len + v.iov_len;
  pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )len << 8) | ((uint16_t )len >> 8)) & 0xffff;
  return TRUE;
}

# 348 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline bool IPForwardingEngineP__ForwardingEvents__default__initiate(uint8_t idx, struct ip6_packet *pkt, 
struct in6_addr *next_hop)
#line 349
{
  return TRUE;
}

# 13 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
inline static bool IPForwardingEngineP__ForwardingEvents__initiate(uint8_t arg_0x40925e40, struct ip6_packet *pkt, struct in6_addr *next_hop){
#line 13
  unsigned char __nesc_result;
#line 13

#line 13
  switch (arg_0x40925e40) {
#line 13
    case RPL_IFACE:
#line 13
      __nesc_result = RPLRankP__ForwardingEvents__initiate(pkt, next_hop);
#line 13
      break;
#line 13
    default:
#line 13
      __nesc_result = IPForwardingEngineP__ForwardingEvents__default__initiate(arg_0x40925e40, pkt, next_hop);
#line 13
      break;
#line 13
    }
#line 13

#line 13
  return __nesc_result;
#line 13
}
#line 13
# 355 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline error_t IPForwardingEngineP__IPForward__default__send(uint8_t ifindex, struct in6_addr *next_hop, 
struct ip6_packet *pkt, 
void *data)
#line 357
{




  return SUCCESS;
}

# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
inline static error_t IPForwardingEngineP__IPForward__send(uint8_t arg_0x40922068, struct in6_addr *next_hop, struct ip6_packet *msg, void *data){
#line 18
  unsigned char __nesc_result;
#line 18

#line 18
  switch (arg_0x40922068) {
#line 18
    case ROUTE_IFACE_154:
#line 18
      __nesc_result = IPNeighborDiscoveryP__IPForward__send(next_hop, msg, data);
#line 18
      break;
#line 18
    default:
#line 18
      __nesc_result = IPForwardingEngineP__IPForward__default__send(arg_0x40922068, next_hop, msg, data);
#line 18
      break;
#line 18
    }
#line 18

#line 18
  return __nesc_result;
#line 18
}
#line 18
# 128 "/opt/tinyos/tos/lib/net/blip/IPAddressP.nc"
static inline bool IPAddressP__IPAddress__isLLAddress(struct in6_addr *addr)
#line 128
{
  if (addr->in6_u.u6_addr16[0] == ((((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff) || (
  addr->in6_u.u6_addr8[0] == 0xff && (
  addr->in6_u.u6_addr8[1] & 0x0f) <= 2)) {
    return TRUE;
    }
#line 133
  return FALSE;
}

# 50 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool IPForwardingEngineP__IPAddress__isLLAddress(struct in6_addr *addr){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = IPAddressP__IPAddress__isLLAddress(addr);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
#line 44
inline static bool IPForwardingEngineP__IPAddress__isLocalAddress(struct in6_addr *addr){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = IPAddressP__IPAddress__isLocalAddress(addr);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 195 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline error_t IPForwardingEngineP__IP__send(struct ip6_packet *pkt)
#line 195
{
  struct route_entry *next_hop_entry = 
  IPForwardingEngineP__ForwardingTable__lookupRoute(pkt->ip6_hdr.ip6_dst.in6_u.u6_addr8, 128);






  if (IPForwardingEngineP__IPAddress__isLocalAddress(& pkt->ip6_hdr.ip6_dst) && 
  pkt->ip6_hdr.ip6_dst.in6_u.u6_addr8[0] != 0xff) {
      ;
#line 206
      ;
      return FAIL;
    }
  else {
#line 208
    if (IPForwardingEngineP__IPAddress__isLLAddress(& pkt->ip6_hdr.ip6_dst) && (
    !next_hop_entry || next_hop_entry->prefixlen < 128)) {
#line 221
        ;
#line 221
        ;
        ;
#line 222
        ;
        ;
#line 223
        ;
        pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_hlim = 1;

        if (pkt->ip6_hdr.ip6_dst.in6_u.u6_addr8[0] != 0xff) {
            return IPForwardingEngineP__do_send(ROUTE_IFACE_154, & pkt->ip6_hdr.ip6_dst, pkt);
          }
        else 
#line 228
          {
            return IPForwardingEngineP__IPForward__send(ROUTE_IFACE_154, & pkt->ip6_hdr.ip6_dst, pkt, (void *)0);
          }
      }
    else {
#line 231
      if (next_hop_entry) {
          ;
#line 232
          ;


          if (!IPForwardingEngineP__ForwardingEvents__initiate(next_hop_entry->ifindex, pkt, 
          & next_hop_entry->next_hop)) {
            return FAIL;
            }
          return IPForwardingEngineP__do_send(next_hop_entry->ifindex, & next_hop_entry->next_hop, pkt);
        }
      }
    }
#line 241
  return FAIL;
}

# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t IPProtocolsP__SubIP__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = IPForwardingEngineP__IP__send(msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 88 "/opt/tinyos/tos/system/PoolP.nc"
static inline /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t */*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__get(void )
#line 88
{
  if (/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free) {
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t *rval = /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__queue[/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index];

#line 91
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__queue[/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index] = (void *)0;
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free--;
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index++;
      if (/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index == 3) {
          /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static IPForwardingEngineP__Pool__t * IPForwardingEngineP__Pool__get(void ){
#line 97
  struct in6_iid *__nesc_result;
#line 97

#line 97
  __nesc_result = /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 5 "/opt/tinyos/tos/lib/net/blip/interfaces/Ieee154Address.nc"
inline static ieee154_panid_t IPNeighborDiscoveryP__Ieee154Address__getPanId(void ){
#line 5
  unsigned int __nesc_result;
#line 5

#line 5
  __nesc_result = Ieee154AddressP__Ieee154Address__getPanId();
#line 5

#line 5
  return __nesc_result;
#line 5
}
#line 5
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool IPNeighborDiscoveryP__IPAddress__getLLAddr(struct in6_addr *addr){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = IPAddressP__IPAddress__getLLAddr(addr);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t IPDispatchP__sendTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(IPDispatchP__sendTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * PacketLinkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 97 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline void PacketLinkP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay)
#line 97
{
  __nesc_hton_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->retryDelay.nxdata, retryDelay);
}

# 53 "/opt/tinyos/tos/interfaces/PacketLink.nc"
inline static void IPDispatchP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay){
#line 53
  PacketLinkP__PacketLink__setRetryDelay(msg, retryDelay);
#line 53
}
#line 53
# 88 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline void PacketLinkP__PacketLink__setRetries(message_t *msg, uint16_t maxRetries)
#line 88
{
  __nesc_hton_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->maxRetries.nxdata, maxRetries);
}

# 46 "/opt/tinyos/tos/interfaces/PacketLink.nc"
inline static void IPDispatchP__PacketLink__setRetries(message_t * msg, uint16_t maxRetries){
#line 46
  PacketLinkP__PacketLink__setRetries(msg, maxRetries);
#line 46
}
#line 46
# 69 "/opt/tinyos/tos/system/QueueC.nc"
static inline void /*IPDispatchC.QueueC*/QueueC__0__printQueue(void )
#line 69
{
}

#line 61
static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize(void )
#line 61
{
  return 12;
}

#line 57
static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__size(void )
#line 57
{
  return /*IPDispatchC.QueueC*/QueueC__0__size;
}

#line 97
static inline error_t /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(/*IPDispatchC.QueueC*/QueueC__0__queue_t newVal)
#line 97
{
  if (/*IPDispatchC.QueueC*/QueueC__0__Queue__size() < /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize()) {
      ;
      /*IPDispatchC.QueueC*/QueueC__0__queue[/*IPDispatchC.QueueC*/QueueC__0__tail] = newVal;
      /*IPDispatchC.QueueC*/QueueC__0__tail++;
      if (/*IPDispatchC.QueueC*/QueueC__0__tail == 12) {
#line 102
        /*IPDispatchC.QueueC*/QueueC__0__tail = 0;
        }
#line 103
      /*IPDispatchC.QueueC*/QueueC__0__size++;
      /*IPDispatchC.QueueC*/QueueC__0__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static error_t IPDispatchP__SendQueue__enqueue(IPDispatchP__SendQueue__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 89 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static error_t IPDispatchP__SendEntryPool__put(IPDispatchP__SendEntryPool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
inline static error_t IPDispatchP__FragPool__put(IPDispatchP__FragPool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 94 "/opt/tinyos/tos/interfaces/Packet.nc"
inline static void IPDispatchP__BarePacket__setPayloadLength(message_t * msg, uint8_t len){
#line 94
  CC2420TinyosNetworkP__BarePacket__setPayloadLength(msg, len);
#line 94
}
#line 94
# 116 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP__BarePacket__maxPayloadLength(void )
#line 116
{
  return 112 + sizeof(cc2420_header_t );
}

# 106 "/opt/tinyos/tos/interfaces/Packet.nc"
inline static uint8_t IPDispatchP__BarePacket__maxPayloadLength(void ){
#line 106
  unsigned char __nesc_result;
#line 106

#line 106
  __nesc_result = CC2420TinyosNetworkP__BarePacket__maxPayloadLength();
#line 106

#line 106
  return __nesc_result;
#line 106
}
#line 106
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 138 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len)
#line 138
{

  cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 141
  return hdr;
}

# 125 "/opt/tinyos/tos/interfaces/Send.nc"
inline static void * IPDispatchP__Ieee154Send__getPayload(message_t * msg, uint8_t len){
#line 125
  void *__nesc_result;
#line 125

#line 125
  __nesc_result = CC2420TinyosNetworkP__BareSend__getPayload(msg, len);
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 337 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t IPDispatchP__setFragDgramOffset(struct packed_lowmsg *msg, uint8_t size)
#line 337
{
  uint8_t *buf = msg->data;

#line 339
  if (buf == (void *)0) {
#line 339
    return 1;
    }




  if ((*buf >> 3) != LOWPAN_FRAGN_PATTERN) {
#line 345
    return 1;
    }
#line 346
  buf += 4;
  *buf = size;
  return 0;
}

#line 321
static __inline uint8_t IPDispatchP__setFragDgramTag(struct packed_lowmsg *msg, uint16_t tag)
#line 321
{
  uint8_t *buf = msg->data;

#line 323
  if (buf == (void *)0) {
#line 323
    return 1;
    }




  if ((*buf >> 3) != LOWPAN_FRAG1_PATTERN && (
  *buf >> 3) != LOWPAN_FRAGN_PATTERN) {
#line 330
    return 1;
    }
#line 331
  buf += 2;

  buf[0] = tag >> 8;
  buf[1] = tag & 0xff;
  return 0;
}

#line 301
static __inline uint8_t IPDispatchP__setFragDgramSize(struct packed_lowmsg *msg, uint16_t size)
#line 301
{
  uint8_t *buf = msg->data;

#line 303
  if (buf == (void *)0) {
#line 303
    return 1;
    }



  if ((*buf >> 3) != LOWPAN_FRAG1_PATTERN && (
  *buf >> 3) != LOWPAN_FRAGN_PATTERN) {
#line 309
    return 1;
    }
#line 310
  size = size & 0x7ff;


  *buf &= 0xf8;
  *buf |= size >> 8;
  buf[1] = size & 0xff;


  return 0;
}

#line 114
static __inline uint8_t IPDispatchP__setupHeaders(struct packed_lowmsg *packed, uint16_t headers)
#line 114
{
  uint8_t *buf = packed->data;
  uint16_t len = packed->len;

#line 117
  if (packed == (void *)0) {
#line 117
    return 1;
    }
#line 118
  if (buf == (void *)0) {
#line 118
    return 1;
    }
#line 119
  packed->headers = 0;
#line 136
  if (headers & LOWMSG_FRAG1_HDR) {
      if (len < LOWMSG_FRAG1_LEN) {
#line 137
        return 1;
        }
#line 138
      packed->headers |= LOWMSG_FRAG1_HDR;
      *buf = LOWPAN_FRAG1_PATTERN << 3;
      buf += LOWMSG_FRAG1_LEN;
      len -= LOWMSG_FRAG1_LEN;
    }
  if (headers & LOWMSG_FRAGN_HDR) {
      if (len < LOWMSG_FRAGN_LEN) {
#line 144
        return 1;
        }
#line 145
      packed->headers |= LOWMSG_FRAGN_HDR;
      *buf = LOWPAN_FRAGN_PATTERN << 3;
    }
  return 0;
}

# 261 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static inline int IPDispatchP__pack_udp(uint8_t *buf, size_t cnt, struct ip6_packet *packet, int offset)
#line 261
{
  struct udp_hdr udp;

  if (cnt < 7) {
      return -1;
    }

  if (iov_read(packet->ip6_data, offset, sizeof(struct udp_hdr ), (void *)&udp) != 
  sizeof(struct udp_hdr )) {
      return -1;
    }

  *buf = LOWPAN_NHC_UDP_PATTERN | LOWPAN_NHC_UDP_PORT_FULL;
  memcpy(buf + 1, & udp.srcport, 4);
  memcpy(buf + 5, & udp.chksum, 2);
  return 7;
}










static inline uint8_t IPDispatchP____ipnh_real_length(uint8_t type, struct ip_iovec *pkt, int offset)
#line 288
{
  int start_offset = offset;
#line 289
  int end_offset = offset + 2;
  struct ip6_ext ext;
  struct tlv_hdr tlv;

#line 292
  if (iov_read(pkt, offset, 2, (void *)&ext) != 2) {
    return -1;
    }


  if (type != IPV6_HOP && type != IPV6_DEST) {
    return (ext.ip6e_len + 1) * 8;
    }
  offset += 2;
  for (; ; ) {
      if (offset >= (ext.ip6e_len + 1) * 8) {
#line 302
        break;
        }
#line 303
      if (iov_read(pkt, offset, 2, (void *)&tlv) != 2) {
        return -1;
        }
      if (tlv.type == IPV6_TLV_PAD1) {
          offset += 1;
        }
      else 
#line 308
        {
          offset += 2 + tlv.len;
          if (tlv.type != IPV6_TLV_PADN) {
              end_offset = offset;
            }
        }
    }


  if (offset - start_offset != (ext.ip6e_len + 1) * 8) {
    return 0;
    }


  return end_offset - start_offset;
}

static inline int IPDispatchP__pack_ipnh(uint8_t *dest, size_t cnt, uint8_t *type, struct ip6_packet *packet, int offset)
#line 325
{
  struct ip6_ext ext;
  uint8_t real_len;


  if (iov_read(packet->ip6_data, offset, 2, (void *)&ext) != 2) {
    return -1;
    }
  if (ext.ip6e_len > cnt) {
    return -1;
    }
  *dest = LOWPAN_NHC_IPV6_PATTERN;
  switch (*type) {
      case IPV6_HOP: 
        *dest |= LOWPAN_NHC_EID_HOP;
#line 339
      break;
      case IPV6_ROUTING: 
        *dest |= LOWPAN_NHC_EID_ROUTING;
#line 341
      break;
      case IPV6_FRAG: 
        *dest |= LOWPAN_NHC_EID_FRAG;
#line 343
      break;
      case IPV6_DEST: 
        *dest |= LOWPAN_NHC_EID_DEST;
#line 345
      break;
      case IPV6_MOBILITY: 
        *dest |= LOWPAN_NHC_EID_MOBILE;
#line 347
      break;
      default: 
        return -1;
    }

  real_len = IPDispatchP____ipnh_real_length(*type, packet->ip6_data, offset);
  if (real_len == 0) {
    return -1;
    }


  *type = ext.ip6e_nxt;

  if ((((((
#line 359
  ext.ip6e_nxt == IPV6_HOP || ext.ip6e_nxt == IPV6_ROUTING) || ext.ip6e_nxt == IPV6_FRAG) || 
  ext.ip6e_nxt == IPV6_DEST) || ext.ip6e_nxt == IPV6_MOBILITY) || ext.ip6e_nxt == IPV6_IPV6) || 
  ext.ip6e_nxt == IANA_UDP) {
      *dest |= LOWPAN_NHC_NH;
    }
  else 
#line 363
    {

      dest++;
      *dest = ext.ip6e_nxt;
    }

  dest++;
  * dest++ = real_len;


  if (iov_read(packet->ip6_data, offset + 2, real_len - 2, dest) != real_len - 2) {
    return -1;
    }

  return (ext.ip6e_len + 1) * 8;
}

static inline int IPDispatchP__pack_nhc_chain(uint8_t **dest, size_t cnt, struct ip6_packet *packet)
#line 380
{
  uint8_t nxt = packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt;
  int offset = 0;
#line 382
  int rv;




  while (((((nxt == IPV6_HOP || nxt == IPV6_ROUTING) || nxt == IPV6_FRAG) || 
  nxt == IPV6_DEST) || nxt == IPV6_MOBILITY) || nxt == IPV6_IPV6) {
      int extra;

#line 390
      rv = IPDispatchP__pack_ipnh(*dest, cnt, &nxt, packet, offset);

      if (rv < 0) {
#line 392
        return -1;
        }



      extra = * *dest & LOWPAN_NHC_NH ? 0 : 1;
      *dest += rv + extra;
      offset += rv;
      cnt -= rv;
    }

  if (nxt == IANA_UDP) {
      rv = IPDispatchP__pack_udp(*dest, cnt, packet, offset);

      if (rv < 0) {
#line 406
        return -1;
        }
#line 407
      offset += sizeof(struct udp_hdr );
      *dest += rv;
    }
  return offset;
}

#line 235
static inline uint8_t *IPDispatchP__pack_multicast(uint8_t *buf, struct in6_addr *addr, uint8_t *flags)
#line 235
{

  *flags = 0;
  if (addr->in6_u.u6_addr16[0] == ((((uint16_t )0xff02 << 8) | ((uint16_t )0xff02 >> 8)) & 0xffff) && 
  IPDispatchP__bit_range_zero_p(addr->in6_u.u6_addr8, 16, 120) == 0) {
      *flags |= LOWPAN_IPHC_AM_M_8;
      *buf = addr->in6_u.u6_addr8[15];
      return buf + 1;
    }
  else {
#line 243
    if (IPDispatchP__bit_range_zero_p(addr->in6_u.u6_addr8, 16, 104) == 0) {
        *flags |= LOWPAN_IPHC_AM_M_32;
        *buf = addr->in6_u.u6_addr8[1];
        memcpy(buf + 1, &addr->in6_u.u6_addr8[13], 3);
        return buf + 4;
      }
    else {
#line 248
      if (IPDispatchP__bit_range_zero_p(addr->in6_u.u6_addr8, 16, 88) == 0) {
          *flags |= LOWPAN_IPHC_AM_M_48;
          *buf = addr->in6_u.u6_addr8[1];
          memcpy(buf + 1, &addr->in6_u.u6_addr8[11], 5);
          return buf + 6;
        }
      else 
#line 253
        {
          *flags += LOWPAN_IPHC_AM_M_128;
          memcpy(buf, addr->in6_u.u6_addr8, 16);
          return buf + 16;
        }
      }
    }
}

# 16 "/opt/tinyos/tos/lib/net/blip/interfaces/NeighborDiscovery.nc"
inline static int IPDispatchP__NeighborDiscovery__matchContext(struct in6_addr *addr, uint8_t *ctx){
#line 16
  int __nesc_result;
#line 16

#line 16
  __nesc_result = IPNeighborDiscoveryP__NeighborDiscovery__matchContext(addr, ctx);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 88 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline int IPDispatchP__lowpan_extern_match_context(struct in6_addr *addr, uint8_t *ctx_id)
#line 88
{
  return IPDispatchP__NeighborDiscovery__matchContext(addr, ctx_id);
}

# 148 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static __inline uint8_t *IPDispatchP__pack_hlim(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch)
#line 148
{
  if (hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim == 1) {
      *dispatch |= LOWPAN_IPHC_HLIM_1;
    }
  else {
#line 151
    if (hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim == 64) {
        *dispatch |= LOWPAN_IPHC_HLIM_64;
      }
    else {
#line 153
      if (hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim == 255) {
          *dispatch |= LOWPAN_IPHC_HLIM_255;
        }
      else 
#line 155
        {
          *dispatch |= LOWPAN_IPHC_HLIM_NONE;
          * buf++ = hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim;
        }
      }
    }
#line 159
  return buf;
}

#line 136
static __inline uint8_t *IPDispatchP__pack_nh(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch)
#line 136
{
  uint8_t nxt = hdr->ip6_ctlun.ip6_un1.ip6_un1_nxt;

  if ((((((
#line 138
  nxt == IPV6_HOP || nxt == IPV6_ROUTING) || nxt == IPV6_FRAG) || 
  nxt == IPV6_DEST) || nxt == IPV6_MOBILITY) || nxt == IPV6_IPV6) || 
  nxt == IANA_UDP) {
      *dispatch |= LOWPAN_IPHC_NH_MASK;
    }
  else 
#line 142
    {
      * buf++ = hdr->ip6_ctlun.ip6_un1.ip6_un1_nxt;
    }
  return buf;
}

#line 105
static __inline uint8_t *IPDispatchP__pack_tcfl(uint8_t *buf, struct ip6_hdr *hdr, uint8_t *dispatch)
#line 105
{
  uint32_t flow = ntohl(hdr->ip6_ctlun.ip6_un1.ip6_un1_flow) & 0x000fffff;
  uint8_t tc = (ntohl(hdr->ip6_ctlun.ip6_un1.ip6_un1_flow) >> 20) & 0xff;

#line 108
  if (flow == 0 && tc == 0) {

      *dispatch |= LOWPAN_IPHC_TF_NONE;
    }
  else {
#line 111
    if (flow == 0) {
        *dispatch |= LOWPAN_IPHC_TF_ECN_DSCP;
        *buf = (tc >> 2) & 0xff;
        *buf |= (tc << 6) & 0xff;
        buf++;
      }
    else {
#line 116
      if ((tc & 0x3) == tc) {
          *dispatch |= LOWPAN_IPHC_TF_ECN_FL;
          *buf = (tc << 6) & 0xff;
          *buf |= (flow >> 16) & 0x0f;
          *(buf + 1) = (flow >> 8) & 0xff;
          *(buf + 2) = flow & 0xff;
          buf += 3;
        }
      else 
#line 123
        {
          *dispatch |= LOWPAN_IPHC_TF_ECN_DSCP_FL;
          *buf = (tc >> 2) & 0xff;
          *buf |= (tc << 6) & 0xff;

          *(buf + 1) = (flow >> 16) & 0x0f;
          *(buf + 2) = (flow >> 8) & 0xff;
          *(buf + 3) = flow & 0xff;
          buf += 4;
        }
      }
    }
#line 133
  return buf;
}

#line 413
static inline uint8_t *IPDispatchP__lowpan_pack_headers(struct ip6_packet *packet, 
struct ieee154_frame_addr *frame, 
uint8_t *buf, size_t cnt)
#line 415
{
  uint8_t *dispatch;
#line 416
  uint8_t temp_dispatch;
#line 416
  uint8_t ctx_match_length;

  if ((packet->ip6_hdr.ip6_ctlun.ip6_un2_vfc & 0xf0) != 0x60) {
      return (void *)0;
    }
#line 433
  dispatch = buf;
  *dispatch = LOWPAN_DISPATCH_BYTE_VAL;
  *(dispatch + 1) = 0;
  buf += 2;

  buf = IPDispatchP__pack_tcfl(buf, & packet->ip6_hdr, dispatch);
  buf = IPDispatchP__pack_nh(buf, & packet->ip6_hdr, dispatch);
  buf = IPDispatchP__pack_hlim(buf, & packet->ip6_hdr, dispatch);


  ctx_match_length = IPDispatchP__lowpan_extern_match_context(& packet->ip6_hdr.ip6_src, &temp_dispatch);
  temp_dispatch = 0;
  buf = IPDispatchP__pack_address(buf, & packet->ip6_hdr.ip6_src, ctx_match_length, 
  & frame->ieee_src, frame->ieee_dstpan, &temp_dispatch);
  *(dispatch + 1) |= temp_dispatch << LOWPAN_IPHC_SAM_SHIFT;

  if (packet->ip6_hdr.ip6_dst.in6_u.u6_addr8[0] != 0xff) {

      ctx_match_length = IPDispatchP__lowpan_extern_match_context(& packet->ip6_hdr.ip6_dst, &temp_dispatch);
      temp_dispatch = 0;
      buf = IPDispatchP__pack_address(buf, & packet->ip6_hdr.ip6_dst, ctx_match_length, 
      & frame->ieee_dst, frame->ieee_dstpan, &temp_dispatch);
      *(dispatch + 1) |= temp_dispatch << LOWPAN_IPHC_DAM_SHIFT;
    }
  else 
#line 456
    {

      buf = IPDispatchP__pack_multicast(buf, & packet->ip6_hdr.ip6_dst, &temp_dispatch);
      *(dispatch + 1) |= (temp_dispatch << LOWPAN_IPHC_DAM_SHIFT) | LOWPAN_IPHC_AM_M;
    }

  return buf;
}

# 102 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_frag.c"
static inline int IPDispatchP__lowpan_frag_get(uint8_t *frag, size_t len, 
struct ip6_packet *packet, 
struct ieee154_frame_addr *frame, 
struct lowpan_ctx *ctx)
#line 105
{
  uint8_t *buf;
#line 106
  uint8_t *lowpan_buf;
#line 106
  uint8_t *ieee_buf = frag;
  uint16_t extra_payload;


  buf = lowpan_buf = IPDispatchP__pack_ieee154_header(frag, len, frame);
  if (ctx->offset == 0) {
      int offset = 0;








      buf = IPDispatchP__lowpan_pack_headers(packet, frame, buf, len - (buf - frag));
      if (!buf) {
#line 122
        return -1;
        }

      offset = IPDispatchP__pack_nhc_chain(&buf, len - (buf - ieee_buf), packet);
      if (offset < 0) {
#line 126
        return -2;
        }


      extra_payload = ((((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff) - offset;


      if (extra_payload > len - (buf - ieee_buf)) {
          struct packed_lowmsg lowmsg;

#line 135
          memmove(lowpan_buf + LOWMSG_FRAG1_LEN, 
          lowpan_buf, 
          buf - lowpan_buf);

          lowmsg.data = lowpan_buf;
          lowmsg.len = LOWMSG_FRAG1_LEN;
          lowmsg.headers = 0;
          IPDispatchP__setupHeaders(&lowmsg, LOWMSG_FRAG1_HDR);
          IPDispatchP__setFragDgramSize(&lowmsg, ((((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff) + sizeof(struct ip6_hdr ));
          IPDispatchP__setFragDgramTag(&lowmsg, ctx->tag);

          lowpan_buf += LOWMSG_FRAG1_LEN;
          buf += LOWMSG_FRAG1_LEN;

          extra_payload = len - (buf - ieee_buf);
          extra_payload -= extra_payload % 8;
        }


      if (iov_read(packet->ip6_data, offset, extra_payload, buf) != extra_payload) {
          return -3;
        }

      ctx->offset = offset + extra_payload + sizeof(struct ip6_hdr );
      return buf - frag + extra_payload;
    }
  else 
#line 160
    {
      struct packed_lowmsg lowmsg;

#line 162
      buf = lowpan_buf = IPDispatchP__pack_ieee154_header(frag, len, frame);


      lowmsg.data = lowpan_buf;
      lowmsg.len = LOWMSG_FRAGN_LEN;
      lowmsg.headers = 0;
      IPDispatchP__setupHeaders(&lowmsg, LOWMSG_FRAGN_HDR);
      if (IPDispatchP__setFragDgramSize(&lowmsg, ((((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff) + sizeof(struct ip6_hdr ))) {
        return -5;
        }
#line 171
      if (IPDispatchP__setFragDgramTag(&lowmsg, ctx->tag)) {
        return -6;
        }
#line 173
      if (IPDispatchP__setFragDgramOffset(&lowmsg, ctx->offset / 8)) {
        return -7;
        }
#line 175
      buf += LOWMSG_FRAGN_LEN;

      extra_payload = ((((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )packet->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff) + sizeof(struct ip6_hdr ) - ctx->offset;
      if (extra_payload > len - (buf - ieee_buf)) {
          extra_payload = len - (buf - ieee_buf);
          extra_payload -= extra_payload % 8;
        }

      if (iov_read(packet->ip6_data, ctx->offset - sizeof(struct ip6_hdr ), extra_payload, buf) != extra_payload) {
          return -4;
        }

      ctx->offset += extra_payload;

      if (extra_payload == 0) {
#line 189
        return 0;
        }
      else {
#line 190
        return lowpan_buf - ieee_buf + LOWMSG_FRAGN_LEN + extra_payload;
        }
    }
}

# 102 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__BarePacket__clear(message_t *msg)
#line 102
{
  memset(msg, 0, sizeof(message_t ));
}

# 65 "/opt/tinyos/tos/interfaces/Packet.nc"
inline static void IPDispatchP__BarePacket__clear(message_t * msg){
#line 65
  CC2420TinyosNetworkP__BarePacket__clear(msg);
#line 65
}
#line 65
# 88 "/opt/tinyos/tos/system/PoolP.nc"
static inline /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t */*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get(void )
#line 88
{
  if (/*IPDispatchC.FragPool.PoolP*/PoolP__0__free) {
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t *rval = /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[/*IPDispatchC.FragPool.PoolP*/PoolP__0__index];

#line 91
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[/*IPDispatchC.FragPool.PoolP*/PoolP__0__index] = (void *)0;
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__free--;
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__index++;
      if (/*IPDispatchC.FragPool.PoolP*/PoolP__0__index == 12) {
          /*IPDispatchC.FragPool.PoolP*/PoolP__0__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static IPDispatchP__FragPool__t * IPDispatchP__FragPool__get(void ){
#line 97
  nx_struct message_t *__nesc_result;
#line 97

#line 97
  __nesc_result = /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 88 "/opt/tinyos/tos/system/PoolP.nc"
static inline /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t */*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get(void )
#line 88
{
  if (/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free) {
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t *rval = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index];

#line 91
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index] = (void *)0;
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free--;
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index++;
      if (/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index == 12) {
          /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static IPDispatchP__SendEntryPool__t * IPDispatchP__SendEntryPool__get(void ){
#line 97
  struct send_entry *__nesc_result;
#line 97

#line 97
  __nesc_result = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 88 "/opt/tinyos/tos/system/PoolP.nc"
static inline /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t */*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get(void )
#line 88
{
  if (/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free) {
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t *rval = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index];

#line 91
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index] = (void *)0;
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free--;
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index++;
      if (/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index == 3) {
          /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static IPDispatchP__SendInfoPool__t * IPDispatchP__SendInfoPool__get(void ){
#line 97
  struct send_info *__nesc_result;
#line 97

#line 97
  __nesc_result = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 140 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline struct send_info *IPDispatchP__getSendInfo(void )
#line 140
{
  struct send_info *ret = IPDispatchP__SendInfoPool__get();

#line 142
  if (ret == (void *)0) {
#line 142
    return ret;
    }
#line 143
  ret->_refcount = 1;
  ret->upper_data = (void *)0;
  ret->failed = FALSE;
  ret->link_transmissions = 0;
  ret->link_fragments = 0;
  ret->link_fragment_attempts = 0;
  return ret;
}

#line 486
static inline error_t IPDispatchP__IPLower__send(struct ieee154_frame_addr *frame_addr, 
struct ip6_packet *msg, 
void *data)
#line 488
{
  struct lowpan_ctx ctx;
  struct send_info *s_info;
  struct send_entry *s_entry;
  message_t *outgoing;

  int frag_len = 1;
  error_t rc = SUCCESS;

  if (IPDispatchP__state != IPDispatchP__S_RUNNING) {
      return EOFF;
    }


  msg->ip6_hdr.ip6_ctlun.ip6_un2_vfc &= ~0xf0;
  msg->ip6_hdr.ip6_ctlun.ip6_un2_vfc |= 0x60;

  ctx.tag = IPDispatchP__current_local_label++;
  ctx.offset = 0;

  s_info = IPDispatchP__getSendInfo();
  if (s_info == (void *)0) {
      rc = ERETRY;
      goto cleanup_outer;
    }
  s_info->upper_data = data;

  while (frag_len > 0) {
      s_entry = IPDispatchP__SendEntryPool__get();
      outgoing = IPDispatchP__FragPool__get();

      if (s_entry == (void *)0 || outgoing == (void *)0) {
          if (s_entry != (void *)0) {
            IPDispatchP__SendEntryPool__put(s_entry);
            }
#line 522
          if (outgoing != (void *)0) {
            IPDispatchP__FragPool__put(outgoing);
            }

          s_info->failed = TRUE;
          ;
#line 527
          ;
          rc = ERETRY;
          goto done;
        }

      IPDispatchP__BarePacket__clear(outgoing);
      frag_len = IPDispatchP__lowpan_frag_get(IPDispatchP__Ieee154Send__getPayload(outgoing, 0), 
      IPDispatchP__BarePacket__maxPayloadLength(), 
      msg, 
      frame_addr, 
      &ctx);
      if (frag_len < 0) {
          ;
#line 539
          ;
        }

      ;
#line 542
      ;
      IPDispatchP__BarePacket__setPayloadLength(outgoing, frag_len);

      if (frag_len <= 0) {
          IPDispatchP__FragPool__put(outgoing);
          IPDispatchP__SendEntryPool__put(s_entry);
          goto done;
        }

      if (IPDispatchP__SendQueue__enqueue(s_entry) != SUCCESS) {
          ;
          s_info->failed = TRUE;
          ;
#line 554
          ;
          goto done;
        }

      s_info->link_fragments++;
      s_entry->msg = outgoing;
      s_entry->info = s_info;


      if (frame_addr->ieee_dst.ieee_mode == IEEE154_ADDR_SHORT && 
      frame_addr->ieee_dst.ieee_addr.saddr == IEEE154_BROADCAST_ADDR) {
          IPDispatchP__PacketLink__setRetries(s_entry->msg, 0);
        }
      else 
#line 566
        {
          IPDispatchP__PacketLink__setRetries(s_entry->msg, 5);
        }
      IPDispatchP__PacketLink__setRetryDelay(s_entry->msg, 103);

      s_info->_refcount++;
    }

  done: 
    ;
  IPDispatchP__SENDINFO_DECR(s_info);
  IPDispatchP__sendTask__postTask();
  cleanup_outer: 
    return rc;
}

# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
inline static error_t IPNeighborDiscoveryP__IPLower__send(struct ieee154_frame_addr *next_hop, struct ip6_packet *msg, void *data){
#line 18
  unsigned char __nesc_result;
#line 18

#line 18
  __nesc_result = IPDispatchP__IPLower__send(next_hop, msg, data);
#line 18

#line 18
  return __nesc_result;
#line 18
}
#line 18
# 63 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static inline int IPDispatchP__iid_eui_cmp(uint8_t *iid, uint8_t *eui)
#line 63
{
  return iid[0] == (eui[7] ^ 0x2) && 
  iid[1] == eui[6] && 
  iid[2] == eui[5] && 
  iid[3] == eui[4] && 
  iid[4] == eui[3] && 
  iid[5] == eui[2] && 
  iid[6] == eui[1] && 
  iid[7] == eui[0];
}

# 103 "/opt/tinyos/tos/system/PoolP.nc"
static inline error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t *newVal)
#line 103
{
  if (/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free >= 3) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index + /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free;

#line 109
      if (emptyIndex >= 3) {
          emptyIndex -= 3;
        }
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[emptyIndex] = newVal;
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free++;
      ;
      return SUCCESS;
    }
}

# 89 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static error_t IPDispatchP__SendInfoPool__put(IPDispatchP__SendInfoPool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
inline static error_t IPForwardingEngineP__Pool__put(IPForwardingEngineP__Pool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 748 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline void CoapUdpServerP__WriteResource__putDone(uint8_t uri_key, error_t result, 
coap_tid_t id, 
uint8_t asyn_message)
#line 750
{

  coap_queue_t *node;
  coap_pdu_t *pdu;
  coap_opt_t *tok;
#line 754
  coap_opt_t *ct;



  if (!(node = coap_find_transaction(CoapUdpServerP__ctx_server->splitphasequeue, id))) {

      return;
    }

  if (result) {

      if (!(pdu = CoapUdpServerP__new_response(CoapUdpServerP__ctx_server, node, 200))) {
        }
    }
  else 


    {

      if (asyn_message) {
          node->pdu->hdr->id = CoapUdpServerP__get_new_tid();
          if (!(pdu = CoapUdpServerP__new_asynresponse(CoapUdpServerP__ctx_server, node))) {
            }
        }
      else {
          if (!(pdu = CoapUdpServerP__new_response(CoapUdpServerP__ctx_server, node, 80))) {
            }
        }
    }



  ct = coap_check_option(node->pdu, 1);
  if (ct) {
      coap_add_option(pdu, 1, ct->lval.flag == 15 ? ct->lval.length + 15 : ct->sval.length, (unsigned char *)& *ct + (ct->lval.flag == 15 ? 2 : 1));
    }


  tok = coap_check_option(node->pdu, 11);
  if (tok) {
    coap_add_option(pdu, 11, tok->lval.flag == 15 ? tok->lval.length + 15 : tok->sval.length, (unsigned char *)& *tok + (tok->lval.flag == 15 ? 2 : 1));
    }

  if (pdu && CoapUdpServerP__LibCoapServer__send(CoapUdpServerP__ctx_server, & node->remote, pdu, 1) == -1) {

      coap_delete_pdu(pdu);
    }


  CoapUdpServerP__coap_extract_node(& CoapUdpServerP__ctx_server->splitphasequeue, node);
  node->next = (void *)0;
  coap_delete_node(node);
}

# 35 "/opt/tinyos/tos/interfaces/WriteResource.nc"
inline static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__putDone(error_t result, coap_tid_t id, uint8_t asyn_message){
#line 35
  CoapUdpServerP__WriteResource__putDone(KEY_LED, result, id, asyn_message);
#line 35
}
#line 35
# 63 "/opt/tinyos/tos/lib/net/coap/CoapLedResourceP.nc"
static inline void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__runTask(void )
#line 63
{
  /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock = FALSE;
  /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__putDone(SUCCESS, /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__temp_id, 0);
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 57 "/opt/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__size(void )
#line 57
{
  return /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__size;
}

# 58 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__size(void ){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__size();
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 89 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__t * newVal){
#line 89
  unsigned char __nesc_result;
#line 89

#line 89
  __nesc_result = /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__put(newVal);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = ICMPCoreP__ICMP_IP__send(155, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 46 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline error_t /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__send(uint8_t code, struct ip6_packet *msg)
#line 46
{
  return /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__send(msg);
}

# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__send(ICMPV6_CODE_DAO, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 113 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getDefaultRoute(struct in6_addr *next_hop){
#line 113
  unsigned char __nesc_result;
#line 113

#line 113
  __nesc_result = RPLRankP__RPLRankInfo__getDefaultRoute(next_hop);
#line 113

#line 113
  return __nesc_result;
#line 113
}
#line 113
# 432 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDefaultRoute(struct in6_addr *next)
#line 432
{
  return /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getDefaultRoute(next);
}

# 43 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDefaultRoute(struct in6_addr *next_hop){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDefaultRoute(next_hop);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__getLLAddr(struct in6_addr *addr){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = IPAddressP__IPAddress__getLLAddr(addr);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 69 "/opt/tinyos/tos/system/QueueC.nc"
static inline void /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__printQueue(void )
#line 69
{
}

#line 53
static inline bool /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__empty(void )
#line 53
{
  return /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__size == 0;
}









static inline /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__head(void )
#line 65
{
  return /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue[/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__head];
}

#line 85
static inline /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__dequeue(void )
#line 85
{
  /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t t = /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__head();

#line 87
  ;
  if (!/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__empty()) {
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__head++;
      if (/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__head == 5) {
#line 90
        /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__head = 0;
        }
#line 91
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__size--;
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__printQueue();
    }
  return t;
}

# 81 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__t  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__dequeue(void ){
#line 81
  struct __nesc_unnamed4340 *__nesc_result;
#line 81

#line 81
  __nesc_result = /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 428 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getMOP(void )
#line 428
{
  return /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MOP;
}

# 52 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getMOP(void ){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getMOP();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 84 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
inline static uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(struct in6_addr *node){
#line 84
  unsigned int __nesc_result;
#line 84

#line 84
  __nesc_result = RPLRankP__RPLRankInfo__getRank(node);
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 468 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getRank(void )
#line 468
{
  return /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP);
}

# 44 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static uint16_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getRank(void ){
#line 44
  unsigned int __nesc_result;
#line 44

#line 44
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getRank();
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 107 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__runTask(void )
#line 107
{
  dao_entry_t *dao_msg;




  struct in6_addr next_hop;
  struct dao_base_t *dao;

  if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getRank() == ROOT_RANK) {
      return;
    }

  if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__size() > 0 && /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getMOP() != 0) {
      dao_msg = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__dequeue();




      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__getLLAddr(& dao_msg->s_pkt.ip6_hdr.ip6_src);
      if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDefaultRoute(&next_hop) != SUCCESS) {
          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(dao_msg);
          ;
#line 129
          ;
          return;
        }
      ip_memcpy((uint8_t *)& dao_msg->s_pkt.ip6_hdr.ip6_dst, 
      (uint8_t *)&next_hop, sizeof(struct in6_addr ));






      dao = (struct dao_base_t *)dao_msg->s_pkt.ip6_data->iov_base;

      ;
#line 142
      ;
      ;
#line 143
      ;

      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__send(& dao_msg->s_pkt);
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(dao_msg);

      if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__size()) {


          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__postTask();
        }
    }
}

# 60 "/opt/tinyos/tos/lib/net/blip/IPAddressP.nc"
static inline bool IPAddressP__IPAddress__getGlobalAddr(struct in6_addr *addr)
#line 60
{
  *addr = IPAddressP__m_addr;
  return IPAddressP__m_valid_addr;
}

# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool RPLRankP__IPAddress__getGlobalAddr(struct in6_addr *addr){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = IPAddressP__IPAddress__getGlobalAddr(addr);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t ICMPCoreP__IP__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = IPProtocolsP__IP__send(IANA_ICMP, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 64 "/opt/tinyos/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 64
  unsigned long __nesc_result;
#line 64

#line 64
  __nesc_result = /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 86 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
}

# 109 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 109
  unsigned long __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getNow();
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
# 159 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(6U, dt);
#line 73
}
#line 73
# 169 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__isRunning(void ){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(6U);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
# 90 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__enqueue(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__t  newVal){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__enqueue(newVal);
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__getGlobalAddr(struct in6_addr *addr){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = IPAddressP__IPAddress__getGlobalAddr(addr);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 45 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getInstanceID(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getInstanceID();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 97 "/opt/tinyos/tos/interfaces/Pool.nc"
inline static /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__t * /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__get(void ){
#line 97
  struct __nesc_unnamed4340 *__nesc_result;
#line 97

#line 97
  __nesc_result = /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__get();
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 424 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__hasDODAG(void )
#line 424
{
  return /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG;
}

# 42 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__hasDODAG(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__hasDODAG();
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 209 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__runTask(void )
#line 209
{
  error_t error;
  dao_entry_t *dao_msg;
  uint16_t length = sizeof(struct dao_base_t );

  if (!/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__hasDODAG() || 
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getRank() == ROOT_RANK) {
      ;
#line 216
      ;
      return;
    }

  dao_msg = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__get();
  if (dao_msg == (void *)0) {
      ;
#line 222
      ;
      return;
    }


  dao_msg->dao_base.icmpv6.type = ICMP_TYPE_RPL_CONTROL;
  dao_msg->dao_base.icmpv6.code = ICMPV6_CODE_DAO;
  __nesc_hton_uint16(dao_msg->dao_base.icmpv6.checksum.nxdata, 0);
  dao_msg->dao_base.DAOsequence = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__daoseq;

  dao_msg->dao_base.instance_id.id = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getInstanceID();

  dao_msg->dao_base.target_option.type = RPL_TARGET_TYPE;
  dao_msg->dao_base.target_option.option_length = 18;

  dao_msg->dao_base.target_option.prefix_length = sizeof(struct in6_addr ) * 8;


  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__getGlobalAddr(& dao_msg->dao_base.target_option.target_prefix);




  dao_msg->dao_base.transit_info_option.type = RPL_TRANSIT_INFORMATION_TYPE;
  dao_msg->dao_base.transit_info_option.option_length = 22;
  dao_msg->dao_base.transit_info_option.path_sequence = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__PATH_SEQUENCE;
  dao_msg->dao_base.transit_info_option.path_control = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__PATH_CONTROL;
  dao_msg->dao_base.transit_info_option.path_lifetime = DEFAULT_LIFETIME;
  if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDefaultRoute(& dao_msg->dao_base.transit_info_option.parent_address) != SUCCESS) {
      ;
#line 251
      ;
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(dao_msg);
      return;
    }

  dao_msg->v[0].iov_base = (uint8_t *)& dao_msg->dao_base;
  dao_msg->v[0].iov_len = length;
  dao_msg->v[0].iov_next = (void *)0;

  dao_msg->s_pkt.ip6_hdr.ip6_ctlun.ip6_un2_vfc = 0x60;
  dao_msg->s_pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IANA_ICMP;
  dao_msg->s_pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )length << 8) | ((uint16_t )length >> 8)) & 0xffff;
  dao_msg->s_pkt.ip6_data = &dao_msg->v[0];

  error = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__enqueue(dao_msg);

  if (error != SUCCESS) {
      ;
#line 268
      ;
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(dao_msg);
      return;
    }
  else 
#line 271
    {

      if (!/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__isRunning()) {
          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__startOneShot(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__delay_dao);
        }
    }
}

# 61 "/opt/tinyos/tos/system/QueueC.nc"
static inline uint8_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__maxSize(void )
#line 61
{
  return 5;
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
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t RPLRankP__IP_DIO__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__send(ICMPV6_CODE_DIO, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 959 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline error_t RPLRankP__IP_DIO_Filter__send(struct ip6_packet *msg)
#line 959
{
  return RPLRankP__IP_DIO__send(msg);
}

# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = RPLRankP__IP_DIO_Filter__send(msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getLLAddr(struct in6_addr *addr){
#line 29
  unsigned char __nesc_result;
#line 29

#line 29
  __nesc_result = IPAddressP__IPAddress__getLLAddr(addr);
#line 29

#line 29
  return __nesc_result;
#line 29
}
#line 29
# 82 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline uint16_t RPLOF0P__RPLOF__getObjectValue(void )
#line 82
{
  return RPLOF0P__nodeEtx;
}

# 9 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static uint16_t RPLRankP__RPLOF__getObjectValue(void ){
#line 9
  unsigned int __nesc_result;
#line 9

#line 9
  __nesc_result = RPLOF0P__RPLOF__getObjectValue();
#line 9

#line 9
  return __nesc_result;
#line 9
}
#line 9
# 309 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline uint16_t RPLRankP__RPLRankInfo__getEtx(void )
#line 309
{
  return RPLRankP__RPLOF__getObjectValue();
}

# 101 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
inline static uint16_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getEtx(void ){
#line 101
  unsigned int __nesc_result;
#line 101

#line 101
  __nesc_result = RPLRankP__RPLRankInfo__getEtx();
#line 101

#line 101
  return __nesc_result;
#line 101
}
#line 101
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getGlobalAddr(struct in6_addr *addr){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = IPAddressP__IPAddress__getGlobalAddr(addr);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 171 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__runTask(void )
#line 171
{
  struct ip6_packet pkt;
  struct ip_iovec v[1];
  uint8_t data[60];
  struct dio_base_t msg;
  struct dio_body_t body;
  struct dio_metric_header_t metric_header;
  struct dio_etx_t etx_value;
  struct dio_dodag_config_t dodag_config;
  uint16_t length;






  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running || !/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG) {

      return;
    }




  msg.icmpv6.type = ICMP_TYPE_RPL_CONTROL;
  msg.icmpv6.code = ICMPV6_CODE_DIO;
  __nesc_hton_uint16(msg.icmpv6.checksum.nxdata, 0);
  msg.flags = 0;
  msg.flags = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__GROUND_STATE << 7;
  msg.flags |= /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MOP << 3;
  msg.flags |= /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DAG_PREF << 0;
  __nesc_hton_uint8(msg.version.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGVersionNumber);
  msg.instance_id.id = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLInstanceID;
  msg.dtsn = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DTSN;
  memcpy(& msg.dodagID, &/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID, sizeof(struct in6_addr ));

  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_ROOT) {

      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getGlobalAddr(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID);



      __nesc_hton_uint16(msg.dagRank.nxdata, ROOT_RANK);
    }
  else 
#line 214
    {
      __nesc_hton_uint16(msg.dagRank.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP));
    }

  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_LEAF) {
      __nesc_hton_uint8(dodag_config.type.nxdata, RPL_DODAG_CONFIG_TYPE);
      __nesc_hton_uint8(dodag_config.length.nxdata, 14);
      dodag_config.flags = 0;
      dodag_config.A = 0;
      dodag_config.PCS = 0;



      __nesc_hton_uint16(dodag_config.ocp.nxdata, 0);

      __nesc_hton_uint8(dodag_config.default_lifetime.nxdata, 0xFF);
      __nesc_hton_uint16(dodag_config.lifetime_unit.nxdata, 0xFFFF);
      __nesc_hton_uint8(dodag_config.DIOIntDoubl.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntDouble);
      __nesc_hton_uint8(dodag_config.DIOIntMin.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntMin);
      __nesc_hton_uint8(dodag_config.DIORedun.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIORedun);
      __nesc_hton_uint16(dodag_config.MaxRankInc.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MaxRankInc);
      __nesc_hton_uint16(dodag_config.MinHopRankInc.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MinHopRankInc);
      __nesc_hton_uint8(dodag_config.reserved.nxdata, 0);


      __nesc_hton_uint16(etx_value.etx.nxdata, /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getEtx());

      metric_header.routing_obj_type = RPL_ROUTE_METRIC_ETX;
      metric_header.reserved = 0;
      metric_header.R_flag = 0;
      metric_header.G_flag = 1;
      metric_header.A_flag = 0;
      metric_header.O_flag = 0;
      metric_header.C_flag = 0;
      __nesc_hton_uint16(metric_header.object_len.nxdata, 2);

      body.type = RPL_DODAG_METRIC_CONTAINER_TYPE;
      body.container_len = 6;

      {
        uint8_t *cur = (uint8_t *)&data;

#line 267
        length = sizeof(struct dio_base_t ) + sizeof(struct dio_dodag_config_t );
        ip_memcpy(cur, (uint8_t *)&msg, sizeof(struct dio_base_t ));
#line 268
        cur += sizeof(struct dio_base_t );
#line 268
        ;
        ip_memcpy(cur, (uint8_t *)&dodag_config, sizeof(struct dio_dodag_config_t ));
#line 269
        cur += sizeof(struct dio_dodag_config_t );
#line 269
        ;
      }




      v[0].iov_base = (uint8_t *)&data;
      v[0].iov_len = length;
      v[0].iov_next = (void *)0;

      pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IANA_ICMP;
      pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )length << 8) | ((uint16_t )length >> 8)) & 0xffff;

      pkt.ip6_data = &v[0];
    }
  else 

    {
      length = sizeof(struct dio_base_t );
      pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IANA_ICMP;
      pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )length << 8) | ((uint16_t )length >> 8)) & 0xffff;

      v[0].iov_base = (uint8_t *)&msg;
      v[0].iov_len = sizeof(struct dio_base_t );
      v[0].iov_next = (void *)0;

      pkt.ip6_data = &v[0];
    }




  ;
#line 301
  ;





  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO) {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO = FALSE;
      memcpy(& pkt.ip6_hdr.ip6_dst, &/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO_ADDR, 16);
    }
  else 
#line 310
    {
      memcpy(& pkt.ip6_hdr.ip6_dst, &/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR, 16);
    }

  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getLLAddr(& pkt.ip6_hdr.ip6_src);




  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__send(&pkt);
}

# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = ICMPCoreP__ICMP_IP__send(155, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 46 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline error_t /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__send(uint8_t code, struct ip6_packet *msg)
#line 46
{
  return /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__send(msg);
}

# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__send(struct ip6_packet *msg){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__send(ICMPV6_CODE_DIS, msg);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 322 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__runTask(void )
#line 322
{
  struct ip6_packet pkt;
  struct ip_iovec v[1];
  struct dis_base_t msg;
  uint16_t length;

  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running) {
    return;
    }
  length = sizeof(struct dis_base_t );
  msg.icmpv6.type = ICMP_TYPE_RPL_CONTROL;
  msg.icmpv6.code = ICMPV6_CODE_DIS;
  __nesc_hton_uint16(msg.icmpv6.checksum.nxdata, 0);


  pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IANA_ICMP;
  pkt.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )length << 8) | ((uint16_t )length >> 8)) & 0xffff;

  v[0].iov_base = (uint8_t *)&msg;
  v[0].iov_len = sizeof(struct dis_base_t );
  v[0].iov_next = (void *)0;
  pkt.ip6_data = &v[0];

  memcpy(& pkt.ip6_hdr.ip6_dst, &/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR, 16);
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getLLAddr(& pkt.ip6_hdr.ip6_src);





  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__send(&pkt);
}

# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(4U, dt);
#line 64
}
#line 64
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 125 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__runTask(void )
#line 125
{

  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MOP = RPL_MOP_Storing_No_Multicast;





  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getGlobalAddr(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP);




  ROOT_RANK = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MinHopRankInc;


  memset(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR.in6_u.u6_addr8, 0, 16);
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR.in6_u.u6_addr8[0] = 0xFF;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR.in6_u.u6_addr8[1] = 0x2;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MULTICAST_ADDR.in6_u.u6_addr8[15] = 0x1A;

  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_ROOT) {

      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__getGlobalAddr(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID);





      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__postTask();
    }
  else 
#line 155
    {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__startPeriodic(DIS_INTERVAL);
    }
}


static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__runTask(void )
#line 161
{
  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_ROOT) {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
    }
}

# 164 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 78 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(3U);
#line 78
}
#line 78
# 383 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__resetTrickleTime(void )
#line 383
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__stop();
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__tricklePeriod = 2 << (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntMin - 1);
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__redunCounter = 0;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__doubleCounter = 0;
}

# 46 "/opt/tinyos/tos/interfaces/Random.nc"
inline static uint32_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__Random__rand32(void ){
#line 46
  unsigned long __nesc_result;
#line 46

#line 46
  __nesc_result = RandomMlcgC__Random__rand32();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 401 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeTrickleRemaining(void )
#line 401
{

  uint32_t remain;

#line 404
  remain = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__tricklePeriod - /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sentDIOFlag = TRUE;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__startOneShot(remain);
}

#line 167
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__runTask(void )
#line 167
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeTrickleRemaining();
}

# 20 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static bool RPLRankP__RPLOF__recomputeRoutes(void ){
#line 20
  unsigned char __nesc_result;
#line 20

#line 20
  __nesc_result = RPLOF0P__RPLOF__recomputeRoutes();
#line 20

#line 20
  return __nesc_result;
#line 20
}
#line 20
# 362 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline void RPLRankP__newParentSearch__runTask(void )
#line 362
{

  RPLRankP__RPLOF__recomputeRoutes();
  RPLRankP__getNewRank();
}

# 94 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
inline static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__inconsistencyDetected(void ){
#line 94
  RPLRankP__RPLRankInfo__inconsistencyDetected();
#line 94
}
#line 94
# 357 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__inconsistencyDetected(void )
#line 357
{

  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__INCONSISTENCY_COUNT++;


  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__inconsistencyDetected();



  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
}

#line 420
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__inconsistency(void )
#line 420
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__inconsistencyDetected();
}

# 56 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static void RPLOF0P__RPLRoute__inconsistency(void ){
#line 56
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__inconsistency();
#line 56
}
#line 56
# 16 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
inline static error_t RPLOF0P__ForwardingTable__delRoute(route_key_t key){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = IPForwardingEngineP__ForwardingTable__delRoute(key);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 369 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteRemoved(void )
#line 369
{
}

# 50 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTableEvents.nc"
inline static void IPForwardingEngineP__ForwardingTableEvents__defaultRouteRemoved(void ){
#line 50
  IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteRemoved();
#line 50
}
#line 50
# 10 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
inline static route_key_t RPLOF0P__ForwardingTable__addRoute(const uint8_t *prefix, int prefix_len_bits, struct in6_addr *next_hop, uint8_t ifindex){
#line 10
  int __nesc_result;
#line 10

#line 10
  __nesc_result = IPForwardingEngineP__ForwardingTable__addRoute(prefix, prefix_len_bits, next_hop, ifindex);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 57 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline int IPForwardingEngineP__alloc_key(void )
#line 57
{
  int i;
  int key;

#line 60
  retry: 
    key = IPForwardingEngineP__last_key++;
  for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (IPForwardingEngineP__routing_table[i].valid && IPForwardingEngineP__routing_table[i].key == key) {
        goto retry;
        }
    }
#line 66
  return key;
}

static inline struct route_entry *IPForwardingEngineP__alloc_entry(int pfxlen)
#line 69
{
  int i;

  if (IPForwardingEngineP__routing_table[ROUTE_TABLE_SZ - 1].valid) {
#line 72
    return (void *)0;
    }
  for (i = 0; i < ROUTE_TABLE_SZ; i++) {


      if (! IPForwardingEngineP__routing_table[i].valid) {
#line 77
        goto init_entry;
        }
      else {
        if (IPForwardingEngineP__routing_table[i].prefixlen >= pfxlen) {
#line 80
          continue;
          }
        }


      memmove((void *)&IPForwardingEngineP__routing_table[i + 1], (void *)&IPForwardingEngineP__routing_table[i], 
      sizeof(struct route_entry ) * (ROUTE_TABLE_SZ - i - 1));
      goto init_entry;
    }
  return (void *)0;
  init_entry: 
    IPForwardingEngineP__routing_table[i].valid = 1;
  IPForwardingEngineP__routing_table[i].key = IPForwardingEngineP__alloc_key();
  return &IPForwardingEngineP__routing_table[i];
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t IPForwardingEngineP__defaultRouteAddedTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(IPForwardingEngineP__defaultRouteAddedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 419 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__newParent(void )
#line 419
{






  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__postTask();
}

# 42 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngine.nc"
inline static void RPLOF0P__RPLDAO__newParent(void ){
#line 42
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__newParent();
#line 42
}
#line 42
# 163 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline parent_t *RPLRankP__RPLParentTable__get(uint8_t i)
#line 163
{
  return &RPLRankP__parentSet[i];
}

# 2 "/opt/tinyos/tos/lib/net/rpl/RPLParentTable.nc"
inline static parent_t *RPLOF0P__ParentTable__get(uint8_t parent_index){
#line 2
  struct __nesc_unnamed4345 *__nesc_result;
#line 2

#line 2
  __nesc_result = RPLRankP__RPLParentTable__get(parent_index);
#line 2

#line 2
  return __nesc_result;
#line 2
}
#line 2
# 97 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline bool RPLOF0P__RPLOF__recalcualateRank(void )
#line 97
{
  uint16_t prevEtx;
#line 98
  uint16_t prevRank;
  parent_t *parentNode = RPLOF0P__ParentTable__get(RPLOF0P__desiredParent);

  if (RPLOF0P__desiredParent == 20) {
      RPLOF0P__nodeRank = INFINITE_RANK;
      return FALSE;
    }

  prevEtx = RPLOF0P__nodeEtx;
  prevRank = RPLOF0P__nodeRank;


  RPLOF0P__nodeEtx = parentNode->etx_hop;
  RPLOF0P__nodeRank = parentNode->rank + RPLOF0P__min_hop_rank_inc;

  if (RPLOF0P__nodeRank < RPLOF0P__min_hop_rank_inc) {
    RPLOF0P__nodeRank = INFINITE_RANK;
    }
  if (RPLOF0P__newParent) {
      RPLOF0P__newParent = FALSE;
      return TRUE;
    }
  else 
#line 119
    {
      return FALSE;
    }
}

# 17 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static bool RPLRankP__RPLOF__recalcualateRank(void ){
#line 17
  unsigned char __nesc_result;
#line 17

#line 17
  __nesc_result = RPLOF0P__RPLOF__recalcualateRank();
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 93 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline uint16_t RPLOF0P__RPLOF__getRank(void )
#line 93
{
  return RPLOF0P__nodeRank;
}

# 14 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static uint16_t RPLRankP__RPLOF__getRank(void ){
#line 14
  unsigned int __nesc_result;
#line 14

#line 14
  __nesc_result = RPLOF0P__RPLOF__getRank();
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 56 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static void RPLRankP__RouteInfo__inconsistency(void ){
#line 56
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__inconsistency();
#line 56
}
#line 56
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t RPLRankP__newParentSearch__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(RPLRankP__newParentSearch);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 11 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static struct in6_addr *RPLRankP__RPLOF__getParent(void ){
#line 11
  struct in6_addr *__nesc_result;
#line 11

#line 11
  __nesc_result = RPLOF0P__RPLOF__getParent();
#line 11

#line 11
  return __nesc_result;
#line 11
}
#line 11
# 369 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline void RPLRankP__evictAll(void )
#line 369
{
  uint8_t indexset;
#line 370
  uint8_t myParent;

#line 371
  myParent = RPLRankP__getParent(RPLRankP__RPLOF__getParent());

  for (indexset = 0; indexset < 20; indexset++) {
      if (RPLRankP__parentSet[indexset].valid && RPLRankP__parentSet[indexset].rank >= RPLRankP__nodeRank) {
          RPLRankP__parentSet[indexset].valid = FALSE;
          RPLRankP__parentNum--;
          ;
#line 377
          ;


          if (indexset == myParent) {

              RPLRankP__newParentSearch__postTask();
              return;
            }
        }
    }
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t PacketLinkP__send__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(PacketLinkP__send);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 75 "/opt/tinyos/tos/interfaces/Send.nc"
inline static error_t PacketLinkP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420CsmaP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 59 "/opt/tinyos/tos/interfaces/PacketAcknowledgements.nc"
inline static error_t PacketLinkP__PacketAcknowledgements__requestAck(message_t * msg){
#line 59
  unsigned char __nesc_result;
#line 59

#line 59
  __nesc_result = CC2420PacketP__Acks__requestAck(msg);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 310 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 310
{
  const uint8_t *base = source;

#line 312
  return ((uint16_t )base[0] << 8) | base[1];
}

# 104 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline uint16_t PacketLinkP__PacketLink__getRetries(message_t *msg)
#line 104
{
  return __nesc_ntoh_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->maxRetries.nxdata);
}

#line 209
static inline void PacketLinkP__send__runTask(void )
#line 209
{
  if (PacketLinkP__PacketLink__getRetries(PacketLinkP__currentSendMsg) > 0) {
      PacketLinkP__PacketAcknowledgements__requestAck(PacketLinkP__currentSendMsg);
    }

  if (PacketLinkP__SubSend__send(PacketLinkP__currentSendMsg, PacketLinkP__currentSendLen) != SUCCESS) {
      PacketLinkP__send__postTask();
    }
}

# 327 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 327
{
  uint8_t *base = target;

#line 329
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
#line 53
inline static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 294 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__RadioBackoff__default__requestCca(message_t *msg)
#line 294
{
}

# 95 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg){
#line 95
  CC2420CsmaP__RadioBackoff__default__requestCca(msg);
#line 95
}
#line 95
# 547 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca)
#line 547
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 548
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 550
            ECANCEL;

            {
#line 550
              __nesc_atomic_end(__nesc_atomic); 
#line 550
              return __nesc_temp;
            }
          }
        }
#line 553
      if (CC2420TransmitP__m_state != CC2420TransmitP__S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 554
            FAIL;

            {
#line 554
              __nesc_atomic_end(__nesc_atomic); 
#line 554
              return __nesc_temp;
            }
          }
        }


      CC2420TransmitP__m_state = CC2420TransmitP__S_LOAD;
      CC2420TransmitP__m_cca = cca;
      CC2420TransmitP__m_msg = p_msg;
      CC2420TransmitP__totalCcaChecks = 0;
    }
#line 564
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }

  return SUCCESS;
}

#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca)
#line 192
{
  return CC2420TransmitP__send(p_msg, useCca);
}

# 51 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Transmit.nc"
inline static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420TransmitP__Send__send(p_msg, useCca);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 70 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 45 "/opt/tinyos/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(tx);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 126 "/opt/tinyos/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/opt/tinyos/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 214 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 361 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableRxIntr(void )
#line 361
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 362
    {
      HplMsp430Usart0P__IFG1 &= ~0x40;
      HplMsp430Usart0P__IE1 |= 0x40;
    }
#line 365
    __nesc_atomic_end(__nesc_atomic); }
}

# 180 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr(void ){
#line 180
  HplMsp430Usart0P__Usart__enableRxIntr();
#line 180
}
#line 180
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
inline static error_t CC2420TinyosNetworkP__grantTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420TinyosNetworkP__grantTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 229 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id)
#line 229
{
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP__grantTask__postTask();
    }
  CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
  return SUCCESS;
}

#line 253
static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client)
#line 253
{
  CC2420TinyosNetworkP__Resource__release(client);
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP__Resource__granted(uint8_t arg_0x412b4e60){
#line 102
    CC2420TinyosNetworkP__Resource__default__granted(arg_0x412b4e60);
#line 102
}
#line 102
# 68 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 68
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead;

#line 72
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
            }
#line 75
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 76
            id;

            {
#line 76
              __nesc_atomic_end(__nesc_atomic); 
#line 76
              return __nesc_temp;
            }
          }
        }
#line 78
      {
        unsigned char __nesc_temp = 
#line 78
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

        {
#line 78
          __nesc_atomic_end(__nesc_atomic); 
#line 78
          return __nesc_temp;
        }
      }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

      {
#line 61
        __nesc_atomic_end(__nesc_atomic); 
#line 61
        return __nesc_temp;
      }
    }
#line 63
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/opt/tinyos/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP__Queue__isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 180 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__grantTask__runTask(void )
#line 180
{


  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && !CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__Queue__dequeue();

          if (CC2420TinyosNetworkP__resource_owner != CC2420TinyosNetworkP__OWNER_NONE) {
              CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
            }
        }
    }
  else 
#line 191
    {
      if (CC2420TinyosNetworkP__next_owner != CC2420TinyosNetworkP__resource_owner) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__next_owner;
          CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
        }
    }
}

# 103 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 103
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(t0, dt);
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
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 73 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 73
}
#line 73
# 102 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__stop();
}

# 73 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 73
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__stop();
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

# 71 "/opt/tinyos/tos/interfaces/State.nc"
inline static uint8_t PacketLinkP__SendState__getState(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = StateImplP__State__getState(4U);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 202 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline void PacketLinkP__DelayTimer__fired(void )
#line 202
{
  if (PacketLinkP__SendState__getState() == PacketLinkP__S_SENDING) {
      PacketLinkP__send__postTask();
    }
}

# 276 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__ip_print_heap(void )
#line 276
{
}

#line 251
static inline void IPDispatchP__reconstruct_age(void *elt)
#line 251
{
  struct lowpan_reconstruct *recon = (struct lowpan_reconstruct *)elt;

#line 253
  if (recon->r_timeout != T_UNUSED) {
    ;
    }
#line 254
  ;


  switch (recon->r_timeout) {
      case T_ACTIVE: 
        recon->r_timeout = T_ZOMBIE;
#line 259
      break;
      case T_FAILED1: 
        recon->r_timeout = T_FAILED2;
#line 261
      break;
      case T_ZOMBIE: 
        case T_FAILED2: 

          ;
#line 265
      ;
      if (recon->r_buf != (void *)0) {
          ;
#line 267
          ;
          ip_free(recon->r_buf);
        }
      recon->r_timeout = T_UNUSED;
      recon->r_buf = (void *)0;
      break;
    }
}

#line 287
static inline void IPDispatchP__ExpireTimer__fired(void )
#line 287
{
  table_map(&IPDispatchP__recon_cache, IPDispatchP__reconstruct_age);


  ;
#line 291
  ;
  ;
#line 292
  ;
  ;
#line 293
  ;
  ;
#line 294
  ;
  IPDispatchP__ip_print_heap();
  ;
#line 296
  ;
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 92 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__isRunning(void ){
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
# 409 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__nextTrickleTime(void )
#line 409
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sentDIOFlag = FALSE;
  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__doubleCounter < /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntDouble) {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__doubleCounter++;
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__tricklePeriod *= 2;
    }
  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__isRunning()) {
    /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__chooseAdvertiseTime();
    }
}

#line 527
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__fired(void )
#line 527
{
  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sentDIOFlag) {


      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__nextTrickleTime();
    }
  else 
#line 532
    {



      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__postTask();

      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__postTask();
    }
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 517 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__fired(void )
#line 517
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__postTask();
}

static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IncreaseVersionTimer__fired(void )
#line 521
{

  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGVersionNumber++;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
}

# 280 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__fired(void )
#line 280
{
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__postTask();
}

# 52 "/opt/tinyos/tos/interfaces/Random.nc"
inline static uint16_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 192 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__fired(void )
#line 192
{
  uint32_t dao_next = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__dao_rate + 
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__Random__rand16() % (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__dao_rate / 10);

#line 195
  ;
#line 195
  ;









  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__postTask();
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__startOneShot(dao_next);
}

# 16 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
inline static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__delRoute(route_key_t key){
#line 16
  unsigned char __nesc_result;
#line 16

#line 16
  __nesc_result = IPForwardingEngineP__ForwardingTable__delRoute(key);
#line 16

#line 16
  return __nesc_result;
#line 16
}
#line 16
# 182 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__getStoreState(void )
#line 182
{

  return TRUE;
}

#line 284
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__fired(void )
#line 284
{

  uint8_t i;
#line 286
  uint8_t j;

#line 287
  if (!/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__getStoreState()) {
    return;
    }
  for (i = 0; i < /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count; i++) {
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[i].lifetime -= /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__remove_time;
      if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[i].lifetime <= /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__remove_time) {

          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__delRoute(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[i].key);
          for (j = i; j < /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count - 1; j++) {
              /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[j] = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[j + 1];
            }
          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count - 1].lifetime = 0;
          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count--;
        }
    }
}

# 204 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 83 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x412069f0){
#line 83
  switch (arg_0x412069f0) {
#line 83
    case 1U:
#line 83
      PacketLinkP__DelayTimer__fired();
#line 83
      break;
#line 83
    case 2U:
#line 83
      IPDispatchP__ExpireTimer__fired();
#line 83
      break;
#line 83
    case 3U:
#line 83
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__fired();
#line 83
      break;
#line 83
    case 4U:
#line 83
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__fired();
#line 83
      break;
#line 83
    case 5U:
#line 83
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IncreaseVersionTimer__fired();
#line 83
      break;
#line 83
    case 6U:
#line 83
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__fired();
#line 83
      break;
#line 83
    case 7U:
#line 83
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__fired();
#line 83
      break;
#line 83
    case 8U:
#line 83
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x412069f0);
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
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 93
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type __nesc_temp = 
#line 93
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;

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
  __nesc_result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__getAlarm();
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

# 303 "/usr/lib/ncc/nesc_nx.h"
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 303
{
#line 303
  __nesc_hton_uint8(target, value);
#line 303
  return value;
}

# 48 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t *IPDispatchP__getLowpanPayload(struct packed_lowmsg *lowmsg)
#line 48
{
  uint8_t len = 0;






  if (lowmsg->headers & LOWMSG_FRAG1_HDR) {
    len += LOWMSG_FRAG1_LEN;
    }
#line 58
  if (lowmsg->headers & LOWMSG_FRAGN_HDR) {
    len += LOWMSG_FRAGN_LEN;
    }
#line 60
  return lowmsg->data + len;
}

# 111 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline int8_t CC2420PacketP__CC2420Packet__getRssi(message_t *p_msg)
#line 111
{
  return __nesc_ntoh_uint8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->rssi.nxdata);
}

# 64 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Packet.nc"
inline static int8_t CC2420ReadLqiC__CC2420Packet__getRssi(message_t *p_msg){
#line 64
  signed char __nesc_result;
#line 64

#line 64
  __nesc_result = CC2420PacketP__CC2420Packet__getRssi(p_msg);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 16 "/opt/tinyos/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static inline uint8_t CC2420ReadLqiC__ReadLqi__readRssi(message_t *msg)
#line 16
{
  return CC2420ReadLqiC__CC2420Packet__getRssi(msg);
}

# 8 "/opt/tinyos/tos/lib/net/blip/interfaces/ReadLqi.nc"
inline static uint8_t IPDispatchP__ReadLqi__readRssi(message_t *msg){
#line 8
  unsigned char __nesc_result;
#line 8

#line 8
  __nesc_result = CC2420ReadLqiC__ReadLqi__readRssi(msg);
#line 8

#line 8
  return __nesc_result;
#line 8
}
#line 8
# 115 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__CC2420Packet__getLqi(message_t *p_msg)
#line 115
{
  return __nesc_ntoh_uint8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->lqi.nxdata);
}

# 72 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420ReadLqiC__CC2420Packet__getLqi(message_t *p_msg){
#line 72
  unsigned char __nesc_result;
#line 72

#line 72
  __nesc_result = CC2420PacketP__CC2420Packet__getLqi(p_msg);
#line 72

#line 72
  return __nesc_result;
#line 72
}
#line 72
# 12 "/opt/tinyos/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static inline uint8_t CC2420ReadLqiC__ReadLqi__readLqi(message_t *msg)
#line 12
{
  return CC2420ReadLqiC__CC2420Packet__getLqi(msg);
}

# 6 "/opt/tinyos/tos/lib/net/blip/interfaces/ReadLqi.nc"
inline static uint8_t IPDispatchP__ReadLqi__readLqi(message_t *msg){
#line 6
  unsigned char __nesc_result;
#line 6

#line 6
  __nesc_result = CC2420ReadLqiC__ReadLqi__readLqi(msg);
#line 6

#line 6
  return __nesc_result;
#line 6
}
#line 6
# 166 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t IPDispatchP__hasFragNHeader(struct packed_lowmsg *msg)
#line 166
{
  return msg->headers & LOWMSG_FRAGN_HDR;
}

#line 68
static __inline uint16_t IPDispatchP__getHeaderBitmap(struct packed_lowmsg *lowmsg)
#line 68
{
  uint16_t headers = 0;
  uint8_t *buf = lowmsg->data;
  int16_t len = lowmsg->len;

#line 72
  if (buf == (void *)0) {
#line 72
    return headers;
    }
  if (len > 0 && (*buf >> 6) == LOWPAN_NALP_PATTERN) {
      return LOWMSG_NALP;
    }
#line 98
  if (len > 0 && (*buf >> 3) == LOWPAN_FRAG1_PATTERN) {
      headers |= LOWMSG_FRAG1_HDR;
      buf += LOWMSG_FRAG1_LEN;
      len -= LOWMSG_FRAG1_LEN;
    }
  if (len > 0 && (*buf >> 3) == LOWPAN_FRAGN_PATTERN) {
      headers |= LOWMSG_FRAGN_HDR;
      buf += LOWMSG_FRAGN_LEN;
      len -= LOWMSG_FRAGN_LEN;
    }
  return headers;
}

# 76 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_frag.c"
static inline int IPDispatchP__lowpan_recon_add(struct lowpan_reconstruct *recon, 
uint8_t *pkt, size_t len)
#line 77
{
  struct packed_lowmsg msg;
  uint8_t *buf;

  msg.data = pkt;
  msg.len = len;
  msg.headers = IPDispatchP__getHeaderBitmap(&msg);
  if (msg.headers == LOWMSG_NALP) {
#line 84
    return -1;
    }
  if (!IPDispatchP__hasFragNHeader(&msg)) {
      return -2;
    }

  buf = IPDispatchP__getLowpanPayload(&msg);
  len -= buf - pkt;

  if (recon->r_size < recon->r_bytes_rcvd + len) {
#line 93
    return -3;
    }

  memcpy(recon->r_buf + recon->r_bytes_rcvd, buf, len);
  recon->r_bytes_rcvd += len;

  return 0;
}

# 163 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t IPDispatchP__hasFrag1Header(struct packed_lowmsg *msg)
#line 163
{
  return msg->headers & LOWMSG_FRAG1_HDR;
}

# 303 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline struct lowpan_reconstruct *IPDispatchP__get_reconstruct(uint16_t key, uint16_t tag)
#line 303
{
  struct lowpan_reconstruct *ret = (void *)0;
  int i;



  for (i = 0; i < N_RECONSTRUCTIONS; i++) {
      struct lowpan_reconstruct *recon = (struct lowpan_reconstruct *)&IPDispatchP__recon_data[i];

      if (recon->r_tag == tag && 
      recon->r_source_key == key) {

          if (recon->r_timeout > T_UNUSED) {
              recon->r_timeout = T_ACTIVE;
              ret = recon;
              goto done;
            }
          else {
#line 320
            if (recon->r_timeout < T_UNUSED) {


                ret = (void *)0;
                goto done;
              }
            }
        }
#line 327
      if (recon->r_timeout == T_UNUSED) {
        ret = recon;
        }
    }
#line 330
  done: 

    return ret;
}

# 272 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t IPDispatchP__getFragDgramTag(struct packed_lowmsg *msg, uint16_t *tag)
#line 272
{
  uint8_t *buf = msg->data;

#line 274
  if (buf == (void *)0 || tag == (void *)0) {
#line 274
    return 1;
    }



  if ((*buf >> 3) != LOWPAN_FRAG1_PATTERN && (
  *buf >> 3) != LOWPAN_FRAGN_PATTERN) {
#line 280
    return 1;
    }
#line 281
  buf += 2;

  *tag = (((uint16_t )* (uint16_t *)buf >> 8) | ((uint16_t )* (uint16_t *)buf << 8)) & 0xffff;
  return 0;
}

# 44 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/ieee154_header.c"
static inline uint8_t *IPDispatchP__unpack_ieee154_hdr(uint8_t *buf, struct ieee154_frame_addr *frame)
#line 44
{
  uint16_t fcf = ((uint16_t )buf[2] << 8) | buf[1];

  frame->ieee_dstpan = ((uint16_t )buf[5] << 8) | buf[4];
  frame->ieee_src.ieee_mode = (fcf >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3;
  frame->ieee_dst.ieee_mode = (fcf >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3;

  buf += IEEE154_MIN_HDR_SZ;

  if (frame->ieee_dst.ieee_mode == IEEE154_ADDR_SHORT) {
      memcpy(& frame->ieee_dst.ieee_addr.saddr, buf, 2);
      buf += 2;
    }
  else {
#line 56
    if (frame->ieee_dst.ieee_mode == IEEE154_ADDR_EXT) {
        memcpy(& frame->ieee_dst.ieee_addr.laddr, buf, 8);
        buf += 8;
      }
    }
  if (frame->ieee_src.ieee_mode == IEEE154_ADDR_SHORT) {
      memcpy(& frame->ieee_src.ieee_addr.saddr, buf, 2);
      buf += 2;
    }
  else {
#line 64
    if (frame->ieee_src.ieee_mode == IEEE154_ADDR_EXT) {
        memcpy(& frame->ieee_src.ieee_addr.laddr, buf, 8);
        buf += 8;
      }
    }
#line 68
  return buf;
}

# 335 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline message_t *IPDispatchP__Ieee154Receive__receive(message_t *msg, void *msg_payload, uint8_t len)
#line 335
{
  struct packed_lowmsg lowmsg;
  struct ieee154_frame_addr frame_address;
  uint8_t *buf = msg_payload;



  ;


  buf = IPDispatchP__unpack_ieee154_hdr(msg_payload, &frame_address);
  len -= buf - (uint8_t *)msg_payload;


  lowmsg.data = buf;
  lowmsg.len = len;
  lowmsg.headers = IPDispatchP__getHeaderBitmap(&lowmsg);
  if (lowmsg.headers == LOWMSG_NALP) {
      goto fail;
    }

  if (IPDispatchP__hasFrag1Header(&lowmsg) || IPDispatchP__hasFragNHeader(&lowmsg)) {

      int rv;
      struct lowpan_reconstruct *recon;
      uint16_t tag;
#line 360
      uint16_t source_key;

      source_key = ieee154_hashaddr(& frame_address.ieee_src);
      IPDispatchP__getFragDgramTag(&lowmsg, &tag);
      recon = IPDispatchP__get_reconstruct(source_key, tag);
      if (!recon) {
          goto fail;
        }



      memcpy(& recon->r_meta.sender, & frame_address.ieee_src, 
      sizeof(ieee154_addr_t ));
      recon->r_meta.lqi = IPDispatchP__ReadLqi__readLqi(msg);
      recon->r_meta.rssi = IPDispatchP__ReadLqi__readRssi(msg);

      if (IPDispatchP__hasFrag1Header(&lowmsg)) {
          if (recon->r_buf != (void *)0) {
#line 377
            goto fail;
            }
#line 378
          rv = IPDispatchP__lowpan_recon_start(&frame_address, recon, buf, len);
        }
      else 
#line 379
        {
          rv = IPDispatchP__lowpan_recon_add(recon, buf, len);
        }

      if (rv < 0) {
          recon->r_timeout = T_FAILED1;
          goto fail;
        }
      else 
#line 386
        {

          recon->r_timeout = T_ACTIVE;
          recon->r_source_key = source_key;
          recon->r_tag = tag;
        }

      if (recon->r_size == recon->r_bytes_rcvd) {
          IPDispatchP__deliver(recon);
        }
    }
  else {

      int rv;
      struct lowpan_reconstruct recon;


      memcpy(& recon.r_meta.sender, & frame_address.ieee_src, 
      sizeof(ieee154_addr_t ));
      recon.r_meta.lqi = IPDispatchP__ReadLqi__readLqi(msg);
      recon.r_meta.rssi = IPDispatchP__ReadLqi__readRssi(msg);

      buf = IPDispatchP__getLowpanPayload(&lowmsg);
      if ((rv = IPDispatchP__lowpan_recon_start(&frame_address, &recon, buf, len)) < 0) {
          goto fail;
        }

      if (recon.r_size == recon.r_bytes_rcvd) {
          IPDispatchP__deliver(&recon);
        }
      else 
#line 415
        {

          ip_free(recon.r_buf);
        }
    }
  goto done;
  fail: 
    ;
  done: 
    return msg;
}

# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__BareReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = IPDispatchP__Ieee154Receive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 247 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__ActiveReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 247
{
  return msg;
}

# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__ActiveReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__ActiveReceive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 81 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline int CC2420PacketP__getAddressLength(int type)
#line 81
{
  switch (type) {
      case IEEE154_ADDR_SHORT: return 2;
      case IEEE154_ADDR_EXT: return 8;
      case IEEE154_ADDR_NONE: return 0;
      default: return -100;
    }
}

static inline uint8_t * CC2420PacketP__getNetwork(message_t * msg)
#line 90
{
  cc2420_header_t *hdr = CC2420PacketP__CC2420PacketBody__getHeader(msg);
  int offset;

  offset = CC2420PacketP__getAddressLength((__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3) + 
  CC2420PacketP__getAddressLength((__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3) + 
  (unsigned short )& ((cc2420_header_t *)0)->dest;

  return (uint8_t *)hdr + offset;
}

#line 119
static inline uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t * p_msg)
#line 119
{



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      unsigned char __nesc_temp = 
#line 124
      *CC2420PacketP__getNetwork(p_msg);

      {
#line 124
        __nesc_atomic_end(__nesc_atomic); 
#line 124
        return __nesc_temp;
      }
    }
#line 126
    __nesc_atomic_end(__nesc_atomic); }
}

# 75 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t * p_msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420PacketP__CC2420Packet__getNetwork(p_msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 157 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 157
{
  uint8_t network = CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg);

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(msg)->crc.nxdata)) {
      return msg;
    }

  if (network == 0x3f) {
      return CC2420TinyosNetworkP__ActiveReceive__receive(msg, payload, len);
    }
  else 
#line 166
    {
      return CC2420TinyosNetworkP__BareReceive__receive(msg, 
      CC2420TinyosNetworkP__BareSend__getPayload(msg, len), 
      len + sizeof(cc2420_header_t ));
    }
}

# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 138 "/opt/tinyos/tos/chips/cc2520/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn)
#line 138
{
  uint8_t element = UniqueReceiveP__recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 142
    {
      if (element == UniqueReceiveP__INVALID_ELEMENT || UniqueReceiveP__writeIndex == element) {

          element = UniqueReceiveP__writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP__receivedMessages[element].source = msgSource;
      UniqueReceiveP__receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP__writeIndex++;
          UniqueReceiveP__writeIndex %= 4;
        }
    }
#line 155
    __nesc_atomic_end(__nesc_atomic); }
}

#line 192
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 192
{
  return msg;
}

# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__DuplicateReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP__DuplicateReceive__default__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 112 "/opt/tinyos/tos/chips/cc2520/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 112
{
  int i;

#line 114
  UniqueReceiveP__recycleSourceElement = UniqueReceiveP__INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP__receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP__receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 121
                    TRUE;

                    {
#line 121
                      __nesc_atomic_end(__nesc_atomic); 
#line 121
                      return __nesc_temp;
                    }
                  }
                }
#line 124
              UniqueReceiveP__recycleSourceElement = i;
            }
        }
    }
#line 127
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 165 "/opt/tinyos/tos/chips/cc2520/unique/UniqueReceiveP.nc"
static inline uint16_t UniqueReceiveP__getSourceKey(message_t * msg)
#line 165
{
  cc2420_header_t *hdr = UniqueReceiveP__CC2420PacketBody__getHeader(msg);
  int s_mode = (__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3;
  int d_mode = (__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3;
  int s_offset = 2;
#line 169
  int s_len = 2;
  uint16_t key = 0;
  uint8_t *current = (uint8_t *)& hdr->dest;
  int i;

  if (s_mode == IEEE154_ADDR_EXT) {
      s_len = 8;
    }
  if (d_mode == IEEE154_ADDR_EXT) {
      s_offset = 8;
    }

  current += s_offset;

  for (i = 0; i < s_len; i++) {
      key += current[i];
    }
  return key;
}

#line 86
static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len)
#line 87
{

  uint16_t msgSource = UniqueReceiveP__getSourceKey(msg);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->dsn.nxdata);

  if (UniqueReceiveP__hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP__DuplicateReceive__receive(msg, payload, len);
    }
  else 
#line 94
    {
      UniqueReceiveP__insert(msgSource, msgDsn);
      return UniqueReceiveP__Receive__receive(msg, payload, len);
    }
}

# 78 "/opt/tinyos/tos/interfaces/Receive.nc"
inline static message_t * CC2420ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP__SubReceive__receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 298 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline ieee_eui64_t CC2420ControlP__CC2420Config__getExtAddr(void )
#line 298
{
  return CC2420ControlP__m_ext_addr;
}

# 66 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
inline static ieee_eui64_t CC2420ReceiveP__CC2420Config__getExtAddr(void ){
#line 66
  struct ieee_eui64 __nesc_result;
#line 66

#line 66
  __nesc_result = CC2420ControlP__CC2420Config__getExtAddr();
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66





inline static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void ){
#line 71
  unsigned int __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Config__getShortAddr();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 355 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void )
#line 355
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 356
    {
      unsigned char __nesc_temp = 
#line 356
      CC2420ControlP__addressRecognition;

      {
#line 356
        __nesc_atomic_end(__nesc_atomic); 
#line 356
        return __nesc_temp;
      }
    }
#line 358
    __nesc_atomic_end(__nesc_atomic); }
}

# 93 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void ){
#line 93
  unsigned char __nesc_result;
#line 93

#line 93
  __nesc_result = CC2420ControlP__CC2420Config__isAddressRecognitionEnabled();
#line 93

#line 93
  return __nesc_result;
#line 93
}
#line 93
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 824 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg)
#line 824
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(msg);
  int mode = (__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 3;
  ieee_eui64_t *ext_addr;

  if (!CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled()) {
      return TRUE;
    }

  if (mode == IEEE154_ADDR_SHORT) {
      return __nesc_ntoh_leuint16(header->dest.nxdata) == CC2420ReceiveP__CC2420Config__getShortAddr()
       || __nesc_ntoh_leuint16(header->dest.nxdata) == IEEE154_BROADCAST_ADDR;
    }
  else {
#line 836
    if (mode == IEEE154_ADDR_EXT) {
        ieee_eui64_t local_addr = CC2420ReceiveP__CC2420Config__getExtAddr();

#line 838
        ext_addr = (ieee_eui64_t * )& header->dest;
        return memcmp(ext_addr->data, local_addr.data, IEEE_EUI64_LENGTH) == 0;
      }
    else 
#line 840
      {

        return FALSE;
      }
    }
}

# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 676 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__receiveDone_task__runTask(void )
#line 676
{
  cc2420_metadata_t *metadata = CC2420ReceiveP__CC2420PacketBody__getMetadata(CC2420ReceiveP__m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.nxdata);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.nxdata, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.nxdata, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.nxdata, buf[length - 1]);

  if (CC2420ReceiveP__passesAddressCheck(CC2420ReceiveP__m_p_rx_buf) && length >= CC2420_SIZE) {
#line 701
      CC2420ReceiveP__m_p_rx_buf = CC2420ReceiveP__Receive__receive(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 704
    CC2420ReceiveP__receivingPacket = FALSE;
#line 704
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

# 255 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_4944.c"
static __inline uint8_t IPDispatchP__getFragDgramSize(struct packed_lowmsg *msg, uint16_t *size)
#line 255
{
  uint8_t *buf = msg->data;
  uint8_t s[2];

#line 258
  if (buf == (void *)0 || size == (void *)0) {
#line 258
    return 1;
    }



  if ((*buf >> 3) != LOWPAN_FRAG1_PATTERN && (
  *buf >> 3) != LOWPAN_FRAGN_PATTERN) {
#line 264
    return 1;
    }
  s[0] = *buf & 0x7;
  buf++;
  s[1] = *buf;
  *size = ((uint16_t )s[0] << 8) | s[1];
  return 0;
}

# 624 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static inline uint8_t *IPDispatchP__unpack_udp(uint8_t *dest, uint8_t *nxt_hdr, uint8_t *buf)
#line 624
{
  struct udp_hdr *udp = (struct udp_hdr *)dest;
  uint8_t dispatch = * buf++;

  *nxt_hdr = IANA_UDP;


  udp->len = 0;

  udp->chksum = 0;


  switch (dispatch & LOWPAN_NHC_UDP_PORT_MASK) {
      case LOWPAN_NHC_UDP_PORT_FULL: 
        udp->srcport = (((uint16_t )((buf[0] << 8) | buf[1]) << 8) | ((uint16_t )((buf[0] << 8) | buf[1]) >> 8)) & 0xffff;
      udp->dstport = (((uint16_t )((buf[2] << 8) | buf[3]) << 8) | ((uint16_t )((buf[2] << 8) | buf[3]) >> 8)) & 0xffff;
      buf += 4;
      break;
      case LOWPAN_NHC_UDP_PORT_SRC_FULL: 
        udp->srcport = (((uint16_t )((buf[0] << 8) | buf[1]) << 8) | ((uint16_t )((buf[0] << 8) | buf[1]) >> 8)) & 0xffff;
      udp->dstport = (((uint16_t )((0xF0 << 8) | buf[2]) << 8) | ((uint16_t )((0xF0 << 8) | buf[2]) >> 8)) & 0xffff;
      buf += 3;
      break;
      case LOWPAN_NHC_UDP_PORT_DST_FULL: 
        udp->srcport = (((uint16_t )((0xF0 << 8) | buf[0]) << 8) | ((uint16_t )((0xF0 << 8) | buf[0]) >> 8)) & 0xffff;
      udp->dstport = (((uint16_t )((buf[1] << 8) | buf[2]) << 8) | ((uint16_t )((buf[1] << 8) | buf[2]) >> 8)) & 0xffff;
      buf += 3;
      break;
      case LOWPAN_NHC_UDP_PORT_SHORT: 
        udp->srcport = (((uint16_t )(0xF0B0 | (buf[0] >> 4)) << 8) | ((uint16_t )(0xF0B0 | (buf[0] >> 4)) >> 8)) & 0xffff;
      udp->dstport = 0xF0B0 | (buf[0] & 0xf);
      udp->dstport = (((uint16_t )udp->dstport << 8) | ((uint16_t )udp->dstport >> 8)) & 0xffff;
      buf += 1;
      break;
    }

  if (!(dispatch & LOWPAN_NHC_UDP_CKSUM)) {
      udp->chksum = (((uint16_t )((buf[0] << 8) | buf[1]) << 8) | ((uint16_t )((buf[0] << 8) | buf[1]) >> 8)) & 0xffff;
      buf += 2;
    }

  return buf;
}






static inline uint8_t *IPDispatchP__unpack_ipnh(uint8_t *dest, size_t cnt, uint8_t *nxt_hdr, uint8_t *buf)
#line 673
{
  if ((*buf & LOWPAN_NHC_IPV6_MASK) == LOWPAN_NHC_IPV6_PATTERN) {
      struct ip6_ext *ext = (struct ip6_ext *)dest;
      uint8_t length;
#line 676
      uint8_t extra;



      switch (*buf & LOWPAN_NHC_EID_MASK) {
          case LOWPAN_NHC_EID_HOP: 
            *nxt_hdr = IPV6_HOP;
#line 682
          break;
          case LOWPAN_NHC_EID_ROUTING: 
            *nxt_hdr = IPV6_ROUTING;
#line 684
          break;
          case LOWPAN_NHC_EID_FRAG: 
            *nxt_hdr = IPV6_FRAG;
#line 686
          break;
          case LOWPAN_NHC_EID_DEST: 
            *nxt_hdr = IPV6_DEST;
#line 688
          break;
          case LOWPAN_NHC_EID_MOBILE: 
            *nxt_hdr = IPV6_MOBILITY;
#line 690
          break;
          case LOWPAN_NHC_EID_IPV6: 

            *nxt_hdr = IPV6_IPV6;
#line 693
          break;
          default: 
            return (void *)0;
        }


      if (!(*buf & LOWPAN_NHC_NH)) {
          buf++;
          ext->ip6e_nxt = *buf;
        }
      buf += 1;
      length = * buf++;
      extra = (8 - length % 8) % 8;

      if (cnt < length + extra - 2) {
        return (void *)0;
        }

      memcpy(dest + 2, buf, length - 2);


      if (*nxt_hdr == IPV6_HOP || *nxt_hdr == IPV6_DEST) {
          if (extra == 1) {

              dest[length] = IPV6_TLV_PAD1;
            }
          else {
#line 718
            if (extra > 1) {
                dest[length] = IPV6_TLV_PADN;
                dest[length + 1] = extra - 2;
              }
            }
        }
#line 723
      ext->ip6e_len = (length + extra) / 8 - 1;

      return buf + length + extra - 2;
    }
  else {
#line 726
    if ((*buf & LOWPAN_NHC_UDP_MASK) == LOWPAN_NHC_UDP_PATTERN) {

        return IPDispatchP__unpack_udp(dest, nxt_hdr, buf);
      }
    }
#line 730
  return (void *)0;
}

static inline uint8_t *IPDispatchP__unpack_nhc_chain(struct lowpan_reconstruct *recon, 
uint8_t **dest, size_t cnt, 
uint8_t *nxt_hdr, uint8_t *buf)
#line 735
{
  uint8_t *dispatch;
  int has_nhc = 1;

  do {
      recon->r_transport_header = *dest;
      dispatch = buf;
      buf = IPDispatchP__unpack_ipnh(*dest, cnt, nxt_hdr, buf);

      if (!buf) {
#line 744
        return (void *)0;
        }
      if ((*dispatch & LOWPAN_NHC_IPV6_MASK) == LOWPAN_NHC_IPV6_PATTERN) {
          struct ip6_ext *ext = (struct ip6_ext *)*dest;

          *dest += (ext->ip6e_len + 1) * 8;
          cnt -= (ext->ip6e_len + 1) * 8;

          if (*dispatch & LOWPAN_NHC_NH) {
              nxt_hdr = & ext->ip6e_nxt;
            }
          else 
#line 754
            {
              has_nhc = 0;
            }
        }
      else {
#line 757
        if ((*dispatch & LOWPAN_NHC_UDP_MASK) == LOWPAN_NHC_UDP_PATTERN) {
            struct udp_hdr *udp = (struct udp_hdr *)*dest;

#line 759
            recon->r_app_len = & udp->len;
            has_nhc = 0;
            *dest += sizeof(struct udp_hdr );
          }
        else 
#line 762
          {
#line 762
            has_nhc = 0;
          }
        }
    }
  while (
#line 763
  has_nhc);
  return buf;
}

#line 592
static inline uint8_t *IPDispatchP__unpack_multicast(struct in6_addr *addr, uint8_t dispatch, 
int context, uint8_t *buf)
#line 593
{
  memset(addr->in6_u.u6_addr8, 0, 16);

  if (!(dispatch & LOWPAN_IPHC_AC_CONTEXT)) {
      int amount;

#line 598
      switch (dispatch & LOWPAN_IPHC_AM_MASK) {
          case LOWPAN_IPHC_AM_M_128: 
            memcpy(addr->in6_u.u6_addr8, buf, 16);
          return buf + 16;
          case LOWPAN_IPHC_AM_M_48: 
            amount = 5;
          goto copy;
          case LOWPAN_IPHC_AM_M_32: 
            amount = 3;
          copy: 
            addr->in6_u.u6_addr8[0] = 0xff;
          addr->in6_u.u6_addr8[1] = buf[0];
          memcpy(&addr->in6_u.u6_addr8[16 - amount], buf + 1, amount);
          return buf + 1 + amount;
          case LOWPAN_IPHC_AM_M_8: 
            addr->in6_u.u6_addr16[0] = (((uint16_t )0xff02 << 8) | ((uint16_t )0xff02 >> 8)) & 0xffff;
          addr->in6_u.u6_addr8[15] = buf[0];
          return buf + 1;
        }
    }
  else 
#line 617
    {
    }


  return (void *)0;
}

#line 508
static inline uint8_t *IPDispatchP__unpack_hlim(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf)
#line 508
{
  switch (dispatch & LOWPAN_IPHC_HLIM_MASK) {
      case LOWPAN_IPHC_HLIM_1: 
        hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim = 1;
      break;
      case LOWPAN_IPHC_HLIM_64: 
        hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim = 64;
      break;
      case LOWPAN_IPHC_HLIM_255: 
        hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim = 255;
      break;
      default: 
        hdr->ip6_ctlun.ip6_un1.ip6_un1_hlim = *buf;
      return buf + 1;
    }
  return buf;
}

#line 499
static inline uint8_t *IPDispatchP__unpack_nh(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf)
#line 499
{
  if ((dispatch & LOWPAN_IPHC_NH_MASK) == LOWPAN_IPHC_NH_INLINE) {
      hdr->ip6_ctlun.ip6_un1.ip6_un1_nxt = *buf;
      return buf + 1;
    }
  else 
#line 503
    {
      return buf;
    }
}

#line 465
static inline uint8_t *IPDispatchP__unpack_tcfl(struct ip6_hdr *hdr, uint8_t dispatch, uint8_t *buf)
#line 465
{
  uint8_t fl[3] = { 0, 0, 0 };
  uint8_t tc = 0;

  switch (dispatch & LOWPAN_IPHC_TF_MASK) {
      case LOWPAN_IPHC_TF_ECN_DSCP: 
        tc = (*buf >> 6) & 0xff;
      tc |= (*buf << 2) & 0xff;
      buf += 1;
      break;
      case LOWPAN_IPHC_TF_ECN_FL: 
        tc = (*buf >> 6) & 0xff;
      fl[2] = buf[0] & 0x0f;
      fl[1] = buf[1];
      fl[0] = buf[2];
      buf += 3;
      break;
      case LOWPAN_IPHC_TF_ECN_DSCP_FL: 
        tc = (*buf >> 6) & 0xff;
      tc |= (*buf << 2) & 0xff;
      fl[2] = buf[1] & 0x0f;
      fl[1] = buf[2];
      fl[0] = buf[3];
      buf += 4;
      break;
    }

  hdr->ip6_ctlun.ip6_un1.ip6_un1_flow = ntohl((((((uint32_t )0x6 << 28) | ((uint32_t )tc << 20)) | ((uint32_t )fl[2] << 16)) | ((uint32_t )fl[1] << 8)) | fl[0]);



  return buf;
}

#line 767
static inline uint8_t *IPDispatchP__lowpan_unpack_headers(struct lowpan_reconstruct *recon, 
struct ieee154_frame_addr *frame, 
uint8_t *buf, size_t cnt)
#line 769
{
  uint8_t *dispatch;
#line 770
  uint8_t *unpack_start = buf;
#line 770
  uint8_t *unpack_end;
  int contexts[2] = { 0, 0 };
  uint8_t *dest = recon->r_buf;
  size_t dst_cnt = recon->r_size;
  struct ip6_hdr *hdr = (struct ip6_hdr *)dest;

  dispatch = buf;
  buf += 2;

  if ((*dispatch & LOWPAN_DISPATCH_BYTE_MASK) != LOWPAN_DISPATCH_BYTE_VAL) {
      return (void *)0;
    }


  if ((*(dispatch + 1) & LOWPAN_IPHC_CID_MASK) == LOWPAN_IPHC_CID_PRESENT) {
      contexts[0] = (*buf >> 4) & 0xf;
      contexts[1] = *buf & 0xf;
      buf += 1;
    }


  buf = IPDispatchP__unpack_tcfl(hdr, *dispatch, buf);
  buf = IPDispatchP__unpack_nh(hdr, *dispatch, buf);
  buf = IPDispatchP__unpack_hlim(hdr, *dispatch, buf);



  buf = IPDispatchP__unpack_address(& hdr->ip6_src, 
  *(dispatch + 1) >> LOWPAN_IPHC_SAM_SHIFT, 
  contexts[0], 
  buf, 
  & frame->ieee_src, 
  frame->ieee_dstpan);
  if (!buf) {
      return (void *)0;
    }


  if (*(dispatch + 1) & LOWPAN_IPHC_M) {

      buf = IPDispatchP__unpack_multicast(& hdr->ip6_dst, 
      *(dispatch + 1) >> LOWPAN_IPHC_DAM_SHIFT, 
      contexts[1], 
      buf);
    }
  else {
      buf = IPDispatchP__unpack_address(& hdr->ip6_dst, 
      *(dispatch + 1) >> LOWPAN_IPHC_DAM_SHIFT, 
      contexts[1], 
      buf, 
      & frame->ieee_dst, 
      frame->ieee_dstpan);
    }
  if (!buf) {
      return (void *)0;
    }




  unpack_end = (uint8_t *)(hdr + 1);
  if (*dispatch & LOWPAN_IPHC_NH_MASK) {
      buf = IPDispatchP__unpack_nhc_chain(recon, 
      &unpack_end, 
      dst_cnt - sizeof(struct ip6_hdr ), 
      & hdr->ip6_ctlun.ip6_un1.ip6_un1_nxt, 
      buf);
      if (!buf) {
          return (void *)0;
        }
    }



  memcpy(unpack_end, buf, cnt - (buf - unpack_start));


  return unpack_end + (cnt - (buf - unpack_start));
}

# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool IPNeighborDiscoveryP__IPAddress__getGlobalAddr(struct in6_addr *addr){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = IPAddressP__IPAddress__getGlobalAddr(addr);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 44 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static inline int IPNeighborDiscoveryP__NeighborDiscovery__getContext(uint8_t context, 
struct in6_addr *ctx)
#line 45
{
  struct in6_addr me;

#line 47
  if (!IPNeighborDiscoveryP__IPAddress__getGlobalAddr(&me)) {
#line 47
    return 0;
    }
#line 48
  if (context == 0) {


      memcpy(ctx->in6_u.u6_addr8, me.in6_u.u6_addr8, 8);
      return 64;
    }
  else 
#line 53
    {
      return 0;
    }
}

# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/NeighborDiscovery.nc"
inline static int IPDispatchP__NeighborDiscovery__getContext(uint8_t context, struct in6_addr *ctx){
#line 17
  int __nesc_result;
#line 17

#line 17
  __nesc_result = IPNeighborDiscoveryP__NeighborDiscovery__getContext(context, ctx);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 84 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline int IPDispatchP__lowpan_extern_read_context(struct in6_addr *addr, int context)
#line 84
{
  return IPDispatchP__NeighborDiscovery__getContext(context, addr);
}

# 442 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline bool RPLRankP__ForwardingEvents__approve(struct ip6_packet *pkt, 
struct in6_addr *next_hop)
#line 443
{

  return TRUE;
}

# 344 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline bool IPForwardingEngineP__ForwardingEvents__default__approve(uint8_t idx, struct ip6_packet *pkt, 
struct in6_addr *next_hop)
#line 345
{
  return TRUE;
}

# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
inline static bool IPForwardingEngineP__ForwardingEvents__approve(uint8_t arg_0x40925e40, struct ip6_packet *pkt, struct in6_addr *next_hop){
#line 28
  unsigned char __nesc_result;
#line 28

#line 28
  switch (arg_0x40925e40) {
#line 28
    case RPL_IFACE:
#line 28
      __nesc_result = RPLRankP__ForwardingEvents__approve(pkt, next_hop);
#line 28
      break;
#line 28
    default:
#line 28
      __nesc_result = IPForwardingEngineP__ForwardingEvents__default__approve(arg_0x40925e40, pkt, next_hop);
#line 28
      break;
#line 28
    }
#line 28

#line 28
  return __nesc_result;
#line 28
}
#line 28
# 14 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
inline static int IPForwardingEngineP__IPPacket__findHeader(struct ip_iovec *payload, uint8_t first_type, uint8_t *search_type){
#line 14
  int __nesc_result;
#line 14

#line 14
  __nesc_result = IPPacketC__IPPacket__findHeader(payload, first_type, search_type);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 44 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__isLocalAddress(struct in6_addr *addr){
#line 44
  unsigned char __nesc_result;
#line 44

#line 44
  __nesc_result = IPAddressP__IPAddress__isLocalAddress(addr);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 554 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 555
{

  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running) {
#line 557
    return;
    }
  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_LEAF) {

      return;
    }

  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__isLocalAddress(& iph->ip6_dst)) {

      if (iph->ip6_dst.in6_u.u6_addr8[0] == 0xff && (
      iph->ip6_dst.in6_u.u6_addr8[1] & 0xf) <= 0x3) {
          /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
        }
      else 
#line 569
        {
          /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO = TRUE;
          memcpy(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__UNICAST_DIO_ADDR, & iph->ip6_src, sizeof(struct in6_addr ));
          /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__postTask();
        }
    }
}

# 50 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__default__recv(uint8_t code, struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta)
#line 51
{
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__recv(uint8_t arg_0x415698a8, struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  switch (arg_0x415698a8) {
#line 23
    case ICMPV6_CODE_DIS:
#line 23
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIS__recv(hdr, packet, len, meta);
#line 23
      break;
#line 23
    default:
#line 23
      /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__default__recv(arg_0x415698a8, hdr, packet, len, meta);
#line 23
      break;
#line 23
    }
#line 23
}
#line 23
# 39 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline void /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__recv(struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta)
#line 40
{
  struct icmp6_hdr *icmph = packet;

  /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__IP__recv(icmph->code, iph, packet, len, meta);
}

# 10 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
inline static route_key_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__addRoute(const uint8_t *prefix, int prefix_len_bits, struct in6_addr *next_hop, uint8_t ifindex){
#line 10
  int __nesc_result;
#line 10

#line 10
  __nesc_result = IPForwardingEngineP__ForwardingTable__addRoute(prefix, prefix_len_bits, next_hop, ifindex);
#line 10

#line 10
  return __nesc_result;
#line 10
}
#line 10
# 476 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDTSN(void )
#line 476
{
  return /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DTSN;
}

# 54 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static uint8_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDTSN(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__getDTSN();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 472 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDTSN(uint8_t dtsn)
#line 472
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DTSN = dtsn;
}

# 53 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__setDTSN(uint8_t dtsn){
#line 53
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDTSN(dtsn);
#line 53
}
#line 53
# 86 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__memcmp_rpl(uint8_t *a, uint8_t *b, uint8_t len)
#line 86
{
  uint8_t i;

#line 88
  for (i = 0; i < len; i++) 
    if (a[i] != b[i]) {
      return FALSE;
      }
#line 91
  return TRUE;
}

# 18 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTable.nc"
inline static struct route_entry */*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__lookupRoute(const uint8_t *prefix, int prefix_len_bits){
#line 18
  struct route_entry *__nesc_result;
#line 18

#line 18
  __nesc_result = IPForwardingEngineP__ForwardingTable__lookupRoute(prefix, prefix_len_bits);
#line 18

#line 18
  return __nesc_result;
#line 18
}
#line 18
# 304 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 305
{
  dao_entry_t *dao_msg;
  error_t error;

  struct dao_base_t *dao = (struct dao_base_t *)payload;
  struct route_entry *entry;
  route_key_t new_key = ROUTE_INVAL_KEY;

  ;
#line 313
  ;
  ;
#line 314
  ;
  if (!/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__m_running) {
#line 315
    return;
    }







  entry = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__lookupRoute(dao->target_option.target_prefix.in6_u.u6_addr8, 
  dao->target_option.prefix_length);

  if (entry != (void *)0 && 
  entry->prefixlen == dao->target_option.prefix_length) {


      if (
#line 330
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__memcmp_rpl((uint8_t *)entry->next_hop.in6_u.u6_addr8, 
      (uint8_t *)iph->ip6_src.in6_u.u6_addr8, 16) == TRUE) {
        }
      else {



          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__setDTSN(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getDTSN() + 1);
          if (dao->target_option.prefix_length > 0) {
            new_key = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__addRoute(dao->target_option.target_prefix.in6_u.u6_addr8, 
            dao->target_option.prefix_length, 
            & iph->ip6_src, 
            RPL_IFACE);
            }
        }
    }
  else 
#line 344
    {

      if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count == ROUTE_TABLE_SZ) {

          return;
        }
      ;
#line 350
      ;
      ;
#line 351
      ;
      if (dao->target_option.prefix_length > 0) {
          new_key = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__ForwardingTable__addRoute(dao->target_option.target_prefix.in6_u.u6_addr8, 
          dao->target_option.prefix_length, 
          & iph->ip6_src, 
          RPL_IFACE);
        }







      if (new_key != ROUTE_INVAL_KEY) {
          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count].lifetime = 
          dao->transit_info_option.path_lifetime;
          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table[/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count].key = new_key;

          /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__downwards_table_count++;
        }
    }





  if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getRank() == ROOT_RANK) {

      return;
    }







  dao_msg = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__get();

  if (dao_msg == (void *)0) {
      return;
    }



  ip_memcpy((uint8_t *)& dao_msg->s_pkt.ip6_hdr, 
  (uint8_t *)iph, sizeof(struct ip6_hdr ));


  ip_memcpy((uint8_t *)& dao_msg->dao_base, 
  (uint8_t *)payload, sizeof(struct dao_base_t ));
  dao_msg->v[0].iov_base = (uint8_t *)& dao_msg->dao_base;
  dao_msg->v[0].iov_len = (((uint16_t )iph->ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )iph->ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff;
  dao_msg->v[0].iov_next = (void *)0;
  dao_msg->s_pkt.ip6_data = &dao_msg->v[0];

  error = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendQueue__enqueue(dao_msg);

  if (error != SUCCESS) {
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__SendPool__put(dao_msg);
      return;
    }
  else 
#line 413
    {
      if (!/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__isRunning()) {
        /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__startOneShot(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__delay_dao);
        }
    }
}

# 40 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngine.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLDAORoutingEngine__startDAO(void ){
#line 40
  unsigned char __nesc_result;
#line 40

#line 40
  __nesc_result = /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__startDAO();
#line 40

#line 40
  return __nesc_result;
#line 40
}
#line 40
# 285 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline uint8_t RPLRankP__RPLRankInfo__hasParent(void )
#line 285
{
  return RPLRankP__parentNum;
}

# 98 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
inline static uint8_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__hasParent(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = RPLRankP__RPLRankInfo__hasParent();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 542 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__compare_ip6_addr(struct in6_addr *node1, struct in6_addr *node2)
#line 542
{
  return !memcmp(node1, node2, sizeof(struct in6_addr ));
}

# 289 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline bool RPLRankP__RPLRankInfo__isLeaf(void )
#line 289
{

  return RPLRankP__leafState;
}

# 99 "/opt/tinyos/tos/lib/net/rpl/RPLRank.nc"
inline static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__isLeaf(void ){
#line 99
  unsigned char __nesc_result;
#line 99

#line 99
  __nesc_result = RPLRankP__RPLRankInfo__isLeaf();
#line 99

#line 99
  return __nesc_result;
#line 99
}
#line 99
# 78 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(4U);
#line 78
}
#line 78
#line 92
inline static bool /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__isRunning(void ){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(4U);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
# 189 "/opt/tinyos/tos/lib/timer/VirtualizeTimerC.nc"
static inline uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
}

# 136 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static uint32_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__getNow(void ){
#line 136
  unsigned long __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__getNow(4U);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 577 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 578
{
  struct dio_base_t *dio = (struct dio_base_t *)payload;

#line 580
  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running) {
#line 580
    return;
    }
  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_ROOT) {
      return;
    }

  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIORedun != 0xFF) {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__redunCounter++;
    }
  else 
#line 588
    {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__redunCounter = 0xFF;
    }



  if (__nesc_ntoh_uint16(dio->dagRank.nxdata) == INFINITE_RANK) {
      if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP) != INFINITE_RANK && 
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__getNow() % 2 == 1) {

          /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__postTask();
        }
      return;
    }

  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__hasParent() && /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__isRunning()) {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__InitDISTimer__stop();
    }


  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_LEAF = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__isLeaf();


  if ((
#line 610
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_LEAF && !/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG)
   || !/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__compare_ip6_addr(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID, & dio->dodagID)) {









      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG = TRUE;


      goto accept_dodag;
    }



  if (
#line 627
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLInstanceID == dio->instance_id.id && 
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__compare_ip6_addr(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID, & dio->dodagID) && 
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGVersionNumber != __nesc_ntoh_uint8(dio->version.nxdata) && 
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG) {





      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGVersionNumber = __nesc_ntoh_uint8(dio->version.nxdata);
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
    }
  else {

    if (
#line 640
    /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP) != /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__node_rank && 
    /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG && 
    /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__node_rank != INFINITE_RANK) {




        if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP) > /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__LOWRANK + /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MaxRankInc && 
        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__node_rank != INFINITE_RANK) {
            /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG = FALSE;
            /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__node_rank = INFINITE_RANK;
          }
        else 
#line 651
          {
            if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__LOWRANK > /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP)) {
                /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__LOWRANK = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP);
              }
            /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__node_rank = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__getRank(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__ADDR_MY_IP);
          }

        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__inconsistencyDetected();
        return;
      }
    }
  if (/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__hasParent() && !/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG) {
      goto accept_dodag;
    }
  else {
#line 664
    if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRankInfo__hasParent() && !/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__I_AM_ROOT) {



        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG = FALSE;
        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__GROUND_STATE = dio->flags & 0x80;

        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__stop();

        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
        /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLDAORoutingEngine__startDAO();
      }
    }
#line 676
  return;
  accept_dodag: 



    /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__hasDODAG = TRUE;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MOP = (dio->flags & 0x3c) >> 3;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DAG_PREF = dio->flags & 0x07;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLInstanceID = dio->instance_id.id;
  memcpy(&/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGID, & dio->dodagID, sizeof(struct in6_addr ));
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DODAGVersionNumber = __nesc_ntoh_uint8(dio->version.nxdata);
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__GROUND_STATE = dio->flags & 0x80;

  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
  return;
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void RPLRankP__IP_DIO_Filter__recv(struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IP_DIO__recv(hdr, packet, len, meta);
#line 23
}
#line 23
# 569 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline bool RPLRankP__compareParent(parent_t oldP, parent_t newP)
#line 569
{
  return oldP.etx_hop + oldP.etx <= newP.etx_hop + newP.etx;
}

# 63 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline bool RPLOF0P__RPLOF__OCP(uint16_t ocp)
#line 63
{
  if (ocp == RPLOF_OCP_OF0) {
    return TRUE;
    }
#line 66
  return FALSE;
}

# 4 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static bool RPLRankP__RPLOF__OCP(uint16_t ocp){
#line 4
  unsigned char __nesc_result;
#line 4

#line 4
  __nesc_result = RPLOF0P__RPLOF__OCP(ocp);
#line 4

#line 4
  return __nesc_result;
#line 4
}
#line 4
# 70 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline bool RPLOF0P__RPLOF__objectSupported(uint16_t objectType)
#line 70
{
  if (objectType == RPLOF_OPTION_SOLICITATION) {
      return TRUE;
    }

  return FALSE;
}

# 7 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static bool RPLRankP__RPLOF__objectSupported(uint16_t objectType){
#line 7
  unsigned char __nesc_result;
#line 7

#line 7
  __nesc_result = RPLOF0P__RPLOF__objectSupported(objectType);
#line 7

#line 7
  return __nesc_result;
#line 7
}
#line 7
# 78 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline void RPLOF0P__RPLOF__setMinHopRankIncrease(uint16_t val)
#line 78
{
  RPLOF0P__min_hop_rank_inc = val;
}

# 22 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static void RPLRankP__RPLOF__setMinHopRankIncrease(uint16_t val){
#line 22
  RPLOF0P__RPLOF__setMinHopRankIncrease(val);
#line 22
}
#line 22
# 436 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDODAGConfig(uint8_t IntDouble, 
uint8_t IntMin, 
uint8_t Redun, 
uint8_t RankInc, 
uint8_t HopRankInc)
#line 440
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntDouble = IntDouble;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIOIntMin = IntMin;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__DIORedun = Redun;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MaxRankInc = RankInc;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__MinHopRankInc = HopRankInc;
}

# 49 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static void RPLRankP__RouteInfo__setDODAGConfig(uint8_t DIOIntDouble, uint8_t DIOIntMin, uint8_t DIORedun, uint8_t MaxRankInc, uint8_t MinHopRankInc){
#line 49
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__setDODAGConfig(DIOIntDouble, DIOIntMin, DIORedun, MaxRankInc, MinHopRankInc);
#line 49
}
#line 49
# 17 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
inline static int RPLRankP__IPPacket__findTLV(struct ip_iovec *header, int ext_offset, uint8_t type){
#line 17
  int __nesc_result;
#line 17

#line 17
  __nesc_result = IPPacketC__IPPacket__findTLV(header, ext_offset, type);
#line 17

#line 17
  return __nesc_result;
#line 17
}
#line 17
# 250 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static inline void RPLOF0P__RPLOF__resetRank(void )
#line 250
{
  RPLOF0P__nodeRank = INFINITE_RANK;
  RPLOF0P__minMetric = 0xFFFF;
}

# 15 "/opt/tinyos/tos/lib/net/rpl/RPLOF.nc"
inline static void RPLRankP__RPLOF__resetRank(void ){
#line 15
  RPLOF0P__RPLOF__resetRank();
#line 15
}
#line 15
# 610 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline void RPLRankP__parseDIO(struct ip6_hdr *iph, 
uint8_t *buf, 
int len)
#line 612
{
  uint16_t pParentRank;
  struct in6_addr rDODAGID;
  uint16_t etx = 0xFFFF;
  parent_t tempParent;
  uint8_t parentIndex;
#line 617
  uint8_t myParent;
  uint16_t preRank;
  uint8_t tempPrf;
  bool newDodag = FALSE;

  struct dio_base_t *dio = (struct dio_base_t *)buf;
  struct dio_body_t *dio_body;
  struct dio_metric_header_t *dio_metric_header;
  struct dio_etx_t *dio_etx;
  struct dio_dodag_config_t *dio_dodag_config;

  struct ip_iovec v[2];
  struct ip6_ext dummy_ext;
  int hdr_off;

#line 631
  buf += sizeof(struct dio_base_t );
  len -= sizeof(struct dio_base_t );
  dummy_ext.ip6e_len = (len - 8) / 8;
  v[0].iov_len = sizeof dummy_ext;
  v[0].iov_base = (uint8_t *)&dummy_ext;
  v[0].iov_next = &v[1];
  v[1].iov_len = len;
  v[1].iov_base = buf;
  v[1].iov_next = (void *)0;


  if (RPLRankP__nodeRank == ROOT_RANK) {
#line 642
    return;
    }

  if (__nesc_ntoh_uint8(dio->version.nxdata) != RPLRankP__VERSION && !memcmp(& dio->dodagID, &RPLRankP__DODAGID, sizeof(struct in6_addr ))) {
      RPLRankP__parentNum = 0;
      RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
      RPLRankP__RPLOF__resetRank();
      RPLRankP__nodeRank = INFINITE_RANK;
      RPLRankP__minRank = INFINITE_RANK;
      RPLRankP__resetValid();
    }









  pParentRank = __nesc_ntoh_uint16(dio->dagRank.nxdata);


  ip_memcpy((uint8_t *)&rDODAGID, 
  (uint8_t *)& dio->dodagID, 
  sizeof(struct in6_addr ));
  tempPrf = dio->flags & 0x07;

  if (! !memcmp(&RPLRankP__DODAGID, &RPLRankP__DODAG_MAX, sizeof(struct in6_addr )) && 
  ! !memcmp(&RPLRankP__DODAGID, &rDODAGID, sizeof(struct in6_addr ))) {

      if (RPLRankP__Prf < tempPrf) {

          RPLRankP__ignore = TRUE;
          return;
        }
      else {
#line 677
        if (RPLRankP__Prf > tempPrf) {

            RPLRankP__Prf = tempPrf;
            ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
            (uint8_t *)&rDODAGID, 
            sizeof(struct in6_addr ));
            RPLRankP__parentNum = 0;
            RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
            RPLRankP__RPLOF__resetRank();
            RPLRankP__nodeRank = INFINITE_RANK;
            RPLRankP__minRank = INFINITE_RANK;

            RPLRankP__resetValid();
            newDodag = TRUE;
          }
        else 
#line 691
          {



            newDodag = TRUE;
          }
        }
    }
  else {
#line 697
    if (!memcmp(&RPLRankP__DODAGID, &RPLRankP__DODAG_MAX, sizeof(struct in6_addr ))) {

        RPLRankP__Prf = tempPrf;
        ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
        (uint8_t *)&rDODAGID, 
        sizeof(struct in6_addr ));
        RPLRankP__parentNum = 0;
        RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
        RPLRankP__RPLOF__resetRank();
        RPLRankP__nodeRank = INFINITE_RANK;
        RPLRankP__minRank = INFINITE_RANK;

        newDodag = TRUE;
        RPLRankP__resetValid();
      }
    else 
#line 711
      {
      }
    }



  hdr_off = RPLRankP__IPPacket__findTLV(v, 0, RPL_DIO_TYPE_METRIC) - 
  sizeof(struct ip6_ext );

  RPLRankP__METRICID = 0;
  RPLRankP__OCP = 0;


  if (hdr_off >= 0) {

      dio_body = (struct dio_body_t *)(buf + hdr_off);
      dio_metric_header = (struct dio_metric_header_t *)(buf + hdr_off + 2);




      if (
#line 731
      dio_body->container_len >= sizeof(struct dio_metric_header_t ) && 
      dio_metric_header->routing_obj_type == RPL_ROUTE_METRIC_ETX && __nesc_ntoh_uint16(
      dio_metric_header->object_len.nxdata) == sizeof(struct dio_etx_t )) {

          dio_etx = (struct dio_etx_t *)(dio_metric_header + 1);
          etx = __nesc_ntoh_uint16(dio_etx->etx.nxdata);
          ;
#line 737
          ;
          RPLRankP__METRICID = RPL_ROUTE_METRIC_ETX;
        }
    }
  else 
#line 740
    {
      etx = pParentRank * 10;
    }
#line 757
  hdr_off = RPLRankP__IPPacket__findTLV(v, 0, RPL_DIO_TYPE_DODAG) - 
  sizeof(struct ip6_ext );


  if (hdr_off >= 0 && !RPLRankP__ignore) {
      dio_dodag_config = (struct dio_dodag_config_t *)(buf + hdr_off);





      RPLRankP__OCP = __nesc_ntoh_uint16(dio_dodag_config->ocp.nxdata);
      RPLRankP__MAX_RANK_INCREASE = __nesc_ntoh_uint16(dio_dodag_config->MaxRankInc.nxdata);


      RPLRankP__RouteInfo__setDODAGConfig(__nesc_ntoh_uint8(dio_dodag_config->DIOIntDoubl.nxdata), __nesc_ntoh_uint8(
      dio_dodag_config->DIOIntMin.nxdata), __nesc_ntoh_uint8(
      dio_dodag_config->DIORedun.nxdata), __nesc_ntoh_uint16(
      dio_dodag_config->MaxRankInc.nxdata), __nesc_ntoh_uint16(
      dio_dodag_config->MinHopRankInc.nxdata));
      RPLRankP__RPLOF__setMinHopRankIncrease(__nesc_ntoh_uint16(dio_dodag_config->MinHopRankInc.nxdata));
    }







  ip_memcpy((uint8_t *)& tempParent.parentIP, 
  (uint8_t *)& iph->ip6_src, sizeof(struct in6_addr ));
  tempParent.rank = pParentRank;
  tempParent.etx_hop = 10;
  tempParent.valid = TRUE;
  tempParent.etx = etx;


  if ((
#line 793
  !RPLRankP__RPLOF__objectSupported(RPLRankP__METRICID) || 
  !RPLRankP__RPLOF__OCP(RPLRankP__OCP)) && RPLRankP__parentNum == 0) {


      RPLRankP__insertParent(tempParent);
      RPLRankP__RPLOF__recomputeRoutes();

      RPLRankP__nodeRank = INFINITE_RANK;
      RPLRankP__leafState = TRUE;
      return;
    }

  if ((parentIndex = RPLRankP__getParent(& iph->ip6_src)) != 20) {



      if (newDodag) {

          if (RPLRankP__parentNum != 0) {



              RPLRankP__RPLOF__recomputeRoutes();

              myParent = RPLRankP__getParent(RPLRankP__RPLOF__getParent());

              if (!RPLRankP__compareParent(RPLRankP__parentSet[myParent], tempParent)) {

                  RPLRankP__Prf = tempPrf;
                  ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
                  (uint8_t *)&rDODAGID, 
                  sizeof(struct in6_addr ));
                  RPLRankP__parentNum = 0;
                  RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
                  RPLRankP__resetValid();
                  RPLRankP__insertParent(tempParent);
                  RPLRankP__RPLOF__recomputeRoutes();
                  RPLRankP__getNewRank();
                }
              else 
#line 831
                {

                  RPLRankP__RPLOF__recomputeRoutes();
                  RPLRankP__getNewRank();
                  RPLRankP__ignore = TRUE;
                }
            }
          else 
#line 837
            {

              RPLRankP__Prf = tempPrf;
              ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
              (uint8_t *)&rDODAGID, 
              sizeof(struct in6_addr ));
              RPLRankP__parentNum = 0;
              RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
              RPLRankP__resetValid();
              RPLRankP__insertParent(tempParent);
              RPLRankP__RPLOF__recomputeRoutes();
              RPLRankP__getNewRank();
            }
        }
      else {



          RPLRankP__parentSet[parentIndex].rank = pParentRank;
          RPLRankP__parentSet[parentIndex].etx = etx;
          RPLRankP__RPLOF__recomputeRoutes();
          RPLRankP__getNewRank();
          RPLRankP__ignore = TRUE;
        }
    }
  else {



      if (RPLRankP__parentNum > 20) {
        return;
        }






      if (newDodag) {


          if (RPLRankP__parentNum != 0) {

              RPLRankP__RPLOF__recomputeRoutes();
              myParent = RPLRankP__getParent(RPLRankP__RPLOF__getParent());
              if (!RPLRankP__compareParent(RPLRankP__parentSet[myParent], tempParent)) {


                  RPLRankP__Prf = tempPrf;
                  ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
                  (uint8_t *)&rDODAGID, 
                  sizeof(struct in6_addr ));
                  RPLRankP__parentNum = 0;
                  RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
                  RPLRankP__resetValid();
                  RPLRankP__insertParent(tempParent);
                  RPLRankP__RPLOF__recomputeRoutes();
                  RPLRankP__getNewRank();
                }
              else 
#line 895
                {

                  RPLRankP__ignore = TRUE;
                }
            }
          else 
#line 899
            {



              RPLRankP__Prf = tempPrf;
              ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
              (uint8_t *)&rDODAGID, 
              sizeof(struct in6_addr ));
              RPLRankP__parentNum = 0;
              RPLRankP__VERSION = __nesc_ntoh_uint8(dio->version.nxdata);
              RPLRankP__resetValid();
              RPLRankP__insertParent(tempParent);
              RPLRankP__RPLOF__recomputeRoutes();
              RPLRankP__getNewRank();
            }
        }
      else 
#line 914
        {



          RPLRankP__insertParent(tempParent);
          RPLRankP__RPLOF__recomputeRoutes();
          preRank = RPLRankP__nodeRank;
          RPLRankP__getNewRank();
        }
    }
}










static inline void RPLRankP__IP_DIO__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 936
{
  struct dio_base_t *dio;

#line 938
  dio = (struct dio_base_t *)payload;

  if (!RPLRankP__m_running) {
#line 940
    return;
    }
  if (RPLRankP__nodeRank != ROOT_RANK && __nesc_ntoh_uint16(dio->dagRank.nxdata) != 0xFFFF) {
    RPLRankP__parseDIO(iph, payload, len);
    }

  if (__nesc_ntoh_uint16(dio->dagRank.nxdata) == 0xFFFF && RPLRankP__getParent(& iph->ip6_src) != 20) {
    RPLRankP__evictParent(RPLRankP__getParent(& iph->ip6_src));
    }

  if (RPLRankP__nodeRank > __nesc_ntoh_uint16(dio->dagRank.nxdata) || __nesc_ntoh_uint16(dio->dagRank.nxdata) == INFINITE_RANK) {
      if (!RPLRankP__ignore) {

          RPLRankP__IP_DIO_Filter__recv(iph, payload, len, meta);
        }
      RPLRankP__ignore = FALSE;
    }
}

# 50 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__default__recv(uint8_t code, struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta)
#line 51
{
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__recv(uint8_t arg_0x415698a8, struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  switch (arg_0x415698a8) {
#line 23
    case ICMPV6_CODE_DIO:
#line 23
      RPLRankP__IP_DIO__recv(hdr, packet, len, meta);
#line 23
      break;
#line 23
    case ICMPV6_CODE_DAO:
#line 23
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IP_DAO__recv(hdr, packet, len, meta);
#line 23
      break;
#line 23
    default:
#line 23
      /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__default__recv(arg_0x415698a8, hdr, packet, len, meta);
#line 23
      break;
#line 23
    }
#line 23
}
#line 23
# 39 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCodeDispatchP.nc"
static inline void /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__recv(struct ip6_hdr *iph, void *packet, 
size_t len, struct ip6_metadata *meta)
#line 40
{
  struct icmp6_hdr *icmph = packet;

  /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__IP__recv(icmph->code, iph, packet, len, meta);
}

# 117 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
static inline void ICMPCoreP__ICMP_IP__default__recv(uint8_t type, struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 118
{
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void ICMPCoreP__ICMP_IP__recv(uint8_t arg_0x4134ee70, struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  switch (arg_0x4134ee70) {
#line 23
    case 155:
#line 23
      /*RPLRoutingC.ICMP_RA.ICMPCodeDispatchP*/ICMPCodeDispatchP__1__RA__recv(hdr, packet, len, meta);
#line 23
      /*RPLRoutingEngineC.ICMP_RS.ICMPCodeDispatchP*/ICMPCodeDispatchP__0__RA__recv(hdr, packet, len, meta);
#line 23
      break;
#line 23
    default:
#line 23
      ICMPCoreP__ICMP_IP__default__recv(arg_0x4134ee70, hdr, packet, len, meta);
#line 23
      break;
#line 23
    }
#line 23
}
#line 23
# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool ICMPCoreP__IPAddress__setSource(struct ip6_hdr *hdr){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = IPAddressP__IPAddress__setSource(hdr);
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 59 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
static inline void ICMPCoreP__IP__recv(struct ip6_hdr *iph, 
void *packet, 
size_t len, 
struct ip6_metadata *meta)
#line 62
{
  struct ip6_hdr *hdr = iph;
  struct ip6_packet reply;
  struct ip_iovec v;
  struct icmp6_hdr *req = (struct icmp6_hdr *)packet;
  uint16_t my_cksum;
#line 67
  uint16_t rx_cksum = (((uint16_t )req->cksum >> 8) | ((uint16_t )req->cksum << 8)) & 0xffff;




  req->cksum = 0;
  v.iov_base = packet;
  v.iov_len = len;
  v.iov_next = (void *)0;
  my_cksum = msg_cksum(iph, &v, IANA_ICMP);
  ;
#line 77
  ;

  if (my_cksum != rx_cksum) {
      ;
#line 80
      ;
      return;
    }

  switch (req->type) {
      case ICMP_TYPE_ECHO_REQUEST: 
        req->type = ICMP_TYPE_ECHO_REPLY;

      memset(&reply, 0, sizeof reply);
      memcpy(reply.ip6_hdr.ip6_dst.in6_u.u6_addr8, hdr->ip6_src.in6_u.u6_addr8, 16);
      ICMPCoreP__IPAddress__setSource(& reply.ip6_hdr);

      reply.ip6_hdr.ip6_ctlun.ip6_un2_vfc = 0x60;
      reply.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt = IANA_ICMP;
      reply.ip6_data = &v;

      reply.ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )len << 8) | ((uint16_t )len >> 8)) & 0xffff;
      ICMPCoreP__ICMP_IP__send(ICMP_TYPE_ECHO_REPLY, &reply);
      break;

      default: 
        ICMPCoreP__ICMP_IP__recv(req->type, iph, packet, len, meta);
    }
}

# 349 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline void CoapUdpServerP__LibCoapServer__read(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta)
#line 350
{
  ;
#line 351
  ;
  coap_read(CoapUdpServerP__ctx_server, from, data, len, meta);
  coap_dispatch(CoapUdpServerP__ctx_server);
}

# 52 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
inline static void LibCoapAdapterP__LibCoapServer__read(struct sockaddr_in6 *from, void *data, uint16_t len, struct ip6_metadata *meta){
#line 52
  CoapUdpServerP__LibCoapServer__read(from, data, len, meta);
#line 52
}
#line 52
# 39 "/opt/tinyos/tos/lib/net/coap/LibCoapAdapterP.nc"
static inline void LibCoapAdapterP__libcoap_server_read(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta)
#line 40
{
  LibCoapAdapterP__LibCoapServer__read(from, data, len, meta);
}

static inline void LibCoapAdapterP__UDPServer__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip6_metadata *meta)
#line 45
{

  LibCoapAdapterP__libcoap_server_read(from, data, len, meta);
}

# 180 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__UDP__default__recvfrom(uint8_t clnt, struct sockaddr_in6 *from, 
void *payload, 
uint16_t len, 
struct ip6_metadata *meta)
#line 183
{
}

# 29 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
inline static void UdpP__UDP__recvfrom(uint8_t arg_0x41652e90, struct sockaddr_in6 *src, void *payload, uint16_t len, struct ip6_metadata *meta){
#line 29
  switch (arg_0x41652e90) {
#line 29
    case 0U:
#line 29
      LibCoapAdapterP__UDPServer__recvfrom(src, payload, len, meta);
#line 29
      break;
#line 29
    default:
#line 29
      UdpP__UDP__default__recvfrom(arg_0x41652e90, src, payload, len, meta);
#line 29
      break;
#line 29
    }
#line 29
}
#line 29
# 64 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__IP__recv(struct ip6_hdr *iph, void *packet, size_t len, struct ip6_metadata *meta)
#line 64
{
  uint8_t i;
  struct sockaddr_in6 addr;
  struct udp_hdr *udph = (struct udp_hdr *)packet;
  uint16_t my_cksum;
#line 68
  uint16_t rx_cksum = (((uint16_t )udph->chksum >> 8) | ((uint16_t )udph->chksum << 8)) & 0xffff;
  struct ip_iovec v;

  ;
#line 71
  ;



  for (i = 0; i < UdpP__N_CLIENTS; i++) 
    if (UdpP__local_ports[i] == udph->dstport) {
      break;
      }
  if (i == UdpP__N_CLIENTS) {

      return;
    }
  memcpy(& addr.sin6_addr, & iph->ip6_src, 16);
  addr.sin6_port = udph->srcport;

  udph->chksum = 0;
  v.iov_base = packet;
  v.iov_len = len;
  v.iov_next = (void *)0;

  my_cksum = msg_cksum(iph, &v, IANA_UDP);
  ;
#line 92
  ;
  if (rx_cksum != my_cksum) {
      ;
      ;
#line 95
      ;

      ;
#line 97
      ;


      return;
    }

  ;
  UdpP__UDP__recvfrom(i, &addr, (void *)(udph + 1), len - sizeof(struct udp_hdr ), meta);
}

# 56 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
static inline void IPProtocolsP__IP__default__recv(uint8_t nxt_hdr, struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 57
{
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void IPProtocolsP__IP__recv(uint8_t arg_0x40905ed0, struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  switch (arg_0x40905ed0) {
#line 23
    case IANA_UDP:
#line 23
      UdpP__IP__recv(hdr, packet, len, meta);
#line 23
      break;
#line 23
    case IANA_ICMP:
#line 23
      ICMPCoreP__IP__recv(hdr, packet, len, meta);
#line 23
      break;
#line 23
    default:
#line 23
      IPProtocolsP__IP__default__recv(arg_0x40905ed0, hdr, packet, len, meta);
#line 23
      break;
#line 23
    }
#line 23
}
#line 23
# 14 "/opt/tinyos/tos/lib/net/blip/interfaces/IPPacket.nc"
inline static int IPProtocolsP__IPPacket__findHeader(struct ip_iovec *payload, uint8_t first_type, uint8_t *search_type){
#line 14
  int __nesc_result;
#line 14

#line 14
  __nesc_result = IPPacketC__IPPacket__findHeader(payload, first_type, search_type);
#line 14

#line 14
  return __nesc_result;
#line 14
}
#line 14
# 18 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
static inline void IPProtocolsP__SubIP__recv(struct ip6_hdr *iph, 
void *payload, 
size_t len, 
struct ip6_metadata *meta)
#line 21
{
  int payload_off;
  uint8_t nxt_hdr;
  struct ip_iovec v = { 
  .iov_base = payload, 
  .iov_len = len, 
  .iov_next = (void *)0 };





  nxt_hdr = IPV6_FRAG;
  payload_off = IPProtocolsP__IPPacket__findHeader(&v, iph->ip6_ctlun.ip6_un1.ip6_un1_nxt, &nxt_hdr);
  if (payload_off >= 0 && ((uint16_t *)((uint8_t *)payload + payload_off))[1] != 0) {
    return;
    }

  nxt_hdr = 0xff;
  payload_off = IPProtocolsP__IPPacket__findHeader(&v, iph->ip6_ctlun.ip6_un1.ip6_un1_nxt, &nxt_hdr);
  ;
#line 41
  ;
  if (payload_off >= 0) {
      IPProtocolsP__IP__recv(nxt_hdr, iph, (uint8_t *)payload + payload_off, 
      len - payload_off, meta);
    }
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void IPForwardingEngineP__IP__recv(struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  IPProtocolsP__SubIP__recv(hdr, packet, len, meta);
#line 23
}
#line 23
# 365 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__IPRaw__default__recv(struct ip6_hdr *iph, void *payload, 
size_t len, struct ip6_metadata *meta)
#line 366
{
}

# 23 "/opt/tinyos/tos/lib/net/blip/interfaces/IP.nc"
inline static void IPForwardingEngineP__IPRaw__recv(struct ip6_hdr *hdr, void *packet, size_t len, struct ip6_metadata *meta){
#line 23
  IPForwardingEngineP__IPRaw__default__recv(hdr, packet, len, meta);
#line 23
}
#line 23
# 248 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__IPForward__recv(uint8_t ifindex, struct ip6_hdr *iph, void *payload, 
struct ip6_metadata *meta)
#line 249
{
  struct ip6_packet pkt;
  struct in6_addr *next_hop;
  size_t len = (((uint16_t )iph->ip6_ctlun.ip6_un1.ip6_un1_plen >> 8) | ((uint16_t )iph->ip6_ctlun.ip6_un1.ip6_un1_plen << 8)) & 0xffff;
  route_key_t next_hop_key = ROUTE_INVAL_KEY;
  uint8_t next_hop_ifindex;
  struct ip_iovec v = { 
  .iov_next = (void *)0, 
  .iov_base = payload, 
  .iov_len = len };



  IPForwardingEngineP__IPRaw__recv(iph, payload, len, meta);

  if (IPForwardingEngineP__IPAddress__isLocalAddress(& iph->ip6_dst)) {


      IPForwardingEngineP__IP__recv(iph, payload, len, meta);
    }
  else 
#line 268
    {

      uint8_t nxt_hdr = IPV6_ROUTING;
      int header_off = IPForwardingEngineP__IPPacket__findHeader(&v, iph->ip6_ctlun.ip6_un1.ip6_un1_nxt, &nxt_hdr);

#line 272
      if (! -- iph->ip6_ctlun.ip6_un1.ip6_un1_hlim) {


          return;
        }

      if (header_off >= 0) {




          return;
        }
      else 
#line 284
        {

          struct route_entry *next_hop_entry = 
          IPForwardingEngineP__ForwardingTable__lookupRoute(iph->ip6_dst.in6_u.u6_addr8, 
          128);

#line 289
          if (next_hop_entry == (void *)0) {



              return;
            }
          next_hop = & next_hop_entry->next_hop;
          next_hop_key = next_hop_entry->key;
          next_hop_ifindex = next_hop_entry->ifindex;
        }

      memcpy(& pkt.ip6_hdr, iph, sizeof(struct ip6_hdr ));
      pkt.ip6_data = &v;




      if (!IPForwardingEngineP__ForwardingEvents__approve(next_hop_ifindex, &pkt, next_hop)) {
        return;
        }
      IPForwardingEngineP__do_send(next_hop_ifindex, next_hop, &pkt);
    }
}

# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
inline static void IPNeighborDiscoveryP__IPForward__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta){
#line 28
  IPForwardingEngineP__IPForward__recv(ROUTE_IFACE_154, iph, payload, meta);
#line 28
}
#line 28
# 124 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static inline void IPNeighborDiscoveryP__IPLower__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta)
#line 124
{
  IPNeighborDiscoveryP__IPForward__recv(iph, payload, meta);
}

# 28 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
inline static void IPDispatchP__IPLower__recv(struct ip6_hdr *iph, void *payload, struct ip6_metadata *meta){
#line 28
  IPNeighborDiscoveryP__IPLower__recv(iph, payload, meta);
#line 28
}
#line 28
# 294 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline uint8_t RPLRankP__getPreExistingParent(struct in6_addr *node)
#line 294
{

  uint8_t indexset;

#line 297
  if (RPLRankP__parentNum == 0) {
      return 20;
    }

  for (indexset = 0; indexset < 20; indexset++) {
      if (!memcmp(& RPLRankP__parentSet[indexset].parentIP, node, sizeof(struct in6_addr ))) {
          return indexset;
        }
    }
  return 20;
}

# 41 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngine.nc"
inline static void RPLRankP__RouteInfo__resetTrickle(void ){
#line 41
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle();
#line 41
}
#line 41
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(8U, dt);
#line 64
}
#line 64
#line 92
inline static bool /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__isRunning(void ){
#line 92
  unsigned char __nesc_result;
#line 92

#line 92
  __nesc_result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(7U);
#line 92

#line 92
  return __nesc_result;
#line 92
}
#line 92
# 178 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline bool CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 128 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
#line 97
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
#line 88
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 111 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline uint16_t PacketLinkP__PacketLink__getRetryDelay(message_t *msg)
#line 111
{
  return __nesc_ntoh_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->retryDelay.nxdata);
}

# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void PacketLinkP__DelayTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(1U, dt);
#line 73
}
#line 73
# 75 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg)
#line 75
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->ack.nxdata);
}

# 85 "/opt/tinyos/tos/interfaces/PacketAcknowledgements.nc"
inline static bool PacketLinkP__PacketAcknowledgements__wasAcked(message_t * msg){
#line 85
  unsigned char __nesc_result;
#line 85

#line 85
  __nesc_result = CC2420PacketP__Acks__wasAcked(msg);
#line 85

#line 85
  return __nesc_result;
#line 85
}
#line 85
# 171 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline void PacketLinkP__SubSend__sendDone(message_t *msg, error_t error)
#line 171
{
  if (PacketLinkP__SendState__getState() == PacketLinkP__S_SENDING) {
      PacketLinkP__totalRetries++;
      if (PacketLinkP__PacketAcknowledgements__wasAcked(msg)) {
          PacketLinkP__signalDone(SUCCESS);
          return;
        }
      else {
#line 178
        if (PacketLinkP__totalRetries < PacketLinkP__PacketLink__getRetries(PacketLinkP__currentSendMsg)) {

            if (PacketLinkP__PacketLink__getRetryDelay(PacketLinkP__currentSendMsg) > 0) {

                PacketLinkP__DelayTimer__startOneShot(PacketLinkP__PacketLink__getRetryDelay(PacketLinkP__currentSendMsg));
              }
            else {

                PacketLinkP__send__postTask();
              }

            return;
          }
        }
    }
  PacketLinkP__signalDone(error);
}

# 100 "/opt/tinyos/tos/interfaces/Send.nc"
inline static void CC2420CsmaP__Send__sendDone(message_t * msg, error_t error){
#line 100
  PacketLinkP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 111 "/opt/tinyos/tos/system/StateImplP.nc"
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP__state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/opt/tinyos/tos/interfaces/State.nc"
inline static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(1U, reqState);
#line 51
}
#line 51
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__stopDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__stopDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__stopVReg(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420ControlP__CC2420Power__stopVReg();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 69 "/opt/tinyos/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 105 "/opt/tinyos/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = CC2420TransmitP__StdControl__stop();
#line 105
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__stop());
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 275 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__shutdown(void )
#line 275
{
  CC2420CsmaP__SubControl__stop();
  CC2420CsmaP__CC2420Power__stopVReg();
  CC2420CsmaP__stopDone_task__postTask();
}

# 66 "/opt/tinyos/tos/interfaces/State.nc"
inline static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = StateImplP__State__isState(1U, myState);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 244 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__sendDone_task__runTask(void )
#line 244
{
  error_t packetErr;

#line 246
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    packetErr = CC2420CsmaP__sendErr;
#line 246
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
      CC2420CsmaP__shutdown();
    }
  else {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
    }

  CC2420CsmaP__Send__sendDone(CC2420CsmaP__m_msg, packetErr);
}

# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 67
{
  /* atomic removed: atomic calls only */
#line 67
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 99 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 99
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 99
}
#line 99
# 135 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 58 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 58
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 58
}
#line 58
# 69 "/opt/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 66 "/opt/tinyos/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 66
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable();
#line 66
}
#line 66
# 102 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 102
{
#line 102
  P1IFG &= ~(1 << 0);
}

# 52 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port10__clear();
#line 52
}
#line 52
# 94 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 94
{
#line 94
  P1IE &= ~(1 << 0);
}

# 47 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port10__disable();
#line 47
}
#line 47
# 69 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 61 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void )
#line 57
{
#line 57
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
#line 57
    __nesc_atomic_end(__nesc_atomic); }
}

# 53 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__clr(void ){
#line 53
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr();
#line 53
}
#line 53
# 49 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__clr(void )
#line 49
{
#line 49
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__clr();
}

# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 41
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__clr();
#line 41
}
#line 41
# 78 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void PacketLinkP__DelayTimer__stop(void ){
#line 78
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(1U);
#line 78
}
#line 78
# 81 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static IPDispatchP__SendQueue__t  IPDispatchP__SendQueue__dequeue(void ){
#line 81
  struct send_entry *__nesc_result;
#line 81

#line 81
  __nesc_result = /*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue();
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 22 "/opt/tinyos/tos/lib/net/blip/interfaces/IPForward.nc"
inline static void IPNeighborDiscoveryP__IPForward__sendDone(struct send_info *status){
#line 22
  IPForwardingEngineP__IPForward__sendDone(ROUTE_IFACE_154, status);
#line 22
}
#line 22
# 128 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static inline void IPNeighborDiscoveryP__IPLower__sendDone(struct send_info *status)
#line 128
{
  IPNeighborDiscoveryP__IPForward__sendDone(status);
}

# 22 "/opt/tinyos/tos/lib/net/blip/interfaces/IPLower.nc"
inline static void IPDispatchP__IPLower__sendDone(struct send_info *status){
#line 22
  IPNeighborDiscoveryP__IPLower__sendDone(status);
#line 22
}
#line 22
# 118 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline bool PacketLinkP__PacketLink__wasDelivered(message_t *msg)
#line 118
{
  return PacketLinkP__PacketAcknowledgements__wasAcked(msg);
}

# 71 "/opt/tinyos/tos/interfaces/PacketLink.nc"
inline static bool IPDispatchP__PacketLink__wasDelivered(message_t * msg){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = PacketLinkP__PacketLink__wasDelivered(msg);
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
#line 59
inline static uint16_t IPDispatchP__PacketLink__getRetries(message_t * msg){
#line 59
  unsigned int __nesc_result;
#line 59

#line 59
  __nesc_result = PacketLinkP__PacketLink__getRetries(msg);
#line 59

#line 59
  return __nesc_result;
#line 59
}
#line 59
# 96 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP__SplitControl__stop(void )
#line 96
{
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPING);
      CC2420CsmaP__shutdown();
      return SUCCESS;
    }
  else {
#line 102
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPED)) {
        return EALREADY;
      }
    else {
#line 105
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_TRANSMITTING)) {
          CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPING);

          return SUCCESS;
        }
      else {
#line 110
        if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
            return SUCCESS;
          }
        }
      }
    }
#line 114
  return EBUSY;
}

# 130 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t IPDispatchP__RadioControl__stop(void ){
#line 130
  unsigned char __nesc_result;
#line 130

#line 130
  __nesc_result = CC2420CsmaP__SplitControl__stop();
#line 130

#line 130
  return __nesc_result;
#line 130
}
#line 130
# 65 "/opt/tinyos/tos/system/QueueC.nc"
static inline /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__head(void )
#line 65
{
  return /*IPDispatchC.QueueC*/QueueC__0__queue[/*IPDispatchC.QueueC*/QueueC__0__head];
}

# 73 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static IPDispatchP__SendQueue__t  IPDispatchP__SendQueue__head(void ){
#line 73
  struct send_entry *__nesc_result;
#line 73

#line 73
  __nesc_result = /*IPDispatchC.QueueC*/QueueC__0__Queue__head();
#line 73

#line 73
  return __nesc_result;
#line 73
}
#line 73
# 582 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__Ieee154Send__sendDone(message_t *msg, error_t error)
#line 582
{
  struct send_entry *s_entry = IPDispatchP__SendQueue__head();

  IPDispatchP__radioBusy = FALSE;



  if (IPDispatchP__state == IPDispatchP__S_STOPPING) {
      IPDispatchP__RadioControl__stop();
      IPDispatchP__state = IPDispatchP__S_STOPPED;
      goto done;
    }

  s_entry->info->link_transmissions += IPDispatchP__PacketLink__getRetries(msg);
  s_entry->info->link_fragment_attempts++;

  if (!IPDispatchP__PacketLink__wasDelivered(msg)) {
      ;
#line 599
      ;

      s_entry->info->failed = TRUE;
      IPDispatchP__IPLower__sendDone(s_entry->info);
    }
  else {


    if (s_entry->info->link_fragment_attempts == 
    s_entry->info->link_fragments) {
        IPDispatchP__IPLower__sendDone(s_entry->info);
      }
    }
  done: 

    IPDispatchP__SENDINFO_DECR(s_entry->info);
  IPDispatchP__FragPool__put(s_entry->msg);
  IPDispatchP__SendEntryPool__put(s_entry);
  IPDispatchP__SendQueue__dequeue();

  IPDispatchP__sendTask__postTask();
}

# 100 "/opt/tinyos/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__BareSend__sendDone(message_t * msg, error_t error){
#line 100
  IPDispatchP__Ieee154Send__sendDone(msg, error);
#line 100
}
#line 100
# 250 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__ActiveSend__default__sendDone(message_t *msg, error_t error)
#line 250
{
}

# 100 "/opt/tinyos/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__ActiveSend__sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP__ActiveSend__default__sendDone(msg, error);
#line 100
}
#line 100
# 148 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error)
#line 148
{
  if (CC2420TinyosNetworkP__m_busy_client == CC2420TinyosNetworkP__CLIENT_AM) {
      CC2420TinyosNetworkP__ActiveSend__sendDone(msg, error);
    }
  else 
#line 151
    {
      CC2420TinyosNetworkP__BareSend__sendDone(msg, error);
    }
}

# 100 "/opt/tinyos/tos/interfaces/Send.nc"
inline static void UniqueSendP__Send__sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 104 "/opt/tinyos/tos/chips/cc2520/unique/UniqueSendP.nc"
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP__State__toIdle();
  UniqueSendP__Send__sendDone(msg, error);
}

# 100 "/opt/tinyos/tos/interfaces/Send.nc"
inline static void PacketLinkP__Send__sendDone(message_t * msg, error_t error){
#line 100
  UniqueSendP__SubSend__sendDone(msg, error);
#line 100
}
#line 100
# 228 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline bool RPLRankP__exceedThreshold(uint8_t indexset, uint8_t ID)
#line 228
{
  return RPLRankP__parentSet[indexset].etx_hop > 3 * 10;
}

#line 524
static inline void RPLRankP__ForwardingEvents__linkResult(struct in6_addr *node, 
struct send_info *info)
#line 525
{
  uint8_t indexset;
#line 526
  uint8_t myParent;
  uint16_t etx_now = info->link_transmissions * 10 / 
  info->link_fragment_attempts;

  ;
#line 530
  ;
  ;
#line 531
  ;
  ;
#line 532
  ;

  myParent = RPLRankP__getParent(RPLRankP__RPLOF__getParent());

  if (RPLRankP__nodeRank == ROOT_RANK) {
      return;
    }

  for (indexset = 0; indexset < 20; indexset++) {
      if (RPLRankP__parentSet[indexset].valid && 
      !memcmp(& RPLRankP__parentSet[indexset].parentIP, node, sizeof(struct in6_addr ))) {
          break;
        }
    }

  if (indexset != 20) {
      RPLRankP__parentSet[indexset].etx_hop = (
      RPLRankP__parentSet[indexset].etx_hop * 8 + etx_now * 2) / 10;

      if (RPLRankP__exceedThreshold(indexset, RPLRankP__METRICID)) {
          RPLRankP__evictParent(indexset);
          if (indexset == myParent && RPLRankP__parentNum > 0) {
            RPLRankP__RPLOF__recomputeRoutes();
            }
        }
#line 556
      RPLRankP__getNewRank();






      return;
    }
}

# 352 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__ForwardingEvents__default__linkResult(uint8_t idx, struct in6_addr *host, 
struct send_info *info)
#line 353
{
}

# 39 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingEvents.nc"
inline static void IPForwardingEngineP__ForwardingEvents__linkResult(uint8_t arg_0x40925e40, struct in6_addr *dest, struct send_info *info){
#line 39
  switch (arg_0x40925e40) {
#line 39
    case RPL_IFACE:
#line 39
      RPLRankP__ForwardingEvents__linkResult(dest, info);
#line 39
      break;
#line 39
    default:
#line 39
      IPForwardingEngineP__ForwardingEvents__default__linkResult(arg_0x40925e40, dest, info);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 108 "CoapBlipP.nc"
static inline void CoapBlipP__RadioControl__stopDone(error_t e)
#line 108
{
}

# 138 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void IPStackControlP__SplitControl__stopDone(error_t error){
#line 138
  CoapBlipP__RadioControl__stopDone(error);
#line 138
}
#line 138
# 37 "/opt/tinyos/tos/lib/net/blip/IPStackControlP.nc"
static inline void IPStackControlP__SubSplitControl__stopDone(error_t error)
#line 37
{
  IPStackControlP__SplitControl__stopDone(error);
}

# 138 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void IPDispatchP__SplitControl__stopDone(error_t error){
#line 138
  IPStackControlP__SubSplitControl__stopDone(error);
#line 138
}
#line 138
# 189 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__RadioControl__stopDone(error_t error)
#line 189
{
  IPDispatchP__SplitControl__stopDone(error);
}

# 138 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__stopDone(error_t error){
#line 138
  IPDispatchP__RadioControl__stopDone(error);
#line 138
}
#line 138
# 265 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__stopDone_task__runTask(void )
#line 265
{
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPED);
  CC2420CsmaP__SplitControl__stopDone(SUCCESS);
}

# 104 "CoapBlipP.nc"
static inline void CoapBlipP__RadioControl__startDone(error_t e)
#line 104
{
  ;
#line 105
  ;
}

# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void IPStackControlP__SplitControl__startDone(error_t error){
#line 113
  CoapBlipP__RadioControl__startDone(error);
#line 113
}
#line 113
# 34 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static bool IPStackControlP__IPAddress__getGlobalAddr(struct in6_addr *addr){
#line 34
  unsigned char __nesc_result;
#line 34

#line 34
  __nesc_result = IPAddressP__IPAddress__getGlobalAddr(addr);
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 48 "/opt/tinyos/tos/lib/net/blip/IPStackControlP.nc"
static inline error_t IPStackControlP__StdControl__default__start(void )
#line 48
{
#line 48
  return SUCCESS;
}

# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
inline static error_t IPStackControlP__StdControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = IPStackControlP__StdControl__default__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 16 "/opt/tinyos/tos/lib/net/blip/IPStackControlP.nc"
static inline void IPStackControlP__SubSplitControl__startDone(error_t error)
#line 16
{
  struct in6_addr addr;

#line 18
  if (error == SUCCESS) {
      IPStackControlP__StdControl__start();
    }


  if (IPStackControlP__IPAddress__getGlobalAddr(&addr)) {
      IPStackControlP__RoutingControl__start();
    }

  IPStackControlP__SplitControl__startDone(error);
}

# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void IPDispatchP__SplitControl__startDone(error_t error){
#line 113
  IPStackControlP__SubSplitControl__startDone(error);
#line 113
}
#line 113
# 64 "/opt/tinyos/tos/lib/timer/Timer.nc"
inline static void IPDispatchP__ExpireTimer__startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(2U, dt);
#line 64
}
#line 64
# 63 "/opt/tinyos/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led2Toggle(void )
#line 63
{
}

# 100 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void IPDispatchP__Leds__led2Toggle(void ){
#line 100
  NoLedsC__Leds__led2Toggle();
#line 100
}
#line 100
# 174 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__RadioControl__startDone(error_t error)
#line 174
{




  if (error == SUCCESS) {
      IPDispatchP__Leds__led2Toggle();
      IPDispatchP__ExpireTimer__startPeriodic(FRAG_EXPIRE_TIME);
      IPDispatchP__state = IPDispatchP__S_RUNNING;
      IPDispatchP__radioBusy = FALSE;
    }

  IPDispatchP__SplitControl__startDone(error);
}

# 113 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__startDone(error_t error){
#line 113
  IPDispatchP__RadioControl__startDone(error);
#line 113
}
#line 113
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__set();
#line 40
}
#line 40
# 196 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 196
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 197
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 199
        CC2420ControlP__SpiResource__release();

        {
#line 199
          __nesc_atomic_end(__nesc_atomic); 
#line 199
          return __nesc_temp;
        }
      }
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420ControlP__Resource__release();
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 268 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 268
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 271
            FAIL;

            {
#line 271
              __nesc_atomic_end(__nesc_atomic); 
#line 271
              return __nesc_temp;
            }
          }
        }
#line 273
      CC2420ControlP__SRXON__strobe();
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 86 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 86
{
#line 86
  P1IE |= 1 << 0;
}

# 42 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port10__enable();
#line 42
}
#line 42
# 118 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__edge(bool l2h)
#line 118
{
  /* atomic removed: atomic calls only */
#line 119
  {
    if (l2h) {
#line 120
      P1IES &= ~(1 << 0);
      }
    else {
#line 121
      P1IES |= 1 << 0;
      }
  }
}

# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 67
}
#line 67
# 52 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 65
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 54 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 157 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 157
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 168 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__start(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__abortSpiRelease = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420TransmitP__StdControl__start();
#line 95
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__StdControl__start());
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 257 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__startDone_task__runTask(void )
#line 257
{
  CC2420CsmaP__SubControl__start();
  CC2420CsmaP__CC2420Power__rxOn();
  CC2420CsmaP__Resource__release();
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
  CC2420CsmaP__SplitControl__startDone(SUCCESS);
}

# 141 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline error_t RPLRankP__StdControl__start(void )
#line 141
{
  uint8_t indexset;

  RPLRankP__DODAG_MAX.in6_u.u6_addr16[7] = (((uint16_t )0 << 8) | ((uint16_t )0 >> 8)) & 0xffff;

  ip_memcpy((uint8_t *)&RPLRankP__DODAGID, 
  (uint8_t *)&RPLRankP__DODAG_MAX, 
  sizeof(struct in6_addr ));

  for (indexset = 0; indexset < 20; indexset++) {
      RPLRankP__parentSet[indexset].valid = FALSE;
    }

  RPLRankP__m_running = TRUE;
  return SUCCESS;
}

# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RankControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = RPLRankP__StdControl__start();
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 500 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__start(void )
#line 500
{

  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running) {
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__postTask();
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RankControl__start();
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running = TRUE;
    }
  return SUCCESS;
}

# 94 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__start(void )
#line 94
{
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__startDAO();
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__m_running = TRUE;
  return SUCCESS;
}

# 106 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP__BarePacket__payloadLength(message_t *msg)
#line 106
{
  cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 108
  return __nesc_ntoh_leuint8(hdr->length.nxdata) + 1 - MAC_FOOTER_SIZE;
}

# 78 "/opt/tinyos/tos/interfaces/Packet.nc"
inline static uint8_t IPDispatchP__BarePacket__payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP__BarePacket__payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 297 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 297
{
  uint8_t *base = target;

#line 299
  base[0] = value;
  return value;
}

# 45 "/opt/tinyos/tos/interfaces/State.nc"
inline static error_t PacketLinkP__SendState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(4U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 130 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static inline error_t PacketLinkP__Send__send(message_t *msg, uint8_t len)
#line 130
{
  error_t error;

#line 132
  if (PacketLinkP__SendState__requestState(PacketLinkP__S_SENDING) == SUCCESS) {

      PacketLinkP__currentSendMsg = msg;
      PacketLinkP__currentSendLen = len;
      PacketLinkP__totalRetries = 0;

      if (PacketLinkP__PacketLink__getRetries(msg) > 0) {
          PacketLinkP__PacketAcknowledgements__requestAck(msg);
        }

      if ((error = PacketLinkP__SubSend__send(msg, len)) != SUCCESS) {
          PacketLinkP__SendState__toIdle();
        }

      return error;
    }
  return EBUSY;
}

# 75 "/opt/tinyos/tos/interfaces/Send.nc"
inline static error_t UniqueSendP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = PacketLinkP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 42 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/opt/tinyos/tos/interfaces/State.nc"
inline static error_t UniqueSendP__State__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(2U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 75 "/opt/tinyos/tos/chips/cc2520/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP__State__requestState(UniqueSendP__S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP__CC2420PacketBody__getHeader(msg)->dsn.nxdata, UniqueSendP__localSendId++);

      if ((error = UniqueSendP__SubSend__send(msg, len)) != SUCCESS) {
          UniqueSendP__State__toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 75 "/opt/tinyos/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP__SubSend__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = UniqueSendP__Send__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 124 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__BareSend__send(message_t *msg, uint8_t len)
#line 124
{
  CC2420TinyosNetworkP__BarePacket__setPayloadLength(msg, len);
  CC2420TinyosNetworkP__m_busy_client = CC2420TinyosNetworkP__CLIENT_BARE;
  return CC2420TinyosNetworkP__SubSend__send(msg, 0);
}

# 75 "/opt/tinyos/tos/interfaces/Send.nc"
inline static error_t IPDispatchP__Ieee154Send__send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420TinyosNetworkP__BareSend__send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 53 "/opt/tinyos/tos/system/QueueC.nc"
static inline bool /*IPDispatchC.QueueC*/QueueC__0__Queue__empty(void )
#line 53
{
  return /*IPDispatchC.QueueC*/QueueC__0__size == 0;
}

# 50 "/opt/tinyos/tos/interfaces/Queue.nc"
inline static bool IPDispatchP__SendQueue__empty(void ){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = /*IPDispatchC.QueueC*/QueueC__0__Queue__empty();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 431 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__sendTask__runTask(void )
#line 431
{
  struct send_entry *s_entry;



  if (IPDispatchP__radioBusy || IPDispatchP__state != IPDispatchP__S_RUNNING) {
#line 436
    return;
    }
#line 437
  if (IPDispatchP__SendQueue__empty()) {
#line 437
    return;
    }
  s_entry = IPDispatchP__SendQueue__head();






  if (s_entry->info->failed) {
      ;
      goto fail;
    }


  if (
#line 451
  IPDispatchP__Ieee154Send__send(s_entry->msg, 
  IPDispatchP__BarePacket__payloadLength(s_entry->msg)) != SUCCESS) {
      ;
      goto fail;
    }
  else 
#line 455
    {
      IPDispatchP__radioBusy = TRUE;
    }

  return;
  fail: 
    ;
#line 461
  ;
  IPDispatchP__sendTask__postTask();
  ;



  s_entry->info->failed = TRUE;
  IPDispatchP__SENDINFO_DECR(s_entry->info);
  IPDispatchP__FragPool__put(s_entry->msg);
  IPDispatchP__SendEntryPool__put(s_entry);
  IPDispatchP__SendQueue__dequeue();
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 184 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 181 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(uint8_t id)
#line 181
{
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(uint8_t arg_0x40ca05e0){
#line 102
  switch (arg_0x40ca05e0) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 102
      CC2420SpiP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__default__granted(arg_0x40ca05e0);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 130 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(uint8_t id)
#line 130
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Resource__granted(id);
}

# 202 "/opt/tinyos/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x40daa520){
#line 102
  switch (arg_0x40daa520) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 102
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x40daa520);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 190 "/opt/tinyos/tos/system/ArbiterP.nc"
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

# 252 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 252
{
}

# 82 "/opt/tinyos/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(uint8_t arg_0x40c9a1b8, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
  switch (arg_0x40c9a1b8) {
#line 82
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 82
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    default:
#line 82
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__default__sendDone(arg_0x40c9a1b8, txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    }
#line 82
}
#line 82
# 245 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone(void )
#line 245
{
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__sendDone(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf, /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len, 
  SUCCESS);
}

#line 228
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__runTask(void )
#line 228
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 229
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
#line 229
    __nesc_atomic_end(__nesc_atomic); }
}

# 486 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 487
{
}

# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 40
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__set();
#line 40
}
#line 40
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP__receiveDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ReceiveP__receiveDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 389 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg)
#line 389
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP__m_msg) {
      ack_header = CC2420TransmitP__CC2420PacketBody__getHeader(ack_msg);
      msg_header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);

      if (CC2420TransmitP__m_state == CC2420TransmitP__S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.nxdata) == __nesc_ntoh_leuint8(ack_header->dsn.nxdata)) {
          CC2420TransmitP__BackoffTimer__stop();

          msg_metadata = CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.nxdata);

          __nesc_hton_int8(msg_metadata->ack.nxdata, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.nxdata, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.nxdata, ack_buf[length] & 0x7f);
          CC2420TransmitP__signalDone(SUCCESS);
        }
    }
}

# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP__CC2420Receive__receive(type, message);
#line 63
}
#line 63
# 70 "/opt/tinyos/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP__PacketTimeStamp__clear(message_t * msg){
#line 70
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 70
}
#line 70








inline static void CC2420ReceiveP__PacketTimeStamp__set(message_t * msg, CC2420ReceiveP__PacketTimeStamp__size_type value){
#line 78
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 78
}
#line 78
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__HplGeneralIO__get(void ){
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
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__8__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 59 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 73 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__HplGeneralIO__get(void ){
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
# 51 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__GeneralIO__get(void )
#line 51
{
#line 51
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__HplGeneralIO__get();
}

# 43 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__7__GeneralIO__get();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 209 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__clr();
#line 41
}
#line 41
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SACK__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SACK);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 382 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void )
#line 382
{
  /* atomic removed: atomic calls only */
#line 383
  {
    unsigned char __nesc_temp = 
#line 383
    CC2420ControlP__hwAutoAckDefault;

#line 383
    return __nesc_temp;
  }
}

# 112 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420ControlP__CC2420Config__isHwAutoAckDefault();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 389 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void )
#line 389
{
  /* atomic removed: atomic calls only */
#line 390
  {
    unsigned char __nesc_temp = 
#line 390
    CC2420ControlP__autoAckEnabled;

#line 390
    return __nesc_temp;
  }
}

# 117 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void ){
#line 117
  unsigned char __nesc_result;
#line 117

#line 117
  __nesc_result = CC2420ControlP__CC2420Config__isAutoAckEnabled();
#line 117

#line 117
  return __nesc_result;
#line 117
}
#line 117
# 530 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 531
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((unsigned short )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 535
  CC2420ReceiveP__rxFrameLength = buf[0];

  switch (CC2420ReceiveP__m_state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FCF;



      if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) 



        {

          CC2420ReceiveP__flush();
        }
      else {
          if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
              CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
            }

          if (CC2420ReceiveP__rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP__rxFrameLength > 0) {
                  if (CC2420ReceiveP__rxFrameLength > CC2420ReceiveP__SACK_HEADER_LENGTH) {

                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP__receivingPacket = FALSE;
                  CC2420ReceiveP__CSN__set();
                  CC2420ReceiveP__SpiResource__release();
                  CC2420ReceiveP__waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP__flush();
            }
        }
      break;

      case CC2420ReceiveP__S_RX_FCF: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;










      if (CC2420ReceiveP__CC2420Config__isAutoAckEnabled() && !CC2420ReceiveP__CC2420Config__isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 597
          header->fcf.nxdata) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.nxdata) == CC2420ReceiveP__CC2420Config__getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.nxdata) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__SACK__strobe();
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__RXFIFO__beginRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
              CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP__RXFIFO__continueRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
      CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 

        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__m_missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }




      if ((((
#line 626
      CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get())
       || !CC2420ReceiveP__m_timestamp_size)
       || CC2420ReceiveP__rxFrameLength <= 10) {
          CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP__m_timestamp_size == 1) {
            CC2420ReceiveP__PacketTimeStamp__set(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__m_timestamp_head]);
            }
#line 634
          CC2420ReceiveP__m_timestamp_head = (CC2420ReceiveP__m_timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__m_timestamp_size--;

          if (CC2420ReceiveP__m_timestamp_size > 0) {
              CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
              CC2420ReceiveP__m_timestamp_head = 0;
              CC2420ReceiveP__m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP__rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 648
          CC2420ReceiveP__CC2420Receive__receive(type, CC2420ReceiveP__m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP__receiveDone_task__postTask();
              return;
            }
        }

      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 370 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x40c03010, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x40c03010) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0x40c03010, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 288 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(message_t *msg)
#line 288
{
}

# 81 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(msg);
#line 81
}
#line 81
# 243 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime)
#line 243
{
  CC2420TransmitP__myInitialBackoff = backoffTime + 1;
}

# 60 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime){
#line 60
  CC2420TransmitP__RadioBackoff__setInitialBackoff(backoffTime);
#line 60
}
#line 60
# 223 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 223
{
  CC2420CsmaP__SubBackoff__setInitialBackoff(CC2420CsmaP__Random__rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestInitialBackoff(msg);
}

# 81 "/opt/tinyos/tos/chips/cc2520/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__sendDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__sendDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 205 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err)
#line 205
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 206
    CC2420CsmaP__sendErr = err;
#line 206
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP__sendDone_task__postTask();
}

# 73 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error){
#line 73
  CC2420CsmaP__CC2420Transmit__sendDone(p_msg, error);
#line 73
}
#line 73
# 454 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 455
{

  CC2420TransmitP__CSN__set();
  if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 459
      {
        CC2420TransmitP__CSN__clr();
        CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CSN__set();
      }
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
    }
  else {
#line 468
    if (!CC2420TransmitP__m_cca) {
        /* atomic removed: atomic calls only */
#line 469
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP__attemptSend();
      }
    else {
        CC2420TransmitP__releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 476
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_SAMPLE_CCA;
        }

        CC2420TransmitP__RadioBackoff__requestInitialBackoff(CC2420TransmitP__m_msg);
        CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myInitialBackoff);
      }
    }
}

# 668 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 668
{
}

# 373 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x40c03010, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x40c03010) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0x40c03010, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__TXCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 533 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__writeTxctrl(void )
#line 533
{
  /* atomic removed: atomic calls only */
#line 534
  {
    CC2420ControlP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
    3 << CC2420_TXCTRL_PA_CURRENT)) | (
    1 << CC2420_TXCTRL_RESERVED)) | ((
    31 & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
  }
}

# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 90 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 90
{
#line 90
  P1IE |= 1 << 4;
}

# 42 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 42
  HplMsp430InterruptP__Port14__enable();
#line 42
}
#line 42
# 142 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__edge(bool l2h)
#line 142
{
  /* atomic removed: atomic calls only */
#line 143
  {
    if (l2h) {
#line 144
      P1IES &= ~(1 << 4);
      }
    else {
#line 145
      P1IES |= 1 << 4;
      }
  }
}

# 67 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 67
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 67
}
#line 67
# 106 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 106
{
#line 106
  P1IFG &= ~(1 << 4);
}

# 52 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 52
  HplMsp430InterruptP__Port14__clear();
#line 52
}
#line 52
# 98 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 98
{
#line 98
  P1IE &= ~(1 << 4);
}

# 47 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 47
  HplMsp430InterruptP__Port14__disable();
#line 47
}
#line 47
# 69 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 69
{
  /* atomic removed: atomic calls only */
#line 70
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 52
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 52
{
  /* atomic removed: atomic calls only */
#line 53
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 61
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 53 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 224 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 224
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 225
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 227
            FAIL;

            {
#line 227
              __nesc_atomic_end(__nesc_atomic); 
#line 227
              return __nesc_temp;
            }
          }
        }
#line 230
      CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
      CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();

      CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP__writeFsctrl();
      CC2420ControlP__writeMdmctrl0();

      CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));

      CC2420ControlP__writeTxctrl();
    }
#line 252
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 214 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__Resource__granted(void )
#line 214
{
  CC2420CsmaP__CC2420Power__startOscillator();
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 102
  CC2420CsmaP__Resource__granted();
#line 102
}
#line 102
# 41 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 41
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__clr();
#line 41
}
#line 41
# 413 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 413
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 53 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 399 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 399
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__writeFsctrl();
  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

#line 545
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data)
#line 545
{
}

# 63 "/opt/tinyos/tos/interfaces/Read.nc"
inline static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val){
#line 63
  CC2420ControlP__ReadRssi__default__readDone(result, val);
#line 63
}
#line 63
# 120 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 287 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 418 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 418
{
  uint16_t data = 0;

#line 420
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP__ReadRssi__readDone(SUCCESS, data);
}

# 416 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 416
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 419
    {
      cur_state = CC2420TransmitP__m_state;
    }
#line 421
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;

      case CC2420TransmitP__S_BEGIN_TRANSMIT: 
        CC2420TransmitP__attemptSend();
      break;

      case CC2420TransmitP__S_CANCEL: 
        CC2420TransmitP__CSN__clr();
      CC2420TransmitP__SFLUSHTX__strobe();
      CC2420TransmitP__CSN__set();
      CC2420TransmitP__releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 437
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
        }
#line 439
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP__releaseSpiResource();
      break;
    }
}

# 513 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 513
{







  CC2420ReceiveP__receive();
}

# 367 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 102 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x40c04558){
#line 102
  switch (arg_0x40c04558) {
#line 102
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 102
      CC2420ControlP__SpiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 102
      CC2420ControlP__SyncResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 102
      CC2420ControlP__RssiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 102
      CC2420TransmitP__SpiResource__granted();
#line 102
      break;
#line 102
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 102
      CC2420ReceiveP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420SpiP__Resource__default__granted(arg_0x40c04558);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 358 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 63 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__IEEEADR__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Ram__write(CC2420_RAM_IEEEADR, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 49 "/opt/tinyos/tos/lib/net/blip/Ieee154AddressP.nc"
static inline void Ieee154AddressP__CC2420Config__syncDone(error_t err)
#line 49
{
}

# 709 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 709
{
}

# 55 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 55
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 55
  Ieee154AddressP__CC2420Config__syncDone(error);
#line 55
}
#line 55
# 469 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 469
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 470
    CC2420ControlP__m_sync_busy = FALSE;
#line 470
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

# 88 "/opt/tinyos/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 323 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__sync(void )
#line 323
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 324
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 326
            FAIL;

            {
#line 326
              __nesc_atomic_end(__nesc_atomic); 
#line 326
              return __nesc_temp;
            }
          }
        }
#line 329
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 332
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 335
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 465
static inline void CC2420ControlP__sync__runTask(void )
#line 465
{
  CC2420ControlP__CC2420Config__sync();
}

# 368 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteAdded(void )
#line 368
{
}

# 43 "/opt/tinyos/tos/lib/net/blip/interfaces/ForwardingTableEvents.nc"
inline static void IPForwardingEngineP__ForwardingTableEvents__defaultRouteAdded(void ){
#line 43
  IPForwardingEngineP__ForwardingTableEvents__default__defaultRouteAdded();
#line 43
}
#line 43
# 96 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__defaultRouteAddedTask__runTask(void )
#line 96
{
  IPForwardingEngineP__ForwardingTableEvents__defaultRouteAdded();
}

# 822 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline coap_pdu_t *CoapUdpServerP__handle_delete(coap_context_t *ctx, coap_queue_t *node, void *data)
#line 822
{

  return (void *)0;
}

#line 815
static inline coap_pdu_t *CoapUdpServerP__handle_post(coap_context_t *ctx, coap_queue_t *node, void *data)
#line 815
{

  return (void *)0;
}

#line 656
static inline coap_pdu_t *CoapUdpServerP__handle_put(coap_context_t *ctx, coap_queue_t *node, void *data)
#line 656
{
  coap_pdu_t *pdu;
  coap_uri_t uri;
  coap_opt_t *tok;
  coap_resource_t *resource;
  unsigned char mediatype = 0xff;
  coap_opt_t *ct;
#line 662
  coap_opt_t *block;
  unsigned int blklen;
#line 663
  unsigned int blk;
  int code;
#line 664
  int finished = 1;
  unsigned int len;
  unsigned char *databuf;


  if (!coap_get_request_uri(node->pdu, &uri)) {
    return (void *)0;
    }

  if (! uri.path.length) {
      pdu = CoapUdpServerP__new_response(ctx, node, 80);

      if (!pdu) {
        return (void *)0;
        }
      CoapUdpServerP__add_contents(pdu, 0, sizeof "CoAPUdpServer: It works!!" - 1, (unsigned char *)"CoAPUdpServer: It works!!");
      return pdu;
    }


  if (!(resource = coap_get_resource(ctx, &uri))) {
    return CoapUdpServerP__new_response(ctx, node, 164);
    }
  if (! resource->writable) {
    return CoapUdpServerP__new_response(ctx, node, 160);
    }

  coap_get_data(node->pdu, &len, &databuf);
  if (len == 0) {
    return CoapUdpServerP__new_response(ctx, node, 81);
    }
  block = coap_check_option(node->pdu, 13);
  if (block) {
      blk = coap_decode_var_bytes((unsigned char *)& *block + (block->lval.flag == 15 ? 2 : 1), 
      block->lval.flag == 15 ? block->lval.length + 15 : block->sval.length);
      blklen = 16 << (blk & 0x07);
    }
  else 
#line 700
    {
      blklen = 512;
      blk = coap_fls(blklen >> 4) - 1;
    }

  tok = coap_check_option(node->pdu, 1);
  resource->mediatype = tok ? *((unsigned char *)& *tok + (tok->lval.flag == 15 ? 2 : 1)) : 0xff;
  resource->dirty = 1;


  if (resource->data) {

      if (
#line 711
      resource->mediatype == 0xff
       && (ct = coap_check_option(node->pdu, 1))) {
          mediatype = *((unsigned char *)& *ct + (ct->lval.flag == 15 ? 2 : 1));
        }

      code = resource->data(&uri, 
      & node->pdu->hdr->id, 
      &mediatype, (
      blk & ~0x0f) << (blk & 0x07), 
      node->pdu->data, 
      &len, 
      &finished, 
      3);

      if (resource->splitphase) {
          if (code == 300) {

              CoapUdpServerP__coap_save_splitphase(ctx, node);
              return (void *)0;
            }
          else 
#line 730
            {
              return CoapUdpServerP__new_response(ctx, node, 160);
            }
        }
      else 
#line 733
        {

          return CoapUdpServerP__new_response(ctx, node, 200);
        }
    }
  else 
#line 737
    {

      return CoapUdpServerP__new_response(ctx, node, 200);
    }
}

# 58 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void )
#line 58
{
#line 58
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 58
    * (volatile uint8_t * )49U ^= 0x01 << 6;
#line 58
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 58
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle();
#line 58
}
#line 58
# 50 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 50
{
#line 50
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 42 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 42
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 42
}
#line 42
# 114 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 114
{
  LedsP__Led2__toggle();
  ;
#line 116
  ;
}

# 100 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void CoapUdpServerP__Leds__led2Toggle(void ){
#line 100
  LedsP__Leds__led2Toggle();
#line 100
}
#line 100
# 404 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline coap_opt_t *CoapUdpServerP__coap_next_option(coap_pdu_t *pdu, coap_opt_t *opt)
#line 404
{
  coap_opt_t *next;

#line 406
  if (!pdu || !opt) {
    return (void *)0;
    }
  next = (coap_opt_t *)((unsigned char *)opt + ((opt->lval.flag == 15 ? opt->lval.length + 15 : opt->sval.length) + (opt->lval.flag == 15 ? 2 : 1)));
  return (unsigned char *)next < pdu->data && next->sval.delta == 0 ? next : (void *)0;
}

static inline int CoapUdpServerP__mediatype_matches(coap_pdu_t *pdu, unsigned char mediatype)
#line 413
{
  coap_opt_t *ct;

  if (mediatype == 0xff) {
    return 1;
    }
  for (ct = coap_check_option(pdu, 1); ct; ct = CoapUdpServerP__coap_next_option(pdu, ct)) {
      if (*((unsigned char *)& *ct + (ct->lval.flag == 15 ? 2 : 1)) == mediatype) {
        return 1;
        }
    }
  return 0;
}



static inline coap_pdu_t *CoapUdpServerP__handle_get(coap_context_t *ctx, coap_queue_t *node, void *data)
#line 429
{
  coap_pdu_t *pdu;
  coap_uri_t uri;
  coap_resource_t *resource;
  coap_opt_t *block;
#line 433
  coap_opt_t *ct;
  unsigned int blklen;
#line 434
  unsigned int blk;
  int code;
#line 435
  int finished = 1;
  unsigned char mediatype = 0xff;
  static unsigned char buf[700];

  if (!coap_get_request_uri(node->pdu, &uri)) {
    return (void *)0;
    }


  if (! uri.path.length) {
      pdu = CoapUdpServerP__new_response(ctx, node, 80);

      if (!pdu) {
        return (void *)0;
        }
      CoapUdpServerP__add_contents(pdu, 0, sizeof "CoAPUdpServer: It works!!" - 1, (unsigned char *)"CoAPUdpServer: It works!!");
      return pdu;
    }


  if (!(resource = coap_get_resource(ctx, &uri))) {
    return CoapUdpServerP__new_response(ctx, node, 164);
    }


  if (
#line 459
  coap_check_option(node->pdu, 1)
   && !CoapUdpServerP__mediatype_matches(node->pdu, resource->mediatype)) {
    return CoapUdpServerP__new_response(ctx, node, 175);
    }
  block = coap_check_option(node->pdu, 13);
  if (block) {
      blk = coap_decode_var_bytes((unsigned char *)& *block + (block->lval.flag == 15 ? 2 : 1), 
      block->lval.flag == 15 ? block->lval.length + 15 : block->sval.length);
      blklen = 16 << (blk & 0x07);
    }
  else 
#line 468
    {
      blklen = 512;
      blk = coap_fls(blklen >> 4) - 1;
    }



  if (resource->data) {

      if (
#line 476
      resource->mediatype == 0xff
       && (ct = coap_check_option(node->pdu, 1))) {
          mediatype = *((unsigned char *)& *ct + (ct->lval.flag == 15 ? 2 : 1));
        }


      code = resource->data(&uri, 
      & node->pdu->hdr->id, 
      &mediatype, (
      blk & ~0x0f) << (blk & 0x07), 
      buf, 
      &blklen, 
      &finished, 
      1);


      if (resource->splitphase) {
          if (code == 300) {
              ;
#line 494
              ;



              CoapUdpServerP__Leds__led2Toggle();

              CoapUdpServerP__coap_save_splitphase(ctx, node);

              return (void *)0;
            }
          else 
#line 503
            {
              ;
#line 504
              ;

              return CoapUdpServerP__new_response(ctx, node, code);
            }
        }
      else 
#line 508
        {


          pdu = CoapUdpServerP__new_response(ctx, node, code);

          if (!pdu) {
            return (void *)0;
            }

          if (!coap_add_data(pdu, blklen, buf)) {
              coap_delete_pdu(pdu);
              if (!(pdu = CoapUdpServerP__new_response(CoapUdpServerP__ctx_server, node, 200))) {
                }
            }


          return pdu;
        }
    }
  else 
#line 526
    {

      return CoapUdpServerP__new_response(ctx, node, 200);
    }
}

#line 829
static inline void CoapUdpServerP__message_handler(coap_context_t *ctx, coap_queue_t *node, void *data)
#line 829
{
  coap_pdu_t *pdu = (void *)0;
  coap_uri_t uri;
  coap_resource_t *resource;




  if (node->pdu->hdr->version != 1) {

      return;
    }


  if (!coap_get_request_uri(node->pdu, &uri)) {
    return;
    }
  resource = coap_get_resource(ctx, &uri);

  switch (node->pdu->hdr->code) {
      case 1: 
        pdu = CoapUdpServerP__handle_get(ctx, node, data);

      if (!pdu && node->pdu->hdr->type == 0) {
          if (resource && resource->splitphase) {
              break;
            }
          pdu = CoapUdpServerP__new_rst(ctx, node, 200);
        }
      break;
      case 3: 
        pdu = CoapUdpServerP__handle_put(ctx, node, data);

      if (!pdu && node->pdu->hdr->type == 0) {
          if (resource && resource->splitphase) {
              break;
            }
          pdu = CoapUdpServerP__new_rst(ctx, node, 200);
        }
      break;
      case 2: 
        pdu = CoapUdpServerP__handle_post(ctx, node, data);
      if (!pdu && node->pdu->hdr->type == 0) {
        pdu = CoapUdpServerP__new_response(ctx, node, 160);
        }
#line 873
      break;
      case 4: 
        pdu = CoapUdpServerP__handle_delete(ctx, node, data);
      if (!pdu && node->pdu->hdr->type == 0) {
        pdu = CoapUdpServerP__new_response(ctx, node, 160);
        }
#line 878
      break;
      default: 
        if (node->pdu->hdr->type == 0) {
            if (node->pdu->hdr->code >= 40) {
              pdu = CoapUdpServerP__new_rst(ctx, node, 200);
              }
            else 
#line 883
              {
                pdu = CoapUdpServerP__new_rst(ctx, node, 165);
              }
          }
    }

  if (pdu && CoapUdpServerP__LibCoapServer__send(ctx, & node->remote, pdu, 1) == -1) {

      coap_delete_pdu(pdu);
    }
}

#line 141
static inline error_t CoapUdpServerP__Init__init(void )
#line 141
{

  CoapUdpServerP__ctx_server = (coap_context_t *)malloc(sizeof(coap_context_t ));
  if (!CoapUdpServerP__ctx_server) {
      return FAIL;
    }
  memset(CoapUdpServerP__ctx_server, 0, sizeof(coap_context_t ));
  coap_register_message_handler(CoapUdpServerP__ctx_server, CoapUdpServerP__message_handler);

  return SUCCESS;
}

# 67 "CoapBlipP.nc"
static inline error_t CoapBlipP__Init__init(void )
#line 67
{
  return SUCCESS;
}

# 15 "/opt/tinyos/tos/lib/net/blip/Ieee154AddressP.nc"
static inline error_t Ieee154AddressP__Init__init(void )
#line 15
{
  Ieee154AddressP__m_saddr = TOS_NODE_ID;
  Ieee154AddressP__m_panid = TOS_AM_GROUP;
  return SUCCESS;
}

# 48 "/opt/tinyos/tos/interfaces/LocalIeeeEui64.nc"
inline static ieee_eui64_t CC2420ControlP__LocalIeeeEui64__getId(void ){
#line 48
  struct ieee_eui64 __nesc_result;
#line 48

#line 48
  __nesc_result = DallasId48ToIeeeEui64C__LocalIeeeEui64__getId();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 93 "/opt/tinyos/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 93
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 95
  myGroup = ActiveMessageAddressC__group;
  return myGroup;
}

# 55 "/opt/tinyos/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void ){
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
# 106 "/opt/tinyos/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__amAddress(void )
#line 106
{
  am_addr_t myAddr;

  /* atomic removed: atomic calls only */
#line 108
  myAddr = ActiveMessageAddressC__addr;
  return myAddr;
}

#line 72
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 72
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/opt/tinyos/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void ){
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
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 46
}
#line 46
# 63 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 63
{
  /* atomic removed: atomic calls only */
#line 63
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 85 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 85
}
#line 85
# 54 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__HplGeneralIO__makeOutput();
}

# 46 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__makeOutput();
#line 46
}
#line 46
# 129 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 129
{
  int i;
#line 130
  int t;

#line 131
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();

  CC2420ControlP__m_short_addr = CC2420ControlP__ActiveMessageAddress__amAddress();
  CC2420ControlP__m_ext_addr = CC2420ControlP__LocalIeeeEui64__getId();
  CC2420ControlP__m_pan = CC2420ControlP__ActiveMessageAddress__amGroup();
  CC2420ControlP__m_tx_power = 31;
  CC2420ControlP__m_channel = 21;

  CC2420ControlP__m_ext_addr = CC2420ControlP__LocalIeeeEui64__getId();
  for (i = 0; i < 4; i++) {
      t = CC2420ControlP__m_ext_addr.data[i];
      CC2420ControlP__m_ext_addr.data[i] = CC2420ControlP__m_ext_addr.data[7 - i];
      CC2420ControlP__m_ext_addr.data[7 - i] = t;
    }





  CC2420ControlP__addressRecognition = TRUE;



  CC2420ControlP__hwAddressRecognition = TRUE;








  CC2420ControlP__autoAckEnabled = TRUE;



  CC2420ControlP__hwAutoAckDefault = TRUE;
  CC2420ControlP__hwAddressRecognition = TRUE;





  return SUCCESS;
}

# 81 "/opt/tinyos/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 5U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 55 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 55
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 193 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline error_t IPDispatchP__Init__init(void )
#line 193
{


  ip_malloc_init();
  return SUCCESS;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4428 {
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
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 44
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__10__GeneralIO__makeInput();
#line 44
}
#line 44


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 46
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__6__GeneralIO__makeOutput();
#line 46
}
#line 46
# 61 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 61
{
  /* atomic removed: atomic calls only */
#line 61
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 78 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__HplGeneralIO__makeInput(void ){
#line 78
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 78
}
#line 78
# 52 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__makeInput(void )
#line 52
{
#line 52
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__HplGeneralIO__makeInput();
}

# 44 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 44
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__5__GeneralIO__makeInput();
#line 44
}
#line 44
# 160 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 160
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 151 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 151
{
  CC2420ReceiveP__m_p_rx_buf = &CC2420ReceiveP__m_rx_buf;
  return SUCCESS;
}

# 57 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)
#line 57
{
#line 57
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4429 {
#line 57
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t f;
#line 57
    uint16_t t;
  } 
#line 57
  c = { .f = x };

#line 57
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(x);
}

#line 105
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl();
}

# 47 "/opt/tinyos/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare();
#line 47
}
#line 47
# 53 "/opt/tinyos/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 55 "/opt/tinyos/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 52 "/opt/tinyos/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP__Random__rand16(void ){
#line 52
  unsigned int __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC__Random__rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 62 "/opt/tinyos/tos/chips/cc2520/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Init__init(void )
#line 62
{
  UniqueSendP__localSendId = UniqueSendP__Random__rand16();
  return SUCCESS;
}

# 71 "/opt/tinyos/tos/chips/cc2520/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP__Init__init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP__receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP__receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 55 "/opt/tinyos/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 65 "/opt/tinyos/tos/system/PoolP.nc"
static inline error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 12; i++) {
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[i] = &/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool[i];
    }
  /*IPDispatchC.FragPool.PoolP*/PoolP__0__free = 12;
  /*IPDispatchC.FragPool.PoolP*/PoolP__0__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 12; i++) {
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[i] = &/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool[i];
    }
  /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free = 12;
  /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 3; i++) {
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[i] = &/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool[i];
    }
  /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free = 3;
  /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 3; i++) {
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__queue[i] = &/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool[i];
    }
  /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free = 3;
  /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 5; i++) {
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__queue[i] = &/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool[i];
    }
  /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free = 5;
  /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index = 0;
  return SUCCESS;
}

# 168 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__BlipStatistics__clear(void )
#line 168
{
}

#line 46
static inline error_t UdpP__Init__init(void )
#line 46
{
  UdpP__BlipStatistics__clear();
  memset((uint8_t *)UdpP__local_ports, 0, sizeof(uint16_t ) * UdpP__N_CLIENTS);
  return SUCCESS;
}

# 62 "/opt/tinyos/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = UdpP__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueReceiveP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueSendP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, RandomMlcgC__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, IPDispatchP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ControlP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, Ieee154AddressP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CoapBlipP__Init__init());
#line 62
  __nesc_result = ecombine(__nesc_result, CoapUdpServerP__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 109 "/opt/tinyos/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Off(void )
#line 109
{
  LedsP__Led2__set();
  ;
#line 111
  ;
}

# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )49U &= ~(0x01 << 6);
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

#line 94
static inline void LedsP__Leds__led1Off(void )
#line 94
{
  LedsP__Led1__set();
  ;
#line 96
  ;
}

# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )49U &= ~(0x01 << 5);
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

#line 79
static inline void LedsP__Leds__led0Off(void )
#line 79
{
  LedsP__Led0__set();
  ;
#line 81
  ;
}

# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__clr(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )49U &= ~(0x01 << 4);
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

#line 136
static inline void LedsP__Leds__set(uint8_t val)
#line 136
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 137
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
#line 156
    __nesc_atomic_end(__nesc_atomic); }
}

# 134 "/opt/tinyos/tos/interfaces/Leds.nc"
inline static void /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__Leds__set(uint8_t val){
#line 134
  LedsP__Leds__set(val);
#line 134
}
#line 134
# 68 "/opt/tinyos/tos/lib/net/coap/CoapLedResourceP.nc"
static inline int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__put(uint8_t *val, size_t buflen, coap_tid_t id)
#line 68
{
  if (*val < 8) {
      if (/*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock == FALSE) {
          /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock = TRUE;
          /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__temp_id = id;
          /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__Leds__set(*val);
          /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__postTask();
          return 300;
        }
      else 
#line 76
        {
          return 203;
        }
    }
  else 
#line 79
    {
      return 200;
    }
}

# 743 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline int CoapUdpServerP__WriteResource__default__put(uint8_t uri_key, uint8_t *val, size_t buflen, coap_tid_t id)
#line 743
{

  return FAIL;
}

# 34 "/opt/tinyos/tos/interfaces/WriteResource.nc"
inline static int CoapUdpServerP__WriteResource__put(uint8_t arg_0x415b4978, uint8_t *val, size_t buflen, coap_tid_t id){
#line 34
  int __nesc_result;
#line 34

#line 34
  switch (arg_0x415b4978) {
#line 34
    case KEY_LED:
#line 34
      __nesc_result = /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__WriteResource__put(val, buflen, id);
#line 34
      break;
#line 34
    default:
#line 34
      __nesc_result = CoapUdpServerP__WriteResource__default__put(arg_0x415b4978, val, buflen, id);
#line 34
      break;
#line 34
    }
#line 34

#line 34
  return __nesc_result;
#line 34
}
#line 34
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(/*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 51 "/opt/tinyos/tos/lib/net/coap/CoapLedResourceP.nc"
static inline int /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__get(coap_tid_t id)
#line 51
{
  if (/*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock == FALSE) {
      /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__lock = TRUE;

      /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__temp_id = id;
      /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__postTask();
      return 300;
    }
  else 
#line 58
    {
      return 203;
    }
}

# 532 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline int CoapUdpServerP__ReadResource__default__get(uint8_t uri_key, coap_tid_t id)
#line 532
{

  return FAIL;
}

# 33 "/opt/tinyos/tos/interfaces/ReadResource.nc"
inline static int CoapUdpServerP__ReadResource__get(uint8_t arg_0x415b4110, coap_tid_t id){
#line 33
  int __nesc_result;
#line 33

#line 33
  switch (arg_0x415b4110) {
#line 33
    case KEY_LED:
#line 33
      __nesc_result = /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__ReadResource__get(id);
#line 33
      break;
#line 33
    default:
#line 33
      __nesc_result = CoapUdpServerP__ReadResource__default__get(arg_0x415b4110, id);
#line 33
      break;
#line 33
    }
#line 33

#line 33
  return __nesc_result;
#line 33
}
#line 33
# 330 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline int CoapUdpServerP__resource_splitphase(coap_uri_t *uri, 
coap_tid_t *id, 
unsigned char *mediatype, 
unsigned int offset, 
unsigned char *buf, 
unsigned int *buflen, 
int *finished, 
unsigned int method)
#line 337
{

  if (method == 1) {
      return CoapUdpServerP__ReadResource__get(CoapUdpServerP__get_key(uri->path.s, uri->path.length), *id);
    }
  else {
#line 341
    if (method == 3) {
        return CoapUdpServerP__WriteResource__put(CoapUdpServerP__get_key(uri->path.s, uri->path.length), buf, *buflen, *id);
      }
    else 
#line 343
      {

        return 165;
      }
    }
}

#line 301
static inline error_t CoapUdpServerP__CoAPServer__registerResource(char uri[5], 
unsigned int uri_length, 
unsigned char mediatype, 
unsigned int writable, 
unsigned int splitphase, 
unsigned int immediately)
#line 306
{
  coap_resource_t *r;
  coap_key_t k;

#line 309
  if (!(r = malloc(sizeof(coap_resource_t )))) {
    return FAIL;
    }
  r->uri = coap_new_uri((const unsigned char *)uri, uri_length);
  r->mediatype = mediatype;
  r->dirty = 0;
  r->writable = writable;
  r->splitphase = splitphase;
  r->immediately = immediately;
  r->data = CoapUdpServerP__resource_splitphase;

  k = coap_add_resource(CoapUdpServerP__ctx_server, r);

  if (k == (coap_key_t )-1) {
      return FAIL;
    }
  return SUCCESS;
}

# 48 "/opt/tinyos/tos/interfaces/CoAPServer.nc"
inline static error_t CoapBlipP__CoAPServer__registerResource(char uri[5], unsigned int uri_length, unsigned char mediatype, unsigned int writable, unsigned int splitphase, unsigned int immediately){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = CoapUdpServerP__CoAPServer__registerResource(uri, uri_length, mediatype, writable, splitphase, immediately);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 160 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline int CoapUdpServerP__print_link(coap_resource_t *resource, unsigned char *buf, size_t buflen)
#line 160
{
  size_t n = 0;



  if (resource == (void *)0 || buf == (void *)0) {
      return -1;
    }

  if (buflen < resource->uri->path.length + 3) {
    return -1;
    }

  buf[n++] = '<';
#line 173
  buf[n++] = '/';

  memcpy(buf + n, resource->uri->path.s, resource->uri->path.length);

  n += resource->uri->path.length;
  buf[n++] = '>';

  if (resource->mediatype != 0xff) {
      if (buflen - n < 7) {
        return -1;
        }
#line 183
      n += snprintf((char *)(buf + n), buflen - n, ";ct=%d", 
      resource->mediatype);
    }

  if (resource->name) {
      if (buflen - n < resource->name->length + 5) {
        return -1;
        }
      memcpy(buf + n, ";n=\"", 4);
      n += 4;
      memcpy(buf + n, resource->name->s, resource->name->length);
      n += resource->name->length;

      if (! resource->writable) {
          if (buflen - n < 12) {
            return -1;
            }
          n += snprintf((char *)(buf + n), buflen - n, " (read-only)");
        }

      buf[n++] = '"';
    }

  return n;
}

static inline int CoapUdpServerP__resource_wellknown(coap_uri_t *uri, 
coap_tid_t *id, 
unsigned char *mediatype, 
unsigned int offset, 
unsigned char *buf, 
unsigned int *buflen, 
int *finished, 
unsigned int method)
#line 216
{

  static unsigned char resources[1000];
  size_t maxlen = 0;
  int n;
  coap_list_t *node;



  if (CoapUdpServerP__ctx_server == (void *)0) {
      return 200;
    }


  for (node = CoapUdpServerP__ctx_server->resources; node; node = node->next) {
      n = CoapUdpServerP__print_link((coap_resource_t *)node->data, resources + maxlen, 
      1000 - maxlen);
      if (n <= 0) {

          resources[maxlen] = '\0';
          break;
        }
      maxlen += n;

      if (node->next) {
        resources[maxlen++] = ',';
        }
      else {
#line 243
        resources[maxlen] = '\0';
        }
    }
  *finished = 1;

  switch (*mediatype) {
      case 0xff: 
        case 40: 
          *mediatype = 40;
      break;
      default: 
        *buflen = 0;
      return 175;
    }

  if (offset > maxlen) {
      *buflen = 0;
      return 160;
    }
  else {
#line 261
    if (offset + *buflen > maxlen) {
      *buflen = maxlen - offset;
      }
    }
#line 264
  memcpy(buf, resources + offset, *buflen);

  *finished = offset + *buflen == maxlen;
  return 80;
}



static inline error_t CoapUdpServerP__CoAPServer__registerWellknownCore(void )
#line 272
{

  coap_resource_t *r;

  if (!(r = malloc(sizeof(coap_resource_t )))) {
      return FAIL;
    }

  r->uri = coap_new_uri((const unsigned char *)".well-known/core", 
  sizeof ".well-known/core");
  r->mediatype = 40;
  r->dirty = 0;
  r->writable = 0;
  r->splitphase = 0;
  r->immediately = 1;
  r->data = CoapUdpServerP__resource_wellknown;
  coap_add_resource(CoapUdpServerP__ctx_server, r);

  return SUCCESS;
}

# 43 "/opt/tinyos/tos/interfaces/CoAPServer.nc"
inline static error_t CoapBlipP__CoAPServer__registerWellknownCore(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = CoapUdpServerP__CoAPServer__registerWellknownCore();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 52 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline error_t UdpP__UDP__bind(uint8_t clnt, uint16_t port)
#line 52
{
  int i;

#line 54
  port = (((uint16_t )port << 8) | ((uint16_t )port >> 8)) & 0xffff;
  if (port > 0) {
      for (i = 0; i < UdpP__N_CLIENTS; i++) 
        if (i != clnt && UdpP__local_ports[i] == port) {
          return FAIL;
          }
    }
#line 60
  UdpP__local_ports[clnt] = port;
  return SUCCESS;
}

# 12 "/opt/tinyos/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t LibCoapAdapterP__UDPServer__bind(uint16_t port){
#line 12
  unsigned char __nesc_result;
#line 12

#line 12
  __nesc_result = UdpP__UDP__bind(0U, port);
#line 12

#line 12
  return __nesc_result;
#line 12
}
#line 12
# 90 "/opt/tinyos/tos/lib/net/coap/LibCoapAdapterP.nc"
static inline error_t LibCoapAdapterP__LibCoapServer__bind(uint16_t port)
#line 90
{
  return LibCoapAdapterP__UDPServer__bind(port);
}

# 47 "/opt/tinyos/tos/interfaces/LibCoAP.nc"
inline static error_t CoapUdpServerP__LibCoapServer__bind(uint16_t port){
#line 47
  unsigned char __nesc_result;
#line 47

#line 47
  __nesc_result = LibCoapAdapterP__LibCoapServer__bind(port);
#line 47

#line 47
  return __nesc_result;
#line 47
}
#line 47
# 153 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static inline error_t CoapUdpServerP__CoAPServer__bind(uint16_t port)
#line 153
{
  return CoapUdpServerP__LibCoapServer__bind(port);
}

# 38 "/opt/tinyos/tos/interfaces/CoAPServer.nc"
inline static error_t CoapBlipP__CoAPServer__bind(uint16_t port){
#line 38
  unsigned char __nesc_result;
#line 38

#line 38
  __nesc_result = CoapUdpServerP__CoAPServer__bind(port);
#line 38

#line 38
  return __nesc_result;
#line 38
}
#line 38
# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t IPDispatchP__RadioControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = CC2420CsmaP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 158 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline error_t IPDispatchP__SplitControl__start(void )
#line 158
{
  return IPDispatchP__RadioControl__start();
}

# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t IPStackControlP__SubSplitControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = IPDispatchP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 12 "/opt/tinyos/tos/lib/net/blip/IPStackControlP.nc"
static inline error_t IPStackControlP__SplitControl__start(void )
#line 12
{
  return IPStackControlP__SubSplitControl__start();
}

# 104 "/opt/tinyos/tos/interfaces/SplitControl.nc"
inline static error_t CoapBlipP__RadioControl__start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = IPStackControlP__SplitControl__start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 71 "CoapBlipP.nc"
static inline void CoapBlipP__Boot__booted(void )
#line 71
{

  uint8_t i;

  CoapBlipP__RadioControl__start();
  ;
#line 76
  ;







  CoapBlipP__CoAPServer__bind(61616L);

  CoapBlipP__CoAPServer__registerWellknownCore();
  for (i = 0; i < 1; i++) {
      CoapBlipP__CoAPServer__registerResource(uri_key_map[i].uri, 
      uri_key_map[i].urilen - 1, 
      uri_key_map[i].mediatype, 
      uri_key_map[i].writable, 
      uri_key_map[i].splitphase, 
      uri_key_map[i].immediately);
    }
}

# 57 "/opt/tinyos/tos/lib/net/blip/dhcp/NoDhcpC.nc"
static inline void NoDhcpC__IPAddress__changed(bool valid)
#line 57
{
}

# 100 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__stop(void )
#line 100
{
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__m_running = FALSE;
  return SUCCESS;
}

# 158 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline error_t RPLRankP__StdControl__stop(void )
#line 158
{
  RPLRankP__m_running = FALSE;
  return SUCCESS;
}

# 105 "/opt/tinyos/tos/interfaces/StdControl.nc"
inline static error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RankControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = RPLRankP__StdControl__stop();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 510 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline error_t /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__stop(void )
#line 510
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running = FALSE;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RankControl__stop();
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__stop();
  return SUCCESS;
}

# 105 "/opt/tinyos/tos/interfaces/StdControl.nc"
inline static error_t IPStackControlP__RoutingControl__stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__stop();
#line 105
  __nesc_result = ecombine(__nesc_result, /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__stop());
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 41 "/opt/tinyos/tos/lib/net/blip/IPStackControlP.nc"
static inline void IPStackControlP__IPAddress__changed(bool valid)
#line 41
{
  if (valid) {
    IPStackControlP__RoutingControl__start();
    }
  else {
#line 45
    IPStackControlP__RoutingControl__stop();
    }
}

# 371 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static inline void IPForwardingEngineP__IPAddress__changed(bool global_valid)
#line 371
{
}

# 115 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
static inline void ICMPCoreP__IPAddress__changed(bool valid)
#line 115
{
}

# 133 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static inline void IPNeighborDiscoveryP__IPAddress__changed(bool global_valid)
#line 133
{
}

# 963 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static inline void RPLRankP__IPAddress__changed(bool global_valid)
#line 963
{
}

# 693 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static inline void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__changed(bool global_valid)
#line 693
{
}

# 429 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static inline void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__changed(bool global_valid)
#line 429
{
}

# 185 "/opt/tinyos/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__IPAddress__changed(bool global_valid)
#line 185
{
}

# 56 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void IPAddressP__IPAddress__changed(bool valid){
#line 56
  UdpP__IPAddress__changed(valid);
#line 56
  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__IPAddress__changed(valid);
#line 56
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__IPAddress__changed(valid);
#line 56
  RPLRankP__IPAddress__changed(valid);
#line 56
  IPNeighborDiscoveryP__IPAddress__changed(valid);
#line 56
  ICMPCoreP__IPAddress__changed(valid);
#line 56
  IPForwardingEngineP__IPAddress__changed(valid);
#line 56
  IPStackControlP__IPAddress__changed(valid);
#line 56
  NoDhcpC__IPAddress__changed(valid);
#line 56
}
#line 56
# 136 "/opt/tinyos/tos/lib/net/blip/IPAddressP.nc"
static inline error_t IPAddressP__IPAddress__setAddress(struct in6_addr *addr)
#line 136
{
  IPAddressP__m_addr = *addr;
#line 153
  IPAddressP__m_valid_addr = TRUE;
  IPAddressP__IPAddress__changed(TRUE);
  return SUCCESS;
}

# 52 "/opt/tinyos/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static error_t NoDhcpC__IPAddress__setAddress(struct in6_addr *addr){
#line 52
  unsigned char __nesc_result;
#line 52

#line 52
  __nesc_result = IPAddressP__IPAddress__setAddress(addr);
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 49 "/opt/tinyos/tos/lib/net/blip/dhcp/NoDhcpC.nc"
static inline void NoDhcpC__Boot__booted(void )
#line 49
{
  struct in6_addr addr;

#line 51
  memset(&addr, 0, sizeof addr);
  inet_pton6("fec0::", &addr);
  addr.in6_u.u6_addr16[7] = (((uint16_t )TOS_NODE_ID << 8) | ((uint16_t )TOS_NODE_ID >> 8)) & 0xffff;
  NoDhcpC__IPAddress__setAddress(&addr);
}

# 133 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__reconstruct_clear(void *ent)
#line 133
{
  struct lowpan_reconstruct *recon = (struct lowpan_reconstruct *)ent;

#line 135
  memset((uint8_t *)& recon->r_meta, 0, sizeof(struct ip6_metadata ));
  recon->r_timeout = T_UNUSED;
  recon->r_buf = (void *)0;
}

#line 658
static inline void IPDispatchP__BlipStatistics__clear(void )
#line 658
{
  memset((uint8_t *)&IPDispatchP__stats, 0, sizeof(ip_statistics_t ));
}

#line 200
static inline void IPDispatchP__Boot__booted(void )
#line 200
{
  IPDispatchP__BlipStatistics__clear();


  table_init(&IPDispatchP__recon_cache, IPDispatchP__recon_data, sizeof(struct lowpan_reconstruct ), N_RECONSTRUCTIONS);
  table_map(&IPDispatchP__recon_cache, IPDispatchP__reconstruct_clear);

  IPDispatchP__SplitControl__start();
}

# 60 "/opt/tinyos/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  IPDispatchP__Boot__booted();
#line 60
  NoDhcpC__Boot__booted();
#line 60
  CoapBlipP__Boot__booted();
#line 60
}
#line 60
# 45 "/opt/tinyos/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP__State__requestState(1U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 66 "/opt/tinyos/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__start(dt);
#line 66
}
#line 66
# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 56
{
#line 56
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    * (volatile uint8_t * )29U |= 0x01 << 5;
#line 56
    __nesc_atomic_end(__nesc_atomic); }
}

# 48 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 48
}
#line 48
# 48 "/opt/tinyos/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__set(void )
#line 48
{
#line 48
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__HplGeneralIO__set();
}

# 40 "/opt/tinyos/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 40
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__11__GeneralIO__set();
#line 40
}
#line 40
# 204 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 204
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 205
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 207
            FAIL;

            {
#line 207
              __nesc_atomic_end(__nesc_atomic); 
#line 207
              return __nesc_temp;
            }
          }
        }
#line 209
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 210
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
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
# 212 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 212
{
  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {

      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
      CC2420ReceiveP__beginReceive();
    }
  else 



    {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 68 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 68
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 68
}
#line 68
# 77 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 72
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 72
}
#line 72
# 103 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 72
  HplMsp430InterruptP__Port11__default__fired();
#line 72
}
#line 72
# 104 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 72
  HplMsp430InterruptP__Port12__default__fired();
#line 72
}
#line 72
# 105 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 72
  HplMsp430InterruptP__Port13__default__fired();
#line 72
}
#line 72
# 67 "/opt/tinyos/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__startDone_task__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__startDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 218 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void )
#line 218
{
  CC2420CsmaP__startDone_task__postTask();
}

# 76 "/opt/tinyos/tos/chips/cc2520/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  CC2420CsmaP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 61 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 441 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 441
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 68 "/opt/tinyos/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 68
  CC2420ControlP__InterruptCCA__fired();
#line 68
}
#line 68
# 77 "/opt/tinyos/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 77
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 72
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 72
}
#line 72
# 107 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 72
  HplMsp430InterruptP__Port15__default__fired();
#line 72
}
#line 72
# 108 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 72
  HplMsp430InterruptP__Port16__default__fired();
#line 72
}
#line 72
# 109 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 72
  HplMsp430InterruptP__Port17__default__fired();
#line 72
}
#line 72
# 206 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 72
  HplMsp430InterruptP__Port20__default__fired();
#line 72
}
#line 72
# 207 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 72
  HplMsp430InterruptP__Port21__default__fired();
#line 72
}
#line 72
# 208 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 72
  HplMsp430InterruptP__Port22__default__fired();
#line 72
}
#line 72
# 209 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 72
  HplMsp430InterruptP__Port23__default__fired();
#line 72
}
#line 72
# 210 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 72
  HplMsp430InterruptP__Port24__default__fired();
#line 72
}
#line 72
# 211 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 72
  HplMsp430InterruptP__Port25__default__fired();
#line 72
}
#line 72
# 212 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 72
  HplMsp430InterruptP__Port26__default__fired();
#line 72
}
#line 72
# 213 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 72 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 72
  HplMsp430InterruptP__Port27__default__fired();
#line 72
}
#line 72
# 98 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
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
# 349 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableRxIntr(void )
#line 349
{
  HplMsp430Usart0P__IE1 &= ~0x40;
}

# 177 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr(void ){
#line 177
  HplMsp430Usart0P__Usart__disableRxIntr();
#line 177
}
#line 177
# 232 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 232
{

  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos < /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len) {
    /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else 
#line 239
    {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__disableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone();
    }
}

# 65 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x40d6f0c0, uint8_t data){
#line 54
  switch (arg_0x40d6f0c0) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x40d6f0c0, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 90 "/opt/tinyos/tos/interfaces/ArbiterInfo.nc"
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
# 54 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
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

# 6 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2C.nc"
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
# 66 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0x40d6f940){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0x40d6f940);
#line 39
}
#line 39
# 59 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 39 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 250 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static inline void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone(void )
#line 250
{
}

# 64 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x40d6f0c0){
#line 49
  switch (arg_0x40d6f0c0) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x40d6f0c0);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/opt/tinyos/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
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
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4064e4b0){
#line 39
  switch (arg_0x4064e4b0) {
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
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4064e4b0);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 147 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

#line 107
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__Counter__get();
#line 109
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 109
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 0);
}

# 80 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__0__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__0__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__0__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 87
      if (/*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
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

# 788 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__congestionBackoff(void )
#line 788
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 789
    {
      CC2420TransmitP__RadioBackoff__requestCongestionBackoff(CC2420TransmitP__m_msg);
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myCongestionBackoff);
    }
#line 792
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/opt/tinyos/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 69
{
  uint32_t mlcg;
#line 70
  uint32_t p;
#line 70
  uint32_t q;
  uint64_t tmpseed;

#line 72
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
#line 84
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

# 795 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__acquireSpiResource(void )
#line 795
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();

#line 797
  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 126 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP__SpiResource__isOwner()) {
          CC2420SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            CC2420SpiP__m_holder = id;
          }
        else {
            CC2420SpiP__WorkingState__toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/opt/tinyos/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
          StateImplP__state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 177 "/opt/tinyos/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 177
{
  /* atomic removed: atomic calls only */
#line 178
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 179
        TRUE;

#line 179
        return __nesc_temp;
      }
    else 
#line 180
      {
        unsigned char __nesc_temp = 
#line 180
        FALSE;

#line 180
        return __nesc_temp;
      }
  }
}

#line 133
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 133
{
  /* atomic removed: atomic calls only */
#line 134
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 138
              SUCCESS;

#line 138
              return __nesc_temp;
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

#line 143
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 147
  return FAIL;
}

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

# 265 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 107 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
          CC2420SpiP__m_holder = id;
          if (CC2420SpiP__SpiResource__isOwner()) {
              CC2420SpiP__grant__postTask();
            }
          else {
              CC2420SpiP__SpiResource__request();
            }
        }
      else {
          CC2420SpiP__m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 743 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 743
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 747
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          CC2420TransmitP__SFLUSHTX__strobe();
          CC2420TransmitP__releaseSpiResource();
          CC2420TransmitP__CSN__set();
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
          CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
          {
#line 754
            __nesc_atomic_end(__nesc_atomic); 
#line 754
            return;
          }
        }





      CC2420TransmitP__CSN__clr();
      status = CC2420TransmitP__m_cca ? CC2420TransmitP__STXONCCA__strobe() : CC2420TransmitP__STXON__strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP__SNOP__strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP__m_state = congestion ? CC2420TransmitP__S_SAMPLE_CCA : CC2420TransmitP__S_SFD;
      CC2420TransmitP__CSN__set();
    }
#line 773
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP__totalCcaChecks = 0;
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__congestionBackoff();
    }
  else 
#line 779
    {
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__CC2420_ABORT_PERIOD);
    }
}

# 318 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 133 "/opt/tinyos/tos/system/StateImplP.nc"
static bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP__state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 134 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static uint8_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiByte__write(uint8_t tx)
#line 134
{
  uint8_t byte;


  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__clrRxIntr();
  byte = /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__rx();

  return byte;
}

# 149 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 247 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

# 56 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 850 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__signalDone(error_t err)
#line 850
{
  /* atomic removed: atomic calls only */
#line 851
  CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
  CC2420TransmitP__abortSpiRelease = FALSE;
  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, err);
}

# 49 "/opt/tinyos/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 49
{
  /* atomic removed: atomic calls only */
#line 50
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 57 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
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

# 260 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

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
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 171 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.nxdata, FALSE);
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.nxdata, CC2420_INVALID_TIMESTAMP);
}

# 107 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Counter__get();
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type expires;
#line 109
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 132
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__from_size_type )remaining << 5);
}

# 80 "/opt/tinyos/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__1__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__1__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__1__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 87
      if (/*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 101
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
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
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x40599108){
#line 75
  switch (arg_0x40599108) {
#line 75
    case IPForwardingEngineP__defaultRouteAddedTask:
#line 75
      IPForwardingEngineP__defaultRouteAddedTask__runTask();
#line 75
      break;
#line 75
    case CC2420ControlP__sync:
#line 75
      CC2420ControlP__sync__runTask();
#line 75
      break;
#line 75
    case CC2420ControlP__syncDone:
#line 75
      CC2420ControlP__syncDone__runTask();
#line 75
      break;
#line 75
    case CC2420SpiP__grant:
#line 75
      CC2420SpiP__grant__runTask();
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
    case IPDispatchP__sendTask:
#line 75
      IPDispatchP__sendTask__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__startDone_task:
#line 75
      CC2420CsmaP__startDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__stopDone_task:
#line 75
      CC2420CsmaP__stopDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP__sendDone_task:
#line 75
      CC2420CsmaP__sendDone_task__runTask();
#line 75
      break;
#line 75
    case CC2420ReceiveP__receiveDone_task:
#line 75
      CC2420ReceiveP__receiveDone_task__runTask();
#line 75
      break;
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
    case CC2420TinyosNetworkP__grantTask:
#line 75
      CC2420TinyosNetworkP__grantTask__runTask();
#line 75
      break;
#line 75
    case PacketLinkP__send:
#line 75
      PacketLinkP__send__runTask();
#line 75
      break;
#line 75
    case RPLRankP__newParentSearch:
#line 75
      RPLRankP__newParentSearch__runTask();
#line 75
      break;
#line 75
    case /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask:
#line 75
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDIOTask__runTask();
#line 75
      break;
#line 75
    case /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask:
#line 75
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__sendDISTask__runTask();
#line 75
      break;
#line 75
    case /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init:
#line 75
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__init__runTask();
#line 75
      break;
#line 75
    case /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO:
#line 75
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__initDIO__runTask();
#line 75
      break;
#line 75
    case /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining:
#line 75
      /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__computeRemaining__runTask();
#line 75
      break;
#line 75
    case /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO:
#line 75
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__sendDAO__runTask();
#line 75
      break;
#line 75
    case /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO:
#line 75
      /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__initDAO__runTask();
#line 75
      break;
#line 75
    case /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed:
#line 75
      /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__getLed__runTask();
#line 75
      break;
#line 75
    case /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone:
#line 75
      /*CoapBlipC.CoapLedResource.CoapLedResourceP*/CoapLedResourceP__0__setLedDone__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x40599108);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 373 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static coap_pdu_t *CoapUdpServerP__new_response(coap_context_t *ctx, coap_queue_t *node, 
unsigned int code)
#line 374
{
  coap_pdu_t *pdu;

  {
#line 377
    pdu = coap_new_pdu();
#line 377
    if (pdu) {
#line 377
        coap_opt_t *tok;

#line 377
        pdu->hdr->type = 2;
#line 377
        pdu->hdr->code = code;
#line 377
        pdu->hdr->id = node->pdu->hdr->id;
#line 377
        tok = coap_check_option(node->pdu, 11);
#line 377
        if (tok && 1) {
#line 377
          coap_add_option(pdu, 11, tok->lval.flag == 15 ? tok->lval.length + 15 : tok->sval.length, (unsigned char *)& *tok + (tok->lval.flag == 15 ? 2 : 1));
          }
      }
  }
#line 377
  ;

  return pdu;
}

#line 121
static unsigned short CoapUdpServerP__get_new_tid(void )
#line 121
{
  if (!CoapUdpServerP__tid) {
    CoapUdpServerP__tid = CoapUdpServerP__Random__rand16();
    }
#line 124
  CoapUdpServerP__tid++;
  return (((uint16_t )CoapUdpServerP__tid >> 8) | ((uint16_t )CoapUdpServerP__tid << 8)) & 0xffff;
}

#line 382
static coap_pdu_t *CoapUdpServerP__new_asynresponse(coap_context_t *ctx, coap_queue_t *node)
#line 382
{
  coap_pdu_t *pdu;

  {
#line 385
    pdu = coap_new_pdu();
#line 385
    if (pdu) {
#line 385
        coap_opt_t *tok;

#line 385
        pdu->hdr->type = 0;
#line 385
        pdu->hdr->code = 80;
#line 385
        pdu->hdr->id = node->pdu->hdr->id;
#line 385
        tok = coap_check_option(node->pdu, 11);
#line 385
        if (tok && 1) {
#line 385
          coap_add_option(pdu, 11, tok->lval.flag == 15 ? tok->lval.length + 15 : tok->sval.length, (unsigned char *)& *tok + (tok->lval.flag == 15 ? 2 : 1));
          }
      }
  }
#line 385
  ;

  return pdu;
}

# 63 "/opt/tinyos/tos/lib/net/coap/LibCoapAdapterP.nc"
  coap_tid_t coap_send_impl(coap_context_t *context, 
struct sockaddr_in6 *dst, 
coap_pdu_t *pdu, 
int free_pdu)
#line 66
{
  coap_tid_t tid;

  if ((!context || !dst) || !pdu) {
    return -1;
    }
  LibCoapAdapterP__UDPServer__sendto(dst, pdu->hdr, pdu->length);

  tid = pdu->hdr->id;

  if (free_pdu) {
    coap_delete_pdu(pdu);
    }
  return (((uint16_t )tid >> 8) | ((uint16_t )tid << 8)) & 0xffff;
}

# 65 "/opt/tinyos/tos/lib/net/blip/IPAddressP.nc"
static bool IPAddressP__IPAddress__setSource(struct ip6_hdr *hdr)
#line 65
{
  enum __nesc_unnamed4430 {
#line 66
    LOCAL, GLOBAL
  } 
#line 66
  type = GLOBAL;

  if (hdr->ip6_dst.in6_u.u6_addr8[0] == 0xff) {

      if ((hdr->ip6_dst.in6_u.u6_addr8[1] & 0x0f) <= 0x2) {
          type = LOCAL;
        }
    }
  else {
#line 73
    if (hdr->ip6_dst.in6_u.u6_addr8[0] == 0xfe) {

        if ((hdr->ip6_dst.in6_u.u6_addr8[1] & 0xf0) <= 0x80) {
            type = LOCAL;
          }
      }
    }
  if (type == LOCAL) {
      return IPAddressP__IPAddress__getLLAddr(& hdr->ip6_src);
    }
  else 
#line 82
    {
      return IPAddressP__IPAddress__getGlobalAddr(& hdr->ip6_src);
    }
}

#line 37
static bool IPAddressP__IPAddress__getLLAddr(struct in6_addr *addr)
#line 37
{
  ieee154_panid_t panid = IPAddressP__Ieee154Address__getPanId();
  ieee154_saddr_t saddr = IPAddressP__Ieee154Address__getShortAddr();
  ieee154_laddr_t laddr = IPAddressP__Ieee154Address__getExtAddr();

  memset(addr->in6_u.u6_addr8, 0, 16);
  addr->in6_u.u6_addr16[0] = (((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff;
  if (IPAddressP__m_short_addr) {
      addr->in6_u.u6_addr16[4] = (((uint16_t )panid << 8) | ((uint16_t )panid >> 8)) & 0xffff;
      addr->in6_u.u6_addr16[5] = (((uint16_t )0x00FF << 8) | ((uint16_t )0x00FF >> 8)) & 0xffff;
      addr->in6_u.u6_addr16[6] = (((uint16_t )0xFE00 << 8) | ((uint16_t )0xFE00 >> 8)) & 0xffff;
      addr->in6_u.u6_addr16[7] = (((uint16_t )saddr << 8) | ((uint16_t )saddr >> 8)) & 0xffff;
      addr->in6_u.u6_addr8[8] &= ~0x2;
    }
  else 
#line 50
    {
      int i;

#line 52
      for (i = 0; i < 8; i++) 
        addr->in6_u.u6_addr8[8 + i] = laddr.data[7 - i];
      addr->in6_u.u6_addr8[8] ^= 0x2;
    }

  return TRUE;
}

# 27 "/opt/tinyos/tos/lib/net/blip/Ieee154AddressP.nc"
static ieee154_laddr_t Ieee154AddressP__Ieee154Address__getExtAddr(void )
#line 27
{
  ieee154_laddr_t addr = Ieee154AddressP__LocalIeeeEui64__getId();
  int i;
  uint8_t tmp;


  for (i = 0; i < 4; i++) {
      tmp = addr.data[i];
      addr.data[i] = addr.data[7 - i];
      addr.data[7 - i] = tmp;
    }
  return addr;
}

# 8 "/opt/tinyos/tos/platforms/epic/chips/ds2411/DallasId48ToIeeeEui64C.nc"
static ieee_eui64_t DallasId48ToIeeeEui64C__LocalIeeeEui64__getId(void )
#line 8
{
  uint8_t id[6];
  ieee_eui64_t eui;

#line 11
  if (DallasId48ToIeeeEui64C__ReadId48__read(id) != SUCCESS) {
      memset(eui.data, 0, 8);
      goto done;
    }

  eui.data[0] = IEEE_EUI64_COMPANY_ID_0;
  eui.data[1] = IEEE_EUI64_COMPANY_ID_1;
  eui.data[2] = IEEE_EUI64_COMPANY_ID_2;



  eui.data[3] = IEEE_EUI64_SERIAL_ID_0;
  eui.data[4] = IEEE_EUI64_SERIAL_ID_1;


  eui.data[5] = id[2];
  eui.data[6] = id[1];
  eui.data[7] = id[0];

  done: 
    return eui;
}

# 63 "/opt/tinyos/tos/lib/timer/BusyWaitCounterC.nc"
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

# 48 "/opt/tinyos/tos/lib/net/blip/IPProtocolsP.nc"
static error_t IPProtocolsP__IP__send(uint8_t nxt_hdr, struct ip6_packet *msg)
#line 48
{
  msg->ip6_hdr.ip6_ctlun.ip6_un2_vfc = 0x60;
  msg->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_hlim = 16;
  ;
#line 51
  ;

  return IPProtocolsP__SubIP__send(msg);
}

# 154 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static struct route_entry *IPForwardingEngineP__ForwardingTable__lookupRoute(const uint8_t *prefix, 
int prefix_len_bits)
#line 155
{
  int i;

#line 157
  for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (IPForwardingEngineP__routing_table[i].valid && (
      IPForwardingEngineP__routing_table[i].prefixlen == 0 || (
      memcmp(prefix, IPForwardingEngineP__routing_table[i].prefix.in6_u.u6_addr8, (
      prefix_len_bits < IPForwardingEngineP__routing_table[i].prefixlen ? prefix_len_bits : IPForwardingEngineP__routing_table[i].prefixlen) / 8) == 0 && 
      prefix_len_bits))) {

          return &IPForwardingEngineP__routing_table[i];
        }
    }
  return (void *)0;
}

# 87 "/opt/tinyos/tos/lib/net/blip/IPAddressP.nc"
static bool IPAddressP__IPAddress__isLocalAddress(struct in6_addr *addr)
#line 87
{
  ieee154_panid_t panid = IPAddressP__Ieee154Address__getPanId();
  ieee154_saddr_t saddr = IPAddressP__Ieee154Address__getShortAddr();
  ieee154_laddr_t eui = IPAddressP__Ieee154Address__getExtAddr();

  if (addr->in6_u.u6_addr16[0] == ((((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff)) {


      if (
#line 94
      IPAddressP__m_short_addr && 
      addr->in6_u.u6_addr16[5] == ((((uint16_t )0x00FF << 8) | ((uint16_t )0x00FF >> 8)) & 0xffff) && 
      addr->in6_u.u6_addr16[6] == ((((uint16_t )0xFE00 << 8) | ((uint16_t )0xFE00 >> 8)) & 0xffff)) {
          if (((((uint16_t )addr->in6_u.u6_addr16[4] >> 8) | ((uint16_t )addr->in6_u.u6_addr16[4] << 8)) & 0xffff) == (panid & ~0x200) && (((
          (uint16_t )addr->in6_u.u6_addr16[7] >> 8) | ((uint16_t )addr->in6_u.u6_addr16[7] << 8)) & 0xffff) == saddr) {
              return TRUE;
            }
          else 
#line 100
            {
              return FALSE;
            }
        }

      return addr->in6_u.u6_addr8[8] == (eui.data[7] ^ 0x2) && 
      addr->in6_u.u6_addr8[9] == eui.data[6] && 
      addr->in6_u.u6_addr8[10] == eui.data[5] && 
      addr->in6_u.u6_addr8[11] == eui.data[4] && 
      addr->in6_u.u6_addr8[12] == eui.data[3] && 
      addr->in6_u.u6_addr8[13] == eui.data[2] && 
      addr->in6_u.u6_addr8[14] == eui.data[1] && 
      addr->in6_u.u6_addr8[15] == eui.data[0];
    }
  else {
#line 114
    if (addr->in6_u.u6_addr8[0] == 0xff) {

        if ((addr->in6_u.u6_addr8[1] & 0x0f) <= 2) {

            return TRUE;
          }
      }
    else {
#line 120
      if (memcmp(addr->in6_u.u6_addr8, IPAddressP__m_addr.in6_u.u6_addr8, 16) == 0) {
          return TRUE;
        }
      }
    }
#line 123
  return FALSE;
}

# 184 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static error_t IPForwardingEngineP__do_send(uint8_t ifindex, struct in6_addr *next, struct ip6_packet *pkt)
#line 184
{
  error_t rc;
  struct in6_iid *iid = IPForwardingEngineP__Pool__get();

#line 187
  if (iid != (void *)0) {
    memcpy(iid->data, &next->in6_u.u6_addr8[8], 8);
    }
#line 189
  rc = IPForwardingEngineP__IPForward__send(ifindex, next, pkt, iid);
  if (rc != SUCCESS && iid != (void *)0) {
    IPForwardingEngineP__Pool__put(iid);
    }
#line 192
  return rc;
}

# 95 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static error_t IPNeighborDiscoveryP__IPForward__send(struct in6_addr *next, struct ip6_packet *msg, void *ptr)
#line 95
{
  struct ieee154_frame_addr fr_addr;
  struct in6_addr local_addr;

#line 98
  fr_addr.ieee_dstpan = IPNeighborDiscoveryP__Ieee154Address__getPanId();
  IPNeighborDiscoveryP__IPAddress__getLLAddr(&local_addr);

  ;
#line 101
  ;
  ;
#line 102
  ;
  ;
#line 103
  ;
  ;
#line 104
  ;
  ;
#line 105
  ;


  if (IPNeighborDiscoveryP__NeighborDiscovery__resolveAddress(&local_addr, & fr_addr.ieee_src) != SUCCESS) {
      ;
#line 109
      ;
      return FAIL;
    }

  if (IPNeighborDiscoveryP__NeighborDiscovery__resolveAddress(next, & fr_addr.ieee_dst) != SUCCESS) {
      ;
#line 114
      ;
      return FAIL;
    }
  ;
#line 117
  ;
#line 117
  ;
#line 117
  ;
  ;
#line 118
  ;
#line 118
  ;
#line 118
  ;
  ;
#line 119
  ;

  return IPNeighborDiscoveryP__IPLower__send(&fr_addr, msg, ptr);
}

#line 58
static error_t IPNeighborDiscoveryP__NeighborDiscovery__resolveAddress(struct in6_addr *addr, 
ieee154_addr_t *link_addr)
#line 59
{
  ieee154_panid_t panid = IPNeighborDiscoveryP__Ieee154Address__getPanId();

  if (addr->in6_u.u6_addr16[0] == ((((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff)) {
      if (addr->in6_u.u6_addr16[5] == ((((uint16_t )0x00FF << 8) | ((uint16_t )0x00FF >> 8)) & 0xffff) && 
      addr->in6_u.u6_addr16[6] == ((((uint16_t )0xFE00 << 8) | ((uint16_t )0xFE00 >> 8)) & 0xffff)) {

          if (((((uint16_t )addr->in6_u.u6_addr16[4] >> 8) | ((uint16_t )addr->in6_u.u6_addr16[4] << 8)) & 0xffff) == (panid & ~0x0200)) {
              link_addr->ieee_mode = IEEE154_ADDR_SHORT;
              link_addr->ieee_addr.saddr = (((uint16_t )addr->in6_u.u6_addr16[7] >> 8) | ((uint16_t )addr->in6_u.u6_addr16[7] << 8)) & 0xffff;
            }
          else 
#line 69
            {
              return FAIL;
            }
        }
      else 
#line 72
        {
          int i;

#line 74
          link_addr->ieee_mode = IEEE154_ADDR_EXT;
          for (i = 0; i < 8; i++) 
            link_addr->ieee_addr.laddr.data[i] = addr->in6_u.u6_addr8[15 - i];
          link_addr->ieee_addr.laddr.data[7] ^= 0x2;
        }
      return SUCCESS;
    }
  else {
#line 80
    if (addr->in6_u.u6_addr8[0] == 0xff) {

        if ((addr->in6_u.u6_addr8[1] & 0x0f) == 0x02) {
            link_addr->ieee_mode = IEEE154_ADDR_SHORT;
            link_addr->ieee_addr.saddr = IEEE154_BROADCAST_ADDR;
            return SUCCESS;
          }
      }
    }
  return FAIL;
}

# 103 "/opt/tinyos/tos/system/PoolP.nc"
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t *newVal)
#line 103
{
  if (/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free >= 12) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index + /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free;

#line 109
      if (emptyIndex >= 12) {
          emptyIndex -= 12;
        }
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[emptyIndex] = newVal;
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free++;
      ;
      return SUCCESS;
    }
}

#line 103
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t *newVal)
#line 103
{
  if (/*IPDispatchC.FragPool.PoolP*/PoolP__0__free >= 12) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPDispatchC.FragPool.PoolP*/PoolP__0__index + /*IPDispatchC.FragPool.PoolP*/PoolP__0__free;

#line 109
      if (emptyIndex >= 12) {
          emptyIndex -= 12;
        }
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[emptyIndex] = newVal;
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__free++;
      ;
      return SUCCESS;
    }
}

# 16 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/ieee154_header.c"
static uint8_t *IPDispatchP__pack_ieee154_header(uint8_t *buf, size_t cnt, 
struct ieee154_frame_addr *frame)
#line 17
{
  uint8_t *ieee_hdr = buf;
  uint16_t fcf;







  buf = buf + IEEE154_MIN_HDR_SZ;
  if (frame->ieee_dst.ieee_mode == IEEE154_ADDR_SHORT) {
#line 28
      uint16_t tmpval = frame->ieee_dst.ieee_addr.saddr;

#line 28
      memcpy(buf, &tmpval, 2);
#line 28
      buf += 2;
    }
  else 
#line 28
    {
#line 28
      memcpy(buf, & frame->ieee_dst.ieee_addr.laddr, 8);
#line 28
      buf += 8;
    }
#line 28
  ;
  if (frame->ieee_src.ieee_mode == IEEE154_ADDR_SHORT) {
#line 29
      uint16_t tmpval = frame->ieee_src.ieee_addr.saddr;

#line 29
      memcpy(buf, &tmpval, 2);
#line 29
      buf += 2;
    }
  else 
#line 29
    {
#line 29
      memcpy(buf, & frame->ieee_src.ieee_addr.laddr, 8);
#line 29
      buf += 8;
    }
#line 29
  ;

  fcf = IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE;
  fcf |= frame->ieee_src.ieee_mode << IEEE154_FCF_SRC_ADDR_MODE;
  fcf |= frame->ieee_dst.ieee_mode << IEEE154_FCF_DEST_ADDR_MODE;
  fcf |= 1 << IEEE154_FCF_INTRAPAN;

  ieee_hdr[1] = fcf & 0xff;
  ieee_hdr[2] = fcf >> 8;
  ieee_hdr[4] = frame->ieee_dstpan & 0xff;
  ieee_hdr[5] = frame->ieee_dstpan >> 8;

  return buf;
}

# 32 "/opt/tinyos/tos/lib/net/blip/IPNeighborDiscoveryP.nc"
static int IPNeighborDiscoveryP__NeighborDiscovery__matchContext(struct in6_addr *addr, 
uint8_t *ctx)
#line 33
{
  struct in6_addr me;

#line 35
  if (!IPNeighborDiscoveryP__IPAddress__getGlobalAddr(&me)) {
#line 35
    return 0;
    }
#line 36
  if (memcmp(me.in6_u.u6_addr8, addr->in6_u.u6_addr8, 8) == 0) {
      *ctx = 0;
      return 64;
    }
  else 
#line 39
    {
      return 0;
    }
}

# 169 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static uint8_t *IPDispatchP__pack_address(uint8_t *buf, struct in6_addr *addr, int context_match_len, 
ieee154_addr_t *l2addr, ieee154_panid_t pan, uint8_t *flags)
#line 170
{
  *flags = 0;
  if (addr->in6_u.u6_addr16[0] == ((((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff) && addr->in6_u.u6_addr16[1] == 0 && addr->in6_u.u6_addr16[2] == 0 && addr->in6_u.u6_addr16[3] == 0) {



      if (
#line 175
      addr->in6_u.u6_addr16[4] == 0 && 
      addr->in6_u.u6_addr16[5] == 0 && 
      addr->in6_u.u6_addr16[6] == 0) {

          *flags |= LOWPAN_IPHC_AM_16;
          memcpy(buf, &addr->in6_u.u6_addr8[14], 2);
          return buf += 2;
        }
      else {



        if ((
#line 183
        addr->in6_u.u6_addr16[4] == ((((uint16_t )(pan & ~0x0200) << 8) | ((uint16_t )(pan & ~0x0200) >> 8)) & 0xffff) && 
        addr->in6_u.u6_addr16[5] == ((((uint16_t )0x00ff << 8) | ((uint16_t )0x00ff >> 8)) & 0xffff) && 
        addr->in6_u.u6_addr16[6] == ((((uint16_t )0xfe00 << 8) | ((uint16_t )0xfe00 >> 8)) & 0xffff) && (
        l2addr->ieee_mode == IEEE154_ADDR_SHORT && 
        addr->in6_u.u6_addr16[7] == ((((uint16_t )l2addr->ieee_addr.saddr << 8) | ((uint16_t )l2addr->ieee_addr.saddr >> 8)) & 0xffff))) || (

        l2addr->ieee_mode == IEEE154_ADDR_EXT && 
        IPDispatchP__iid_eui_cmp(&addr->in6_u.u6_addr8[8], l2addr->ieee_addr.laddr.data))) {

            *flags |= LOWPAN_IPHC_AM_0;
            return buf;
          }
        else 
#line 194
          {
            *flags |= LOWPAN_IPHC_AM_64;
            memcpy(buf, &addr->in6_u.u6_addr8[8], 8);
            return buf + 8;
          }
        }
    }
  else {
#line 199
    if (context_match_len > 0) {
        int extra = 0;

        *flags |= LOWPAN_IPHC_AC_CONTEXT;
        if (context_match_len == 128) {
            *flags |= LOWPAN_IPHC_AM_0;
          }
        else {
#line 205
          if (IPDispatchP__bit_range_zero_p(&addr->in6_u.u6_addr8[0], context_match_len, 112) == 0) {
              *flags |= LOWPAN_IPHC_AM_16;
              memcpy(buf, &addr->in6_u.u6_addr8[14], 2);
              extra = 2;
            }
          else {
#line 209
            if (IPDispatchP__bit_range_zero_p(&addr->in6_u.u6_addr8[0], context_match_len, 64) == 0) {
                *flags |= LOWPAN_IPHC_AM_64;
                memcpy(buf, &addr->in6_u.u6_addr8[8], 8);
                extra = 8;
              }
            else 
#line 213
              {
                *flags |= LOWPAN_IPHC_AM_128;
                *flags &= ~LOWPAN_IPHC_AC_CONTEXT;
                memcpy(buf, &addr->in6_u.u6_addr8[0], 16);
                extra = 16;
              }
            }
          }
#line 219
        return buf + extra;
      }
    else {
#line 220
      if (addr->in6_u.u6_addr16[0] == 0 && addr->in6_u.u6_addr16[1] == 0 && addr->in6_u.u6_addr16[2] == 0 && addr->in6_u.u6_addr16[3] == 0 && addr->in6_u.u6_addr16[4] == 0 && addr->in6_u.u6_addr16[5] == 0 && addr->in6_u.u6_addr16[6] == 0 && addr->in6_u.u6_addr16[7] == 0) {

          *flags |= LOWPAN_IPHC_AC_CONTEXT | LOWPAN_IPHC_AM_128;
          return buf;
        }
      else 
#line 224
        {

          *flags |= LOWPAN_IPHC_AM_128;
          memcpy(buf, addr->in6_u.u6_addr8, 16);
          return buf + 16;
        }
      }
    }
}

#line 76
static int IPDispatchP__bit_range_zero_p(uint8_t *buf, int start, int end)
#line 76
{
  int start_byte = start / 8;
  int end_byte = end / 8;
  int i;
  uint8_t start_mask = 0xff << (8 - start % 8);
  uint8_t end_mask = 0xff << (8 - end % 8);



  if ((buf[start_byte] & start_mask) != 0) {
      return -1;
    }
  if ((buf[end_byte] & end_mask) != 0) {
      return -1;
    }
  for (i = start_byte; i < end_byte; i++) {
      if (buf[i] != 0) {
#line 92
        return -1;
        }
    }
#line 94
  return 0;
}

# 111 "/opt/tinyos/tos/chips/cc2520/lowpan/CC2420TinyosNetworkP.nc"
static void CC2420TinyosNetworkP__BarePacket__setPayloadLength(message_t *msg, uint8_t len)
#line 111
{
  cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 113
  __nesc_hton_leuint8(hdr->length.nxdata, len - 1 + MAC_FOOTER_SIZE);
}

# 152 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static void IPDispatchP__SENDINFO_DECR(struct send_info *si)
#line 152
{
  if (-- si->_refcount == 0) {
      IPDispatchP__SendInfoPool__put(si);
    }
}

# 103 "/opt/tinyos/tos/system/PoolP.nc"
static error_t /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__Pool__put(/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__pool_t *newVal)
#line 103
{
  if (/*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free >= 3) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__index + /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free;

#line 109
      if (emptyIndex >= 3) {
          emptyIndex -= 3;
        }
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__queue[emptyIndex] = newVal;
      /*IPStackC.FwdAddrPoolC.PoolP*/PoolP__3__free++;
      ;
      return SUCCESS;
    }
}

# 74 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static 
#line 73
int 
CoapUdpServerP__coap_extract_node(coap_queue_t **queue, coap_queue_t *node)
{
  coap_queue_t *q;

#line 77
  if (!queue) {
    return 0;
    }
  q = *queue;
  if (q == node) 
    {
      *queue = node->next;
      return 1;
    }

  for (; q; q = q->next) 
    {
      if (q->next == node) 
        {
          q->next = node->next;
          return 1;
        }
    }
  return 0;
}

# 189 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static uint16_t RPLRankP__RPLRankInfo__getRank(struct in6_addr *node)
#line 189
{
  uint8_t indexset;
  struct in6_addr my_addr;


  RPLRankP__IPAddress__getGlobalAddr(&my_addr);




  if (!memcmp(&my_addr, node, sizeof(struct in6_addr ))) {

      if (RPLRankP__ROOT) {
          RPLRankP__nodeRank = ROOT_RANK;
        }
      return RPLRankP__nodeRank;
    }

  indexset = RPLRankP__getParent(node);

  if (indexset != 20) {
      return RPLRankP__parentSet[indexset].rank;
    }

  return 0x1234;
}

#line 238
static uint8_t RPLRankP__getParent(struct in6_addr *node)
#line 238
{
  uint8_t indexset;

#line 240
  if (RPLRankP__parentNum == 0) {
      return 20;
    }
  for (indexset = 0; indexset < 20; indexset++) {
      if (!memcmp(& RPLRankP__parentSet[indexset].parentIP, node, sizeof(struct in6_addr )) && 
      RPLRankP__parentSet[indexset].valid) {
          return indexset;
        }
    }
  return 20;
}

#line 216
static error_t RPLRankP__RPLRankInfo__getDefaultRoute(struct in6_addr *next)
#line 216
{


  if (RPLRankP__parentNum) {
      ip_memcpy((uint8_t *)next, 
      (uint8_t *)RPLRankP__RPLOF__getParent(), 
      sizeof(struct in6_addr ));
      return SUCCESS;
    }
  return FAIL;
}

# 87 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static struct in6_addr *RPLOF0P__RPLOF__getParent(void )
#line 87
{
  parent_t *parentNode = RPLOF0P__ParentTable__get(RPLOF0P__desiredParent);

#line 89
  return & parentNode->parentIP;
}

# 103 "/opt/tinyos/tos/system/PoolP.nc"
static error_t /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__put(/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t *newVal)
#line 103
{
  if (/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free >= 5) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index + /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free;

#line 109
      if (emptyIndex >= 5) {
          emptyIndex -= 5;
        }
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__queue[emptyIndex] = newVal;
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free++;
      ;
      return SUCCESS;
    }
}

# 105 "/opt/tinyos/tos/lib/net/blip/icmp/ICMPCoreP.nc"
static error_t ICMPCoreP__ICMP_IP__send(uint8_t type, struct ip6_packet *pkt)
#line 105
{
  struct icmp6_hdr *req = (struct icmp6_hdr *)pkt->ip6_data->iov_base;

#line 107
  if (pkt->ip6_data->iov_len >= sizeof(struct icmp6_hdr ) && 
  pkt->ip6_hdr.ip6_ctlun.ip6_un1.ip6_un1_nxt == IANA_ICMP) {
      req->cksum = 0;
      req->cksum = (((uint16_t )msg_cksum(& pkt->ip6_hdr, pkt->ip6_data, IANA_ICMP) << 8) | ((uint16_t )msg_cksum(& pkt->ip6_hdr, pkt->ip6_data, IANA_ICMP) >> 8)) & 0xffff;
    }
  return ICMPCoreP__IP__send(pkt);
}

# 88 "/opt/tinyos/tos/system/PoolP.nc"
static /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t */*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__Pool__get(void )
#line 88
{
  if (/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free) {
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__pool_t *rval = /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__queue[/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index];

#line 91
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__queue[/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index] = (void *)0;
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__free--;
      /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index++;
      if (/*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index == 5) {
          /*RPLDAORoutingEngineC.SendPoolP.PoolP*/PoolP__4__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/opt/tinyos/tos/system/QueueC.nc"
static error_t /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__enqueue(/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue_t newVal)
#line 97
{
  if (/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__size() < /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__Queue__maxSize()) {
      ;
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__queue[/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__tail] = newVal;
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__tail++;
      if (/*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__tail == 5) {
#line 102
        /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__tail = 0;
        }
#line 103
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__size++;
      /*RPLDAORoutingEngineC.SendQueueP*/QueueC__1__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
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

static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 462 "/opt/tinyos/tos/lib/net/rpl/RPLRoutingEngineP.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__RPLRouteInfo__resetTrickle(void )
#line 462
{
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__resetTrickleTime();
  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__isRunning()) {
    /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__chooseAdvertiseTime();
    }
}

#line 390
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__chooseAdvertiseTime(void )
#line 390
{
  if (!/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__running) {
      return;
    }
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__stop();
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__tricklePeriod;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime /= 2;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime += /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__Random__rand32() % /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime;
  /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__startOneShot(/*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__randomTime);
}

# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__TrickleTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(3U, dt);
#line 73
}
#line 73
# 125 "/opt/tinyos/tos/lib/net/rpl/RPLOF0P.nc"
static bool RPLOF0P__RPLOF__recomputeRoutes(void )
#line 125
{
  uint8_t indexset;
  uint8_t min = 0;
#line 127
  uint8_t count = 0;
  uint16_t minDesired;
  parent_t *parentNode;
#line 129
  parent_t *previousParent;

  parentNode = RPLOF0P__ParentTable__get(min);

  while (! parentNode->valid && 
  min < 20 && 
  parentNode->rank != INFINITE_RANK) {
      min++;
      parentNode = RPLOF0P__ParentTable__get(min);
    }

  minDesired = parentNode->etx_hop + parentNode->rank * 10;

  if (min == 20) {
      RPLOF0P__RPLOF__resetRank();
      RPLOF0P__RPLRoute__inconsistency();
      RPLOF0P__ForwardingTable__delRoute(RPLOF0P__route_key);
      RPLOF0P__route_key = ROUTE_INVAL_KEY;
      return FALSE;
    }





  parentNode = RPLOF0P__ParentTable__get(RPLOF0P__desiredParent);

  if (((((uint16_t )parentNode->parentIP.in6_u.u6_addr16[7] << 8) | ((uint16_t )parentNode->parentIP.in6_u.u6_addr16[7] >> 8)) & 0xffff) != 0) {
      RPLOF0P__minMetric = parentNode->etx_hop + parentNode->rank * 10;
    }




  if (min == RPLOF0P__desiredParent) {
    RPLOF0P__minMetric = minDesired;
    }
  for (indexset = min + 1; indexset < 20; indexset++) {
      parentNode = RPLOF0P__ParentTable__get(indexset);









      if (
#line 174
      parentNode->valid && 
      parentNode->etx_hop >= 0 && 
      parentNode->etx_hop + parentNode->rank * 10 < minDesired && 
      parentNode->rank < RPLOF0P__nodeRank && 
      parentNode->rank != INFINITE_RANK) {
          count++;
          min = indexset;
          minDesired = parentNode->etx_hop + parentNode->rank * 10;


          if (min == RPLOF0P__desiredParent) {

              RPLOF0P__minMetric = minDesired;
            }
        }
      else {
#line 188
        if (min == RPLOF0P__desiredParent) {
            RPLOF0P__minMetric = minDesired;
          }
        }
    }
  parentNode = RPLOF0P__ParentTable__get(min);


  if (parentNode->rank == INFINITE_RANK) {

      RPLOF0P__desiredParent = 20;
      RPLOF0P__ForwardingTable__delRoute(RPLOF0P__route_key);
      RPLOF0P__route_key = ROUTE_INVAL_KEY;
      return FALSE;
    }

  previousParent = RPLOF0P__ParentTable__get(RPLOF0P__desiredParent);


  if (
#line 206
  minDesired * 10 + 5 >= RPLOF0P__minMetric * 10 && 
  RPLOF0P__minMetric != 0 && 
  previousParent->valid) {





      min = RPLOF0P__desiredParent;
      minDesired = RPLOF0P__minMetric;
    }

  RPLOF0P__minMetric = minDesired;
  RPLOF0P__desiredParent = min;
  parentNode = RPLOF0P__ParentTable__get(RPLOF0P__desiredParent);








  RPLOF0P__route_key = RPLOF0P__ForwardingTable__addRoute((void *)0, 
  0, 
  & parentNode->parentIP, 
  RPL_IFACE);

  if (RPLOF0P__prevParent != parentNode->parentIP.in6_u.u6_addr16[7]) {







      RPLOF0P__newParent = TRUE;
      RPLOF0P__RPLDAO__newParent();
    }

  RPLOF0P__prevParent = parentNode->parentIP.in6_u.u6_addr16[7];
  return TRUE;
}

# 275 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static void RPLRankP__RPLRankInfo__inconsistencyDetected(void )
#line 275
{
  RPLRankP__parentNum = 0;
  RPLRankP__RPLOF__resetRank();
  RPLRankP__nodeRank = INFINITE_RANK;
  RPLRankP__resetValid();
}

#line 265
static void RPLRankP__resetValid(void )
#line 265
{
  uint8_t indexset;

#line 267
  for (indexset = 0; indexset < 20; indexset++) {
      RPLRankP__parentSet[indexset].valid = FALSE;
    }
}

# 130 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static error_t IPForwardingEngineP__ForwardingTable__delRoute(route_key_t key)
#line 130
{
  int i;

#line 132
  for (i = 0; i < ROUTE_TABLE_SZ; i++) {
      if (IPForwardingEngineP__routing_table[i].key == key) {

          if (IPForwardingEngineP__routing_table[i].prefixlen == 0) {
              IPForwardingEngineP__ForwardingTableEvents__defaultRouteRemoved();
            }

          memmove((void *)&IPForwardingEngineP__routing_table[i], (void *)&IPForwardingEngineP__routing_table[i + 1], 
          sizeof(struct route_entry ) * (ROUTE_TABLE_SZ - i - 1));
          IPForwardingEngineP__routing_table[ROUTE_TABLE_SZ - 1].valid = 0;
          return SUCCESS;
        }
    }
  return FAIL;
}

#line 100
static route_key_t IPForwardingEngineP__ForwardingTable__addRoute(const uint8_t *prefix, 
int prefix_len_bits, 
struct in6_addr *next_hop, 
uint8_t ifindex)
#line 103
{
  struct route_entry *entry;

  if (prefix_len_bits % 8 != 0 || prefix_len_bits > 128) {
#line 106
    return ROUTE_INVAL_KEY;
    }
#line 107
  entry = IPForwardingEngineP__ForwardingTable__lookupRoute(prefix, prefix_len_bits);
  if (entry == (void *)0 || entry->prefixlen != prefix_len_bits) {



      entry = IPForwardingEngineP__alloc_entry(prefix_len_bits);


      if (prefix_len_bits == 0) {
          IPForwardingEngineP__defaultRouteAddedTask__postTask();
        }
    }
  if (entry == (void *)0) {
    return ROUTE_INVAL_KEY;
    }
  entry->prefixlen = prefix_len_bits;
  entry->ifindex = ifindex;
  memcpy(& entry->prefix, prefix, prefix_len_bits / 8);
  if (next_hop) {
    memcpy(& entry->next_hop, next_hop, sizeof(struct in6_addr ));
    }
#line 127
  return entry->key;
}

# 573 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static void RPLRankP__getNewRank(void )
#line 573
{
  uint16_t prevRank = RPLRankP__nodeRank;
  bool newParent = FALSE;

  newParent = RPLRankP__RPLOF__recalcualateRank();
  RPLRankP__nodeRank = RPLRankP__RPLOF__getRank();




  if (newParent) {
      RPLRankP__minRank = RPLRankP__nodeRank;
      return;
    }

  if (RPLRankP__nodeRank < RPLRankP__minRank) {
      RPLRankP__minRank = RPLRankP__nodeRank;
      return;
    }



  if (
#line 594
  RPLRankP__nodeRank > prevRank && 
  RPLRankP__nodeRank - RPLRankP__minRank > RPLRankP__MAX_RANK_INCREASE && 
  RPLRankP__MAX_RANK_INCREASE != 0) {


      RPLRankP__nodeRank = INFINITE_RANK;
      RPLRankP__minRank = INFINITE_RANK;
      RPLRankP__RouteInfo__inconsistency();
      return;
    }
  RPLRankP__evictAll();
}

# 65 "/opt/tinyos/tos/chips/cc2520/packet/CC2420PacketP.nc"
static error_t CC2420PacketP__Acks__requestAck(message_t *p_msg)
#line 65
{
  unsigned char *__nesc_temp46;

#line 66
  (__nesc_temp46 = CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp46, __nesc_ntoh_leuint16(__nesc_temp46) | (1 << IEEE154_FCF_ACK_REQ)));
  return SUCCESS;
}

# 122 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len)
#line 122
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 124
  cc2420_header_t *header = CC2420CsmaP__CC2420PacketBody__getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP__CC2420PacketBody__getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (!CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 129
            FAIL;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
#line 132
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_TRANSMITTING);
      CC2420CsmaP__m_msg = p_msg;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }








  (__nesc_temp42 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (((1 << IEEE154_FCF_ACK_REQ) | (
  0x3 << IEEE154_FCF_SRC_ADDR_MODE)) | (
  0x3 << IEEE154_FCF_DEST_ADDR_MODE))));

  (__nesc_temp43 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN))));

  __nesc_hton_int8(metadata->ack.nxdata, FALSE);
  __nesc_hton_uint8(metadata->rssi.nxdata, 0);
  __nesc_hton_uint8(metadata->lqi.nxdata, 0);

  __nesc_hton_uint32(metadata->timestamp.nxdata, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP__ccaOn = TRUE;
  CC2420CsmaP__RadioBackoff__requestCca(CC2420CsmaP__m_msg);

  CC2420CsmaP__CC2420Transmit__send(CC2420CsmaP__m_msg, CC2420CsmaP__ccaOn);
  return SUCCESS;
}

# 825 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 825
{
  cc2420_header_t *header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg)->tx_power.nxdata);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP__CSN__clr();

  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP__m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.nxdata) - 1;

#line 846
    CC2420TransmitP__TXFIFO__write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.nxdata) - 1);
  }
}

# 305 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 206 "/opt/tinyos/tos/chips/msp430/usart/Msp430SpiNoDmaP.nc"
static error_t /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 208
{

  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_client = id;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf = tx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_rx_buf = rx_buf;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_len = len;
  /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos = 0;

  if (len) {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__enableRxIntr();
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp();
    }
  else {
      /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

#line 183
static void /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__continueOp(void )
#line 183
{

  uint8_t end;
  uint8_t tmp;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
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
#line 200
          /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__Usart__tx(/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf ? /*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_tx_buf[/*Msp430SpiNoDma0P.SpiP*/Msp430SpiNoDmaP__0__m_pos] : 0);
        }
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
}

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

# 73 "/opt/tinyos/tos/lib/timer/Timer.nc"
static void /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__startOneShot(uint32_t dt){
#line 73
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(7U, dt);
#line 73
}
#line 73
# 143 "/opt/tinyos/tos/system/StateImplP.nc"
static uint8_t StateImplP__State__getState(uint8_t id)
#line 143
{
  uint8_t theState;

#line 145
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 145
    theState = StateImplP__state[id];
#line 145
    __nesc_atomic_end(__nesc_atomic); }
  return theState;
}

# 147 "/opt/tinyos/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}

# 302 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void )
#line 302
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 303
    {
      unsigned int __nesc_temp = 
#line 303
      CC2420ControlP__m_short_addr;

      {
#line 303
        __nesc_atomic_end(__nesc_atomic); 
#line 303
        return __nesc_temp;
      }
    }
#line 305
    __nesc_atomic_end(__nesc_atomic); }
}

# 15 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan_frag.c"
static int IPDispatchP__lowpan_recon_start(struct ieee154_frame_addr *frame_addr, 
struct lowpan_reconstruct *recon, 
uint8_t *pkt, size_t len)
#line 17
{
  uint8_t *unpack_point;
#line 18
  uint8_t *unpack_end;
  struct packed_lowmsg msg;

  msg.data = pkt;
  msg.len = len;
  msg.headers = IPDispatchP__getHeaderBitmap(&msg);
  if (msg.headers == LOWMSG_NALP) {
#line 24
    return -1;
    }

  unpack_point = IPDispatchP__getLowpanPayload(&msg);
  len -= unpack_point - pkt;


  if (IPDispatchP__hasFrag1Header(&msg)) {
      IPDispatchP__getFragDgramTag(&msg, & recon->r_tag);
      IPDispatchP__getFragDgramSize(&msg, & recon->r_size);
    }
  else 
#line 34
    {
      recon->r_size = LIB6LOWPAN_MAX_LEN + LOWPAN_LINK_MTU;
    }
  recon->r_buf = ip_malloc(recon->r_size);
  if (! recon->r_buf) {
#line 38
    return -2;
    }
#line 39
  memset(recon->r_buf, 0, recon->r_size);
  recon->r_app_len = (void *)0;

  if (*unpack_point == LOWPAN_IPV6_PATTERN) {

      unpack_point++;
#line 44
      len--;
      memcpy(recon->r_buf, unpack_point, len);
      unpack_end = recon->r_buf + len;
    }
  else 
#line 47
    {

      unpack_end = IPDispatchP__lowpan_unpack_headers(recon, 
      frame_addr, 
      unpack_point, len);
    }

  if (!unpack_end) {
      ip_free(recon->r_buf);
      return -3;
    }

  if (!IPDispatchP__hasFrag1Header(&msg)) {
      recon->r_size = unpack_end - recon->r_buf;
    }
  recon->r_bytes_rcvd = unpack_end - recon->r_buf;
  ((struct ip6_hdr *)recon->r_buf)->ip6_ctlun.ip6_un1.ip6_un1_plen = ((
  (uint16_t )(recon->r_size - sizeof(struct ip6_hdr )) << 8) | ((uint16_t )(recon->r_size - sizeof(struct ip6_hdr )) >> 8)) & 0xffff;

  if (recon->r_app_len) {
      * recon->r_app_len = ((
      (uint16_t )(recon->r_size - (recon->r_transport_header - recon->r_buf)) << 8) | ((uint16_t )(recon->r_size - (recon->r_transport_header - recon->r_buf)) >> 8)) & 0xffff;
    }



  return 0;
}

# 526 "/opt/tinyos/support/sdk/c/blip/lib6lowpan/lib6lowpan.c"
static uint8_t *IPDispatchP__unpack_address(struct in6_addr *addr, uint8_t dispatch, 
int context, uint8_t *buf, 
ieee154_addr_t *frame, ieee154_panid_t pan)
#line 528
{
  memset(addr, 0, 16);
  if (!(dispatch & LOWPAN_IPHC_AC_CONTEXT)) {

      switch (dispatch & LOWPAN_IPHC_AM_MASK) {
          case LOWPAN_IPHC_AM_128: 
            memcpy(addr, buf, 16);
          return buf + 16;
          case LOWPAN_IPHC_AM_64: 
            addr->in6_u.u6_addr16[0] = (((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff;
          memcpy(&addr->in6_u.u6_addr8[8], buf, 8);
          return buf + 8;
          case LOWPAN_IPHC_AM_16: 
            addr->in6_u.u6_addr16[0] = (((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff;
          memcpy(&addr->in6_u.u6_addr8[14], buf, 2);
          return buf + 2;
          default: 
            addr->in6_u.u6_addr16[0] = (((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff;
          if (frame->ieee_mode == IEEE154_ADDR_EXT) {
              int i;

#line 548
              for (i = 0; i < 8; i++) 
                addr->in6_u.u6_addr8[i + 8] = frame->ieee_addr.laddr.data[7 - i];
              addr->in6_u.u6_addr8[8] ^= 0x2;
            }
          else 
#line 551
            {
              addr->in6_u.u6_addr16[4] = (((uint16_t )(pan & ~0x0200) << 8) | ((uint16_t )(pan & ~0x0200) >> 8)) & 0xffff;
              addr->in6_u.u6_addr8[11] = 0xff;
              addr->in6_u.u6_addr8[12] = 0xfe;
              addr->in6_u.u6_addr16[7] = (((uint16_t )frame->ieee_addr.saddr << 8) | ((uint16_t )frame->ieee_addr.saddr >> 8)) & 0xffff;
            }
          return buf;
        }
    }
  else 
#line 559
    {

      if ((dispatch & LOWPAN_IPHC_AM_MASK) == LOWPAN_IPHC_AM_128) {

          return buf;
        }
      else 
#line 564
        {
          int ctxlen = IPDispatchP__lowpan_extern_read_context(addr, context);

#line 566
          switch (dispatch & LOWPAN_IPHC_AM_MASK) {
              case LOWPAN_IPHC_AM_64: 
                memcpy(&addr->in6_u.u6_addr8[8], buf, 8);
              return buf + 8;
              case LOWPAN_IPHC_AM_16: 
                memcpy(&addr->in6_u.u6_addr8[14], buf, 2);
              return buf + 2;
              case LOWPAN_IPHC_AM_0: 


                if (ctxlen <= 64 && frame->ieee_mode == IEEE154_ADDR_EXT) {
                    int i;

#line 578
                    for (i = 0; i < 8; i++) 
                      addr->in6_u.u6_addr8[i + 8] = frame->ieee_addr.laddr.data[7 - i];
                    addr->in6_u.u6_addr8[8] ^= 0x2;
                  }
                else {
#line 581
                  if (ctxlen <= 112) {
                      memset(&addr->in6_u.u6_addr8[8], 0, 8);
                      addr->in6_u.u6_addr16[7] = (((uint16_t )frame->ieee_addr.saddr << 8) | ((uint16_t )frame->ieee_addr.saddr >> 8)) & 0xffff;
                    }
                  }
#line 585
              return buf;
            }
        }
    }
  return (void *)0;
}

# 213 "/opt/tinyos/tos/lib/net/blip/IPDispatchP.nc"
static void IPDispatchP__deliver(struct lowpan_reconstruct *recon)
#line 213
{
  struct ip6_hdr *iph = (struct ip6_hdr *)recon->r_buf;





  iph->ip6_ctlun.ip6_un1.ip6_un1_plen = (((uint16_t )(recon->r_bytes_rcvd - sizeof(struct ip6_hdr )) << 8) | ((uint16_t )(recon->r_bytes_rcvd - sizeof(struct ip6_hdr )) >> 8)) & 0xffff;
  IPDispatchP__IPLower__recv(iph, (void *)(iph + 1), & recon->r_meta);


  ip_free(recon->r_buf);
  recon->r_timeout = T_UNUSED;
  recon->r_buf = (void *)0;
}

# 23 "/opt/tinyos/tos/lib/net/blip/IPPacketC.nc"
static int IPPacketC__IPPacket__findHeader(struct ip_iovec *payload, 
uint8_t first_type, uint8_t *search_type)
#line 24
{
  int off = 0;
  uint8_t nxt = first_type;
  struct ip6_ext ext;



  while ((*search_type == 0xff && ((((
  nxt == IPV6_HOP || nxt == IPV6_ROUTING) || nxt == IPV6_FRAG) || 
  nxt == IPV6_DEST) || nxt == IPV6_MOBILITY)) || (
  *search_type != 0xff && *search_type != nxt)) {

      if (iov_read(payload, off, sizeof ext, (void *)&ext) != sizeof ext) {
        return -1;
        }
      nxt = ext.ip6e_nxt;
      off += (ext.ip6e_len + 1) * 8;
    }
  if (*search_type == 0xff) {
    *search_type = nxt;
    }
#line 44
  if (nxt == IPV6_NONEXT) {
    return -1;
    }
  else {
#line 47
    return off;
    }
}









static int IPPacketC__IPPacket__findTLV(struct ip_iovec *header, int ext_offset, uint8_t type)
#line 59
{
  struct ip6_ext ext;
  struct tlv_hdr tlv;
  int off = ext_offset;

  if (iov_read(header, off, sizeof ext, (void *)&ext) != sizeof ext) {
    return -1;
    }
#line 66
  off += sizeof ext;

  while (off - ext_offset < (ext.ip6e_len + 1) * 8) {
      if (iov_read(header, off, sizeof tlv, (void *)&tlv) != sizeof tlv) {
        return -1;
        }
#line 71
      if (tlv.type == type) {
#line 71
        return off;
        }
      else {
#line 72
        off += sizeof tlv + tlv.len;
        }
    }
#line 74
  return -1;
}

# 313 "/opt/tinyos/tos/lib/net/rpl/RPLRankP.nc"
static void RPLRankP__insertParent(parent_t parent)
#line 313
{
  uint8_t indexset;
  uint16_t tempEtx_hop;

  indexset = RPLRankP__getPreExistingParent(& parent.parentIP);

  ;
#line 319
  ;

  if (indexset != 20) {

      tempEtx_hop = RPLRankP__parentSet[indexset].etx_hop;
      RPLRankP__parentSet[indexset] = parent;

      if (tempEtx_hop > 10 && tempEtx_hop < 5) {
          tempEtx_hop = tempEtx_hop - 10;
          if (tempEtx_hop < 10) {
            tempEtx_hop = 10;
            }
        }
      else 
#line 330
        {
          tempEtx_hop = 10;
        }

      RPLRankP__parentSet[indexset].etx_hop = tempEtx_hop;
      RPLRankP__parentNum++;

      return;
    }

  for (indexset = 0; indexset < 20; indexset++) {
      if (! RPLRankP__parentSet[indexset].valid) {
          RPLRankP__parentSet[indexset] = parent;
          RPLRankP__parentNum++;
          break;
        }
    }
}


static void RPLRankP__evictParent(uint8_t indexset)
#line 350
{
  RPLRankP__parentSet[indexset].valid = FALSE;
  RPLRankP__parentNum--;
  ;
#line 353
  ;


  if (RPLRankP__parentNum == 0) {

      RPLRankP__RouteInfo__resetTrickle();
    }
}

# 156 "/opt/tinyos/tos/lib/net/rpl/RPLDAORoutingEngineP.nc"
static error_t /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLDAORouteInfo__startDAO(void )
#line 156
{


  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RemoveTimer__startPeriodic(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__remove_time);









  /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__DelayDAOTimer__startOneShot(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__delay_dao + /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__Random__rand16() % 100);

  if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__isRunning()) {
      return SUCCESS;
    }
  else {
#line 173
    if (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__RPLRouteInfo__getRank() == ROOT_RANK) {
        return SUCCESS;
      }
    else 
#line 175
      {
        /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__GenerateDAOTimer__startOneShot(/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__dao_rate + 
        /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__Random__rand16() % (/*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__dao_rate / 10));
      }
    }
#line 179
  return SUCCESS;
}

# 769 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__waitForNextPacket(void )
#line 769
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 770
    {
      if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 773
            __nesc_atomic_end(__nesc_atomic); 
#line 773
            return;
          }
        }
      CC2420ReceiveP__receivingPacket = FALSE;
#line 788
      if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__m_missed_packets) {
              CC2420ReceiveP__m_missed_packets--;
            }





          CC2420ReceiveP__beginReceive();
        }
      else 
        {

          CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__m_missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 807
    __nesc_atomic_end(__nesc_atomic); }
}

#line 716
static void CC2420ReceiveP__beginReceive(void )
#line 716
{
  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 718
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 722
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 759
static void CC2420ReceiveP__receive(void )
#line 759
{
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf), 1);
}

# 189 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

# 179 "/opt/tinyos/tos/chips/cc2520/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__StdControl__stop(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__BackoffTimer__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 171 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static error_t CC2420ReceiveP__StdControl__stop(void )
#line 171
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 813
static void CC2420ReceiveP__reset_state(void )
#line 813
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 815
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__m_timestamp_head = 0;
  CC2420ReceiveP__m_timestamp_size = 0;
  CC2420ReceiveP__m_missed_packets = 0;
}

# 216 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static error_t CC2420ControlP__CC2420Power__stopVReg(void )
#line 216
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 220 "/opt/tinyos/tos/chips/cc2520/link/PacketLinkP.nc"
static void PacketLinkP__signalDone(error_t error)
#line 220
{
  PacketLinkP__DelayTimer__stop();
  PacketLinkP__SendState__toIdle();


  if (__nesc_ntoh_uint16(PacketLinkP__CC2420PacketBody__getMetadata(PacketLinkP__currentSendMsg)->maxRetries.nxdata) > 0) {
    __nesc_hton_uint16(PacketLinkP__CC2420PacketBody__getMetadata(PacketLinkP__currentSendMsg)->maxRetries.nxdata, PacketLinkP__totalRetries);
    }
  PacketLinkP__Send__sendDone(PacketLinkP__currentSendMsg, error);
}

# 56 "/opt/tinyos/tos/interfaces/State.nc"
static void PacketLinkP__SendState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(4U);
#line 56
}
#line 56
static void UniqueSendP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(2U);
#line 56
}
#line 56
# 313 "/opt/tinyos/tos/lib/net/blip/IPForwardingEngineP.nc"
static void IPForwardingEngineP__IPForward__sendDone(uint8_t ifindex, struct send_info *status)
#line 313
{
  struct in6_addr next;
  struct in6_iid *iid = (struct in6_iid *)status->upper_data;

#line 316
  memset(next.in6_u.u6_addr8, 0, 16);
  next.in6_u.u6_addr16[0] = (((uint16_t )0xfe80 << 8) | ((uint16_t )0xfe80 >> 8)) & 0xffff;
  ;
#line 318
  ;

  if (iid != (void *)0) {
      memcpy(&next.in6_u.u6_addr8[8], iid->data, 8);
      IPForwardingEngineP__ForwardingEvents__linkResult(ifindex, &next, status);
      IPForwardingEngineP__Pool__put(iid);
    }
}

# 85 "/opt/tinyos/tos/system/QueueC.nc"
static /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue(void )
#line 85
{
  /*IPDispatchC.QueueC*/QueueC__0__queue_t t = /*IPDispatchC.QueueC*/QueueC__0__Queue__head();

#line 87
  ;
  if (!/*IPDispatchC.QueueC*/QueueC__0__Queue__empty()) {
      /*IPDispatchC.QueueC*/QueueC__0__head++;
      if (/*IPDispatchC.QueueC*/QueueC__0__head == 12) {
#line 90
        /*IPDispatchC.QueueC*/QueueC__0__head = 0;
        }
#line 91
      /*IPDispatchC.QueueC*/QueueC__0__size--;
      /*IPDispatchC.QueueC*/QueueC__0__printQueue();
    }
  return t;
}

# 95 "/opt/tinyos/tos/interfaces/StdControl.nc"
static error_t IPStackControlP__RoutingControl__start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = /*RPLRoutingEngineC.Routing*/RPLRoutingEngineP__0__StdControl__start();
#line 95
  __nesc_result = ecombine(__nesc_result, /*RPLDAORoutingEngineC.DAORouting*/RPLDAORoutingEngineP__0__StdControl__start());
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 329 "/opt/tinyos/tos/chips/cc2520/spi/CC2420SpiP.nc"
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 733 "/opt/tinyos/tos/chips/cc2520/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 733
{








  CC2420ReceiveP__reset_state();

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

# 479 "/opt/tinyos/tos/chips/cc2520/control/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 479
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 482
    {
      channel = CC2420ControlP__m_channel;
    }
#line 484
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP__writeMdmctrl0(void )
#line 496
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 497
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP__addressRecognition && CC2420ControlP__hwAddressRecognition ? 1 : 0) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 506
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 515
{
  nxle_uint16_t id[6];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 518
    {

      memcpy((uint8_t *)id, CC2420ControlP__m_ext_addr.data, 8);
      __nesc_hton_leuint16(id[4].nxdata, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[5].nxdata, CC2420ControlP__m_short_addr);
    }
#line 523
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__IEEEADR__write(0, (uint8_t *)&id, 12);
}

# 391 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static void CoapUdpServerP__add_contents(coap_pdu_t *pdu, unsigned char mediatype, 
unsigned int len, unsigned char *data)
#line 392
{

  unsigned char ct = 40;

#line 395
  if (!pdu) {
    return;
    }

  coap_add_option(pdu, 1, 1, &ct);

  coap_add_data(pdu, len, data);
}

#line 897
static int CoapUdpServerP__coap_save_splitphase(coap_context_t *ctx, 
coap_queue_t *node)
#line 898
{
  coap_queue_t *new_node;
  coap_opt_t *opt;

  ;
#line 902
  ;

  new_node = coap_new_node();
  if (!new_node) {
      ;
#line 906
      ;
      return -1;
    }

  new_node->pdu = coap_new_pdu();
  if (! new_node->pdu) {
      ;
#line 912
      ;
      coap_delete_node(new_node);
      return -1;
    }

  memcpy(& new_node->remote, & node->remote, sizeof(struct sockaddr_in6 ));
  ;
#line 918
  ;


  memcpy(new_node->pdu->hdr, node->pdu->hdr, node->pdu->length);
  new_node->pdu->length = node->pdu->length;


  {
#line 925
    unsigned char opt_code = 0;
#line 925
    unsigned char cnt;

#line 925
    * &opt = (coap_opt_t *)((unsigned char *)node->pdu->hdr + sizeof(coap_hdr_t ));
#line 925
    for (cnt = new_node->pdu->hdr->optcnt; cnt; --cnt) {
#line 925
        opt_code += (* &opt)->sval.delta;
#line 925
        * &opt = (coap_opt_t *)((unsigned char *)* &opt + (((* &opt)->lval.flag == 15 ? (* &opt)->lval.length + 15 : (* &opt)->sval.length) + ((* &opt)->lval.flag == 15 ? 2 : 1)));
      }
  }
#line 925
  ;

  if ((unsigned char *)new_node->pdu->hdr + new_node->pdu->length < 
  (unsigned char *)opt) {
    new_node->pdu->data = (unsigned char *)new_node->pdu->hdr + new_node->pdu->length;
    }
  else {
#line 931
    new_node->pdu->data = (unsigned char *)opt;
    }


  coap_insert_node(& ctx->splitphasequeue, new_node, order_transaction_id);





  return 0;
}

#line 366
static coap_pdu_t *CoapUdpServerP__new_rst(coap_context_t *ctx, coap_queue_t *node, 
unsigned int code)
#line 367
{
  coap_pdu_t *pdu;

#line 369
  {
#line 369
    pdu = coap_new_pdu();
#line 369
    if (pdu) {
#line 369
        coap_opt_t *tok;

#line 369
        pdu->hdr->type = 3;
#line 369
        pdu->hdr->code = code;
#line 369
        pdu->hdr->id = node->pdu->hdr->id;
#line 369
        tok = coap_check_option(node->pdu, 11);
#line 369
        if (tok && 1) {
#line 369
          coap_add_option(pdu, 11, tok->lval.flag == 15 ? tok->lval.length + 15 : tok->sval.length, (unsigned char *)& *tok + (tok->lval.flag == 15 ? 2 : 1));
          }
      }
  }
#line 369
  ;
  return pdu;
}

# 81 "/opt/tinyos/tos/chips/cc2520/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP__SplitControl__start(void )
#line 81
{
  if (CC2420CsmaP__SplitControlState__requestState(CC2420CsmaP__S_STARTING) == SUCCESS) {
      CC2420CsmaP__CC2420Power__startVReg();
      return SUCCESS;
    }
  else {
#line 86
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
        return EALREADY;
      }
    else {
#line 89
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 93
  return EBUSY;
}

# 110 "/opt/tinyos/tos/lib/net/coap/CoapUdpServerP.nc"
static uint8_t CoapUdpServerP__get_key(uint8_t *uri, uint8_t len)
#line 110
{
  uint8_t i;

  for (i = 0; i < 1; i++) {
      if (strncmp(uri_key_map[i].uri, (const char *)uri, len) == 0) {
        return uri_key_map[i].key;
        }
    }
#line 117
  return COAP_NO_SUCH_RESOURCE;
}

# 64 "/opt/tinyos/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
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

# 96 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(0x0012)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 153 "/opt/tinyos/tos/system/ArbiterP.nc"
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

# 101 "/opt/tinyos/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
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

