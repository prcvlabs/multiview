/*************************************************************************
ALGLIB 3.12.0 (source code generated 2017-08-22)
Copyright (c) Sergey Bochkanov (ALGLIB project).

>>> SOURCE LICENSE >>>
This  program  is  a  Commercial Edition  of the ALGLIB package  licensed
to Perceive, Inc. (Licensee), agreement ID is AGR-20170916-1

As of 2017-09-16:
* DEV-1 license plan is purchased (1 developer)
* following developers are registered for this license:
  * amichaux@perceiveinc.com

========================== GENERAL INFORMATION ==========================

1. Only Licensee and  its Sublicensees  can  use/distribute  it according
to  the  ALGLIB  License  Agreement (see  below) between  Licensor  (Sole
Proprietor Bochkanov Sergey Anatolyevich) and Licensee.

2. All developers working  for  Licensee  should  register  themselves at
alglib.net. Quick access link for Licensee's account can be found at  the
section 9 of scanned copy of ALGLIB License Agreement or in e-mails  from
ALGLIB Project.

3. This  source  code  may  contain  modifications  made  by  Licensee or
Sublicensees  which  fall under the terms of the ALGLIB License Agreement
too.

4. Text  below  is  an excerpt from complete ALGLIB License Agreement,  a
part which governs usage and redistribution rights granted  to  Licensee.
See agreement-v6.pdf at the root of ALGLIB distribution for complete text
of license agreement.

================ ALGLIB LICENSE AGREEMENT ( APPENDIX A ) ================

DEFINITIONS:
* "ALGLIB" – software delivered by Licensor  to  Licensee  under  present
  Agreement. ALGLIB may include  Binary  Components  (delivered  only  in
  binary form) and Source  Code  Components  (with  optional  precompiled
  binary form).
  ALGLIB  may  include  optional  third-party  components, which may have
  their own licensing terms. Specific list of components  and  additional
  licensing terms (if there are any) is included in the license.txt  file
  in the root of archive containing ALGLIB. If you  decide  not  to  link
  ALGLIB with these optional components, their  licensing  terms  do  not
  apply to you.
* "Application"  -  program  developed  by  Licensee  (either  standalone
  application or software development library) which includes  ALGLIB  as
  one of its parts .
* "Sublicensee"  -  any  party  (including  resellers)   which   receives
  Application from Licensee or another Sublicensee.
* "Application License Agreement"  -  agreement  which   governs   usage/
  redistribution of the Application.
  
LICENSE GRANT:
Subject to the License Restrictions below, Licensor  grants  to  Licensee
the following non-exclusive royalty-free licenses:
A. To modify Source Code Components of ALGLIB and to use modified version
   on the terms of this Agreement.
B. To  develop  Applications  which  use  ALGLIB  and  to distribute such
   Applications in Binary and/or Source Code forms,  with  ALGLIB  either
   statically or dynamically linked. This right is granted provided that:
   * distribution of Source Code forms of Application/ALGLIB is performed
     subject to additional conditions set by clause H (this clause is not
     applied to binary-only distribution)
   * such Applications add significant  primary  functionality  different
     from that of the ALGLIB.
   * such Applications do not expose ALGLIB API (application  programming
     interface) either directly or indirectly
   * Sublicensee  has  no   right   to  use  ALGLIB  except  as  part  of
     the Application
   * any  subsequent  redistribution   respects    conditions    of   the
     present Agreement
   * all developers working for Licensee  should  register  at  company's
     account at www.alglib.net (in order to find login link, see  section
     9 of scanned copy of ALGLIB License Agreement -or find it in e-mails
     from ALGLIB Project).
C. To use Resellers for distribution of the  Application  (in  Binary  or
   Source Code forms), provided that the only activity Reseller  performs
   with Application is redistribution.
   
LICENSE RESTRICTIONS:
D. Licensee/Sublicensee may NOT use, copy or distribute ALGLIB except  as
   provided in this Agreement.
D2. Licensee/Sublicensee may NOT rent or lease ALGLIB to any third party.
E. Licensee/Sublicensee may NOT disassemble, reverse engineer, decompile,
   modify Binary Components of ALGLIB or compiled forms  of  Source  Code
   components.
F. Licensee/Sublicensee  may  NOT  remove  any  copyright notice from the
   Source Code / Binary Components.
G. Licensee/Sublicensee may NOT  disable/remove  code  which  checks  for
   presence of license keys (if such code is included in ALGLIB) from the
   Source Code / Binary Components.
H. Distribution of  Source  Code  forms  of  Application/ALGLIB  must  be
   performed subject to additional conditions:
   * Source Code Components of ALGLIB are distributed only as part of the
     Application. They are not  publicly  distributed.  Sublicensee  must
     explicitly accept Application License Agreement in order  to  access
     ALGLIB source code.
   * Sublicensee has no right to redistribute Application/ALGLIB (in  any
     form, Binary or Source Code), unless Sublicensee is Reseller who  is
     fully compliant with conditions set by clause C.
   * Sublicensee has no right to modify ALGLIB Source  Code,  except  for
     the purpose of fixing bugs
   * Sublicensee has no right to workaround "use ALGLIB only as  part  of
     the Application" limitation by sequentially modifying Application in
     a way which effectively creates new program with different  purpose.
     Application   License  Agreement  may  (a)  explicitly  forbid  such
     modifications, or (b) allow only limited set of "safe" modifications
     (developing plugins, fixing bugs, modifying only specific  parts  of
     the Application).
     
COPYRIGHT:
Title to the ALGLIB and all copies  thereof  remain  with  Licensor.  The
ALGLIB is copyrighted and is protected  by  Russian  copyright  laws  and
international treaty provisions. You will not remove any copyright notice
from the ALGLIB files. You agree to prevent any unauthorized  copying  of
the ALGLIB. Except as expressly provided herein, Licensor does not  grant
any express or implied right to you under Licensor  patents,  copyrights,
trademarks, or trade secret information.
>>> END OF LICENSE >>>
*************************************************************************/
#ifndef _ap_h
#define _ap_h

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <cstring>
#include <iostream>
#include <math.h>

#if defined(__CODEGEARC__)
#include <list>
#include <vector>
#elif defined(__BORLANDC__)
#include <list.h>
#include <vector.h>
#else
#include <list>
#include <vector>
#endif

#define AE_USE_CPP
/* Definitions */
#define AE_UNKNOWN 0
#define AE_MSVC 1
#define AE_GNUC 2
#define AE_SUNC 3
#define AE_INTEL 1
#define AE_SPARC 2
#define AE_WINDOWS 1
#define AE_POSIX 2
#define AE_LOCK_ALIGNMENT 16

/* in case no OS is defined, use AE_UNKNOWN */
#ifndef AE_OS
#define AE_OS AE_UNKNOWN
#endif

/* automatically determine compiler */
#define AE_COMPILER AE_UNKNOWN
#ifdef __GNUC__
#undef AE_COMPILER
#define AE_COMPILER AE_GNUC
#endif
#if defined(__SUNPRO_C)||defined(__SUNPRO_CC)
#undef AE_COMPILER
#define AE_COMPILER AE_SUNC
#endif
#ifdef _MSC_VER
#undef AE_COMPILER
#define AE_COMPILER AE_MSVC
#endif

/* compiler-specific definitions */
#if AE_COMPILER==AE_MSVC
#define ALIGNED __declspec(align(8))
#elif AE_COMPILER==AE_GNUC
#define ALIGNED __attribute__((aligned(8)))
#else
#define ALIGNED
#endif

/* now we are ready to include headers */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <math.h>
#include <stddef.h>

#if defined(AE_HAVE_STDINT)
#include <stdint.h>
#endif

/*
 * SSE2 intrinsics
 *
 * Preprocessor directives below:
 * - include headers for SSE2 intrinsics
 * - define AE_HAS_SSE2_INTRINSICS definition
 *
 * These actions are performed when we have:
 * - x86 architecture definition (AE_CPU==AE_INTEL)
 * - compiler which supports intrinsics
 *
 * Presence of AE_HAS_SSE2_INTRINSICS does NOT mean that our CPU
 * actually supports SSE2 - such things should be determined at runtime
 * with ae_cpuid() call. It means that we are working under Intel and
 * out compiler can issue SSE2-capable code.
 *
 */
#if defined(AE_CPU)
#if AE_CPU==AE_INTEL
#if AE_COMPILER==AE_MSVC
#include <emmintrin.h>
#define AE_HAS_SSE2_INTRINSICS
#endif
#if AE_COMPILER==AE_GNUC
#include <xmmintrin.h>
#define AE_HAS_SSE2_INTRINSICS
#endif
#if AE_COMPILER==AE_SUNC
#include <xmmintrin.h>
#include <emmintrin.h>
#define AE_HAS_SSE2_INTRINSICS
#endif
#endif
#endif



/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS DECLARATIONS FOR BASIC FUNCTIONALITY 
// LIKE MEMORY MANAGEMENT FOR VECTORS/MATRICES WHICH IS SHARED 
// BETWEEN C++ AND PURE C LIBRARIES
//
/////////////////////////////////////////////////////////////////////////
namespace alglib_impl
{

/* if we work under C++ environment, define several conditions */
#ifdef AE_USE_CPP
#define AE_USE_CPP_BOOL
#define AE_USE_CPP_ERROR_HANDLING
#define AE_USE_CPP_SERIALIZATION
#include <iostream>
#endif

/*
 * define ae_int32_t, ae_int64_t, ae_int_t, ae_bool, ae_complex, ae_error_type and ae_datatype
 */

#if defined(AE_INT32_T)
typedef AE_INT32_T ae_int32_t;
#endif
#if defined(AE_HAVE_STDINT) && !defined(AE_INT32_T)
typedef int32_t ae_int32_t;
#endif
#if !defined(AE_HAVE_STDINT) && !defined(AE_INT32_T)
#if AE_COMPILER==AE_MSVC
typedef __int32 ae_int32_t;
#endif
#if (AE_COMPILER==AE_GNUC) || (AE_COMPILER==AE_SUNC) || (AE_COMPILER==AE_UNKNOWN)
typedef int ae_int32_t;
#endif
#endif

#if defined(AE_INT64_T)
typedef AE_INT64_T ae_int64_t;
#endif
#if defined(AE_HAVE_STDINT) && !defined(AE_INT64_T)
typedef int64_t ae_int64_t;
#endif
#if !defined(AE_HAVE_STDINT) && !defined(AE_INT64_T)
#if AE_COMPILER==AE_MSVC
typedef __int64 ae_int64_t;
#endif
#if (AE_COMPILER==AE_GNUC) || (AE_COMPILER==AE_SUNC) || (AE_COMPILER==AE_UNKNOWN)
typedef signed long long ae_int64_t;
#endif
#endif

#if !defined(AE_INT_T)
typedef ptrdiff_t ae_int_t;
#endif

#if !defined(AE_USE_CPP_BOOL)
#define ae_bool char
#define ae_true 1
#define ae_false 0
#else
#define ae_bool bool
#define ae_true true
#define ae_false false
#endif

typedef struct { double x, y; } ae_complex;

typedef enum
{
    ERR_OK = 0,
    ERR_OUT_OF_MEMORY = 1,
    ERR_XARRAY_TOO_LARGE = 2,
    ERR_ASSERTION_FAILED = 3
} ae_error_type;

typedef ae_int_t ae_datatype;

/*
 * other definitions
 */
enum { OWN_CALLER=1, OWN_AE=2 };
enum { ACT_UNCHANGED=1, ACT_SAME_LOCATION=2, ACT_NEW_LOCATION=3 };
enum { DT_BOOL=1, DT_INT=2, DT_REAL=3, DT_COMPLEX=4 };
enum { CPU_SSE2=1 };

/************************************************************************
x-string (zero-terminated):
    owner       OWN_CALLER or OWN_AE. Determines what to do on realloc().
                If vector is owned by caller, X-interface  will  just set
                ptr to NULL before realloc(). If it is  owned  by  X,  it
                will call ae_free/x_free/aligned_free family functions.

    last_action ACT_UNCHANGED, ACT_SAME_LOCATION, ACT_NEW_LOCATION
                contents is either: unchanged, stored at the same location,
                stored at the new location.
                this field is set on return from X.

    ptr         pointer to the actual data

Members of this structure are ae_int64_t to avoid alignment problems.
************************************************************************/
typedef struct
{
    ALIGNED ae_int64_t     owner;
    ALIGNED ae_int64_t     last_action;
    ALIGNED char *ptr;
} x_string;

/************************************************************************
x-vector:
    cnt         number of elements

    datatype    one of the DT_XXXX values

    owner       OWN_CALLER or OWN_AE. Determines what to do on realloc().
                If vector is owned by caller, X-interface  will  just set
                ptr to NULL before realloc(). If it is  owned  by  X,  it
                will call ae_free/x_free/aligned_free family functions.

    last_action ACT_UNCHANGED, ACT_SAME_LOCATION, ACT_NEW_LOCATION
                contents is either: unchanged, stored at the same location,
                stored at the new location.
                this field is set on return from X interface and may be
                used by caller as hint when deciding what to do with data
                (if it was ACT_UNCHANGED or ACT_SAME_LOCATION, no array
                reallocation or copying is required).

    ptr         pointer to the actual data

Members of this structure are ae_int64_t to avoid alignment problems.
************************************************************************/
typedef struct
{
    ALIGNED ae_int64_t     cnt;
    ALIGNED ae_int64_t     datatype;
    ALIGNED ae_int64_t     owner;
    ALIGNED ae_int64_t     last_action;
    ALIGNED void *ptr;
} x_vector;


/************************************************************************
x-matrix:
    rows        number of rows. may be zero only when cols is zero too.

    cols        number of columns. may be zero only when rows is zero too.

    stride      stride, i.e. distance between first elements of rows (in bytes)

    datatype    one of the DT_XXXX values

    owner       OWN_CALLER or OWN_AE. Determines what to do on realloc().
                If vector is owned by caller, X-interface  will  just set
                ptr to NULL before realloc(). If it is  owned  by  X,  it
                will call ae_free/x_free/aligned_free family functions.

    last_action ACT_UNCHANGED, ACT_SAME_LOCATION, ACT_NEW_LOCATION
                contents is either: unchanged, stored at the same location,
                stored at the new location.
                this field is set on return from X interface and may be
                used by caller as hint when deciding what to do with data
                (if it was ACT_UNCHANGED or ACT_SAME_LOCATION, no array
                reallocation or copying is required).

    ptr         pointer to the actual data, stored rowwise

Members of this structure are ae_int64_t to avoid alignment problems.
************************************************************************/
typedef struct
{
    ALIGNED ae_int64_t     rows;
    ALIGNED ae_int64_t     cols;
    ALIGNED ae_int64_t     stride;
    ALIGNED ae_int64_t     datatype;
    ALIGNED ae_int64_t     owner;
    ALIGNED ae_int64_t     last_action;
    ALIGNED void *ptr;
} x_matrix;


/************************************************************************
dynamic block which may be automatically deallocated during stack unwinding

p_next          next block in the stack unwinding list.
                NULL means that this block is not in the list
deallocator     deallocator function which should be used to deallocate block.
                NULL for "special" blocks (frame/stack boundaries)
ptr             pointer which should be passed to the deallocator.
                may be null (for zero-size block), DYN_BOTTOM or DYN_FRAME
                for "special" blocks (frame/stack boundaries).

************************************************************************/
typedef struct ae_dyn_block
{
    struct ae_dyn_block * volatile p_next;
    /* void *deallocator; */
    void (*deallocator)(void*);
    void * volatile ptr;
} ae_dyn_block;

typedef void(*ae_deallocator)(void*);

/************************************************************************
frame marker
************************************************************************/
typedef struct ae_frame
{
    ae_dyn_block db_marker;
} ae_frame;

/************************************************************************
ALGLIB environment state
************************************************************************/
typedef struct ae_state
{
    /*
     * endianness type: AE_LITTLE_ENDIAN or AE_BIG_ENDIAN
     */
    ae_int_t endianness;
    
    /*
     * double value for NAN
     */
    double v_nan;
    
    /*
     * double value for +INF
     */
    double v_posinf;
    
    /*
     * double value for -INF
     */
    double v_neginf;
    
    /*
     * pointer to the top block in a stack of frames
     * which hold dynamically allocated objects
     */
    ae_dyn_block * volatile p_top_block;
    ae_dyn_block last_block;
    
    /*
     * jmp_buf for cases when C-style exception handling is used
     */
#ifndef AE_USE_CPP_ERROR_HANDLING
    jmp_buf * volatile break_jump;
#endif

    /*
     * ae_error_type of the last error (filled when exception is thrown)
     */
    ae_error_type volatile last_error;
    
    /*
     * human-readable message (filled when exception is thrown)
     */
    const char* volatile error_msg;
    
    /*
     * threading information:
     * a) current thread pool
     * b) current worker thread
     * c) parent task (one we are solving right now)
     * d) thread exception handler (function which must be called
     *    by ae_assert before raising exception).
     *
     * NOTE: we use void* to store pointers in order to avoid explicit dependency on smp.h
     */
    void *worker_thread;
    void *parent_task;
    void (*thread_exception_handler)(void*);
    
} ae_state;

/************************************************************************
Serializer:

* ae_stream_writer type is a function pointer for stream  writer  method;
  this pointer is used by X-core for out-of-core serialization  (say,  to
  serialize ALGLIB structure directly to managed C# stream).
  
  This function accepts two parameters: pointer to  ANSI  (7-bit)  string
  and pointer-sized integer passed to serializer  during  initialization.
  String being passed is a part of the data stream; aux paramerer may  be
  arbitrary value intended to be used by actual implementation of  stream
  writer. String parameter may include spaces and  linefeed  symbols,  it
  should be written to stream as is.
  
  Return value must be zero for success or non-zero for failure.
  
* ae_stream_reader type is a function pointer for stream  reader  method;
  this pointer is used by X-core for out-of-core unserialization (say, to
  unserialize ALGLIB structure directly from managed C# stream).
  
  This function accepts three parameters: pointer-sized integer passed to
  serializer  during  initialization; number  of  symbols  to  read  from
  stream; pointer to buffer used to store next  token  read  from  stream
  (ANSI encoding is used, buffer is large enough to store all symbols and
  trailing zero symbol).
  
  Number of symbols to read is always positive.
  
  After being called by X-core, this function must:
  * skip all space and linefeed characters from the current  position  at
    the stream and until first non-space non-linefeed character is found
  * read exactly cnt symbols  from  stream  to  buffer;  check  that  all
    symbols being read are non-space non-linefeed ones
  * append trailing zero symbol to buffer
  * return value must be zero on success, non-zero if  even  one  of  the
    conditions above fails. When reader returns non-zero value,  contents
    of buf is not used.
************************************************************************/
typedef char(*ae_stream_writer)(const char *p_string, ae_int_t aux);
typedef char(*ae_stream_reader)(ae_int_t aux, ae_int_t cnt, char *p_buf);

typedef struct
{
    ae_int_t mode;
    ae_int_t entries_needed;
    ae_int_t entries_saved;
    ae_int_t bytes_asked;
    ae_int_t bytes_written;

#ifdef AE_USE_CPP_SERIALIZATION
    std::string     *out_cppstr;
#endif
    char            *out_str; /* pointer to the current position at the output buffer; advanced with each write operation */
    const char      *in_str;  /* pointer to the current position at the input  buffer; advanced with each read  operation */
    ae_int_t         stream_aux;
    ae_stream_writer stream_writer;
    ae_stream_reader stream_reader;
} ae_serializer;


typedef struct ae_vector
{
    /*
     * Number of elements in array, cnt>=0
     */
    ae_int_t cnt;
    
    /*
     * Either DT_BOOL, DT_INT, DT_REAL or DT_COMPLEX
     */
    ae_datatype datatype;
    
    /*
     * If ptr points to memory owned and managed by ae_vector itself,
     * this field is ae_false. If vector was attached to x_vector structure
     * with ae_vector_attach_to_x(), this field is ae_true.
     */
    ae_bool is_attached;
    
    /*
     * ae_dyn_block structure which manages data in ptr. This structure
     * is responsible for automatic deletion of object when its frame
     * is destroyed.
     */
    ae_dyn_block data;
    
    /*
     * Pointer to data.
     * User usually works with this field.
     */
    union
    {
        void *p_ptr;
        ae_bool *p_bool;
        ae_int_t *p_int;
        double *p_double;
        ae_complex *p_complex;
    } ptr;
} ae_vector;

typedef struct ae_matrix
{
    ae_int_t rows;
    ae_int_t cols;
    ae_int_t stride;
    ae_datatype datatype;
    
    /*
     * If ptr points to memory owned and managed by ae_vector itself,
     * this field is ae_false. If vector was attached to x_vector structure
     * with ae_vector_attach_to_x(), this field is ae_true.
     */
    ae_bool is_attached;
    
    ae_dyn_block data;
    union
    {
        void *p_ptr;
        void **pp_void;
        ae_bool **pp_bool;
        ae_int_t **pp_int;
        double **pp_double;
        ae_complex **pp_complex;
    } ptr;
} ae_matrix;

typedef struct ae_smart_ptr
{
    /* pointer to subscriber; all changes in ptr are translated to subscriber */
    void **subscriber;
    
    /* pointer to object */
    void *ptr;
    
    /* whether smart pointer owns ptr */
    ae_bool is_owner;
    
    /* whether object pointed by ptr is dynamic - clearing such object requires BOTH
       calling destructor function AND calling ae_free for memory occupied by object. */
    ae_bool is_dynamic;
    
    /* destructor function for pointer; clears all dynamically allocated memory */
    void (*destroy)(void*);
    
    /* frame entry; used to ensure automatic deallocation of smart pointer in case of exception/exit */
    ae_dyn_block frame_entry;
} ae_smart_ptr;


/*************************************************************************
Lock.

This structure provides OS-independent non-reentrant lock:
* under Windows/Posix systems it uses system-provided locks
* under Boost it uses OS-independent lock provided by Boost package
* when no OS is defined, it uses "fake lock" (just stub which is not thread-safe):
  a) "fake lock" can be in locked or free mode
  b) "fake lock" can be used only from one thread - one which created lock
  c) when thread acquires free lock, it immediately returns
  d) when thread acquires busy lock, program is terminated
     (because lock is already acquired and no one else can free it)
*************************************************************************/
typedef struct
{
    /*
     * Pointer to _lock structure. This pointer has type void* in order to
     * make header file OS-independent (lock declaration depends on OS).
     */
    void *ptr;
} ae_lock;


/*************************************************************************
Shared pool: data structure used to provide thread-safe access to pool  of
temporary variables.
*************************************************************************/
typedef struct ae_shared_pool_entry
{
    void * volatile obj;
    void * volatile next_entry;
} ae_shared_pool_entry;

typedef struct ae_shared_pool
{
    /* lock object which protects pool */
    ae_lock pool_lock;
    
    /* seed object (used to create new instances of temporaries) */
    void                    * volatile seed_object;
    
    /*
     * list of recycled OBJECTS:
     * 1. entries in this list store pointers to recycled objects
     * 2. every time we retrieve object, we retrieve first entry from this list,
     *    move it to recycled_entries and return its obj field to caller/
     */
    ae_shared_pool_entry    * volatile recycled_objects;
    
    /* 
     * list of recycled ENTRIES:
     * 1. this list holds entries which are not used to store recycled objects;
     *    every time recycled object is retrieved, its entry is moved to this list.
     * 2. every time object is recycled, we try to fetch entry for him from this list
     *    before allocating it with malloc()
     */
    ae_shared_pool_entry    * volatile recycled_entries;
    
    /* enumeration pointer, points to current recycled object*/
    ae_shared_pool_entry    * volatile enumeration_counter;
    
    /* size of object; this field is used when we call malloc() for new objects */
    ae_int_t                size_of_object;
    
    /* initializer function; accepts pointer to malloc'ed object, initializes its fields */
    void (*init)(void* dst, ae_state* state);
    
    /* copy constructor; accepts pointer to malloc'ed, but not initialized object */
    void (*init_copy)(void* dst, void* src, ae_state* state);
    
    /* destructor function; */
    void (*destroy)(void* ptr);
    
    /* frame entry; contains pointer to the pool object itself */
    ae_dyn_block frame_entry;
} ae_shared_pool;
 
ae_int_t ae_misalignment(const void *ptr, size_t alignment);
void* ae_align(void *ptr, size_t alignment);
void* aligned_malloc(size_t size, size_t alignment);
void  aligned_free(void *block);

void* ae_malloc(size_t size, ae_state *state);
void  ae_free(void *p);
ae_int_t ae_sizeof(ae_datatype datatype);
void ae_touch_ptr(void *p);

void ae_state_init(ae_state *state);
void ae_state_clear(ae_state *state);
#ifndef AE_USE_CPP_ERROR_HANDLING
void ae_state_set_break_jump(ae_state *state, jmp_buf *buf);
#endif
void ae_break(ae_state *state, ae_error_type error_type, const char *msg);

void ae_frame_make(ae_state *state, ae_frame *tmp);
void ae_frame_leave(ae_state *state);

void ae_db_attach(ae_dyn_block *block, ae_state *state);
ae_bool ae_db_malloc(ae_dyn_block *block, ae_int_t size, ae_state *state, ae_bool make_automatic);
ae_bool ae_db_realloc(ae_dyn_block *block, ae_int_t size, ae_state *state);
void ae_db_free(ae_dyn_block *block);
void ae_db_swap(ae_dyn_block *block1, ae_dyn_block *block2);

void ae_vector_init(ae_vector *dst, ae_int_t size, ae_datatype datatype, ae_state *state);
void ae_vector_init_copy(ae_vector *dst, ae_vector *src, ae_state *state);
void ae_vector_init_from_x(ae_vector *dst, x_vector *src, ae_state *state);
void ae_vector_attach_to_x(ae_vector *dst, x_vector *src, ae_state *state);
ae_bool ae_vector_set_length(ae_vector *dst, ae_int_t newsize, ae_state *state);
void ae_vector_clear(ae_vector *dst);
void ae_vector_destroy(ae_vector *dst);
void ae_swap_vectors(ae_vector *vec1, ae_vector *vec2);

void ae_matrix_init(ae_matrix *dst, ae_int_t rows, ae_int_t cols, ae_datatype datatype, ae_state *state);
void ae_matrix_init_copy(ae_matrix *dst, ae_matrix *src, ae_state *state);
void ae_matrix_init_from_x(ae_matrix *dst, x_matrix *src, ae_state *state);
void ae_matrix_attach_to_x(ae_matrix *dst, x_matrix *src, ae_state *state);
ae_bool ae_matrix_set_length(ae_matrix *dst, ae_int_t rows, ae_int_t cols, ae_state *state);
void ae_matrix_clear(ae_matrix *dst);
void ae_matrix_destroy(ae_matrix *dst);
void ae_swap_matrices(ae_matrix *mat1, ae_matrix *mat2);

void ae_smart_ptr_init(ae_smart_ptr *dst, void **subscriber, ae_state *state);
void ae_smart_ptr_clear(void *_dst); /* accepts ae_smart_ptr* */
void ae_smart_ptr_destroy(void *_dst);
void ae_smart_ptr_assign(ae_smart_ptr *dst, void *new_ptr, ae_bool is_owner, ae_bool is_dynamic, void (*destroy)(void*));
void ae_smart_ptr_release(ae_smart_ptr *dst);

void ae_yield();
void ae_init_lock(ae_lock *lock);
void ae_acquire_lock(ae_lock *lock);
void ae_release_lock(ae_lock *lock);
void ae_free_lock(ae_lock *lock);

void ae_shared_pool_init(void *_dst, ae_state *state);
void ae_shared_pool_init_copy(void *_dst, void *_src, ae_state *state);
void ae_shared_pool_clear(void *dst);
void ae_shared_pool_destroy(void *dst);
ae_bool ae_shared_pool_is_initialized(void *_dst);
void ae_shared_pool_set_seed(
    ae_shared_pool  *dst,
    void            *seed_object,
    ae_int_t        size_of_object,
    void            (*init)(void* dst, ae_state* state),
    void            (*init_copy)(void* dst, void* src, ae_state* state),
    void            (*destroy)(void* ptr),
    ae_state        *state);
void ae_shared_pool_retrieve(
    ae_shared_pool  *pool,
    ae_smart_ptr    *pptr,
    ae_state        *state);
void ae_shared_pool_recycle(
    ae_shared_pool  *pool,
    ae_smart_ptr    *pptr,
    ae_state        *state);
void ae_shared_pool_clear_recycled(
    ae_shared_pool  *pool,
    ae_state        *state);
void ae_shared_pool_first_recycled(
    ae_shared_pool  *pool,
    ae_smart_ptr    *pptr,
    ae_state        *state);
void ae_shared_pool_next_recycled(
    ae_shared_pool  *pool,
    ae_smart_ptr    *pptr,
    ae_state        *state);
void ae_shared_pool_reset(
    ae_shared_pool  *pool,
    ae_state        *state);

void ae_x_set_vector(x_vector *dst, ae_vector *src, ae_state *state);
void ae_x_set_matrix(x_matrix *dst, ae_matrix *src, ae_state *state);
void ae_x_attach_to_vector(x_vector *dst, ae_vector *src);
void ae_x_attach_to_matrix(x_matrix *dst, ae_matrix *src);

void x_vector_clear(x_vector *dst);

ae_bool x_is_symmetric(x_matrix *a);
ae_bool x_is_hermitian(x_matrix *a);
ae_bool x_force_symmetric(x_matrix *a);
ae_bool x_force_hermitian(x_matrix *a);
ae_bool ae_is_symmetric(ae_matrix *a);
ae_bool ae_is_hermitian(ae_matrix *a);
ae_bool ae_force_symmetric(ae_matrix *a);
ae_bool ae_force_hermitian(ae_matrix *a);

void ae_serializer_init(ae_serializer *serializer);
void ae_serializer_clear(ae_serializer *serializer);

void ae_serializer_alloc_start(ae_serializer *serializer);
void ae_serializer_alloc_entry(ae_serializer *serializer);
ae_int_t ae_serializer_get_alloc_size(ae_serializer *serializer);

#ifdef AE_USE_CPP_SERIALIZATION
void ae_serializer_sstart_str(ae_serializer *serializer, std::string *buf);
void ae_serializer_ustart_str(ae_serializer *serializer, const std::string *buf);
void ae_serializer_sstart_stream(ae_serializer *serializer, std::ostream *stream);
void ae_serializer_ustart_stream(ae_serializer *serializer, const std::istream *stream);
#endif
void ae_serializer_sstart_str(ae_serializer *serializer, char *buf);
void ae_serializer_ustart_str(ae_serializer *serializer, const char *buf);
void ae_serializer_sstart_stream(ae_serializer *serializer, ae_stream_writer writer, ae_int_t aux);
void ae_serializer_ustart_stream(ae_serializer *serializer, ae_stream_reader reader, ae_int_t aux);

void ae_serializer_serialize_bool(ae_serializer *serializer, ae_bool v, ae_state *state);
void ae_serializer_serialize_int(ae_serializer *serializer, ae_int_t v, ae_state *state);
void ae_serializer_serialize_double(ae_serializer *serializer, double v, ae_state *state);
void ae_serializer_unserialize_bool(ae_serializer *serializer, ae_bool *v, ae_state *state);
void ae_serializer_unserialize_int(ae_serializer *serializer, ae_int_t *v, ae_state *state);
void ae_serializer_unserialize_double(ae_serializer *serializer, double *v, ae_state *state);

void ae_serializer_stop(ae_serializer *serializer, ae_state *state);

/************************************************************************
Service functions
************************************************************************/
void ae_assert(ae_bool cond, const char *msg, ae_state *state);
ae_int_t ae_cpuid();

/************************************************************************
Real math functions:
* IEEE-compliant floating point comparisons
* standard functions
************************************************************************/
ae_bool ae_fp_eq(double v1, double v2);
ae_bool ae_fp_neq(double v1, double v2);
ae_bool ae_fp_less(double v1, double v2);
ae_bool ae_fp_less_eq(double v1, double v2);
ae_bool ae_fp_greater(double v1, double v2);
ae_bool ae_fp_greater_eq(double v1, double v2);

ae_bool ae_isfinite_stateless(double x, ae_int_t endianness);
ae_bool ae_isnan_stateless(double x,    ae_int_t endianness);
ae_bool ae_isinf_stateless(double x,    ae_int_t endianness);
ae_bool ae_isposinf_stateless(double x, ae_int_t endianness);
ae_bool ae_isneginf_stateless(double x, ae_int_t endianness);

ae_int_t ae_get_endianness();

ae_bool ae_isfinite(double x,ae_state *state);
ae_bool ae_isnan(double x,   ae_state *state);
ae_bool ae_isinf(double x,   ae_state *state);
ae_bool ae_isposinf(double x,ae_state *state);
ae_bool ae_isneginf(double x,ae_state *state);

double   ae_fabs(double x,   ae_state *state);
ae_int_t ae_iabs(ae_int_t x, ae_state *state);
double   ae_sqr(double x,    ae_state *state);
double   ae_sqrt(double x,   ae_state *state);

ae_int_t ae_sign(double x,   ae_state *state);
ae_int_t ae_round(double x,  ae_state *state);
ae_int_t ae_trunc(double x,  ae_state *state);
ae_int_t ae_ifloor(double x, ae_state *state);
ae_int_t ae_iceil(double x,  ae_state *state);

ae_int_t ae_maxint(ae_int_t m1, ae_int_t m2, ae_state *state);
ae_int_t ae_minint(ae_int_t m1, ae_int_t m2, ae_state *state);
double   ae_maxreal(double m1, double m2, ae_state *state);
double   ae_minreal(double m1, double m2, ae_state *state);
double   ae_randomreal(ae_state *state);
ae_int_t ae_randominteger(ae_int_t maxv, ae_state *state);

double   ae_sin(double x, ae_state *state);
double   ae_cos(double x, ae_state *state);
double   ae_tan(double x, ae_state *state);
double   ae_sinh(double x, ae_state *state);
double   ae_cosh(double x, ae_state *state);
double   ae_tanh(double x, ae_state *state);
double   ae_asin(double x, ae_state *state);
double   ae_acos(double x, ae_state *state);
double   ae_atan(double x, ae_state *state);
double   ae_atan2(double y, double x, ae_state *state);

double   ae_log(double x, ae_state *state);
double   ae_pow(double x, double y, ae_state *state);
double   ae_exp(double x, ae_state *state);

/************************************************************************
Complex math functions:
* basic arithmetic operations
* standard functions
************************************************************************/
ae_complex ae_complex_from_i(ae_int_t v);
ae_complex ae_complex_from_d(double v);

ae_complex ae_c_neg(ae_complex lhs);
ae_bool ae_c_eq(ae_complex lhs,       ae_complex rhs);
ae_bool ae_c_neq(ae_complex lhs,      ae_complex rhs);
ae_complex ae_c_add(ae_complex lhs,   ae_complex rhs);
ae_complex ae_c_mul(ae_complex lhs,   ae_complex rhs);
ae_complex ae_c_sub(ae_complex lhs,   ae_complex rhs);
ae_complex ae_c_div(ae_complex lhs,   ae_complex rhs);
ae_bool ae_c_eq_d(ae_complex lhs,     double rhs);
ae_bool ae_c_neq_d(ae_complex lhs,    double rhs);
ae_complex ae_c_add_d(ae_complex lhs, double rhs);
ae_complex ae_c_mul_d(ae_complex lhs, double rhs);
ae_complex ae_c_sub_d(ae_complex lhs, double rhs);
ae_complex ae_c_d_sub(double lhs,     ae_complex rhs);
ae_complex ae_c_div_d(ae_complex lhs, double rhs);
ae_complex ae_c_d_div(double lhs,   ae_complex rhs);

ae_complex ae_c_conj(ae_complex lhs, ae_state *state);
ae_complex ae_c_sqr(ae_complex lhs, ae_state *state);
double     ae_c_abs(ae_complex z, ae_state *state);

/************************************************************************
Complex BLAS operations
************************************************************************/
ae_complex ae_v_cdotproduct(const ae_complex *v0, ae_int_t stride0, const char *conj0, const ae_complex *v1, ae_int_t stride1, const char *conj1, ae_int_t n);
void ae_v_cmove(ae_complex *vdst,    ae_int_t stride_dst, const ae_complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void ae_v_cmoveneg(ae_complex *vdst, ae_int_t stride_dst, const ae_complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void ae_v_cmoved(ae_complex *vdst,   ae_int_t stride_dst, const ae_complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, double alpha);
void ae_v_cmovec(ae_complex *vdst,   ae_int_t stride_dst, const ae_complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, ae_complex alpha);
void ae_v_cadd(ae_complex *vdst,     ae_int_t stride_dst, const ae_complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void ae_v_caddd(ae_complex *vdst,    ae_int_t stride_dst, const ae_complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, double alpha);
void ae_v_caddc(ae_complex *vdst,    ae_int_t stride_dst, const ae_complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, ae_complex alpha);
void ae_v_csub(ae_complex *vdst,     ae_int_t stride_dst, const ae_complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void ae_v_csubd(ae_complex *vdst, ae_int_t stride_dst, const ae_complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, double alpha);
void ae_v_csubc(ae_complex *vdst, ae_int_t stride_dst, const ae_complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, ae_complex alpha);
void ae_v_cmuld(ae_complex *vdst, ae_int_t stride_dst, ae_int_t n, double alpha);
void ae_v_cmulc(ae_complex *vdst, ae_int_t stride_dst, ae_int_t n, ae_complex alpha);

/************************************************************************
Real BLAS operations
************************************************************************/
double ae_v_dotproduct(const double *v0, ae_int_t stride0, const double *v1, ae_int_t stride1, ae_int_t n);
void ae_v_move(double *vdst,    ae_int_t stride_dst, const double* vsrc,  ae_int_t stride_src, ae_int_t n);
void ae_v_moveneg(double *vdst, ae_int_t stride_dst, const double* vsrc,  ae_int_t stride_src, ae_int_t n);
void ae_v_moved(double *vdst,   ae_int_t stride_dst, const double* vsrc,  ae_int_t stride_src, ae_int_t n, double alpha);
void ae_v_add(double *vdst,     ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n);
void ae_v_addd(double *vdst,    ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n, double alpha);
void ae_v_sub(double *vdst,     ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n);
void ae_v_subd(double *vdst,    ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n, double alpha);
void ae_v_muld(double *vdst,  ae_int_t stride_dst, ae_int_t n, double alpha);

/************************************************************************
Other functions
************************************************************************/
ae_int_t ae_v_len(ae_int_t a, ae_int_t b);

/*
extern const double ae_machineepsilon;
extern const double ae_maxrealnumber;
extern const double ae_minrealnumber;
extern const double ae_pi;
*/
#define ae_machineepsilon 5E-16
#define ae_maxrealnumber  1E300
#define ae_minrealnumber  1E-300
#define ae_pi 3.1415926535897932384626433832795


/************************************************************************
RComm functions
************************************************************************/
typedef struct rcommstate
{
    int stage;
    ae_vector ia;
    ae_vector ba;
    ae_vector ra;
    ae_vector ca;
} rcommstate;
void _rcommstate_init(rcommstate* p, ae_state *_state);
void _rcommstate_init_copy(rcommstate* dst, rcommstate* src, ae_state *_state);
void _rcommstate_clear(rcommstate* p);
void _rcommstate_destroy(rcommstate* p);

/************************************************************************
Allocation counter, inactive by default.
Turned on when needed for debugging purposes.
************************************************************************/
extern ae_int64_t _alloc_counter;
extern ae_bool    _use_alloc_counter;


/************************************************************************
debug functions (must be turned on by preprocessor definitions):
* tickcount(), which is wrapper around GetTickCount()
* flushconsole(), fluches console
* ae_debugrng(), returns random number generated with high-quality random numbers generator
* ae_set_seed(), sets seed of the debug RNG (NON-THREAD-SAFE!!!)
* ae_get_seed(), returns two seed values of the debug RNG (NON-THREAD-SAFE!!!)
************************************************************************/
#ifdef AE_DEBUG4WINDOWS
#define flushconsole(s) fflush(stdout)
#define tickcount(s) _tickcount()
int _tickcount();
#endif
#ifdef AE_DEBUG4POSIX
#define flushconsole(s) fflush(stdout)
#define tickcount(s) _tickcount()
int _tickcount();
#endif


}


/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS DECLARATIONS FOR C++ RELATED FUNCTIONALITY
//
/////////////////////////////////////////////////////////////////////////

namespace alglib
{

typedef alglib_impl::ae_int_t ae_int_t;

/********************************************************************
Class forwards
********************************************************************/
class complex;

ae_int_t vlen(ae_int_t n1, ae_int_t n2);

/********************************************************************
Exception class.
********************************************************************/
class ap_error
{
public:
    std::string msg;
    
    ap_error();
    ap_error(const char *s);
    static void make_assertion(bool bClause);
    static void make_assertion(bool bClause, const char *p_msg);
private:
};

/********************************************************************
Complex number with double precision.
********************************************************************/
class complex
{
public:
    complex();
    complex(const double &_x);
    complex(const double &_x, const double &_y);
    complex(const complex &z);

    complex& operator= (const double& v);
    complex& operator+=(const double& v);
    complex& operator-=(const double& v);
    complex& operator*=(const double& v);
    complex& operator/=(const double& v);

    complex& operator= (const complex& z);
    complex& operator+=(const complex& z);
    complex& operator-=(const complex& z);
    complex& operator*=(const complex& z);
    complex& operator/=(const complex& z);

    alglib_impl::ae_complex*       c_ptr();
    const alglib_impl::ae_complex* c_ptr() const;
    
    std::string tostring(int dps) const;

    double x, y;
};

const alglib::complex operator/(const alglib::complex& lhs, const alglib::complex& rhs);
const bool operator==(const alglib::complex& lhs, const alglib::complex& rhs);
const bool operator!=(const alglib::complex& lhs, const alglib::complex& rhs);
const alglib::complex operator+(const alglib::complex& lhs);
const alglib::complex operator-(const alglib::complex& lhs);
const alglib::complex operator+(const alglib::complex& lhs, const alglib::complex& rhs);
const alglib::complex operator+(const alglib::complex& lhs, const double& rhs);
const alglib::complex operator+(const double& lhs, const alglib::complex& rhs);
const alglib::complex operator-(const alglib::complex& lhs, const alglib::complex& rhs);
const alglib::complex operator-(const alglib::complex& lhs, const double& rhs);
const alglib::complex operator-(const double& lhs, const alglib::complex& rhs);
const alglib::complex operator*(const alglib::complex& lhs, const alglib::complex& rhs);
const alglib::complex operator*(const alglib::complex& lhs, const double& rhs);
const alglib::complex operator*(const double& lhs, const alglib::complex& rhs);
const alglib::complex operator/(const alglib::complex& lhs, const alglib::complex& rhs);
const alglib::complex operator/(const double& lhs, const alglib::complex& rhs);
const alglib::complex operator/(const alglib::complex& lhs, const double& rhs);
double abscomplex(const alglib::complex &z);
alglib::complex conj(const alglib::complex &z);
alglib::complex csqr(const alglib::complex &z);
void setnworkers(alglib::ae_int_t nworkers);

/********************************************************************
Level 1 BLAS functions

NOTES:
* destination and source should NOT overlap
* stride is assumed to be positive, but it is not 
  assert'ed within function
* conj_src parameter specifies whether complex source is conjugated 
  before processing or not. Pass string which starts with 'N' or 'n'
  ("No conj", for example) to use unmodified parameter. All other
  values will result in conjugation of input, but it is recommended
  to use "Conj" in such cases.
********************************************************************/
double vdotproduct(const double *v0, ae_int_t stride0, const double *v1, ae_int_t stride1, ae_int_t n);
double vdotproduct(const double *v1, const double *v2, ae_int_t N);

alglib::complex vdotproduct(const alglib::complex *v0, ae_int_t stride0, const char *conj0, const alglib::complex *v1, ae_int_t stride1, const char *conj1, ae_int_t n);
alglib::complex vdotproduct(const alglib::complex *v1, const alglib::complex *v2, ae_int_t N);

void vmove(double *vdst,  ae_int_t stride_dst, const double* vsrc,  ae_int_t stride_src, ae_int_t n);
void vmove(double *vdst, const double* vsrc, ae_int_t N);

void vmove(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void vmove(alglib::complex *vdst, const alglib::complex* vsrc, ae_int_t N);

void vmoveneg(double *vdst,  ae_int_t stride_dst, const double* vsrc,  ae_int_t stride_src, ae_int_t n);
void vmoveneg(double *vdst, const double *vsrc, ae_int_t N);

void vmoveneg(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void vmoveneg(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N);

void vmove(double *vdst,  ae_int_t stride_dst, const double* vsrc,  ae_int_t stride_src, ae_int_t n, double alpha);
void vmove(double *vdst, const double *vsrc, ae_int_t N, double alpha);

void vmove(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, double alpha);
void vmove(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N, double alpha);

void vmove(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex* vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, alglib::complex alpha);
void vmove(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N, alglib::complex alpha);

void vadd(double *vdst,  ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n);
void vadd(double *vdst, const double *vsrc, ae_int_t N);

void vadd(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void vadd(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N);

void vadd(double *vdst,  ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n, double alpha);
void vadd(double *vdst, const double *vsrc, ae_int_t N, double alpha);

void vadd(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, double alpha);
void vadd(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N, double alpha);

void vadd(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, alglib::complex alpha);
void vadd(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N, alglib::complex alpha);

void vsub(double *vdst,  ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n);
void vsub(double *vdst, const double *vsrc, ae_int_t N);

void vsub(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n);
void vsub(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N);

void vsub(double *vdst,  ae_int_t stride_dst, const double *vsrc,  ae_int_t stride_src, ae_int_t n, double alpha);
void vsub(double *vdst, const double *vsrc, ae_int_t N, double alpha);

void vsub(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, double alpha);
void vsub(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N, double alpha);

void vsub(alglib::complex *vdst, ae_int_t stride_dst, const alglib::complex *vsrc, ae_int_t stride_src, const char *conj_src, ae_int_t n, alglib::complex alpha);
void vsub(alglib::complex *vdst, const alglib::complex *vsrc, ae_int_t N, alglib::complex alpha);

void vmul(double *vdst,  ae_int_t stride_dst, ae_int_t n, double alpha);
void vmul(double *vdst, ae_int_t N, double alpha);

void vmul(alglib::complex *vdst, ae_int_t stride_dst, ae_int_t n, double alpha);
void vmul(alglib::complex *vdst, ae_int_t N, double alpha);

void vmul(alglib::complex *vdst, ae_int_t stride_dst, ae_int_t n, alglib::complex alpha);
void vmul(alglib::complex *vdst, ae_int_t N, alglib::complex alpha);



/********************************************************************
string conversion functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
********************************************************************/

/********************************************************************
1- and 2-dimensional arrays
********************************************************************/
class ae_vector_wrapper
{
public:
    ae_vector_wrapper();
    virtual ~ae_vector_wrapper();

    void setlength(ae_int_t iLen);
    ae_int_t length() const;

    void attach_to(alglib_impl::ae_vector *ptr);
    void allocate_own(ae_int_t size, alglib_impl::ae_datatype datatype);
    const alglib_impl::ae_vector* c_ptr() const;
    alglib_impl::ae_vector* c_ptr();
private:
    ae_vector_wrapper(const ae_vector_wrapper &rhs);
    const ae_vector_wrapper& operator=(const ae_vector_wrapper &rhs);
protected:
    //
    // Copies source vector RHS into current object.
    //
    // Current object is considered empty (this function should be
    // called from copy constructor).
    //
    void create(const ae_vector_wrapper &rhs);
    
    //
    // Copies array given by string into current object. Additional
    // parameter DATATYPE contains information about type of the data
    // in S and type of the array to create.
    //
    // Current object is considered empty (this function should be
    // called from copy constructor).
    //
    void create(const char *s, alglib_impl::ae_datatype datatype);
    
    //
    // Assigns RHS to current object.
    //
    // It has several branches depending on target object status:
    // * in case it is proxy object, data are copied into memory pointed by
    //   proxy. Function checks that source has exactly same size as target
    //   (exception is thrown on failure).
    // * in case it is non-proxy object, data allocated by object are cleared
    //   and a copy of RHS is created in target.
    //
    // NOTE: this function correctly handles assignments of the object to itself.
    //
    void assign(const ae_vector_wrapper &rhs);
    
    alglib_impl::ae_vector *p_vec;
    alglib_impl::ae_vector vec;
};

class boolean_1d_array : public ae_vector_wrapper
{
public:
    boolean_1d_array();
    boolean_1d_array(const char *s);
    boolean_1d_array(const boolean_1d_array &rhs);
    boolean_1d_array(alglib_impl::ae_vector *p);
    const boolean_1d_array& operator=(const boolean_1d_array &rhs);
    virtual ~boolean_1d_array() ;

    const ae_bool& operator()(ae_int_t i) const;
    ae_bool& operator()(ae_int_t i);

    const ae_bool& operator[](ae_int_t i) const;
    ae_bool& operator[](ae_int_t i);

    void setcontent(ae_int_t iLen, const bool *pContent );
    ae_bool* getcontent();
    const ae_bool* getcontent() const;

    std::string tostring() const;
};

class integer_1d_array : public ae_vector_wrapper
{
public:
    integer_1d_array();
    integer_1d_array(const char *s);
    integer_1d_array(const integer_1d_array &rhs);
    integer_1d_array(alglib_impl::ae_vector *p);
    const integer_1d_array& operator=(const integer_1d_array &rhs);
    virtual ~integer_1d_array();

    const ae_int_t& operator()(ae_int_t i) const;
    ae_int_t& operator()(ae_int_t i);

    const ae_int_t& operator[](ae_int_t i) const;
    ae_int_t& operator[](ae_int_t i);

    void setcontent(ae_int_t iLen, const ae_int_t *pContent );

    ae_int_t* getcontent();
    const ae_int_t* getcontent() const;

    std::string tostring() const;
};

class real_1d_array : public ae_vector_wrapper
{
public:
    real_1d_array();
    real_1d_array(const char *s);
    real_1d_array(const real_1d_array &rhs);
    real_1d_array(alglib_impl::ae_vector *p);
    const real_1d_array& operator=(const real_1d_array &rhs);
    virtual ~real_1d_array();

    const double& operator()(ae_int_t i) const;
    double& operator()(ae_int_t i);

    const double& operator[](ae_int_t i) const;
    double& operator[](ae_int_t i);

    void setcontent(ae_int_t iLen, const double *pContent );
    double* getcontent();
    const double* getcontent() const;

    std::string tostring(int dps) const;
};

class complex_1d_array : public ae_vector_wrapper
{
public:
    complex_1d_array();
    complex_1d_array(const char *s);
    complex_1d_array(const complex_1d_array &rhs);
    complex_1d_array(alglib_impl::ae_vector *p);
    const complex_1d_array& operator=(const complex_1d_array &rhs);
    virtual ~complex_1d_array();

    const alglib::complex& operator()(ae_int_t i) const;
    alglib::complex& operator()(ae_int_t i);

    const alglib::complex& operator[](ae_int_t i) const;
    alglib::complex& operator[](ae_int_t i);

    void setcontent(ae_int_t iLen, const alglib::complex *pContent );
    alglib::complex* getcontent();
    const alglib::complex* getcontent() const;

    std::string tostring(int dps) const;
};

class ae_matrix_wrapper
{
public:
    ae_matrix_wrapper();
    virtual ~ae_matrix_wrapper();
    const ae_matrix_wrapper& operator=(const ae_matrix_wrapper &rhs);

    void setlength(ae_int_t rows, ae_int_t cols);
    ae_int_t rows() const;
    ae_int_t cols() const;
    bool isempty() const;
	ae_int_t getstride() const;

    void attach_to(alglib_impl::ae_matrix *ptr);
    void allocate_own(ae_int_t rows, ae_int_t cols, alglib_impl::ae_datatype datatype);
    const alglib_impl::ae_matrix* c_ptr() const;
    alglib_impl::ae_matrix* c_ptr();
private:
    ae_matrix_wrapper(const ae_matrix_wrapper &rhs);
protected:
    //
    // Copies source matrix RHS into current object.
    //
    // Current object is considered empty (this function should be
    // called from copy constructor).
    //
    void create(const ae_matrix_wrapper &rhs);
    
    //
    // Copies array given by string into current object. Additional
    // parameter DATATYPE contains information about type of the data
    // in S and type of the array to create.
    //
    // Current object is considered empty (this function should be
    // called from copy constructor).
    //
    void create(const char *s, alglib_impl::ae_datatype datatype);
    
    //
    // Assigns RHS to current object.
    //
    // It has several branches depending on target object status:
    // * in case it is proxy object, data are copied into memory pointed by
    //   proxy. Function checks that source has exactly same size as target
    //   (exception is thrown on failure).
    // * in case it is non-proxy object, data allocated by object are cleared
    //   and a copy of RHS is created in target.
    //
    // NOTE: this function correctly handles assignments of the object to itself.
    //
    void assign(const ae_matrix_wrapper &rhs);
    
    alglib_impl::ae_matrix *p_mat;
    alglib_impl::ae_matrix mat;
};

class boolean_2d_array : public ae_matrix_wrapper
{
public:
    boolean_2d_array();
    boolean_2d_array(const boolean_2d_array &rhs);
    boolean_2d_array(alglib_impl::ae_matrix *p);
    boolean_2d_array(const char *s);
    virtual ~boolean_2d_array();

    const ae_bool& operator()(ae_int_t i, ae_int_t j) const;
    ae_bool& operator()(ae_int_t i, ae_int_t j);

    const ae_bool* operator[](ae_int_t i) const;
    ae_bool* operator[](ae_int_t i);
    
    void setcontent(ae_int_t irows, ae_int_t icols, const bool *pContent );
    
    std::string tostring() const ;
};

class integer_2d_array : public ae_matrix_wrapper
{
public:
    integer_2d_array();
    integer_2d_array(const integer_2d_array &rhs);
    integer_2d_array(alglib_impl::ae_matrix *p);
    integer_2d_array(const char *s);
    virtual ~integer_2d_array();

    const ae_int_t& operator()(ae_int_t i, ae_int_t j) const;
    ae_int_t& operator()(ae_int_t i, ae_int_t j);

    const ae_int_t* operator[](ae_int_t i) const;
    ae_int_t* operator[](ae_int_t i);

    void setcontent(ae_int_t irows, ae_int_t icols, const ae_int_t *pContent );
    
    std::string tostring() const;
};

class real_2d_array : public ae_matrix_wrapper
{
public:
    real_2d_array();
    real_2d_array(const real_2d_array &rhs);
    real_2d_array(alglib_impl::ae_matrix *p);
    real_2d_array(const char *s);
    virtual ~real_2d_array();

    const double& operator()(ae_int_t i, ae_int_t j) const;
    double& operator()(ae_int_t i, ae_int_t j);

    const double* operator[](ae_int_t i) const;
    double* operator[](ae_int_t i);

    void setcontent(ae_int_t irows, ae_int_t icols, const double *pContent );

    std::string tostring(int dps) const;
};

class complex_2d_array : public ae_matrix_wrapper
{
public:
    complex_2d_array();
    complex_2d_array(const complex_2d_array &rhs);
    complex_2d_array(alglib_impl::ae_matrix *p);
    complex_2d_array(const char *s);
    virtual ~complex_2d_array();

    const alglib::complex& operator()(ae_int_t i, ae_int_t j) const;
    alglib::complex& operator()(ae_int_t i, ae_int_t j);

    const alglib::complex* operator[](ae_int_t i) const;
    alglib::complex* operator[](ae_int_t i);

    void setcontent(ae_int_t irows, ae_int_t icols, const alglib::complex *pContent );

    std::string tostring(int dps) const;
};

/********************************************************************
CSV operations: reading CSV file to real matrix.

This function reads CSV  file  and  stores  its  contents  to  double
precision 2D array. Format of the data file must conform to RFC  4180
specification, with additional notes:
* file size should be less than 2GB
* ASCI encoding, UTF-8 without BOM (in header names) are supported
* any character (comma/tab/space) may be used as field separator,  as
  long as it is distinct from one used for decimal point
* multiple subsequent field separators (say, two  spaces) are treated
  as MULTIPLE separators, not one big separator
* both comma and full stop may be used as decimal point. Parser  will
  automatically determine specific character being used.  Both  fixed
  and exponential number formats are  allowed.   Thousand  separators
  are NOT allowed.
* line may end with \n (Unix style) or \r\n (Windows  style),  parser
  will automatically adapt to chosen convention
* escaped fields (ones in double quotes) are not supported

INPUT PARAMETERS:
    filename        relative/absolute path
    separator       character used to separate fields.  May  be  ' ',
                    ',', '\t'. Other separators are possible too.
    flags           several values combined with bitwise OR:
                    * alglib::CSV_SKIP_HEADERS -  if present, first row
                      contains headers  and  will  be  skipped.   Its
                      contents is used to determine fields count, and
                      that's all.
                    If no flags are specified, default value 0x0  (or
                    alglib::CSV_DEFAULT, which is same) should be used.
                    
OUTPUT PARAMETERS:
    out             2D matrix, CSV file parsed with atof()
    
HANDLING OF SPECIAL CASES:
* file does not exist - alglib::ap_error exception is thrown
* empty file - empty array is returned (no exception)
* skip_first_row=true, only one row in file - empty array is returned
* field contents is not recognized by atof() - field value is replaced
  by 0.0
********************************************************************/
void read_csv(const char *filename, char separator, int flags, alglib::real_2d_array &out);


/********************************************************************
dataset information.

can store regression dataset, classification dataset, or non-labeled
task:
* nout==0 means non-labeled task (clustering, for example)
* nout>0 && nclasses==0 means regression task
* nout>0 && nclasses>0 means classification task
********************************************************************/
/*class dataset
{
public:
    dataset():nin(0), nout(0), nclasses(0), trnsize(0), valsize(0), tstsize(0), totalsize(0){};

    int nin, nout, nclasses;

    int trnsize;
    int valsize;
    int tstsize;
    int totalsize;

    alglib::real_2d_array trn;
    alglib::real_2d_array val;
    alglib::real_2d_array tst;
    alglib::real_2d_array all;
};

bool opendataset(std::string file, dataset *pdataset);

//
// internal functions
//
std::string strtolower(const std::string &s);
bool readstrings(std::string file, std::list<std::string> *pOutput);
bool readstrings(std::string file, std::list<std::string> *pOutput, std::string comment);
void explodestring(std::string s, char sep, std::vector<std::string> *pOutput);
std::string xtrim(std::string s);*/

/********************************************************************
Constants and functions introduced for compatibility with AlgoPascal
********************************************************************/
extern const double machineepsilon;
extern const double maxrealnumber;
extern const double minrealnumber;
extern const double fp_nan;
extern const double fp_posinf;
extern const double fp_neginf;
extern const ae_int_t endianness;
static const int CSV_DEFAULT = 0x0;
static const int CSV_SKIP_HEADERS = 0x1;

int sign(double x);
double randomreal();
ae_int_t randominteger(ae_int_t maxv);
int round(double x);
int trunc(double x);
int ifloor(double x);
int iceil(double x);
double pi();
double sqr(double x);
int maxint(int m1, int m2);
int minint(int m1, int m2);
double maxreal(double m1, double m2);
double minreal(double m1, double m2);

bool fp_eq(double v1, double v2);
bool fp_neq(double v1, double v2);
bool fp_less(double v1, double v2);
bool fp_less_eq(double v1, double v2);
bool fp_greater(double v1, double v2);
bool fp_greater_eq(double v1, double v2);

bool fp_isnan(double x);
bool fp_isposinf(double x);
bool fp_isneginf(double x);
bool fp_isinf(double x);
bool fp_isfinite(double x);


}//namespace alglib


/////////////////////////////////////////////////////////////////////////
//
// THIS SECTIONS CONTAINS DECLARATIONS FOR OPTIMIZED LINEAR ALGEBRA CODES
// IT IS SHARED BETWEEN C++ AND PURE C LIBRARIES
//
/////////////////////////////////////////////////////////////////////////

namespace alglib_impl
{
#define ALGLIB_INTERCEPTS_ABLAS
void _ialglib_vzero(ae_int_t n, double *p, ae_int_t stride);
void _ialglib_vzero_complex(ae_int_t n, ae_complex *p, ae_int_t stride);
void _ialglib_vcopy(ae_int_t n, const double *a, ae_int_t stridea, double *b, ae_int_t strideb);
void _ialglib_vcopy_complex(ae_int_t n, const ae_complex *a, ae_int_t stridea, double *b, ae_int_t strideb, const char *conj);
void _ialglib_vcopy_dcomplex(ae_int_t n, const double *a, ae_int_t stridea, double *b, ae_int_t strideb, const char *conj);
void _ialglib_mcopyblock(ae_int_t m, ae_int_t n, const double *a, ae_int_t op, ae_int_t stride, double *b);
void _ialglib_mcopyunblock(ae_int_t m, ae_int_t n, const double *a, ae_int_t op, double *b, ae_int_t stride);
void _ialglib_mcopyblock_complex(ae_int_t m, ae_int_t n, const ae_complex *a, ae_int_t op, ae_int_t stride, double *b);
void _ialglib_mcopyunblock_complex(ae_int_t m, ae_int_t n, const double *a, ae_int_t op, ae_complex* b, ae_int_t stride);

ae_bool _ialglib_i_rmatrixgemmf(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     ae_matrix *a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     ae_matrix *b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     double beta,
     ae_matrix *c,
     ae_int_t ic,
     ae_int_t jc);
ae_bool _ialglib_i_cmatrixgemmf(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     ae_complex alpha,
     ae_matrix *a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     ae_matrix *b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     ae_complex beta,
     ae_matrix *c,
     ae_int_t ic,
     ae_int_t jc);
ae_bool _ialglib_i_cmatrixrighttrsmf(ae_int_t m,
     ae_int_t n,
     ae_matrix *a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix *x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_rmatrixrighttrsmf(ae_int_t m,
     ae_int_t n,
     ae_matrix *a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix *x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_cmatrixlefttrsmf(ae_int_t m,
     ae_int_t n,
     ae_matrix *a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix *x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_rmatrixlefttrsmf(ae_int_t m,
     ae_int_t n,
     ae_matrix *a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix *x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_cmatrixherkf(ae_int_t n,
     ae_int_t k,
     double alpha,
     ae_matrix *a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     ae_matrix *c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper);
ae_bool _ialglib_i_rmatrixsyrkf(ae_int_t n,
     ae_int_t k,
     double alpha,
     ae_matrix *a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     ae_matrix *c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper);
ae_bool _ialglib_i_cmatrixrank1f(ae_int_t m,
     ae_int_t n,
     ae_matrix *a,
     ae_int_t ia,
     ae_int_t ja,
     ae_vector *u,
     ae_int_t uoffs,
     ae_vector *v,
     ae_int_t voffs);
ae_bool _ialglib_i_rmatrixrank1f(ae_int_t m,
     ae_int_t n,
     ae_matrix *a,
     ae_int_t ia,
     ae_int_t ja,
     ae_vector *u,
     ae_int_t uoffs,
     ae_vector *v,
     ae_int_t voffs);



}


/////////////////////////////////////////////////////////////////////////
//
// THIS SECTION CONTAINS PARALLEL SUBROUTINES
//
/////////////////////////////////////////////////////////////////////////


namespace alglib_impl
{
#define AE_HPC
#ifdef AE_HAS_SSE2_INTRINSICS
#define ALGLIB_INTERCEPTS_SSE2
#endif
#ifdef AE_MKL
#define ALGLIB_INTERCEPTS_MKL
#endif
#define AE_SMP_MAXPARAMS 32


/*************************************************************************
Event.

This structure provides OS-independent event:
* under Windows/Posix systems it uses system-provided event
* under Boost it uses OS-independent event provided by Boost package
* when no OS is defined, it is just a stub, "fake event":
  a) "fake event" can be in signalled or non-signalled mode
  b) "fake event" can be used only from one thread - one which created event
  c) when thread waits for signalling event, it immediately returns
  d) when thread waits for non-singalling event, program is terminated
     (because no one else can set event to signalling state)
     
     
IMPORTANT: event working in an auto reset mode does not  support  multiple
           threads  waiting  for  it  to  signal . It is reset after first
           thread is released.
*************************************************************************/
typedef struct
{
    /*
     * Pointer to _event structure. This pointer has type void* in order to
     * make header file OS-independent (event declaration depends on OS).
     */
    void *ptr;
} ae_event;

/*************************************************************************
Structure which is used to store parameters of the task.
*************************************************************************/
typedef struct
{
    union
    {
        ae_complex  cval;
        double      dval;
        ae_int_t    ival;
        ae_bool     bval;
        void        *val;
    }
    value;
} ae_task_parameter;


/*************************************************************************
Task data: set of task parameters and pointer to task function.
*************************************************************************/
typedef struct ae_task_data
{
    /*
     * Task parameters
     */
    ae_task_parameter parameters[AE_SMP_MAXPARAMS];

    /*
     * Pointer on a function which is called from the worker thread.
     */
    void (*func)(struct ae_task_data*, ae_state*);
} ae_task_data;


/*************************************************************************
Task information: task data and synchronization structures
*************************************************************************/
typedef struct ae_task_info
{
	/*
     * DESCRIPTION: auto reset event which is set when root task is solved.
     *              non-signalling by default.
     */
	ae_event done_event;

	/*
     * DESCRIPTION: task data, undefined by default.
     * PROTECTION:  no lock-based protection. However, it is assumed that
     *              a) before push_task() only one thread works with data
     *              b) push_task() generates memory barrier
     *              c) after pop_task() or steal_task() only one thread
     *                 works with data
     *              These assumptions should give enough of protection.
     */
	ae_task_data data;

	/*
     * DESCRIPTION: parent task group. can be null. null by default.
     *              when isn't null, task completion is signalled by:
     *              * acquiring parent_group.group_lock
     *              * decreasing waiting_count (and setting exception if needed)
     *              * in case new value of waiting_count is not 0 OR wake_up_worker_on_completion
     *                is false, we release group_lock and exit.
     *              * otherwise, we wake up sleeping worker and
     *                give him our queue by:
     *                * updating worker_idx
     *                * releasing group_lock
     *                * clearing wake_up_worker_on_completion field
     *                * signalling parent_worker.wakeup_event
     *                Finally, we dispose themseles (dispose_thread()
     *                is called and we wait for the wakeup_event)
     *              This field is NULL by default.
     *
     * PROTECTION:  same as for task_info.data
     */
	struct ae_task_group * volatile parent_group;

	/*
     * DESCRIPTION: a list of child task groups owned by this task.
     *              Groups are linked using ae_task_group->next_group field.
     *              This field is NULL on initial creation.
     *              Every time we create group it is added to this list.
     *              Every time we dispose group, it is removed from the list.
     *
     * PROTECTION:  no protection. Only one worker thread works with this list.
     */
	struct ae_task_group * volatile child_groups;

	/*
     * DESCRIPTION: set to non-null value when task (or one of its childs)
     *              spawned exception during execution.
     *              null by default.
     * PROTECTION:  no protection. This field is modified in the context
     *              of the thread which works with task.
     */
    volatile const char * volatile exception;

	/*
     * DESCRIPTION: used to organize child tasks into linked list
     *              (or to organize disposed tasks into list).
     * PROTECTION:  not needed (only parent worker references it).
     */
	struct ae_task_info * volatile next_task;
} ae_task_info;


/*************************************************************************
Group of child tasks
*************************************************************************/
typedef struct ae_task_group
{
    /*
     * DESCRIPTION: parent thread which spawned this specific group of tasks.
     *              can NOT be null.
     * PROTECTION:  not needed. This field is modified only by
     *              create_task_group() method which generates appropriate barriers
     *              so its value is visible to all other threads. Other methods
     *              and/or threads can read this value, but can not modify it.
     */
    struct ae_worker_thread * volatile parent_worker;

    /*
     * DESCRIPTION: number of child tasks waiting for processing (i.e. tasks which
     *              were attached to group and push'ed to queue). This field is
     *              incremented by push_task(), decremented by solve_task().
     * PROTECTION:  by group_lock.
     */
    volatile ae_int_t waiting_count;
    
    /*
     * DESCRIPTION: worker thread sets this field to true when it wants to be
     *              waked up on completion of this group. This field is set to
     *              false by thread which wakes up worker.
     * PROTECTION:  by group_lock.
     */
    volatile ae_bool wake_up_worker_on_completion;

    /*
     * DESCRIPTION: lock which protects group
     * PROTECTION:  by itself
     */
    ae_lock group_lock;

    /*
     * DESCRIPTION: list of child tasks, updated by push_task().
     *              contains tasks which were not processed immediately
     *              after push_task() in the context of the worker thread.
     * PROTECTION:  not needed (only parent_worker works with list)
     */
    ae_task_info * volatile childs;

	/*
     * DESCRIPTION: set to non-NULL value when child task raised exception 
     *              during execution. NULL by default.
     * PROTECTION:  before group is completed - protected by group_lock.
     *              after completion - it is possible to access this field without locking.
     */
    volatile const char * volatile exception;

    /*
     * DESCRIPTION: this field is used to organize groups in a linked list:
     *              either list of child groups of the worker thread or
     *              list of disposed groups. null by default.
     * PROTECTION:  not needed (only parent worker can modify this list).
     */
    struct ae_task_group * volatile next_group;
} ae_task_group;


/*************************************************************************
Worker thread
*************************************************************************/
typedef struct ae_worker_thread
{
	/*
     * Auto-reset event which is set to signalled state when all child 
     * are done (thread, other than the worker thread, which decreases
     * waiting_count down to zero, sets this event).
     *
     * When this event signalled, all tasks were processed (solved
     * or failed), waiting_count is zero, failed_count contains
     * number of failed tasks.
     */
	ae_event wakeup_event;

	/*
     * DESCRIPTION: thread handle (pointer to ae_thread_handle structure),
     *              set during initial creation, remains unmodified through
     *              all the lifetime.
     * PROTECTION:  not needed, other threads do not reference it.
     */
	void *thread_handle;

	/*
     * DESCRIPTION: worker_idx - index of the worker thread, from 1 to  queue_count-1.
     *              This index is set by create_worker(). It can be modified by another
     *              worker thread which wakes up current worker.
     *
     *              It is responsibility of the party who changed worker_idx to call
     *              ae_pin_thread()/ae_unpin_thread() in order to change information
     *              about ownership. The only exception from this rule is initial
     *              creation, when ownership on the queue is acquired in the worker_loop()
     *              function.
     *
     * PROTECTION:  protected, but has no dedicated lock. Protection protocol
     *              is described below:
     *
     *              1. modification of worker_idx means that thread is assigned
     *                 to some queue or reassigned to new queue
     *
     *              2. worker_idx is modified only when:
     *                 a) thread is created
     *                 b) thread starts to wait for the unfinished child group
     *                     (worker_idx is set to -1)
     *                 c) other thread which finished child group awoke thread
     *                 d) thread is disposed
     *                 e) disposed thread is reused
     *
     *              3. in case (a) worker_idx is set before thread is created,
     *                 protection is provided by implicit barriers which OS
     *                 generates during thread creation
     *
     *              4. in cases (b) and (c) modification of worker_idx is protected
     *                 by group_lock of the group being waited for:
     *                 * in case (b) thread modifies its own worker_idx (sets to -1)
     *                   after acquiring group_lock of group G1 (group it waits for).
     *                 * in case (c) another thread mofifies worker_idx of the thread
     *                   after acquiring group_lock of G1.
     *                 * in both cases worker_idx is modified simultaneously with
     *                   wake_up_worker_on_completion flag
     *                 In these cases protection is provided by the fact that only
     *                 one group can have wake_up_worker_on_completion flag set to
     *                 TRUE, only one thread can finish this group, and all operations
     *                 with this flag and worker_idx are performed within group_lock.
     *
     *              5. in cases (d) and (e) it is guaranteed that all child groups were
     *                 finished prior to disposing/waking up thread. So, no one can
     *                 modify worker_idx in attempt to wakeup thread.
     *                 In these cases protection is provided by thread's wakeup_event:
     *                 * in case (d) worker_idx is set to -1 before waiting for event
     *                 * in case (e) worker_idx is modified before thread wakes up.
     *
     */
	volatile ae_int_t worker_idx;

	/*
     * DESCRIPTION: this field is used to organize disposed threads into linked list.
     * PROTECTION:  not needed (only dispose_thread can modify this list).
     */
	struct ae_worker_thread *next_thread;
} ae_worker_thread;

ae_int_t ae_cores_count();
void ae_set_cores_to_use(ae_int_t ncores);
void ae_set_mkl_max_matrix_size(ae_int_t matrixsize);
void ae_init_event(ae_event *event, ae_bool manual_reset);
void ae_wait_for_event(ae_event *event);
void ae_set_event(ae_event *event);
void ae_reset_event(ae_event *event);
void ae_free_event(ae_event *event);

ae_task_info* ae_create_task(ae_task_group *parent_group, ae_state *_state);
void ae_dispose_task(ae_task_info *task);
ae_task_group* ae_create_task_group(ae_state *_state);
void ae_dispose_group(ae_task_group *group);

void ae_push_root_task(ae_task_info *task);
void ae_push_task(ae_task_info *task, ae_state *_state, ae_bool execute_immediately);
void ae_wait_for_group(
    ae_task_group *group,
    ae_bool dispose_on_success,
    ae_state *_state);
void ae_set_smp_support(
    ae_task_group **_child_tasks,
    ae_bool *_smp_enabled,
    ae_bool new_support_status,
    ae_state *_state);
void ae_sync(
    ae_task_group *_child_tasks,
    ae_bool smp_enabled,
    ae_bool dispose_group,
    ae_state *_state);

ae_bool ae_smpselftests();
    
#ifdef AE_SMP_DEBUGCOUNTERS
extern volatile ae_int64_t _ae_dbg_threads_spawned;
extern volatile ae_int64_t _ae_dbg_tasks_created;
extern volatile ae_int64_t _ae_dbg_tasks_stolen;
extern volatile ae_int64_t _ae_dbg_groups_created;
#endif

/*
 * MKL links
 *
 * NOTE: this code is located in smp.c because it is NOT included in HPC
 *       version of ALGLIB.
 */
#ifdef ALGLIB_INTERCEPTS_MKL
ae_bool _ialglib_i_rmatrixsyrkmkl(ae_int_t n,
     ae_int_t k,
     double alpha,
     ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper);
ae_bool _ialglib_i_cmatrixherkmkl(ae_int_t n,
     ae_int_t k,
     double alpha,
     ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     double beta,
     ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc,
     ae_bool isupper);
ae_bool _ialglib_i_rmatrixgemmmkl(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     double alpha,
     ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     double beta,
     ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc);
ae_bool _ialglib_i_cmatrixgemmmkl(ae_int_t m,
     ae_int_t n,
     ae_int_t k,
     ae_complex alpha,
     ae_matrix* a,
     ae_int_t ia,
     ae_int_t ja,
     ae_int_t optypea,
     ae_matrix* b,
     ae_int_t ib,
     ae_int_t jb,
     ae_int_t optypeb,
     ae_complex beta,
     ae_matrix* c,
     ae_int_t ic,
     ae_int_t jc);
ae_bool _ialglib_i_rmatrixlefttrsmmkl(ae_int_t m,
     ae_int_t n,
     ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_rmatrixrighttrsmmkl(ae_int_t m,
     ae_int_t n,
     ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_cmatrixlefttrsmmkl(ae_int_t m,
     ae_int_t n,
     ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_cmatrixrighttrsmmkl(ae_int_t m,
     ae_int_t n,
     ae_matrix* a,
     ae_int_t i1,
     ae_int_t j1,
     ae_bool isupper,
     ae_bool isunit,
     ae_int_t optype,
     ae_matrix* x,
     ae_int_t i2,
     ae_int_t j2);
ae_bool _ialglib_i_spdmatrixcholeskymkl(
     ae_matrix* a,
     ae_int_t offs,
     ae_int_t n,
     ae_bool isupper,
     ae_bool* cholresult);
ae_bool _ialglib_i_rmatrixplumkl(
     ae_matrix* a,
     ae_int_t offs,
     ae_int_t m,
     ae_int_t n,
     ae_vector* pivots);
ae_bool _ialglib_i_rmatrixbdmkl(
     ae_matrix* a,
     ae_int_t m,
     ae_int_t n,
     ae_vector* d,
     ae_vector* e,
     ae_vector* tauq,
     ae_vector* taup);
ae_bool _ialglib_i_rmatrixbdmultiplybymkl(
     ae_matrix* qp,
     ae_int_t m,
     ae_int_t n,
     ae_vector* tauq,
     ae_vector* taup,
     ae_matrix* z,
     ae_int_t zrows,
     ae_int_t zcolumns,
     ae_bool byq,
     ae_bool fromtheright,
     ae_bool dotranspose);
ae_bool _ialglib_i_rmatrixhessenbergmkl(
     ae_matrix* a,
     ae_int_t n,
     ae_vector* tau);
ae_bool _ialglib_i_rmatrixhessenbergunpackqmkl(
     ae_matrix* a,
     ae_int_t n,
     ae_vector* tau,
     ae_matrix* q);
ae_bool _ialglib_i_smatrixtdmkl(
     ae_matrix* a,
     ae_int_t n,
     ae_bool isupper,
     ae_vector* tau,
     ae_vector* d,
     ae_vector* e);
ae_bool _ialglib_i_smatrixtdunpackqmkl(
     ae_matrix* a,
     ae_int_t n,
     ae_bool isupper,
     ae_vector* tau,
     ae_matrix* q);
ae_bool _ialglib_i_hmatrixtdmkl(
     ae_matrix* a,
     ae_int_t n,
     ae_bool isupper,
     ae_vector* tau,
     ae_vector* d,
     ae_vector* e);
ae_bool _ialglib_i_hmatrixtdunpackqmkl(
     ae_matrix* a,
     ae_int_t n,
     ae_bool isupper,
     ae_vector* tau,
     ae_matrix* q);
ae_bool _ialglib_i_rmatrixbdsvdmkl(
     ae_vector* d,
     ae_vector* e,
     ae_int_t n,
     ae_bool isupper,
     ae_matrix* u,
     ae_int_t nru,
     ae_matrix* c,
     ae_int_t ncc,
     ae_matrix* vt,
     ae_int_t ncvt,
     ae_bool* svdresult);
ae_bool _ialglib_i_rmatrixinternalschurdecompositionmkl(
     ae_matrix* h,
     ae_int_t n,
     ae_int_t tneeded,
     ae_int_t zneeded,
     ae_vector* wr,
     ae_vector* wi,
     ae_matrix* z,
     ae_int_t* info);
ae_bool _ialglib_i_rmatrixinternaltrevcmkl(
     ae_matrix* t,
     ae_int_t n,
     ae_int_t side,
     ae_int_t howmny,
     ae_matrix* vl,
     ae_matrix* vr,
     ae_int_t* m,
     ae_int_t* info);
ae_bool _ialglib_i_smatrixtdevdmkl(
     ae_vector* d,
     ae_vector* e,
     ae_int_t n,
     ae_int_t zneeded,
     ae_matrix* z,
     ae_bool* evdresult);
#endif

/*
 * HPC cores with SSE2 support
 */
#ifdef ALGLIB_INTERCEPTS_SSE2
ae_bool _ialglib_i_hpcpreparechunkedgradientx(
    /* Real    */  ae_vector* weights,
                   ae_int_t wcount,
     /* Real    */ ae_vector* hpcbuf);
ae_bool _ialglib_i_hpcfinalizechunkedgradientx(
    /* Real    */  ae_vector* hpcbuf,
                   ae_int_t wcount,
     /* Real    */ ae_vector* grad);
ae_bool _ialglib_i_hpcchunkedgradient(/* Real    */ ae_vector* weights,
     /* Integer */ ae_vector* _structinfo,
     /* Real    */ ae_vector* _columnmeans,
     /* Real    */ ae_vector* _columnsigmas,
     /* Real    */ ae_matrix* xy,
     ae_int_t cstart,
     ae_int_t csize,
     /* Real    */ ae_vector* _batch4buf,
     /* Real    */ ae_vector* _hpcbuf,
     double* e,
     ae_bool naturalerrorfunc);
ae_bool _ialglib_i_hpcchunkedprocess(/* Real    */ ae_vector* weights,
     /* Integer */ ae_vector* _structinfo,
     /* Real    */ ae_vector* _columnmeans,
     /* Real    */ ae_vector* _columnsigmas,
     /* Real    */ ae_matrix* xy,
     ae_int_t cstart,
     ae_int_t csize,
     /* Real    */ ae_vector* _batch4buf,
     /* Real    */ ae_vector* _hpcbuf);
#endif


}


#endif

