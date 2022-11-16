#ifndef NVDLA_HEADER
#define NVDLA_HEADER
unsigned int *BASE_ADDR = (unsigned int*) 0xfff0e00000;
#endif

#ifndef _MK_MASK_CONST
  #define _MK_MASK_CONST(_constant_) _constant_
#endif

#ifndef _MK_FIELD_CONST
  #define _MK_FIELD_CONST(_mask_, _shift_) (_MK_MASK_CONST(_mask_) << _MK_SHIFT_CONST(_shift_))
#endif

#ifndef _MK_SHIFT_CONST
  #define _MK_SHIFT_CONST(_constant_) _constant_
#endif

#define GLB_S_INTR_MASK_0_CDP_DONE_MASK0_SHIFT			_MK_SHIFT_CONST(2)
#define GLB_S_INTR_MASK_0_CDP_DONE_MASK0_FIELD			_MK_FIELD_CONST(0x1, GLB_S_INTR_MASK_0_CDP_DONE_MASK0_SHIFT)
#define GLB_S_INTR_MASK_0_CDP_DONE_MASK1_SHIFT			_MK_SHIFT_CONST(3)
#define GLB_S_INTR_MASK_0_CDP_DONE_MASK1_FIELD			_MK_FIELD_CONST(0x1, GLB_S_INTR_MASK_0_CDP_DONE_MASK1_SHIFT)
