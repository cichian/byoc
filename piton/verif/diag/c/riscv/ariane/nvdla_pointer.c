//#include <stdio.h>
#include <stdint.h>
#include "../piton/verif/diag/c/riscv/ariane/nvdla_cdma.h"
#include "../piton/verif/diag/c/riscv/ariane/nvdla.h"
#include "../piton/verif/diag/c/riscv/ariane/mmio.h"

#define NVDLA_BASE 0xfff0e00000
#define reg_write(addr,val) reg_write32(NVDLA_BASE+addr,val)
#define reg_read(addr) reg_read32(NVDLA_BASE+addr)
// tests proper functioning of ping-pong system
int main(int argc, char ** argv) {
  volatile unsigned int old = reg_read(CDMA_D_MISC_CFG);
  printf("CDMA_D_MISC_CFG %08x\n", old);

  reg_write(CDMA_S_POINTER, -1);
  reg_write(CDMA_D_MISC_CFG, reg_read(CDMA_D_MISC_CFG)+1);
  reg_write(CDMA_S_POINTER, 0);

  volatile unsigned int new = reg_read(CDMA_D_MISC_CFG); 
  printf("CDMA_D_MISC_CFG %08x\n", new);

  printf("TEST %sED\n", old == new ? "PASS" : "FAIL");
  return 0;
}
