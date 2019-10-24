#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _CaL_reg(void);
extern void _KV2_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," CaL.mod");
    fprintf(stderr," KV2.mod");
    fprintf(stderr, "\n");
  }
  _CaL_reg();
  _KV2_reg();
}
