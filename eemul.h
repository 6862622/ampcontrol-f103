#ifndef EEEMUL_H
#define EEEMUL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define EE_PAGE_SIZE    1024

enum {
    EE_PAGE_FM  = 123,

    EE_PAGE_0   = 124,
    EE_PAGE_1   = 126,

    EE_PAGE_END = 128,
};

void eeInit(void);

void *eeGetPageAddr(uint16_t page);
void eeErasePages(uint16_t page, uint16_t count);
void eeWritePage(uint16_t page, void *addr, uint16_t bytes);

void eeUpdateRaw(uint16_t addr, uint16_t data);
uint16_t eeReadRaw(uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif // EEEMUL_H
