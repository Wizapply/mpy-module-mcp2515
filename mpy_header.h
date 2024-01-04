/*
 *  Created on: Jan 4, 2024
 *      Author: Wizapply
 */

#ifndef INC_MPY_HEADER_H_
#define INC_MPY_HEADER_H_

#include "py/dynruntime.h"
#include "can.h"

//define
#define MP_IMPORT_MODULE(mname) mp_obj_t mp_mod##mname = mp_import_name(MP_QSTR_##mname, mp_const_none, 0);
#define MP_GETATTR(name, attr) mp_obj_t mp_##attr = mp_load_attr(mp_##name, MP_QSTR_##attr);
#define MP_GET_BOUND_METHOD(name, attr) mp_obj_t meth_##attr[2]; \
                                        mp_load_method(mp_##name, MP_QSTR_##attr, meth_##attr); \
                                        mp_obj_t mp_##name##_##attr = meth_##attr[0];

//struct
typedef struct spi_inst spi_inst_t;
typedef struct _machine_spi_obj_t {
    mp_obj_base_t base;
    spi_inst_t *const spi_inst;
    uint8_t spi_id;
    uint8_t polarity;
    uint8_t phase;
    uint8_t bits;
    uint8_t firstbit;
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
    uint32_t baudrate;
} machine_spi_obj_t;

//function
void* memset(void *s, int c, size_t n);

#define MCU_DATAQUEUE_MAXNUM (50)    //Optimized for rp2040

#endif /* INC_MPY_HEADER_H_ */
