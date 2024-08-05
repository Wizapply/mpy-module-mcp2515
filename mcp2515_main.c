/*
 *  Created on: Jan 4, 2024
 *      Author: Wizapply
 */

 //MPY include
#include "mpy_header.h"
#include "MCP2515.h"

//------------- MPY Global Functions -------------

#if !defined(__linux__)
void* memset(void* s, int c, size_t n) {
    return mp_fun_table.memset_(s, c, n);
}
void* memcpy(void* dest, const void* src, size_t n) {
    return mp_fun_table.memmove_(dest, src, n);
}
#endif

mp_obj_type_t* mp_SPI_ptr;
mp_obj_type_t* mp_Pin_ptr;

mp_obj_t mp_function_Pin_high;
mp_obj_t mp_function_Pin_low;
mp_obj_t mp_function_utime_sleepms;
mp_obj_t mp_function_utime_ticksms;
mp_obj_t mp_function_SPI_write;
mp_obj_t mp_function_SPI_read;

mp_obj_t mp_sv_Pin_cs;
mp_obj_t mp_sv_SPI_obj;

//data
canFrame g_queue_data[MCU_DATAQUEUE_MAXNUM];
int g_queue_tail;

//-------queue-------

bool pop(canFrame* ref)
{
    if (g_queue_tail < 0)
        return false;

    *ref = g_queue_data[g_queue_tail];
    g_queue_tail--;
    return true;
}

bool push(canFrame* ref)
{
    if (g_queue_tail >= MCU_DATAQUEUE_MAXNUM-1)
        return false;

    g_queue_tail++;
    g_queue_data[g_queue_tail] = *ref;

    return true;
}

//-----------------------------

void MCP_startSpi(void)
{
    //gpio_put(5, 0); // chip deselect, active low
    mp_call_function_n_kw(mp_function_Pin_low, 1, 0, &mp_sv_Pin_cs);
}

void MCP_endSpi(void)
{
    //gpio_put(5, 1);
    mp_call_function_n_kw(mp_function_Pin_high, 1, 0, &mp_sv_Pin_cs);
}

void MCP_transmitSpi(spi_inst_t* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    //spi_write_blocking(hspi, pData, Size);
    mp_obj_t write_data = mp_obj_new_bytes(pData, Size);
    mp_obj_t argv[] = { mp_sv_SPI_obj, write_data };
    mp_call_function_n_kw(mp_function_SPI_write, 2, 0, argv);
}

void MCP_receiveSpi(spi_inst_t* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    //spi_read_blocking(hspi, 0, pData, Size);
    mp_obj_t n = mp_obj_new_int(Size);
    mp_obj_t argv[] = { mp_sv_SPI_obj, n };
    mp_obj_t res = mp_call_function_n_kw(mp_function_SPI_read, 2, 0, argv);

    mp_buffer_info_t bufinfo;
    if (mp_get_buffer(res, &bufinfo, MP_BUFFER_READ)) {
        const uint8_t* src = bufinfo.buf;
        memcpy(pData, src, bufinfo.len);
    }
}

uint32_t MCP_getTick(void)
{
    //return (uint32_t)get_absolute_time();
    mp_obj_t res = mp_call_function_n_kw(mp_function_utime_ticksms, 0, 0, NULL);
    return (uint32_t)mp_obj_get_int(res);
}

void MCP_delay(uint32_t delay)
{
    //sleep_ms(delay);
    mp_obj_t dv = mp_obj_new_int(delay);
    mp_call_function_n_kw(mp_function_utime_sleepms, 1, 0, &dv);
}

//------------- MPY Functions -------------

static mp_obj_t can_init(mp_obj_t spi_obj, mp_obj_t cs_pin, mp_obj_t can_baudrate) {
    if (!mp_obj_is_type(spi_obj, mp_SPI_ptr))
        return mp_const_false;

    if (!mp_obj_is_type(cs_pin, mp_Pin_ptr))
        return mp_const_false;

    mp_sv_SPI_obj = spi_obj;
    mp_sv_Pin_cs = cs_pin;
    MCP_CAN_SPEED can_baudrate_config = (MCP_CAN_SPEED)mp_obj_get_int(can_baudrate);

    // reset & config the MCP2515
    if (MCP_reset() != ERROR_OK) return mp_const_false;
    if (MCP_setBitrate(can_baudrate_config, MCP_20MHZ) != ERROR_OK) return mp_const_false;
    if (MCP_setNormalMode() != ERROR_OK) return mp_const_false;

    g_queue_tail = -1;

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_3(can_init_obj, can_init);

static mp_obj_t can_mode(mp_obj_t mode) {
    if (mp_sv_SPI_obj == mp_const_none)
        return mp_const_none;

    int mode_int = mp_obj_get_int(mode);
    switch (mode_int) {
    case 0: MCP_setNormalMode();
        break;
    case 1: MCP_setListenOnlyMode();
        break;
    case 2: MCP_setLoopbackMode();
    case 3:
    default:
        MCP_setSleepMode();
        break;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(can_mode_obj, can_mode);

static mp_obj_t can_filter(mp_obj_t mask, mp_obj_t filter, mp_obj_t ext) {
    if (mp_sv_SPI_obj == mp_const_none)
        return mp_const_false;

    uint32_t mask_int = (uint32_t)mp_obj_get_int(mask);
    uint32_t filter_int = (uint32_t)mp_obj_get_int(filter);
    bool ext_b = mp_obj_get_int(ext) != 0;
    if (MCP_setFilter(RXF0, ext_b, filter_int) != ERROR_OK) {
        return mp_const_false;
    }
    MCP_setFilterMask(MASK0, ext_b, mask_int);
    MCP_setFilterMask(MASK1, ext_b, mask_int);

    MCP_setNormalMode();	//set
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_3(can_filter_obj, can_filter);

static mp_obj_t can_send(mp_obj_t canid, mp_obj_t byte_data, mp_obj_t opt) {
    if (mp_sv_SPI_obj == mp_const_none)
        return mp_const_none;

    canFrame canf;
    mp_buffer_info_t bufinfo;

    uint32_t id_int = (uint32_t)mp_obj_get_int(canid);
    uint32_t opt_int = (uint32_t)mp_obj_get_int(opt);
    if (opt_int & CAN_EFF_FLAG)
        id_int &= CAN_EFF_MASK;
    else
        id_int &= CAN_SFF_MASK;
    canf.id = id_int | (opt_int & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG));

    if (mp_get_buffer(byte_data, &bufinfo, MP_BUFFER_READ)) {
        const uint8_t* src = bufinfo.buf;
        canf.dlc = bufinfo.len;
        if (canf.dlc > 8) canf.dlc = 8;
        memcpy(canf.data, src, canf.dlc);
    }

    if (MCP_sendMessage(&canf) != ERROR_OK)
        return mp_const_false;

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_3(can_send_obj, can_send);

static mp_obj_t can_read() {
    if (mp_sv_SPI_obj == mp_const_none)
        return mp_const_none;

    canFrame frame;
    if (pop(&frame)) {
        mp_obj_t tuple[3];
        if (frame.id & CAN_EFF_FLAG)
            tuple[0] = mp_obj_new_int(frame.id & CAN_EFF_MASK);
        else
            tuple[0] = mp_obj_new_int(frame.id & CAN_SFF_MASK);
        tuple[1] = mp_obj_new_bytes(frame.data, frame.dlc);
        tuple[2] = mp_obj_new_int(frame.id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG));
        return mp_obj_new_tuple(3, tuple);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(can_read_obj, can_read);

//poling
static mp_obj_t can_recv_poling() {
    if (mp_sv_SPI_obj == mp_const_none)
        return mp_const_none;

    canFrame frame = { 0 };
    while (MCP_checkReceive()) {
        if (MCP_readMessage(&frame) == ERROR_OK) {
            if (!push(&frame)) break;
        }
    }

    return mp_obj_new_int(g_queue_tail+1);
}
static MP_DEFINE_CONST_FUN_OBJ_0(can_recv_poling_obj, can_recv_poling);

//Entry Point
mp_obj_t mpy_init(mp_obj_fun_bc_t* self, size_t n_args, size_t n_kw, mp_obj_t* args) {
    // This must be first, it sets up the globals dict and other things
    MP_DYNRUNTIME_INIT_ENTRY

        //function
        MP_IMPORT_MODULE(machine) // MP_QSTR_machine
        MP_GETATTR(modmachine, Pin) // MP_QSTR_Pin
        MP_GETATTR(modmachine, SPI) // MP_QSTR_SPI

        mp_Pin_ptr = MP_OBJ_TO_PTR(mp_Pin);
    MP_GET_BOUND_METHOD(Pin, high) // MP_QSTR_high
        mp_function_Pin_high = mp_Pin_high;
    MP_GET_BOUND_METHOD(Pin, low) // MP_QSTR_low
        mp_function_Pin_low = mp_Pin_low;

    MP_IMPORT_MODULE(utime) // MP_QSTR_utime
        MP_GETATTR(modutime, sleep_ms) // MP_QSTR_sleep_ms
        mp_function_utime_sleepms = mp_sleep_ms;
    MP_GETATTR(modutime, ticks_ms) // MP_QSTR_ticks_ms
        mp_function_utime_ticksms = mp_ticks_ms;

    mp_SPI_ptr = MP_OBJ_TO_PTR(mp_SPI);
    MP_GET_BOUND_METHOD(SPI, write) // MP_QSTR_write
        mp_function_SPI_write = mp_SPI_write;
    MP_GET_BOUND_METHOD(SPI, read) // MP_QSTR_read
        mp_function_SPI_read = mp_SPI_read;

    // define store
    mp_store_global(MP_QSTR_can_init, MP_OBJ_FROM_PTR(&can_init_obj));
    mp_store_global(MP_QSTR_can_recv_poling, MP_OBJ_FROM_PTR(&can_recv_poling_obj));
    mp_store_global(MP_QSTR_can_send, MP_OBJ_FROM_PTR(&can_send_obj));
    mp_store_global(MP_QSTR_can_read, MP_OBJ_FROM_PTR(&can_read_obj));
    mp_store_global(MP_QSTR_can_filter, MP_OBJ_FROM_PTR(&can_filter_obj));
    mp_store_global(MP_QSTR_can_mode, MP_OBJ_FROM_PTR(&can_mode_obj));

    mp_store_global(MP_QSTR_NormalMode, mp_obj_new_int(0));
    mp_store_global(MP_QSTR_ListenMode, mp_obj_new_int(1));
    mp_store_global(MP_QSTR_LoopbackMode, mp_obj_new_int(2));
    mp_store_global(MP_QSTR_SleepMode, mp_obj_new_int(3));

    mp_store_global(MP_QSTR_OptExtFrame, mp_obj_new_int(0x80000000UL));
    mp_store_global(MP_QSTR_OptRemoteReq, mp_obj_new_int(0x40000000UL));
    mp_store_global(MP_QSTR_OptErrorFrame, mp_obj_new_int(0x20000000UL));

    mp_sv_Pin_cs = mp_const_none;
    mp_sv_SPI_obj = mp_const_none;
    g_queue_tail = -1;

    // This must be last, it restores the globals dict
    MP_DYNRUNTIME_INIT_EXIT
}
