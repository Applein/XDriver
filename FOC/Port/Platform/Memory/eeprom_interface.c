/*
 * eeprom_interface.c
 *
 * 上游: Port/Platform/Memory/eeprom_interface.h (内联实现 + class EEPROM 模板)
 *
 * 用 emulated_eeprom.c 提供的字节级接口实现按块读写,
 * 取代上游的模板 get<T> / put<T>。
 */

#include "Platform/Memory/eeprom_interface.h"

#include <stddef.h>     /* NULL */
#include <string.h>

#include "Platform/Memory/emulated_eeprom.h"

void eeprom_get_blob(uint16_t offset, void *dst, uint16_t len)
{
    uint8_t *out = (uint8_t *)dst;
    uint16_t i;

    if ((dst == NULL) || (len == 0U))
    {
        return;
    }
    if (((uint32_t)offset + (uint32_t)len) > EEPROM_EMU_SIZE)
    {
        return;     /* 越界: 直接放弃, 不破坏调用方缓冲 */
    }

    /* 上游 EEPROM::get() 在 !_initialized 时先 init() -> eeprom_buffer_fill() */
    eeprom_buffer_fill();

    for (i = 0; i < len; i++)
    {
        out[i] = eeprom_buffered_read_byte((uint32_t)offset + i);
    }
}

void eeprom_put_blob(uint16_t offset, const void *src, uint16_t len)
{
    const uint8_t *in = (const uint8_t *)src;
    uint16_t i;

    if ((src == NULL) || (len == 0U))
    {
        return;
    }
    if (((uint32_t)offset + (uint32_t)len) > EEPROM_EMU_SIZE)
    {
        return;
    }

    /* 先 fill 再改, 保证未修改的部分保持原值 */
    eeprom_buffer_fill();

    for (i = 0; i < len; i++)
    {
        eeprom_buffered_write_byte((uint32_t)offset + i, in[i]);
    }

    /* 上游 _commitASAP = true: 立即落盘 */
    eeprom_buffer_flush();
}
