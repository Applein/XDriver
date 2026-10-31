/*
 * eeprom_interface.h
 *
 * 上游: Port/Platform/Memory/eeprom_interface.h
 *       该文件把 FlashStorage_STM32 库的宏、eeprom_buffer 定义、字节读写函数
 *       与 class EEPROM (含模板 get<T>/put<T>) 全部塞在一个头文件里。
 *       C 语言下必须在头文件里放"定义"和多处函数实现 -> 多 TU 重定义,
 *       因此这里改为: 头文件只留声明, 实现放到 eeprom_interface.c。
 *
 * C 化改动:
 *   1. 删除 class EEPROM 与模板 get<T>/put<T>
 *      -> eeprom_get_blob() / eeprom_put_blob()  (按字节长度搬运)
 *   2. 存储位置不变: Flash 最后一页 1KB
 *        FLASH_BASE_ADDRESS = FLASH_END + 1 - FLASH_PAGE_SIZE
 *        STM32F103CBT6 => 0x0801FC00, 与 stockpile_config.h 的
 *        STOCKPILE_APP_DATA_ADDR 是同一页。本工程只用其中一个
 *        (stockpile_data 未被使用), 不会互相覆盖。
 *   3. 实现依赖 emulated_eeprom.c (Stm32_eeprom_emulation 库,
 *      缓冲 + 整页擦除 + 双字编程)
 *
 * 行为差异说明 (与上游 EEPROM 类对比):
 *   上游 EEPROM::put() 在 _commitASAP = true 时立即写 Flash;
 *   本实现 eeprom_put_blob() 同样立即落盘。
 *   上游 EEPROM() 构造会初始化, get/put 首次调用时才 fill; 本实现每次
 *   eeprom_get_blob() 都 fill 一次 (1KB memcpy, 只在配置变更流程里调用,
 *   不影响 20 kHz 控制环)。
 *
 * 注意: 擦写一页会让 CPU 阻塞数毫秒到数十毫秒 (取决于页大小与等待周期),
 *       因此调用方必须保证此时电机处于停止/休眠状态, 且不要在临界区内调用。
 */

#ifndef CTRL_STEP_FW_EEPROM_INTERFACE_H
#define CTRL_STEP_FW_EEPROM_INTERFACE_H

#include <stdint.h>

/* EEPROM 模拟区容量 (字节): 最后一页 1KB */
#define EEPROM_EMU_SIZE     1024U

/* 从模拟 EEPROM 读取一段数据 (offset + len 必须落在 EEPROM_EMU_SIZE 内) */
void eeprom_get_blob(uint16_t offset, void *dst, uint16_t len);

/* 写入一段数据到模拟 EEPROM 并立即落盘 (会擦除整页) */
void eeprom_put_blob(uint16_t offset, const void *src, uint16_t len);

#endif /* CTRL_STEP_FW_EEPROM_INTERFACE_H */
