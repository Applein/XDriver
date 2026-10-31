/*
 * configurations.h
 *
 * 上游: UserApp/configurations.h
 *
 * C 化改动:
 *   1. 删除 extern "C" 分支 (本工程不再有 C++ 源文件)
 *   2. <stdint-gcc.h> -> <stdint.h>
 *   3. 删除 C++ 作用域里的 #include <Platform/Memory/eeprom_interface.h> 与
 *      "Motor/motor.h"。这两个包含在上游是为了让使用者通过本头文件间接拿到
 *      EEPROM 与 Motor 的声明; C 里由使用者显式包含更清晰, 也避免头文件循环
 *      (motor.c 需要 boardConfig, 而 motor.h 不需要 eeprom)。
 *   4. 结构体字段与顺序保持不变 (它会整体写入 Flash 配置页, 布局不能动)
 */

#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <stdbool.h>
#include <stdint.h>

/* 上游: configStatus_t */
typedef enum
{
    CONFIG_RESTORE = 0,
    CONFIG_OK,
    CONFIG_COMMIT,
} configStatus_t;

/*
 * 上游: Config_t (应用层对外叫 BoardConfig_t)
 *
 * 警告: 本结构体会被原样写入 Flash 配置页 (STOCKPILE_APP_DATA_ADDR),
 *       字段顺序/类型一旦改动, 旧 Flash 内容会被解析成错误的配置。
 *       如需扩展, 应新增 magic/version 字段并做校验。
 */
typedef struct
{
    configStatus_t config_status;
    uint32_t can_node_id;
    int32_t encoder_home_offset;
    uint32_t default_mode;
    int32_t current_limit;
    int32_t velocity_limit;
    int32_t velocity_acc;
    int32_t calibration_current;
    int32_t dce_kp;
    int32_t dce_kv;
    int32_t dce_ki;
    int32_t dce_kd;
    bool enable_motor_on_boot;
    bool enable_stall_protect;
} BoardConfig_t;

extern BoardConfig_t boardConfig;

#endif /* CONFIGURATIONS_H */
