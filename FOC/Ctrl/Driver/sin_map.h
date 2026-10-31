/*
 * sin_map.h
 *
 * 上游: Ctrl/Driver/sin_map.h
 *
 * C 化改动:
 *   1. 数组定义搬到 sin_map.c, 头文件只保留 extern 声明
 *      原因: C 语言下若在头文件里定义 const 数组, 每个包含它的编译单元都会
 *      生成一份定义 -> 链接期 multiple definition; 改成 static const 则
 *      2049 项 (约 4KB) 的表会被复制多份。
 *   2. <cstdint> / <cmath> -> <stdint.h> (cmath 在上游也未被用到)
 *   3. 删除 extern "C" 与结尾多余的空白行
 *   4. 宏名 sin_pi_m2_dpix / sin_pi_m2_dpiybit 保持不变 (tb67h450_base.c 依赖)
 */

#ifndef SIN_FORM_H
#define SIN_FORM_H

#include <stdint.h>

#define sin_pi_m2_dpix      1024
#define sin_pi_m2_dpiybit   12

/* 正弦表: sin_pi_m2[i] = round(sin(i * pi / 2 / 1024) * 4096)
 * 定义在 sin_map.c, 共 sin_pi_m2_dpix + 1 项 (第一个四分之一周期 + 端点) */
extern const int16_t sin_pi_m2[sin_pi_m2_dpix + 1];

#endif /* SIN_FORM_H */
