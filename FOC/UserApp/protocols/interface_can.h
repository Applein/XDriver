/*
 * interface_can.h
 *
 * 上游: UserApp/protocols/interface_can.cpp (OnCanCmd)
 *      + Core/Src/can.c 的 USER CODE 1
 *        (CAN_Send / HAL_CAN_RxFifo0MsgPendingCallback)
 *      + Core/Src/can.c 的 USER CODE 2
 *        (过滤器配置 / HAL_CAN_Start / ActivateNotification)
 *
 * C 化改动 (逐条对应实现):
 *   1. 上游在 CAN 接收中断里直接跑 OnCanCmd(), 会就地改 motor / boardConfig,
 *      与 20 kHz 中断、100 Hz 线程争用同一批字段。
 *      这里改为: 中断只把帧拷进环形队列 + rt_sem_release, 命令在
 *      can_protocol 线程里执行, 与 msh 命令的上下文一致。
 *   2. `*(float*)RxData` / `*(uint32_t*)RxData` -> memcpy。
 *      消除未对齐访问与严格别名 UB (收发 buffer 不保证 4 字节对齐)。
 *   3. 过滤器/启动从 MX_CAN_Init 的 USER CODE 挪到 can_protocol_init():
 *      本工程 MX_CAN_Init() 由 rt_hw_board_init() 调用, 那时 boardConfig
 *      还没从 Flash 读出来; 而过滤器是全通, 节点号在软件里比对。
 *   4. 发送失败改为计数, 不调用 Error_Handler(): 线程里死等/停机没有意义。
 */

#ifndef CTRL_STEP_FW_INTERFACE_CAN_H
#define CTRL_STEP_FW_INTERFACE_CAN_H

#include <stdint.h>

#include "Motor/motor.h"
#include "Sensor/Encoder/encoder_calibrator_base.h"

/*
 * 初始化 (必须在 load_board_config() 与 motor_init() 之后调用)
 *
 *   - 配置全通滤波器 -> HAL_CAN_Start -> 打开 RX FIFO0 挂起中断
 *   - 建 can_protocol 线程 (静态栈), 命令在该线程执行
 */
void can_protocol_init(Motor *motor, EncoderCalibrator *calibrator);

/* 上游: void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len)
 * 线程上下文执行; 不要在中断里调用 (内部有临界区和 HAL 发送)。 */
void can_protocol_on_cmd(uint8_t cmd, uint8_t *data, uint32_t len);

/* 诊断计数 (在 xdrv status 里打印, 只增不减, 重启清零) */
extern volatile uint32_t can_protocol_rx_count;      /* 中断收到的帧数 */
extern volatile uint32_t can_protocol_cmd_count;     /* 实际执行的命令数 */
extern volatile uint32_t can_protocol_filter_count;  /* 节点号不匹配被忽略 */
extern volatile uint32_t can_protocol_drop_count;    /* 环形队列满被丢弃 */
extern volatile uint32_t can_protocol_tx_fail_count; /* 应答发送失败 */

#endif /* CTRL_STEP_FW_INTERFACE_CAN_H */
