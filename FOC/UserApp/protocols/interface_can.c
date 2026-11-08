/*
 * interface_can.c
 *
 * 上游: UserApp/protocols/interface_can.cpp (OnCanCmd)
 *      + Core/Src/can.c 的 USER CODE 1 / USER CODE 2
 *        (CAN_Send, HAL_CAN_RxFifo0MsgPendingCallback, 过滤器, HAL_CAN_Start)
 *
 * 数据流:
 *
 *   USB_LP_CAN1_RX0_IRQHandler            (优先级 3)
 *     -> HAL_CAN_IRQHandler
 *        -> HAL_CAN_RxFifo0MsgPendingCallback   <- 本文件实现在这里
 *           -> 取出所有帧, 拷进 can_rx_queue, rt_sem_release
 *
 *   can_protocol 线程 (优先级 14)
 *     -> rt_sem_take
 *        -> 逐帧 can_protocol_dispatch()  (节点号过滤)
 *           -> can_protocol_on_cmd()      (上游 OnCanCmd)
 *
 * 为什么不在中断里直接跑命令 (与上游的唯一行为差异):
 *   上游 OnCanCmd 会调用 SetPositionSetPointWithTime() 等带除法的长路径,
 *   还会 HAL_CAN_AddTxMessage()。RT-Thread 下这些都在线程里做更安全:
 *   临界区可以精确圈住改状态的那几行, 不会整段关中断拖慢 20 kHz 环。
 */

#include "interface_can.h"

#include <stdbool.h>
#include <string.h>

#include <rtthread.h>
#include <rthw.h>

#include "can.h"                /* hcan / CAN_TxHeaderTypeDef */
#include "configurations.h"     /* boardConfig */

/* ========================================================================== */
/* 帧队列 (单生产者 = RX 中断, 单消费者 = can_protocol 线程)                   */
/* ========================================================================== */

#define CAN_RX_QUEUE_LEN        8       /* 2 的幂不必要: 取模由编译器优化常数 */
#define CAN_THREAD_STACK_SIZE   768
#define CAN_THREAD_PRIORITY     14      /* 低于 foc_hk(12), 高于 shell(20) */
#define CAN_THREAD_TICK         10

typedef struct
{
    uint32_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} CanRxFrame;

static CanRxFrame can_rx_queue[CAN_RX_QUEUE_LEN];
static volatile uint32_t can_rx_head;   /* 只有中断写 */
static volatile uint32_t can_rx_tail;   /* 只有线程写 */

static struct rt_semaphore can_rx_sem;
static struct rt_thread can_thread;
static rt_uint8_t can_thread_stack[CAN_THREAD_STACK_SIZE];

static Motor *can_motor;
static EncoderCalibrator *can_calibrator;
static bool can_inited;

volatile uint32_t can_protocol_rx_count;
volatile uint32_t can_protocol_cmd_count;
volatile uint32_t can_protocol_filter_count;
volatile uint32_t can_protocol_drop_count;
volatile uint32_t can_protocol_tx_fail_count;

/*
 * 临界区: 只有"改 controller / motion_planner 状态"的那几行才需要,
 * 发送应答、读状态都在区外做 (HAL_CAN_AddTxMessage 依赖 HAL_GetTick,
 * 而 tick 由 TIM1 中断递增 —— 在关中断的临界区里它会等不到 tick)。
 *
 * 用法约束: 只能在声明了 `rt_base_t irq_level;` 的函数里用 (本文件只有
 * can_protocol_on_cmd 用)。C99 不允许标签后面直接跟声明, 所以不用
 * "宏里声明局部变量" 的写法, 避免每个 case 都要加一对花括号。
 */
#define CAN_CRITICAL_ENTER()    (irq_level = rt_hw_interrupt_disable())
#define CAN_CRITICAL_EXIT()     rt_hw_interrupt_enable(irq_level)

/* ========================================================================== */
/* 取数 / 应答 (取代上游的 *(float*) / *(uint32_t*) 强制转换)                  */
/* ========================================================================== */

static float can_read_float(const uint8_t *src)
{
    float value = 0.0f;

    memcpy(&value, src, sizeof(value));

    return value;
}

static uint32_t can_read_uint32(const uint8_t *src)
{
    uint32_t value = 0U;

    memcpy(&value, src, sizeof(value));

    return value;
}

static int32_t can_read_int32(const uint8_t *src)
{
    int32_t value = 0;

    memcpy(&value, src, sizeof(value));

    return value;
}

/*
 * 上游: void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
 *
 * 差异: 上游失败时调用 Error_Handler() (停机死循环)。这里是后台线程,
 *       一个应答发不出去(总线无 ACK / 邮箱满)不该让整个驱动器停摆, 改为计数。
 */
static bool can_send(uint32_t std_id, const uint8_t *data)
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;
    uint8_t payload[8];

    memcpy(payload, data, sizeof(payload));

    header.StdId = std_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &header, payload, &mailbox) != HAL_OK)
    {
        can_protocol_tx_fail_count++;

        return false;
    }

    return true;
}

/* 应答 ID: (节点号 << 7) | 命令号, 与上游一致 */
static uint32_t can_reply_id(uint32_t cmd)
{
    return ((boardConfig.can_node_id & 0x0FU) << 7) | (cmd & 0x7FU);
}

/*
 * 上游没有这一步, 但本工程的 'off' / 'brake' 用的是 set_disable / set_brake,
 * 而 20 kHz 环的前两个分支只看 soft_disable / soft_brake, 不看 mode_running。
 * 不清理就会出现"CAN 发了速度命令, 电机却纹丝不动"。与 msh 的
 * foc_motion_prepare() 保持一致。
 */
static void can_motion_prepare(MotionController *ctrl)
{
    motion_ctrl_set_disable(ctrl, false);
    motion_ctrl_set_brake(ctrl, false);
}

/* ========================================================================== */
/* 命令处理 (上游 OnCanCmd 的逐 case 移植)                                     */
/* ========================================================================== */

void can_protocol_on_cmd(uint8_t cmd, uint8_t *data, uint32_t len)
{
    MotionController *ctrl;
    uint8_t reply[8];
    bool need_ack = false;
    float tmp_f;
    rt_base_t irq_level = 0;

    if ((can_motor == NULL) || (can_calibrator == NULL))
    {
        return;
    }

    ctrl = can_motor->controller;

    /* 上游直接复用收到的 RxData 当应答缓冲; 帧短时尾部是上一次的残留,
     * 这里先整体拷一份, ISR 已把 DLC 之后的字节清零。 */
    memcpy(reply, data, sizeof(reply));

    (void)len;

    switch (cmd)
    {
    /* ---- 0x00~0x0F 不落 Flash 的命令 ---- */
    case 0x01:  /* Enable Motor: 1 = 速度模式, 其它 = 停止 */
        CAN_CRITICAL_ENTER();
        if (can_read_uint32(data) == 1U)
        {
            can_motion_prepare(ctrl);
            ctrl->request_mode = MOTOR_MODE_COMMAND_VELOCITY;
        }
        else
        {
            ctrl->request_mode = MOTOR_MODE_STOP;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x02:  /* Do Calibration */
        can_calibrator->is_triggered = true;
        break;

    case 0x03:  /* Set Current SetPoint (A -> mA) */
        CAN_CRITICAL_ENTER();
        can_motion_prepare(ctrl);
        if (ctrl->mode_running != MOTOR_MODE_COMMAND_CURRENT)
        {
            motion_ctrl_set_ctrl_mode(ctrl, MOTOR_MODE_COMMAND_CURRENT);
        }
        motion_ctrl_set_current_set_point(ctrl, (int32_t)(can_read_float(data) * 1000.0f));
        CAN_CRITICAL_EXIT();
        break;

    case 0x04:  /* Set Velocity SetPoint (rev/s -> steps/s) */
        CAN_CRITICAL_ENTER();
        can_motion_prepare(ctrl);
        if (ctrl->mode_running != MOTOR_MODE_COMMAND_VELOCITY)
        {
            can_motor->config.motion_params.rated_velocity = boardConfig.velocity_limit;
            motion_ctrl_set_ctrl_mode(ctrl, MOTOR_MODE_COMMAND_VELOCITY);
        }
        motion_ctrl_set_velocity_set_point(
            ctrl, (int32_t)(can_read_float(data) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        CAN_CRITICAL_EXIT();
        break;

    case 0x05:  /* Set Position SetPoint, data[4] != 0 时回带位置与到位标志 */
        need_ack = (data[4] != 0U);

        CAN_CRITICAL_ENTER();
        can_motion_prepare(ctrl);
        if (ctrl->mode_running != MOTOR_MODE_COMMAND_POSITION)
        {
            can_motor->config.motion_params.rated_velocity = boardConfig.velocity_limit;
            motion_ctrl_set_ctrl_mode(ctrl, MOTOR_MODE_COMMAND_POSITION);
        }
        motion_ctrl_set_position_set_point(
            ctrl, (int32_t)(can_read_float(data) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));

        if (need_ack)
        {
            tmp_f = motion_ctrl_get_position(ctrl, false);
            memcpy(reply, &tmp_f, sizeof(tmp_f));
            reply[4] = (uint8_t)((ctrl->state == MOTOR_STATE_FINISH) ? 1 : 0);
        }
        CAN_CRITICAL_EXIT();

        if (need_ack)
        {
            (void)can_send(can_reply_id(0x23U), reply);
        }
        break;

    case 0x06:  /* Set Position with Time (位置 + 到达时间 r/s) */
        need_ack = (data[4] != 0U);

        CAN_CRITICAL_ENTER();
        can_motion_prepare(ctrl);
        if (ctrl->mode_running != MOTOR_MODE_COMMAND_POSITION)
        {
            motion_ctrl_set_ctrl_mode(ctrl, MOTOR_MODE_COMMAND_POSITION);
        }
        (void)motion_ctrl_set_position_set_point_with_time(
            ctrl,
            (int32_t)(can_read_float(data) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
            can_read_float(data + 4));

        if (need_ack)
        {
            tmp_f = motion_ctrl_get_position(ctrl, false);
            memcpy(reply, &tmp_f, sizeof(tmp_f));
            reply[4] = (uint8_t)((ctrl->state == MOTOR_STATE_FINISH) ? 1 : 0);
        }
        CAN_CRITICAL_EXIT();

        if (need_ack)
        {
            (void)can_send(can_reply_id(0x23U), reply);
        }
        break;

    case 0x07:  /* Set Position with Velocity-Limit, 总是回带位置与到位标志 */
        CAN_CRITICAL_ENTER();
        can_motion_prepare(ctrl);
        if (ctrl->mode_running != MOTOR_MODE_COMMAND_POSITION)
        {
            can_motor->config.motion_params.rated_velocity = boardConfig.velocity_limit;
            motion_ctrl_set_ctrl_mode(ctrl, MOTOR_MODE_COMMAND_POSITION);
        }
        can_motor->config.motion_params.rated_velocity =
            (int32_t)(can_read_float(data + 4) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        motion_ctrl_set_position_set_point(
            ctrl, (int32_t)(can_read_float(data) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));

        tmp_f = motion_ctrl_get_position(ctrl, false);
        memcpy(reply, &tmp_f, sizeof(tmp_f));
        reply[4] = (uint8_t)((ctrl->state == MOTOR_STATE_FINISH) ? 1 : 0);
        CAN_CRITICAL_EXIT();

        (void)can_send(can_reply_id(0x23U), reply);
        break;

    /* ---- 0x10~0x1F 会落 Flash 的命令 (data[4] != 0 表示立即提交) ---- */
    case 0x11:  /* Set Node-ID */
        CAN_CRITICAL_ENTER();
        boardConfig.can_node_id = can_read_uint32(data);
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x12:  /* Set Current-Limit (A -> mA) */
        CAN_CRITICAL_ENTER();
        can_motor->config.motion_params.rated_current =
            (int32_t)(can_read_float(data) * 1000.0f);
        boardConfig.current_limit = can_motor->config.motion_params.rated_current;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x13:  /* Set Velocity-Limit (rev/s -> steps/s) */
        CAN_CRITICAL_ENTER();
        can_motor->config.motion_params.rated_velocity =
            (int32_t)(can_read_float(data) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        boardConfig.velocity_limit = can_motor->config.motion_params.rated_velocity;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x14:  /* Set Acceleration (rev/s^2 -> steps/s^2) */
        CAN_CRITICAL_ENTER();
        tmp_f = can_read_float(data) * (float)MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        can_motor->config.motion_params.rated_velocity_acc = (int32_t)tmp_f;
        velocity_tracker_set_velocity_acc(&can_motor->motion_planner.velocity_tracker,
                                          (int32_t)tmp_f);
        position_tracker_set_velocity_acc(&can_motor->motion_planner.position_tracker,
                                          (int32_t)tmp_f);
        boardConfig.velocity_acc = can_motor->config.motion_params.rated_velocity_acc;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x15:  /* Apply Home-Position */
        CAN_CRITICAL_ENTER();
        motion_ctrl_apply_pos_as_home_offset(ctrl);
        boardConfig.encoder_home_offset =
            can_motor->config.motion_params.encoder_home_offset %
            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        boardConfig.config_status = CONFIG_COMMIT;
        CAN_CRITICAL_EXIT();
        break;

    case 0x16:  /* Set Auto-Enable */
        CAN_CRITICAL_ENTER();
        boardConfig.enable_motor_on_boot = (can_read_uint32(data) == 1U);
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x17:  /* Set DCE Kp */
        CAN_CRITICAL_ENTER();
        can_motor->config.ctrl_params.dce.kp = can_read_int32(data);
        boardConfig.dce_kp = can_motor->config.ctrl_params.dce.kp;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x18:  /* Set DCE Kv */
        CAN_CRITICAL_ENTER();
        can_motor->config.ctrl_params.dce.kv = can_read_int32(data);
        boardConfig.dce_kv = can_motor->config.ctrl_params.dce.kv;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x19:  /* Set DCE Ki */
        CAN_CRITICAL_ENTER();
        can_motor->config.ctrl_params.dce.ki = can_read_int32(data);
        boardConfig.dce_ki = can_motor->config.ctrl_params.dce.ki;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x1A:  /* Set DCE Kd */
        CAN_CRITICAL_ENTER();
        can_motor->config.ctrl_params.dce.kd = can_read_int32(data);
        boardConfig.dce_kd = can_motor->config.ctrl_params.dce.kd;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    case 0x1B:  /* Set Enable Stall-Protect */
        CAN_CRITICAL_ENTER();
        can_motor->config.ctrl_params.stall_protect_switch = (can_read_uint32(data) == 1U);
        boardConfig.enable_stall_protect = can_motor->config.ctrl_params.stall_protect_switch;
        if (data[4] != 0U)
        {
            boardConfig.config_status = CONFIG_COMMIT;
        }
        CAN_CRITICAL_EXIT();
        break;

    /* ---- 0x20~0x2F 查询命令 ---- */
    case 0x21:  /* Get Current (mA) */
        tmp_f = motion_ctrl_get_foc_current(ctrl);
        memcpy(reply, &tmp_f, sizeof(tmp_f));
        reply[4] = (uint8_t)((ctrl->state == MOTOR_STATE_FINISH) ? 1 : 0);
        (void)can_send(can_reply_id(0x21U), reply);
        break;

    case 0x22:  /* Get Velocity (steps/s) */
        tmp_f = motion_ctrl_get_velocity(ctrl);
        memcpy(reply, &tmp_f, sizeof(tmp_f));
        reply[4] = (uint8_t)((ctrl->state == MOTOR_STATE_FINISH) ? 1 : 0);
        (void)can_send(can_reply_id(0x22U), reply);
        break;

    case 0x23:  /* Get Position (rev, 浮点) */
        tmp_f = motion_ctrl_get_position(ctrl, false);
        memcpy(reply, &tmp_f, sizeof(tmp_f));
        reply[4] = (uint8_t)((ctrl->state == MOTOR_STATE_FINISH) ? 1 : 0);
        (void)can_send(can_reply_id(0x23U), reply);
        break;

    case 0x24:  /* Get Offset (steps, 整数) */
    {
        int32_t offset = can_motor->config.motion_params.encoder_home_offset;

        memcpy(reply, &offset, sizeof(offset));
        (void)can_send(can_reply_id(0x24U), reply);
        break;
    }

    case 0x7E:  /* Erase Configs: 100 Hz 线程写 Flash 后复位 */
        boardConfig.config_status = CONFIG_RESTORE;
        break;

    case 0x7F:  /* Reboot */
        HAL_NVIC_SystemReset();
        break;

    default:
        break;
    }
}

/* ========================================================================== */
/* 帧分发 (节点号过滤, 上游在 HAL_CAN_RxFifo0MsgPendingCallback 里做)          */
/* ========================================================================== */

static void can_protocol_dispatch(const CanRxFrame *frame)
{
    uint8_t node = (uint8_t)(frame->std_id >> 7);   /* 4 bit 节点号 */
    uint8_t cmd = (uint8_t)(frame->std_id & 0x7FU); /* 7 bit 命令号 */

    if ((node != 0U) && (node != (uint8_t)boardConfig.can_node_id))
    {
        can_protocol_filter_count++;

        return;
    }

    can_protocol_cmd_count++;
    can_protocol_on_cmd(cmd, (uint8_t *)frame->data, frame->dlc);
}

static void can_protocol_thread_entry(void *parameter)
{
    (void)parameter;

    for (;;)
    {
        CanRxFrame frame;

        if (rt_sem_take(&can_rx_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        while (can_rx_tail != can_rx_head)
        {
            frame = can_rx_queue[can_rx_tail];
            can_rx_tail = (can_rx_tail + 1U) % CAN_RX_QUEUE_LEN;

            can_protocol_dispatch(&frame);
        }
    }
}

/* ========================================================================== */
/* 中断入口 (上游: HAL_CAN_RxFifo0MsgPendingCallback)                          */
/* ========================================================================== */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    CAN_RxHeaderTypeDef header;
    CanRxFrame frame;
    uint32_t queued = 0U;

    /*
     * 这个中断由 HAL_CAN_IRQHandler 进来, RT-Thread 的向量钩子并不知道,
     * 必须显式 enter/leave: 否则 rt_sem_release() 里判断"是否在中断中"
     * 会得到 0, 从而尝试在 ISR 里做现场切换。
     */
    rt_interrupt_enter();

    /* 一次挂起中断里可能积了多帧, 取干净为止 */
    while (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, frame.data) == HAL_OK)
    {
        uint32_t head = can_rx_head;
        uint32_t next = (head + 1U) % CAN_RX_QUEUE_LEN;

        frame.std_id = header.StdId;
        frame.dlc = (uint8_t)header.DLC;

        /* 短帧尾部清零, 避免上一条帧的残留被当成有效数据 */
        if (frame.dlc < 8U)
        {
            memset(&frame.data[frame.dlc], 0, sizeof(frame.data) - frame.dlc);
        }

        can_protocol_rx_count++;

        if (next == can_rx_tail)
        {
            can_protocol_drop_count++;
            continue;
        }

        can_rx_queue[head] = frame;
        can_rx_head = next;
        queued++;
    }

    if (queued != 0U)
    {
        (void)rt_sem_release(&can_rx_sem);
    }

    rt_interrupt_leave();
}

/* ========================================================================== */
/* 初始化                                                                     */
/* ========================================================================== */

/*
 * 上游在 MX_CAN_Init 的 USER CODE 2 里做这三件事。本工程 MX_CAN_Init()
 * 由 rt_hw_board_init() 调用 (早于 Flash 配置载入), 所以搬到这里。
 */
static bool can_filter_config(void)
{
    CAN_FilterTypeDef filter;

    /* 全通: 与上游一致, 节点号在软件里比对 (0 号广播给所有节点) */
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        rt_kprintf("can: filter config failed\n");

        return false;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        rt_kprintf("can: start failed\n");

        return false;
    }

    /* 只要 RX FIFO0 挂起; 错误/溢出中断不开, 省掉无谓的中断开销 */
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        rt_kprintf("can: notification enable failed\n");

        return false;
    }

    return true;
}

void can_protocol_init(Motor *motor, EncoderCalibrator *calibrator)
{
    if (can_inited)
    {
        return;
    }

    can_motor = motor;
    can_calibrator = calibrator;
    can_rx_head = 0U;
    can_rx_tail = 0U;

    rt_sem_init(&can_rx_sem, "canrx", 0, RT_IPC_FLAG_FIFO);

    rt_thread_init(&can_thread, "can", can_protocol_thread_entry, RT_NULL,
                   can_thread_stack, sizeof(can_thread_stack),
                   CAN_THREAD_PRIORITY, CAN_THREAD_TICK);
    rt_thread_startup(&can_thread);

    if (can_filter_config())
    {
        /* 节点号是总线上最容易配错的一项, 上电打一行便于对照 */
        rt_kprintf("can     : ready, 1Mbit/s, node=%u (filters everything, matches in sw)\n",
                   (unsigned)boardConfig.can_node_id);
    }

    can_inited = true;
}
