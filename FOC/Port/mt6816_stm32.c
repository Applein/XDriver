/*
 * mt6816_stm32.c
 *
 * 上游: Port/mt6816_stm32.cpp
 *
 * 片选 (PA15) 用 BSRR/BRR 直接操作: 单条指令, 适合 20 kHz 路径。
 * spi_init 保留上游的 MX_SPI1_Init() 调用 (幂等, 不涉及计数器状态)。
 */

#include "mt6816_stm32.h"

#include "main.h"
#include "spi.h"

#include "Platform/Memory/stockpile_config.h"

/* -------------------------------------------------------------------------- */
/* 诊断状态与闩锁标志 (定义在此处, spi_init 需要提前访问)                      */
/* -------------------------------------------------------------------------- */
volatile uint32_t mt6816_spi_fail_count = 0;
volatile uint32_t mt6816_spi_err_txe = 0;
volatile uint32_t mt6816_spi_err_rxne = 0;
volatile uint32_t mt6816_spi_err_bsy = 0;
volatile uint16_t mt6816_spi_last_sr = 0;
volatile uint16_t mt6816_spi_last_cr1 = 0;

static bool     mt6816_spi_dead = false;
static uint32_t mt6816_spi_skip = 0;
static uint16_t mt6816_spi_fail_sentinel = 0x0001U;

/* -------------------------------------------------------------------------- */
/* 上游: void MT6816::SpiInit()  { MX_SPI1_Init(); }                           */
/* -------------------------------------------------------------------------- */
static void mt6816_stm32_spi_init(void)
{
    MX_SPI1_Init();

    /* 复位诊断状态: 重新初始化 SPI 视为"清故障", 允许立即重试 */
    mt6816_spi_dead = false;
    mt6816_spi_skip = 0;
    mt6816_spi_fail_sentinel = 0x0001U;

    mt6816_spi_fail_count = 0;
    mt6816_spi_err_txe = 0;
    mt6816_spi_err_rxne = 0;
    mt6816_spi_err_bsy = 0;
}

/* -------------------------------------------------------------------------- */
/* 上游: uint16_t MT6816::SpiTransmitAndRead16Bits(uint16_t _dataTx)
 *
 *   PA15 低 -> 收发 1 个 16bit -> PA15 高
 *
 * 为什么不用 HAL_SPI_TransmitReceive():
 *
 *   本函数在 TIM4 中断 (抢占优先级 0) 里被 20 kHz 调用, 而 HAL 的超时判定
 *   依赖 HAL_GetTick(), 后者由 TIM1_UP 递增 —— TIM1_UP 是优先级 15, 永远
 *   抢不到 TIM4 的 CPU。也就是说, 超时值填多少都不会生效:
 *     - 编码器掉线 / SPI 卡在 BSY  -> 死等, USART1(优先级 5) 收不到中断
 *     - 死等发生在最高优先级 ISR 里 -> 整机锁死, 只有复位能救
 *
 * 两道保险 (缺一不可):
 *
 *  1) 有界等待: 三个等待循环都用固定迭代次数做上界, 不依赖任何时基。
 *     200 次 x 约 10 周期 @72MHz 约 28us, 是正常传输 (1.78us) 的 15 倍。
 *
 *  2) 失败闩锁 (mt6816_spi_dead): 一旦某次超时, 后续调用立即返回,
 *     每 MT6816_SPI_RETRY_TICKS 次才真正重试一次。
 *     这是关键 —— 单次 update_angle() 会做 3 次重试 x 2 次传输 = 6 次调用,
 *     若每次都真等到超时, 一个 50us 的控制周期会被吃掉数百 us, CPU 被
 *     最高优先级 ISR 占满, 串口输入彻底收不到 (症状: msh 敲什么都没反应)。
 *     有了闩锁, SPI 坏掉时稳态开销为 0。
 *
 * 失败返回值: 0x0001 / 0x0000 交替。
 *   上游校验规则是"16bit 里 1 的个数必须为偶数"。若失败统一返回 0,
 *   则 raw_data = 0 恰好是偶数 -> 校验"通过" -> 角度被读成 0 -> 电机猛跳。
 *   交替返回 0 和 1, 拼接出的 raw_data 为 0x0001 或 0x0100 (只有 1 个 bit),
 *   必然奇校验失败 -> 上游沿用上一次的角度, 电机不会突跳。
 * -------------------------------------------------------------------------- */
#define MT6816_SPI_GUARD            400U        /* 单次等待的迭代上界 (~33us) */
#define MT6816_SPI_RETRY_TICKS      64U         /* 闩锁后每 N 次调用重试一次 */

/*
 * 恢复 SPI 外设
 *
 * 为什么需要: F1 的 SPI 一旦置位 OVR (接收溢出) 或 MODF (模式错误),
 * 接收器就停摆, 之后再怎么等 RXNE 都不会来 —— 这正是"一直超时"的成因。
 * 手册给出的清除办法是把 SPE 清掉再置位 (禁用 -> 读 SR/DR -> 重新使能)。
 * 不做这一步, 一次故障就会让 SPI 永久失效, 只能靠复位板子恢复。
 */
static void mt6816_spi_recover(void)
{
    SPI1->CR1 &= ~SPI_CR1_SPE;      /* 关闭 SPI, 复位内部状态机 */

    (void)SPI1->SR;                 /* 清 OVR: 先读 SR 再读 DR */
    (void)SPI1->DR;

    SPI1->CR1 |= SPI_CR1_SPE;       /* 重新使能 */
}

/* 失败时的返回值: 0x0001 / 0x0000 交替, 保证拼接后奇校验必然失败 */
static uint16_t mt6816_spi_fail_value(void)
{
    uint16_t value = mt6816_spi_fail_sentinel;

    mt6816_spi_fail_sentinel ^= 0x0001U;

    return value;
}

static uint16_t mt6816_stm32_spi_transmit_and_read16(uint16_t data_tx)
{
    uint16_t data_rx = 0;
    uint32_t guard;
    bool ok = true;

    /* 闩锁生效中: 绝大多数调用直接返回, 只偶尔试探一次是否已恢复 */
    if (mt6816_spi_dead)
    {
        if (++mt6816_spi_skip < MT6816_SPI_RETRY_TICKS)
        {
            return mt6816_spi_fail_value();
        }
        mt6816_spi_skip = 0;
    }

    GPIOA->BRR = (uint32_t)SPI1_CS_Pin;         /* 片选有效 (低) */

    /* 清掉上一轮可能残留的 RXNE/OVR: 读 SR 再读 DR (F1 手册规定顺序) */
    (void)SPI1->SR;
    (void)SPI1->DR;

    /* 1) 等发送寄存器空 */
    guard = MT6816_SPI_GUARD;
    while ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
        if (--guard == 0U)
        {
            ok = false;
            mt6816_spi_err_txe++;
            break;
        }
    }

    if (ok)
    {
        SPI1->DR = data_tx;

        /* 2) 等接收寄存器非空 */
        guard = MT6816_SPI_GUARD;
        while ((SPI1->SR & SPI_SR_RXNE) == 0U)
        {
            if (--guard == 0U)
            {
                ok = false;
                mt6816_spi_err_rxne++;
                break;
            }
        }
    }

    if (ok)
    {
        data_rx = (uint16_t)SPI1->DR;
    }

    /* 3) 等总线空闲, 否则下一次片选会踩到上一帧的尾巴 */
    guard = MT6816_SPI_GUARD;
    while ((SPI1->SR & SPI_SR_BSY) != 0U)
    {
        if (--guard == 0U)
        {
            ok = false;
            mt6816_spi_err_bsy++;
            break;
        }
    }

    GPIOA->BSRR = (uint32_t)SPI1_CS_Pin;        /* 片选无效 (高) */

    if (!ok)
    {
        /* 快照出错瞬间的寄存器, 供 xdrv status 判读 */
        mt6816_spi_last_sr = (uint16_t)SPI1->SR;
        mt6816_spi_last_cr1 = (uint16_t)SPI1->CR1;

        mt6816_spi_fail_count++;
        mt6816_spi_dead = true;                 /* 进入闩锁, 后续调用零开销 */

        /* 立刻尝试恢复外设, 避免 OVR/MODF 让 SPI 永久停摆 */
        mt6816_spi_recover();

        return mt6816_spi_fail_value();
    }

    mt6816_spi_dead = false;

    return data_rx;
}

const MT6816PortOps mt6816_stm32_port_ops =
{
    .spi_init = mt6816_stm32_spi_init,
    .spi_transmit_and_read16 = mt6816_stm32_spi_transmit_and_read16,
};

void mt6816_stm32_setup(MT6816Base *self)
{
    /* 上游: explicit MT6816() : MT6816Base((uint16_t*)(STOCKPILE_APP_CALI_ADDR)) */
    mt6816_base_init(self, &mt6816_stm32_port_ops,
                     (uint16_t *)(STOCKPILE_APP_CALI_ADDR));
}
