/*
 * encoder_calibrator_base.c
 *
 * 上游: Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp
 *
 * 逐段移植。要点:
 *   1. 上游 #include <valarray> 无用, 已删除。
 *   2. 所有 Cycle* 辅助函数改为文件内 static。
 *   3. `motor->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` / `motor->SOFT_DIVIDE_NUM` /
 *      `motor->MOTOR_ONE_CIRCLE_HARD_STEPS` 这类"通过对象访问常量"的写法
 *      改成直接使用 motor_types.h 里的枚举。
 *   4. 循环内的整数除法顺序 (SOFT_DIVIDE_NUM * stepY / dataI32) 保持原样,
 *      改了会引入舍入差异, 影响校准表。
 */

#include "Sensor/Encoder/encoder_calibrator_base.h"

#include <stdlib.h>     /* abs */

#include "main.h"       /* HAL_NVIC_SystemReset (校准成功后复位) */

/* -------------------------------------------------------------------------- */
/* 上游: static uint32_t EncoderCalibratorBase::CycleMod(uint32_t, uint32_t)
 * -------------------------------------------------------------------------- */
static uint32_t cycle_mod(uint32_t a, uint32_t b)
{
    return (a + b) % b;
}

/* 上游: static int32_t EncoderCalibratorBase::CycleSubtract(int32_t, int32_t, int32_t) */
static int32_t cycle_subtract(int32_t a, int32_t b, int32_t cyc)
{
    int32_t sub_data;

    sub_data = a - b;
    if (sub_data > (cyc >> 1))
    {
        sub_data -= cyc;
    }
    if (sub_data < (-cyc >> 1))
    {
        sub_data += cyc;
    }

    return sub_data;
}

/* 上游: static int32_t EncoderCalibratorBase::CycleAverage(int32_t, int32_t, int32_t) */
static int32_t cycle_average(int32_t a, int32_t b, int32_t cyc)
{
    int32_t sub_data;
    int32_t ave_data;

    sub_data = a - b;
    ave_data = (a + b) >> 1;

    if (abs(sub_data) > (cyc >> 1))
    {
        if (ave_data >= (cyc >> 1))
        {
            ave_data -= (cyc >> 1);
        }
        else
        {
            ave_data += (cyc >> 1);
        }
    }

    return ave_data;
}

/* 上游: static int32_t EncoderCalibratorBase::CycleDataAverage(
 *           const uint16_t* _data, uint16_t _length, int32_t _cyc) */
static int32_t cycle_data_average(const uint16_t *data, uint16_t length, int32_t cyc)
{
    int32_t sum_data = 0;
    int32_t sub_data;
    int32_t diff_data;
    uint16_t i;

    sum_data += (int32_t)data[0];
    for (i = 1; i < length; i++)
    {
        diff_data = (int32_t)data[i];
        sub_data = (int32_t)data[i] - (int32_t)data[0];
        if (sub_data > (cyc >> 1))
        {
            diff_data = (int32_t)data[i] - cyc;
        }
        if (sub_data < (-cyc >> 1))
        {
            diff_data = (int32_t)data[i] + cyc;
        }
        sum_data += diff_data;
    }

    sum_data = sum_data / length;

    if (sum_data < 0)
    {
        sum_data += cyc;
    }
    if (sum_data > cyc)
    {
        sum_data -= cyc;
    }

    return sum_data;
}

/* -------------------------------------------------------------------------- */
/* 上游: void EncoderCalibratorBase::CalibrationDataCheck()                    */
/* -------------------------------------------------------------------------- */
static void calibration_data_check(EncoderCalibrator *self)
{
    uint32_t count;
    int32_t sub_data;
    int32_t calib_sample_resolution;
    uint32_t step_num;

    calib_sample_resolution = ENCODER_RESOLUTION / MOTOR_ONE_CIRCLE_HARD_STEPS;

    /* 正/反向采样取平均 */
    for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS + 1; count++)
    {
        self->sample_data_average_forward[count] =
            (uint16_t)cycle_average((int32_t)self->sample_data_average_forward[count],
                                    (int32_t)self->sample_data_average_backward[count],
                                    ENCODER_RESOLUTION);
    }

    sub_data = cycle_subtract((int32_t)self->sample_data_average_forward[0],
                              (int32_t)self->sample_data_average_forward
                                  [MOTOR_ONE_CIRCLE_HARD_STEPS - 1],
                              ENCODER_RESOLUTION);
    if (sub_data == 0)
    {
        self->error_code = CALI_ERR_AVERAGE_DIR;
        return;
    }
    else
    {
        self->go_direction = sub_data > 0;
    }

    /* 相邻步的增量连续性与方向一致性 */
    for (count = 1; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
    {
        sub_data = cycle_subtract((int32_t)self->sample_data_average_forward[count],
                                  (int32_t)self->sample_data_average_forward[count - 1],
                                  ENCODER_RESOLUTION);
        if (abs(sub_data) > (calib_sample_resolution * 3 / 2))    /* 增量过大 */
        {
            self->error_code = CALI_ERR_AVERAGE_CONTINUTY;
            return;
        }
        if (abs(sub_data) < (calib_sample_resolution * 1 / 2))    /* 增量过小 */
        {
            self->error_code = CALI_ERR_AVERAGE_CONTINUTY;
            return;
        }
        if (sub_data == 0)
        {
            self->error_code = CALI_ERR_AVERAGE_DIR;
            return;
        }
        if ((sub_data > 0) && (!self->go_direction))
        {
            self->error_code = CALI_ERR_AVERAGE_DIR;
            return;
        }
        if ((sub_data < 0) && (self->go_direction))
        {
            self->error_code = CALI_ERR_AVERAGE_DIR;
            return;
        }
    }

    /* 找相序跳变点 (rcdX / rcdY) */
    step_num = 0;
    if (self->go_direction)
    {
        for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
        {
            sub_data =
                (int32_t)self->sample_data_average_forward
                    [cycle_mod(count + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)] -
                (int32_t)self->sample_data_average_forward
                    [cycle_mod(count, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            if (sub_data < 0)
            {
                step_num++;
                self->rcd_x = (int32_t)count;
                self->rcd_y = (ENCODER_RESOLUTION - 1) -
                              self->sample_data_average_forward
                                  [cycle_mod((uint32_t)self->rcd_x,
                                             MOTOR_ONE_CIRCLE_HARD_STEPS)];
            }
        }
        if (step_num != 1)
        {
            self->error_code = CALI_ERR_PHASE_STEP;
            return;
        }
    }
    else
    {
        for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
        {
            sub_data =
                (int32_t)self->sample_data_average_forward
                    [cycle_mod(count + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)] -
                (int32_t)self->sample_data_average_forward
                    [cycle_mod(count, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            if (sub_data > 0)
            {
                step_num++;
                self->rcd_x = (int32_t)count;
                self->rcd_y = (ENCODER_RESOLUTION - 1) -
                              self->sample_data_average_forward
                                  [cycle_mod((uint32_t)self->rcd_x + 1,
                                             MOTOR_ONE_CIRCLE_HARD_STEPS)];
            }
        }
        if (step_num != 1)
        {
            self->error_code = CALI_ERR_PHASE_STEP;
            return;
        }
    }

    self->error_code = CALI_NO_ERROR;
}

/* 上游: void EncoderCalibratorBase::WriteCalibrationEntry(uint16_t _dataU16) */
static void write_calibration_entry(EncoderCalibrator *self, uint16_t data_u16)
{
    /* 全分辨率表: 每一项都要写
     * 16384 项 x uint16_t = 32KB, 存放在专用 APP_CALI 区 */
    self->flash->write16_append(data_u16);
    self->result_num++;
}

/* -------------------------------------------------------------------------- */
/* 上游: void EncoderCalibratorBase::Tick20kHz()                               */
/* -------------------------------------------------------------------------- */
void encoder_calibrator_tick_20khz(EncoderCalibrator *self)
{
    Motor *motor = self->motor;

    encoder_update_angle(motor->encoder);

    switch (self->state)
    {
    case CALI_STATE_DISABLE:
        if (self->is_triggered)
        {
            driver_set_foc_current_vector(motor->driver, self->go_position,
                                          motor->config.motion_params.cali_current);
            self->go_position = MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            self->sample_count = 0;
            self->state = CALI_STATE_FORWARD_PREPARE;
            self->error_code = CALI_NO_ERROR;
        }
        break;

    case CALI_STATE_FORWARD_PREPARE:
        self->go_position += CALI_AUTO_SPEED;
        driver_set_foc_current_vector(motor->driver, self->go_position,
                                      motor->config.motion_params.cali_current);
        if (self->go_position == 2 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
        {
            self->go_position = MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            self->state = CALI_STATE_FORWARD_MEASURE;
        }
        break;

    case CALI_STATE_FORWARD_MEASURE:
        if ((self->go_position % SOFT_DIVIDE_NUM) == 0)
        {
            self->sample_data_raw[self->sample_count++] =
                motor->encoder->angle_data.raw_angle;
            if (self->sample_count == CALI_SAMPLE_COUNTS_PER_STEP)
            {
                self->sample_data_average_forward[(self->go_position -
                                                   MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) /
                                                  SOFT_DIVIDE_NUM] =
                    (uint16_t)cycle_data_average(self->sample_data_raw,
                                                 CALI_SAMPLE_COUNTS_PER_STEP,
                                                 ENCODER_RESOLUTION);

                self->sample_count = 0;
                self->go_position += CALI_FINE_TUNE_CALIB_SPEED;
            }
        }
        else
        {
            self->go_position += CALI_FINE_TUNE_CALIB_SPEED;
        }

        driver_set_foc_current_vector(motor->driver, self->go_position,
                                      motor->config.motion_params.cali_current);

        if (self->go_position > (2 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS))
        {
            self->state = CALI_STATE_BACKWARD_RETURN;
        }
        break;

    case CALI_STATE_BACKWARD_RETURN:
        self->go_position += CALI_FINE_TUNE_CALIB_SPEED;
        driver_set_foc_current_vector(motor->driver, self->go_position,
                                      motor->config.motion_params.cali_current);

        if (self->go_position ==
            (2 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS + SOFT_DIVIDE_NUM * 20))
        {
            self->state = CALI_STATE_BACKWARD_GAP_DISMISS;
        }
        break;

    case CALI_STATE_BACKWARD_GAP_DISMISS:
        self->go_position -= CALI_FINE_TUNE_CALIB_SPEED;
        driver_set_foc_current_vector(motor->driver, self->go_position,
                                      motor->config.motion_params.cali_current);
        if (self->go_position == (2 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS))
        {
            self->state = CALI_STATE_BACKWARD_MEASURE;
        }
        break;

    case CALI_STATE_BACKWARD_MEASURE:
        if ((self->go_position % SOFT_DIVIDE_NUM) == 0)
        {
            self->sample_data_raw[self->sample_count++] =
                motor->encoder->angle_data.raw_angle;
            if (self->sample_count == CALI_SAMPLE_COUNTS_PER_STEP)
            {
                self->sample_data_average_backward[(self->go_position -
                                                    MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS) /
                                                   SOFT_DIVIDE_NUM] =
                    (uint16_t)cycle_data_average(self->sample_data_raw,
                                                 CALI_SAMPLE_COUNTS_PER_STEP,
                                                 ENCODER_RESOLUTION);

                self->sample_count = 0;
                self->go_position -= CALI_FINE_TUNE_CALIB_SPEED;
            }
        }
        else
        {
            self->go_position -= CALI_FINE_TUNE_CALIB_SPEED;
        }

        driver_set_foc_current_vector(motor->driver, self->go_position,
                                      motor->config.motion_params.cali_current);

        if (self->go_position < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
        {
            self->state = CALI_STATE_CALCULATING;
        }
        break;

    case CALI_STATE_CALCULATING:
        driver_set_foc_current_vector(motor->driver, 0, 0);
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
/* 上游: void EncoderCalibratorBase::TickMainLoop()
 *   注意: 本函数会擦除并写入 32KB Flash, 期间 CPU 被阻塞几十毫秒。
 *         只能在电机处于休眠/停止状态时调用 (上游同样只在 CALI_CALCULATING 时进入)。
 * -------------------------------------------------------------------------- */
void encoder_calibrator_tick_main_loop(EncoderCalibrator *self)
{
    Motor *motor = self->motor;
    int32_t data_i32;
    uint16_t data_u16;
    int32_t step_x;
    int32_t step_y;

    if (self->state != CALI_STATE_CALCULATING)
    {
        return;
    }

    driver_sleep(motor->driver);

    calibration_data_check(self);

    if (self->error_code == CALI_NO_ERROR)
    {
        self->result_num = 0;

        self->flash->clear();
        self->flash->begin_write();

        if (self->go_direction)
        {
            for (step_x = self->rcd_x;
                 step_x < self->rcd_x + MOTOR_ONE_CIRCLE_HARD_STEPS + 1;
                 step_x++)
            {
                data_i32 = cycle_subtract(
                    (int32_t)self->sample_data_average_forward
                        [cycle_mod((uint32_t)(step_x + 1), MOTOR_ONE_CIRCLE_HARD_STEPS)],
                    (int32_t)self->sample_data_average_forward
                        [cycle_mod((uint32_t)step_x, MOTOR_ONE_CIRCLE_HARD_STEPS)],
                    ENCODER_RESOLUTION);

                if (step_x == self->rcd_x)
                {
                    for (step_y = self->rcd_y; step_y < data_i32; step_y++)
                    {
                        data_u16 = (uint16_t)cycle_mod(
                            (uint32_t)(SOFT_DIVIDE_NUM * step_x +
                                       SOFT_DIVIDE_NUM * step_y / data_i32),
                            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
                        write_calibration_entry(self, data_u16);
                    }
                }
                else if (step_x == self->rcd_x + MOTOR_ONE_CIRCLE_HARD_STEPS)
                {
                    for (step_y = 0; step_y < self->rcd_y; step_y++)
                    {
                        data_u16 = (uint16_t)cycle_mod(
                            (uint32_t)(SOFT_DIVIDE_NUM * step_x +
                                       SOFT_DIVIDE_NUM * step_y / data_i32),
                            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
                        write_calibration_entry(self, data_u16);
                    }
                }
                else
                {
                    for (step_y = 0; step_y < data_i32; step_y++)
                    {
                        data_u16 = (uint16_t)cycle_mod(
                            (uint32_t)(SOFT_DIVIDE_NUM * step_x +
                                       SOFT_DIVIDE_NUM * step_y / data_i32),
                            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
                        write_calibration_entry(self, data_u16);
                    }
                }
            }
        }
        else
        {
            for (step_x = self->rcd_x + MOTOR_ONE_CIRCLE_HARD_STEPS;
                 step_x > self->rcd_x - 1;
                 step_x--)
            {
                data_i32 = cycle_subtract(
                    (int32_t)self->sample_data_average_forward
                        [cycle_mod((uint32_t)step_x, MOTOR_ONE_CIRCLE_HARD_STEPS)],
                    (int32_t)self->sample_data_average_forward
                        [cycle_mod((uint32_t)(step_x + 1), MOTOR_ONE_CIRCLE_HARD_STEPS)],
                    ENCODER_RESOLUTION);

                if (step_x == self->rcd_x + MOTOR_ONE_CIRCLE_HARD_STEPS)
                {
                    for (step_y = self->rcd_y; step_y < data_i32; step_y++)
                    {
                        data_u16 = (uint16_t)cycle_mod(
                            (uint32_t)(SOFT_DIVIDE_NUM * (step_x + 1) -
                                       SOFT_DIVIDE_NUM * step_y / data_i32),
                            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
                        write_calibration_entry(self, data_u16);
                    }
                }
                else if (step_x == self->rcd_x)
                {
                    for (step_y = 0; step_y < self->rcd_y; step_y++)
                    {
                        data_u16 = (uint16_t)cycle_mod(
                            (uint32_t)(SOFT_DIVIDE_NUM * (step_x + 1) -
                                       SOFT_DIVIDE_NUM * step_y / data_i32),
                            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
                        write_calibration_entry(self, data_u16);
                    }
                }
                else
                {
                    for (step_y = 0; step_y < data_i32; step_y++)
                    {
                        data_u16 = (uint16_t)cycle_mod(
                            (uint32_t)(SOFT_DIVIDE_NUM * (step_x + 1) -
                                       SOFT_DIVIDE_NUM * step_y / data_i32),
                            MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
                        write_calibration_entry(self, data_u16);
                    }
                }
            }
        }

        self->flash->end_write();

        if (self->result_num < (uint32_t)(ENCODER_RESOLUTION / 4))
        {
            self->error_code = CALI_ERR_ANALYSIS_QUANTITY;
        }
    }

    if (self->error_code == CALI_NO_ERROR)
    {
        motor->encoder->angle_data.rectify_valid = true;
        HAL_NVIC_SystemReset();
    }
    else
    {
        motor->encoder->angle_data.rectify_valid = false;
        self->flash->clear();
    }

    motor->controller->is_stalled = true;

    self->state = CALI_STATE_DISABLE;
    self->is_triggered = false;
}

/* -------------------------------------------------------------------------- */
/* 上游: explicit EncoderCalibratorBase(Motor* _motor)
 *   构造函数只做: motor = _motor; isTriggered=false; errorCode=CALI_NO_ERROR;
 *                 state=CALI_DISABLE; goPosition=0; rcdX=0; rcdY=0; resultNum=0;
 *   注意上游没有初始化 goDirection / sampleCount 的成员初始化器
 *        (sampleCount 有 = 0), goDirection 是未定义值 —— 但它在
 *        calibration_data_check() 里一定会被赋值后才使用, 这里统一清零更安全。
 * -------------------------------------------------------------------------- */
void encoder_calibrator_init(EncoderCalibrator *self, Motor *motor,
                             const CaliFlashOps *flash)
{
    uint32_t i;

    self->motor = motor;
    self->flash = flash;

    self->is_triggered = false;
    self->error_code = CALI_NO_ERROR;
    self->state = CALI_STATE_DISABLE;

    self->go_position = 0;
    self->go_direction = false;

    self->sample_count = 0;
    for (i = 0; i < CALI_SAMPLE_COUNTS_PER_STEP; i++)
    {
        self->sample_data_raw[i] = 0;
    }
    for (i = 0; i < MOTOR_ONE_CIRCLE_HARD_STEPS + 1; i++)
    {
        self->sample_data_average_forward[i] = 0;
        self->sample_data_average_backward[i] = 0;
    }

    self->rcd_x = 0;
    self->rcd_y = 0;
    self->result_num = 0;
}
