#
# FOC (Dummy-Robot Ctrl-Step-Driver) build integration
#
# 重要:
#   1. 本模块是纯 C 实现, 不启用 C++ (不需要 enable_language(CXX))
#   2. 源文件使用【显式列表】而不是 GLOB_RECURSE, 原因:
#        - FOC/ 下同时保存了上游的 *.cpp 作为转换参照, 它们不能参与构建
#        - Port/Platform/retarget.c 与 Core/Src/syscalls.c + rt_board.c 冲突
#        - 转换进度可见: 每完成一个模块就把它的 .c 加进 FOC_SRC
#

set(FOC_DIR ${CMAKE_SOURCE_DIR}/FOC)

if(NOT EXISTS ${FOC_DIR}/Ctrl/Motor/motor.h)
    message(FATAL_ERROR "FOC sources not found at ${FOC_DIR}")
endif()

# ---------------------------------------------------------------------------
# 已完成 C 化的源文件
#
# 阶段 A (骨架): 空列表, 保证构建仍然可用
# 阶段 B 按依赖顺序逐个放开:
#   信号层   Ctrl/Signal/button_base.c   Ctrl/Signal/led_base.c
#            Port/button_stm32.c        Port/led_stm32.c
#   驱动层   Ctrl/Driver/sin_map.c       Ctrl/Driver/tb67h450_base.c
#            Port/tb67h450_stm32.c
#   编码器   Ctrl/Sensor/Encoder/mt6816_base.c   Port/mt6816_stm32.c
#   规划器   Ctrl/Motor/motion_planner.c
#   电机     Ctrl/Motor/motor.c
#   校准器   Ctrl/Sensor/Encoder/encoder_calibrator_base.c
#            Port/encoder_calibrator_stm32.c
#   存储层   Port/Platform/Memory/emulated_eeprom.c
#            Port/Platform/Memory/stockpile_f103cb.c
#   应用层   Port/Platform/Utils/st_hardware.c
#            UserApp/app_foc.c            UserApp/protocols/interface_can.c
# ---------------------------------------------------------------------------
set(FOC_SRC
    # --- 阶段 B: 信号层 (纯逻辑, 无 HAL 依赖) ---
    ${FOC_DIR}/Ctrl/Signal/button_base.c
    ${FOC_DIR}/Ctrl/Signal/led_base.c

    # --- 阶段 B: 信号层 Port (依赖 CubeMX 引脚标签, 阶段 D 后启用) ---
    # ${FOC_DIR}/Port/button_stm32.c   # 需要 BUTTON1_Pin / BUTTON2_Pin
    # ${FOC_DIR}/Port/led_stm32.c      # 需要 LED1_Pin / LED2_Pin
)

# ---------------------------------------------------------------------------
# 明确排除 (不要加入 FOC_SRC)
#   Port/Platform/retarget.c              _write / __io_putchar 与
#                                         Core/Src/syscalls.c + rt_board.c 重复定义
#   UserApp/protocols/interface_uart.cpp  USART1 由 FinSH 独占
#   UserApp/main.cpp                      改名 app_foc.c (避免与 Core/Src/main.c 冲突)
#   Ctrl/**/*.cpp  Port/**/*.cpp          C++ 参照实现, 不参与构建
# ---------------------------------------------------------------------------

if(FOC_SRC)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${FOC_SRC})

    # 20 kHz 闭环跑在 TIM4 中断里; Debug 预设是 -O0,
    # 单次 tick 可能超过 50 us 预算, 因此 FOC 源固定用 -Os。
    # COMPILE_OPTIONS 追加在构建类型 flags 之后, 后者生效。
    set_source_files_properties(${FOC_SRC} PROPERTIES COMPILE_OPTIONS "-Os")
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FOC_DIR}                  # Platform/... , Sensor/... 这种上游写法
    ${FOC_DIR}/Ctrl
    ${FOC_DIR}/Port
    ${FOC_DIR}/UserApp
)
