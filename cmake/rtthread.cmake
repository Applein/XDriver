#
# RT-Thread (kernel + FinSH) build integration
#
# 重要: 内核与 FinSH 源文件必须直接编进最终可执行文件, 不能打成静态库,
#       否则 .rti_fn.* / FSymTab 这类自动初始化段会被链接器丢弃。
#

set(RTTHREAD_DIR ${CMAKE_SOURCE_DIR}/RT-Thread)

if(NOT EXISTS ${RTTHREAD_DIR}/src/components.c)
    message(FATAL_ERROR "RT-Thread sources not found at ${RTTHREAD_DIR}")
endif()

set(RTTHREAD_KERNEL_SRC
    ${RTTHREAD_DIR}/src/clock.c
    ${RTTHREAD_DIR}/src/components.c
    ${RTTHREAD_DIR}/src/cpu.c
    ${RTTHREAD_DIR}/src/device.c
    ${RTTHREAD_DIR}/src/idle.c
    ${RTTHREAD_DIR}/src/ipc.c
    ${RTTHREAD_DIR}/src/irq.c
    ${RTTHREAD_DIR}/src/kservice.c
    ${RTTHREAD_DIR}/src/mem.c
    ${RTTHREAD_DIR}/src/memheap.c
    ${RTTHREAD_DIR}/src/mempool.c
    ${RTTHREAD_DIR}/src/object.c
    ${RTTHREAD_DIR}/src/scheduler.c
    ${RTTHREAD_DIR}/src/signal.c
    ${RTTHREAD_DIR}/src/slab.c
    ${RTTHREAD_DIR}/src/thread.c
    ${RTTHREAD_DIR}/src/timer.c
)

set(RTTHREAD_CPU_SRC
    ${RTTHREAD_DIR}/libcpu/arm/cortex-m3/cpuport.c
    ${RTTHREAD_DIR}/libcpu/arm/cortex-m3/context_gcc.S
)

set(RTTHREAD_FINSH_SRC
    ${RTTHREAD_DIR}/components/finsh/cmd.c
    ${RTTHREAD_DIR}/components/finsh/msh.c
    ${RTTHREAD_DIR}/components/finsh/msh_parse.c
    ${RTTHREAD_DIR}/components/finsh/shell.c
)

set(RTTHREAD_BOARD_SRC
    ${CMAKE_SOURCE_DIR}/Core/Src/rt_board.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${RTTHREAD_KERNEL_SRC}
    ${RTTHREAD_CPU_SRC}
    ${RTTHREAD_FINSH_SRC}
    ${RTTHREAD_BOARD_SRC}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}                                 # rtconfig.h
    ${RTTHREAD_DIR}/include
    ${RTTHREAD_DIR}/components/finsh
    ${RTTHREAD_DIR}/libcpu/arm/cortex-m3
)
