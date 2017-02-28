# Add Source files 
SRCS = main.c system_stm32f4xx.c funcs1.s FreeRTOS-openOCD.c

ACCELEROMETER_SRCS = accelerometers/accelerometers.c accelerometers/tm_accelerometers/tm_stm32f4_lis302dl_lis3dsh.c accelerometers/tm_accelerometers/tm_stm32f4_spi.c
SRCS += $(ACCELEROMETER_SRCS)

RNG_SRCS = RNG/random_number_generator.c
SRCS += $(RNG_SRCS)

TEMPERATURE_SRCS = temperature/temperature.c
SRCS += $(TEMPERATURE_SRCS)

USART_SRCS = usart/usart.c
SRCS += $(USART_SRCS)

CIRCARR_SRCS = circarray/circarray.c
SRCS += $(CIRCARR_SRCS)

I2C_SRCS = i2c/i2c.c
SRCS += $(I2C_SRCS)


# Project Name. Binary will be generated with this name (as .elf)
PROJ_NAME=NYUSat

# Beware changing anything below this line! (should be rare)
#######################################################################################

#build directory used to store .o files
BUILDDIR = build

# STM32F4 library code directory
STM_COMMON=STM32F4-Discovery_FW_V1.1.0

#FreeRTOS directory
FREERTOS = FreeRTOS
SRCS += $(FREERTOS)/event_groups.c $(FREERTOS)/list.c $(FREERTOS)/portable/GCC/ARM_CM4F/port.c $(FREERTOS)/queue.c $(FREERTOS)/tasks.c $(FREERTOS)/timers.c $(FREERTOS)/portable/MemMang/heap_4.c

# use these Tools
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE=arm-none-eabi-size
ifeq ($(OS),Windows_NT)
MKDIR=busybox mkdir
else
MKDIR=mkdir
endif

#CFLAGS
###############################
CFLAGS  = -ggdb -O0 -Wall

CFLAGS += -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -D__FPU_PRESENT=1 -D__FPU_USED=1 -DSTM32F40_41xxx -DUSE_STM32F4_DISCO -DSTM32F4XX -DARM_MATH_CM4

CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -Xassembler -mimplicit-it=always
CFLAGS += -fsingle-precision-constant -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.
CFLAGS += -std=gnu99

CFLAGS += -I$(FREERTOS)/include
CFLAGS += -I$(FREERTOS)/portable/GCC/ARM_CM4F

# Include files from STM32 libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include 
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc


# Linker script ld file
LDSCRIPT = stm32_flash.ld
# add it to linker
LDFLAGS += -T$(LDSCRIPT)

# add STM32 Startup file to srcs
SRCS += $(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Source/Templates/TrueSTUDIO/startup_stm32f4xx.s

# add required files from STM32 standard peripheral library
STM_SRCDIR = $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src
SRCS += $(STM_SRCDIR)/stm32f4xx_gpio.c $(STM_SRCDIR)/stm32f4xx_spi.c $(STM_SRCDIR)/stm32f4xx_rcc.c $(STM_SRCDIR)/stm32f4xx_rng.c $(STM_SRCDIR)/stm32f4xx_adc.c $(STM_SRCDIR)/stm32f4xx_exti.c $(STM_SRCDIR)/stm32f4xx_usart.c $(STM_SRCDIR)/misc.c $(STM_SRCDIR)/stm32f4xx_i2c.c

OBJS = $(addprefix $(BUILDDIR)/, $(addsuffix .o, $(basename $(SRCS))))

SEMIHOSTING_FLAGS = --specs=rdimon.specs -lc -lrdimon -u _printf_float

ELF = $(PROJ_NAME).elf
BIN = $(PROJ_NAME).bin

$(BUILDDIR)/%.o: %.c
	$(MKDIR) -p $(dir $@)
	$(CC) -c $(SEMIHOSTING_FLAGS) $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.s
	$(MKDIR) -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY: all

all: $(ELF) $(BIN)

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) $(SEMIHOSTING_FLAGS) $(CFLAGS) -o $@ $(OBJS) $(STM_COMMON)/Libraries/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a
	$(SIZE) $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@


clean:
	rm -f $(ELF) $(BIN) $(OBJS)


#######################################################
# Debugging targets
#######################################################
gdb: all
	arm-none-eabi-gdb -tui $(ELF)

# Start OpenOCD GDB server (supports semihosting)
openocd: 
	openocd -f board/stm32f4discovery.cfg 

flash: $(BIN)
	st-flash write $(BIN) 0x8000000

