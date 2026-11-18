# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = pcan_$(BOARD)_hw
TARGET_VARIANT = $(shell echo $(BOARD) | tr  '[:lower:]' '[:upper:]')

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build-$(BOARD)

######################################
# mcu selection (can be overridden per board)
######################################
MCU_SERIES ?= STM32F072xB
STARTUP    ?= startup_stm32f072xb.s
LDSCRIPT   ?= STM32F072CBTx_FLASH.ld

######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/usbd_conf.c \
Src/usbd_desc.c \
Src/pcan_usb.c \
Src/pcan_usb_dbg.c \
Src/pcan_can.c \
Src/pcan_led.c \
Src/pcan_protocol.c \
Src/pcan_ucan.c \
Src/pcan_timestamp.c \
Src/plin_protocol.c \
Src/plin_lin.c \
Src/plin_store.c \
Src/system_stm32f0xx.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_ll_usb.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pcd_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c \
Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_can.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \

# ASM sources
ASM_SOURCES = $(STARTUP)


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m0

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-D$(MCU_SERIES) \
-DNDEBUG \
-DUSBD_MAX_NUM_INTERFACES=2U \
$(BOARD_DEFS)


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ISrc \
-IDrivers/STM32F0xx_HAL_Driver/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IDrivers/CMSIS/Device/ST/STM32F0xx/Include \
-IDrivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fno-common -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wpedantic -Wextra -fno-common -fdata-sections -ffunction-sections -std=c99 \
$(BOARD_FLAGS) \
-D$(TARGET_VARIANT)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

.PHONY : all

# default action: build target board
all: usbcan_f072

usbcan_f072:
	$(MAKE) BOARD=usbcan_f072 DEBUG=0 OPT=-Os TARGET_VARIANT=USBCAN_F072 \
	  MCU_SERIES=STM32F072xB STARTUP=startup_stm32f072xb.s LDSCRIPT=STM32F072CBTx_FLASH.ld \
	  BOARD_FLAGS='-DHSE_VALUE=8000000 -DPCAN_IDENTITY_PRO_FD' elf hex bin

# 经典 PCAN-USB 身份（0C72:000C，SJA1000 协议，无 LIN）
usbcan_f072_sja1000:
	$(MAKE) BOARD=usbcan_f072_sja1000 DEBUG=0 OPT=-Os TARGET_VARIANT=USBCAN_F072 \
	  MCU_SERIES=STM32F072xB STARTUP=startup_stm32f072xb.s LDSCRIPT=STM32F072CBTx_FLASH.ld \
	  BOARD_FLAGS='-DHSE_VALUE=8000000 -DPCAN_IDENTITY_SJA1000' elf hex bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

ELF_TARGET = $(BUILD_DIR)/$(TARGET).elf
BIN_TARGET = $(BUILD_DIR)/$(TARGET).bin
HEX_TARGET = $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

bin: $(BIN_TARGET)

elf: $(ELF_TARGET)

hex: $(HEX_TARGET)

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)*

clean_obj:
	-rm -f $(BUILD_DIR)*/*.o $(BUILD_DIR)*/*.d $(BUILD_DIR)*/*.lst

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
