#******************************************************************************
# Makefile - Build rules for buen_robotv42 (EK-TM4C1294XL)
#******************************************************************************

# MCU (EK-TM4C1294XL)
PART=TM4C1294NCPDT

# TivaWare root (ajústalo si tu árbol es distinto)
ROOT=../../../..

# ===== Reglas comunes de TivaWare (usa PART, IPATH, VPATH, etc.)
include ${ROOT}/makedefs

# ===== Include paths explícitos (para inc/hw_nvic.h y demás)
IPATH += $(ROOT)
IPATH += $(ROOT)/inc
IPATH += $(ROOT)/driverlib
IPATH += $(ROOT)/utils

# ===== Target por defecto
all: ${COMPILER}
all: ${COMPILER}/buen_robotv42.axf

# Limpieza
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

# Carpeta de salida
${COMPILER}:
	@mkdir -p ${COMPILER}

# ===== Enlazado: .axf depende de startup, tu .c y driverlib + .ld
${COMPILER}/buen_robotv42.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/buen_robotv42.axf: ${COMPILER}/buen_robotv42.o
${COMPILER}/buen_robotv42.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/buen_robotv42.axf: buen_robotv42.ld

# Linker script y entry (el nombre debe coincidir con el stem del .axf)
SCATTERgcc_buen_robotv42=buen_robotv42.ld
ENTRY_buen_robotv42=ResetISR

# ===== Flags extra (robustez)
CFLAGSgcc  += -DTARGET_IS_TM4C129_RA2
# evita tablas de backtrace (ahorra FLASH y previene solapes)
CFLAGSgcc  += -fno-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables
# caminos de include forzados (además de IPATH)
CFLAGSgcc  += -I$(ROOT) -I$(ROOT)/inc -I$(ROOT)/driverlib -I$(ROOT)/utils
# recolecta secciones no usadas
LDFLAGSgcc += -Wl,--gc-sections

# Dependencias automáticas
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

