# Project Name
TARGET = RNBO

# Sources
C_SOURCES = 3rdparty/tlsf/tlsf.c
CPP_SOURCES = RNBO.cpp 

# Library Locations
LIBDAISY_DIR = 3rdparty/libDaisy

APP_TYPE = BOOT_QSPI

# Includes FatFS source files within project.
USE_FATFS = 1

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

INC_DIR = export/common/
CFLAGS += -I$(INC_DIR)

TLSF_INC_DIR = 3rdparty/tlsf/
CFLAGS += -I$(TLSF_INC_DIR)
