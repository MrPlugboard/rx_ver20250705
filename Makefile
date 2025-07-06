TARGET = rx_ver20250705

RTOS = FreeRTOS

NUCLEI_SDK_ROOT = ../../../../../../../..

SRCDIRS = .

INCDIRS = .

LDFLAGS += -lm

include $(NUCLEI_SDK_ROOT)/Build/Makefile.base
