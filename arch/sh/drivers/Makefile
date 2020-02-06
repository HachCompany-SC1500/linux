#
# Makefile for the Linux SuperH-specific device drivers.
#

obj-y		+= dma/

obj-$(CONFIG_PCI)		+= pci/
obj-$(CONFIG_SUPERHYWAY)	+= superhyway/
obj-$(CONFIG_PUSH_SWITCH)	+= push-switch.o
obj-$(CONFIG_HEARTBEAT)		+= heartbeat.o
obj-$(CONFIG_SH_PWM_TPU)        += pwm_tpu.o
obj-$(CONFIG_GPR_PRIORITIES)	+= gpr_priority.o
obj-$(CONFIG_GPIO_PRINT_ENUM)   += gpio_print_enum.o
obj-$(CONFIG_I2C_GPIO_SH)       += i2c-gpio-sh.o
