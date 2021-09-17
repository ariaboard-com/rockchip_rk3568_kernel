obj-$(CONFIG_FAST_CLASSIFIER) += fast-classifier.o

ifeq ($(CONFIG_SFE_SUPPORT_IPV6),)
ccflags-$(CONFIG_SFE_SUPPORT_IPV6) += -DSFE_SUPPORT_IPV6
endif

ccflags-y += -I$(obj)/../shortcut-fe

obj ?= .
