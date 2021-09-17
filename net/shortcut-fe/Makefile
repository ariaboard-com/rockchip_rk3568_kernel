#
# Makefile for Shortcut FE.
#

obj-$(CONFIG_SHORTCUT_FE) += shortcut-fe.o

ifdef CONFIG_SFE_SUPPORT_IPV6
obj-$(CONFIG_SHORTCUT_FE) += shortcut-fe-ipv6.o
endif

obj-$(CONFIG_SHORTCUT_FE_CM) += shortcut-fe-cm.o

shortcut-fe-objs := \
	sfe_ipv4.o

ifdef CONFIG_SFE_SUPPORT_IPV6
shortcut-fe-ipv6-objs := \
	sfe_ipv6.o
endif

shortcut-fe-cm-objs := \
	sfe_cm.o

