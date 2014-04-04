PROBE_HOST ?= native
PLATFORM_DIR = platforms/$(PROBE_HOST)
VPATH += $(PLATFORM_DIR)

BUILDDATE := `date +"%Y%m%d"`

CFLAGS += -Wall -Wextra -Wno-pointer-sign -Wno-char-subscripts\
	-Wno-sign-compare \
	-O0 -std=gnu99 -g3 -DBUILDDATE=\"$(BUILDDATE)\"\
	-I. -Iinclude -I$(PLATFORM_DIR) -MD

include $(PLATFORM_DIR)/Makefile.inc

OBJ = $(SRC:.c=.o)

gdb: stm32f1_dfu
	cgdb -d arm-cortexm3-eabi-gdb $^

.PHONY:	gdb clean host_clean

clean:	host_clean
	-@$(RM) *.o *.d *~ $(HOSTFILES)
	-@$(RM) platforms/*/*.o platforms/*/*.d mapfile

-include *.d

