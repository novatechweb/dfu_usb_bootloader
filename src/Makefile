PROBE_HOST ?= native
PLATFORM_DIR = platforms/$(PROBE_HOST)
VPATH += $(PLATFORM_DIR)

BUILDDATE := `date +"%Y%m%d"`

CFLAGS += -Wall -Wextra -Wno-pointer-sign -Wno-char-subscripts\
	-Wno-sign-compare \
	-O0 -std=gnu99 -g3 -DBUILDDATE=\"$(BUILDDATE)\"\
	-I. -Iinclude -I$(PLATFORM_DIR) \
	-DVERSION_SUFFIX=\"`../scripts/setlocalversion`\" -MD

include $(PLATFORM_DIR)/Makefile.inc

OBJ = $(SRC:.c=.o)

blackmagic: $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

gdb: blackmagic
	cgdb -d arm-cortexm3-eabi-gdb $^

gdb-dfu: blackmagic_dfu blackmagic
	cgdb -d arm-cortexm3-eabi-gdb $^

.PHONY:	gdb gdb-boot clean host_clean

clean:	host_clean
	-@$(RM) *.o *.d *~ $(HOSTFILES)
	-@$(RM) platforms/*/*.o platforms/*/*.d mapfile

-include *.d

