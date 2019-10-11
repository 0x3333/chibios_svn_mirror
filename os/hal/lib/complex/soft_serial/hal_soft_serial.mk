# List of all the Software Serial subsystem files.
SSDSRC := $(CHIBIOS)/os/hal/lib/complex/soft_serial/hal_soft_serial.c

# Required include directories
SSDINC := $(CHIBIOS)/os/hal/lib/complex/soft_serial

# Shared variables
ALLCSRC += $(SSDSRC)
ALLINC  += $(SSDINC)
