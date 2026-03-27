# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# Upgrade project C++ mode without editing the vendored VEX toolchain config.
# Some older toolchains accept gnu++2a (C++20 draft) but reject gnu++20.
CXX_FLAGS := $(subst -std=gnu++11,-std=gnu++2a,$(CXX_FLAGS))

# Suppress known linker warnings when the local linker supports these options.
LD_HAS_NO_WARN_EXECSTACK := $(shell $(LINK) --help 2>/dev/null | grep -q -- '--no-warn-execstack' && echo 1 || echo 0)
LD_HAS_NO_WARN_RWX := $(shell $(LINK) --help 2>/dev/null | grep -q -- '--no-warn-rwx-segments' && echo 1 || echo 0)

ifeq ($(LD_HAS_NO_WARN_EXECSTACK),1)
LNK_FLAGS += --no-warn-execstack
endif

ifeq ($(LD_HAS_NO_WARN_RWX),1)
LNK_FLAGS += --no-warn-rwx-segments
endif

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
# include/**/*.h changes should trigger recompilation as well
SRC_H  = $(shell find include -type f -name '*.h')

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk
