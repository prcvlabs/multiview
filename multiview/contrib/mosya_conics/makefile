 

# ------- Depends on...
# eigen3, opencv (with non-free)
# edit 'config.h' to switch on/off triclops

VERSION ?= release
CC:=g++

SRCS:=$(shell find src -type f -name '*.cpp' | grep -v main.cpp)

# -msse, -msse2, -msse3, -mmmx, -m3dnow
# -mach=native -pipe

# -Wno-psabi 
# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -march=native -Wno-strict-overflow
CPP_DEFINES:=
CPP_INCLUDES:=$(shell pkg-config --cflags eigen3)

C_INCLUDES:=

# ----------------------------- build 
BUILD_SO:=1
TARGET:=mosya-conics
C_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(C_INCLUDES)
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:= -lm -lstdc++ 

# ----------------------------- options
PCH:=src/mystuff.h
MOCD:=
ADDRESS_SANITIZE ?= 0

# ----------------------------- dependent libraries
DEPENDS_ON:=

# ----------------------------- 

-include master-makefile

