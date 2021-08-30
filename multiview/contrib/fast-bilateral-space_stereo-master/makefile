 

# ------- Depends on...
# eigen3, opencv (with non-free)
# edit 'config.h' to switch on/off triclops

VERSION ?= release
CC:=g++

SRCS:=src/bilateral_grid_simplified.cpp \
	src/fast_bilateral_solver.cpp \
	src/stereo_matcher_birchfield_tomasi.cpp 

# -msse, -msse2, -msse3, -mmmx, -m3dnow
# -mach=native -pipe

# -Wno-psabi 
# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -march=native -Wno-strict-overflow
CPP_DEFINES:=
CPP_INCLUDES:=-isystem /usr/local/include -isystem /usr/include \
	-I$(CURDIR)/include \
	$(shell pkg-config --cflags eigen3) \
	$(shell pkg-config --cflags opencv)

C_INCLUDES:=

# ----------------------------- build 
BUILD_SO:=1
TARGET:=fast-bilateral-space-stereo
C_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(C_INCLUDES)
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:= -lm -lutil -ldl -lpthread -lstdc++  -fopenmp \
	-lceres -lglog -lcholmod -lcxsparse -llapack \
	$(shell pkg-config --libs blas-atlas) \
	$(shell pkg-config --libs opencv)


# ----------------------------- options
PCH:=
MOCD:=
ADDRESS_SANITIZE ?= 0

# ----------------------------- dependent libraries
DEPENDS_ON:=

# ----------------------------- 

-include master-makefile

