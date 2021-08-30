
CXX:=/opt/llvm/6.0/bin/clang++

BASE_DIR:=/opt/llvm/6.0/local_stdc++
PCODE:=$(PERCEIVE_CODE)/multiview/multiview_cpp/src

CONTRIB_DIR:=$(PCODE)/perceive/contrib

OPENPOSE_DIR:=$(BASE_DIR)/openpose
OPENPOSE_INC:=-DWITH_OPENPOSE -DOPENPOSE_DIR="\"$(OPENPOSE_DIR)\"" -isystem$(OPENPOSE_DIR)/cuda/include
OPENPOSE_LINK:=-L$(OPENPOSE_DIR)/cuda/lib -Wl,-rpath,$(OPENPOSE_DIR)/cuda/lib, -lcaffe -lopenpose -lgflags

CV_LINK:=-L$(BASE_DIR)/lib -Wl,-rpath,$(BASE_DIR)/lib, -lopencv_imgproc -lopencv_ximgproc -lopencv_core -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lopencv_features2d -lopencv_calib3d -lopencv_bgsegm -lopencv_aruco

VERSION_INC:=-DMULTIVIEW_VERSION="\"$(shell $(PERCEIVE_CODE)/bin/multiview-version.sh)\""


STATIC_MATH_DIR:=$(PERCEIVE_CODE)/multiview/contrib/static-math_20180201/include
EIGEN_DIR=/usr/include/eigen3

CPP_FLAGS:=-O2 $(VERSION_INC) -DWITH_CUDA -isystem$(EIGEN_DIR) -isystem$(STATIC_MATH_DIR) -isystem$(CONTRIB_DIR) -I$(PCODE) -isystem$(BASE_DIR)/include $(CV_INC) $(OPENPOSE_INC)
CPP_LINK:=$(CV_LINK) $(OPENPOSE_LINK)

OBJECT_FILES:=main.o op-inc.o jsoncpp.o struct-meta.o json-io.o timestamp.o cuda-spec.o config.o

.PHONY: clean test

a.out: $(OBJECT_FILES)
	$(CXX) -std=c++17 $(OBJECT_FILES) $(CPP_LINK) -o $@

main.o: main.cpp op-inc.hpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

op-inc.o: op-inc.cpp op-inc.hpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

jsoncpp.o: $(CONTRIB_DIR)/jsoncpp.cpp
	$(CXX) -std=c++17 -isystem$(CONTRIB_DIR) -c $< -o $@

struct-meta.o: $(PCODE)/perceive/io/struct-meta.cpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

json-io.o: $(PCODE)/perceive/io/json-io.cpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

timestamp.o: $(PCODE)/perceive/utils/timestamp.cpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

cuda-spec.o: $(PCODE)/perceive/utils/cuda-spec.cpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

config.o: $(PCODE)/perceive/config.cpp
	$(CXX) -std=c++17 $(CPP_FLAGS) -c $< -o $@

test: a.out
	@cd $(CURDIR)
	GLOG_minloglevel=3 GLOG_logtostderr=1 ./a.out $(PERCEIVE_DATA)/computer-vision/test-data/openpose-worker-test/test-images-1/*.png

clean:
	rm -f *.o a.out
