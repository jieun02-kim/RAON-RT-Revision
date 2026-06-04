VISP_BUILD    = /home/raimlab/visp/build
VISP_SRC      = /home/raimlab/visp/modules
VISP_LIB      = $(VISP_BUILD)/lib
RS2_INCLUDE   = /usr/local/include
RS2_LIB       = /usr/local/lib

VISP_INCLUDES = -I$(VISP_BUILD)/include \
                -I$(VISP_SRC)/core/include \
                -I$(VISP_SRC)/sensor/include \
                -I$(VISP_SRC)/io/include

CFLAGS  = -w -O2 -std=c++17 $(VISP_INCLUDES) -I$(RS2_INCLUDE) \
          -I/usr/include/opencv4 \
          -I/usr/include/pcl-1.14 \
          -I/usr/include/eigen3
LDFLAGS = -L$(VISP_LIB)  -lvisp_core -lvisp_sensor -lvisp_io \
          -L$(RS2_LIB)   -lrealsense2 \
          -Wl,-rpath,$(VISP_LIB)

all: save_camera_params

save_camera_params: save_camera_params.cpp
	g++ $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f save_camera_params
