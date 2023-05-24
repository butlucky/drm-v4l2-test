LOCAL_PATH      := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE           := v4l2_capture 
LOCAL_SRC_FILES        := v4l2_capture.cpp
LOCAL_CFLAGS           += -Wno-extern-c-compat
LOCAL_CPPFLAGS         += -std=c++17
include $(BUILD_EXECUTABLE)

