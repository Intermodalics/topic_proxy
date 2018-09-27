LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE    := topic_proxy
LOCAL_SRC_FILES := $(LOCAL_PATH)/src/service_client.cpp $(LOCAL_PATH)/src/topic_proxy.cpp $(LOCAL_PATH)/../blob/src/blob/compression.cpp $(LOCAL_PATH)/../blob/src/blob/shape_shifter.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include $(LOCAL_PATH)/../blob/include
LOCAL_CFLAGS  += -O2 --std=c++11 -pthread -fPIC -fexceptions -frtti -Wunused-parameter -Wunused-variable
LOCAL_CPPFLAGS += -DANDROID
LOCAL_LDLIBS += -landroid -lm -llog -lz
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
LOCAL_STATIC_LIBRARIES += bzip2
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include $(LOCAL_PATH)/../blob/include
LOCAL_DISABLE_FATAL_LINKER_WARNINGS=true
include $(BUILD_STATIC_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT)/../third_party)
$(call import-module,roscpp_android_ndk)
$(call import-module,cosp-android-bzip2)
