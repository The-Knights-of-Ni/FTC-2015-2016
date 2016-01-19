# Copyright (C) 2009 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_ARM_NEON := true
    LOCAL_CFLAGS += -D __ARM_NEON
endif

LOCAL_MODULE    := test
LOCAL_SRC_FILES := test.cpp

include $(BUILD_SHARED_LIBRARY)
include $(CLEAR_VARS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_ARM_NEON := true
    LOCAL_CFLAGS += -D __ARM_NEON
endif

LOCAL_MODULE    := camera_test
LOCAL_SRC_FILES := camera_test.cpp

include $(BUILD_SHARED_LIBRARY)
include $(CLEAR_VARS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_ARM_NEON := true
    LOCAL_CFLAGS += -D __ARM_NEON
endif

LOCAL_MODULE    := Mk3Teleop
LOCAL_SRC_FILES := Mk3Teleop.cpp

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_ARM_NEON := true
    LOCAL_CFLAGS += -D __ARM_NEON
endif

LOCAL_MODULE    := arm_test
LOCAL_SRC_FILES := arm_test.cpp

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
    LOCAL_ARM_NEON := true
    LOCAL_CFLAGS += -D __ARM_NEON
endif

LOCAL_MODULE    := Mk3Auto
LOCAL_SRC_FILES := Mk3Auto.cpp

include $(BUILD_SHARED_LIBRARY)