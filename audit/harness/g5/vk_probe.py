#!/usr/bin/env python3
"""G5 prep: does a Vulkan instance come up in this container? Calls vkCreateInstance through the container's own
libvulkan.so.1 (ctypes, no GPU work) and lists the physical devices. Run with VK_LOADER_DEBUG=error,warn,driver to see
why an ICD is rejected. Prints VK_PROBE result=<VkResult> devices=<n>.
usage (inside the 6.2 container): /isaac-sim/kit/python/bin/python3 vk_probe.py"""
import ctypes
import os

lib = None
for name in ("libvulkan.so.1", "libvulkan.so"):
    try:
        lib = ctypes.CDLL(name)
        break
    except OSError as e:
        print("dlopen", name, e)
if lib is None:
    raise SystemExit("VK_PROBE result=no_libvulkan")


class VkApplicationInfo(ctypes.Structure):
    _fields_ = [("sType", ctypes.c_int), ("pNext", ctypes.c_void_p), ("pApplicationName", ctypes.c_char_p),
                ("applicationVersion", ctypes.c_uint32), ("pEngineName", ctypes.c_char_p),
                ("engineVersion", ctypes.c_uint32), ("apiVersion", ctypes.c_uint32)]


class VkInstanceCreateInfo(ctypes.Structure):
    _fields_ = [("sType", ctypes.c_int), ("pNext", ctypes.c_void_p), ("flags", ctypes.c_uint32),
                ("pApplicationInfo", ctypes.POINTER(VkApplicationInfo)), ("enabledLayerCount", ctypes.c_uint32),
                ("ppEnabledLayerNames", ctypes.c_void_p), ("enabledExtensionCount", ctypes.c_uint32),
                ("ppEnabledExtensionNames", ctypes.c_void_p)]


app = VkApplicationInfo(0, None, b"vk_probe", 1, b"none", 1, (1 << 22) | (1 << 12))  # VK_API_VERSION_1_1
ci = VkInstanceCreateInfo(1, None, 0, ctypes.pointer(app), 0, None, 0, None)
inst = ctypes.c_void_p()
res = lib.vkCreateInstance(ctypes.byref(ci), None, ctypes.byref(inst))
n = ctypes.c_uint32(0)
if res == 0:
    lib.vkEnumeratePhysicalDevices(inst, ctypes.byref(n), None)
print(f"VK_PROBE result={res} devices={n.value} VK_DRIVER_FILES={os.environ.get('VK_DRIVER_FILES')} "
      f"VK_ICD_FILENAMES={os.environ.get('VK_ICD_FILENAMES')} CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES')}")
