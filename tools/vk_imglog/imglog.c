/* A minimal Vulkan layer: logs every vkCreateImage (extent, layers, mips, format, samples) and every allocation of 8 MiB
   or more to stderr, to see which images a renderer makes frame by frame. Build: gcc -shared -fPIC -o libimglog.so imglog.c;
   use: VK_LAYER_PATH=<folder with imglog.json and libimglog.so> VK_INSTANCE_LAYERS=VK_LAYER_HK_imglog (waymark [U8i]). */
#include <vulkan/vulkan.h>
#include <vulkan/vk_layer.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static PFN_vkGetInstanceProcAddr next_gipa; static PFN_vkGetDeviceProcAddr next_gdpa;
static PFN_vkCreateImage next_create_image; static PFN_vkAllocateMemory next_alloc;
#define EXPORT __attribute__((visibility("default")))
static VKAPI_ATTR VkResult VKAPI_CALL my_CreateImage(VkDevice d, const VkImageCreateInfo* ci, const VkAllocationCallbacks* a, VkImage* img) {
    VkResult r = next_create_image(d, ci, a, img);
    fprintf(stderr, "VKIMG %ux%ux%u layers=%u mips=%u fmt=%d samples=%d usage=0x%x r=%d\n", ci->extent.width, ci->extent.height,
            ci->extent.depth, ci->arrayLayers, ci->mipLevels, ci->format, ci->samples, ci->usage, r);
    return r;
}
static VKAPI_ATTR VkResult VKAPI_CALL my_AllocateMemory(VkDevice d, const VkMemoryAllocateInfo* ai, const VkAllocationCallbacks* a, VkDeviceMemory* m) {
    VkResult r = next_alloc(d, ai, a, m);
    if (ai->allocationSize >= (8u << 20)) fprintf(stderr, "VKMEM alloc %llu MiB r=%d\n", (unsigned long long)(ai->allocationSize >> 20), r);
    return r;
}
static VKAPI_ATTR VkResult VKAPI_CALL my_CreateInstance(const VkInstanceCreateInfo* ci, const VkAllocationCallbacks* a, VkInstance* inst) {
    VkLayerInstanceCreateInfo* li = (VkLayerInstanceCreateInfo*)ci->pNext;
    while (li && !(li->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO && li->function == VK_LAYER_LINK_INFO)) li = (VkLayerInstanceCreateInfo*)li->pNext;
    if (!li) return VK_ERROR_INITIALIZATION_FAILED;
    next_gipa = li->u.pLayerInfo->pfnNextGetInstanceProcAddr; li->u.pLayerInfo = li->u.pLayerInfo->pNext;
    PFN_vkCreateInstance f = (PFN_vkCreateInstance)next_gipa(NULL, "vkCreateInstance");
    return f(ci, a, inst);
}
static VKAPI_ATTR VkResult VKAPI_CALL my_CreateDevice(VkPhysicalDevice p, const VkDeviceCreateInfo* ci, const VkAllocationCallbacks* a, VkDevice* dev) {
    VkLayerDeviceCreateInfo* li = (VkLayerDeviceCreateInfo*)ci->pNext;
    while (li && !(li->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO && li->function == VK_LAYER_LINK_INFO)) li = (VkLayerDeviceCreateInfo*)li->pNext;
    if (!li) return VK_ERROR_INITIALIZATION_FAILED;
    PFN_vkGetInstanceProcAddr gipa = li->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    next_gdpa = li->u.pLayerInfo->pfnNextGetDeviceProcAddr; li->u.pLayerInfo = li->u.pLayerInfo->pNext;
    PFN_vkCreateDevice f = (PFN_vkCreateDevice)gipa(NULL, "vkCreateDevice");
    VkResult r = f(p, ci, a, dev);
    if (r == VK_SUCCESS) {
        next_create_image = (PFN_vkCreateImage)next_gdpa(*dev, "vkCreateImage");
        next_alloc = (PFN_vkAllocateMemory)next_gdpa(*dev, "vkAllocateMemory");
    }
    return r;
}
EXPORT VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL imglog_GetDeviceProcAddr(VkDevice d, const char* n) {
    if (!strcmp(n, "vkCreateImage")) return (PFN_vkVoidFunction)my_CreateImage;
    if (!strcmp(n, "vkAllocateMemory")) return (PFN_vkVoidFunction)my_AllocateMemory;
    if (!strcmp(n, "vkGetDeviceProcAddr")) return (PFN_vkVoidFunction)imglog_GetDeviceProcAddr;
    return next_gdpa(d, n);
}
EXPORT VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL imglog_GetInstanceProcAddr(VkInstance i, const char* n) {
    if (!strcmp(n, "vkCreateInstance")) return (PFN_vkVoidFunction)my_CreateInstance;
    if (!strcmp(n, "vkCreateDevice")) return (PFN_vkVoidFunction)my_CreateDevice;
    if (!strcmp(n, "vkGetInstanceProcAddr")) return (PFN_vkVoidFunction)imglog_GetInstanceProcAddr;
    if (!strcmp(n, "vkGetDeviceProcAddr")) return (PFN_vkVoidFunction)imglog_GetDeviceProcAddr;
    if (!strcmp(n, "vkCreateImage")) return (PFN_vkVoidFunction)my_CreateImage;
    if (!strcmp(n, "vkAllocateMemory")) return (PFN_vkVoidFunction)my_AllocateMemory;
    return next_gipa(i, n);
}
