index c2c688c..9cf5b84 100644
--- a/hw/misc/vfio.c
+++ b/hw/misc/vfio.c
@@ -1043,7 +1043,7 @@ static void vfio_bar_write(void *opaque, hwaddr addr,
buf.dword = cpu_to_le32(data);
break;
default:
-        hw_error("vfio: unsupported write size, %d bytesn", size);
+        hw_error("vfio: unsupported write size, %d bytes", size);
break;
}

@@ -1103,7 +1103,7 @@ static uint64_t vfio_bar_read(void *opaque,
data = le32_to_cpu(buf.dword);
break;
default:
-        hw_error("vfio: unsupported read size, %d bytesn", size);
+        hw_error("vfio: unsupported read size, %d bytes", size);
break;
}

@@ -1157,7 +1157,7 @@ static void vfio_pci_load_rom(VFIODevice *vdev)
if (!vdev->rom_size) {
vdev->rom_read_failed = true;
error_report("vfio-pci: Cannot read device rom at "
-                    "%04x:%02x:%02x.%xn",
+                    "%04x:%02x:%02x.%x",
vdev->host.domain, vdev->host.bus, vdev->host.slot,
vdev->host.function);
error_printf("Device option ROM contents are probably invalid "
@@ -1192,12 +1192,9 @@ static uint64_t vfio_rom_read(void *opaque, hwaddr addr, unsigned size)
uint64_t val = ((uint64_t)1 << (size * 8)) - 1;

/* Load the ROM lazily when the guest tries to read it */
-    if (unlikely(!vdev->rom)) {
-        vfio_pci_load_rom(vdev);
if (unlikely(!vdev->rom && !vdev->rom_read_failed)) {
vfio_pci_load_rom(vdev);
}
-    }

memcpy(&val, vdev->rom + addr,
(addr < vdev->rom_size) ? MIN(size, vdev->rom_size - addr) : 0);
@@ -1341,7 +1338,7 @@ static void vfio_vga_write(void *opaque, hwaddr addr,
buf.dword = cpu_to_le32(data);
break;
default:
-        hw_error("vfio: unsupported write size, %d bytesn", size);
+        hw_error("vfio: unsupported write size, %d bytes", size);
break;
}

@@ -1384,7 +1381,7 @@ static uint64_t vfio_vga_read(void *opaque, hwaddr addr, unsigned size)
data = le32_to_cpu(buf.dword);
break;
default:
-        hw_error("vfio: unsupported read size, %d bytesn", size);
+        hw_error("vfio: unsupported read size, %d bytes", size);
break;
}

@@ -1429,7 +1426,7 @@ static uint64_t vfio_generic_window_quirk_read(void *opaque,

if (!vfio_range_contained(addr, size, quirk->data.data_offset,
quirk->data.data_size)) {
-            hw_error("%s: window data read not fully contained: %sn",
+            hw_error("%s: window data read not fully contained: %s",
__func__, memory_region_name(&quirk->mem));
}

@@ -1458,7 +1455,7 @@ static void vfio_generic_window_quirk_write(void *opaque, hwaddr addr,
quirk->data.address_offset, quirk->data.address_size)) {

if (addr != quirk->data.address_offset) {
-            hw_error("%s: offset write into address window: %sn",
+            hw_error("%s: offset write into address window: %s",
__func__, memory_region_name(&quirk->mem));
}

@@ -1479,7 +1476,7 @@ static void vfio_generic_window_quirk_write(void *opaque, hwaddr addr,

if (!vfio_range_contained(addr, size, quirk->data.data_offset,
quirk->data.data_size)) {
-            hw_error("%s: window data write not fully contained: %sn",
+            hw_error("%s: window data write not fully contained: %s",
__func__, memory_region_name(&quirk->mem));
}

@@ -1515,7 +1512,7 @@ static uint64_t vfio_generic_quirk_read(void *opaque,
ranges_overlap(addr, size, offset, quirk->data.address_mask + 1)) {
if (!vfio_range_contained(addr, size, offset,
quirk->data.address_mask + 1)) {
-            hw_error("%s: read not fully contained: %sn",
+            hw_error("%s: read not fully contained: %s",
__func__, memory_region_name(&quirk->mem));
}

@@ -1544,7 +1541,7 @@ static void vfio_generic_quirk_write(void *opaque, hwaddr addr,
ranges_overlap(addr, size, offset, quirk->data.address_mask + 1)) {
if (!vfio_range_contained(addr, size, offset,
quirk->data.address_mask + 1)) {
-            hw_error("%s: write not fully contained: %sn",
+            hw_error("%s: write not fully contained: %s",
__func__, memory_region_name(&quirk->mem));
}

@@ -2302,7 +2299,7 @@ static void vfio_listener_region_add(MemoryListener *listener,
container->iommu_data.type1.error = ret;
}
} else {
-            hw_error("vfio: DMA mapping failed, unable to continuen");
+            hw_error("vfio: DMA mapping failed, unable to continue");
}
}
}
@@ -2972,7 +2969,7 @@ static void vfio_pci_pre_reset(VFIODevice *vdev)
pmcsr = vfio_pci_read_config(pdev, vdev->pm_cap + PCI_PM_CTRL, 2);
state = pmcsr & PCI_PM_CTRL_STATE_MASK;
if (state) {
-                error_report("vfio: Unable to power on device, stuck in D%dn",
+                error_report("vfio: Unable to power on device, stuck in D%d",
state);
}
}
@@ -3271,7 +3268,7 @@ static void vfio_kvm_device_del_group(VFIOGroup *group)
}

if (ioctl(vfio_kvm_device_fd, KVM_SET_DEVICE_ATTR, &attr)) {
-        error_report("Failed to remove group %d to KVM VFIO device: %m",
+        error_report("Failed to remove group %d from KVM VFIO device: %m",
group->groupid);
}
#endif
@@ -3339,7 +3336,7 @@ static int vfio_connect_container(VFIOGroup *group)
vfio_listener_release(container);
g_free(container);
close(fd);
-            error_report("vfio: memory listener initialization failed for containern");
+            error_report("vfio: memory listener initialization failed for container");
return ret;
}

