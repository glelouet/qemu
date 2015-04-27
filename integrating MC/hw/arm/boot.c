index dc62918..3d1f4a2 100644
--- a/hw/arm/boot.c
+++ b/hw/arm/boot.c
@@ -448,6 +448,7 @@ void arm_load_kernel(ARMCPU *cpu, struct arm_boot_info *info)
int initrd_size;
int is_linux = 0;
uint64_t elf_entry;
+    int elf_machine;
hwaddr entry, kernel_load_offset;
int big_endian;
static const ARMInsnFixup *primary_loader;
@@ -463,9 +464,11 @@ void arm_load_kernel(ARMCPU *cpu, struct arm_boot_info *info)
if (arm_feature(&cpu->env, ARM_FEATURE_AARCH64)) {
primary_loader = bootloader_aarch64;
kernel_load_offset = KERNEL64_LOAD_ADDR;
+        elf_machine = EM_AARCH64;
} else {
primary_loader = bootloader;
kernel_load_offset = KERNEL_LOAD_ADDR;
+        elf_machine = EM_ARM;
}

info->dtb_filename = qemu_opt_get(qemu_get_machine_opts(), "dtb");
@@ -501,7 +504,7 @@ void arm_load_kernel(ARMCPU *cpu, struct arm_boot_info *info)

/* Assume that raw images are linux kernels, and ELF images are not.  */
kernel_size = load_elf(info->kernel_filename, NULL, NULL, &elf_entry,
-                           NULL, NULL, big_endian, ELF_MACHINE, 1);
+                           NULL, NULL, big_endian, elf_machine, 1);
entry = elf_entry;
if (kernel_size < 0) {
kernel_size = load_uimage(info->kernel_filename, &entry, NULL,
