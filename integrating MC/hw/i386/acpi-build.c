index f1054dd..7597517 100644
--- a/hw/i386/acpi-build.c
+++ b/hw/i386/acpi-build.c
@@ -1055,9 +1055,21 @@ build_ssdt(GArray *table_data, GArray *linker,

{
GArray *package = build_alloc_array();
-            uint8_t op = 0x13; /* VarPackageOp */
+            uint8_t op;

+            /*
+             * Note: The ability to create variable-sized packages was first introduced in ACPI 2.0. ACPI 1.0 only
+             * allowed fixed-size packages with up to 255 elements.
+             * Windows guests up to win2k8 fail when VarPackageOp is used.
+             */
+            if (acpi_cpus <= 255) {
+                op = 0x12; /* PackageOp */
+                build_append_byte(package, acpi_cpus); /* NumElements */
+            } else {
+                op = 0x13; /* VarPackageOp */
build_append_int(package, acpi_cpus); /* VarNumElements */
+            }
+
for (i = 0; i < acpi_cpus; i++) {
uint8_t b = test_bit(i, cpu->found_cpus) ? 0x01 : 0x00;
build_append_byte(package, b);
