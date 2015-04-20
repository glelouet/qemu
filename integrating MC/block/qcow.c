index 1e128be..d5a7d5f 100644
--- a/block/qcow.c
+++ b/block/qcow.c
@@ -723,7 +723,7 @@ static int qcow_create(const char *filename, QEMUOptionParameter *options,
backing_file = NULL;
}
header.cluster_bits = 9; /* 512 byte cluster to avoid copying
-                                    unmodifyed sectors */
+                                    unmodified sectors */
header.l2_bits = 12; /* 32 KB L2 tables */
} else {
header.cluster_bits = 12; /* 4 KB clusters */
