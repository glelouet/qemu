index ae49cd8..ac9a025 100644
--- a/block/vdi.c
+++ b/block/vdi.c
@@ -31,7 +31,7 @@
* Allocation of blocks could be optimized (less writes to block map and
* header).
*
- * Read and write of adjacents blocks could be done in one operation
+ * Read and write of adjacent blocks could be done in one operation
* (current code uses one operation per block (1 MiB).
*
* The code is not thread safe (missing locks for changes in header and
