index e5ec798..e85e133 100644
--- a/qemu-file.c
+++ b/qemu-file.c
@@ -5,37 +5,6 @@
#include "migration/migration.h"
#include "migration/qemu-file.h"

-#define IO_BUF_SIZE 32768
-#define MAX_IOV_SIZE MIN(IOV_MAX, 64)
-
-struct QEMUFile {
-    const QEMUFileOps *ops;
-    void *opaque;
-
-    int64_t bytes_xfer;
-    int64_t xfer_limit;
-
-    int64_t pos; /* start of buffer when writing, end of buffer
-                    when reading */
-    int buf_index;
-    int buf_size; /* 0 when writing */
-    uint8_t buf[IO_BUF_SIZE];
-
-    struct iovec iov[MAX_IOV_SIZE];
-    unsigned int iovcnt;
-
-    int last_error;
-};
-
-typedef struct QEMUFileStdio {
-    FILE *stdio_file;
-    QEMUFile *file;
-} QEMUFileStdio;
-
-typedef struct QEMUFileSocket {
-    int fd;
-    QEMUFile *file;
-} QEMUFileSocket;

static ssize_t socket_writev_buffer(void *opaque, struct iovec *iov, int iovcnt,
int64_t pos)
@@ -508,14 +477,17 @@ void ram_control_load_hook(QEMUFile *f, uint64_t flags)
}
}

-size_t ram_control_save_page(QEMUFile *f, ram_addr_t block_offset,
-                         ram_addr_t offset, size_t size, int *bytes_sent)
+int ram_control_save_page(QEMUFile *f, ram_addr_t block_offset,
+                         uint8_t *host_addr,
+                         ram_addr_t offset, long size, int *bytes_sent)
{
if (f->ops->save_page) {
int ret = f->ops->save_page(f, f->opaque, block_offset,
+                                    host_addr,
offset, size, bytes_sent);

-        if (ret != RAM_SAVE_CONTROL_DELAYED) {
+        if (ret != RAM_SAVE_CONTROL_DELAYED
+                && ret != RAM_SAVE_CONTROL_NOT_SUPP) {
if (bytes_sent && *bytes_sent > 0) {
qemu_update_position(f, *bytes_sent);
} else if (ret < 0) {
@@ -529,6 +501,77 @@ size_t ram_control_save_page(QEMUFile *f, ram_addr_t block_offset,
return RAM_SAVE_CONTROL_NOT_SUPP;
}

+int ram_control_load_page(QEMUFile *f, void *host_addr, long size)
+{
+    if (f->ops->load_page) {
+        int ret = f->ops->load_page(f, f->opaque, host_addr, size);
+
+        if (ret != RAM_LOAD_CONTROL_DELAYED
+                && ret != RAM_LOAD_CONTROL_NOT_SUPP) {
+            if (ret < 0) {
+                qemu_file_set_error(f, ret);
+            }
+        }
+
+        return ret;
+    }
+
+    return RAM_LOAD_CONTROL_NOT_SUPP;
+}
+
+int ram_control_copy_page(QEMUFile *f,
+                             ram_addr_t block_offset_dest,
+                             ram_addr_t offset_dest,
+                             ram_addr_t block_offset_source,
+                             ram_addr_t offset_source,
+                             long size)
+{
+    if (f->ops->copy_page) {
+        int ret = f->ops->copy_page(f, f->opaque,
+                                    block_offset_dest,
+                                    offset_dest,
+                                    block_offset_source,
+                                    offset_source,
+                                    size);
+
+        if (ret != RAM_COPY_CONTROL_DELAYED) {
+            if (ret < 0) {
+                qemu_file_set_error(f, ret);
+            }
+        }
+
+        return ret;
+    }
+
+    return RAM_COPY_CONTROL_NOT_SUPP;
+}
+
+
+void ram_control_add(QEMUFile *f, void *host_addr,
+                         ram_addr_t block_offset, uint64_t length)
+{
+    int ret = 0;
+
+    if (f->ops->add) {
+        ret = f->ops->add(f, f->opaque, host_addr, block_offset, length);
+        if (ret < 0) {
+            qemu_file_set_error(f, ret);
+        }
+    }
+}
+
+void ram_control_remove(QEMUFile *f, ram_addr_t block_offset)
+{
+    int ret = 0;
+
+    if (f->ops->remove) {
+        ret = f->ops->remove(f, f->opaque, block_offset);
+        if (ret < 0) {
+            qemu_file_set_error(f, ret);
+        }
+    }
+}
+
static void qemu_fill_buffer(QEMUFile *f)
{
int len;
