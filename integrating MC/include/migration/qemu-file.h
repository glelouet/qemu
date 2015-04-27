index a191fb6..0e5fa01 100644
--- a/include/migration/qemu-file.h
+++ b/include/migration/qemu-file.h
@@ -71,17 +71,63 @@ typedef int (QEMURamHookFunc)(QEMUFile *f, void *opaque, uint64_t flags);
#define RAM_CONTROL_ROUND    1
#define RAM_CONTROL_HOOK     2
#define RAM_CONTROL_FINISH   3
+#define RAM_CONTROL_FLUSH    4

/*
* This function allows override of where the RAM page
* is saved (such as RDMA, for example.)
*/
-typedef size_t (QEMURamSaveFunc)(QEMUFile *f, void *opaque,
+typedef int (QEMURamSaveFunc)(QEMUFile *f, void *opaque,
ram_addr_t block_offset,
+                               uint8_t *host_addr,
ram_addr_t offset,
-                               size_t size,
+                               long size,
int *bytes_sent);

+/*
+ * This function allows override of where the RAM page
+ * is saved (such as RDMA, for example.)
+ */
+typedef int (QEMURamLoadFunc)(QEMUFile *f,
+                               void *opaque,
+                               void *host_addr,
+                               long size);
+
+/*
+ * This function allows *local* RDMA copying memory between two registered
+ * RAMBlocks, both real ones as well as private memory areas independently
+ * registered by external callers (such as MC). If RDMA is not available,
+ * then this function does nothing and the caller should just use memcpy().
+ */
+typedef int (QEMURamCopyFunc)(QEMUFile *f, void *opaque,
+                               ram_addr_t block_offset_dest,
+                               ram_addr_t offset_dest,
+                               ram_addr_t block_offset_source,
+                               ram_addr_t offset_source,
+                               long size);
+
+/*
+ * Inform the underlying transport of a new virtual memory area.
+ * If this area is an actual RAMBlock, then pass the corresponding
+ * parameters of that block.
+ * If this area is an arbitrary virtual memory address, then
+ * pass the same value for both @host_addr and @block_offset.
+ */
+typedef int (QEMURamAddFunc)(QEMUFile *f, void *opaque,
+                               void *host_addr,
+                               ram_addr_t block_offset,
+                               uint64_t length);
+
+/*
+ * Remove an underlying new virtual memory area.
+ * If this area is an actual RAMBlock, then pass the corresponding
+ * parameters of that block.
+ * If this area is an arbitrary virtual memory address, then
+ * pass the same value for both @host_addr and @block_offset.
+ */
+typedef int (QEMURamRemoveFunc)(QEMUFile *f, void *opaque,
+                               ram_addr_t block_offset);
+
typedef struct QEMUFileOps {
QEMUFilePutBufferFunc *put_buffer;
QEMUFileGetBufferFunc *get_buffer;
@@ -92,12 +138,50 @@ typedef struct QEMUFileOps {
QEMURamHookFunc *after_ram_iterate;
QEMURamHookFunc *hook_ram_load;
QEMURamSaveFunc *save_page;
+    QEMURamLoadFunc *load_page;
+    QEMURamCopyFunc *copy_page;
+    QEMURamAddFunc *add;
+    QEMURamRemoveFunc *remove;
} QEMUFileOps;

+
+#define IO_BUF_SIZE 32768
+#define MAX_IOV_SIZE MIN(IOV_MAX, 64)
+
+struct QEMUFile {
+    const QEMUFileOps *ops;
+    void *opaque;
+
+    int64_t bytes_xfer;
+    int64_t xfer_limit;
+
+    int64_t pos; /* start of buffer when writing, end of buffer
+                    when reading */
+    int buf_index;
+    int buf_size; /* 0 when writing */
+    uint8_t buf[IO_BUF_SIZE];
+
+    struct iovec iov[MAX_IOV_SIZE];
+    unsigned int iovcnt;
+
+    int last_error;
+};
+
+typedef struct QEMUFileStdio {
+    FILE *stdio_file;
+    QEMUFile *file;
+} QEMUFileStdio;
+
+typedef struct QEMUFileSocket {
+    int fd;
+    QEMUFile *file;
+} QEMUFileSocket;
+
QEMUFile *qemu_fopen_ops(void *opaque, const QEMUFileOps *ops);
QEMUFile *qemu_fopen(const char *filename, const char *mode);
QEMUFile *qemu_fdopen(int fd, const char *mode);
QEMUFile *qemu_fopen_socket(int fd, const char *mode);
+QEMUFile *qemu_fopen_mc(void *opaque, const char *mode);
QEMUFile *qemu_popen_cmd(const char *command, const char *mode);
int qemu_get_fd(QEMUFile *f);
int qemu_fclose(QEMUFile *f);
@@ -267,4 +351,5 @@ static inline void qemu_get_sbe64s(QEMUFile *f, int64_t *pv)
{
qemu_get_be64s(f, (uint64_t *)pv);
}
+
#endif
