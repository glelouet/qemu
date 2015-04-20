index 3e1e6c7..dbe4d78 100644
--- a/include/migration/migration.h
+++ b/include/migration/migration.h
@@ -46,21 +46,27 @@ struct MigrationState
int64_t bandwidth_limit;
size_t bytes_xfer;
size_t xfer_limit;
-    QemuThread thread;
+    QemuThread *thread;
QEMUBH *cleanup_bh;
QEMUFile *file;

int state;
MigrationParams params;
double mbps;
+    double copy_mbps;
int64_t total_time;
int64_t downtime;
int64_t expected_downtime;
+    int64_t xmit_time;
+    int64_t ram_copy_time;
+    int64_t log_dirty_time;
+    int64_t bitmap_time;
int64_t dirty_pages_rate;
int64_t dirty_bytes_rate;
bool enabled_capabilities[MIGRATION_CAPABILITY_MAX];
int64_t xbzrle_cache_size;
int64_t setup_time;
+    int64_t checkpoints;
};

void process_incoming_migration(QEMUFile *f);
@@ -101,7 +107,9 @@ int migrate_fd_close(MigrationState *s);

void add_migration_state_change_notifier(Notifier *notify);
void remove_migration_state_change_notifier(Notifier *notify);
+bool migration_is_active(MigrationState *);
bool migration_in_setup(MigrationState *);
+bool migration_is_mc(MigrationState *s);
bool migration_has_finished(MigrationState *);
bool migration_has_failed(MigrationState *);
MigrationState *migrate_get_current(void);
@@ -121,10 +129,29 @@ uint64_t skipped_mig_bytes_transferred(void);
uint64_t skipped_mig_pages_transferred(void);
uint64_t norm_mig_bytes_transferred(void);
uint64_t norm_mig_pages_transferred(void);
+uint64_t norm_mig_log_dirty_time(void);
+uint64_t norm_mig_bitmap_time(void);
uint64_t xbzrle_mig_bytes_transferred(void);
uint64_t xbzrle_mig_pages_transferred(void);
uint64_t xbzrle_mig_pages_overflow(void);
uint64_t xbzrle_mig_pages_cache_miss(void);
+void acct_clear(void);
+
+void migrate_set_state(MigrationState *s, int old_state, int new_state);
+
+enum {
+    MIG_STATE_ERROR = -1,
+    MIG_STATE_NONE,
+    MIG_STATE_SETUP,
+    MIG_STATE_CANCELLED,
+    MIG_STATE_CANCELLING,
+    MIG_STATE_ACTIVE,
+    MIG_STATE_CHECKPOINTING,
+    MIG_STATE_COMPLETED,
+};
+
+void mc_init_checkpointer(MigrationState *s);
+void mc_process_incoming_checkpoints_if_requested(QEMUFile *f);

void ram_handle_compressed(void *host, uint8_t ch, uint64_t size);

@@ -159,6 +186,12 @@ int64_t xbzrle_cache_resize(int64_t new_size);
void ram_control_before_iterate(QEMUFile *f, uint64_t flags);
void ram_control_after_iterate(QEMUFile *f, uint64_t flags);
void ram_control_load_hook(QEMUFile *f, uint64_t flags);
+void ram_control_add(QEMUFile *f, void *host_addr,
+                         ram_addr_t block_offset, uint64_t length);
+void ram_control_remove(QEMUFile *f, ram_addr_t block_offset);
+
+#define MBPS(bytes, time) time ? ((((double) bytes * 8)         +        / ((double) time / 1000.0)) / 1000.0 / 1000.0) : 0.0

/* Whenever this is found in the data stream, the flags
* will be passed to ram_control_load_hook in the incoming-migration
@@ -169,9 +202,38 @@ void ram_control_load_hook(QEMUFile *f, uint64_t flags);

#define RAM_SAVE_CONTROL_NOT_SUPP -1000
#define RAM_SAVE_CONTROL_DELAYED  -2000
+#define RAM_LOAD_CONTROL_NOT_SUPP -3000
+#define RAM_LOAD_CONTROL_DELAYED  -4000
+#define RAM_COPY_CONTROL_NOT_SUPP -5000
+#define RAM_COPY_CONTROL_DELAYED  -6000

-size_t ram_control_save_page(QEMUFile *f, ram_addr_t block_offset,
-                             ram_addr_t offset, size_t size,
+#define RDMA_CONTROL_VERSION_CURRENT 1
+
+int ram_control_save_page(QEMUFile *f, ram_addr_t block_offset,
+                             uint8_t *host_addr,
+                             ram_addr_t offset, long size,
int *bytes_sent);

+int ram_control_load_page(QEMUFile *f,
+                             void *host_addr,
+                             long size);
+
+int ram_control_copy_page(QEMUFile *f,
+                             ram_addr_t block_offset_dest,
+                             ram_addr_t offset_dest,
+                             ram_addr_t block_offset_source,
+                             ram_addr_t offset_source,
+                             long size);
+
+int migrate_use_mc(void);
+int migrate_use_mc_rdma_copy(void);
+void mc_configure_net(MigrationState *s);
+
+#define MC_VERSION 1
+
+int mc_info_load(QEMUFile *f, void *opaque, int version_id);
+void mc_info_save(QEMUFile *f, void *opaque);
+
+void qemu_rdma_info_save(QEMUFile *f, void *opaque);
+int qemu_rdma_info_load(QEMUFile *f, void *opaque, int version_id);
#endif
