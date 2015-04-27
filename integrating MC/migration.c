index e0e24d4..e44a7c5 100644
--- a/migration.c
+++ b/migration.c
@@ -36,16 +36,6 @@
do { } while (0)
#endif

-enum {
-    MIG_STATE_ERROR = -1,
-    MIG_STATE_NONE,
-    MIG_STATE_SETUP,
-    MIG_STATE_CANCELLING,
-    MIG_STATE_CANCELLED,
-    MIG_STATE_ACTIVE,
-    MIG_STATE_COMPLETED,
-};
-
#define MAX_THROTTLE  (32 << 20)      /* Migration speed throttling */

/* Amount of time to allocate to each "chunk" of bandwidth-throttled
@@ -69,7 +59,6 @@ MigrationState *migrate_get_current(void)
.state = MIG_STATE_NONE,
.bandwidth_limit = MAX_THROTTLE,
.xbzrle_cache_size = DEFAULT_MIGRATE_CACHE_SIZE,
-        .mbps = -1,
};

return &current_migration;
@@ -105,6 +94,9 @@ static void process_incoming_migration_co(void *opaque)
int ret;

ret = qemu_loadvm_state(f);
+    if (ret >= 0) {
+        mc_process_incoming_checkpoints_if_requested(f);
+    }
qemu_fclose(f);
free_xbzrle_decoded_buf();
if (ret < 0) {
@@ -189,6 +181,31 @@ static void get_xbzrle_cache_stats(MigrationInfo *info)
}
}

+static void get_ram_stats(MigrationState *s, MigrationInfo *info)
+{
+    info->has_total_time = true;
+    info->total_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME)
+        - s->total_time;
+
+    info->has_ram = true;
+    info->ram = g_malloc0(sizeof(*info->ram));
+    info->ram->transferred = ram_bytes_transferred();
+    info->ram->total = ram_bytes_total();
+    info->ram->duplicate = dup_mig_pages_transferred();
+    info->ram->skipped = skipped_mig_pages_transferred();
+    info->ram->normal = norm_mig_pages_transferred();
+    info->ram->normal_bytes = norm_mig_bytes_transferred();
+    info->ram->mbps = s->mbps;
+
+    if (blk_mig_active()) {
+        info->has_disk = true;
+        info->disk = g_malloc0(sizeof(*info->disk));
+        info->disk->transferred = blk_mig_bytes_transferred();
+        info->disk->remaining = blk_mig_bytes_remaining();
+        info->disk->total = blk_mig_bytes_total();
+    }
+}
+
MigrationInfo *qmp_query_migrate(Error **errp)
{
MigrationInfo *info = g_malloc0(sizeof(*info));
@@ -215,26 +232,8 @@ MigrationInfo *qmp_query_migrate(Error **errp)
info->has_setup_time = true;
info->setup_time = s->setup_time;

-        info->has_ram = true;
-        info->ram = g_malloc0(sizeof(*info->ram));
-        info->ram->transferred = ram_bytes_transferred();
-        info->ram->remaining = ram_bytes_remaining();
-        info->ram->total = ram_bytes_total();
-        info->ram->duplicate = dup_mig_pages_transferred();
-        info->ram->skipped = skipped_mig_pages_transferred();
-        info->ram->normal = norm_mig_pages_transferred();
-        info->ram->normal_bytes = norm_mig_bytes_transferred();
+        get_ram_stats(s, info);
info->ram->dirty_pages_rate = s->dirty_pages_rate;
-        info->ram->mbps = s->mbps;
-
-        if (blk_mig_active()) {
-            info->has_disk = true;
-            info->disk = g_malloc0(sizeof(*info->disk));
-            info->disk->transferred = blk_mig_bytes_transferred();
-            info->disk->remaining = blk_mig_bytes_remaining();
-            info->disk->total = blk_mig_bytes_total();
-        }
-
get_xbzrle_cache_stats(info);
break;
case MIG_STATE_COMPLETED:
@@ -243,22 +242,37 @@ MigrationInfo *qmp_query_migrate(Error **errp)
info->has_status = true;
info->status = g_strdup("completed");
info->has_total_time = true;
-        info->total_time = s->total_time;
+        info->total_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME)
+            - s->total_time;
info->has_downtime = true;
info->downtime = s->downtime;
info->has_setup_time = true;
info->setup_time = s->setup_time;

-        info->has_ram = true;
-        info->ram = g_malloc0(sizeof(*info->ram));
-        info->ram->transferred = ram_bytes_transferred();
-        info->ram->remaining = 0;
-        info->ram->total = ram_bytes_total();
-        info->ram->duplicate = dup_mig_pages_transferred();
-        info->ram->skipped = skipped_mig_pages_transferred();
-        info->ram->normal = norm_mig_pages_transferred();
-        info->ram->normal_bytes = norm_mig_bytes_transferred();
-        info->ram->mbps = s->mbps;
+        get_ram_stats(s, info);
+        break;
+    case MIG_STATE_CHECKPOINTING:
+        info->has_status = true;
+        info->status = g_strdup("checkpointing");
+        info->has_setup_time = true;
+        info->setup_time = s->setup_time;
+        info->has_downtime = true;
+        info->downtime = s->downtime;
+
+        get_ram_stats(s, info);
+        info->ram->dirty_pages_rate = s->dirty_pages_rate;
+        get_xbzrle_cache_stats(info);
+
+
+        info->has_mc = true;
+        info->mc = g_malloc0(sizeof(*info->mc));
+        info->mc->xmit_time = s->xmit_time;
+        info->mc->log_dirty_time = s->log_dirty_time;
+        info->mc->migration_bitmap_time = s->bitmap_time;
+        info->mc->ram_copy_time = s->ram_copy_time;
+        info->mc->copy_mbps = s->copy_mbps;
+        info->mc->mbps = s->mbps;
+        info->mc->checkpoints = s->checkpoints;
break;
case MIG_STATE_ERROR:
info->has_status = true;
@@ -279,7 +293,7 @@ void qmp_migrate_set_capabilities(MigrationCapabilityStatusList *params,
MigrationState *s = migrate_get_current();
MigrationCapabilityStatusList *cap;

-    if (s->state == MIG_STATE_ACTIVE || s->state == MIG_STATE_SETUP) {
+    if (migration_is_active(s)) {
error_set(errp, QERR_MIGRATION_ACTIVE);
return;
}
@@ -291,7 +305,13 @@ void qmp_migrate_set_capabilities(MigrationCapabilityStatusList *params,

/* shared migration helpers */

-static void migrate_set_state(MigrationState *s, int old_state, int new_state)
+bool migration_is_active(MigrationState *s)
+{
+    return (s->state == MIG_STATE_ACTIVE) || s->state == MIG_STATE_SETUP
+            || s->state == MIG_STATE_CHECKPOINTING;
+}
+
+void migrate_set_state(MigrationState *s, int old_state, int new_state)
{
if (atomic_cmpxchg(&s->state, old_state, new_state) == new_state) {
trace_migrate_set_state(new_state);
@@ -302,20 +322,23 @@ static void migrate_fd_cleanup(void *opaque)
{
MigrationState *s = opaque;

+    if(s->cleanup_bh) {
qemu_bh_delete(s->cleanup_bh);
s->cleanup_bh = NULL;
+    }

if (s->file) {
DPRINTF("closing filen");
qemu_mutex_unlock_iothread();
-        qemu_thread_join(&s->thread);
+        qemu_thread_join(s->thread);
qemu_mutex_lock_iothread();
+        g_free(s->thread);

qemu_fclose(s->file);
s->file = NULL;
}

-    assert(s->state != MIG_STATE_ACTIVE);
+    assert(!migration_is_active(s));

if (s->state != MIG_STATE_COMPLETED) {
qemu_savevm_state_cancel();
@@ -365,6 +388,11 @@ bool migration_in_setup(MigrationState *s)
return s->state == MIG_STATE_SETUP;
}

+bool migration_is_mc(MigrationState *s)
+{
+        return s->state == MIG_STATE_CHECKPOINTING;
+}
+
bool migration_has_finished(MigrationState *s)
{
return s->state == MIG_STATE_COMPLETED;
@@ -425,7 +453,8 @@ void qmp_migrate(const char *uri, bool has_blk, bool blk,
params.shared = has_inc && inc;

if (s->state == MIG_STATE_ACTIVE || s->state == MIG_STATE_SETUP ||
-        s->state == MIG_STATE_CANCELLING) {
+        s->state == MIG_STATE_CANCELLING
+         || s->state == MIG_STATE_CHECKPOINTING) {
error_set(errp, QERR_MIGRATION_ACTIVE);
return;
}
@@ -630,7 +659,10 @@ static void *migration_thread(void *opaque)
}

if (!qemu_file_get_error(s->file)) {
-                    migrate_set_state(s, MIG_STATE_ACTIVE, MIG_STATE_COMPLETED);
+                    if (!migrate_use_mc()) {
+                        migrate_set_state(s,
+                            MIG_STATE_ACTIVE, MIG_STATE_COMPLETED);
+                    }
break;
}
}
@@ -647,8 +679,7 @@ static void *migration_thread(void *opaque)
double bandwidth = transferred_bytes / time_spent;
max_size = bandwidth * migrate_max_downtime() / 1000000;

-            s->mbps = time_spent ? (((double) transferred_bytes * 8.0) /
-                    ((double) time_spent / 1000.0)) / 1000.0 / 1000.0 : -1;
+            s->mbps = MBPS(transferred_bytes, time_spent);

DPRINTF("transferred %" PRIu64 " time_spent %" PRIu64
" bandwidth %g max_size %" PRId64 "n",
@@ -676,11 +707,21 @@ static void *migration_thread(void *opaque)
s->downtime = end_time - start_time;
runstate_set(RUN_STATE_POSTMIGRATE);
} else {
+        if(migrate_use_mc()) {
+            mc_configure_net(s);
+        }
+
if (old_vm_running) {
vm_start();
}
}
+
+    if (migrate_use_mc() && s->state != MIG_STATE_ERROR) {
+        mc_init_checkpointer(s);
+    } else {
qemu_bh_schedule(s->cleanup_bh);
+    }
+
qemu_mutex_unlock_iothread();

return NULL;
@@ -701,6 +742,7 @@ void migrate_fd_connect(MigrationState *s)
/* Notify before starting migration thread */
notifier_list_notify(&migration_state_notifiers, s);

-    qemu_thread_create(&s->thread, "migration", migration_thread, s,
+    s->thread = g_malloc0(sizeof(*s->thread));
+    qemu_thread_create(s->thread, "migration", migration_thread, s,
QEMU_THREAD_JOINABLE);
}
