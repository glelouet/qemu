index 60c975d..a1fa370 100644
--- a/arch_init.c
+++ b/arch_init.c
@@ -230,6 +230,8 @@ typedef struct AccountingInfo {
uint64_t skipped_pages;
uint64_t norm_pages;
uint64_t iterations;
+    uint64_t log_dirty_time;
+    uint64_t migration_bitmap_time;
uint64_t xbzrle_bytes;
uint64_t xbzrle_pages;
uint64_t xbzrle_cache_miss;
@@ -238,7 +240,7 @@ typedef struct AccountingInfo {

static AccountingInfo acct_info;

-static void acct_clear(void)
+void acct_clear(void)
{
memset(&acct_info, 0, sizeof(acct_info));
}
@@ -273,6 +275,16 @@ uint64_t norm_mig_pages_transferred(void)
return acct_info.norm_pages;
}

+uint64_t norm_mig_log_dirty_time(void)
+{
+    return acct_info.log_dirty_time;
+}
+
+uint64_t norm_mig_bitmap_time(void)
+{
+    return acct_info.migration_bitmap_time;
+}
+
uint64_t xbzrle_mig_bytes_transferred(void)
{
return acct_info.xbzrle_bytes;
@@ -479,27 +491,35 @@ static void migration_bitmap_sync(void)
static int64_t num_dirty_pages_period;
int64_t end_time;
int64_t bytes_xfer_now;
+    int64_t begin_time;
+    int64_t dirty_time;

if (!bytes_xfer_prev) {
bytes_xfer_prev = ram_bytes_transferred();
}

+    begin_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
if (!start_time) {
start_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
}
-
trace_migration_bitmap_sync_start();
address_space_sync_dirty_bitmap(&address_space_memory);

+    dirty_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
+
QTAILQ_FOREACH(block, &ram_list.blocks, next) {
migration_bitmap_sync_range(block->mr->ram_addr, block->length);
}
+
trace_migration_bitmap_sync_end(migration_dirty_pages
- num_dirty_pages_init);
num_dirty_pages_period += migration_dirty_pages - num_dirty_pages_init;
end_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);

-    /* more than 1 second = 1000 millisecons */
+    acct_info.log_dirty_time += dirty_time - begin_time;
+    acct_info.migration_bitmap_time += end_time - dirty_time;
+
+    /* more than 1 second = 1000 milliseconds */
if (end_time > start_time + 1000) {
if (migrate_auto_converge()) {
/* The following detection logic can be refined later. For now:
@@ -547,6 +567,10 @@ static int ram_save_block(QEMUFile *f, bool last_stage)
if (!block)
block = QTAILQ_FIRST(&ram_list.blocks);

+    if (!last_seen_block) {
+        last_seen_block = block;
+    }
+
while (true) {
mr = block->mr;
offset = migration_bitmap_find_and_reset_dirty(mr, offset);
@@ -574,7 +598,7 @@ static int ram_save_block(QEMUFile *f, bool last_stage)
/* In doubt sent page as normal */
bytes_sent = -1;
ret = ram_control_save_page(f, block->offset,
-                               offset, TARGET_PAGE_SIZE, &bytes_sent);
+                       block->host, offset, TARGET_PAGE_SIZE, &bytes_sent);

XBZRLE_cache_lock();

@@ -617,6 +641,7 @@ static int ram_save_block(QEMUFile *f, bool last_stage)
/* XBZRLE overflow or normal page */
if (bytes_sent == -1) {
bytes_sent = save_block_hdr(f, block, offset, cont, RAM_SAVE_FLAG_PAGE);
+                if (ret != RAM_SAVE_CONTROL_DELAYED) {
if (send_async) {
qemu_put_buffer_async(f, p, TARGET_PAGE_SIZE);
} else {
@@ -625,6 +650,7 @@ static int ram_save_block(QEMUFile *f, bool last_stage)
bytes_sent += TARGET_PAGE_SIZE;
acct_info.norm_pages++;
}
+            }

XBZRLE_cache_unlock();
/* if page is unmodified, continue to the next */
@@ -712,13 +738,13 @@ static void ram_migration_cancel(void *opaque)
migration_end();
}

-static void reset_ram_globals(void)
+static void reset_ram_globals(bool reset_bulk_stage)
{
last_seen_block = NULL;
last_sent_block = NULL;
last_offset = 0;
last_version = ram_list.version;
-    ram_bulk_stage = true;
+    ram_bulk_stage = reset_bulk_stage;
}

#define MAX_WAIT 50 /* ms, half buffered_file limit */
@@ -728,6 +754,15 @@ static int ram_save_setup(QEMUFile *f, void *opaque)
RAMBlock *block;
int64_t ram_pages = last_ram_offset() >> TARGET_PAGE_BITS;

+    /*
+     * RAM stays open during micro-checkpointing for the next transaction.
+     */
+    if (migration_is_mc(migrate_get_current())) {
+        qemu_mutex_lock_ramlist();
+        reset_ram_globals(false);
+        goto skip_setup;
+    }
+
migration_bitmap = bitmap_new(ram_pages);
bitmap_set(migration_bitmap, 0, ram_pages);
migration_dirty_pages = ram_pages;
@@ -768,12 +803,14 @@ static int ram_save_setup(QEMUFile *f, void *opaque)
qemu_mutex_lock_iothread();
qemu_mutex_lock_ramlist();
bytes_transferred = 0;
-    reset_ram_globals();
+    reset_ram_globals(true);

memory_global_dirty_log_start();
migration_bitmap_sync();
qemu_mutex_unlock_iothread();

+skip_setup:
+
qemu_put_be64(f, ram_bytes_total() | RAM_SAVE_FLAG_MEM_SIZE);

QTAILQ_FOREACH(block, &ram_list.blocks, next) {
@@ -802,7 +839,7 @@ static int ram_save_iterate(QEMUFile *f, void *opaque)
qemu_mutex_lock_ramlist();

if (ram_list.version != last_version) {
-        reset_ram_globals();
+        reset_ram_globals(true);
}

ram_control_before_iterate(f, RAM_CONTROL_ROUND);
@@ -883,7 +920,15 @@ static int ram_save_complete(QEMUFile *f, void *opaque)
}

ram_control_after_iterate(f, RAM_CONTROL_FINISH);
+
+    /*
+     * Only cleanup at the end of normal migrations
+     * or if the MC destination failed and we got an error.
+     * Otherwise, we are (or will soon be) in MIG_STATE_CHECKPOINTING.
+     */
+    if(!migrate_use_mc() || migration_has_failed(migrate_get_current())) {
migration_end();
+    }

qemu_mutex_unlock_ramlist();
qemu_put_be64(f, RAM_SAVE_FLAG_EOS);
@@ -1062,13 +1107,18 @@ static int ram_load(QEMUFile *f, void *opaque, int version_id)
ram_handle_compressed(host, ch, TARGET_PAGE_SIZE);
} else if (flags & RAM_SAVE_FLAG_PAGE) {
void *host;
+            int r;

host = host_from_stream_offset(f, addr, flags);
if (!host) {
return -EINVAL;
}

+            r = ram_control_load_page(f, host, TARGET_PAGE_SIZE);
+
+            if (r == RAM_LOAD_CONTROL_NOT_SUPP) {
qemu_get_buffer(f, host, TARGET_PAGE_SIZE);
+            }
} else if (flags & RAM_SAVE_FLAG_XBZRLE) {
void *host = host_from_stream_offset(f, addr, flags);
if (!host) {
