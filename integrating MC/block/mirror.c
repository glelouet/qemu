index dd5ee05..0ef41f9 100644
--- a/block/mirror.c
+++ b/block/mirror.c
@@ -98,8 +98,15 @@ static void mirror_iteration_done(MirrorOp *op, int ret)

qemu_iovec_destroy(&op->qiov);
g_slice_free(MirrorOp, op);
+
+    /* Enter coroutine when it is not sleeping.  The coroutine sleeps to
+     * rate-limit itself.  The coroutine will eventually resume since there is
+     * a sleep timeout so don't wake it early.
+     */
+    if (s->common.busy) {
qemu_coroutine_enter(s->common.co, NULL);
}
+}

static void mirror_write_complete(void *opaque, int ret)
{
@@ -139,11 +146,12 @@ static void mirror_read_complete(void *opaque, int ret)
mirror_write_complete, op);
}

-static void coroutine_fn mirror_iteration(MirrorBlockJob *s)
+static uint64_t coroutine_fn mirror_iteration(MirrorBlockJob *s)
{
BlockDriverState *source = s->common.bs;
int nb_sectors, sectors_per_chunk, nb_chunks;
int64_t end, sector_num, next_chunk, next_sector, hbitmap_next_sector;
+    uint64_t delay_ns;
MirrorOp *op;

s->sector_num = hbitmap_iter_next(&s->hbi);
@@ -231,7 +239,12 @@ static void coroutine_fn mirror_iteration(MirrorBlockJob *s)
nb_chunks += added_chunks;
next_sector += added_sectors;
next_chunk += added_chunks;
-    } while (next_sector < end);
+        if (!s->synced && s->common.speed) {
+            delay_ns = ratelimit_calculate_delay(&s->limit, added_sectors);
+        } else {
+            delay_ns = 0;
+        }
+    } while (delay_ns == 0 && next_sector < end);

/* Allocate a MirrorOp that is used as an AIO callback.  */
op = g_slice_new(MirrorOp);
@@ -268,6 +281,7 @@ static void coroutine_fn mirror_iteration(MirrorBlockJob *s)
trace_mirror_one_iteration(s, sector_num, nb_sectors);
bdrv_aio_readv(source, sector_num, &op->qiov, nb_sectors,
mirror_read_complete, op);
+    return delay_ns;
}

static void mirror_free_init(MirrorBlockJob *s)
@@ -362,7 +376,7 @@ static void coroutine_fn mirror_run(void *opaque)
bdrv_dirty_iter_init(bs, s->dirty_bitmap, &s->hbi);
last_pause_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
for (;;) {
-        uint64_t delay_ns;
+        uint64_t delay_ns = 0;
int64_t cnt;
bool should_complete;

@@ -386,10 +400,12 @@ static void coroutine_fn mirror_run(void *opaque)
qemu_coroutine_yield();
continue;
} else if (cnt != 0) {
-                mirror_iteration(s);
+                delay_ns = mirror_iteration(s);
+                if (delay_ns == 0) {
continue;
}
}
+        }

should_complete = false;
if (s->in_flight == 0 && cnt == 0) {
@@ -432,17 +448,10 @@ static void coroutine_fn mirror_run(void *opaque)
}

ret = 0;
-        trace_mirror_before_sleep(s, cnt, s->synced);
+        trace_mirror_before_sleep(s, cnt, s->synced, delay_ns);
if (!s->synced) {
/* Publish progress */
s->common.offset = (end - cnt) * BDRV_SECTOR_SIZE;
-
-            if (s->common.speed) {
-                delay_ns = ratelimit_calculate_delay(&s->limit, sectors_per_chunk);
-            } else {
-                delay_ns = 0;
-            }
-
block_job_sleep_ns(&s->common, QEMU_CLOCK_REALTIME, delay_ns);
if (block_job_is_cancelled(&s->common)) {
break;
