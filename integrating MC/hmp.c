index 2f279c4..22589c6 100644
--- a/hmp.c
+++ b/hmp.c
@@ -203,6 +203,23 @@ void hmp_info_migrate(Monitor *mon, const QDict *qdict)
info->disk->total >> 10);
}

+    if (info->has_mc) {
+        monitor_printf(mon, "checkpoints: %" PRIu64 "n",
+                       info->mc->checkpoints);
+        monitor_printf(mon, "xmit_time: %" PRIu64 " msn",
+                       info->mc->xmit_time);
+        monitor_printf(mon, "log_dirty_time: %" PRIu64 " msn",
+                       info->mc->log_dirty_time);
+        monitor_printf(mon, "migration_bitmap_time: %" PRIu64 " msn",
+                       info->mc->migration_bitmap_time);
+        monitor_printf(mon, "ram_copy_time: %" PRIu64 " msn",
+                       info->mc->ram_copy_time);
+        monitor_printf(mon, "copy_mbps: %0.2f mbpsn",
+                       info->mc->copy_mbps);
+        monitor_printf(mon, "throughput: %0.2f mbpsn",
+                       info->mc->mbps);
+    }
+
if (info->has_xbzrle_cache) {
monitor_printf(mon, "cache size: %" PRIu64 " bytesn",
info->xbzrle_cache->cache_size);
@@ -1012,6 +1029,12 @@ void hmp_migrate_set_downtime(Monitor *mon, const QDict *qdict)
qmp_migrate_set_downtime(value, NULL);
}

+void hmp_migrate_set_mc_delay(Monitor *mon, const QDict *qdict)
+{
+    int64_t value = qdict_get_int(qdict, "value");
+    qmp_migrate_set_mc_delay(value, NULL);
+}
+
void hmp_migrate_set_cache_size(Monitor *mon, const QDict *qdict)
{
int64_t value = qdict_get_int(qdict, "value");
