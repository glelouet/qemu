index ed58f0e..068b2c1 100644
--- a/hmp.h
+++ b/hmp.h
@@ -60,6 +60,7 @@ void hmp_drive_mirror(Monitor *mon, const QDict *qdict);
void hmp_drive_backup(Monitor *mon, const QDict *qdict);
void hmp_migrate_cancel(Monitor *mon, const QDict *qdict);
void hmp_migrate_set_downtime(Monitor *mon, const QDict *qdict);
+void hmp_migrate_set_mc_delay(Monitor *mon, const QDict *qdict);
void hmp_migrate_set_speed(Monitor *mon, const QDict *qdict);
void hmp_migrate_set_capability(Monitor *mon, const QDict *qdict);
void hmp_migrate_set_cache_size(Monitor *mon, const QDict *qdict);
