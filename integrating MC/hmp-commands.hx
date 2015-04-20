index f3fc514..2066c76 100644
--- a/hmp-commands.hx
+++ b/hmp-commands.hx
@@ -965,6 +965,20 @@ Set maximum tolerated downtime (in seconds) for migration.
ETEXI

{
+        .name       = "migrate-set-mc-delay",
+        .args_type  = "value:i",
+        .params     = "value",
+        .help       = "Set maximum delay (in milliseconds) between micro-checkpoints",
+        .mhandler.cmd = hmp_migrate_set_mc_delay,
+    },
+
+STEXI
+@item migrate-set-mc-delay @var{millisecond}
+@findex migrate-set-mc-delay
+Set maximum delay (in milliseconds) between micro-checkpoints.
+ETEXI
+
+    {
.name       = "migrate_set_capability",
.args_type  = "capability:s,state:b",
.params     = "capability state",
