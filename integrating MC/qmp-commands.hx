index a22621f..5e3cae5 100644
--- a/qmp-commands.hx
+++ b/qmp-commands.hx
@@ -754,6 +754,29 @@ Example:
EQMP

{
+        .name       = "migrate-set-mc-delay",
+        .args_type  = "value:i",
+        .mhandler.cmd_new = qmp_marshal_input_migrate_set_mc_delay,
+    },
+
+SQMP
+migrate-set-mc-delay
+--------------------
+
+Set maximum delay (in milliseconds) between micro-checkpoints.
+
+Arguments:
+
+- "value": maximum delay (json-int)
+
+Example:
+
+-> { "execute": "migrate-set-mc-delay", "arguments": { "value": 100 } }
+<- { "return": {} }
+
+EQMP
+
+    {
.name       = "client_migrate_info",
.args_type  = "protocol:s,hostname:s,port:i?,tls-port:i?,cert-subject:s?",
.params     = "protocol hostname port tls-port cert-subject",
@@ -3407,6 +3430,7 @@ Each array entry contains the following:
- "promiscuous": promiscuous mode is enabled (json-bool)
- "multicast": multicast receive state (one of 'normal', 'none', 'all')
- "unicast": unicast receive state  (one of 'normal', 'none', 'all')
+- "vlan": vlan receive state (one of 'normal', 'none', 'all') (Since 2.0)
- "broadcast-allowed": allow to receive broadcast (json-bool)
- "multicast-overflow": multicast table is overflowed (json-bool)
- "unicast-overflow": unicast table is overflowed (json-bool)
@@ -3424,6 +3448,7 @@ Example:
"name": "vnet0",
"main-mac": "52:54:00:12:34:56",
"unicast": "normal",
+            "vlan": "normal",
"vlan-table": [
4,
0
