index acd97a8..8b3b22f 100644
--- a/vl.c
+++ b/vl.c
@@ -583,6 +583,7 @@ static const RunStateTransition runstate_transitions_def[] = {

{ RUN_STATE_PAUSED, RUN_STATE_RUNNING },
{ RUN_STATE_PAUSED, RUN_STATE_FINISH_MIGRATE },
+    { RUN_STATE_PAUSED, RUN_STATE_CHECKPOINT_VM },

{ RUN_STATE_POSTMIGRATE, RUN_STATE_RUNNING },
{ RUN_STATE_POSTMIGRATE, RUN_STATE_FINISH_MIGRATE },
@@ -593,14 +594,18 @@ static const RunStateTransition runstate_transitions_def[] = {

{ RUN_STATE_FINISH_MIGRATE, RUN_STATE_RUNNING },
{ RUN_STATE_FINISH_MIGRATE, RUN_STATE_POSTMIGRATE },
+    { RUN_STATE_FINISH_MIGRATE, RUN_STATE_CHECKPOINT_VM },

{ RUN_STATE_RESTORE_VM, RUN_STATE_RUNNING },

+    { RUN_STATE_CHECKPOINT_VM, RUN_STATE_RUNNING },
+
{ RUN_STATE_RUNNING, RUN_STATE_DEBUG },
{ RUN_STATE_RUNNING, RUN_STATE_INTERNAL_ERROR },
{ RUN_STATE_RUNNING, RUN_STATE_IO_ERROR },
{ RUN_STATE_RUNNING, RUN_STATE_PAUSED },
{ RUN_STATE_RUNNING, RUN_STATE_FINISH_MIGRATE },
+    { RUN_STATE_RUNNING, RUN_STATE_CHECKPOINT_VM },
{ RUN_STATE_RUNNING, RUN_STATE_RESTORE_VM },
{ RUN_STATE_RUNNING, RUN_STATE_SAVE_VM },
{ RUN_STATE_RUNNING, RUN_STATE_SHUTDOWN },
@@ -616,9 +621,11 @@ static const RunStateTransition runstate_transitions_def[] = {
{ RUN_STATE_RUNNING, RUN_STATE_SUSPENDED },
{ RUN_STATE_SUSPENDED, RUN_STATE_RUNNING },
{ RUN_STATE_SUSPENDED, RUN_STATE_FINISH_MIGRATE },
+    { RUN_STATE_SUSPENDED, RUN_STATE_CHECKPOINT_VM },

{ RUN_STATE_WATCHDOG, RUN_STATE_RUNNING },
{ RUN_STATE_WATCHDOG, RUN_STATE_FINISH_MIGRATE },
+    { RUN_STATE_WATCHDOG, RUN_STATE_CHECKPOINT_VM },

{ RUN_STATE_GUEST_PANICKED, RUN_STATE_RUNNING },
{ RUN_STATE_GUEST_PANICKED, RUN_STATE_FINISH_MIGRATE },
@@ -2970,14 +2977,6 @@ int main(int argc, char **argv, char **envp)
qemu_init_exec_dir(argv[0]);

g_mem_set_vtable(&mem_trace);
-    if (!g_thread_supported()) {
-#if !GLIB_CHECK_VERSION(2, 31, 0)
-        g_thread_init(NULL);
-#else
-        fprintf(stderr, "glib threading failed to initialize.n");
-        exit(1);
-#endif
-    }

module_call_init(MODULE_INIT_QOM);

@@ -4293,6 +4292,8 @@ int main(int argc, char **argv, char **envp)
default_drive(default_sdcard, snapshot, IF_SD, 0, SD_OPTS);

register_savevm_live(NULL, "ram", 0, 4, &savevm_ram_handlers, NULL);
+    register_savevm(NULL, "mc", -1, MC_VERSION, mc_info_save,
+                                mc_info_load, NULL);

if (nb_numa_nodes > 0) {
int i;
