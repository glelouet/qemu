index 1104d61..6a30cd4 100644
--- a/cpus.c
+++ b/cpus.c
@@ -531,8 +531,15 @@ static int do_vm_stop(RunState state)
pause_all_vcpus();
runstate_set(state);
vm_state_notify(0, state);
+        /*
+         * If MC is enabled, libvirt gets confused
+         * because it thinks the VM is stopped when
+         * its just being micro-checkpointed.
+         */
+        if(state != RUN_STATE_CHECKPOINT_VM) {
monitor_protocol_event(QEVENT_STOP, NULL);
}
+    }

bdrv_drain_all();
ret = bdrv_flush_all();
