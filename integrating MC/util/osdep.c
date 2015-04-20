index bd4f530..a9029f8 100644
--- a/util/osdep.c
+++ b/util/osdep.c
@@ -436,6 +436,24 @@ int socket_init(void)
return 0;
}

+/* Ensure that glib is running in multi-threaded mode */
+static void __attribute__((constructor)) thread_init(void)
+{
+    if (!g_thread_supported()) {
+#if !GLIB_CHECK_VERSION(2, 31, 0)
+        /* Old versions of glib require explicit initialization.  Failure to do
+         * this results in the single-threaded code paths being taken inside
+         * glib.  For example, the g_slice allocator will not be thread-safe
+         * and cause crashes.
+         */
+        g_thread_init(NULL);
+#else
+        fprintf(stderr, "glib threading failed to initialize.n");
+        exit(1);
+#endif
+    }
+}
+
#ifndef CONFIG_IOVEC
/* helper function for iov_send_recv() */
static ssize_t
