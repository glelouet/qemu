index 57572c4..aaa010e 100644
--- a/trace/simple.c
+++ b/trace/simple.c
@@ -414,15 +414,6 @@ bool trace_backend_init(const char *events, const char *file)
{
GThread *thread;

-    if (!g_thread_supported()) {
-#if !GLIB_CHECK_VERSION(2, 31, 0)
-        g_thread_init(NULL);
-#else
-        fprintf(stderr, "glib threading failed to initialize.n");
-        exit(1);
-#endif
-    }
-
#if !GLIB_CHECK_VERSION(2, 31, 0)
trace_available_cond = g_cond_new();
trace_empty_cond = g_cond_new();
