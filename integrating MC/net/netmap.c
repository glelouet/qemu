index 8213304..0c1772b 100644
--- a/net/netmap.c
+++ b/net/netmap.c
@@ -177,7 +177,7 @@ static void netmap_poll(NetClientState *nc, bool enable)
NetmapState *s = DO_UPCAST(NetmapState, nc, nc);

if (s->read_poll != enable || s->write_poll != enable) {
-        s->read_poll = enable;
+        s->write_poll = enable;
s->read_poll  = enable;
netmap_update_fd_handler(s);
}
