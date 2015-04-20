index f7bd024..0eb33ee 100644
--- a/block/sheepdog.c
+++ b/block/sheepdog.c
@@ -909,9 +909,9 @@ static void co_write_request(void *opaque)
}

/*
- * Return a socket discriptor to read/write objects.
+ * Return a socket descriptor to read/write objects.
*
- * We cannot use this discriptor for other operations because
+ * We cannot use this descriptor for other operations because
* the block driver may be on waiting response from the server.
*/
static int get_sheep_fd(BDRVSheepdogState *s)
@@ -1896,7 +1896,7 @@ static int sd_create_branch(BDRVSheepdogState *s)

/*
* Even If deletion fails, we will just create extra snapshot based on
-     * the workding VDI which was supposed to be deleted. So no need to
+     * the working VDI which was supposed to be deleted. So no need to
* false bail out.
*/
deleted = sd_delete(s);
@@ -2194,7 +2194,7 @@ cleanup:
* We implement rollback(loadvm) operation to the specified snapshot by
* 1) switch to the snapshot
* 2) rely on sd_create_branch to delete working VDI and
- * 3) create a new working VDI based on the speicified snapshot
+ * 3) create a new working VDI based on the specified snapshot
*/
static int sd_snapshot_goto(BlockDriverState *bs, const char *snapshot_id)
{
