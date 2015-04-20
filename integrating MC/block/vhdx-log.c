index 02755b8..a77c040 100644
--- a/block/vhdx-log.c
+++ b/block/vhdx-log.c
@@ -578,7 +578,7 @@ static int vhdx_validate_log_entry(BlockDriverState *bs, BDRVVHDXState *s,
total_sectors = hdr.entry_length / VHDX_LOG_SECTOR_SIZE;


-    /* read_desc() will incrememnt the read idx */
+    /* read_desc() will increment the read idx */
ret = vhdx_log_read_desc(bs, s, log, &desc_buffer);
if (ret < 0) {
goto free_and_exit;
