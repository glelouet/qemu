index 34478f0..d4ada4f 100644
--- a/hw/scsi/spapr_vscsi.c
+++ b/hw/scsi/spapr_vscsi.c
@@ -690,7 +690,7 @@ static void vscsi_inquiry_no_target(VSCSIState *s, vscsi_req *req)
int rc, len, alen;

/* We dont do EVPD. Also check that page_code is 0 */
-    if ((cdb[1] & 0x01) || (cdb[1] & 0x01) || cdb[2] != 0) {
+    if ((cdb[1] & 0x01) || cdb[2] != 0) {
/* Send INVALID FIELD IN CDB */
vscsi_makeup_sense(s, req, ILLEGAL_REQUEST, 0x24, 0);
vscsi_send_rsp(s, req, CHECK_CONDITION, 0, 0);
