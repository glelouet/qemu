index 7d344b9..e35bff7 100644
--- a/hw/scsi/vmw_pvscsi.c
+++ b/hw/scsi/vmw_pvscsi.c
@@ -479,12 +479,13 @@ static void
pvscsi_command_complete(SCSIRequest *req, uint32_t status, size_t resid)
{
PVSCSIRequest *pvscsi_req = req->hba_private;
-    PVSCSIState *s = pvscsi_req->dev;
+    PVSCSIState *s;

if (!pvscsi_req) {
trace_pvscsi_command_complete_not_found(req->tag);
return;
}
+    s = pvscsi_req->dev;

if (resid) {
/* Short transfer.  */
