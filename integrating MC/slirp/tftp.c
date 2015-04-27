index 1a79c45..a329fb2 100644
--- a/slirp/tftp.c
+++ b/slirp/tftp.c
@@ -279,7 +279,7 @@ static void tftp_handle_rrq(Slirp *slirp, struct tftp_t *tp, int pktlen)

spt = &slirp->tftp_sessions[s];

-  /* unspecifed prefix means service disabled */
+  /* unspecified prefix means service disabled */
if (!slirp->tftp_prefix) {
tftp_send_error(spt, 2, "Access violation", tp);
return;
