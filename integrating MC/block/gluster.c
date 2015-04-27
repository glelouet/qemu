index a44d612..8836085 100644
--- a/block/gluster.c
+++ b/block/gluster.c
@@ -80,7 +80,7 @@ static int parse_volume_options(GlusterConf *gconf, char *path)
* 'server' specifies the server where the volume file specification for
* the given volume resides. This can be either hostname, ipv4 address
* or ipv6 address. ipv6 address needs to be within square brackets [ ].
- * If transport type is 'unix', then 'server' field should not be specifed.
+ * If transport type is 'unix', then 'server' field should not be specified.
* The 'socket' field needs to be populated with the path to unix domain
* socket.
*
