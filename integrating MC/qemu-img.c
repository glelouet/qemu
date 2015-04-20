index 2e40cc1..77d946b 100644
--- a/qemu-img.c
+++ b/qemu-img.c
@@ -565,7 +565,7 @@ static int img_check(int argc, char **argv)
static const struct option long_options[] = {
{"help", no_argument, 0, 'h'},
{"format", required_argument, 0, 'f'},
-            {"repair", no_argument, 0, 'r'},
+            {"repair", required_argument, 0, 'r'},
{"output", required_argument, 0, OPTION_OUTPUT},
{0, 0, 0, 0}
};
