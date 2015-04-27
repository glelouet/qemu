index 99a2c58..d2380b6 100644
--- a/linux-user/elfload.c
+++ b/linux-user/elfload.c
@@ -1076,7 +1076,7 @@ struct exec
#define TARGET_ELF_PAGESTART(_v) ((_v) & ~(unsigned long)(TARGET_ELF_EXEC_PAGESIZE-1))
#define TARGET_ELF_PAGEOFFSET(_v) ((_v) & (TARGET_ELF_EXEC_PAGESIZE-1))

-#define DLINFO_ITEMS 13
+#define DLINFO_ITEMS 14

static inline void memcpy_fromfs(void * to, const void * from, unsigned long n)
{
