index baabf86..f056e40 100644
--- a/ui/gtk.c
+++ b/ui/gtk.c
@@ -54,7 +54,9 @@
#include <gdk/gdkkeysyms.h>
#include <glib/gi18n.h>
#include <locale.h>
+#if defined(CONFIG_VTE)
#include <vte/vte.h>
+#endif
#include <math.h>

#include "trace.h"
@@ -68,6 +70,9 @@

#define MAX_VCS 10

+#if !defined(CONFIG_VTE)
+# define VTE_CHECK_VERSION(a, b, c) 0
+#endif

/* Compatibility define to let us build on both Gtk2 and Gtk3 */
#if GTK_CHECK_VERSION(3, 0, 0)
@@ -105,8 +110,10 @@ typedef struct VirtualConsole
{
GtkWidget *menu_item;
GtkWidget *terminal;
+#if defined(CONFIG_VTE)
GtkWidget *scrolled_window;
CharDriverState *chr;
+#endif
int fd;
} VirtualConsole;

@@ -1063,6 +1070,7 @@ static void gd_change_page(GtkNotebook *nb, gpointer arg1, guint arg2,
if (arg2 == 0) {
gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(s->vga_item), TRUE);
} else {
+#if defined(CONFIG_VTE)
VirtualConsole *vc = &s->vc[arg2 - 1];
VteTerminal *term = VTE_TERMINAL(vc->terminal);
int width, height;
@@ -1072,6 +1080,9 @@ static void gd_change_page(GtkNotebook *nb, gpointer arg1, guint arg2,

gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(vc->menu_item), TRUE);
gtk_widget_set_size_request(vc->terminal, width, height);
+#else
+        g_assert_not_reached();
+#endif
}

gtk_widget_set_sensitive(s->grab_item, on_vga);
@@ -1117,7 +1128,7 @@ static int gd_vc_chr_write(CharDriverState *chr, const uint8_t *buf, int len)
{
VirtualConsole *vc = chr->opaque;

-    return write(vc->fd, buf, len);
+    return vc ? write(vc->fd, buf, len) : len;
}

static int nb_vcs;
@@ -1142,6 +1153,7 @@ void early_gtk_display_init(void)
register_vc_handler(gd_vc_handler);
}

+#if defined(CONFIG_VTE)
static gboolean gd_vc_in(GIOChannel *chan, GIOCondition cond, void *opaque)
{
VirtualConsole *vc = opaque;
@@ -1157,10 +1169,12 @@ static gboolean gd_vc_in(GIOChannel *chan, GIOCondition cond, void *opaque)

return TRUE;
}
+#endif

static GSList *gd_vc_init(GtkDisplayState *s, VirtualConsole *vc, int index, GSList *group,
GtkWidget *view_menu)
{
+#if defined(CONFIG_VTE)
const char *label;
char buffer[32];
char path[32];
@@ -1230,6 +1244,7 @@ static GSList *gd_vc_init(GtkDisplayState *s, VirtualConsole *vc, int index, GSL
chan = g_io_channel_unix_new(vc->fd);
g_io_add_watch(chan, G_IO_IN, gd_vc_in, vc);

+#endif /* CONFIG_VTE */
return group;
}

