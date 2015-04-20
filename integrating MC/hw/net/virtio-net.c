index fd23c46..439477b 100644
--- a/hw/net/virtio-net.c
+++ b/hw/net/virtio-net.c
@@ -222,13 +222,33 @@ static char *mac_strdup_printf(const uint8_t *mac)
mac[1], mac[2], mac[3], mac[4], mac[5]);
}

+static intList *get_vlan_table(VirtIONet *n)
+{
+    intList *list, *entry;
+    int i, j;
+
+    list = NULL;
+    for (i = 0; i < MAX_VLAN >> 5; i++) {
+        for (j = 0; n->vlans[i] && j <= 0x1f; j++) {
+            if (n->vlans[i] & (1U << j)) {
+                entry = g_malloc0(sizeof(*entry));
+                entry->value = (i << 5) + j;
+                entry->next = list;
+                list = entry;
+            }
+        }
+    }
+
+    return list;
+}
+
static RxFilterInfo *virtio_net_query_rxfilter(NetClientState *nc)
{
VirtIONet *n = qemu_get_nic_opaque(nc);
+    VirtIODevice *vdev = VIRTIO_DEVICE(n);
RxFilterInfo *info;
strList *str_list, *entry;
-    intList *int_list, *int_entry;
-    int i, j;
+    int i;

info = g_malloc0(sizeof(*info));
info->name = g_strdup(nc->name);
@@ -273,19 +293,15 @@ static RxFilterInfo *virtio_net_query_rxfilter(NetClientState *nc)
str_list = entry;
}
info->multicast_table = str_list;
+    info->vlan_table = get_vlan_table(n);

-    int_list = NULL;
-    for (i = 0; i < MAX_VLAN >> 5; i++) {
-        for (j = 0; n->vlans[i] && j < 0x1f; j++) {
-            if (n->vlans[i] & (1U << j)) {
-                int_entry = g_malloc0(sizeof(*int_entry));
-                int_entry->value = (i << 5) + j;
-                int_entry->next = int_list;
-                int_list = int_entry;
-            }
-        }
+    if (!((1 << VIRTIO_NET_F_CTRL_VLAN) & vdev->guest_features)) {
+        info->vlan = RX_STATE_ALL;
+    } else if (!info->vlan_table) {
+        info->vlan = RX_STATE_NONE;
+    } else {
+        info->vlan = RX_STATE_NORMAL;
}
-    info->vlan_table = int_list;

/* enable event notification after query */
nc->rxfilter_notify_enabled = 1;
@@ -514,6 +530,12 @@ static void virtio_net_set_features(VirtIODevice *vdev, uint32_t features)
}
vhost_net_ack_features(tap_get_vhost_net(nc->peer), features);
}
+
+    if ((1 << VIRTIO_NET_F_CTRL_VLAN) & features) {
+        memset(n->vlans, 0, MAX_VLAN >> 3);
+    } else {
+        memset(n->vlans, 0xff, MAX_VLAN >> 3);
+    }
}

static int virtio_net_handle_rx_mode(VirtIONet *n, uint8_t cmd,
