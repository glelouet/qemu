index eeb4302..58e69b7 100644
--- a/migration-rdma.c
+++ b/migration-rdma.c
@@ -31,28 +31,39 @@
//#define DEBUG_RDMA_VERBOSE
//#define DEBUG_RDMA_REALLY_VERBOSE

+/*
+ * Ability to runtime-enable debug statements while inside GDB.
+ * Choices are 1, 2, or 3 (so far).
+ */
+#if !defined(DEBUG_RDMA) || !defined(DEBUG_RDMA_VERBOSE) || +    !defined(DEBUG_RDMA_REALLY_VERBOSE)
+static int rdma_debug = 0;
+#endif
+
+#define RPRINTF(fmt, ...) printf("rdma: " fmt, ## __VA_ARGS__)
+
#ifdef DEBUG_RDMA
#define DPRINTF(fmt, ...) -    do { printf("rdma: " fmt, ## __VA_ARGS__); } while (0)
+    do { RPRINTF(fmt, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) -    do { } while (0)
+    do { if (rdma_debug >= 1) RPRINTF(fmt, ## __VA_ARGS__); } while (0)
#endif

#ifdef DEBUG_RDMA_VERBOSE
#define DDPRINTF(fmt, ...) -    do { printf("rdma: " fmt, ## __VA_ARGS__); } while (0)
+    do { RPRINTF(fmt, ## __VA_ARGS__); } while (0)
#else
#define DDPRINTF(fmt, ...) -    do { } while (0)
+    do { if (rdma_debug >= 2) RPRINTF(fmt, ## __VA_ARGS__); } while (0)
#endif

#ifdef DEBUG_RDMA_REALLY_VERBOSE
#define DDDPRINTF(fmt, ...) -    do { printf("rdma: " fmt, ## __VA_ARGS__); } while (0)
+    do { RPRINTF(fmt, ## __VA_ARGS__); } while (0)
#else
#define DDDPRINTF(fmt, ...) -    do { } while (0)
+    do { if (rdma_debug >= 3) RPRINTF(fmt, ## __VA_ARGS__); } while (0)
#endif

/*
@@ -60,17 +71,20 @@
*/
#define ERROR(errp, fmt, ...)      do { +        Error **e = errp;          fprintf(stderr, "RDMA ERROR: " fmt "n", ## __VA_ARGS__); -        if (errp && (*(errp) == NULL)) { -            error_setg(errp, "RDMA ERROR: " fmt, ## __VA_ARGS__); +        if (e && ((*e) == NULL)) { +            error_setg(e, "RDMA ERROR: " fmt, ## __VA_ARGS__);          }      } while (0)

+#define SET_ERROR(rdma, err) if (!rdma->error_state) rdma->error_state = err
+
#define RDMA_RESOLVE_TIMEOUT_MS 10000

/* Do not merge data if larger than this. */
#define RDMA_MERGE_MAX (2 * 1024 * 1024)
-#define RDMA_SIGNALED_SEND_MAX (RDMA_MERGE_MAX / 4096)
+#define RDMA_SEND_MAX (RDMA_MERGE_MAX / 4096)

#define RDMA_REG_CHUNK_SHIFT 20 /* 1 MB */

@@ -87,18 +101,30 @@
*/
#define RDMA_CONTROL_MAX_BUFFER (512 * 1024)
#define RDMA_CONTROL_MAX_COMMANDS_PER_MESSAGE 4096
-
-#define RDMA_CONTROL_VERSION_CURRENT 1
/*
* Capabilities for negotiation.
*/
#define RDMA_CAPABILITY_PIN_ALL 0x01
+#define RDMA_CAPABILITY_KEEPALIVE 0x02
+
+/*
+ * Max # missed keepalive before we assume remote side is unavailable.
+ */
+#define RDMA_CONNECTION_INTERVAL_MS 300
+#define RDMA_KEEPALIVE_INTERVAL_MS 300
+#define RDMA_KEEPALIVE_FIRST_MISSED_OFFSET 1000
+#define RDMA_MAX_LOST_KEEPALIVE 10
+#define RDMA_MAX_STARTUP_MISSED_KEEPALIVE 400

/*
* Add the other flags above to this list of known capabilities
* as they are introduced.
*/
-static uint32_t known_capabilities = RDMA_CAPABILITY_PIN_ALL;
+static uint32_t known_capabilities = RDMA_CAPABILITY_PIN_ALL
+                                   | RDMA_CAPABILITY_KEEPALIVE
+                                   ;
+static QEMUTimer *connection_timer = NULL;
+static QEMUTimer *keepalive_timer = NULL;

#define CHECK_ERROR_STATE()      do { @@ -143,14 +169,18 @@ static uint32_t known_capabilities = RDMA_CAPABILITY_PIN_ALL;
*/
enum {
RDMA_WRID_NONE = 0,
-    RDMA_WRID_RDMA_WRITE = 1,
+    RDMA_WRID_RDMA_WRITE_REMOTE = 1,
+    RDMA_WRID_RDMA_WRITE_LOCAL = 2,
+    RDMA_WRID_RDMA_KEEPALIVE = 3,
RDMA_WRID_SEND_CONTROL = 2000,
RDMA_WRID_RECV_CONTROL = 4000,
};

const char *wrid_desc[] = {
[RDMA_WRID_NONE] = "NONE",
-    [RDMA_WRID_RDMA_WRITE] = "WRITE RDMA",
+    [RDMA_WRID_RDMA_WRITE_REMOTE] = "WRITE RDMA REMOTE",
+    [RDMA_WRID_RDMA_WRITE_LOCAL] = "WRITE RDMA LOCAL",
+    [RDMA_WRID_RDMA_KEEPALIVE] = "KEEPALIVE",
[RDMA_WRID_SEND_CONTROL] = "CONTROL SEND",
[RDMA_WRID_RECV_CONTROL] = "CONTROL RECV",
};
@@ -216,21 +246,41 @@ typedef struct {
/*
* Negotiate RDMA capabilities during connection-setup time.
*/
-typedef struct {
+typedef struct QEMU_PACKED RDMACapabilities {
uint32_t version;
uint32_t flags;
+    uint32_t keepalive_rkey;
+    uint64_t keepalive_addr;
} RDMACapabilities;

+static uint64_t htonll(uint64_t v)
+{
+    union { uint32_t lv[2]; uint64_t llv; } u;
+    u.lv[0] = htonl(v >> 32);
+    u.lv[1] = htonl(v & 0xFFFFFFFFULL);
+    return u.llv;
+}
+
+static uint64_t ntohll(uint64_t v) {
+    union { uint32_t lv[2]; uint64_t llv; } u;
+    u.llv = v;
+    return ((uint64_t)ntohl(u.lv[0]) << 32) | (uint64_t) ntohl(u.lv[1]);
+}
+
static void caps_to_network(RDMACapabilities *cap)
{
cap->version = htonl(cap->version);
cap->flags = htonl(cap->flags);
+    cap->keepalive_rkey = htonl(cap->keepalive_rkey);
+    cap->keepalive_addr = htonll(cap->keepalive_addr);
}

static void network_to_caps(RDMACapabilities *cap)
{
cap->version = ntohl(cap->version);
cap->flags = ntohl(cap->flags);
+    cap->keepalive_rkey = ntohl(cap->keepalive_rkey);
+    cap->keepalive_addr = ntohll(cap->keepalive_addr);
}

/*
@@ -245,8 +295,12 @@ typedef struct RDMALocalBlock {
uint64_t remote_host_addr; /* remote virtual address */
uint64_t offset;
uint64_t length;
-    struct   ibv_mr **pmr;     /* MRs for chunk-level registration */
+    struct ibv_mr **pmr;      /* MRs for remote chunk-level registration */
struct ibv_mr *mr;        /* MR for non-chunk-level registration */
+    struct ibv_mr **pmr_src;  /* MRs for copy chunk-level registration */
+    struct ibv_mr *mr_src;    /* MR for copy non-chunk-level registration */
+    struct ibv_mr **pmr_dest; /* MRs for copy chunk-level registration */
+    struct ibv_mr *mr_dest;   /* MR for copy non-chunk-level registration */
uint32_t *remote_keys;    /* rkeys for chunk-level registration */
uint32_t remote_rkey;     /* rkeys for non-chunk-level registration */
int      index;           /* which block are we */
@@ -271,20 +325,6 @@ typedef struct QEMU_PACKED RDMARemoteBlock {
uint32_t padding;
} RDMARemoteBlock;

-static uint64_t htonll(uint64_t v)
-{
-    union { uint32_t lv[2]; uint64_t llv; } u;
-    u.lv[0] = htonl(v >> 32);
-    u.lv[1] = htonl(v & 0xFFFFFFFFULL);
-    return u.llv;
-}
-
-static uint64_t ntohll(uint64_t v) {
-    union { uint32_t lv[2]; uint64_t llv; } u;
-    u.llv = v;
-    return ((uint64_t)ntohl(u.lv[0]) << 32) | (uint64_t) ntohl(u.lv[1]);
-}
-
static void remote_block_to_network(RDMARemoteBlock *rb)
{
rb->remote_host_addr = htonll(rb->remote_host_addr);
@@ -313,15 +353,74 @@ typedef struct RDMALocalBlocks {
} RDMALocalBlocks;

/*
+ * We provide RDMA to QEMU by way of 2 mechanisms:
+ *
+ * 1. Local copy to remote copy
+ * 2. Local copy to local copy - like memcpy().
+ *
+ * Three instances of this structure are maintained inside of RDMAContext
+ * to manage both mechanisms.
+ */
+typedef struct RDMACurrentChunk {
+    /* store info about current buffer so that we can
+       merge it with future sends */
+    uint64_t current_addr;
+    uint64_t current_length;
+    /* index of ram block the current buffer belongs to */
+    int current_block_idx;
+    /* index of the chunk in the current ram block */
+    int current_chunk;
+
+    uint64_t block_offset;
+    uint64_t offset;
+
+    /* parameters for qemu_rdma_write() */
+    uint64_t chunk_idx;
+    uint8_t *chunk_start;
+    uint8_t *chunk_end;
+    RDMALocalBlock *block;
+    uint8_t *addr;
+    uint64_t chunks;
+} RDMACurrentChunk;
+
+/*
+ * Three copies of the following strucuture are used to hold the infiniband
+ * connection variables for each of the aformentioned mechanisms, one for
+ * remote copy and two local copy.
+ */
+typedef struct RDMALocalContext {
+    bool source;
+    bool dest;
+    bool connected;
+    char *host;
+    int port;
+    struct rdma_cm_id *cm_id;
+    struct rdma_cm_id *listen_id;
+    struct rdma_event_channel *channel;
+    struct ibv_context *verbs;
+    struct ibv_pd *pd;
+    struct ibv_comp_channel *comp_chan;
+    struct ibv_cq *cq;
+    struct ibv_qp *qp;
+    int nb_sent;
+    int64_t start_time;
+    int max_nb_sent;
+    const char * id_str;
+} RDMALocalContext;
+
+/*
* Main data structure for RDMA state.
* While there is only one copy of this structure being allocated right now,
* this is the place where one would start if you wanted to consider
* having more than one RDMA connection open at the same time.
+ *
+ * It is used for performing both local and remote RDMA operations
+ * with a single RDMA connection.
+ *
+ * Local operations are done by allocating separate queue pairs after
+ * the initial RDMA remote connection is initalized.
*/
typedef struct RDMAContext {
-    char *host;
-    int port;
-
RDMAWorkRequestData wr_data[RDMA_WRID_MAX];

/*
@@ -333,37 +432,15 @@ typedef struct RDMAContext {
*/
int control_ready_expected;

-    /* number of outstanding writes */
+    /* number of posts */
int nb_sent;

-    /* store info about current buffer so that we can
-       merge it with future sends */
-    uint64_t current_addr;
-    uint64_t current_length;
-    /* index of ram block the current buffer belongs to */
-    int current_index;
-    /* index of the chunk in the current ram block */
-    int current_chunk;
+    RDMACurrentChunk chunk_remote;
+    RDMACurrentChunk chunk_local_src;
+    RDMACurrentChunk chunk_local_dest;

bool pin_all;
-
-    /*
-     * infiniband-specific variables for opening the device
-     * and maintaining connection state and so forth.
-     *
-     * cm_id also has ibv_context, rdma_event_channel, and ibv_qp in
-     * cm_id->verbs, cm_id->channel, and cm_id->qp.
-     */
-    struct rdma_cm_id *cm_id;               /* connection manager ID */
-    struct rdma_cm_id *listen_id;
-    bool connected;
-
-    struct ibv_context          *verbs;
-    struct rdma_event_channel   *channel;
-    struct ibv_qp *qp;                      /* queue pair */
-    struct ibv_comp_channel *comp_channel;  /* completion channel */
-    struct ibv_pd *pd;                      /* protection domain */
-    struct ibv_cq *cq;                      /* completion queue */
+    bool do_keepalive;

/*
* If a previous write failed (perhaps because of a failed
@@ -384,17 +461,150 @@ typedef struct RDMAContext {
* Then use coroutine yield function.
* Source runs in a thread, so we don't care.
*/
-    int migration_started_on_destination;
+    bool migration_started;

int total_registrations;
int total_writes;

int unregister_current, unregister_next;
-    uint64_t unregistrations[RDMA_SIGNALED_SEND_MAX];
+    uint64_t unregistrations[RDMA_SEND_MAX];

GHashTable *blockmap;
+
+    uint64_t keepalive;
+    uint64_t last_keepalive;
+    uint64_t nb_missed_keepalive;
+    uint64_t next_keepalive;
+    struct ibv_mr *keepalive_mr;
+    struct ibv_mr *next_keepalive_mr;
+    uint32_t keepalive_rkey;
+    uint64_t keepalive_addr;
+    bool keepalive_startup;
+
+    RDMALocalContext lc_src;
+    RDMALocalContext lc_dest;
+    RDMALocalContext lc_remote;
+
+    /* who are we? */
+    bool source;
+    bool dest;
} RDMAContext;

+static void close_ibv(RDMAContext *rdma, RDMALocalContext *lc)
+{
+
+    if (lc->cq) {
+        ibv_destroy_cq(lc->cq);
+        lc->cq = NULL;
+    }
+
+    if (lc->comp_chan) {
+        ibv_destroy_comp_channel(lc->comp_chan);
+        lc->comp_chan = NULL;
+    }
+
+    if (lc->pd) {
+        ibv_dealloc_pd(lc->pd);
+        lc->pd = NULL;
+    }
+
+    if (lc->verbs) {
+        ibv_close_device(lc->verbs);
+        lc->verbs = NULL;
+    }
+
+    if (lc->listen_id) {
+        rdma_destroy_id(lc->listen_id);
+        lc->listen_id = NULL;
+    }
+    if (lc->cm_id) {
+        if (lc->qp) {
+            struct ibv_qp_attr attr = {.qp_state = IBV_QPS_ERR };
+            ibv_modify_qp(lc->qp, &attr, IBV_QP_STATE);
+            rdma_destroy_qp(lc->cm_id);
+            lc->qp = NULL;
+        }
+
+        rdma_destroy_id(lc->cm_id);
+        rdma->lc_remote.cm_id = NULL;
+    }
+    if (lc->channel) {
+        rdma_destroy_event_channel(lc->channel);
+        lc->channel = NULL;
+    }
+
+    g_free(lc->host);
+    lc->host = NULL;
+}
+
+/*
+ * Create protection domain and completion queues
+ */
+static int qemu_rdma_alloc_pd_cq_qp(RDMAContext *rdma, RDMALocalContext *lc)
+{
+    struct rlimit r = { .rlim_cur = RLIM_INFINITY, .rlim_max = RLIM_INFINITY };
+    struct ibv_qp_init_attr attr = { 0 };
+    int ret;
+
+    if (getrlimit(RLIMIT_MEMLOCK, &r) < 0) {
+        perror("getrlimit");
+        ERROR(NULL, "getrlimit(RLIMIT_MEMLOCK)");
+        goto err_alloc;
+    }
+
+    DPRINTF("MemLock Limits cur: %" PRId64 " max: %" PRId64 "n",
+            r.rlim_cur, r.rlim_max);
+
+    lc->pd = ibv_alloc_pd(lc->verbs);
+    if (!lc->pd) {
+        ERROR(NULL, "allocate protection domain");
+        goto err_alloc;
+    }
+
+    /* create completion channel */
+    lc->comp_chan = ibv_create_comp_channel(lc->verbs);
+    if (!lc->comp_chan) {
+        ERROR(NULL, "allocate completion channel");
+        goto err_alloc;
+    }
+
+    /*
+     * Completion queue can be filled by both read and write work requests,
+     * so must reflect the sum of both possible queue sizes.
+     */
+    lc->cq = ibv_create_cq(lc->verbs, (RDMA_SEND_MAX * 3), NULL,
+                           lc->comp_chan, 0);
+    if (!lc->cq) {
+        ERROR(NULL, "allocate completion queue");
+        goto err_alloc;
+    }
+
+    attr.cap.max_send_wr = RDMA_SEND_MAX;
+    attr.cap.max_recv_wr = 3;
+    attr.cap.max_send_sge = 1;
+    attr.cap.max_recv_sge = 1;
+    attr.send_cq = lc->cq;
+    attr.recv_cq = lc->cq;
+    attr.qp_type = IBV_QPT_RC;
+
+    ret = rdma_create_qp(lc->cm_id, lc->pd, &attr);
+    if (ret) {
+        ERROR(NULL, "alloc queue pair");
+        goto err_alloc;
+    }
+
+    lc->qp = lc->cm_id->qp;
+
+    return 0;
+
+err_alloc:
+    ERROR(NULL, "allocating pd and cq and qp! Your mlock()"
+                " limits may be too low. Please check $ ulimit -a # and "
+                "search for 'ulimit -l' in the output");
+    close_ibv(rdma, lc);
+    return -EINVAL;
+}
+
/*
* Interface to the rest of the migration call stack.
*/
@@ -440,7 +650,7 @@ typedef struct QEMU_PACKED {
uint64_t current_addr;  /* offset into the ramblock of the chunk */
uint64_t chunk;         /* chunk to lookup if unregistering */
} key;
-    uint32_t current_index; /* which ramblock the chunk belongs to */
+    uint32_t current_block_idx;     /* which ramblock the chunk belongs to */
uint32_t padding;
uint64_t chunks;            /* how many sequential chunks to register */
} RDMARegister;
@@ -448,14 +658,14 @@ typedef struct QEMU_PACKED {
static void register_to_network(RDMARegister *reg)
{
reg->key.current_addr = htonll(reg->key.current_addr);
-    reg->current_index = htonl(reg->current_index);
+    reg->current_block_idx = htonl(reg->current_block_idx);
reg->chunks = htonll(reg->chunks);
}

static void network_to_register(RDMARegister *reg)
{
reg->key.current_addr = ntohll(reg->key.current_addr);
-    reg->current_index = ntohl(reg->current_index);
+    reg->current_block_idx = ntohl(reg->current_block_idx);
reg->chunks = ntohll(reg->chunks);
}

@@ -578,10 +788,10 @@ static int __qemu_rdma_add_block(RDMAContext *rdma, void *host_addr,

g_hash_table_insert(rdma->blockmap, (void *) block_offset, block);

-    DDPRINTF("Added Block: %d, addr: %" PRIu64 ", offset: %" PRIu64
-           " length: %" PRIu64 " end: %" PRIu64 " bits %" PRIu64 " chunks %dn",
-            local->nb_blocks, (uint64_t) block->local_host_addr, block->offset,
-            block->length, (uint64_t) (block->local_host_addr + block->length),
+    DDPRINTF("Added Block: %d, addr: %p, offset: %" PRIu64
+           " length: %" PRIu64 " end: %p bits %" PRIu64 " chunks %dn",
+            local->nb_blocks, block->local_host_addr, block->offset,
+            block->length, block->local_host_addr + block->length,
BITS_TO_LONGS(block->nb_chunks) *
sizeof(unsigned long) * 8, block->nb_chunks);

@@ -621,36 +831,52 @@ static int qemu_rdma_init_ram_blocks(RDMAContext *rdma)
return 0;
}

-static int __qemu_rdma_delete_block(RDMAContext *rdma, ram_addr_t block_offset)
+static void qemu_rdma_free_pmrs(RDMAContext *rdma, RDMALocalBlock *block,
+                               struct ibv_mr ***mrs)
{
-    RDMALocalBlocks *local = &rdma->local_ram_blocks;
-    RDMALocalBlock *block = g_hash_table_lookup(rdma->blockmap,
-        (void *) block_offset);
-    RDMALocalBlock *old = local->block;
-    int x;
-
-    assert(block);
-
-    if (block->pmr) {
+    if (*mrs) {
int j;

for (j = 0; j < block->nb_chunks; j++) {
-            if (!block->pmr[j]) {
+            if (!(*mrs)[j]) {
continue;
}
-            ibv_dereg_mr(block->pmr[j]);
+            ibv_dereg_mr((*mrs)[j]);
rdma->total_registrations--;
}
-        g_free(block->pmr);
-        block->pmr = NULL;
+        g_free(*mrs);
+
+        *mrs = NULL;
+    }
}

-    if (block->mr) {
-        ibv_dereg_mr(block->mr);
+static void qemu_rdma_free_mr(RDMAContext *rdma, struct ibv_mr **mr)
+{
+    if (*mr) {
+        ibv_dereg_mr(*mr);
rdma->total_registrations--;
-        block->mr = NULL;
+        *mr = NULL;
+    }
}

+static int __qemu_rdma_delete_block(RDMAContext *rdma, ram_addr_t block_offset)
+{
+    RDMALocalBlocks *local = &rdma->local_ram_blocks;
+    RDMALocalBlock *block = g_hash_table_lookup(rdma->blockmap,
+        (void *) block_offset);
+    RDMALocalBlock *old = local->block;
+    int x;
+
+    assert(block);
+
+    qemu_rdma_free_pmrs(rdma, block, &block->pmr);
+    qemu_rdma_free_pmrs(rdma, block, &block->pmr_src);
+    qemu_rdma_free_pmrs(rdma, block, &block->pmr_dest);
+
+    qemu_rdma_free_mr(rdma, &block->mr);
+    qemu_rdma_free_mr(rdma, &block->mr_src);
+    qemu_rdma_free_mr(rdma, &block->mr_dest);
+
g_free(block->transit_bitmap);
block->transit_bitmap = NULL;

@@ -674,7 +900,12 @@ static int __qemu_rdma_delete_block(RDMAContext *rdma, ram_addr_t block_offset)
}

if (block->index < (local->nb_blocks - 1)) {
-            memcpy(local->block + block->index, old + (block->index + 1),
+            RDMALocalBlock * end = old + (block->index + 1);
+            for (x = 0; x < (local->nb_blocks - (block->index + 1)); x++) {
+                end[x].index--;
+            }
+
+            memcpy(local->block + block->index, end,
sizeof(RDMALocalBlock) *
(local->nb_blocks - (block->index + 1)));
}
@@ -683,6 +914,10 @@ static int __qemu_rdma_delete_block(RDMAContext *rdma, ram_addr_t block_offset)
local->block = NULL;
}

+    g_free(old);
+
+    local->nb_blocks--;
+
DDPRINTF("Deleted Block: %d, addr: %" PRIu64 ", offset: %" PRIu64
" length: %" PRIu64 " end: %" PRIu64 " bits %" PRIu64 " chunks %dn",
local->nb_blocks, (uint64_t) block->local_host_addr, block->offset,
@@ -690,10 +925,6 @@ static int __qemu_rdma_delete_block(RDMAContext *rdma, ram_addr_t block_offset)
BITS_TO_LONGS(block->nb_chunks) *
sizeof(unsigned long) * 8, block->nb_chunks);

-    g_free(old);
-
-    local->nb_blocks--;
-
if (local->nb_blocks) {
for (x = 0; x < local->nb_blocks; x++) {
g_hash_table_insert(rdma->blockmap, (void *)local->block[x].offset,
@@ -873,190 +1104,66 @@ static int qemu_rdma_broken_ipv6_kernel(Error **errp, struct ibv_context *verbs)
return 0;
}

-/*
- * Figure out which RDMA device corresponds to the requested IP hostname
- * Also create the initial connection manager identifiers for opening
- * the connection.
- */
-static int qemu_rdma_resolve_host(RDMAContext *rdma, Error **errp)
+static int qemu_rdma_reg_keepalive(RDMAContext *rdma)
{
-    int ret;
-    struct rdma_addrinfo *res;
-    char port_str[16];
-    struct rdma_cm_event *cm_event;
-    char ip[40] = "unknown";
-    struct rdma_addrinfo *e;
-
-    if (rdma->host == NULL || !strcmp(rdma->host, "")) {
-        ERROR(errp, "RDMA hostname has not been set");
-        return -EINVAL;
-    }
-
-    /* create CM channel */
-    rdma->channel = rdma_create_event_channel();
-    if (!rdma->channel) {
-        ERROR(errp, "could not create CM channel");
-        return -EINVAL;
-    }
-
-    /* create CM id */
-    ret = rdma_create_id(rdma->channel, &rdma->cm_id, NULL, RDMA_PS_TCP);
-    if (ret) {
-        ERROR(errp, "could not create channel id");
-        goto err_resolve_create_id;
-    }
-
-    snprintf(port_str, 16, "%d", rdma->port);
-    port_str[15] = '0';
-
-    ret = rdma_getaddrinfo(rdma->host, port_str, NULL, &res);
-    if (ret < 0) {
-        ERROR(errp, "could not rdma_getaddrinfo address %s", rdma->host);
-        goto err_resolve_get_addr;
-    }
-
-    for (e = res; e != NULL; e = e->ai_next) {
-        inet_ntop(e->ai_family,
-            &((struct sockaddr_in *) e->ai_dst_addr)->sin_addr, ip, sizeof ip);
-        DPRINTF("Trying %s => %sn", rdma->host, ip);
-
-        ret = rdma_resolve_addr(rdma->cm_id, NULL, e->ai_dst_addr,
-                RDMA_RESOLVE_TIMEOUT_MS);
-        if (!ret) {
-            if (e->ai_family == AF_INET6) {
-                ret = qemu_rdma_broken_ipv6_kernel(errp, rdma->cm_id->verbs);
-                if (ret) {
-                    continue;
-                }
-            }
-            goto route;
-        }
-    }
-
-    ERROR(errp, "could not resolve address %s", rdma->host);
-    goto err_resolve_get_addr;
-
-route:
-    qemu_rdma_dump_gid("source_resolve_addr", rdma->cm_id);
+    rdma->keepalive_mr = ibv_reg_mr(rdma->lc_remote.pd,
+            &rdma->keepalive, sizeof(rdma->keepalive),
+            IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);

-    ret = rdma_get_cm_event(rdma->channel, &cm_event);
-    if (ret) {
-        ERROR(errp, "could not perform event_addr_resolved");
-        goto err_resolve_get_addr;
+    if (!rdma->keepalive_mr) {
+        perror("Failed to register keepalive location!");
+        SET_ERROR(rdma, -ENOMEM);
+        goto err_alloc;
}

-    if (cm_event->event != RDMA_CM_EVENT_ADDR_RESOLVED) {
-        ERROR(errp, "result not equal to event_addr_resolved %s",
-                rdma_event_str(cm_event->event));
-        perror("rdma_resolve_addr");
-        ret = -EINVAL;
-        goto err_resolve_get_addr;
-    }
-    rdma_ack_cm_event(cm_event);
+    rdma->next_keepalive_mr = ibv_reg_mr(rdma->lc_remote.pd,
+            &rdma->next_keepalive, sizeof(rdma->next_keepalive),
+            IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);

-    /* resolve route */
-    ret = rdma_resolve_route(rdma->cm_id, RDMA_RESOLVE_TIMEOUT_MS);
-    if (ret) {
-        ERROR(errp, "could not resolve rdma route");
-        goto err_resolve_get_addr;
+    if (!rdma->next_keepalive_mr) {
+        perror("Failed to register next keepalive location!");
+        SET_ERROR(rdma, -ENOMEM);
+        goto err_alloc;
}

-    ret = rdma_get_cm_event(rdma->channel, &cm_event);
-    if (ret) {
-        ERROR(errp, "could not perform event_route_resolved");
-        goto err_resolve_get_addr;
-    }
-    if (cm_event->event != RDMA_CM_EVENT_ROUTE_RESOLVED) {
-        ERROR(errp, "result not equal to event_route_resolved: %s",
-                        rdma_event_str(cm_event->event));
-        rdma_ack_cm_event(cm_event);
-        ret = -EINVAL;
-        goto err_resolve_get_addr;
-    }
-    rdma_ack_cm_event(cm_event);
-    rdma->verbs = rdma->cm_id->verbs;
-    qemu_rdma_dump_id("source_resolve_host", rdma->cm_id->verbs);
-    qemu_rdma_dump_gid("source_resolve_host", rdma->cm_id);
return 0;

-err_resolve_get_addr:
-    rdma_destroy_id(rdma->cm_id);
-    rdma->cm_id = NULL;
-err_resolve_create_id:
-    rdma_destroy_event_channel(rdma->channel);
-    rdma->channel = NULL;
-    return ret;
-}
-
-/*
- * Create protection domain and completion queues
- */
-static int qemu_rdma_alloc_pd_cq(RDMAContext *rdma)
-{
-    /* allocate pd */
-    rdma->pd = ibv_alloc_pd(rdma->verbs);
-    if (!rdma->pd) {
-        fprintf(stderr, "failed to allocate protection domainn");
-        return -1;
-    }
+err_alloc:

-    /* create completion channel */
-    rdma->comp_channel = ibv_create_comp_channel(rdma->verbs);
-    if (!rdma->comp_channel) {
-        fprintf(stderr, "failed to allocate completion channeln");
-        goto err_alloc_pd_cq;
+    if (rdma->keepalive_mr) {
+        ibv_dereg_mr(rdma->keepalive_mr);
+        rdma->keepalive_mr = NULL;
}

-    /*
-     * Completion queue can be filled by both read and write work requests,
-     * so must reflect the sum of both possible queue sizes.
-     */
-    rdma->cq = ibv_create_cq(rdma->verbs, (RDMA_SIGNALED_SEND_MAX * 3),
-            NULL, rdma->comp_channel, 0);
-    if (!rdma->cq) {
-        fprintf(stderr, "failed to allocate completion queuen");
-        goto err_alloc_pd_cq;
+    if (rdma->next_keepalive_mr) {
+        ibv_dereg_mr(rdma->next_keepalive_mr);
+        rdma->next_keepalive_mr = NULL;
}

-    return 0;
-
-err_alloc_pd_cq:
-    if (rdma->pd) {
-        ibv_dealloc_pd(rdma->pd);
-    }
-    if (rdma->comp_channel) {
-        ibv_destroy_comp_channel(rdma->comp_channel);
-    }
-    rdma->pd = NULL;
-    rdma->comp_channel = NULL;
return -1;
-
}

-/*
- * Create queue pairs.
- */
-static int qemu_rdma_alloc_qp(RDMAContext *rdma)
+static int qemu_rdma_reg_whole_mr(RDMAContext *rdma,
+                                  struct ibv_pd *pd,
+                                  struct ibv_mr **mr,
+                                  int index)
{
-    struct ibv_qp_init_attr attr = { 0 };
-    int ret;
-
-    attr.cap.max_send_wr = RDMA_SIGNALED_SEND_MAX;
-    attr.cap.max_recv_wr = 3;
-    attr.cap.max_send_sge = 1;
-    attr.cap.max_recv_sge = 1;
-    attr.send_cq = rdma->cq;
-    attr.recv_cq = rdma->cq;
-    attr.qp_type = IBV_QPT_RC;
+    RDMALocalBlocks *local = &rdma->local_ram_blocks;

-    ret = rdma_create_qp(rdma->cm_id, rdma->pd, &attr);
-    if (ret) {
+    *mr = ibv_reg_mr(pd,
+                local->block[index].local_host_addr,
+                local->block[index].length,
+                IBV_ACCESS_LOCAL_WRITE |
+                IBV_ACCESS_REMOTE_WRITE
+                );
+    if (!(*mr)) {
+        perror("Failed to register local dest ram block!n");
return -1;
}
+    rdma->total_registrations++;

-    rdma->qp = rdma->cm_id->qp;
return 0;
-}
+};

static int qemu_rdma_reg_whole_ram_blocks(RDMAContext *rdma)
{
@@ -1064,18 +1171,23 @@ static int qemu_rdma_reg_whole_ram_blocks(RDMAContext *rdma)
RDMALocalBlocks *local = &rdma->local_ram_blocks;

for (i = 0; i < local->nb_blocks; i++) {
-        local->block[i].mr =
-            ibv_reg_mr(rdma->pd,
-                    local->block[i].local_host_addr,
-                    local->block[i].length,
-                    IBV_ACCESS_LOCAL_WRITE |
-                    IBV_ACCESS_REMOTE_WRITE
-                    );
-        if (!local->block[i].mr) {
-            perror("Failed to register local dest ram block!n");
+        if (qemu_rdma_reg_whole_mr(rdma, rdma->lc_remote.pd, &local->block[i].mr, i)) {
break;
}
-        rdma->total_registrations++;
+
+        if (migrate_use_mc_rdma_copy()) {
+            if (rdma->source) {
+                if (qemu_rdma_reg_whole_mr(rdma, rdma->lc_src.pd,
+                        &local->block[i].mr_src, i)) {
+                    break;
+                }
+            } else {
+                if (qemu_rdma_reg_whole_mr(rdma, rdma->lc_dest.pd,
+                        &local->block[i].mr_dest, i)) {
+                    break;
+                }
+            }
+        }
}

if (i >= local->nb_blocks) {
@@ -1083,8 +1195,12 @@ static int qemu_rdma_reg_whole_ram_blocks(RDMAContext *rdma)
}

for (i--; i >= 0; i--) {
-        ibv_dereg_mr(local->block[i].mr);
-        rdma->total_registrations--;
+        qemu_rdma_free_mr(rdma, &local->block[i].mr);
+        if (migrate_use_mc_rdma_copy()) {
+            qemu_rdma_free_mr(rdma, rdma->source ?
+                                &local->block[i].mr_src :
+                                &local->block[i].mr_dest);
+        }
}

return -1;
@@ -1129,24 +1245,34 @@ static int qemu_rdma_search_ram_block(RDMAContext *rdma,
* to perform the actual RDMA operation.
*/
static int qemu_rdma_register_and_get_keys(RDMAContext *rdma,
-        RDMALocalBlock *block, uint8_t *host_addr,
-        uint32_t *lkey, uint32_t *rkey, int chunk,
-        uint8_t *chunk_start, uint8_t *chunk_end)
+                                           RDMACurrentChunk *cc,
+                                           RDMALocalContext *lc,
+                                           bool copy,
+                                           uint32_t *lkey,
+                                           uint32_t *rkey)
{
-    if (block->mr) {
+    struct ibv_mr ***pmr = copy ? (rdma->source ? &cc->block->pmr_src :
+                           &cc->block->pmr_dest) : &cc->block->pmr;
+    struct ibv_mr **mr = copy ? (rdma->source ? &cc->block->mr_src :
+                         &cc->block->mr_dest) : &cc->block->mr;
+
+    /*
+     * Use pre-registered keys for the entire VM, if available.
+     */
+    if (*mr) {
if (lkey) {
-            *lkey = block->mr->lkey;
+            *lkey = (*mr)->lkey;
}
if (rkey) {
-            *rkey = block->mr->rkey;
+            *rkey = (*mr)->rkey;
}
return 0;
}

/* allocate memory to store chunk MRs */
-    if (!block->pmr) {
-        block->pmr = g_malloc0(block->nb_chunks * sizeof(struct ibv_mr *));
-        if (!block->pmr) {
+    if (!(*pmr)) {
+        *pmr = g_malloc0(cc->block->nb_chunks * sizeof(struct ibv_mr *));
+        if (!(*pmr)) {
return -1;
}
}
@@ -1154,38 +1280,38 @@ static int qemu_rdma_register_and_get_keys(RDMAContext *rdma,
/*
* If 'rkey', then we're the destination, so grant access to the source.
*
-     * If 'lkey', then we're the source VM, so grant access only to ourselves.
+     * If 'lkey', then we're the source, so grant access only to ourselves.
*/
-    if (!block->pmr[chunk]) {
-        uint64_t len = chunk_end - chunk_start;
+    if (!(*pmr)[cc->chunk_idx]) {
+        uint64_t len = cc->chunk_end - cc->chunk_start;

DDPRINTF("Registering %" PRIu64 " bytes @ %pn",
-                 len, chunk_start);
+                 len, cc->chunk_start);

-        block->pmr[chunk] = ibv_reg_mr(rdma->pd,
-                chunk_start, len,
+        (*pmr)[cc->chunk_idx] = ibv_reg_mr(lc->pd, cc->chunk_start, len,
(rkey ? (IBV_ACCESS_LOCAL_WRITE |
IBV_ACCESS_REMOTE_WRITE) : 0));

-        if (!block->pmr[chunk]) {
+        if (!(*pmr)[cc->chunk_idx]) {
perror("Failed to register chunk!");
-            fprintf(stderr, "Chunk details: block: %d chunk index %d"
+            fprintf(stderr, "Chunk details: block: %d chunk index %" PRIu64
" start %" PRIu64 " end %" PRIu64 " host %" PRIu64
" local %" PRIu64 " registrations: %dn",
-                            block->index, chunk, (uint64_t) chunk_start,
-                            (uint64_t) chunk_end, (uint64_t) host_addr,
-                            (uint64_t) block->local_host_addr,
+                            cc->block->index, cc->chunk_idx, (uint64_t) cc->chunk_start,
+                            (uint64_t) cc->chunk_end, (uint64_t) cc->addr,
+                            (uint64_t) cc->block->local_host_addr,
rdma->total_registrations);
return -1;
}
+
rdma->total_registrations++;
}

if (lkey) {
-        *lkey = block->pmr[chunk]->lkey;
+        *lkey = (*pmr)[cc->chunk_idx]->lkey;
}
if (rkey) {
-        *rkey = block->pmr[chunk]->rkey;
+        *rkey = (*pmr)[cc->chunk_idx]->rkey;
}
return 0;
}
@@ -1196,7 +1322,7 @@ static int qemu_rdma_register_and_get_keys(RDMAContext *rdma,
*/
static int qemu_rdma_reg_control(RDMAContext *rdma, int idx)
{
-    rdma->wr_data[idx].control_mr = ibv_reg_mr(rdma->pd,
+    rdma->wr_data[idx].control_mr = ibv_reg_mr(rdma->lc_remote.pd,
rdma->wr_data[idx].control, RDMA_CONTROL_MAX_BUFFER,
IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);
if (rdma->wr_data[idx].control_mr) {
@@ -1257,11 +1383,11 @@ static int qemu_rdma_unregister_waiting(RDMAContext *rdma)
uint64_t wr_id = rdma->unregistrations[rdma->unregister_current];
uint64_t chunk =
(wr_id & RDMA_WRID_CHUNK_MASK) >> RDMA_WRID_CHUNK_SHIFT;
-        uint64_t index =
+        uint64_t block_index =
(wr_id & RDMA_WRID_BLOCK_MASK) >> RDMA_WRID_BLOCK_SHIFT;
RDMALocalBlock *block =
-            &(rdma->local_ram_blocks.block[index]);
-        RDMARegister reg = { .current_index = index };
+            &(rdma->local_ram_blocks.block[block_index]);
+        RDMARegister reg = { .current_block_idx = block_index };
RDMAControlHeader resp = { .type = RDMA_CONTROL_UNREGISTER_FINISHED,
};
RDMAControlHeader head = { .len = sizeof(RDMARegister),
@@ -1275,7 +1401,7 @@ static int qemu_rdma_unregister_waiting(RDMAContext *rdma)
rdma->unregistrations[rdma->unregister_current] = 0;
rdma->unregister_current++;

-        if (rdma->unregister_current == RDMA_SIGNALED_SEND_MAX) {
+        if (rdma->unregister_current == RDMA_SEND_MAX) {
rdma->unregister_current = 0;
}

@@ -1339,7 +1465,7 @@ static void qemu_rdma_signal_unregister(RDMAContext *rdma, uint64_t index,
uint64_t chunk, uint64_t wr_id)
{
if (rdma->unregistrations[rdma->unregister_next] != 0) {
-        fprintf(stderr, "rdma migration: queue is full!n");
+        ERROR(NULL, "queue is full!");
} else {
RDMALocalBlock *block = &(rdma->local_ram_blocks.block[index]);

@@ -1350,7 +1476,7 @@ static void qemu_rdma_signal_unregister(RDMAContext *rdma, uint64_t index,
rdma->unregistrations[rdma->unregister_next++] =
qemu_rdma_make_wrid(wr_id, index, chunk);

-            if (rdma->unregister_next == RDMA_SIGNALED_SEND_MAX) {
+            if (rdma->unregister_next == RDMA_SEND_MAX) {
rdma->unregister_next = 0;
}
} else {
@@ -1365,14 +1491,21 @@ static void qemu_rdma_signal_unregister(RDMAContext *rdma, uint64_t index,
* (of any kind) has completed.
* Return the work request ID that completed.
*/
-static uint64_t qemu_rdma_poll(RDMAContext *rdma, uint64_t *wr_id_out,
+static uint64_t qemu_rdma_poll(RDMAContext *rdma,
+                               RDMALocalContext *lc,
+                               uint64_t *wr_id_out,
uint32_t *byte_len)
{
+    int64_t current_time;
int ret;
struct ibv_wc wc;
uint64_t wr_id;

-    ret = ibv_poll_cq(rdma->cq, 1, &wc);
+    if (!lc->start_time) {
+        lc->start_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
+    }
+
+    ret = ibv_poll_cq(lc->cq, 1, &wc);

if (!ret) {
*wr_id_out = RDMA_WRID_NONE;
@@ -1380,16 +1513,17 @@ static uint64_t qemu_rdma_poll(RDMAContext *rdma, uint64_t *wr_id_out,
}

if (ret < 0) {
-        fprintf(stderr, "ibv_poll_cq return %d!n", ret);
+        fprintf(stderr, "ibv_poll_cq return %d (%s)!n", ret, lc->id_str);
return ret;
}

wr_id = wc.wr_id & RDMA_WRID_TYPE_MASK;

if (wc.status != IBV_WC_SUCCESS) {
-        fprintf(stderr, "ibv_poll_cq wc.status=%d %s!n",
-                        wc.status, ibv_wc_status_str(wc.status));
-        fprintf(stderr, "ibv_poll_cq wrid=%s!n", wrid_desc[wr_id]);
+        fprintf(stderr, "ibv_poll_cq wc.status=%d %s! (%s)n",
+                        wc.status, ibv_wc_status_str(wc.status), lc->id_str);
+        fprintf(stderr, "ibv_poll_cq wrid=%s! (%s)n", wrid_desc[wr_id],
+                                                        lc->id_str);

return -1;
}
@@ -1397,29 +1531,49 @@ static uint64_t qemu_rdma_poll(RDMAContext *rdma, uint64_t *wr_id_out,
if (rdma->control_ready_expected &&
(wr_id >= RDMA_WRID_RECV_CONTROL)) {
DDDPRINTF("completion %s #%" PRId64 " received (%" PRId64 ")"
-                  " left %dn", wrid_desc[RDMA_WRID_RECV_CONTROL],
-                  wr_id - RDMA_WRID_RECV_CONTROL, wr_id, rdma->nb_sent);
+                  " left %d (per qp %d) (%s)n",
+                  wrid_desc[RDMA_WRID_RECV_CONTROL],
+                  wr_id - RDMA_WRID_RECV_CONTROL, wr_id,
+                  rdma->nb_sent, lc->nb_sent, lc->id_str);
rdma->control_ready_expected = 0;
}

-    if (wr_id == RDMA_WRID_RDMA_WRITE) {
+    if (wr_id == RDMA_WRID_RDMA_WRITE_REMOTE) {
uint64_t chunk =
(wc.wr_id & RDMA_WRID_CHUNK_MASK) >> RDMA_WRID_CHUNK_SHIFT;
-        uint64_t index =
+        uint64_t block_idx =
(wc.wr_id & RDMA_WRID_BLOCK_MASK) >> RDMA_WRID_BLOCK_SHIFT;
-        RDMALocalBlock *block = &(rdma->local_ram_blocks.block[index]);
-
-        DDDPRINTF("completions %s (%" PRId64 ") left %d, "
-                 "block %" PRIu64 ", chunk: %" PRIu64 " %p %pn",
-                 print_wrid(wr_id), wr_id, rdma->nb_sent, index, chunk,
-                 block->local_host_addr, (void *)block->remote_host_addr);
+        RDMALocalBlock *block = &(rdma->local_ram_blocks.block[block_idx]);

clear_bit(chunk, block->transit_bitmap);

+        if (lc->nb_sent > lc->max_nb_sent) {
+            lc->max_nb_sent = lc->nb_sent;
+        }
+
+        current_time = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
+
+        if ((current_time - lc->start_time) > 1000) {
+            lc->start_time = current_time;
+            DDPRINTF("outstanding %s total: %d context: %d max %d (%s)n",
+                lc->id_str, rdma->nb_sent, lc->nb_sent, lc->max_nb_sent,
+                lc->id_str);
+        }
+
if (rdma->nb_sent > 0) {
rdma->nb_sent--;
}

+        if (lc->nb_sent > 0) {
+            lc->nb_sent--;
+        }
+
+        DDDPRINTF("completions %s (%" PRId64 ") left %d (per qp %d), "
+                 "block %" PRIu64 ", chunk: %" PRIu64 " %p %p (%s)n",
+                 print_wrid(wr_id), wr_id, rdma->nb_sent, lc->nb_sent,
+                 block_idx, chunk, block->local_host_addr,
+                 (void *)block->remote_host_addr, lc->id_str);
+
if (!rdma->pin_all) {
/*
* FYI: If one wanted to signal a specific chunk to be unregistered
@@ -1428,12 +1582,15 @@ static uint64_t qemu_rdma_poll(RDMAContext *rdma, uint64_t *wr_id_out,
* unregistered later.
*/
#ifdef RDMA_UNREGISTRATION_EXAMPLE
-            qemu_rdma_signal_unregister(rdma, index, chunk, wc.wr_id);
+             if (block->pmr[chunk]) {
+                 qemu_rdma_signal_unregister(rdma, block_idx, chunk, wc.wr_id);
+             }
#endif
}
} else {
-        DDDPRINTF("other completion %s (%" PRId64 ") received left %dn",
-            print_wrid(wr_id), wr_id, rdma->nb_sent);
+        DDDPRINTF("other completion %s (%"
+                  PRId64 ") received left %d (per qp %d) (%s)n",
+            print_wrid(wr_id), wr_id, rdma->nb_sent, lc->nb_sent, lc->id_str);
}

*wr_id_out = wc.wr_id;
@@ -1457,7 +1614,9 @@ static uint64_t qemu_rdma_poll(RDMAContext *rdma, uint64_t *wr_id_out,
* completions only need to be recorded, but do not actually
* need further processing.
*/
-static int qemu_rdma_block_for_wrid(RDMAContext *rdma, int wrid_requested,
+static int qemu_rdma_block_for_wrid(RDMAContext *rdma,
+                                    RDMALocalContext *lc,
+                                    int wrid_requested,
uint32_t *byte_len)
{
int num_cq_events = 0, ret = 0;
@@ -1465,12 +1624,15 @@ static int qemu_rdma_block_for_wrid(RDMAContext *rdma, int wrid_requested,
void *cq_ctx;
uint64_t wr_id = RDMA_WRID_NONE, wr_id_in;

-    if (ibv_req_notify_cq(rdma->cq, 0)) {
-        return -1;
+    ret = ibv_req_notify_cq(lc->cq, 0);
+    if (ret) {
+        perror("ibv_req_notify_cq");
+        return -ret;
}
+
/* poll cq first */
while (wr_id != wrid_requested) {
-        ret = qemu_rdma_poll(rdma, &wr_id_in, byte_len);
+        ret = qemu_rdma_poll(rdma, lc, &wr_id_in, byte_len);
if (ret < 0) {
return ret;
}
@@ -1481,9 +1643,9 @@ static int qemu_rdma_block_for_wrid(RDMAContext *rdma, int wrid_requested,
break;
}
if (wr_id != wrid_requested) {
-            DDDPRINTF("A Wanted wrid %s (%d) but got %s (%" PRIu64 ")n",
+            DDDPRINTF("A Wanted wrid %s (%d) but got %s (%" PRIu64 ") (%s)n",
print_wrid(wrid_requested),
-                wrid_requested, print_wrid(wr_id), wr_id);
+                wrid_requested, print_wrid(wr_id), wr_id, lc->id_str);
}
}

@@ -1496,23 +1658,27 @@ static int qemu_rdma_block_for_wrid(RDMAContext *rdma, int wrid_requested,
* Coroutine doesn't start until process_incoming_migration()
* so don't yield unless we know we're running inside of a coroutine.
*/
-        if (rdma->migration_started_on_destination) {
-            yield_until_fd_readable(rdma->comp_channel->fd);
+        if (qemu_in_coroutine()) {
+            yield_until_fd_readable(lc->comp_chan->fd);
}

-        if (ibv_get_cq_event(rdma->comp_channel, &cq, &cq_ctx)) {
+        ret = ibv_get_cq_event(lc->comp_chan, &cq, &cq_ctx);
+        if (ret < 0) {
perror("ibv_get_cq_event");
goto err_block_for_wrid;
}

num_cq_events++;

-        if (ibv_req_notify_cq(cq, 0)) {
+        ret = ibv_req_notify_cq(cq, 0);
+        if (ret) {
+            ret = -ret;
+            perror("ibv_req_notify_cq");
goto err_block_for_wrid;
}

while (wr_id != wrid_requested) {
-            ret = qemu_rdma_poll(rdma, &wr_id_in, byte_len);
+            ret = qemu_rdma_poll(rdma, lc, &wr_id_in, byte_len);
if (ret < 0) {
goto err_block_for_wrid;
}
@@ -1523,9 +1689,9 @@ static int qemu_rdma_block_for_wrid(RDMAContext *rdma, int wrid_requested,
break;
}
if (wr_id != wrid_requested) {
-                DDDPRINTF("B Wanted wrid %s (%d) but got %s (%" PRIu64 ")n",
+                DDDPRINTF("B Wanted wrid %s (%d) but got %s (%" PRIu64 ") (%s)n",
print_wrid(wrid_requested), wrid_requested,
-                    print_wrid(wr_id), wr_id);
+                    print_wrid(wr_id), wr_id, lc->id_str);
}
}

@@ -1588,19 +1754,17 @@ static int qemu_rdma_post_send_control(RDMAContext *rdma, uint8_t *buf,
memcpy(wr->control + sizeof(RDMAControlHeader), buf, head->len);
}

+    ret = ibv_post_send(rdma->lc_remote.qp, &send_wr, &bad_wr);

-    if (ibv_post_send(rdma->qp, &send_wr, &bad_wr)) {
-        return -1;
-    }
-
-    if (ret < 0) {
-        fprintf(stderr, "Failed to use post IB SEND for control!n");
-        return ret;
+    if (ret > 0) {
+        ERROR(NULL, "Failed to use post IB SEND for control!");
+        return -ret;
}

-    ret = qemu_rdma_block_for_wrid(rdma, RDMA_WRID_SEND_CONTROL, NULL);
+    ret = qemu_rdma_block_for_wrid(rdma, &rdma->lc_remote,
+                                   RDMA_WRID_SEND_CONTROL, NULL);
if (ret < 0) {
-        fprintf(stderr, "rdma migration: send polling control error!n");
+        ERROR(NULL, "send polling control!");
}

return ret;
@@ -1626,7 +1790,7 @@ static int qemu_rdma_post_recv_control(RDMAContext *rdma, int idx)
};


-    if (ibv_post_recv(rdma->qp, &recv_wr, &bad_wr)) {
+    if (ibv_post_recv(rdma->lc_remote.qp, &recv_wr, &bad_wr)) {
return -1;
}

@@ -1640,11 +1804,12 @@ static int qemu_rdma_exchange_get_response(RDMAContext *rdma,
RDMAControlHeader *head, int expecting, int idx)
{
uint32_t byte_len;
-    int ret = qemu_rdma_block_for_wrid(rdma, RDMA_WRID_RECV_CONTROL + idx,
+    int ret = qemu_rdma_block_for_wrid(rdma, &rdma->lc_remote,
+                                       RDMA_WRID_RECV_CONTROL + idx,
&byte_len);

if (ret < 0) {
-        fprintf(stderr, "rdma migration: recv polling control error!n");
+        ERROR(NULL, "recv polling control!");
return ret;
}

@@ -1731,8 +1896,7 @@ static int qemu_rdma_exchange_send(RDMAContext *rdma, RDMAControlHeader *head,
if (resp) {
ret = qemu_rdma_post_recv_control(rdma, RDMA_WRID_DATA);
if (ret) {
-            fprintf(stderr, "rdma migration: error posting"
-                    " extra control recv for anticipated result!");
+            ERROR(NULL, "posting extra control recv for anticipated result!");
return ret;
}
}
@@ -1742,7 +1906,7 @@ static int qemu_rdma_exchange_send(RDMAContext *rdma, RDMAControlHeader *head,
*/
ret = qemu_rdma_post_recv_control(rdma, RDMA_WRID_READY);
if (ret) {
-        fprintf(stderr, "rdma migration: error posting first control recv!");
+        ERROR(NULL, "posting first control recv!");
return ret;
}

@@ -1752,7 +1916,7 @@ static int qemu_rdma_exchange_send(RDMAContext *rdma, RDMAControlHeader *head,
ret = qemu_rdma_post_send_control(rdma, data, head);

if (ret < 0) {
-        fprintf(stderr, "Failed to send control buffer!n");
+        ERROR(NULL, "sending control buffer!");
return ret;
}

@@ -1829,30 +1993,51 @@ static int qemu_rdma_exchange_recv(RDMAContext *rdma, RDMAControlHeader *head,
*/
ret = qemu_rdma_post_recv_control(rdma, RDMA_WRID_READY);
if (ret) {
-        fprintf(stderr, "rdma migration: error posting second control recv!");
+        ERROR(NULL, "posting second control recv!");
return ret;
}

return 0;
}

+static inline void install_boundaries(RDMAContext *rdma, RDMACurrentChunk *cc)
+{
+    uint64_t len = cc->block->is_ram_block ?
+                   cc->current_length : cc->block->length;
+
+    cc->chunks = len / (1UL << RDMA_REG_CHUNK_SHIFT);
+
+    if (cc->chunks && ((len % (1UL << RDMA_REG_CHUNK_SHIFT)) == 0)) {
+        cc->chunks--;
+    }
+
+    cc->addr = (uint8_t *) (uint64_t)(cc->block->local_host_addr +
+                                 (cc->current_addr - cc->block->offset));
+
+    cc->chunk_idx = ram_chunk_index(cc->block->local_host_addr, cc->addr);
+    cc->chunk_start = ram_chunk_start(cc->block, cc->chunk_idx);
+    cc->chunk_end = ram_chunk_end(cc->block, cc->chunk_idx + cc->chunks);
+
+    DDPRINTF("Block %d chunk %" PRIu64 " has %" PRIu64
+             " chunks, (%" PRIu64 " MB)n", cc->block->index, cc->chunk_idx,
+                cc->chunks + 1, (cc->chunks + 1) *
+                    (1UL << RDMA_REG_CHUNK_SHIFT) / 1024 / 1024);
+
+}
+
/*
- * Write an actual chunk of memory using RDMA.
- *
- * If we're using dynamic registration on the dest-side, we have to
- * send a registration command first.
+ * Push out any unwritten RDMA operations.
*/
-static int qemu_rdma_write_one(QEMUFile *f, RDMAContext *rdma,
-                               int current_index, uint64_t current_addr,
-                               uint64_t length)
+static int qemu_rdma_write(QEMUFile *f, RDMAContext *rdma,
+                                 RDMACurrentChunk *src,
+                                 RDMACurrentChunk *dest)
{
struct ibv_sge sge;
struct ibv_send_wr send_wr = { 0 };
struct ibv_send_wr *bad_wr;
int reg_result_idx, ret, count = 0;
-    uint64_t chunk, chunks;
-    uint8_t *chunk_start, *chunk_end;
-    RDMALocalBlock *block = &(rdma->local_ram_blocks.block[current_index]);
+    bool copy;
+    RDMALocalContext *lc;
RDMARegister reg;
RDMARegisterResult *reg_result;
RDMAControlHeader resp = { .type = RDMA_CONTROL_REGISTER_RESULT };
@@ -1861,32 +2046,27 @@ static int qemu_rdma_write_one(QEMUFile *f, RDMAContext *rdma,
.repeat = 1,
};

-retry:
-    sge.addr = (uint64_t)(block->local_host_addr +
-                            (current_addr - block->offset));
-    sge.length = length;
-
-    chunk = ram_chunk_index(block->local_host_addr, (uint8_t *) sge.addr);
-    chunk_start = ram_chunk_start(block, chunk);
-
-    if (block->is_ram_block) {
-        chunks = length / (1UL << RDMA_REG_CHUNK_SHIFT);
-
-        if (chunks && ((length % (1UL << RDMA_REG_CHUNK_SHIFT)) == 0)) {
-            chunks--;
+    if (!src->current_length) {
+        return 0;
}
-    } else {
-        chunks = block->length / (1UL << RDMA_REG_CHUNK_SHIFT);

-        if (chunks && ((block->length % (1UL << RDMA_REG_CHUNK_SHIFT)) == 0)) {
-            chunks--;
-        }
+    if (dest == src) {
+        dest = NULL;
}

-    DDPRINTF("Writing %" PRIu64 " chunks, (%" PRIu64 " MB)n",
-        chunks + 1, (chunks + 1) * (1UL << RDMA_REG_CHUNK_SHIFT) / 1024 / 1024);
+    copy = dest ? true : false;
+
+    lc = (migrate_use_mc_rdma_copy() && copy) ?
+        (rdma->source ? &rdma->lc_src : &rdma->lc_dest) : &rdma->lc_remote;
+
+retry:
+    src->block = &(rdma->local_ram_blocks.block[src->current_block_idx]);
+    install_boundaries(rdma, src);

-    chunk_end = ram_chunk_end(block, chunk + chunks);
+    if (dest) {
+        dest->block = &(rdma->local_ram_blocks.block[dest->current_block_idx]);
+        install_boundaries(rdma, dest);
+    }

if (!rdma->pin_all) {
#ifdef RDMA_UNREGISTRATION_EXAMPLE
@@ -1894,49 +2074,56 @@ retry:
#endif
}

-    while (test_bit(chunk, block->transit_bitmap)) {
+    while (test_bit(src->chunk_idx, src->block->transit_bitmap)) {
(void)count;
-        DDPRINTF("(%d) Not clobbering: block: %d chunk %" PRIu64
-                " current %" PRIu64 " len %" PRIu64 " %d %dn",
-                count++, current_index, chunk,
-                sge.addr, length, rdma->nb_sent, block->nb_chunks);
+        DPRINTF("(%d) Not clobbering: block: %d chunk %" PRIu64
+                " current_addr %" PRIu64 " len %" PRIu64
+                " left %d (per qp left %d) chunks %d (%s)n",
+                count++, src->current_block_idx, src->chunk_idx,
+                (uint64_t) src->addr, src->current_length,
+                rdma->nb_sent, lc->nb_sent, src->block->nb_chunks, lc->id_str);

-        ret = qemu_rdma_block_for_wrid(rdma, RDMA_WRID_RDMA_WRITE, NULL);
+        ret = qemu_rdma_block_for_wrid(rdma, lc,
+                                       RDMA_WRID_RDMA_WRITE_REMOTE, NULL);

if (ret < 0) {
fprintf(stderr, "Failed to Wait for previous write to complete "
"block %d chunk %" PRIu64
-                    " current %" PRIu64 " len %" PRIu64 " %dn",
-                    current_index, chunk, sge.addr, length, rdma->nb_sent);
+                    " current_addr %" PRIu64 " len %" PRIu64
+                    " left %d (per qp left %d) (%s)n",
+                    src->current_block_idx, src->chunk_idx, (uint64_t) src->addr,
+                    src->current_length, rdma->nb_sent, lc->nb_sent, lc->id_str);
return ret;
}
}

-    if (!rdma->pin_all || !block->is_ram_block) {
-        if (!block->remote_keys[chunk]) {
+    if (!rdma->pin_all || !src->block->is_ram_block) {
+        if (!src->block->remote_keys[src->chunk_idx]) {
/*
* This chunk has not yet been registered, so first check to see
* if the entire chunk is zero. If so, tell the other size to
* memset() + madvise() the entire chunk without RDMA.
*/

-            if (can_use_buffer_find_nonzero_offset((void *)sge.addr, length)
-                   && buffer_find_nonzero_offset((void *)sge.addr,
-                                                    length) == length) {
+            if (src->block->is_ram_block &&
+                   can_use_buffer_find_nonzero_offset(src->addr, src->current_length)
+                   && buffer_find_nonzero_offset(src->addr,
+                                                    src->current_length) == src->current_length) {
RDMACompress comp = {
-                                        .offset = current_addr,
+                                        .offset = src->current_addr,
.value = 0,
-                                        .block_idx = current_index,
-                                        .length = length,
+                                        .block_idx = src->current_block_idx,
+                                        .length = src->current_length,
};

head.len = sizeof(comp);
head.type = RDMA_CONTROL_COMPRESS;

-                DDPRINTF("Entire chunk is zero, sending compress: %"
-                    PRIu64 " for %d "
-                    "bytes, index: %d, offset: %" PRId64 "...n",
-                    chunk, sge.length, current_index, current_addr);
+                DDPRINTF("Entire chunk is zero, sending compress: %" PRIu64
+                         " for %" PRIu64 " bytes, index: %d"
+                         ", offset: %" PRId64 " (%s)...n",
+                         src->chunk_idx, src->current_length,
+                         src->current_block_idx, src->current_addr, lc->id_str);

compress_to_network(&comp);
ret = qemu_rdma_exchange_send(rdma, &head,
@@ -1946,25 +2133,28 @@ retry:
return -EIO;
}

-                acct_update_position(f, sge.length, true);
+                acct_update_position(f, src->current_length, true);

return 1;
}

/*
-             * Otherwise, tell other side to register.
+             * Otherwise, tell other side to register. (Only for remote RDMA)
*/
-            reg.current_index = current_index;
-            if (block->is_ram_block) {
-                reg.key.current_addr = current_addr;
+            if (!dest) {
+                reg.current_block_idx = src->current_block_idx;
+                if (src->block->is_ram_block) {
+                    reg.key.current_addr = src->current_addr;
} else {
-                reg.key.chunk = chunk;
+                    reg.key.chunk = src->chunk_idx;
}
-            reg.chunks = chunks;
+                reg.chunks = src->chunks;

-            DDPRINTF("Sending registration request chunk %" PRIu64 " for %d "
-                    "bytes, index: %d, offset: %" PRId64 "...n",
-                    chunk, sge.length, current_index, current_addr);
+                DDPRINTF("Sending registration request chunk %" PRIu64
+                         " for %" PRIu64 " bytes, index: %d, offset: %"
+                         PRId64 " (%s)...n",
+                         src->chunk_idx, src->current_length,
+                         src->current_block_idx, src->current_addr, lc->id_str);

register_to_network(&reg);
ret = qemu_rdma_exchange_send(rdma, &head, (uint8_t *) &reg,
@@ -1972,83 +2162,96 @@ retry:
if (ret < 0) {
return ret;
}
+            }

/* try to overlap this single registration with the one we sent. */
-            if (qemu_rdma_register_and_get_keys(rdma, block,
-                                                (uint8_t *) sge.addr,
-                                                &sge.lkey, NULL, chunk,
-                                                chunk_start, chunk_end)) {
+            if (qemu_rdma_register_and_get_keys(rdma, src, lc, copy,
+                                                &sge.lkey, NULL)) {
fprintf(stderr, "cannot get lkey!n");
return -EINVAL;
}

+            if (!dest) {
reg_result = (RDMARegisterResult *)
rdma->wr_data[reg_result_idx].control_curr;

network_to_result(reg_result);

DDPRINTF("Received registration result:"
-                    " my key: %x their key %x, chunk %" PRIu64 "n",
-                    block->remote_keys[chunk], reg_result->rkey, chunk);
+                        " my key: %x their key %x, chunk %" PRIu64 " (%s)n",
+                        src->block->remote_keys[src->chunk_idx],
+                        reg_result->rkey, src->chunk_idx, lc->id_str);

-            block->remote_keys[chunk] = reg_result->rkey;
-            block->remote_host_addr = reg_result->host_addr;
+                src->block->remote_keys[src->chunk_idx] = reg_result->rkey;
+                src->block->remote_host_addr = reg_result->host_addr;
+            }
} else {
/* already registered before */
-            if (qemu_rdma_register_and_get_keys(rdma, block,
-                                                (uint8_t *)sge.addr,
-                                                &sge.lkey, NULL, chunk,
-                                                chunk_start, chunk_end)) {
+            if (qemu_rdma_register_and_get_keys(rdma, src, lc, copy,
+                                                &sge.lkey, NULL)) {
fprintf(stderr, "cannot get lkey!n");
return -EINVAL;
}
}

-        send_wr.wr.rdma.rkey = block->remote_keys[chunk];
+        send_wr.wr.rdma.rkey = src->block->remote_keys[src->chunk_idx];
} else {
-        send_wr.wr.rdma.rkey = block->remote_rkey;
+        send_wr.wr.rdma.rkey = src->block->remote_rkey;

-        if (qemu_rdma_register_and_get_keys(rdma, block, (uint8_t *)sge.addr,
-                                                     &sge.lkey, NULL, chunk,
-                                                     chunk_start, chunk_end)) {
+        if (qemu_rdma_register_and_get_keys(rdma, src, lc, copy,
+                                            &sge.lkey, NULL)) {
fprintf(stderr, "cannot get lkey!n");
return -EINVAL;
}
}

+    if (migrate_use_mc_rdma_copy() && dest) {
+        if (qemu_rdma_register_and_get_keys(rdma, dest,
+                                            &rdma->lc_dest, copy,
+                                            NULL, &send_wr.wr.rdma.rkey)) {
+            fprintf(stderr, "cannot get rkey!n");
+            return -EINVAL;
+        }
+    }
+
/*
* Encode the ram block index and chunk within this wrid.
* We will use this information at the time of completion
* to figure out which bitmap to check against and then which
* chunk in the bitmap to look for.
*/
-    send_wr.wr_id = qemu_rdma_make_wrid(RDMA_WRID_RDMA_WRITE,
-                                        current_index, chunk);
+    send_wr.wr_id = qemu_rdma_make_wrid(RDMA_WRID_RDMA_WRITE_REMOTE,
+                                        src->current_block_idx, src->chunk_idx);

+    sge.length = src->current_length;
+    sge.addr = (uint64_t) src->addr;
send_wr.opcode = IBV_WR_RDMA_WRITE;
send_wr.send_flags = IBV_SEND_SIGNALED;
send_wr.sg_list = &sge;
send_wr.num_sge = 1;
-    send_wr.wr.rdma.remote_addr = block->remote_host_addr +
-                                (current_addr - block->offset);
+    send_wr.wr.rdma.remote_addr = (dest ? (uint64_t) dest->addr :
+                (src->block->remote_host_addr +
+                    (src->current_addr - src->block->offset)));

-    DDDPRINTF("Posting chunk: %" PRIu64 ", addr: %lx"
-              " remote: %lx, bytes %" PRIu32 "n",
-              chunk, sge.addr, send_wr.wr.rdma.remote_addr,
-              sge.length);
+    DDPRINTF("Posting chunk: %" PRIu64 ", addr: %lx"
+             " remote: %lx, bytes %" PRIu32 " lkey %" PRIu32
+             " rkey %" PRIu32 " (%s)n",
+             src->chunk_idx, sge.addr,
+             send_wr.wr.rdma.remote_addr, sge.length,
+             sge.lkey, send_wr.wr.rdma.rkey, lc->id_str);

/*
* ibv_post_send() does not return negative error numbers,
* per the specification they are positive - no idea why.
*/
-    ret = ibv_post_send(rdma->qp, &send_wr, &bad_wr);
+    ret = ibv_post_send(lc->qp, &send_wr, &bad_wr);

if (ret == ENOMEM) {
DDPRINTF("send queue is full. wait a little....n");
-        ret = qemu_rdma_block_for_wrid(rdma, RDMA_WRID_RDMA_WRITE, NULL);
+        ret = qemu_rdma_block_for_wrid(rdma, lc,
+                                       RDMA_WRID_RDMA_WRITE_REMOTE, NULL);
if (ret < 0) {
-            fprintf(stderr, "rdma migration: failed to make "
-                            "room in full send queue! %dn", ret);
+            ERROR(NULL, "could not make room in full send queue! %d", ret);
return ret;
}

@@ -2059,80 +2262,67 @@ retry:
return -ret;
}

-    set_bit(chunk, block->transit_bitmap);
-    acct_update_position(f, sge.length, false);
-    rdma->total_writes++;
+    set_bit(src->chunk_idx, src->block->transit_bitmap);

-    return 0;
+    if (!dest) {
+        acct_update_position(f, sge.length, false);
}

-/*
- * Push out any unwritten RDMA operations.
- *
- * We support sending out multiple chunks at the same time.
- * Not all of them need to get signaled in the completion queue.
- */
-static int qemu_rdma_write_flush(QEMUFile *f, RDMAContext *rdma)
-{
-    int ret;
+    rdma->total_writes++;
+    rdma->nb_sent++;
+    lc->nb_sent++;

-    if (!rdma->current_length) {
-        return 0;
-    }
+    DDDPRINTF("sent total: %d sent lc: %d (%s)n",
+                rdma->nb_sent, lc->nb_sent, lc->id_str);

-    ret = qemu_rdma_write_one(f, rdma,
-            rdma->current_index, rdma->current_addr, rdma->current_length);
+    src->current_length = 0;
+    src->current_addr = 0;

-    if (ret < 0) {
-        return ret;
+    if (dest) {
+        dest->current_length = 0;
+        dest->current_addr = 0;
}

-    if (ret == 0) {
-        rdma->nb_sent++;
-        DDDPRINTF("sent total: %dn", rdma->nb_sent);
-    }
-
-    rdma->current_length = 0;
-    rdma->current_addr = 0;
-
return 0;
}

static inline int qemu_rdma_buffer_mergable(RDMAContext *rdma,
-                    uint64_t offset, uint64_t len)
+                                            RDMACurrentChunk *cc,
+                                            uint64_t current_addr,
+                                            uint64_t len)
{
RDMALocalBlock *block;
uint8_t *host_addr;
uint8_t *chunk_end;

-    if (rdma->current_index < 0) {
+    if (cc->current_block_idx < 0) {
return 0;
}

-    if (rdma->current_chunk < 0) {
+    if (cc->current_chunk < 0) {
return 0;
}

-    block = &(rdma->local_ram_blocks.block[rdma->current_index]);
-    host_addr = block->local_host_addr + (offset - block->offset);
-    chunk_end = ram_chunk_end(block, rdma->current_chunk);
+    block = &(rdma->local_ram_blocks.block[cc->current_block_idx]);
+    host_addr = block->local_host_addr + (current_addr - block->offset);
+    chunk_end = ram_chunk_end(block, cc->current_chunk);

-    if (rdma->current_length == 0) {
+    if (cc->current_length == 0) {
return 0;
}

/*
* Only merge into chunk sequentially.
*/
-    if (offset != (rdma->current_addr + rdma->current_length)) {
+    if (current_addr != (cc->current_addr + cc->current_length)) {
return 0;
}

-    if (offset < block->offset) {
+    if (current_addr < block->offset) {
return 0;
}

-    if ((offset + len) > (block->offset + block->length)) {
+    if ((current_addr + len) > (block->offset + block->length)) {
return 0;
}

@@ -2143,80 +2333,148 @@ static inline int qemu_rdma_buffer_mergable(RDMAContext *rdma,
return 1;
}

+static int write_start(RDMAContext *rdma,
+                        RDMACurrentChunk *cc,
+                        uint64_t len,
+                        uint64_t current_addr)
+{
+    int ret;
+    uint64_t block_idx, chunk;
+
+    cc->current_addr = current_addr;
+    block_idx = cc->current_block_idx;
+    chunk = cc->current_chunk;
+
+    ret = qemu_rdma_search_ram_block(rdma, cc->block_offset,
+                                     cc->offset, len, &block_idx, &chunk);
+    if (ret) {
+        ERROR(NULL, "ram block search failed");
+        return ret;
+    }
+
+    cc->current_block_idx = block_idx;
+    cc->current_chunk = chunk;
+
+    return 0;
+}
+
/*
- * We're not actually writing here, but doing three things:
- *
- * 1. Identify the chunk the buffer belongs to.
- * 2. If the chunk is full or the buffer doesn't belong to the current
- *    chunk, then start a new chunk and flush() the old chunk.
- * 3. To keep the hardware busy, we also group chunks into batches
- *    and only require that a batch gets acknowledged in the completion
- *    qeueue instead of each individual chunk.
+ * If we cannot merge it, we flush the current buffer first.
*/
-static int qemu_rdma_write(QEMUFile *f, RDMAContext *rdma,
-                           uint64_t block_offset, uint64_t offset,
-                           uint64_t len)
+static int qemu_rdma_flush_unmergable(RDMAContext *rdma,
+                                      RDMACurrentChunk *src,
+                                      RDMACurrentChunk *dest,
+                                      QEMUFile *f, uint64_t len)
{
-    uint64_t current_addr = block_offset + offset;
-    uint64_t index = rdma->current_index;
-    uint64_t chunk = rdma->current_chunk;
+    uint64_t current_addr_src;
+    uint64_t current_addr_dest;
int ret;

-    /* If we cannot merge it, we flush the current buffer first. */
-    if (!qemu_rdma_buffer_mergable(rdma, current_addr, len)) {
-        ret = qemu_rdma_write_flush(f, rdma);
+    current_addr_src = src->block_offset + src->offset;
+
+    if (dest) {
+        current_addr_dest = dest->block_offset + dest->offset;
+    }
+
+    if (qemu_rdma_buffer_mergable(rdma, src, current_addr_src, len)) {
+        if (dest) {
+            if (qemu_rdma_buffer_mergable(rdma, dest, current_addr_dest, len)) {
+                goto merge;
+            }
+        } else {
+            goto merge;
+        }
+    }
+
+    ret = qemu_rdma_write(f, rdma, src, dest);
+
if (ret) {
return ret;
}
-        rdma->current_length = 0;
-        rdma->current_addr = current_addr;

-        ret = qemu_rdma_search_ram_block(rdma, block_offset,
-                                         offset, len, &index, &chunk);
+    ret = write_start(rdma, src, len, current_addr_src);
+
if (ret) {
-            fprintf(stderr, "ram block search failedn");
return ret;
}
-        rdma->current_index = index;
-        rdma->current_chunk = chunk;
-    }

-    /* merge it */
-    rdma->current_length += len;
+    if (dest) {
+        ret = write_start(rdma, dest, len, current_addr_dest);
+
+        if (ret) {
+            return ret;
+        }
+    }

-    /* flush it if buffer is too large */
-    if (rdma->current_length >= RDMA_MERGE_MAX) {
-        return qemu_rdma_write_flush(f, rdma);
+merge:
+    src->current_length += len;
+    if (dest) {
+        dest->current_length += len;
}

return 0;
}

-static void qemu_rdma_cleanup(RDMAContext *rdma)
+static void disconnect_ibv(RDMAContext *rdma, RDMALocalContext *lc, bool force)
{
struct rdma_cm_event *cm_event;
-    int ret, idx;
+    int ret;

-    if (rdma->cm_id && rdma->connected) {
-        if (rdma->error_state) {
+    if (!lc->cm_id || !lc->connected) {
+        return;
+    }
+
+    if ((lc == (&rdma->lc_remote)) && rdma->error_state) {
+        if (rdma->error_state != -ENETUNREACH) {
RDMAControlHeader head = { .len = 0,
.type = RDMA_CONTROL_ERROR,
.repeat = 1,
};
fprintf(stderr, "Early error. Sending error.n");
qemu_rdma_post_send_control(rdma, NULL, &head);
+        } else {
+            fprintf(stderr, "Early error.n");
+            rdma_disconnect(lc->cm_id);
+            goto finish;
+        }
}

-        ret = rdma_disconnect(rdma->cm_id);
-        if (!ret) {
+    ret = rdma_disconnect(lc->cm_id);
+    if (!ret && !force) {
DDPRINTF("waiting for disconnectn");
-            ret = rdma_get_cm_event(rdma->channel, &cm_event);
+        ret = rdma_get_cm_event(lc->channel, &cm_event);
if (!ret) {
rdma_ack_cm_event(cm_event);
}
}
+
+finish:
+
DDPRINTF("Disconnected.n");
-        rdma->connected = false;
+    lc->verbs = NULL;
+    lc->connected = false;
+}
+
+static void qemu_rdma_cleanup(RDMAContext *rdma, bool force)
+{
+    int idx;
+
+    if (connection_timer) {
+        timer_del(connection_timer);
+        timer_free(connection_timer);
+        connection_timer = NULL;
+    }
+
+    if (keepalive_timer) {
+        timer_del(keepalive_timer);
+        timer_free(keepalive_timer);
+        keepalive_timer = NULL;
+    }
+
+    disconnect_ibv(rdma, &rdma->lc_remote, force);
+    if (migrate_use_mc_rdma_copy()) {
+        disconnect_ibv(rdma, &rdma->lc_src, force);
+        disconnect_ibv(rdma, &rdma->lc_dest, force);
}

g_free(rdma->block);
@@ -2237,40 +2495,192 @@ static void qemu_rdma_cleanup(RDMAContext *rdma)
}
}

-    if (rdma->qp) {
-        rdma_destroy_qp(rdma->cm_id);
-        rdma->qp = NULL;
+    close_ibv(rdma, &rdma->lc_remote);
+    if (migrate_use_mc_rdma_copy()) {
+        close_ibv(rdma, &rdma->lc_src);
+        close_ibv(rdma, &rdma->lc_dest);
}
-    if (rdma->cq) {
-        ibv_destroy_cq(rdma->cq);
-        rdma->cq = NULL;
+
+    if (rdma->keepalive_mr) {
+        ibv_dereg_mr(rdma->keepalive_mr);
+        rdma->keepalive_mr = NULL;
}
-    if (rdma->comp_channel) {
-        ibv_destroy_comp_channel(rdma->comp_channel);
-        rdma->comp_channel = NULL;
+    if (rdma->next_keepalive_mr) {
+        ibv_dereg_mr(rdma->next_keepalive_mr);
+        rdma->next_keepalive_mr = NULL;
}
-    if (rdma->pd) {
-        ibv_dealloc_pd(rdma->pd);
-        rdma->pd = NULL;
}
-    if (rdma->listen_id) {
-        rdma_destroy_id(rdma->listen_id);
-        rdma->listen_id = NULL;
+
+static int qemu_rdma_device_init(RDMAContext *rdma, Error **errp,
+                                 RDMALocalContext *lc)
+{
+    struct rdma_cm_event *cm_event;
+    int ret = -EINVAL;
+    char ip[40] = "unknown";
+    struct rdma_addrinfo *res;
+    char port_str[16];
+
+    if (lc->host == NULL) {
+        ERROR(errp, "RDMA host is not set!");
+        SET_ERROR(rdma, -EINVAL);
+        return -1;
+    }
+
+    /* create CM channel */
+    lc->channel = rdma_create_event_channel();
+    if (!lc->channel) {
+        ERROR(errp, "could not create rdma event channel (%s)", lc->id_str);
+        SET_ERROR(rdma, -EINVAL);
+        return -1;
+    }
+
+    /* create CM id */
+    if (lc->listen_id) {
+        lc->cm_id = lc->listen_id;
+    } else {
+        ret = rdma_create_id(lc->channel, &lc->cm_id, NULL, RDMA_PS_TCP);
+        if (ret) {
+            ERROR(errp, "could not create cm_id! (%s)", lc->id_str);
+            goto err_device_init_create_id;
+        }
+    }
+
+    snprintf(port_str, 16, "%d", lc->port);
+    port_str[15] = '0';
+
+    if (lc->host && strcmp("", lc->host)) {
+        struct rdma_addrinfo *e;
+
+        ret = rdma_getaddrinfo(lc->host, port_str, NULL, &res);
+        if (ret < 0) {
+            ERROR(errp, "could not rdma_getaddrinfo address %s (%s)",
+                        lc->host, lc->id_str);
+            goto err_device_init_bind_addr;
+        }
+
+        for (e = res; e != NULL; e = e->ai_next) {
+            inet_ntop(e->ai_family,
+                &((struct sockaddr_in *) e->ai_dst_addr)->sin_addr, ip, sizeof ip);
+            DPRINTF("Trying %s => %s (port %s) (%s)n", lc->host,
+                                                        ip, port_str,
+                                                        lc->id_str);
+
+            if (lc->dest) {
+                ret = rdma_bind_addr(lc->cm_id, e->ai_dst_addr);
+            } else {
+                ret = rdma_resolve_addr(lc->cm_id, NULL, e->ai_dst_addr,
+                    RDMA_RESOLVE_TIMEOUT_MS);
+            }
+            if (!ret) {
+                if (e->ai_family == AF_INET6) {
+                    ret = qemu_rdma_broken_ipv6_kernel(errp, lc->cm_id->verbs);
+                    if (ret) {
+                        continue;
}
-    if (rdma->cm_id) {
-        rdma_destroy_id(rdma->cm_id);
-        rdma->cm_id = NULL;
}
-    if (rdma->channel) {
-        rdma_destroy_event_channel(rdma->channel);
-        rdma->channel = NULL;
+
+                goto next;
+            }
+        }
+
+        ERROR(errp, "initialize/bind/resolve device! (%s)", lc->id_str);
+        goto err_device_init_bind_addr;
+    } else {
+        ERROR(errp, "migration host and port not specified! (%s)", lc->id_str);
+        ret = -EINVAL;
+        goto err_device_init_bind_addr;
}
-    g_free(rdma->host);
-    rdma->host = NULL;
+next:
+    qemu_rdma_dump_gid("device_init", lc->cm_id);
+
+    if(lc->source) {
+        ret = rdma_get_cm_event(lc->channel, &cm_event);
+        if (ret) {
+            ERROR(errp, "could not perform event_addr_resolved (%s)", lc->id_str);
+            goto err_device_init_bind_addr;
}

+        if (cm_event->event != RDMA_CM_EVENT_ADDR_RESOLVED) {
+            ERROR(errp, "result not equal to event_addr_resolved %s (%s)",
+                    rdma_event_str(cm_event->event), lc->id_str);
+            perror("rdma_resolve_addr");
+            ret = -EINVAL;
+            goto err_device_init_bind_addr;
+        }

-static int qemu_rdma_source_init(RDMAContext *rdma, Error **errp, bool pin_all)
+        rdma_ack_cm_event(cm_event);
+
+        /* resolve route */
+        ret = rdma_resolve_route(lc->cm_id, RDMA_RESOLVE_TIMEOUT_MS);
+        if (ret) {
+            ERROR(errp, "could not resolve rdma route");
+            goto err_device_init_bind_addr;
+        }
+
+        ret = rdma_get_cm_event(lc->channel, &cm_event);
+        if (ret) {
+            ERROR(errp, "could not perform event_route_resolved");
+            goto err_device_init_bind_addr;
+        }
+
+        if (cm_event->event != RDMA_CM_EVENT_ROUTE_RESOLVED) {
+            ERROR(errp, "result not equal to event_route_resolved: %s",
+                            rdma_event_str(cm_event->event));
+            rdma_ack_cm_event(cm_event);
+            ret = -EINVAL;
+            goto err_device_init_bind_addr;
+        }
+
+        lc->verbs = lc->cm_id->verbs;
+        printf("verbs: %p (%s)n", lc->verbs, lc->id_str);
+
+        rdma_ack_cm_event(cm_event);
+
+        ret = qemu_rdma_alloc_pd_cq_qp(rdma, lc);
+        if (ret) {
+            goto err_device_init_bind_addr;
+        }
+
+        qemu_rdma_dump_id("rdma_accept_start", lc->verbs);
+    } else {
+        lc->listen_id = lc->cm_id;
+        lc->cm_id = NULL;
+
+        ret = rdma_listen(lc->listen_id, 1);
+
+        if (ret) {
+            perror("rdma_listen");
+            ERROR(errp, "listening on socket! (%s)", lc->id_str);
+            goto err_device_init_bind_addr;
+        }
+
+        DPRINTF("rdma_listen successn");
+    }
+
+    DPRINTF("qemu_rdma_device_init successn");
+    return 0;
+
+err_device_init_bind_addr:
+    if (lc->cm_id) {
+        rdma_destroy_id(lc->cm_id);
+        lc->cm_id = NULL;
+    }
+    if (lc->listen_id) {
+        rdma_destroy_id(lc->listen_id);
+        lc->listen_id = NULL;
+    }
+err_device_init_create_id:
+    if (lc->channel) {
+        rdma_destroy_event_channel(lc->channel);
+        lc->channel = NULL;
+    }
+    SET_ERROR(rdma, ret);
+    return ret;
+}
+
+static int qemu_rdma_init_outgoing(RDMAContext *rdma,
+                                 Error **errp,
+                                 MigrationState *s)
{
int ret, idx;
Error *local_err = NULL, **temp = &local_err;
@@ -2279,62 +2689,116 @@ static int qemu_rdma_source_init(RDMAContext *rdma, Error **errp, bool pin_all)
* Will be validated against destination's actual capabilities
* after the connect() completes.
*/
-    rdma->pin_all = pin_all;
+    rdma->pin_all = s->enabled_capabilities[MIGRATION_CAPABILITY_RDMA_PIN_ALL];
+    rdma->do_keepalive = s->enabled_capabilities[MIGRATION_CAPABILITY_RDMA_KEEPALIVE];

-    ret = qemu_rdma_resolve_host(rdma, temp);
-    if (ret) {
-        goto err_rdma_source_init;
+    for (idx = 0; idx < RDMA_WRID_MAX; idx++) {
+        rdma->wr_data[idx].control_len = 0;
+        rdma->wr_data[idx].control_curr = NULL;
}

-    ret = qemu_rdma_alloc_pd_cq(rdma);
+    rdma->source = true;
+    rdma->dest = false;
+    rdma->lc_remote.source = true;
+    rdma->lc_remote.dest = false;
+
+    ret = qemu_rdma_device_init(rdma, temp, &rdma->lc_remote);
if (ret) {
-        ERROR(temp, "rdma migration: error allocating pd and cq! Your mlock()"
-                    " limits may be too low. Please check $ ulimit -a # and "
-                    "search for 'ulimit -l' in the output");
-        goto err_rdma_source_init;
+        goto err_rdma_init_outgoing;
}

-    ret = qemu_rdma_alloc_qp(rdma);
+    ret = qemu_rdma_reg_keepalive(rdma);
+
if (ret) {
-        ERROR(temp, "rdma migration: error allocating qp!");
-        goto err_rdma_source_init;
+        ERROR(temp, "allocating keepalive structures");
+        goto err_rdma_init_outgoing;
}

ret = qemu_rdma_init_ram_blocks(rdma);
if (ret) {
-        ERROR(temp, "rdma migration: error initializing ram blocks!");
-        goto err_rdma_source_init;
+        ERROR(temp, "initializing ram blocks!");
+        goto err_rdma_init_outgoing;
}

for (idx = 0; idx < RDMA_WRID_MAX; idx++) {
ret = qemu_rdma_reg_control(rdma, idx);
if (ret) {
-            ERROR(temp, "rdma migration: error registering %d control!",
-                                                            idx);
-            goto err_rdma_source_init;
+            ERROR(temp, "registering %d control!", idx);
+            goto err_rdma_init_outgoing;
}
}

return 0;

-err_rdma_source_init:
+err_rdma_init_outgoing:
error_propagate(errp, local_err);
-    qemu_rdma_cleanup(rdma);
+    qemu_rdma_cleanup(rdma, false);
return -1;
}

+static int qemu_rdma_connect_finish(RDMAContext *rdma,
+                                    RDMALocalContext *lc,
+                                    Error **errp,
+                                    struct rdma_cm_event **return_event)
+{
+    int ret = 0;
+    struct rdma_cm_event *cm_event;
+
+    ret = rdma_get_cm_event(lc->channel, &cm_event);
+    if (ret) {
+        perror("rdma_get_cm_event after rdma_connect");
+        rdma_ack_cm_event(cm_event);
+        goto err;
+    }
+
+    if (cm_event->event != RDMA_CM_EVENT_ESTABLISHED) {
+        perror("rdma_get_cm_event != EVENT_ESTABLISHED after rdma_connect");
+        rdma_ack_cm_event(cm_event);
+        ret = -1;
+        goto err;
+    }
+
+    /*
+     * The rdmacm "private data area" may contain information from the receiver,
+     * just as we may have done the same from the sender side. If so, we cannot
+     * ack this CM event until we have processed/copied this small data
+     * out of the cm_event structure, otherwise, the ACK will free the structure
+     * and we will lose the data.
+     *
+     * Thus, we allow the caller to ACK this event if there is important
+     * information inside. Otherwise, we will ACK by ourselves.
+     */
+    if (return_event) {
+        *return_event = cm_event;
+    } else {
+        rdma_ack_cm_event(cm_event);
+    }
+
+    lc->connected = true;
+
+    return 0;
+err:
+    ERROR(errp, "connecting to destination!");
+    rdma_destroy_id(lc->cm_id);
+    lc->cm_id = NULL;
+    return ret;
+}
+
static int qemu_rdma_connect(RDMAContext *rdma, Error **errp)
{
RDMACapabilities cap = {
.version = RDMA_CONTROL_VERSION_CURRENT,
.flags = 0,
+                                .keepalive_rkey = rdma->keepalive_mr->rkey,
+                                .keepalive_addr = (uint64_t) &rdma->keepalive,
};
struct rdma_conn_param conn_param = { .initiator_depth = 2,
.retry_count = 5,
.private_data = &cap,
.private_data_len = sizeof(cap),
};
-    struct rdma_cm_event *cm_event;
+
+    struct rdma_cm_event *cm_event = NULL;
int ret;

/*
@@ -2346,40 +2810,36 @@ static int qemu_rdma_connect(RDMAContext *rdma, Error **errp)
cap.flags |= RDMA_CAPABILITY_PIN_ALL;
}

+    if (rdma->do_keepalive) {
+        DPRINTF("Keepalives requested.n");
+        cap.flags |= RDMA_CAPABILITY_KEEPALIVE;
+    }
+
+    DDPRINTF("Sending keepalive params: key %x addr: %" PRIx64 "n",
+            cap.keepalive_rkey, cap.keepalive_addr);
caps_to_network(&cap);

-    ret = rdma_connect(rdma->cm_id, &conn_param);
+    ret = rdma_connect(rdma->lc_remote.cm_id, &conn_param);
if (ret) {
perror("rdma_connect");
-        ERROR(errp, "connecting to destination!");
-        rdma_destroy_id(rdma->cm_id);
-        rdma->cm_id = NULL;
goto err_rdma_source_connect;
}

-    ret = rdma_get_cm_event(rdma->channel, &cm_event);
-    if (ret) {
-        perror("rdma_get_cm_event after rdma_connect");
-        ERROR(errp, "connecting to destination!");
-        rdma_ack_cm_event(cm_event);
-        rdma_destroy_id(rdma->cm_id);
-        rdma->cm_id = NULL;
-        goto err_rdma_source_connect;
-    }
+    ret = qemu_rdma_connect_finish(rdma, &rdma->lc_remote, errp, &cm_event);

-    if (cm_event->event != RDMA_CM_EVENT_ESTABLISHED) {
-        perror("rdma_get_cm_event != EVENT_ESTABLISHED after rdma_connect");
-        ERROR(errp, "connecting to destination!");
-        rdma_ack_cm_event(cm_event);
-        rdma_destroy_id(rdma->cm_id);
-        rdma->cm_id = NULL;
+    if (ret) {
goto err_rdma_source_connect;
}
-    rdma->connected = true;

memcpy(&cap, cm_event->param.conn.private_data, sizeof(cap));
network_to_caps(&cap);

+    rdma->keepalive_rkey = cap.keepalive_rkey;
+    rdma->keepalive_addr = cap.keepalive_addr;
+
+    DDPRINTF("Received keepalive params: key %x addr: %" PRIx64 "n",
+            cap.keepalive_rkey, cap.keepalive_addr);
+
/*
* Verify that the *requested* capabilities are supported by the destination
* and disable them otherwise.
@@ -2390,7 +2850,14 @@ static int qemu_rdma_connect(RDMAContext *rdma, Error **errp)
rdma->pin_all = false;
}

+    if (rdma->do_keepalive && !(cap.flags & RDMA_CAPABILITY_KEEPALIVE)) {
+        ERROR(errp, "Server cannot support keepalives. "
+                        "Will not check for them.");
+        rdma->do_keepalive = false;
+    }
+
DPRINTF("Pin all memory: %sn", rdma->pin_all ? "enabled" : "disabled");
+    DPRINTF("Keepalives: %sn", rdma->do_keepalive ? "enabled" : "disabled");

rdma_ack_cm_event(cm_event);

@@ -2405,93 +2872,115 @@ static int qemu_rdma_connect(RDMAContext *rdma, Error **errp)
return 0;

err_rdma_source_connect:
-    qemu_rdma_cleanup(rdma);
-    return -1;
+    SET_ERROR(rdma, ret);
+    qemu_rdma_cleanup(rdma, false);
+    return rdma->error_state;
}

-static int qemu_rdma_dest_init(RDMAContext *rdma, Error **errp)
+static void send_keepalive(void *opaque)
{
-    int ret = -EINVAL, idx;
-    struct rdma_cm_id *listen_id;
-    char ip[40] = "unknown";
-    struct rdma_addrinfo *res;
-    char port_str[16];
+    RDMAContext *rdma = opaque;
+    struct ibv_sge sge;
+    struct ibv_send_wr send_wr = { 0 };
+    struct ibv_send_wr *bad_wr;
+    int ret;

-    for (idx = 0; idx < RDMA_WRID_MAX; idx++) {
-        rdma->wr_data[idx].control_len = 0;
-        rdma->wr_data[idx].control_curr = NULL;
+    if (!rdma->migration_started) {
+        goto reset;
}

-    if (rdma->host == NULL) {
-        ERROR(errp, "RDMA host is not set!");
-        rdma->error_state = -EINVAL;
-        return -1;
-    }
-    /* create CM channel */
-    rdma->channel = rdma_create_event_channel();
-    if (!rdma->channel) {
-        ERROR(errp, "could not create rdma event channel");
-        rdma->error_state = -EINVAL;
-        return -1;
-    }
+    rdma->next_keepalive++;
+retry:

-    /* create CM id */
-    ret = rdma_create_id(rdma->channel, &listen_id, NULL, RDMA_PS_TCP);
-    if (ret) {
-        ERROR(errp, "could not create cm_id!");
-        goto err_dest_init_create_listen_id;
-    }
+    sge.addr = (uint64_t) &rdma->next_keepalive;
+    sge.length = sizeof(rdma->next_keepalive);
+    sge.lkey = rdma->next_keepalive_mr->lkey;
+    send_wr.wr_id = RDMA_WRID_RDMA_KEEPALIVE;
+    send_wr.opcode = IBV_WR_RDMA_WRITE;
+    send_wr.send_flags = 0;
+    send_wr.sg_list = &sge;
+    send_wr.num_sge = 1;
+    send_wr.wr.rdma.remote_addr = rdma->keepalive_addr;
+    send_wr.wr.rdma.rkey = rdma->keepalive_rkey;

-    snprintf(port_str, 16, "%d", rdma->port);
-    port_str[15] = '0';
+    DDPRINTF("Posting keepalive: addr: %lx"
+              " remote: %lx, bytes %" PRIu32 "n",
+              sge.addr, send_wr.wr.rdma.remote_addr, sge.length);

-    if (rdma->host && strcmp("", rdma->host)) {
-        struct rdma_addrinfo *e;
+    ret = ibv_post_send(rdma->lc_remote.qp, &send_wr, &bad_wr);

-        ret = rdma_getaddrinfo(rdma->host, port_str, NULL, &res);
-        if (ret < 0) {
-            ERROR(errp, "could not rdma_getaddrinfo address %s", rdma->host);
-            goto err_dest_init_bind_addr;
+    if (ret == ENOMEM) {
+        DPRINTF("send queue is full. wait a little....n");
+        g_usleep(RDMA_KEEPALIVE_INTERVAL_MS * 1000);
+        goto retry;
+    } else if (ret > 0) {
+        perror("rdma migration: post keepalive");
+        SET_ERROR(rdma, -ret);
+        return;
}

-        for (e = res; e != NULL; e = e->ai_next) {
-            inet_ntop(e->ai_family,
-                &((struct sockaddr_in *) e->ai_dst_addr)->sin_addr, ip, sizeof ip);
-            DPRINTF("Trying %s => %sn", rdma->host, ip);
-            ret = rdma_bind_addr(listen_id, e->ai_dst_addr);
-            if (!ret) {
-                if (e->ai_family == AF_INET6) {
-                    ret = qemu_rdma_broken_ipv6_kernel(errp, listen_id->verbs);
-                    if (ret) {
-                        continue;
+reset:
+    timer_mod(keepalive_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
+                    RDMA_KEEPALIVE_INTERVAL_MS);
}
+
+static void check_qp_state(void *opaque)
+{
+    RDMAContext *rdma = opaque;
+    int first_missed = 0;
+
+    if (!rdma->migration_started) {
+        goto reset;
}

-                goto listen;
+    if (rdma->last_keepalive == rdma->keepalive) {
+        rdma->nb_missed_keepalive++;
+        if (rdma->nb_missed_keepalive == 1) {
+            first_missed = RDMA_KEEPALIVE_FIRST_MISSED_OFFSET;
+            DDPRINTF("Setting first missed additional delayn");
+        } else {
+            DPRINTF("WARN: missed keepalive: %" PRIu64 "n",
+                        rdma->nb_missed_keepalive);
}
+    } else {
+        rdma->keepalive_startup = true;
+        rdma->nb_missed_keepalive = 0;
}

-        ERROR(errp, "Error: could not rdma_bind_addr!");
-        goto err_dest_init_bind_addr;
+    rdma->last_keepalive = rdma->keepalive;
+
+    if (rdma->keepalive_startup) {
+        if (rdma->nb_missed_keepalive > RDMA_MAX_LOST_KEEPALIVE) {
+            struct ibv_qp_attr attr = {.qp_state = IBV_QPS_ERR };
+            SET_ERROR(rdma, -ENETUNREACH);
+            ERROR(NULL, "peer keepalive failed.");
+
+            if (ibv_modify_qp(rdma->lc_remote.qp, &attr, IBV_QP_STATE)) {
+                ERROR(NULL, "modify QP to RTR");
+                return;
+            }
+            return;
+        }
+    } else if (rdma->nb_missed_keepalive < RDMA_MAX_STARTUP_MISSED_KEEPALIVE) {
+        DDPRINTF("Keepalive startup waiting: %" PRIu64 "n",
+                rdma->nb_missed_keepalive);
} else {
-        ERROR(errp, "migration host and port not specified!");
-        ret = -EINVAL;
-        goto err_dest_init_bind_addr;
+        DDPRINTF("Keepalive startup too long.n");
+        rdma->keepalive_startup = true;
}
-listen:

-    rdma->listen_id = listen_id;
-    qemu_rdma_dump_gid("dest_init", listen_id);
-    return 0;
-
-err_dest_init_bind_addr:
-    rdma_destroy_id(listen_id);
-err_dest_init_create_listen_id:
-    rdma_destroy_event_channel(rdma->channel);
-    rdma->channel = NULL;
-    rdma->error_state = ret;
-    return ret;
+reset:
+    timer_mod(connection_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
+                    RDMA_KEEPALIVE_INTERVAL_MS + first_missed);
+}

+static void qemu_rdma_keepalive_start(void)
+{
+    DPRINTF("Starting up keepalives....n");
+    timer_mod(connection_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
+                    RDMA_CONNECTION_INTERVAL_MS);
+    timer_mod(keepalive_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
+                    RDMA_KEEPALIVE_INTERVAL_MS);
}

static void *qemu_rdma_data_init(const char *host_port, Error **errp)
@@ -2502,20 +2991,33 @@ static void *qemu_rdma_data_init(const char *host_port, Error **errp)
if (host_port) {
rdma = g_malloc0(sizeof(RDMAContext));
memset(rdma, 0, sizeof(RDMAContext));
-        rdma->current_index = -1;
-        rdma->current_chunk = -1;
+        rdma->chunk_remote.current_block_idx = -1;
+        rdma->chunk_remote.current_chunk = -1;
+        rdma->chunk_local_src.current_block_idx = -1;
+        rdma->chunk_local_src.current_chunk = -1;
+        rdma->chunk_local_dest.current_block_idx = -1;
+        rdma->chunk_local_dest.current_chunk = -1;

addr = inet_parse(host_port, NULL);
if (addr != NULL) {
-            rdma->port = atoi(addr->port);
-            rdma->host = g_strdup(addr->host);
+            rdma->lc_remote.port = atoi(addr->port);
+            rdma->lc_remote.host = g_strdup(addr->host);
} else {
ERROR(errp, "bad RDMA migration address '%s'", host_port);
g_free(rdma);
-            return NULL;
+            rdma = NULL;
}
+
+        qapi_free_InetSocketAddress(addr);
}

+    rdma->keepalive_startup = false;
+    connection_timer = timer_new_ms(QEMU_CLOCK_REALTIME, check_qp_state, rdma);
+    keepalive_timer = timer_new_ms(QEMU_CLOCK_REALTIME, send_keepalive, rdma);
+    rdma->lc_dest.id_str = "local destination";
+    rdma->lc_src.id_str = "local src";
+    rdma->lc_remote.id_str = "remote";
+
return rdma;
}

@@ -2540,9 +3042,9 @@ static int qemu_rdma_put_buffer(void *opaque, const uint8_t *buf,
* Push out any writes that
* we're queued up for pc.ram.
*/
-    ret = qemu_rdma_write_flush(f, rdma);
+    ret = qemu_rdma_write(f, rdma, &rdma->chunk_remote, NULL);
if (ret < 0) {
-        rdma->error_state = ret;
+        SET_ERROR(rdma, ret);
return ret;
}

@@ -2558,7 +3060,7 @@ static int qemu_rdma_put_buffer(void *opaque, const uint8_t *buf,
ret = qemu_rdma_exchange_send(rdma, &head, data, NULL, NULL, NULL);

if (ret < 0) {
-            rdma->error_state = ret;
+            SET_ERROR(rdma, ret);
return ret;
}

@@ -2618,7 +3120,7 @@ static int qemu_rdma_get_buffer(void *opaque, uint8_t *buf,
ret = qemu_rdma_exchange_recv(rdma, &head, RDMA_CONTROL_QEMU_FILE);

if (ret < 0) {
-        rdma->error_state = ret;
+        SET_ERROR(rdma, ret);
return ret;
}

@@ -2631,18 +3133,23 @@ static int qemu_rdma_get_buffer(void *opaque, uint8_t *buf,
/*
* Block until all the outstanding chunks have been delivered by the hardware.
*/
-static int qemu_rdma_drain_cq(QEMUFile *f, RDMAContext *rdma)
+static int qemu_rdma_drain_cq(QEMUFile *f, RDMAContext *rdma,
+                              RDMACurrentChunk *src,
+                              RDMACurrentChunk *dest)
{
int ret;
+    RDMALocalContext *lc = (migrate_use_mc_rdma_copy() && dest && dest != src) ?
+            (rdma->source ? &rdma->lc_src : &rdma->lc_dest) : &rdma->lc_remote;

-    if (qemu_rdma_write_flush(f, rdma) < 0) {
+    if (qemu_rdma_write(f, rdma, src, dest) < 0) {
return -EIO;
}

-    while (rdma->nb_sent) {
-        ret = qemu_rdma_block_for_wrid(rdma, RDMA_WRID_RDMA_WRITE, NULL);
+    while (lc->nb_sent) {
+        ret = qemu_rdma_block_for_wrid(rdma, lc,
+                                       RDMA_WRID_RDMA_WRITE_REMOTE, NULL);
if (ret < 0) {
-            fprintf(stderr, "rdma migration: complete polling error!n");
+            ERROR(NULL, "complete polling!");
return -EIO;
}
}
@@ -2657,13 +3164,190 @@ static int qemu_rdma_close(void *opaque)
DPRINTF("Shutting down connection.n");
QEMUFileRDMA *r = opaque;
if (r->rdma) {
-        qemu_rdma_cleanup(r->rdma);
+        qemu_rdma_cleanup(r->rdma, false);
g_free(r->rdma);
}
g_free(r);
return 0;
}

+static int qemu_rdma_instruct_unregister(RDMAContext *rdma, QEMUFile *f,
+                                         ram_addr_t block_offset,
+                                         ram_addr_t offset, long size)
+{
+    int ret;
+    uint64_t block, chunk;
+
+    if (size < 0) {
+        ret = qemu_rdma_drain_cq(f, rdma, &rdma->chunk_remote, NULL);
+        if (ret < 0) {
+            fprintf(stderr, "rdma: failed to synchronously drain"
+                            " completion queue before unregistration.n");
+            return ret;
+        }
+    }
+
+    ret = qemu_rdma_search_ram_block(rdma, block_offset,
+                                     offset, size, &block, &chunk);
+
+    if (ret) {
+        fprintf(stderr, "ram block search failedn");
+        return ret;
+    }
+
+    qemu_rdma_signal_unregister(rdma, block, chunk, 0);
+
+    /*
+     * Synchronous, gauranteed unregistration (should not occur during
+     * fast-path). Otherwise, unregisters will process on the next call to
+     * qemu_rdma_drain_cq()
+     */
+    if (size < 0) {
+        qemu_rdma_unregister_waiting(rdma);
+    }
+
+    return 0;
+}
+
+
+static int qemu_rdma_poll_until_empty(RDMAContext *rdma, RDMALocalContext *lc)
+{
+    uint64_t wr_id, wr_id_in;
+    int ret;
+
+    /*
+     * Drain the Completion Queue if possible, but do not block,
+     * just poll.
+     *
+     * If nothing to poll, the end of the iteration will do this
+     * again to make sure we don't overflow the request queue.
+     */
+    while (1) {
+        ret = qemu_rdma_poll(rdma, lc, &wr_id_in, NULL);
+        if (ret < 0) {
+            ERROR(NULL, "empty polling error! %d", ret);
+            return ret;
+        }
+
+        wr_id = wr_id_in & RDMA_WRID_TYPE_MASK;
+
+        if (wr_id == RDMA_WRID_NONE) {
+            break;
+        }
+    }
+
+    return 0;
+}
+
+/*
+ * Parameters:
+ *    @offset_{source|dest} == 0 :
+ *        This means that 'block_offset' is a full virtual address that does not
+ *        belong to a RAMBlock of the virtual machine and instead
+ *        represents a private malloc'd memory area that the caller wishes to
+ *        transfer. Source and dest can be different (either real RAMBlocks or
+ *        private).
+ *
+ *    @offset != 0 :
+ *        Offset is an offset to be added to block_offset and used
+ *        to also lookup the corresponding RAMBlock. Source and dest can be different
+ *        (either real RAMBlocks or private).
+ *
+ *    @size > 0 :
+ *        Amount of memory to copy locally using RDMA.
+ *
+ *    @size == 0 :
+ *        A 'hint' or 'advice' that means that we wish to speculatively
+ *        and asynchronously unregister either the source or destination memory.
+ *        In this case, there is no gaurantee that the unregister will actually happen,
+ *        for example, if the memory is being actively copied. Additionally, the memory
+ *        may be re-registered at any future time if a copy within the same
+ *        range was requested again, even if you attempted to unregister it here.
+ *
+ *    @size < 0 : TODO, not yet supported
+ *        Unregister the memory NOW. This means that the caller does not
+ *        expect there to be any future RDMA copies and we just want to clean
+ *        things up. This is used in case the upper layer owns the memory and
+ *        cannot wait for qemu_fclose() to occur.
+ */
+static int qemu_rdma_copy_page(QEMUFile *f, void *opaque,
+                                  ram_addr_t block_offset_dest,
+                                  ram_addr_t offset_dest,
+                                  ram_addr_t block_offset_source,
+                                  ram_addr_t offset_source,
+                                  long size)
+{
+    QEMUFileRDMA *rfile = opaque;
+    RDMAContext *rdma = rfile->rdma;
+    int ret;
+    RDMACurrentChunk *src = &rdma->chunk_local_src;
+    RDMACurrentChunk *dest = &rdma->chunk_local_dest;
+
+    CHECK_ERROR_STATE();
+
+    qemu_fflush(f);
+
+    if (size > 0) {
+        /*
+         * Add this page to the current 'chunk'. If the chunk
+         * is full, or the page doen't belong to the current chunk,
+         * an actual RDMA write will occur and a new chunk will be formed.
+         */
+        src->block_offset = block_offset_source;
+        src->offset = offset_source;
+        dest->block_offset = block_offset_dest;
+        dest->offset = offset_dest;
+
+        DDPRINTF("Copy page: %p src offset %" PRIu64
+                " dest %p offset %" PRIu64 "n",
+                (void *) block_offset_source, offset_source,
+                (void *) block_offset_dest, offset_dest);
+
+        ret = qemu_rdma_flush_unmergable(rdma, src, dest, f, size);
+
+        if (ret) {
+            ERROR(NULL, "local copy flush");
+            goto err;
+        }
+
+        if ((src->current_length >= RDMA_MERGE_MAX) ||
+            (dest->current_length >= RDMA_MERGE_MAX)) {
+            ret = qemu_rdma_write(f, rdma, src, dest);
+
+            if (ret < 0) {
+                goto err;
+            }
+        } else {
+            ret = 0;
+        }
+    } else {
+        ret = qemu_rdma_instruct_unregister(rdma, f, block_offset_source,
+                                                  offset_source, size);
+        if (ret) {
+            goto err;
+        }
+
+        ret = qemu_rdma_instruct_unregister(rdma, f, block_offset_dest,
+                                                  offset_dest, size);
+
+        if (ret) {
+            goto err;
+        }
+    }
+
+    ret = qemu_rdma_poll_until_empty(rdma,
+                rdma->source ? &rdma->lc_src : &rdma->lc_dest);
+
+    if (ret) {
+        goto err;
+    }
+
+    return RAM_COPY_CONTROL_DELAYED;
+err:
+    SET_ERROR(rdma, ret);
+    return ret;
+}
+
/*
* Parameters:
*    @offset == 0 :
@@ -2672,6 +3356,20 @@ static int qemu_rdma_close(void *opaque)
*        represents a private malloc'd memory area that the caller wishes to
*        transfer.
*
+ *        This allows callers to initiate RDMA transfers of arbitrary memory
+ *        areas and not just only by migration itself.
+ *
+ *        If this is true, then the virtual address specified by 'block_offset'
+ *        below must have been pre-registered with us in advance by calling the
+ *        new QEMUFileOps->add()/remove() functions on both sides of the
+ *        connection.
+ *
+ *        Also note: add()/remove() must been called in the *same sequence* and
+ *        against the *same size* private virtual memory on both sides of the
+ *        connection for this to work, regardless whether or not transfer of
+ *        this private memory was initiated by the migration code or a private
+ *        caller.
+ *
*    @offset != 0 :
*        Offset is an offset to be added to block_offset and used
*        to also lookup the corresponding RAMBlock.
@@ -2680,7 +3378,7 @@ static int qemu_rdma_close(void *opaque)
*        Initiate an transfer this size.
*
*    @size == 0 :
- *        A 'hint' or 'advice' that means that we wish to speculatively
+ *        A 'hint' that means that we wish to speculatively
*        and asynchronously unregister this memory. In this case, there is no
*        guarantee that the unregister will actually happen, for example,
*        if the memory is being actively transmitted. Additionally, the memory
@@ -2698,12 +3396,15 @@ static int qemu_rdma_close(void *opaque)
*                  sent. Usually, this will not be more than a few bytes of
*                  the protocol because most transfers are sent asynchronously.
*/
-static size_t qemu_rdma_save_page(QEMUFile *f, void *opaque,
-                                  ram_addr_t block_offset, ram_addr_t offset,
-                                  size_t size, int *bytes_sent)
+static int qemu_rdma_save_page(QEMUFile *f, void *opaque,
+                                  ram_addr_t block_offset,
+                                  uint8_t *host_addr,
+                                  ram_addr_t offset,
+                                  long size, int *bytes_sent)
{
QEMUFileRDMA *rfile = opaque;
RDMAContext *rdma = rfile->rdma;
+    RDMACurrentChunk *cc = &rdma->chunk_remote;
int ret;

CHECK_ERROR_STATE();
@@ -2716,11 +3417,26 @@ static size_t qemu_rdma_save_page(QEMUFile *f, void *opaque,
* is full, or the page doen't belong to the current chunk,
* an actual RDMA write will occur and a new chunk will be formed.
*/
-        ret = qemu_rdma_write(f, rdma, block_offset, offset, size);
+        cc->block_offset = block_offset;
+        cc->offset = offset;
+
+        ret = qemu_rdma_flush_unmergable(rdma, cc, NULL, f, size);
+
+        if (ret) {
+            ERROR(NULL, "remote flush unmergable");
+            goto err;
+        }
+
+        if (cc->current_length >= RDMA_MERGE_MAX) {
+            ret = qemu_rdma_write(f, rdma, cc, NULL);
+
if (ret < 0) {
-            fprintf(stderr, "rdma migration: write error! %dn", ret);
+                ERROR(NULL, "remote write! %d", ret);
goto err;
}
+        } else {
+            ret = 0;
+        }

/*
* We always return 1 bytes because the RDMA
@@ -2734,65 +3450,100 @@ static size_t qemu_rdma_save_page(QEMUFile *f, void *opaque,
*bytes_sent = 1;
}
} else {
-        uint64_t index, chunk;
+        ret = qemu_rdma_instruct_unregister(rdma, f, block_offset, offset, size);

-        /* TODO: Change QEMUFileOps prototype to be signed: size_t => long
-        if (size < 0) {
-            ret = qemu_rdma_drain_cq(f, rdma);
-            if (ret < 0) {
-                fprintf(stderr, "rdma: failed to synchronously drain"
-                                " completion queue before unregistration.n");
+        if (ret) {
goto err;
}
}
-        */

-        ret = qemu_rdma_search_ram_block(rdma, block_offset,
-                                         offset, size, &index, &chunk);
+    ret = qemu_rdma_poll_until_empty(rdma, &rdma->lc_remote);

if (ret) {
-            fprintf(stderr, "ram block search failedn");
goto err;
}

-        qemu_rdma_signal_unregister(rdma, index, chunk, 0);
+    return RAM_SAVE_CONTROL_DELAYED;
+err:
+    SET_ERROR(rdma, ret);
+    return ret;
+}

-        /*
-         * TODO: Synchronous, guaranteed unregistration (should not occur during
-         * fast-path). Otherwise, unregisters will process on the next call to
-         * qemu_rdma_drain_cq()
-        if (size < 0) {
-            qemu_rdma_unregister_waiting(rdma);
+static int qemu_rdma_accept_start(RDMAContext *rdma,
+                                  RDMALocalContext *lc,
+                                  struct rdma_cm_event **return_event)
+{
+    struct rdma_cm_event *cm_event = NULL;
+    int ret;
+
+    ret = rdma_get_cm_event(lc->channel, &cm_event);
+    if (ret) {
+        ERROR(NULL, "failed to wait for initial connect request");
+        goto err;
}
-        */
+
+    if (cm_event->event != RDMA_CM_EVENT_CONNECT_REQUEST) {
+        ERROR(NULL, "initial connect request is invalid");
+        ret = -EINVAL;
+        rdma_ack_cm_event(cm_event);
+        goto err;
}

-    /*
-     * Drain the Completion Queue if possible, but do not block,
-     * just poll.
-     *
-     * If nothing to poll, the end of the iteration will do this
-     * again to make sure we don't overflow the request queue.
-     */
-    while (1) {
-        uint64_t wr_id, wr_id_in;
-        int ret = qemu_rdma_poll(rdma, &wr_id_in, NULL);
-        if (ret < 0) {
-            fprintf(stderr, "rdma migration: polling error! %dn", ret);
+    if (lc->verbs && (lc->verbs != cm_event->id->verbs)) {
+        ret = -EINVAL;
+        ERROR(NULL, "ibv context %p != %p!", lc->verbs,
+                                             cm_event->id->verbs);
goto err;
}

-        wr_id = wr_id_in & RDMA_WRID_TYPE_MASK;
+    lc->cm_id = cm_event->id;
+    lc->verbs = cm_event->id->verbs;

-        if (wr_id == RDMA_WRID_NONE) {
-            break;
+    DPRINTF("verbs context after listen: %pn", lc->verbs);
+    qemu_rdma_dump_id("rdma_accept_start", lc->verbs);
+
+    if (return_event) {
+        *return_event = cm_event;
+    } else {
+        rdma_ack_cm_event(cm_event);
}
+
+    ret = qemu_rdma_alloc_pd_cq_qp(rdma, lc);
+    if (ret) {
+        goto err;
}

-    return RAM_SAVE_CONTROL_DELAYED;
+    return 0;
err:
-    rdma->error_state = ret;
-    return ret;
+    SET_ERROR(rdma, ret);
+    return rdma->error_state;
+}
+
+static int qemu_rdma_accept_finish(RDMAContext *rdma,
+                                   RDMALocalContext *lc)
+{
+    struct rdma_cm_event *cm_event;
+    int ret;
+
+    ret = rdma_get_cm_event(lc->channel, &cm_event);
+    if (ret) {
+        ERROR(NULL, "rdma_accept get_cm_event failed %d!", ret);
+        goto err;
+    }
+
+    if (cm_event->event != RDMA_CM_EVENT_ESTABLISHED) {
+        ERROR(NULL, "rdma_accept not event established!");
+        rdma_ack_cm_event(cm_event);
+        goto err;
+    }
+
+    rdma_ack_cm_event(cm_event);
+    lc->connected = true;
+
+    return 0;
+err:
+    SET_ERROR(rdma, ret);
+    return rdma->error_state;
}

static int qemu_rdma_accept(RDMAContext *rdma)
@@ -2804,19 +3555,10 @@ static int qemu_rdma_accept(RDMAContext *rdma)
.private_data_len = sizeof(cap),
};
struct rdma_cm_event *cm_event;
-    struct ibv_context *verbs;
int ret = -EINVAL;
int idx;

-    ret = rdma_get_cm_event(rdma->channel, &cm_event);
-    if (ret) {
-        goto err_rdma_dest_wait;
-    }
-
-    if (cm_event->event != RDMA_CM_EVENT_CONNECT_REQUEST) {
-        rdma_ack_cm_event(cm_event);
-        goto err_rdma_dest_wait;
-    }
+    ret = qemu_rdma_accept_start(rdma, &rdma->lc_remote, &cm_event);

memcpy(&cap, cm_event->param.conn.private_data, sizeof(cap));

@@ -2829,6 +3571,13 @@ static int qemu_rdma_accept(RDMAContext *rdma)
goto err_rdma_dest_wait;
}

+    rdma->keepalive_rkey = cap.keepalive_rkey;
+    rdma->keepalive_addr = cap.keepalive_addr;
+
+    DDPRINTF("Received keepalive params: key %x addr: %" PRIx64
+            " local %" PRIx64 "n",
+            cap.keepalive_rkey, cap.keepalive_addr, (uint64_t) &rdma->keepalive);
+
/*
* Respond with only the capabilities this version of QEMU knows about.
*/
@@ -2838,103 +3587,80 @@ static int qemu_rdma_accept(RDMAContext *rdma)
* Enable the ones that we do know about.
* Add other checks here as new ones are introduced.
*/
-    if (cap.flags & RDMA_CAPABILITY_PIN_ALL) {
-        rdma->pin_all = true;
-    }
-
-    rdma->cm_id = cm_event->id;
-    verbs = cm_event->id->verbs;
-
-    rdma_ack_cm_event(cm_event);
+    rdma->pin_all = cap.flags & RDMA_CAPABILITY_PIN_ALL;
+    rdma->do_keepalive = cap.flags & RDMA_CAPABILITY_KEEPALIVE;

DPRINTF("Memory pin all: %sn", rdma->pin_all ? "enabled" : "disabled");
+    DPRINTF("Keepalives: %sn", rdma->do_keepalive ? "enabled" : "disabled");

-    caps_to_network(&cap);
+    rdma_ack_cm_event(cm_event);

-    DPRINTF("verbs context after listen: %pn", verbs);
+    ret = qemu_rdma_reg_keepalive(rdma);

-    if (!rdma->verbs) {
-        rdma->verbs = verbs;
-    } else if (rdma->verbs != verbs) {
-            fprintf(stderr, "ibv context not matching %p, %p!n",
-                    rdma->verbs, verbs);
+    if (ret) {
+        ERROR(NULL, "allocating keepalive structures");
goto err_rdma_dest_wait;
}

-    qemu_rdma_dump_id("dest_init", verbs);
+    cap.keepalive_rkey = rdma->keepalive_mr->rkey,
+    cap.keepalive_addr = (uint64_t) &rdma->keepalive;

-    ret = qemu_rdma_alloc_pd_cq(rdma);
+    DDPRINTF("Sending keepalive params: key %x addr: %" PRIx64
+            " remote: %" PRIx64 "n",
+            cap.keepalive_rkey, cap.keepalive_addr, rdma->keepalive_addr);
+
+    caps_to_network(&cap);
+
+    ret = rdma_accept(rdma->lc_remote.cm_id, &conn_param);
if (ret) {
-        fprintf(stderr, "rdma migration: error allocating pd and cq!n");
+        ERROR(NULL, "rdma_accept returns %d!", ret);
goto err_rdma_dest_wait;
}

-    ret = qemu_rdma_alloc_qp(rdma);
+    ret = qemu_rdma_accept_finish(rdma, &rdma->lc_remote);
+
if (ret) {
-        fprintf(stderr, "rdma migration: error allocating qp!n");
+        ERROR(NULL, "finishing connection with capabilities to source");
goto err_rdma_dest_wait;
}

ret = qemu_rdma_init_ram_blocks(rdma);
if (ret) {
-        fprintf(stderr, "rdma migration: error initializing ram blocks!n");
+        ERROR(NULL, "initializing ram blocks!");
goto err_rdma_dest_wait;
}

for (idx = 0; idx < RDMA_WRID_MAX; idx++) {
ret = qemu_rdma_reg_control(rdma, idx);
if (ret) {
-            fprintf(stderr, "rdma: error registering %d control!n", idx);
-            goto err_rdma_dest_wait;
-        }
-    }
-
-    qemu_set_fd_handler2(rdma->channel->fd, NULL, NULL, NULL, NULL);
-
-    ret = rdma_accept(rdma->cm_id, &conn_param);
-    if (ret) {
-        fprintf(stderr, "rdma_accept returns %d!n", ret);
-        goto err_rdma_dest_wait;
-    }
-
-    ret = rdma_get_cm_event(rdma->channel, &cm_event);
-    if (ret) {
-        fprintf(stderr, "rdma_accept get_cm_event failed %d!n", ret);
+            ERROR(NULL, "registering %d control!", idx);
goto err_rdma_dest_wait;
}
-
-    if (cm_event->event != RDMA_CM_EVENT_ESTABLISHED) {
-        fprintf(stderr, "rdma_accept not event established!n");
-        rdma_ack_cm_event(cm_event);
-        goto err_rdma_dest_wait;
}

-    rdma_ack_cm_event(cm_event);
-    rdma->connected = true;
+    qemu_set_fd_handler2(rdma->lc_remote.channel->fd, NULL, NULL, NULL, NULL);

ret = qemu_rdma_post_recv_control(rdma, RDMA_WRID_READY);
if (ret) {
-        fprintf(stderr, "rdma migration: error posting second control recv!n");
+        ERROR(NULL, "posting second control recv!");
goto err_rdma_dest_wait;
}

-    qemu_rdma_dump_gid("dest_connect", rdma->cm_id);
+    qemu_rdma_dump_gid("dest_connect", rdma->lc_remote.cm_id);

return 0;

err_rdma_dest_wait:
-    rdma->error_state = ret;
-    qemu_rdma_cleanup(rdma);
+    SET_ERROR(rdma, ret);
+    qemu_rdma_cleanup(rdma, false);
return ret;
}

/*
* During each iteration of the migration, we listen for instructions
- * by the source VM to perform dynamic page registrations before they
+ * by the source VM to perform pinning operations before they
* can perform RDMA operations.
*
- * We respond with the 'rkey'.
- *
* Keep doing this until the source tells us to stop.
*/
static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,
@@ -2957,8 +3683,8 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,
RDMARegister *reg, *registers;
RDMACompress *comp;
RDMARegisterResult *reg_result;
-    static RDMARegisterResult results[RDMA_CONTROL_MAX_COMMANDS_PER_MESSAGE];
RDMALocalBlock *block;
+    static RDMARegisterResult results[RDMA_CONTROL_MAX_COMMANDS_PER_MESSAGE];
void *host_addr;
int ret = 0;
int idx = 0;
@@ -3009,8 +3735,7 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,
if (rdma->pin_all) {
ret = qemu_rdma_reg_whole_ram_blocks(rdma);
if (ret) {
-                    fprintf(stderr, "rdma migration: error dest "
-                                    "registering ram blocks!n");
+                    ERROR(NULL, "dest registering ram blocks!");
goto out;
}
}
@@ -3043,7 +3768,7 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,
(uint8_t *) rdma->block, &blocks);

if (ret < 0) {
-                fprintf(stderr, "rdma migration: error sending remote info!n");
+                ERROR(NULL, "sending remote info!");
goto out;
}

@@ -3055,8 +3780,7 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,
registers = (RDMARegister *) rdma->wr_data[idx].control_curr;

for (count = 0; count < head.repeat; count++) {
-                uint64_t chunk;
-                uint8_t *chunk_start, *chunk_end;
+                RDMACurrentChunk cc;

reg = &registers[count];
network_to_register(reg);
@@ -3065,30 +3789,28 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,

DDPRINTF("Registration request (%d): index %d, current_addr %"
PRIu64 " chunks: %" PRIu64 "n", count,
-                         reg->current_index, reg->key.current_addr, reg->chunks);
-
-                block = &(rdma->local_ram_blocks.block[reg->current_index]);
-                if (block->is_ram_block) {
-                    host_addr = (block->local_host_addr +
-                                (reg->key.current_addr - block->offset));
-                    chunk = ram_chunk_index(block->local_host_addr,
-                                            (uint8_t *) host_addr);
+                         reg->current_block_idx, reg->key.current_addr, reg->chunks);
+
+                cc.block = &(rdma->local_ram_blocks.block[reg->current_block_idx]);
+                if (cc.block->is_ram_block) {
+                    cc.addr = (cc.block->local_host_addr +
+                                (reg->key.current_addr - cc.block->offset));
+                    cc.chunk_idx = ram_chunk_index(block->local_host_addr, cc.addr);
} else {
-                    chunk = reg->key.chunk;
-                    host_addr = block->local_host_addr +
+                    cc.chunk_idx = reg->key.chunk;
+                    cc.addr = cc.block->local_host_addr +
(reg->key.chunk * (1UL << RDMA_REG_CHUNK_SHIFT));
}
-                chunk_start = ram_chunk_start(block, chunk);
-                chunk_end = ram_chunk_end(block, chunk + reg->chunks);
-                if (qemu_rdma_register_and_get_keys(rdma, block,
-                            (uint8_t *)host_addr, NULL, &reg_result->rkey,
-                            chunk, chunk_start, chunk_end)) {
+                cc.chunk_start = ram_chunk_start(cc.block, cc.chunk_idx);
+                cc.chunk_end = ram_chunk_end(cc.block, cc.chunk_idx + reg->chunks);
+                if (qemu_rdma_register_and_get_keys(rdma, &cc, &rdma->lc_remote,
+                                            false, NULL, &reg_result->rkey)) {
fprintf(stderr, "cannot get rkey!n");
ret = -EINVAL;
goto out;
}

-                reg_result->host_addr = (uint64_t) block->local_host_addr;
+                reg_result->host_addr = (uint64_t) cc.block->local_host_addr;

DDPRINTF("Registered rkey for this request: %xn",
reg_result->rkey);
@@ -3115,9 +3837,9 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,

DDPRINTF("Unregistration request (%d): "
" index %d, chunk %" PRIu64 "n",
-                         count, reg->current_index, reg->key.chunk);
+                         count, reg->current_block_idx, reg->key.chunk);

-                block = &(rdma->local_ram_blocks.block[reg->current_index]);
+                block = &(rdma->local_ram_blocks.block[reg->current_block_idx]);

ret = ibv_dereg_mr(block->pmr[reg->key.chunk]);
block->pmr[reg->key.chunk] = NULL;
@@ -3154,7 +3876,7 @@ static int qemu_rdma_registration_handle(QEMUFile *f, void *opaque,
} while (1);
out:
if (ret < 0) {
-        rdma->error_state = ret;
+        SET_ERROR(rdma, ret);
}
return ret;
}
@@ -3168,7 +3890,23 @@ static int qemu_rdma_registration_start(QEMUFile *f, void *opaque,
CHECK_ERROR_STATE();

DDDPRINTF("start section: %" PRIu64 "n", flags);
+
+    if (flags == RAM_CONTROL_FLUSH) {
+        int ret;
+
+        if (rdma->source) {
+            ret = qemu_rdma_drain_cq(f, rdma, &rdma->chunk_local_src,
+                                              &rdma->chunk_local_dest);
+
+            if (ret < 0) {
+                return ret;
+            }
+        }
+
+    } else {
qemu_put_be64(f, RAM_SAVE_FLAG_HOOK);
+    }
+
qemu_fflush(f);

return 0;
@@ -3190,7 +3928,7 @@ static int qemu_rdma_registration_stop(QEMUFile *f, void *opaque,
CHECK_ERROR_STATE();

qemu_fflush(f);
-    ret = qemu_rdma_drain_cq(f, rdma);
+    ret = qemu_rdma_drain_cq(f, rdma, &rdma->chunk_remote, NULL);

if (ret < 0) {
goto err;
@@ -3225,13 +3963,13 @@ static int qemu_rdma_registration_stop(QEMUFile *f, void *opaque,
/*
* The protocol uses two different sets of rkeys (mutually exclusive):
* 1. One key to represent the virtual address of the entire ram block.
-         *    (dynamic chunk registration disabled - pin everything with one rkey.)
+         *    (pinning enabled - pin everything with one rkey.)
* 2. One to represent individual chunks within a ram block.
-         *    (dynamic chunk registration enabled - pin individual chunks.)
+         *    (pinning disabled - pin individual chunks.)
*
* Once the capability is successfully negotiated, the destination transmits
* the keys to use (or sends them later) including the virtual addresses
-         * and then propagates the remote ram block descriptions to his local copy.
+         * and then propagates the remote ram block descriptions to their local copy.
*/

if (local->nb_blocks != nb_remote_blocks) {
@@ -3285,7 +4023,7 @@ static int qemu_rdma_registration_stop(QEMUFile *f, void *opaque,

return 0;
err:
-    rdma->error_state = ret;
+    SET_ERROR(rdma, ret);
return ret;
}

@@ -3294,7 +4032,23 @@ static int qemu_rdma_get_fd(void *opaque)
QEMUFileRDMA *rfile = opaque;
RDMAContext *rdma = rfile->rdma;

-    return rdma->comp_channel->fd;
+    return rdma->lc_remote.comp_chan->fd;
+}
+
+static int qemu_rdma_delete_block(QEMUFile *f, void *opaque,
+                                  ram_addr_t block_offset)
+{
+    QEMUFileRDMA *rfile = opaque;
+    return __qemu_rdma_delete_block(rfile->rdma, block_offset);
+}
+
+
+static int qemu_rdma_add_block(QEMUFile *f, void *opaque, void *host_addr,
+                         ram_addr_t block_offset, uint64_t length)
+{
+    QEMUFileRDMA *rfile = opaque;
+    return __qemu_rdma_add_block(rfile->rdma, host_addr,
+                                 block_offset, length);
}

const QEMUFileOps rdma_read_ops = {
@@ -3302,6 +4056,9 @@ const QEMUFileOps rdma_read_ops = {
.get_fd        = qemu_rdma_get_fd,
.close         = qemu_rdma_close,
.hook_ram_load = qemu_rdma_registration_handle,
+    .copy_page     = qemu_rdma_copy_page,
+    .add           = qemu_rdma_add_block,
+    .remove        = qemu_rdma_delete_block,
};

const QEMUFileOps rdma_write_ops = {
@@ -3310,6 +4067,9 @@ const QEMUFileOps rdma_write_ops = {
.before_ram_iterate = qemu_rdma_registration_start,
.after_ram_iterate  = qemu_rdma_registration_stop,
.save_page          = qemu_rdma_save_page,
+    .copy_page          = qemu_rdma_copy_page,
+    .add                = qemu_rdma_add_block,
+    .remove             = qemu_rdma_delete_block,
};

static void *qemu_fopen_rdma(RDMAContext *rdma, const char *mode)
@@ -3331,6 +4091,91 @@ static void *qemu_fopen_rdma(RDMAContext *rdma, const char *mode)
return r->file;
}

+static int init_local(RDMAContext *rdma)
+{
+    int ret;
+    struct rdma_conn_param cp_dest   = { .responder_resources = 2 },
+                           cp_source = { .initiator_depth = 2,
+                                         .retry_count = 5,
+                                       };
+
+    if (!migrate_use_mc_rdma_copy()) {
+        printf("RDMA local copy is disabled.n");
+        return 0;
+    }
+
+    rdma->lc_dest.port = 0;
+    rdma->lc_src.host = g_malloc(100);
+    rdma->lc_dest.host = g_malloc(100);
+    strcpy(rdma->lc_src.host, "127.0.0.1");
+    strcpy(rdma->lc_dest.host, rdma->lc_src.host);
+    rdma->lc_src.source = true;
+    rdma->lc_src.dest = false;
+    rdma->lc_dest.source = false;
+    rdma->lc_dest.dest = true;
+
+    /* bind & listen */
+    ret = qemu_rdma_device_init(rdma, NULL, &rdma->lc_dest);
+    if (ret) {
+        ERROR(NULL, "initialize local device destination");
+        goto err;
+    }
+
+    rdma->lc_src.port = ntohs(rdma_get_src_port(rdma->lc_dest.listen_id));
+
+    DPRINTF("bound to port: %dn", rdma->lc_src.port);
+
+    /* resolve */
+    ret = qemu_rdma_device_init(rdma, NULL, &rdma->lc_src);
+
+    if (ret) {
+        ERROR(NULL, "Failed to initialize local device source");
+        goto err;
+    }
+
+    /* async connect */
+    ret = rdma_connect(rdma->lc_src.cm_id, &cp_source);
+    if (ret) {
+        ERROR(NULL, "connect local device source");
+        goto err;
+    }
+
+    /* async accept */
+    ret = qemu_rdma_accept_start(rdma, &rdma->lc_dest, NULL);
+    if (ret) {
+        ERROR(NULL, "starting accept for local connection");
+        goto err;
+    }
+
+    /* accept */
+    ret = rdma_accept(rdma->lc_dest.cm_id, &cp_dest);
+    if (ret) {
+        ERROR(NULL, "rdma_accept returns %d (%s)!", ret, rdma->lc_dest.id_str);
+        goto err;
+    }
+
+    /* ack accept */
+    ret = qemu_rdma_connect_finish(rdma, &rdma->lc_src, NULL, NULL);
+    if (ret) {
+        ERROR(NULL, "finish local connection with source");
+        goto err;
+    }
+
+    /* established */
+    ret = qemu_rdma_accept_finish(rdma, &rdma->lc_dest);
+
+    if (ret) {
+        ERROR(NULL, "finish accept connection");
+        goto err;
+    }
+
+    return 0;
+err:
+    perror("init_local");
+    SET_ERROR(rdma, -ret);
+    return rdma->error_state;
+}
+
static void rdma_accept_incoming_migration(void *opaque)
{
RDMAContext *rdma = opaque;
@@ -3342,7 +4187,7 @@ static void rdma_accept_incoming_migration(void *opaque)
ret = qemu_rdma_accept(rdma);

if (ret) {
-        ERROR(errp, "RDMA Migration initialization failed!");
+        ERROR(errp, "initialization failed!");
return;
}

@@ -3351,12 +4196,45 @@ static void rdma_accept_incoming_migration(void *opaque)
f = qemu_fopen_rdma(rdma, "rb");
if (f == NULL) {
ERROR(errp, "could not qemu_fopen_rdma!");
-        qemu_rdma_cleanup(rdma);
-        return;
+        goto err;
+    }
+
+    if (rdma->do_keepalive) {
+        qemu_rdma_keepalive_start();
}

-    rdma->migration_started_on_destination = 1;
+    rdma->migration_started = 1;
process_incoming_migration(f);
+    return;
+err:
+    qemu_rdma_cleanup(rdma, false);
+}
+
+static int qemu_rdma_init_incoming(RDMAContext *rdma, Error **errp)
+{
+    int ret;
+    Error *local_err = NULL;
+
+    rdma->source = false;
+    rdma->dest = true;
+    rdma->lc_remote.source = false;
+    rdma->lc_remote.dest = true;
+
+    ret = qemu_rdma_device_init(rdma, &local_err, &rdma->lc_remote);
+
+    if (ret) {
+        goto err;
+    }
+
+    return 0;
+err:
+    if (rdma->lc_remote.listen_id) {
+        rdma_destroy_id(rdma->lc_remote.listen_id);
+        rdma->lc_remote.listen_id = NULL;
+    }
+    error_propagate(errp, local_err);
+
+    return ret;
}

void rdma_start_incoming_migration(const char *host_port, Error **errp)
@@ -3372,24 +4250,13 @@ void rdma_start_incoming_migration(const char *host_port, Error **errp)
goto err;
}

-    ret = qemu_rdma_dest_init(rdma, &local_err);
+    ret = qemu_rdma_init_incoming(rdma, &local_err);

if (ret) {
goto err;
}

-    DPRINTF("qemu_rdma_dest_init successn");
-
-    ret = rdma_listen(rdma->listen_id, 5);
-
-    if (ret) {
-        ERROR(errp, "listening on socket!");
-        goto err;
-    }
-
-    DPRINTF("rdma_listen successn");
-
-    qemu_set_fd_handler2(rdma->channel->fd, NULL,
+    qemu_set_fd_handler2(rdma->lc_remote.channel->fd, NULL,
rdma_accept_incoming_migration, NULL,
(void *)(intptr_t) rdma);
return;
@@ -3411,14 +4278,21 @@ void rdma_start_outgoing_migration(void *opaque,
goto err;
}

-    ret = qemu_rdma_source_init(rdma, &local_err,
-        s->enabled_capabilities[MIGRATION_CAPABILITY_RDMA_PIN_ALL]);
+    rdma->source = true;
+    rdma->dest = false;
+
+    if (init_local(rdma)) {
+        ERROR(temp, "could not initialize local rdma queue pairs!");
+        goto err;
+    }
+
+    ret = qemu_rdma_init_outgoing(rdma, &local_err, s);

if (ret) {
goto err;
}

-    DPRINTF("qemu_rdma_source_init successn");
+    DPRINTF("qemu_rdma_init_outgoing successn");
ret = qemu_rdma_connect(rdma, &local_err);

if (ret) {
@@ -3428,6 +4302,12 @@ void rdma_start_outgoing_migration(void *opaque,
DPRINTF("qemu_rdma_source_connect successn");

s->file = qemu_fopen_rdma(rdma, "wb");
+    rdma->migration_started = 1;
+
+    if (rdma->do_keepalive) {
+        qemu_rdma_keepalive_start();
+    }
+
migrate_fd_connect(s);
return;
err:
