index 71dccae..05f82d2 100644
--- a/target-mips/translate.c
+++ b/target-mips/translate.c
@@ -1216,20 +1216,28 @@ static void gen_store_fpr32(TCGv_i32 t, int reg)
tcg_temp_free_i64(t64);
}

-static void gen_load_fpr32h(TCGv_i32 t, int reg)
+static void gen_load_fpr32h(DisasContext *ctx, TCGv_i32 t, int reg)
{
+    if (ctx->hflags & MIPS_HFLAG_F64) {
TCGv_i64 t64 = tcg_temp_new_i64();
tcg_gen_shri_i64(t64, fpu_f64[reg], 32);
tcg_gen_trunc_i64_i32(t, t64);
tcg_temp_free_i64(t64);
+    } else {
+        gen_load_fpr32(t, reg | 1);
+    }
}

-static void gen_store_fpr32h(TCGv_i32 t, int reg)
+static void gen_store_fpr32h(DisasContext *ctx, TCGv_i32 t, int reg)
{
+    if (ctx->hflags & MIPS_HFLAG_F64) {
TCGv_i64 t64 = tcg_temp_new_i64();
tcg_gen_extu_i32_i64(t64, t);
tcg_gen_deposit_i64(fpu_f64[reg], fpu_f64[reg], t64, 32, 32);
tcg_temp_free_i64(t64);
+    } else {
+        gen_store_fpr32(t, reg | 1);
+    }
}

static void gen_load_fpr64(DisasContext *ctx, TCGv_i64 t, int reg)
@@ -6613,7 +6621,7 @@ static void gen_mftr(CPUMIPSState *env, DisasContext *ctx, int rt, int rd,
} else {
TCGv_i32 fp0 = tcg_temp_new_i32();

-            gen_load_fpr32h(fp0, rt);
+            gen_load_fpr32h(ctx, fp0, rt);
tcg_gen_ext_i32_tl(t0, fp0);
tcg_temp_free_i32(fp0);
}
@@ -6812,7 +6820,7 @@ static void gen_mttr(CPUMIPSState *env, DisasContext *ctx, int rd, int rt,
TCGv_i32 fp0 = tcg_temp_new_i32();

tcg_gen_trunc_tl_i32(fp0, t0);
-            gen_store_fpr32h(fp0, rd);
+            gen_store_fpr32h(ctx, fp0, rd);
tcg_temp_free_i32(fp0);
}
break;
@@ -7283,7 +7291,7 @@ static void gen_cp1 (DisasContext *ctx, uint32_t opc, int rt, int fs)
{
TCGv_i32 fp0 = tcg_temp_new_i32();

-            gen_load_fpr32h(fp0, fs);
+            gen_load_fpr32h(ctx, fp0, fs);
tcg_gen_ext_i32_tl(t0, fp0);
tcg_temp_free_i32(fp0);
}
@@ -7296,7 +7304,7 @@ static void gen_cp1 (DisasContext *ctx, uint32_t opc, int rt, int fs)
TCGv_i32 fp0 = tcg_temp_new_i32();

tcg_gen_trunc_tl_i32(fp0, t0);
-            gen_store_fpr32h(fp0, fs);
+            gen_store_fpr32h(ctx, fp0, fs);
tcg_temp_free_i32(fp0);
}
opn = "mthc1";
@@ -7383,7 +7391,8 @@ static inline void gen_movcf_d (DisasContext *ctx, int fs, int fd, int cc, int t
gen_set_label(l1);
}

-static inline void gen_movcf_ps (int fs, int fd, int cc, int tf)
+static inline void gen_movcf_ps(DisasContext *ctx, int fs, int fd,
+                                int cc, int tf)
{
int cond;
TCGv_i32 t0 = tcg_temp_new_i32();
@@ -7403,8 +7412,8 @@ static inline void gen_movcf_ps (int fs, int fd, int cc, int tf)

tcg_gen_andi_i32(t0, fpu_fcr31, 1 << get_fp_bit(cc+1));
tcg_gen_brcondi_i32(cond, t0, 0, l2);
-    gen_load_fpr32h(t0, fs);
-    gen_store_fpr32h(t0, fd);
+    gen_load_fpr32h(ctx, t0, fs);
+    gen_store_fpr32h(ctx, t0, fd);
tcg_temp_free_i32(t0);
gen_set_label(l2);
}
@@ -8389,7 +8398,7 @@ static void gen_farith (DisasContext *ctx, enum fopcode op1,
break;
case OPC_MOVCF_PS:
check_cp1_64bitmode(ctx);
-        gen_movcf_ps(fs, fd, (ft >> 2) & 0x7, ft & 0x1);
+        gen_movcf_ps(ctx, fs, fd, (ft >> 2) & 0x7, ft & 0x1);
opn = "movcf.ps";
break;
case OPC_MOVZ_PS:
@@ -8514,7 +8523,7 @@ static void gen_farith (DisasContext *ctx, enum fopcode op1,
{
TCGv_i32 fp0 = tcg_temp_new_i32();

-            gen_load_fpr32h(fp0, fs);
+            gen_load_fpr32h(ctx, fp0, fs);
gen_helper_float_cvts_pu(fp0, cpu_env, fp0);
gen_store_fpr32(fp0, fd);
tcg_temp_free_i32(fp0);
@@ -8553,7 +8562,7 @@ static void gen_farith (DisasContext *ctx, enum fopcode op1,

gen_load_fpr32(fp0, fs);
gen_load_fpr32(fp1, ft);
-            gen_store_fpr32h(fp0, fd);
+            gen_store_fpr32h(ctx, fp0, fd);
gen_store_fpr32(fp1, fd);
tcg_temp_free_i32(fp0);
tcg_temp_free_i32(fp1);
@@ -8567,9 +8576,9 @@ static void gen_farith (DisasContext *ctx, enum fopcode op1,
TCGv_i32 fp1 = tcg_temp_new_i32();

gen_load_fpr32(fp0, fs);
-            gen_load_fpr32h(fp1, ft);
+            gen_load_fpr32h(ctx, fp1, ft);
gen_store_fpr32(fp1, fd);
-            gen_store_fpr32h(fp0, fd);
+            gen_store_fpr32h(ctx, fp0, fd);
tcg_temp_free_i32(fp0);
tcg_temp_free_i32(fp1);
}
@@ -8581,10 +8590,10 @@ static void gen_farith (DisasContext *ctx, enum fopcode op1,
TCGv_i32 fp0 = tcg_temp_new_i32();
TCGv_i32 fp1 = tcg_temp_new_i32();

-            gen_load_fpr32h(fp0, fs);
+            gen_load_fpr32h(ctx, fp0, fs);
gen_load_fpr32(fp1, ft);
gen_store_fpr32(fp1, fd);
-            gen_store_fpr32h(fp0, fd);
+            gen_store_fpr32h(ctx, fp0, fd);
tcg_temp_free_i32(fp0);
tcg_temp_free_i32(fp1);
}
@@ -8596,10 +8605,10 @@ static void gen_farith (DisasContext *ctx, enum fopcode op1,
TCGv_i32 fp0 = tcg_temp_new_i32();
TCGv_i32 fp1 = tcg_temp_new_i32();

-            gen_load_fpr32h(fp0, fs);
-            gen_load_fpr32h(fp1, ft);
+            gen_load_fpr32h(ctx, fp0, fs);
+            gen_load_fpr32h(ctx, fp1, ft);
gen_store_fpr32(fp1, fd);
-            gen_store_fpr32h(fp0, fd);
+            gen_store_fpr32h(ctx, fp0, fd);
tcg_temp_free_i32(fp0);
tcg_temp_free_i32(fp1);
}
@@ -8763,23 +8772,23 @@ static void gen_flt3_arith (DisasContext *ctx, uint32_t opc,

tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0, l1);
gen_load_fpr32(fp, fs);
-            gen_load_fpr32h(fph, fs);
+            gen_load_fpr32h(ctx, fph, fs);
gen_store_fpr32(fp, fd);
-            gen_store_fpr32h(fph, fd);
+            gen_store_fpr32h(ctx, fph, fd);
tcg_gen_br(l2);
gen_set_label(l1);
tcg_gen_brcondi_tl(TCG_COND_NE, t0, 4, l2);
tcg_temp_free(t0);
#ifdef TARGET_WORDS_BIGENDIAN
gen_load_fpr32(fp, fs);
-            gen_load_fpr32h(fph, ft);
-            gen_store_fpr32h(fp, fd);
+            gen_load_fpr32h(ctx, fph, ft);
+            gen_store_fpr32h(ctx, fp, fd);
gen_store_fpr32(fph, fd);
#else
-            gen_load_fpr32h(fph, fs);
+            gen_load_fpr32h(ctx, fph, fs);
gen_load_fpr32(fp, ft);
gen_store_fpr32(fph, fd);
-            gen_store_fpr32h(fp, fd);
+            gen_store_fpr32h(ctx, fp, fd);
#endif
gen_set_label(l2);
tcg_temp_free_i32(fp);
@@ -11976,7 +11985,7 @@ static void decode_micromips32_opc (CPUMIPSState *env, DisasContext *ctx,
gen_movcf_d(ctx, rs, rt, cc, 0);
break;
case FMT_SDPS_PS:
-                        gen_movcf_ps(rs, rt, cc, 0);
+                        gen_movcf_ps(ctx, rs, rt, cc, 0);
break;
default:
goto pool32f_invalid;
@@ -11991,7 +12000,7 @@ static void decode_micromips32_opc (CPUMIPSState *env, DisasContext *ctx,
gen_movcf_d(ctx, rs, rt, cc, 1);
break;
case FMT_SDPS_PS:
-                        gen_movcf_ps(rs, rt, cc, 1);
+                        gen_movcf_ps(ctx, rs, rt, cc, 1);
break;
default:
goto pool32f_invalid;
