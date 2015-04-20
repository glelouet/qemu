index 9f06450..9175e48 100644
--- a/target-arm/translate-a64.c
+++ b/target-arm/translate-a64.c
@@ -8925,7 +8925,7 @@ static void disas_simd_3same_int(DisasContext *s, uint32_t insn)

genfn = fns[size][is_sub];
read_vec_element_i32(s, tcg_op1, rd, pass, MO_32);
-                genfn(tcg_res, tcg_res, tcg_op1);
+                genfn(tcg_res, tcg_op1, tcg_res);
}

write_vec_element_i32(s, tcg_res, rd, pass, MO_32);
