// Copyright (c) 2011 ARM Limited. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifdef __ARM_NEON__
 /* Initial ARM Neon implementation of core YUV2RGB functions. */

    .text
    .align
    .global yuv422_2_rgb8888_neon
    .type   yuv422_2_rgb8888_neon, %function
    .global yuv422_2_rgb4444_neon
    .type   yuv422_2_rgb4444_neon, %function
    .global yuv422_2_rgb565_neon
    .type   yuv422_2_rgb565_neon, %function
#define coef_y         D0
#define coef_v_r       D1
#define coef_u_g       D2
#define coef_v_g       D3
#define coef_u_b       D4
/* D5 is spare */
#define bias_r         Q3
#define bias_r_lo      D6
#define bias_r_hi      D7
#define bias_g         Q4
#define bias_g_lo      D8
#define bias_g_hi      D9
#define bias_b         Q5
#define bias_b_lo      D10
#define bias_b_hi      D11

/* Input data */
#define y_even         D25
#define u              D27 /*overlaps with Q8 - b_delta, but safe */
#define y_odd          D29
#define v              D31 /*overlaps with Q8 - b_delta, but safe */

/* Chrominance signal for whole 16x2 block */
#define r_delta        Q6
#define g_delta        Q7
#define b_delta        Q8

/* Current group of 8 pixels */
#define red            Q9
#define grn            Q10
#define blu            Q11
#define y_scale        Q12

/* output area, in the right order for interleaved output with VST4 */
#define red8_o         D24 /* overlaps with Q12 - y_scale, but safe */
#define blu8_o         D25 /* overlaps with Q12 - y_scale, but safe */
#define red8_e         D26 /* overlaps with y_even, but safe */
#define blu8_e         D27
#define grn8_o         D28 /* overlaps with y_odd, but safe */
#define alp8_o         D29
#define grn8_e         D30
#define alp8_e         D31

/* ARM registers */
#define rgb_t_ptr      r0
#define src_t_ptr      r1
#define width          r2
#define height         r3
#define yuv_pitch      r4
#define rgb_pitch      r5
#define count          r6
#define aligned_count  sl

/* Constants */
/* 8-bit constants can be loaded into vectors using VMOV */
#define C_Y_SCALE      64   /* Y scale , 74 */
#define C_V_RED        89  /* v -> red coefficient, 102 */
#define C_U_GREEN      22   /* u -> green , -25 */
#define C_V_GREEN      45   /* v -> green , -52 */
#define C_U_BLUE       113  /* u -> blue, +129 */

/* Coefficients */
coefficients:
coeff_bias_r:
    .short  -11392  /* bias_r = 64 * (0)                + (89 * -128) */
                    /*          -0                    + -11,392      */
coeff_bias_g:
    .short    8576  /* bias_g = 64 * (0) -  22 * (-128) - ( 45 * -128) */
                    /*          -0     -  -2816       - -5,760       */
coeff_bias_b:
    .short  -14464  /* bias_b = 64 * (0) + 113 * (-128)                */
                    /*          -0     + -14,464                     */
coeff_pad:
    .short       0

#define rgb_ptr     rgb_t_ptr
#define src_ptr     src_t_ptr

yuv422_2_rgb8888_neon:
    /*  r0 = dst_ptr */
    /*  r1 = src_ptr */
    /*  r2 = width */
    /*  r3 = height */
    /*  <> = yuv_pitch */
    /*  <> = rgb_pitch */
    .fnstart
        PUSH            {r4-r12, lr}         /* 10 words */
        VPUSH           {q4-q7}              /* 4Q -> 16 words */

        LDR             yuv_pitch, [sp, #26*4]
        LDR             rgb_pitch, [sp, #27*4]
        ADR             lr, coefficients

        /* We can't cope with a width less than 16. Check for that. */
        CMP             width, #16
        VPOPLT          {q4-q7}
        POPLT           {r4-r12, pc}

        /* Load up vectors containing the bias values. */
        VLD1.S16        {bias_r_lo[], bias_r_hi[]}, [lr]!
        VLD1.S16        {bias_g_lo[], bias_g_hi[]}, [lr]!
        VLD1.S16        {bias_b_lo[], bias_b_hi[]}, [lr]!

        /* Build coefficient vectors containing the same value in each element. */
        VMOV.U8         coef_y, #C_Y_SCALE
        VMOV.U8         coef_v_r, #C_V_RED
        VMOV.U8         coef_u_g, #C_U_GREEN
        VMOV.U8         coef_v_g, #C_V_GREEN
        VMOV.U8         coef_u_b, #C_U_BLUE

loop_v_422:
        MOV             aligned_count, width
        /* If width is not an integer multiple of 16, run the
           first pass through the loop with the correct number
           of pixels to correct the size for the remaining loops. */
        ANDS            count, width, #15
        /* If we're already aligned (i.e. count is now 0), set count
           to 16 to run the first loop as normal. */
        MOVEQ           count, #16

        /*ADD             rgb_ptr, width, LSL #2*/
        MOV						r8,src_ptr

loop_h_422:
        /* Load 16 pixels of yuv */
        VLD4.U8         {y_even, u, y_odd, v}, [src_ptr]

				MOV							r7,count
				ADD							r7,r7,#1/**/
				AND						r7,r7,#0x1E			/**/
				ADD							src_ptr,r7, LSL #1

        /* Attempt to preload the next set of yuv data, for better
           performance. */
        PLD             [src_ptr]/**/

        /* Calculate contribution from chrominance signals. */
        VMULL.U8        r_delta, v, coef_v_r
        VMULL.U8        g_delta, u, coef_u_g
        VMLAL.U8        g_delta, v, coef_v_g
        VMULL.U8        b_delta, u, coef_u_b

        /* Add bias. */
        VADD.S16        r_delta, r_delta, bias_r
        VSUB.S16        g_delta, bias_g, g_delta
        VADD.S16        b_delta, b_delta, bias_b

        /* Even: combine luminance and chrominance. */
        VMULL.U8        y_scale, y_even, coef_y
        VQADD.S16       red, y_scale, r_delta
        VQADD.S16       grn, y_scale, g_delta
        VQADD.S16       blu, y_scale, b_delta

        /* Even: clamp, rescale and clip colour components to 8 bits. */
        VQRSHRUN.S16    red8_e, red, #6
        VQRSHRUN.S16    grn8_e, grn, #6
        VQRSHRUN.S16    blu8_e, blu, #6

        /* Odd: combine luminance and chrominance. */
        VMULL.U8        y_scale, y_odd, coef_y
        VQADD.S16       red, y_scale, r_delta
        VQADD.S16       grn, y_scale, g_delta
        VQADD.S16       blu, y_scale, b_delta

        /* Even: set up alpha data. */
        VMOV.U8         alp8_e, #0xFF

        /* Odd: set up alpha data. */
        VMOV.U8         alp8_o, #0xFF

        /* Odd: clamp, rescale and clip colour components to 8 bits. */
        VQRSHRUN.S16    red8_o, red, #6
        VQRSHRUN.S16    blu8_o, blu, #6
        VQRSHRUN.S16    grn8_o, grn, #6

        /* Even: Interleave the colour and alpha components
           ready for storage.
        VZIP.U8         red8_e, alp8_e
        VZIP.U8         blu8_e, grn8_e*/

/*			for ABGR32  */
				VZIP.U8         blu8_e, alp8_e
        VZIP.U8         red8_e, grn8_e


        /* Odd: Interleave the colour and alpha components
           ready for storage.
        VZIP.U8         red8_o, alp8_o
        VZIP.U8         blu8_o, grn8_o*/

/*			for ABGR32	*/
        VZIP.U8         blu8_o, alp8_o
        VZIP.U8         red8_o, grn8_o

				VSWP						Q12,Q13
				VSWP						Q14,Q15



				MOV							r7,#16
				SUB							r7,r7,count
				AND							r7,r7,#1
				CMP							r7,#0
				BGT							not_16_path

        /* Store 16 pixels of ARGB32, interleaving even and odd. */
        VST4.U16        {red8_o, blu8_o, red8_e, blu8_e}, [rgb_ptr]
        ADD             rgb_ptr, count, LSL #1
        VST4.U16        {grn8_o, alp8_o, grn8_e, alp8_e}, [rgb_ptr]
				ADD             rgb_ptr, count, LSL #1
				B								next_loop

not_16_path:
				MOV							r7,count
				VST4.U16        {red8_o, blu8_o, red8_e, blu8_e}, [rgb_ptr]
				SUB							r7,r7,#1
        ADD             rgb_ptr, r7, LSL #1
        VST4.U16        {grn8_o, alp8_o, grn8_e, alp8_e}, [rgb_ptr]
        ADD							r7,r7,#2
				ADD             rgb_ptr, r7, LSL #1
				/*B								next_loop				*/
/*
				MOV							r7,#16
				SUB							r7,r7,count
				AND						r7,r7,#0x0E
				SUB							src_ptr,r7, LSL #1
*/

next_loop:
        /* Have we reached the end of the row yet? */
        SUBS            aligned_count, aligned_count, count

        /* On the second (and subsequent) passes through this code,
           we'll always be working on 16 pixels at once. */
        MOV             count, #16
        BGT             loop_h_422

        /* Update pointers for new row of data. */
        /*SUB             rgb_ptr, width, LSL #2*/
        MOV							src_ptr,r8
        /*SUB             src_ptr, width, LSL #1*/
        /*ADD             rgb_ptr, rgb_pitch*/
        ADD             src_ptr, yuv_pitch

        /* Have we reached the bottom yet? */
        SUBS            height, height, #1
        BGT             loop_v_422

        VPOP            {q4-q7}
        POP             {r4-r12, pc}
        .fnend



yuv422_2_rgb4444_neon:
    /*  r0 = dst_ptr */
    /*  r1 = src_ptr */
    /*  r2 = width */
    /*  r3 = height */
    /*  <> = yuv_pitch */
    /*  <> = rgb_pitch */
    .fnstart
        PUSH            {r4-r12, lr}         /* 10 words */
        VPUSH           {q4-q7}              /* 4Q -> 16 words */

        LDR             yuv_pitch, [sp, #26*4]
        LDR             rgb_pitch, [sp, #27*4]
        ADR             lr, coefficients

        /* We can't cope with a width less than 16. Check for that. */
        CMP             width, #16
        VPOPLT          {q4-q7}
        POPLT           {r4-r12, pc}

        /* Load up vectors containing the bias values. */
        VLD1.S16        {bias_r_lo[], bias_r_hi[]}, [lr]!
        VLD1.S16        {bias_g_lo[], bias_g_hi[]}, [lr]!
        VLD1.S16        {bias_b_lo[], bias_b_hi[]}, [lr]!

        /* Build coefficient vectors containing the same value in each element. */
        VMOV.U8         coef_y, #C_Y_SCALE
        VMOV.U8         coef_v_r, #C_V_RED
        VMOV.U8         coef_u_g, #C_U_GREEN
        VMOV.U8         coef_v_g, #C_V_GREEN
        VMOV.U8         coef_u_b, #C_U_BLUE

argb4444_loop_v_422:
        MOV             aligned_count, width
        /* If width is not an integer multiple of 16, run the
           first pass through the loop with the correct number
           of pixels to correct the size for the remaining loops. */
        ANDS            count, width, #15
        /* If we're already aligned (i.e. count is now 0), set count
           to 16 to run the first loop as normal. */
        MOVEQ           count, #16

        /*ADD             rgb_ptr, width, LSL #2*/
        MOV						r8,src_ptr

argb4444_loop_h_422:
        /* Load 16 pixels of yuv */
        VLD4.U8         {y_even, u, y_odd, v}, [src_ptr]

				MOV							r7,count
				ADD							r7,r7,#1/**/
				AND						r7,r7,#0x1E			/**/
				ADD							src_ptr,r7, LSL #1

        /* Attempt to preload the next set of yuv data, for better
           performance. */
        PLD             [src_ptr]/**/

        /* Calculate contribution from chrominance signals. */
        VMULL.U8        r_delta, v, coef_v_r
        VMULL.U8        g_delta, u, coef_u_g
        VMLAL.U8        g_delta, v, coef_v_g
        VMULL.U8        b_delta, u, coef_u_b

        /* Add bias. */
        VADD.S16        r_delta, r_delta, bias_r
        VSUB.S16        g_delta, bias_g, g_delta
        VADD.S16        b_delta, b_delta, bias_b

        /* Even: combine luminance and chrominance. */
        VMULL.U8        y_scale, y_even, coef_y
        VQADD.S16       red, y_scale, r_delta
        VQADD.S16       grn, y_scale, g_delta
        VQADD.S16       blu, y_scale, b_delta

        /* Even: clamp, rescale and clip colour components to 8 bits. */
        VQRSHRUN.S16    red8_e, red, #6
        VQRSHRUN.S16    grn8_e, grn, #6
        VQRSHRUN.S16    blu8_e, blu, #6

        /* Odd: combine luminance and chrominance. */
        VMULL.U8        y_scale, y_odd, coef_y
        VQADD.S16       red, y_scale, r_delta
        VQADD.S16       grn, y_scale, g_delta
        VQADD.S16       blu, y_scale, b_delta

        /* Even: set up alpha data. */
        VMOV.U8         alp8_e, #0xFF

        /* Odd: set up alpha data. */
        VMOV.U8         alp8_o, #0xFF

        /* Odd: clamp, rescale and clip colour components to 8 bits. */
        VQRSHRUN.S16    red8_o, red, #6
        VQRSHRUN.S16    blu8_o, blu, #6
        VQRSHRUN.S16    grn8_o, grn, #6

        /* Even: Interleave the colour and alpha components
           ready for storage. for ABGR32  */
				/*VZIP.U8         blu8_e, alp8_e
        VZIP.U8         red8_e, grn8_e*/


        /* Odd: Interleave the colour and alpha components
           ready for storage.for ABGR32	*/
        /*VZIP.U8         blu8_o, alp8_o
        VZIP.U8         red8_o, grn8_o*/

				/*even*/
				VSRI.8				blu8_e,alp8_e,#4

				VSRI.8				red8_e,grn8_e,#4

				/*odd*/
				VSRI.8				blu8_o,alp8_o,#4

				VSRI.8				red8_o,grn8_o,#4

				VSWP						D24,D25
				VSWP						D26,D27
				VSWP						Q12,Q13

				MOV							r7,#16
				SUB							r7,r7,count
				AND							r7,r7,#1
				CMP							r7,#0
				BGT							argb4444_not_16_path

        /* Store 16 pixels of ARGB16, interleaving even and odd. */
        VST4.U8        {red8_o, blu8_o,red8_e, blu8_e}, [rgb_ptr]
				ADD             rgb_ptr, count, LSL #1
				B								argb4444_next_loop

argb4444_not_16_path:
				MOV							r7,count
				VST4.U8        {red8_o, blu8_o,red8_e, blu8_e}, [rgb_ptr]
				SUB							r7,r7,#1
        ADD             rgb_ptr, r7, LSL #0

        ADD							r7,r7,#2
				ADD             rgb_ptr, r7, LSL #0


argb4444_next_loop:
        /* Have we reached the end of the row yet? */
        SUBS            aligned_count, aligned_count, count

        /* On the second (and subsequent) passes through this code,
           we'll always be working on 16 pixels at once. */
        MOV             count, #16
        BGT             argb4444_loop_h_422

        /* Update pointers for new row of data. */
        /*SUB             rgb_ptr, width, LSL #2*/
        MOV							src_ptr,r8
        /*SUB             src_ptr, width, LSL #1*/
        /*ADD             rgb_ptr, rgb_pitch*/
        ADD             src_ptr, yuv_pitch

        /* Have we reached the bottom yet? */
        SUBS            height, height, #1
        BGT             argb4444_loop_v_422

        VPOP            {q4-q7}
        POP             {r4-r12, pc}
        .fnend


yuv422_2_rgb565_neon:
    /*  r0 = dst_ptr */
    /*  r1 = src_ptr */
    /*  r2 = width */
    /*  r3 = height */
    /*  <> = yuv_pitch */
    /*  <> = rgb_pitch */
    .fnstart
        PUSH            {r4-r12, lr}         /* 10 words */
        VPUSH           {q4-q7}              /* 4Q -> 16 words */

        LDR             yuv_pitch, [sp, #26*4]
        LDR             rgb_pitch, [sp, #27*4]
        ADR             lr, coefficients

        /* We can't cope with a width less than 16. Check for that. */
        CMP             width, #16
        VPOPLT          {q4-q7}
        POPLT           {r4-r12, pc}

        /* Load up vectors containing the bias values. */
        VLD1.S16        {bias_r_lo[], bias_r_hi[]}, [lr]!
        VLD1.S16        {bias_g_lo[], bias_g_hi[]}, [lr]!
        VLD1.S16        {bias_b_lo[], bias_b_hi[]}, [lr]!

        /* Build coefficient vectors containing the same value in each element. */
        VMOV.U8         coef_y, #C_Y_SCALE
        VMOV.U8         coef_v_r, #C_V_RED
        VMOV.U8         coef_u_g, #C_U_GREEN
        VMOV.U8         coef_v_g, #C_V_GREEN
        VMOV.U8         coef_u_b, #C_U_BLUE

rgb565_loop_v_422:
        MOV             aligned_count, width
        /* If width is not an integer multiple of 16, run the
           first pass through the loop with the correct number
           of pixels to correct the size for the remaining loops. */
        ANDS            count, width, #15
        /* If we're already aligned (i.e. count is now 0), set count
           to 16 to run the first loop as normal. */
        MOVEQ           count, #16

        /*ADD             rgb_ptr, width, LSL #2*/
        MOV						r8,src_ptr

rgb565_loop_h_422:
        /* Load 16 pixels of yuv */
        VLD4.U8         {y_even, u, y_odd, v}, [src_ptr]

/**/
				MOV							r7,count
				ADD							r7,r7,#1
				AND						r7,r7,#0x1E
				ADD							src_ptr,r7, LSL #1

        /* Attempt to preload the next set of yuv data, for better
           performance. */
        PLD             [src_ptr]/**/

        /* Calculate contribution from chrominance signals. */
        VMULL.U8        r_delta, v, coef_v_r
        VMULL.U8        g_delta, u, coef_u_g
        VMLAL.U8        g_delta, v, coef_v_g
        VMULL.U8        b_delta, u, coef_u_b

        /* Add bias. */
        VADD.S16        r_delta, r_delta, bias_r
        VSUB.S16        g_delta, bias_g, g_delta
        VADD.S16        b_delta, b_delta, bias_b

        /* Even: combine luminance and chrominance. */
        VMULL.U8        y_scale, y_even, coef_y
        VQADD.S16       red, y_scale, r_delta
        VQADD.S16       grn, y_scale, g_delta
        VQADD.S16       blu, y_scale, b_delta

        /* Even: clamp, rescale and clip colour components to 8 bits. */
        VQRSHRUN.S16    red8_e, red, #6
        VQRSHRUN.S16    grn8_e, grn, #6
        VQRSHRUN.S16    blu8_e, blu, #6

        /* Odd: combine luminance and chrominance. */
        VMULL.U8        y_scale, y_odd, coef_y
        VQADD.S16       red, y_scale, r_delta
        VQADD.S16       grn, y_scale, g_delta
        VQADD.S16       blu, y_scale, b_delta

        /* Odd: clamp, rescale and clip colour components to 8 bits. */
        VQRSHRUN.S16    red8_o, red, #6
        VQRSHRUN.S16    blu8_o, blu, #6
        VQRSHRUN.S16    grn8_o, grn, #6

				/*even: Q10 take rgb565 layout*/
				VSHLL.U8 	Q10, red8_e, #8
	VSHLL.U8 	Q9, grn8_e, #8
	VSRI.16   Q10, Q9, #5
	VSHLL.U8 	Q9, blu8_e, #8
	VSRI.16   Q10, Q9, #11

	/*VREV16.U8 Q10,Q10*/


				/*odd: Q11 take rgb565 layout*/
				VSHLL.U8 	Q11, red8_o, #8
	VSHLL.U8 	Q9, grn8_o, #8
	VSRI.16   Q11, Q9, #5
	VSHLL.U8 	Q9, blu8_o, #8
	VSRI.16   Q11, Q9, #11

				/*VREV16.U8 Q11,Q11*/

				MOV							r7,#16
				SUB							r7,r7,count
				AND							r7,r7,#1
				CMP							r7,#0
				BGT							rgb565_not_16_path

        /* Store 16 pixels of ARGB16, interleaving even and odd. */
        VST2.U16        {D20, D22}, [rgb_ptr]
				ADD             rgb_ptr, count, LSL #0
				VST2.U16        {D21, D23}, [rgb_ptr]
				ADD             rgb_ptr, count, LSL #0
				B								rgb565_next_loop

rgb565_not_16_path:
				MOV							r7,count
				VST2.U16        {D20, D22}, [rgb_ptr]
				SUB							r7,r7,#1
        ADD             rgb_ptr, r7, LSL #0
VST2.U16        {D21, D23}, [rgb_ptr]
        ADD							r7,r7,#2
				ADD             rgb_ptr, r7, LSL #0


rgb565_next_loop:
        /* Have we reached the end of the row yet? */
        SUBS            aligned_count, aligned_count, count

        /* On the second (and subsequent) passes through this code,
           we'll always be working on 16 pixels at once. */
        MOV             count, #16
        BGT             rgb565_loop_h_422

        /* Update pointers for new row of data. */
        /*SUB             rgb_ptr, width, LSL #2*/
        MOV							src_ptr,r8
        /*SUB             src_ptr, width, LSL #1*/
        /*ADD             rgb_ptr, rgb_pitch*/
        ADD             src_ptr, yuv_pitch

        /* Have we reached the bottom yet? */
        SUBS            height, height, #1
        BGT             rgb565_loop_v_422

        VPOP            {q4-q7}
        POP             {r4-r12, pc}
        .fnend


#endif /* __ARM_NEON__ */
