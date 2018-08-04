	# 15-tap halfband FIR filter for 2:1 decimation
	# June 2018, Phil Karn, KA9Q
	# prototype:
	# float hb15(float even_samples[4],float odd_samples[4],float state[4],float coefficients[4])
	# Write sample 0 to even_samples[0], then sample 1 to odd_samples[0], then execute to get filter
	# output after sample 1. Then repeat with samples 2 and 3 etc
	
	# Assumes middle tap coefficient = 1
	# repeated execution without overwriting even_samples[0] and odd_samples[0] each time gives undefined results
	# after execution the input fields contain internal temp values and are not overwritten to save time

	.file	"foo.c"
	.text
	.p2align 4,,15
	.globl	hb15
hb15:
.LFB4:
	.cfi_startproc
	movups	(%rsi), %xmm1   # newer odd states (second arg)
	movups	(%rdx), %xmm3   # older odd states (third arg)
	movaps  %xmm3, %xmm0
	addps	%xmm1, %xmm0    # sum of newer and older states
	mulps	(%rcx), %xmm0   # coefficients (fourth arg)
	haddps  %xmm0, %xmm0
	haddps  %xmm0, %xmm0    # sum of four temp terms
	movups  (%rdi), %xmm2   # even states(first arg), word we want is in [3]
	# left rotate it so [3] goes to [0] and also save for next time	
	# 2 1 0 3,  10 01 00 11 = 0x93
	shufps  $0x93,%xmm2,%xmm2
	addss   %xmm2,%xmm0     # our final result in low word of xmm0
	movups  %xmm2,(%rdi)    # save updated new_odd state
	shufps  $0x39,%xmm3,%xmm3  # right shift older odd state X 3 2 1 = 00 11 10 01 = 0x39
	# copy high word 3 from newer odd state to older odd state word 3
	# from word 3 to word 3, don't zero anything 11 11   0 0 0 0 = 0xf0
	insertps $0xf0,%xmm1,%xmm3
	movups  %xmm3,(%rdx)    # save older odd state
	# left shift newer odd state 1 word
	shufps  $0x90,%xmm1,%xmm1   # 1001 0000    2 1 0 X = 0x90
	movups  %xmm1,(%rsi)    # save newer odd state
	ret
	.cfi_endproc
.LFE4:
	.ident	"KA9Q: 15-tap halfband filter "
