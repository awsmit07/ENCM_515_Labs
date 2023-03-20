@ static int16_t ProcessSample(int16_t newsample, int16_t* history) {
ProcessSample:
    push    {r7}
    sub     sp, #28
    add     r7, sp, #0
    mov     r3, r0
    str     r1, [r7, #0]
    strh    r3, [r7, #6]
@ history[0] = newsample;
    ldr     r3, [r7, #0]
    ldrh    r2, [r7, #6]
    strh    r2, [r3, #0]
@ int tap = 0;
    movs    r3, #0
    str     r3, [r7, #20]
@ int32_t accumulator = 0;
    movs    r3, #0
    str     r3, [r7, #16]
@ for (tap = 0; tap < NUMBER_OF_TAPS; tap++) {
    movs    r3, #0
    str     r3, [r7, #20]
    b.n     0x80012d2 <ProcessSample+70>
@ accumulator += (int32_t)filter_coeffs[tap] * (int32_t)history[tap];
    ldr     r2, [pc, #156]  ; (0x800134c <ProcessSample+192>)
    ldr     r3, [r7, #20]
    ldrsh.w r3, [r2, r3, lsl #1]
    mov     r1, r3
    ldr     r3, [r7, #20]
    lsls    r3, r3, #1
    ldr     r2, [r7, #0]
    add     r3, r2
    ldrsh.w r3, [r3]
    mul.w   r3, r1, r3
    ldr     r2, [r7, #16]
    add     r3, r2
    str     r3, [r7, #16]
@ for (tap = 0; tap < NUMBER_OF_TAPS; tap++) {
    ldr     r3, [r7, #20]
    adds    r3, #1
    str     r3, [r7, #20]
    ldr     r3, [r7, #20]
    cmp     r3, #219        ; 0xdb
    ble.n   0x80012ac <ProcessSample+32>
@ for(tap = NUMBER_OF_TAPS-2; tap > -1; tap--) {
    movs    r3, #218        ; 0xda
    str     r3, [r7, #20]
    b.n     0x80012fc <ProcessSample+112>
@ history[tap+1] = history[tap];
    ldr     r3, [r7, #20]
    lsls    r3, r3, #1
    ldr     r2, [r7, #0]
    add     r2, r3
    ldr     r3, [r7, #20]int
    adds    r3, #1
    lsls    r3, r3, #1
    ldr     r1, [r7, #0]
    add     r3, r1
    ldrsh.w r2, [r2]
    strh    r2, [r3, #0]
@ for(tap = NUMBER_OF_TAPS-2; tap > -1; tap--) {
    ldr     r3, [r7, #20]
    subs    r3, #1
    str     r3, [r7, #20]
    ldr     r3, [r7, #20]
    cmp     r3, #0
    bge.n   0x80012de <ProcessSample+82>
@ if (accumulator > 0x3FFFFFFF) {
    ldr     r3, [r7, #16]
    cmp.w   r3, #1073741824 ; 0x40000000
    blt.n   0x800131c <ProcessSample+144>
@ accumulator = 0x3FFFFFFF;
    mvn.w   r3, #3221225472 ; 0xc0000000
    str     r3, [r7, #16]
@ overflow_count++;
    ldr     r3, [pc, #60]   ; (0x8001350 <ProcessSample+196>)
    ldr     r3, [r3, #0]
    adds    r3, #1
    ldr     r2, [pc, #56]   ; (0x8001350 <ProcessSample+196>)
    str     r3, [r2, #0]
    b.n     0x8001334 <ProcessSample+168>
@ } else if (accumulator < -0x40000000) {
    ldr     r3, [r7, #16]
    cmp.w   r3, #3221225472 ; 0xc0000000
    bge.n   0x8001334 <ProcessSample+168>
@ accumulator = -0x40000000;
    mov.w   r3, #3221225472 ; 0xc0000000
    str     r3, [r7, #16]
@ underflow_count++;
    ldr     r3, [pc, #40]   ; (0x8001354 <ProcessSample+200>)
    ldr     r3, [r3, #0]
    adds    r3, #1
    ldr     r2, [pc, #32]   ; (0x8001354 <ProcessSample+200>)
    str     r3, [r2, #0]
@ int16_t temp = (int16_t)(accumulator >> 15);
    ldr     r3, [r7, #16]
    asrs    r3, r3, #15
    strh    r3, [r7, #14]
@ return temp;
    ldrsh.w r3, [r7, #14]
@ }