extern void asm_main(void);  // funkcja w ASM

int main(void) {
    asm_main();
    while(1); // Nigdy nie wychodzimy
}