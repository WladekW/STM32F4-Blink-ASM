.syntax unified
.thumb
.cpu cortex-m4

.global asm_main
.type asm_main, %function

.include "src/registers.s"

asm_main:

  @RCC ports enabling
    LDR R1, = RCC_BASE + RCC_AHB1ENR
    LDR R0, [R1]
    ORR R0, R0, #( 1 << 1 ) 
    STR R0, [R1]

    LDR R1, = RCC_BASE + RCC_APB1ENR
    LDR R0, [R1]
    ORR R0, R0, #1
    STR R0, [R1]

  @GPIO config
    LDR	R1, = GPIOB_BASE + GPIO_MODER
    LDR R0, [R1]
    BIC R0, R0, #( 3 << 20 )
    ORR R0, R0, #( 1 << 20 )
    STR R0, [R1] 
    
  @TIM2 config
    LDR R1, = TIM2_BASE + TIM_PSC
    MOV R0, #( 16000 - 1 )
    STR R0, [R1]

    LDR R1, = TIM2_BASE + TIM_ARR
    MOV R0, #( 1000 - 1)
    STR R0, [R1]

    LDR R1, = TIM2_BASE + TIM_CR1
    LDR R0, [R1]
    ORR R0, R0, #( 1 << 3 )
    STR R0, [R1]

  @REGS thats shouldn't be changed in the loop
    LDR	R4, = GPIOB_BASE + GPIO_BSRR
loop:
    
    MOV R0, #( 1 << 10 )
    STR R0, [R4] 

    BL delay

    MOV R0, #( 1 << 26 )
    STR R0, [R4] 

    BL delay

    B loop

delay:
    LDR R1, = TIM2_BASE + TIM_SR
    MOV R0, #0
    STR R0, [R1]

    LDR R1, = TIM2_BASE + TIM_CR1
    LDR R0, [R1]
    ORR R0, R0, #1
    STR R0, [R1]

delay_loop:
    LDR R1, = TIM2_BASE + TIM_SR
    LDR R0, [R1]
    TST R0, #1
    BEQ delay_loop
    
    BX LR
