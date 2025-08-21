/* Core & memory map bases */
.equ FLASH_BASE,        0x08000000
.equ SRAM1_BASE,        0x20000000
.equ PERIPH_BASE,       0x40000000

/* Bus peripheral base addresses */
.equ PERIPH_BASE_APB1,  (PERIPH_BASE + 0x00000)    /* 0x40000000 */
.equ PERIPH_BASE_APB2,  (PERIPH_BASE + 0x10000)    /* 0x40010000 */
.equ PERIPH_BASE_AHB1,  (PERIPH_BASE + 0x20000)    /* 0x40020000 */
.equ PERIPH_BASE_AHB2,  0x50000000                /* external/periph (USB OTG FS) */
.equ PERIPH_BB_BASE,    0x02000000

/* Cortex-M core peripherals (system) */
.equ SCS_BASE,          0xE000E000
.equ SCB_BASE,          0xE000ED00
.equ NVIC_ISER0,        0xE000E100
.equ NVIC_ICER0,        0xE000E180

/* ------------------------------------------------------------------------- */
/* AHB1 peripherals (full list for STM32F411xC) */
.equ GPIOA_BASE,        (PERIPH_BASE_AHB1 + 0x0000)    /* 0x40020000 */
.equ GPIOB_BASE,        (PERIPH_BASE_AHB1 + 0x0400)    /* 0x40020400 */
.equ GPIOC_BASE,        (PERIPH_BASE_AHB1 + 0x0800)    /* 0x40020800 */
.equ GPIOD_BASE,        (PERIPH_BASE_AHB1 + 0x0C00)    /* 0x40020C00 */
.equ GPIOE_BASE,        (PERIPH_BASE_AHB1 + 0x1000)    /* 0x40021000 */
.equ GPIOF_BASE,        (PERIPH_BASE_AHB1 + 0x1400)    /* 0x40021400 */
.equ GPIOG_BASE,        (PERIPH_BASE_AHB1 + 0x1800)    /* 0x40021800 */
.equ GPIOH_BASE,        (PERIPH_BASE_AHB1 + 0x1C00)    /* 0x40021C00 */

.equ CRC_BASE,          (PERIPH_BASE_AHB1 + 0x2300)    /* 0x40022300 */
.equ RCC_BASE,          (PERIPH_BASE_AHB1 + 0x3800)    /* 0x40023800 */
.equ FLASH_R_BASE,      (PERIPH_BASE_AHB1 + 0x3C00)    /* Option regs region (some families) */
.equ DMA1_BASE,         (PERIPH_BASE_AHB1 + 0x6000)    /* 0x40026000 */
.equ DMA2_BASE,         (PERIPH_BASE_AHB1 + 0x6400)    /* 0x40026400 */

/* AHB2 peripherals */
.equ USB_OTG_FS_BASE,   (PERIPH_BASE_AHB2 + 0x0000)    /* 0x50000000 */

/* APB1 peripherals */
.equ TIM2_BASE,         (PERIPH_BASE_APB1 + 0x0000)    /* 0x40000000 */
.equ TIM3_BASE,         (PERIPH_BASE_APB1 + 0x0400)    /* 0x40000400 */
.equ TIM4_BASE,         (PERIPH_BASE_APB1 + 0x0800)    /* 0x40000800 */
.equ TIM5_BASE,         (PERIPH_BASE_APB1 + 0x0C00)    /* 0x40000C00 */
.equ TIM6_BASE,         (PERIPH_BASE_APB1 + 0x1000)    /* 0x40001000 */
.equ TIM7_BASE,         (PERIPH_BASE_APB1 + 0x1400)    /* 0x40001400 */
.equ TIM12_BASE,        (PERIPH_BASE_APB1 + 0x1800)    /* 0x40001800 */
.equ TIM13_BASE,        (PERIPH_BASE_APB1 + 0x1C00)    /* 0x40001C00 */
.equ TIM14_BASE,        (PERIPH_BASE_APB1 + 0x2000)    /* 0x40002000 */

.equ RTC_BASE,          (PERIPH_BASE_APB1 + 0x2800)    /* backup/rtc area (through PWR/BKP) */

.equ WWDG_BASE,         (PERIPH_BASE_APB1 + 0x2C00)
.equ IWDG_BASE,         (PERIPH_BASE + 0x30003000)     /* independent watchdog (implementation dependent) */

.equ I2C1_BASE,         (PERIPH_BASE_APB1 + 0x5400)    /* 0x40005400 */
.equ I2C2_BASE,         (PERIPH_BASE_APB1 + 0x5800)    /* 0x40005800 */
.equ I2C3_BASE,         (PERIPH_BASE_APB1 + 0x5C00)    /* 0x40005C00 */

.equ CAN1_BASE,         (PERIPH_BASE_APB1 + 0x6400)    /* CAN not present on F411 variants but kept for compatibility */

.equ PWR_BASE,          (PERIPH_BASE_APB1 + 0x7000)

.equ SPI2_BASE,         (PERIPH_BASE_APB1 + 0x3800)    /* 0x40003800 */
.equ SPI3_BASE,         (PERIPH_BASE_APB1 + 0x3C00)    /* 0x40003C00 */

.equ USART2_BASE,       (PERIPH_BASE_APB1 + 0x4400)    /* 0x40004400 */
.equ USART3_BASE,       (PERIPH_BASE_APB1 + 0x4800)    /* 0x40004800 */
.equ UART4_BASE,        (PERIPH_BASE_APB1 + 0x4C00)    /* 0x40004C00 */
.equ UART5_BASE,        (PERIPH_BASE_APB1 + 0x5000)    /* 0x40005000 */

/* APB2 peripherals */
.equ TIM1_BASE,         (PERIPH_BASE_APB2 + 0x0000)    /* 0x40010000 */
.equ TIM8_BASE,         (PERIPH_BASE_APB2 + 0x0400)    /* 0x40010400 */
.equ USART1_BASE,       (PERIPH_BASE_APB2 + 0x1000)    /* 0x40011000 */
.equ USART6_BASE,       (PERIPH_BASE_APB2 + 0x1400)    /* 0x40011400 */
.equ ADC1_BASE,         (PERIPH_BASE_APB2 + 0x1200)    /* 0x40012000 (family dependent) */
.equ SDIO_BASE,         (PERIPH_BASE_APB2 + 0x2C00)
.equ SPI1_BASE,         (PERIPH_BASE_APB2 + 0x3000)    /* 0x40013000 */
.equ SYSCFG_BASE,       (PERIPH_BASE_APB2 + 0x3800)    /* 0x40013800 */

/* ------------------------------------------------------------------------- */
/* Generic register offsets (common layouts per peripheral family) */

/* ---------------- GPIO ---------------- */
.equ GPIO_MODER,   0x00
.equ GPIO_OTYPER,  0x04
.equ GPIO_OSPEEDR, 0x08
.equ GPIO_PUPDR,   0x0C
.equ GPIO_IDR,     0x10
.equ GPIO_ODR,     0x14
.equ GPIO_BSRR,    0x18
.equ GPIO_LCKR,    0x1C
.equ GPIO_AFRL,    0x20
.equ GPIO_AFRH,    0x24

/* ================= GPIOA ================= */
.equ GPIOA_MODER,   (GPIOA_BASE + GPIO_MODER)
.equ GPIOA_OTYPER,  (GPIOA_BASE + GPIO_OTYPER)
.equ GPIOA_OSPEEDR, (GPIOA_BASE + GPIO_OSPEEDR)
.equ GPIOA_PUPDR,   (GPIOA_BASE + GPIO_PUPDR)
.equ GPIOA_IDR,     (GPIOA_BASE + GPIO_IDR)
.equ GPIOA_ODR,     (GPIOA_BASE + GPIO_ODR)
.equ GPIOA_BSRR,    (GPIOA_BASE + GPIO_BSRR)
.equ GPIOA_LCKR,    (GPIOA_BASE + GPIO_LCKR)
.equ GPIOA_AFRL,    (GPIOA_BASE + GPIO_AFRL)
.equ GPIOA_AFRH,    (GPIOA_BASE + GPIO_AFRH)

/* ================= GPIOB ================= */
.equ GPIOB_MODER,   (GPIOB_BASE + GPIO_MODER)
.equ GPIOB_OTYPER,  (GPIOB_BASE + GPIO_OTYPER)
.equ GPIOB_OSPEEDR, (GPIOB_BASE + GPIO_OSPEEDR)
.equ GPIOB_PUPDR,   (GPIOB_BASE + GPIO_PUPDR)
.equ GPIOB_IDR,     (GPIOB_BASE + GPIO_IDR)
.equ GPIOB_ODR,     (GPIOB_BASE + GPIO_ODR)
.equ GPIOB_BSRR,    (GPIOB_BASE + GPIO_BSRR)
.equ GPIOB_LCKR,    (GPIOB_BASE + GPIO_LCKR)
.equ GPIOB_AFRL,    (GPIOB_BASE + GPIO_AFRL)
.equ GPIOB_AFRH,    (GPIOB_BASE + GPIO_AFRH)

/* ================= GPIOC ================= */
.equ GPIOC_MODER,   (GPIOC_BASE + GPIO_MODER)
.equ GPIOC_OTYPER,  (GPIOC_BASE + GPIO_OTYPER)
.equ GPIOC_OSPEEDR, (GPIOC_BASE + GPIO_OSPEEDR)
.equ GPIOC_PUPDR,   (GPIOC_BASE + GPIO_PUPDR)
.equ GPIOC_IDR,     (GPIOC_BASE + GPIO_IDR)
.equ GPIOC_ODR,     (GPIOC_BASE + GPIO_ODR)
.equ GPIOC_BSRR,    (GPIOC_BASE + GPIO_BSRR)
.equ GPIOC_LCKR,    (GPIOC_BASE + GPIO_LCKR)
.equ GPIOC_AFRL,    (GPIOC_BASE + GPIO_AFRL)
.equ GPIOC_AFRH,    (GPIOC_BASE + GPIO_AFRH)

/* ================= GPIOD ================= */
.equ GPIOD_MODER,   (GPIOD_BASE + GPIO_MODER)
.equ GPIOD_OTYPER,  (GPIOD_BASE + GPIO_OTYPER)
.equ GPIOD_OSPEEDR, (GPIOD_BASE + GPIO_OSPEEDR)
.equ GPIOD_PUPDR,   (GPIOD_BASE + GPIO_PUPDR)
.equ GPIOD_IDR,     (GPIOD_BASE + GPIO_IDR)
.equ GPIOD_ODR,     (GPIOD_BASE + GPIO_ODR)
.equ GPIOD_BSRR,    (GPIOD_BASE + GPIO_BSRR)
.equ GPIOD_LCKR,    (GPIOD_BASE + GPIO_LCKR)
.equ GPIOD_AFRL,    (GPIOD_BASE + GPIO_AFRL)
.equ GPIOD_AFRH,    (GPIOD_BASE + GPIO_AFRH)

/* ================= GPIOE ================= */
.equ GPIOE_MODER,   (GPIOE_BASE + GPIO_MODER)
.equ GPIOE_OTYPER,  (GPIOE_BASE + GPIO_OTYPER)
.equ GPIOE_OSPEEDR, (GPIOE_BASE + GPIO_OSPEEDR)
.equ GPIOE_PUPDR,   (GPIOE_BASE + GPIO_PUPDR)
.equ GPIOE_IDR,     (GPIOE_BASE + GPIO_IDR)
.equ GPIOE_ODR,     (GPIOE_BASE + GPIO_ODR)
.equ GPIOE_BSRR,    (GPIOE_BASE + GPIO_BSRR)
.equ GPIOE_LCKR,    (GPIOE_BASE + GPIO_LCKR)
.equ GPIOE_AFRL,    (GPIOE_BASE + GPIO_AFRL)
.equ GPIOE_AFRH,    (GPIOE_BASE + GPIO_AFRH)

/* ================= GPIOH ================= */
.equ GPIOH_MODER,   (GPIOH_BASE + GPIO_MODER)
.equ GPIOH_OTYPER,  (GPIOH_BASE + GPIO_OTYPER)
.equ GPIOH_OSPEEDR, (GPIOH_BASE + GPIO_OSPEEDR)
.equ GPIOH_PUPDR,   (GPIOH_BASE + GPIO_PUPDR)
.equ GPIOH_IDR,     (GPIOH_BASE + GPIO_IDR)
.equ GPIOH_ODR,     (GPIOH_BASE + GPIO_ODR)
.equ GPIOH_BSRR,    (GPIOH_BASE + GPIO_BSRR)
.equ GPIOH_LCKR,    (GPIOH_BASE + GPIO_LCKR)
.equ GPIOH_AFRL,    (GPIOH_BASE + GPIO_AFRL)
.equ GPIOH_AFRH,    (GPIOH_BASE + GPIO_AFRH)


/* ---------------- RCC ---------------- */
.equ RCC_CR,           0x00
.equ RCC_PLLCFGR,      0x04
.equ RCC_CFGR,         0x08
.equ RCC_CIR,          0x0C
.equ RCC_AHB1RSTR,     0x10
.equ RCC_AHB2RSTR,     0x14
.equ RCC_AHB3RSTR,     0x18
.equ RCC_APB1RSTR,     0x20
.equ RCC_APB2RSTR,     0x24
.equ RCC_AHB1ENR,      0x30
.equ RCC_AHB2ENR,      0x34
.equ RCC_AHB3ENR,      0x38
.equ RCC_APB1ENR,      0x40
.equ RCC_APB2ENR,      0x44
.equ RCC_AHB1LPENR,    0x50
.equ RCC_AHB2LPENR,    0x54
.equ RCC_AHB3LPENR,    0x58
.equ RCC_APB1LPENR,    0x60
.equ RCC_APB2LPENR,    0x64
.equ RCC_BDCR,         0x70
.equ RCC_CSR,          0x74
.equ RCC_SSCGR,        0x80
.equ RCC_PLLI2SCFGR,   0x84

.equ RCC_CR_ADDR,      (RCC_BASE + RCC_CR)
.equ RCC_AHB1ENR_ADDR, (RCC_BASE + RCC_AHB1ENR)

/* ---------------- EXTI ---------------- */
.equ EXTI_BASE,        (PERIPH_BASE_APB2 + 0x3C00)    /* 0x40013C00 */
.equ EXTI_IMR,         0x00
.equ EXTI_EMR,         0x04
.equ EXTI_RTSR,        0x08
.equ EXTI_FTSR,        0x0C
.equ EXTI_SWIER,       0x10
.equ EXTI_PR,          0x14

.equ EXTI_IMR_ADDR,    (EXTI_BASE + EXTI_IMR)

/* ---------------- SYSCFG ---------------- */
.equ SYSCFG_MEMRMP,    0x00
.equ SYSCFG_PMC,       0x04
.equ SYSCFG_EXTICR1,   0x08
.equ SYSCFG_EXTICR2,   0x0C
.equ SYSCFG_EXTICR3,   0x10
.equ SYSCFG_EXTICR4,   0x14
.equ SYSCFG_CMPCR,     0x20

/* ---------------- TIM (general) ---------------- */
.equ TIM_CR1,    0x00
.equ TIM_CR2,    0x04
.equ TIM_SMCR,   0x08
.equ TIM_DIER,   0x0C
.equ TIM_SR,     0x10
.equ TIM_EGR,    0x14
.equ TIM_CCMR1,  0x18
.equ TIM_CCMR2,  0x1C
.equ TIM_CCER,   0x20
.equ TIM_CNT,    0x24
.equ TIM_PSC,    0x28
.equ TIM_ARR,    0x2C
.equ TIM_CCR1,   0x34
.equ TIM_CCR2,   0x38
.equ TIM_CCR3,   0x3C
.equ TIM_CCR4,   0x40
.equ TIM_BDTR,   0x44    /* advanced timers only (TIM1/TIM8) */

/* TIM1 convenience addresses */
.equ TIM1_CR1_ADDR, (TIM1_BASE + TIM_CR1)
.equ TIM2_CR1_ADDR, (TIM2_BASE + TIM_CR1)

/* ---------------- USART ---------------- */
.equ USART_SR,   0x00
.equ USART_DR,   0x04
.equ USART_BRR,  0x08
.equ USART_CR1,  0x0C
.equ USART_CR2,  0x10
.equ USART_CR3,  0x14
.equ USART_GTPR, 0x18

.equ USART1_SR_ADDR, (USART1_BASE + USART_SR)
.equ USART2_SR_ADDR, (USART2_BASE + USART_SR)

/* ---------------- SPI ---------------- */
.equ SPI_CR1,    0x00
.equ SPI_CR2,    0x04
.equ SPI_SR,     0x08
.equ SPI_DR,     0x0C
.equ SPI_CRCPR,  0x10
.equ SPI_RXCRCR, 0x14
.equ SPI_TXCRCR, 0x18

/* ---------------- I2C ---------------- */
.equ I2C_CR1,    0x00
.equ I2C_CR2,    0x04
.equ I2C_OAR1,   0x08
.equ I2C_OAR2,   0x0C
.equ I2C_DR,     0x10
.equ I2C_SR1,    0x14
.equ I2C_SR2,    0x18
.equ I2C_CCR,    0x1C
.equ I2C_TRISE,  0x20

/* ---------------- ADC ---------------- */
.equ ADC_SR,     0x00
.equ ADC_CR1,    0x04
.equ ADC_CR2,    0x08
.equ ADC_SMPR1,  0x0C
.equ ADC_SMPR2,  0x10
.equ ADC_JOFR1,  0x14
.equ ADC_JOFR2,  0x18
.equ ADC_JOFR3,  0x1C
.equ ADC_JOFR4,  0x20
.equ ADC_HTR,    0x24
.equ ADC_LTR,    0x28
.equ ADC_SQR1,   0x2C
.equ ADC_SQR2,   0x30
.equ ADC_SQR3,   0x34
.equ ADC_JSQR,   0x38
.equ ADC_JDR1,   0x3C
.equ ADC_JDR2,   0x40
.equ ADC_JDR3,   0x44
.equ ADC_JDR4,   0x48
.equ ADC_DR,     0x4C

/* ADC convenience */
.equ ADC1_SR_ADDR, (ADC1_BASE + ADC_SR)
.equ ADC1_DR_ADDR, (ADC1_BASE + ADC_DR)

/* ---------------- DMA (controllers and streams) ---------------- */
.equ DMA_LISR,   0x00
.equ DMA_HISR,   0x04
.equ DMA_LIFCR,  0x08
.equ DMA_HIFCR,  0x0C

/* Stream registers start at offset 0x10 + stream_index*0x18 */
.equ DMA_SxCR,    0x10
.equ DMA_SxNDTR,  0x14
.equ DMA_SxPAR,   0x18
.equ DMA_SxM0AR,  0x1C
.equ DMA_SxM1AR,  0x20
.equ DMA_SxFCR,   0x24

/* DMA1 stream addresses (streams 0..7) */
.equ DMA1_S0_CR,  (DMA1_BASE + 0x10 + (0x18 * 0))
.equ DMA1_S1_CR,  (DMA1_BASE + 0x10 + (0x18 * 1))
.equ DMA1_S2_CR,  (DMA1_BASE + 0x10 + (0x18 * 2))
.equ DMA1_S3_CR,  (DMA1_BASE + 0x10 + (0x18 * 3))
.equ DMA1_S4_CR,  (DMA1_BASE + 0x10 + (0x18 * 4))
.equ DMA1_S5_CR,  (DMA1_BASE + 0x10 + (0x18 * 5))
.equ DMA1_S6_CR,  (DMA1_BASE + 0x10 + (0x18 * 6))
.equ DMA1_S7_CR,  (DMA1_BASE + 0x10 + (0x18 * 7))

/* ---------------- CRC ---------------- */
.equ CRC_DR,      0x00
.equ CRC_IDR,     0x04
.equ CRC_CR_REG,  0x08
.equ CRC_DR_ADDR, (CRC_BASE + CRC_DR)

/* ---------------- USB OTG FS (high-level offsets) ---------------- */
/* Full USB register map is large; provide base and key offsets.*/
.equ USB_OTG_GOTGCTL,  0x000
.equ USB_OTG_GOTGINT,  0x004
.equ USB_OTG_GAHBCSR,  0x008
.equ USB_OTG_GINTSTS,  0x014
.equ USB_OTG_GINTMSK,  0x018

/* ---------------- Power / Backup ---------------- */
.equ PWR_CR,       0x00
.equ PWR_CSR,      0x04
.equ PWR_CR_ADDR,  (PWR_BASE + PWR_CR)

/* ---------------- FLASH (program/option) ---------------- */
.equ FLASH_ACR,    0x00
.equ FLASH_KEYR,   0x04
.equ FLASH_OPTKEYR,0x08
.equ FLASH_SR,     0x0C
.equ FLASH_CR,     0x10
.equ FLASH_OPTCR,  0x14

/* ---------------- System control / SCB ---------------- */
.equ SCB_CPUID,    (SCB_BASE + 0x00)
.equ SCB_ICSR,     (SCB_BASE + 0x04)
.equ SCB_VTOR,     (SCB_BASE + 0x08)
.equ SCB_AIRCR,    (SCB_BASE + 0x0C)
.equ SCB_SCR,      (SCB_BASE + 0x10)
.equ SCB_CCR,      (SCB_BASE + 0x14)

/* ---------------- Convenience: common full addresses examples --------------- */
.equ SPI1_CR1_ADDR, (SPI1_BASE + SPI_CR1)
.equ SPI2_CR1_ADDR, (SPI2_BASE + SPI_CR1)
.equ I2C1_CR1_ADDR, (I2C1_BASE + I2C_CR1)
.equ USART1_CR1_ADDR,(USART1_BASE + USART_CR1)
