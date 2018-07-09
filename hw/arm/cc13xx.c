/*
 * TI CC13XX
 *
 * Copyright (c) 2018 Beuth Hochschule für Technik Berlin.
 * Written by Marco Hermann and Benjamin Bimmmermann
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "net/net.h"
#include "hw/boards.h"
#include "qemu/log.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/char/pl011.h"
#include "hw/misc/unimp.h"
#include "cpu.h"

#define FLASH_SIZE_KB 128
#define SRAM_SIZE_KB 20
#define KB_TO_MB 1024
#define FLASH_BASE_ADRESS 0x00000000
#define SRAM_BASE_ADRESS 0x20000000

#define BP_OLED_I2C  0x01
#define NUM_IRQ_LINES 64

typedef const struct {
    const char *name;
    uint32_t did0;
    uint32_t did1;
    uint32_t dc0;
    uint32_t dc1;
    uint32_t dc2;
    uint32_t dc3;
    uint32_t dc4;
    uint32_t peripherals;
} cc13xx_board_info;

/* General purpose timer module.  */

#define TYPE_CC13XX_GPTM "cc13xx-gptm"
#define CC13XX_GPTM(obj) \
    OBJECT_CHECK(gptm_state, (obj), TYPE_CC13XX_GPTM)

typedef struct gptm_state {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t config;
    uint32_t mode[2];
    uint32_t control;
    uint32_t state;
    uint32_t mask;
    uint32_t load[2];
    uint32_t match[2];
    uint32_t prescale[2];
    uint32_t match_prescale[2];
    uint32_t rtc;
    int64_t tick[2];
    struct gptm_state *opaque[2];
    QEMUTimer *timer[2];
    /* The timers have an alternate output used to trigger the ADC.  */
    qemu_irq trigger;
    qemu_irq irq;
} gptm_state;



/* System controller.  */

typedef struct {
    MemoryRegion iomem;
    uint32_t pborctl;
    uint32_t ldopctl;
    uint32_t int_status;
    uint32_t int_mask;
    uint32_t resc;
    uint32_t rcc;
    uint32_t rcc2;
    uint32_t rcgc[3];
    uint32_t scgc[3];
    uint32_t dcgc[3];
    uint32_t clkvclr;
    uint32_t ldoarst;
    uint32_t user0;
    uint32_t user1;
    qemu_irq irq;
    cc13xx_board_info *board;
} ssys_state;




#define DID0_VER_MASK        0x70000000
#define DID0_VER_0           0x00000000
#define DID0_VER_1           0x10000000

#define DID0_CLASS_MASK      0x00FF0000
#define DID0_CLASS_SANDSTORM 0x00000000
#define DID0_CLASS_FURY      0x00010000





/* Board init.  */
static cc13xx_board_info cc13xx_boards[] = {
  { "CC1310",
    0,          // did0
    0x0032000e, // did1
    0x001f001f, // dc0
    0x001132bf, // dc1
    0x01071013, // dc2
    0x3f0f01ff, // dc3
    0x0000001f, // dc4
    BP_OLED_I2C // peripherals
  }
};

static void cc13xx_init(MachineState *ms, cc13xx_board_info *board)
{
    //spaeter
    //static const int uart_irq[] = {5, 6, 33, 34};
    //static const int timer_irq[] = {19, 21, 23, 35};
    //static const uint32_t gpio_addr[7] =
    //  { 0x40004000, 0x40005000, 0x40006000, 0x40007000,
    //    0x40024000, 0x40025000, 0x40026000};
    //static const int gpio_irq[7] = {0, 1, 2, 3, 4, 30, 31};

    /* Memory map of SoC devices, from
     * CC13XX Technical Reference Manual - 3.2.8
     * http://www.ti.com/lit/ug/swcu117h/swcu117h.pdf
     *
     * 00000000 flashmem
     * 10000000 BROM
     * 11000000 GPRAM
     * 20000000 SRAM
     * 21000000 RFC_RAM
     * 40000000 SSI0
     * 40001000 UART0
     * 40002000 i2c0
     * 40008000 SSI1
     * 40010000 GPTO
     * 40011000 GPT1
     * 40012000 GPT2
     * 40013000 GPT3
     * 40020000 UDMA0
     * 40021000 I2SO
     * 40022000 GPIO
     * 40024000 CRYPTO
     * 40028000 TRNG
     * 40030000 FLASH
     * 40040000 RFC_PWR
     * 40041000 RFC_DBELL
     * 40043000 RFC_RAT
     * 40044000 RFC_FSCA
     * 40080000 WDT
     * 40081000 IOC
     * 50001000 FCFG1
     * 50002000 FCFG2
     * 50003000 CCFG
     */

    //DeviceState *gpio_dev[7], *nvic;
    //qemu_irq gpio_in[7][8];
    //qemu_irq gpio_out[7][8];
    //qemu_irq adc;
    int sram_size;
    int flash_size;
    //I2CBus *i2c;
    //DeviceState *dev;
    //int i;
    //int j;



    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *system_memory = get_system_memory();
    qemu_irq pic = 0;

    flash_size = (FLASH_SIZE_KB * KB_TO_MB); // 128 KB
    sram_size = (SRAM_SIZE_KB * KB_TO_MB);  // 20 KB

    /* Flash programming is done via the SCU, so pretend it is ROM.  */
    memory_region_init_ram(flash, NULL, "cc1310xx.flash", flash_size,
                           &error_fatal);
    memory_region_set_readonly(flash, true);
    memory_region_add_subregion(system_memory, FLASH_BASE_ADRESS, flash);

    memory_region_init_ram(sram, NULL, "cc13xx.sram", sram_size,
                           &error_fatal);
    memory_region_add_subregion(system_memory, SRAM_BASE_ADRESS, sram);

    armv7m_init(system_memory, flash_size, NUM_IRQ_LINES,
                       ms->kernel_filename, ms->cpu_type);

    /* UART0 */
    pl011_create(0x40001000,pic,serial_hds[0]);


    //sysbus_connect_irq(s, 0, irq);
                    //did1
 //   cc13xx_sys_init(0x400fe000, qdev_get_gpio_in(nvic, 28),
 //                      board, nd_table[0].macaddr.a);


/*
    if (board->dc2 & (1 << 12)) {
        dev = sysbus_create_simple(TYPE_STELLARIS_I2C, 0x40020000,
                                   qdev_get_gpio_in(nvic, 8));
        i2c = (I2CBus *)qdev_get_child_bus(dev, "i2c");
        if (board->peripherals & BP_OLED_I2C) {
            i2c_create_slave(i2c, "ssd0303", 0x3d);
        }
    }
*/


    /*
    if (board->dc4 & (1 << 28)) {
        DeviceState *enet;

        qemu_check_nic_model(&nd_table[0], "cc13xx");

        enet = qdev_create(NULL, "cc13xx_enet");
        qdev_set_nic_properties(enet, &nd_table[0]);
        qdev_init_nofail(enet);
        sysbus_mmio_map(SYS_BUS_DEVICE(enet), 0, 0x40048000);
        sysbus_connect_irq(SYS_BUS_DEVICE(enet), 0, qdev_get_gpio_in(nvic, 42));
    }
    */



    /* Add dummy regions for the devices we don't implement yet,
     * so guest accesses don't cause unlogged crashes.
     */
    create_unimplemented_device("flashmem", 0x00000000, 0x1000);
    create_unimplemented_device("BROM", 0x10000000, 0x1000);
    create_unimplemented_device("GPRAM", 0x11000000, 0x1000);
    create_unimplemented_device("SRAM", 0x20000000, 0x1000);
    create_unimplemented_device("RFC_RAM", 0x21000000, 0x1000);
    create_unimplemented_device("SSI0", 0x40000000, 0x1000);
    //create_unimplemented_device("UART0", 0x40001000, 0x1000);
    create_unimplemented_device("i2c0", 0x40002000, 0x1000);
    create_unimplemented_device("SSI1",40008000, 0x1000);
    create_unimplemented_device("GPT0", 0x40010000, 0x1000);
    create_unimplemented_device("GPT1", 0x40011000, 0x1000);
    create_unimplemented_device("GPT2", 0x40012000, 0x1000);
    create_unimplemented_device("GPT3", 400130000, 0x1000);
    create_unimplemented_device("UDMA0", 0x40020000, 0x1000);
    create_unimplemented_device("I2S0", 0x4002100, 0x1000);
    create_unimplemented_device("GPIO", 0x40022000, 0x1000);
    create_unimplemented_device("CRYPTO", 0x40024000, 0x1000);
    create_unimplemented_device("TRNG", 0x40028000, 0x1000);
    create_unimplemented_device("FLASH", 0x40030000, 0x1000);
    create_unimplemented_device("RFC_PWR", 0x40040000, 0x1000);
    create_unimplemented_device("RFC_DBELL", 0x40041000, 0x1000);
    create_unimplemented_device("RFC_RAT", 0x40043000, 0x1000);
    create_unimplemented_device("RFC_FSCA", 0x40044000, 0x1000);
    create_unimplemented_device("WDT", 0x40080000, 0x1000);
    create_unimplemented_device("IOC", 0x40081000, 0x1000);
    create_unimplemented_device("FCFG1", 0x50001000, 0x1000);
    create_unimplemented_device("FCFG2", 0x50002000, 0x1000);
    create_unimplemented_device("CCFG", 0x50003000, 0x1000);
    create_unimplemented_device("CPU_SCS", 0xE000E000, 0x1000);
}

/* FIXME: Figure out how to generate these from cc13xx_boards.  */
static void cc1310_init(MachineState *machine)
{
    cc13xx_init(machine, &cc13xx_boards[0]);
}



static void cc1310_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "CC13XX - CC1310";
    mc->init = cc1310_init;
//    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
}

static const TypeInfo cc1310_type = {
    .name = MACHINE_TYPE_NAME("cc1310"),
    .parent = TYPE_MACHINE,
    .class_init = cc1310_class_init,
};


static void cc13xx_machine_init(void)
{
    type_register_static(&cc1310_type);
}

type_init(cc13xx_machine_init)
