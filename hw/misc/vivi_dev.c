#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h" /* provides error_fatal() handler */
#include "hw/sysbus.h" /* provides all sysbus registering func */
#include "hw/misc/vivi_dev.h"
#include "hw/ptimer.h"

#define TYPE_BANANA_ROM "banana_rom"
typedef struct BananaRomState BananaRomState;
DECLARE_INSTANCE_CHECKER(BananaRomState, BANANA_ROM, TYPE_BANANA_ROM)

#define REG_ID 	0x0
#define CHIP_ID	0xBA000001

struct BananaRomState {
	SysBusDevice parent_obj;
	MemoryRegion iomem;
	uint64_t chip_id;
	ptimer_state *timer;
	qemu_irq irq;
};

static uint64_t banana_rom_read(void *opaque, hwaddr addr, unsigned int size)
{
	BananaRomState *s = opaque;
	qemu_log_mask(LOG_GUEST_ERROR, "%s: read: addr=0x%x size=%d\n",
                  __func__, (int)addr,size);
	switch (addr) {
	case REG_ID:
		return s->chip_id;
		break;

	default:
		return 0xDEADBEEF;
		break;
	}

	return 0;
}

static void banana_rom_write(void *obj, hwaddr addr, uint64_t value, unsigned int size)
{
	BananaRomState *s = BANANA_ROM(obj);
	qemu_log_mask(LOG_GUEST_ERROR, "*********%s: write: addr=0x%x size=%d\n",
                  __func__, (int)addr,size);
	
	if(addr == 0){
		// clear interrupt
		qemu_set_irq(s->irq, 0);
		return;
	}

	ptimer_transaction_begin(s->timer);
	ptimer_run(s->timer, 0);
	ptimer_transaction_commit(s->timer);
}

static const MemoryRegionOps banana_rom_ops = {
	.read = banana_rom_read,
	.write = banana_rom_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void mm_sens_update_data(void *obj)
{
	BananaRomState *s = BANANA_ROM(obj);
	qemu_set_irq(s->irq, 1);
}

static void banana_rom_instance_init(Object *obj)
{
	qemu_log("^^^^^^^^^^");
	qemu_log_mask(LOG_GUEST_ERROR, "**********");
	BananaRomState *s = BANANA_ROM(obj);

	/* allocate memory map region */
	memory_region_init_io(&s->iomem, obj, &banana_rom_ops, s, TYPE_BANANA_ROM, 0x10000);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
	
	s->chip_id = CHIP_ID;

    s->timer = ptimer_init(mm_sens_update_data, s, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    ptimer_transaction_begin(s->timer);
    ptimer_set_period(s->timer, 1000000000);
	ptimer_set_limit(s->timer, 3, 1);
    ptimer_transaction_commit(s->timer);

}

/* create a new type to define the info related to our device */
static const TypeInfo banana_rom_info = {
	.name = TYPE_BANANA_ROM,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(BananaRomState),
	.instance_init = banana_rom_instance_init,
};

static void banana_rom_register_types(void)
{
    type_register_static(&banana_rom_info);
}

type_init(banana_rom_register_types)

/*
 * Create the Banana Rom device.
 */
DeviceState *vivi_dev_create(hwaddr addr, qemu_irq irq)
{
	DeviceState *dev = qdev_new(TYPE_BANANA_ROM);
	sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
	sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
	sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

	return dev;
}
