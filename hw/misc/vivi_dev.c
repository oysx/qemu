#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h" /* provides error_fatal() handler */
#include "hw/sysbus.h" /* provides all sysbus registering func */
#include "hw/misc/vivi_dev.h"
#include "hw/ptimer.h"
#include "hw/register.h"

#define TYPE_BANANA_ROM "banana_rom"
typedef struct BananaRomState BananaRomState;
DECLARE_INSTANCE_CHECKER(BananaRomState, BANANA_ROM, TYPE_BANANA_ROM)

static char *
real_time_iso8601(void)
{
#if GLIB_CHECK_VERSION(2,62,0)
    g_autoptr(GDateTime) dt = g_date_time_new_now_utc();
    /* ignore deprecation warning, since GLIB_VERSION_MAX_ALLOWED is 2.56 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return g_date_time_format_iso8601(dt);
#pragma GCC diagnostic pop
#else
    GTimeVal tv;
    g_get_current_time(&tv);
    return g_time_val_to_iso8601(&tv);
#endif
}

#define LOG(lvl, fmt, ...) {\
	gchar *ts;\
	ts = real_time_iso8601();\
	qemu_log_mask(lvl, "%s ", ts);\
	g_free(ts);\
	qemu_log_mask(lvl, fmt, ## __VA_ARGS__);\
}

REG32(CTRL, 0x00)
    FIELD(CTRL,     EN,     0,  1)      /* component enable */
    FIELD(CTRL,     IEN,    1,  1)      /* interrupt enable */
    FIELD(CTRL,     FREQ,   8,  8)      /* sampling frequency setting */

REG32(STATUS, 0x04)
    FIELD(STATUS,   IFG,    1,  1)      /* interrupt flag */

REG32(DATA, 0x08)
    FIELD(DATA,     SAMPLE, 0,  16)     /* current value */

#define R_MAX   ((R_DATA) + 1)

#define REG_ID 	0x0
#define CHIP_ID	0xBA000001

struct BananaRomState {
	SysBusDevice parent_obj;
	MemoryRegion iomem;
	uint64_t chip_id;
	uint8_t timeout;	//ms unit
	ptimer_state *timer;
	qemu_irq irq;
    uint32_t regs[R_MAX];
    RegisterInfo regs_info[R_MAX];

};

static uint64_t banana_rom_read(void *opaque, hwaddr addr, unsigned int size)
{
	BananaRomState *s = opaque;
	LOG(LOG_GUEST_ERROR, "%s: read: addr=0x%x size=%d\n",
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
	LOG(LOG_GUEST_ERROR, "*********%s: write: addr=0x%x size=%d\n",
                  __func__, (int)addr,size);
	(void)s;
}

static const MemoryRegionOps banana_rom_ops = {
	.read = banana_rom_read,
	.write = banana_rom_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

static void mm_sens_update_irq(void *obj)
{
	BananaRomState *s = BANANA_ROM(obj);
    bool pending = s->regs[R_CTRL] & s->regs[R_STATUS] & R_CTRL_IEN_MASK;
    qemu_set_irq(s->irq, pending);
}

static void mm_sens_update_data(void *obj)
{
	BananaRomState *s = BANANA_ROM(obj);
	s->regs[R_STATUS] |= R_STATUS_IFG_MASK;	//simulate interrupt occur

	mm_sens_update_irq(obj);
}

static void mm_sens_reset(DeviceState *dev)
{
	BananaRomState *s = BANANA_ROM(dev);
	s->timeout = 1;
    for (int i = 0; i < R_MAX; ++i) {
        register_reset(&s->regs_info[i]);
    }
}

static void r_ctrl_post_write(RegisterInfo *reg, uint64_t val)
{
	BananaRomState *s = BANANA_ROM(reg->opaque);
	LOG(LOG_GUEST_ERROR, "%s: write: val=0x%lx\n", __func__, (unsigned long)val);

	uint8_t timeout = (s->regs[R_CTRL] & R_CTRL_FREQ_MASK) >> R_CTRL_FREQ_SHIFT;

	ptimer_transaction_begin(s->timer);

	if (timeout != s->timeout){
		s->timeout = timeout;
		ptimer_set_limit(s->timer, timeout, 1);
	}

	if(s->regs[R_CTRL] & R_CTRL_EN_MASK){
		ptimer_run(s->timer, 0);

        if (s->regs[R_CTRL] & R_CTRL_IEN_MASK) {
            mm_sens_update_irq(s);
        }
	}else{
		ptimer_stop(s->timer);
	}
	ptimer_transaction_commit(s->timer);
}

static void r_status_post_write(RegisterInfo *reg, uint64_t val)
{
	BananaRomState *s = BANANA_ROM(reg->opaque);
	LOG(LOG_GUEST_ERROR, "%s: write: val=0x%lx\n", __func__, (unsigned long)val);
	mm_sens_update_irq(s);
}

static const RegisterAccessInfo mm_sens_regs_info[] = {
    {   .name = "CTRL",           .addr = A_CTRL,
        .reset = 0,
        .rsvd = ~(R_CTRL_EN_MASK | R_CTRL_IEN_MASK | R_CTRL_FREQ_MASK),
        .post_write = r_ctrl_post_write,
    },
    {   .name = "STATUS",           .addr = A_STATUS,
        .reset = 0,
        .rsvd = ~R_STATUS_IFG_MASK,
        .post_write = r_status_post_write,
    },
    {   .name = "DATA",         .addr = A_DATA,
        .reset = 0,
        .rsvd = ~R_DATA_SAMPLE_MASK,
        .ro = R_DATA_SAMPLE_MASK,
    },
};

static const MemoryRegionOps mm_sens_reg_ops = {
    .read = register_read_memory,
    .write = register_write_memory,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static void banana_rom_instance_init(Object *obj)
{
	BananaRomState *s = BANANA_ROM(obj);

	/* allocate memory map region */
	memory_region_init_io(&s->iomem, obj, &banana_rom_ops, s, TYPE_BANANA_ROM, 0x10000);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    RegisterInfoArray *reg_array = register_init_block32(DEVICE(obj), mm_sens_regs_info,
                                      ARRAY_SIZE(mm_sens_regs_info),
                                      s->regs_info, s->regs,
                                      &mm_sens_reg_ops,
                                      0,	//debug_enable
                                      R_MAX * 4);
    memory_region_add_subregion(&s->iomem,
                                A_CTRL,
                                &reg_array->mem);

	s->chip_id = CHIP_ID;

    s->timer = ptimer_init(mm_sens_update_data, s, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    ptimer_transaction_begin(s->timer);
    ptimer_set_period(s->timer, 1000000);	//set to ms unit
	ptimer_set_limit(s->timer, 1000, 1);	//default to 1s
    ptimer_transaction_commit(s->timer);

}

static void mm_sens_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = mm_sens_reset;
    // dc->vmsd = &vmstate_mm_sens;
}

/* create a new type to define the info related to our device */
static const TypeInfo banana_rom_info = {
	.name = TYPE_BANANA_ROM,
	.parent = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(BananaRomState),
	.instance_init = banana_rom_instance_init,
	.class_init     = mm_sens_class_init,
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
