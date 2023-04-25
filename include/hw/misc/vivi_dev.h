#ifndef HW_VIVI_DEV_H
#define HW_VIVI_DEV_H

#include "qom/object.h"
#include "hw/irq.h"

DeviceState *vivi_dev_create(hwaddr, qemu_irq irq);

#endif
