#include <linux/kobject.h>
#include "wilc_wfi_cfgoperations.h"

struct p2p_mode
{
struct device *dev;
unsigned int mode;
};


struct p2p_mode *p2p_mode_p;

static struct kobject *p2p_kobj;
static int p2p_mode;
static int device_created;
static struct wilc_vif *vif;

static ssize_t p2p_mode_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	return sprintf(buf, "%d\n", p2p_mode);
}

static ssize_t p2p_mode_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &p2p_mode);
	if(ret < 0) {
		PRINT_ER("Failed to convert p2p_mode string");
		return ret;
	}

	if(p2p_mode)
		vif->p2p_mode = 1;
	else
		vif->p2p_mode = 0;

	return count;
}

static struct kobj_attribute p2p_mode_attr =
	__ATTR(mode, 0664, p2p_mode_show, p2p_mode_store);

static struct attribute *p2p_attrs[] = {
	&p2p_mode_attr.attr,
	NULL
};

static struct attribute_group attr_group = {
   .attrs = p2p_attrs,
};

void p2p_sysfs_init(struct wilc_vif *vi)
{
	int retval;
	vif = vi;

	if(device_created)
		return;

	p2p_kobj = kobject_create_and_add("wilc_p2p", NULL);
	if (!p2p_kobj) {
		retval = -ENOMEM;
		return;
	}

	vif->p2p_mode = 1;
	
	retval = sysfs_create_group(p2p_kobj, &attr_group);
	device_created = 1;
}
EXPORT_SYMBOL_GPL(p2p_sysfs_init);

void p2p_sysfs_exit(void)
{
	device_created=0;
	sysfs_remove_group(p2p_kobj, &attr_group);
	kobject_put(p2p_kobj);
}

