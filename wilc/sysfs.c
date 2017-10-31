#include <linux/kobject.h>
#include "wilc_wfi_cfgoperations.h"

static struct kobject *wilc_kobj;
static int device_created;
static struct wilc_vif *vif;

static ssize_t wilc_sysfs_show(struct kobject *kobj, struct kobj_attribute *attr,
		      char *buf)
{
	int attr_val = -1;

	if (strcmp(attr->attr.name, "p2p_mode") == 0)
		attr_val = vif->attr_sysfs.p2p_mode;
	if (strcmp(attr->attr.name, "ant_swtch_mode") == 0)
		attr_val = vif->attr_sysfs.ant_swtch_mode;
	else if(strcmp(attr->attr.name, "antenna1") == 0)
		attr_val = vif->attr_sysfs.antenna1;
	else if(strcmp(attr->attr.name, "antenna2") == 0)
		attr_val = vif->attr_sysfs.antenna2;

	return sprintf(buf, "%d\n", attr_val);
}

static ssize_t wilc_sysfs_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	int attr_val;

	if(kstrtoint(buf, 10, &attr_val))
		PRINT_ER("Failed to convert p2p_mode string");
	if (strcmp(attr->attr.name, "p2p_mode") == 0)
		vif->attr_sysfs.p2p_mode = (attr_val?1:0);
	else if(strcmp(attr->attr.name, "ant_swtch_mode") == 0)
		{
			if (attr_val > ANT_SWTCH_DUAL_GPIO_CTRL)
				PRINT_ER("Valid antenna modes: \n1-Single Antenna, 2-Dual Antenna & 0-Disable\n");
			else
				vif->attr_sysfs.ant_swtch_mode = attr_val;
		}
	else if(strcmp(attr->attr.name, "antenna1") == 0)
		vif->attr_sysfs.antenna1 = attr_val;
	else if(strcmp(attr->attr.name, "antenna2") == 0)
		vif->attr_sysfs.antenna2 = attr_val;

	return count;
}

static struct kobj_attribute p2p_mode_attr =
	__ATTR(p2p_mode, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute ant_swtch_mode_attr =
	__ATTR(ant_swtch_mode, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute ant_swtch_antenna1_attr =
	__ATTR(antenna1, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute ant_swtch_antenna2_attr =
	__ATTR(antenna2, 0664, wilc_sysfs_show, wilc_sysfs_store);


static struct attribute *wilc_attrs[] = {
	&p2p_mode_attr.attr,
	&ant_swtch_mode_attr.attr,
	&ant_swtch_antenna1_attr.attr,
	&ant_swtch_antenna2_attr.attr,
	NULL
};

static struct attribute_group attr_group = {
   .attrs = wilc_attrs,
};

void wilc_sysfs_init(struct wilc_vif *vi)
{
	int retval;
	vif = vi;

	if(device_created)
		return;

	wilc_kobj = kobject_create_and_add("wilc", NULL);
	if (!wilc_kobj) {
		retval = -ENOMEM;
		return;
	}
	
	vif->attr_sysfs.p2p_mode = 1;		/* By default p2p mode is Group Owner */

	vif->attr_sysfs.ant_swtch_mode = 0; /* switch off antenna diversity */
	vif->attr_sysfs.antenna1 =0;
	vif->attr_sysfs.antenna2 = 0;
	
	retval = sysfs_create_group(wilc_kobj, &attr_group);
	device_created = 1;
}
EXPORT_SYMBOL_GPL(wilc_sysfs_init);

void wilc_sysfs_exit(void)
{
	device_created=0;
	sysfs_remove_group(wilc_kobj, &attr_group);
	kobject_put(wilc_kobj);
}

