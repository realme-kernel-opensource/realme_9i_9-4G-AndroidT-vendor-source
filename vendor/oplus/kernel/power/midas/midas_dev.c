/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2019-2020 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME " %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/sched/task.h>

#include "midas_dev.h"

static struct midas_dev_info *g_dev_info;

static int midas_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct midas_dev_info *info = filp->private_data;
	if (IS_ERR_OR_NULL(info))
		return -EINVAL;

	if (remap_vmalloc_range(vma, info->mmap_addr,
		  vma->vm_pgoff)) {
		pr_err("remap failed\n");
		return -EAGAIN;
	}

	return 0;
}

static int midas_dev_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct midas_dev_info *info = container_of(cdev, struct midas_dev_info, cdev);
	info->mmap_addr = vmalloc_user(sizeof(struct midas_mmap_data));
	if (IS_ERR_OR_NULL(info->mmap_addr)) {
		pr_err("malloc failed!\n");
		return -ENOMEM;
	}

	filp->private_data = info;

	return 0;
}

static int midas_dev_release(struct inode *inode, struct file *filp)
{
	struct midas_dev_info *info = filp->private_data;
	if (IS_ERR_OR_NULL(info))
		return 0;

	if (info->mmap_addr != NULL)
		vfree(info->mmap_addr);

	return 0;
}

static const struct file_operations midas_dev_fops = {
	.open = midas_dev_open,
	.release = midas_dev_release,
	.mmap = midas_dev_mmap,
	.unlocked_ioctl = midas_dev_ioctl,
};

static int __init midas_dev_init(void)
{
	int ret = 0;
	struct device *dev;

	g_dev_info = kzalloc(sizeof(struct midas_dev_info), GFP_KERNEL);
	if (IS_ERR_OR_NULL(g_dev_info)) {
		pr_err("Fail to alloc dev info\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}

	ret = alloc_chrdev_region(&g_dev_info->devno, 0, 1, "midas_dev");
	if (ret) {
		pr_err("Fail to alloc devno, ret=%d\n", ret);
		goto err_cdev_alloc;
	}

	cdev_init(&g_dev_info->cdev, &midas_dev_fops);
	ret = cdev_add(&g_dev_info->cdev, g_dev_info->devno, 1);
	if (ret) {
		pr_err("Fail to add cdev, ret=%d\n", ret);
		goto err_cdev_add;
	}

	g_dev_info->class = class_create(THIS_MODULE, "midas");
	if (IS_ERR_OR_NULL(g_dev_info->class)) {
		pr_err("Fail to create class, ret=%d\n", ret);
		goto err_class_create;
	}

	dev = device_create(g_dev_info->class, NULL, g_dev_info->devno,
				g_dev_info, "midas_dev");

	if (IS_ERR_OR_NULL(dev)) {
		pr_err("Fail to create device, ret=%d\n", ret);
		goto err_device_create;
	}

	return 0;

err_device_create:
	class_destroy(g_dev_info->class);
err_class_create:
	cdev_del(&g_dev_info->cdev);
err_cdev_add:
	unregister_chrdev_region(g_dev_info->devno, 1);
err_cdev_alloc:
	kfree(g_dev_info);
err_info_alloc:
	return ret;
}


static void __exit midas_dev_exit(void)
{
	if (!g_dev_info)
		return;

	device_destroy(g_dev_info->class, g_dev_info->devno);
	class_destroy(g_dev_info->class);
	cdev_del(&g_dev_info->cdev);
	unregister_chrdev_region(g_dev_info->devno, 1);
	kfree(g_dev_info);
}

module_init(midas_dev_init);
module_exit(midas_dev_exit);
