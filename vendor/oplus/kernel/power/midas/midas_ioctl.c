/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2019-2020 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME " %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "midas_dev.h"

#define MIDAS_IOCTL_DEF(ioctl, _func) \
	[MIDAS_IOCTL_NR(ioctl)] = { \
		.cmd = ioctl, \
		.func = _func, \
	}

#define MIDAS_IOCTL_BASE		'm'
#define MIDAS_IO(nr)			_IO(MIDAS_IOCTL_BASE, nr)
#define MIDAS_IOR(nr, type)		_IOR(MIDAS_IOCTL_BASE, nr, type)
#define MIDAS_IOW(nr, type)		_IOW(MIDAS_IOCTL_BASE, nr, type)
#define MIDAS_IOWR(nr, type)		_IOWR(MIDAS_IOCTL_BASE, nr, type)
#define MIDAS_IOCTL_NR(n)		_IOC_NR(n)
#define MIDAS_CORE_IOCTL_CNT		ARRAY_SIZE(midas_ioctls)

#define MIDAS_IOCTL_GET_TIME_IN_STATE	MIDAS_IOR(0x1, unsigned int)

#define SYS_UID				1000
#define ROOT_UID			0
#define WQ_NAME_LEN			24

enum {
        TYPE_PROCESS = 0,
        TYPE_APP,
        TYPE_TOTAL,
};

static struct midas_mmap_data midas_mmap_buf;
static DEFINE_SPINLOCK(midas_data_lock);

static int find_or_create_entry_locked(uid_t uid, struct task_struct *p, unsigned int type)
{
	unsigned int id_cnt = midas_mmap_buf.cnt;
	unsigned int index = (type == TYPE_PROCESS) ? ID_PID : ID_UID;
	unsigned int id = (type == TYPE_PROCESS) ? task_pid_nr(p) : uid;
	char buf[WQ_NAME_LEN];
	struct task_struct *task;
	int i;

	for (i = 0; i < id_cnt; i++) {
		if (midas_mmap_buf.entrys[i].type == type
				&& midas_mmap_buf.entrys[i].id[index] == id) {
			return i;
		}
	}

	if (i == id_cnt && id < CNT_MAX) {
		midas_mmap_buf.entrys[i].id[ID_UID] = uid;
		midas_mmap_buf.entrys[i].type = type;
		if (type == TYPE_PROCESS) {
			midas_mmap_buf.entrys[i].id[ID_PID] = task_pid_nr(p);
			midas_mmap_buf.entrys[i].id[ID_TGID] = task_tgid_nr(p);
			if (!(p->flags & PF_WQ_WORKER)) {
				strncpy(midas_mmap_buf.entrys[i].pid_name, p->comm, TASK_COMM_LEN);
			} else {
				get_worker_info(p, buf);
				if (buf[0])
					strncpy(midas_mmap_buf.entrys[i].pid_name, buf, TASK_COMM_LEN);
				else
					strncpy(midas_mmap_buf.entrys[i].pid_name, p->comm, TASK_COMM_LEN);
			}
			/* Get tgid name */
			rcu_read_lock();
			task = find_task_by_vpid(midas_mmap_buf.entrys[i].id[ID_TGID]);
			rcu_read_unlock();
			strncpy(midas_mmap_buf.entrys[i].tgid_name, task->comm, TASK_COMM_LEN);
		}
		id_cnt = ((id_cnt + 1) > CNT_MAX) ? CNT_MAX : (id_cnt + 1);
	}
	midas_mmap_buf.cnt = id_cnt;
	return id_cnt;
}

void midas_record_task_times(uid_t uid, u64 cputime, struct task_struct *p,
					unsigned int state) {
	int index;
	unsigned long flags;

	spin_lock_irqsave(&midas_data_lock, flags);

	index = find_or_create_entry_locked(uid, p, TYPE_APP);
	/* the unit of time_in_state is ms */
	midas_mmap_buf.entrys[index].time_in_state[state] += cputime / NSEC_PER_MSEC;

	if (uid == ROOT_UID || uid == SYS_UID) {
		index = find_or_create_entry_locked(uid, p, TYPE_PROCESS);
		/* the unit of time_in_state is ms */
		midas_mmap_buf.entrys[index].time_in_state[state] += cputime / NSEC_PER_MSEC;
	}


	spin_unlock_irqrestore(&midas_data_lock, flags);
}
EXPORT_SYMBOL(midas_record_task_times);

static int midas_ioctl_get_time_in_state(void *kdata, void *dev_info)
{
	unsigned long flags;
	struct midas_dev_info *info = dev_info;

	spin_lock_irqsave(&midas_data_lock, flags);

	memcpy(info->mmap_addr, &midas_mmap_buf, sizeof(struct midas_mmap_data));
	memset(&midas_mmap_buf, 0, sizeof(struct midas_mmap_data));

	spin_unlock_irqrestore(&midas_data_lock, flags);
	return 0;
}

/* Ioctl table */
static const struct midas_ioctl_desc midas_ioctls[] = {
	MIDAS_IOCTL_DEF(MIDAS_IOCTL_GET_TIME_IN_STATE, midas_ioctl_get_time_in_state),
};

long midas_dev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct midas_dev_info *info = filp->private_data;
	const struct midas_ioctl_desc *ioctl = NULL;
	midas_ioctl_t *func;
	unsigned int nr = MIDAS_IOCTL_NR(cmd);
	int ret = -EINVAL;
	char kdata[PAGE_SIZE] = { };
	unsigned int in_size, out_size;

	if (nr >=  MIDAS_CORE_IOCTL_CNT)
		return -EINVAL;

	ioctl = &midas_ioctls[nr];

	out_size = in_size = _IOC_SIZE(cmd);
	if ((cmd & IOC_IN) == 0)
		in_size = 0;
	if ((cmd & IOC_OUT) == 0)
		out_size = 0;

	if (out_size > PAGE_SIZE || in_size > PAGE_SIZE) {
		pr_err("out of memory\n");
		ret = -EINVAL;
		goto err_out_of_mem;
	}

	func = ioctl->func;
	if (unlikely(!func)) {
		pr_err("no func\n");
		ret = -EINVAL;
		goto err_no_func;
	}

	if (copy_from_user(kdata, (void __user *)arg, in_size)) {
		ret = -EFAULT;
		goto err_fail_cp;
	}

	ret = func(kdata, info);

	if (copy_to_user((void __user *)arg, kdata, out_size))
		ret = -EFAULT;

err_fail_cp:
err_no_func:
err_out_of_mem:
	return ret;
}
