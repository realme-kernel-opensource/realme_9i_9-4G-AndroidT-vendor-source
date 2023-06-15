/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_CFS_COMMON_H_
#define _OPLUS_CFS_COMMON_H_

#define ux_err(fmt, ...) \
		printk_deferred(KERN_ERR "[UIFIRST_ERR][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_warn(fmt, ...) \
		printk_deferred(KERN_WARNING "[UIFIRST_WARN][%s]"fmt, __func__, ##__VA_ARGS__)
#define ux_debug(fmt, ...) \
		printk_deferred(KERN_INFO "[UIFIRST_INFO][%s]"fmt, __func__, ##__VA_ARGS__)

#define UX_MSG_LEN 64
#define UX_DEPTH_MAX 5
/* define for sched assist scene type, keep same as the define in java file */
#define SA_SCENE_OPT_CLEAR  (0)
#define SA_ANIM             (1 << 4)
#define SA_SCENE_OPT_SET    (1 << 7)

enum SCHED_ASSIST_SCENE
{
	SA_LAUNCH = 0,
	SA_SLIDE,
	SA_INPUT,
	SA_MAX,
};

enum DYNAMIC_UX_TYPE
{
	DYNAMIC_UX_BINDER = 0,
	DYNAMIC_UX_RWSEM,
	DYNAMIC_UX_MUTEX,
	DYNAMIC_UX_SEM,
	DYNAMIC_UX_FUTEX,
	DYNAMIC_UX_MAX,
};

#define SF_GROUP_COUNT 2
struct ux_util_record{
	char val[64];
	u64 ux_load;
	int util;
};
extern struct ux_util_record sf_target[SF_GROUP_COUNT];
extern bool task_is_sf_group(struct task_struct *tsk);

struct ux_sched_cluster {
        struct cpumask cpus;
        unsigned long capacity;
};

struct ux_sched_cputopo {
        int cls_nr;
	int big_cluster_idx;
        struct ux_sched_cluster sched_cls[NR_CPUS];
};

#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
extern unsigned int walt_scale_demand_divisor;
#define scale_demand(d) ((d)/walt_scale_demand_divisor)
#endif

#define SA_SYSTRACE_MAGIC 123
#define sched_assist_systrace(...)  sched_assist_systrace_pid(SA_SYSTRACE_MAGIC, __VA_ARGS__)

struct rq;
extern int ux_prefer_cpu[];
extern int sysctl_animation_type;
extern int sysctl_input_boost_enabled;
extern int sched_assist_ib_duration_coedecay;
extern u64 sched_assist_input_boost_duration;

extern bool oplus_task_boost_check(struct task_struct *p, int cpu);
extern void sched_assist_systrace_pid(pid_t pid, int val, const char *fmt, ...);
extern bool oplus_task_misfit(struct task_struct *p, int cpu);
extern void ux_init_rq_data(struct rq *rq);
extern void ux_init_cpu_data(void);
extern bool is_task_util_over(struct task_struct *task, int threshold);

extern void enqueue_ux_thread(struct rq *rq, struct task_struct *p);
extern void dequeue_ux_thread(struct rq *rq, struct task_struct *p);
extern void drop_ux_task_cpus(struct task_struct *p, struct cpumask *lowest_mask);

extern void pick_ux_thread(struct rq *rq, struct task_struct **p, struct sched_entity **se);

extern void dynamic_ux_dequeue(struct task_struct *task, int type);
extern void dynamic_ux_dequeue_refs(struct task_struct *task, int type, int value);
extern void dynamic_ux_enqueue(struct task_struct *task, int type, int depth);
extern void dynamic_ux_inc(struct task_struct *task, int type);
extern void dynamic_ux_sub(struct task_struct *task, int type, int value);
extern bool should_ux_task_skip_cpu(struct task_struct *task, unsigned int cpu);

extern bool test_task_ux(struct task_struct *task);
extern bool test_task_ux_depth(int ux_depth);
extern bool test_dynamic_ux(struct task_struct *task, int type);
extern bool test_set_dynamic_ux(struct task_struct *tsk);

extern bool test_ux_task_cpu(int cpu);
extern bool test_ux_prefer_cpu(struct task_struct *tsk, int cpu);
extern void find_ux_task_cpu(struct task_struct *tsk, int *target_cpu);
extern int set_ux_task_cpu_common_by_prio(struct task_struct *task, int *target_cpu, bool boost, bool prefer_idle);
extern int set_ux_task_cpu_common(struct task_struct *task, int prev_cpu, int *target_cpu);
static inline void find_slide_boost_task_cpu(struct task_struct *tsk, int *target_cpu) {}

static inline is_animator_ux_task(struct task_struct *task)
{
	return task->static_ux == 1;
}

static inline bool is_heavy_ux_task(struct task_struct *task)
{
	return task->static_ux == 2;
}

extern void place_entity_adjust_ux_task(struct cfs_rq *cfs_rq, struct sched_entity *se, int initial);
extern bool should_ux_task_skip_further_check(struct sched_entity *se);
extern bool should_ux_preempt_wakeup(struct task_struct *wake_task, struct task_struct *curr_task);
extern bool ux_skip_sync_wakeup(struct task_struct *task, int *sync);
extern void select_cpu_dur_anim(struct task_struct *task, int *target_cpu);
static inline bool is_anim_related(struct task_struct *task)
{
	return task->static_ux == 1 || test_dynamic_ux(task, DYNAMIC_UX_BINDER);
}
#endif
