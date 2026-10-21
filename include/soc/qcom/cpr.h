/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019 Linaro Limited
 * Copyright (c) 2021, AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 */

#ifndef __CPR_H__
#define __CPR_H__

#include <linux/err.h>
#include <linux/of.h>

struct cpr_ext_data {
	int mem_acc_threshold_uV;
	int apm_threshold_uV;
};

struct device;

/**
 * qcom_cpr_cpu_pd_index() - Find the CPR domain in a CPU's power-domains
 * @cpu_np: CPU node
 *
 * A CPU can sit in more than one power domain: with PSCI OS-initiated idle
 * its PSCI domain comes first (of_get_cpu_state_node() reads the domain idle
 * states from power-domains[0]) and the CPR thread that supplies it is
 * named "cpr" in power-domain-names. A CPU node without names lists only
 * the CPR domain.
 *
 * Returns: The index into power-domains, or -ENODEV when the names do not
 *          include "cpr".
 */
static inline int qcom_cpr_cpu_pd_index(const struct device_node *cpu_np)
{
	int idx = of_property_match_string(cpu_np, "power-domain-names", "cpr");

	if (idx == -EINVAL)
		return 0;

	return idx < 0 ? -ENODEV : idx;
}

#if IS_ENABLED(CONFIG_QCOM_CPR3)
struct cpr_ext_data *qcom_cpr3_cprh_init(struct device *cpu_dev);
int qcom_cpr3_cprh_enable(struct device *cpu_dev);
#else
static inline struct cpr_ext_data *qcom_cpr3_cprh_init(struct device *cpu_dev)
{
	return ERR_PTR(-ENODEV);
}

static inline int qcom_cpr3_cprh_enable(struct device *cpu_dev)
{
	return -ENODEV;
}
#endif

#endif /* __CPR_H__ */
