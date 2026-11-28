/*
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#ifndef BE_MXU3_H
#define BE_MXU3_H
#include "nnaeng.h"
void nna_be_mxu3(struct nna_be *be);
/* keep the ISP firmware thread on CPU0 (avoids a T41 dual-core deadlock); call before NN work */
int nna_pin_isp_cpu0(void);
#endif
