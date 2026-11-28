/* camera backend: the real NN instructions (T41, CPU0 only). The immediates are part of the
 * instruction word, so every value nnaeng.c uses has its own case. 
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */
#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "be_mxu3.h"
#include "hal_ivs_nna_pin.h"
#include "mxu3.h"

#define CASE_RWR(r) case r: NNRWR(r); break;

static void m_rwr(void *c, unsigned reg, uint32_t v)
{
	(void)c;
	VWR0_SET(v);
	switch (reg) {
	CASE_RWR(0x29) CASE_RWR(0x2b) CASE_RWR(0x2c) CASE_RWR(0x2e) CASE_RWR(0x2f) CASE_RWR(0x30)
	CASE_RWR(0x37) CASE_RWR(0x38) CASE_RWR(0x39) CASE_RWR(0x3a) CASE_RWR(0x3b) CASE_RWR(0x3c)
	CASE_RWR(0x3e) CASE_RWR(0x3f) CASE_RWR(0x46) CASE_RWR(0x50) CASE_RWR(0x55) CASE_RWR(0x5a)
	CASE_RWR(0x5c) CASE_RWR(0x21) CASE_RWR(0x23) CASE_RWR(0x28) CASE_RWR(0x45) CASE_RWR(0x5b) CASE_RWR(0x58) CASE_RWR(0x59) CASE_RWR(0x31)
	CASE_RWR(0x42) CASE_RWR(0x43) CASE_RWR(0x44) CASE_RWR(0x5e)
	default:
		fprintf(stderr, "be_mxu3: register 0x%x has no case\n", reg);
		abort();
	}
}

static void m_dwr(void *c, unsigned imm, const uint8_t v[64])
{
	(void)c;
	VPR0_LOAD(v);
	switch (imm) {
	case 0x00: NNDWR(0x00); break;
	case 0x02: NNDWR(0x02); break;
	case 0x04: NNDWR(0x04); break;
	case 0x07: NNDWR(0x07); break;
	case 0x20: NNDWR(0x20); break;
	case 0x40: NNDWR(0x40); break;
	default:
		fprintf(stderr, "be_mxu3: NNDWR 0x%x has no case\n", imm);
		abort();
	}
}

#define CASE_DRD(k) case k: NNDRD(k); break;

static void m_drd(void *c, unsigned imm, uint8_t v[64])
{
	(void)c;
	switch (imm) {
	CASE_DRD(0) CASE_DRD(1) CASE_DRD(2) CASE_DRD(3) CASE_DRD(4) CASE_DRD(5) CASE_DRD(6) CASE_DRD(7)
	CASE_DRD(8) CASE_DRD(9) CASE_DRD(10) CASE_DRD(11) CASE_DRD(12) CASE_DRD(13) CASE_DRD(14) CASE_DRD(15)
	CASE_DRD(0x20)
	default:
		fprintf(stderr, "be_mxu3: NNDRD 0x%x has no case\n", imm);
		abort();
	}
	VPR0_STORE(v);
}

static void m_mac(void *c, unsigned imm, uint32_t word)
{
	(void)c;
	VWR0_SET(word);
	switch (imm) {
	case 0: NNMAC(0); break;
	case 1: NNMAC(1); break;
	default:
		fprintf(stderr, "be_mxu3: NNMAC 0x%x has no case\n", imm);
		abort();
	}
}

static void m_cmd(void *c, unsigned imm)
{
	(void)c;
	switch (imm) {
	case 0: NNCMD(0); break;
	case 3: NNCMD(3); break;
	case 0x60: NNCMD(0x60); break;
	case 0x83: NNCMD(0x83); break;
	case 0x84: NNCMD(0x84); break;
	case 0x8b: NNCMD(0x8b); break;
	default:
		fprintf(stderr, "be_mxu3: NNCMD 0x%x has no case\n", imm);
		abort();
	}
}

void nna_be_mxu3(struct nna_be *be)
{
	be->ctx = NULL;
	be->rwr = m_rwr, be->dwr = m_dwr, be->drd = m_drd, be->mac = m_mac, be->cmd = m_cmd;
	be->direct = 1;
}

/* The T41 deadlocks (both cores stall, the watchdog resets the SoC) when CPU0 interleaves NN
 * instructions with MXU3 loads/stores while CPU1 runs the ISP firmware's kernel MXU3 code
 * (tisp_tmo_fpga in the isp_fw_process thread). Keeping that thread on CPU0 means the two never
 * run at once (nna/open-runtime.md, step 18). The /proc scan and the pin are
 * hal_ivs_nna_pin.h, shared with raptor-hal's venus/Uranus backend (gtxaspec/raptor-hal#21).
 * ONNA_NO_ISP_PIN set: no pin. Returns the pid pinned, 0 if none, -1 on error. */
int nna_pin_isp_cpu0(void)
{
	pid_t pid;

	if (getenv("ONNA_NO_ISP_PIN"))
		return 0;
	pid = nna_pin_isp_core0();
	if (pid < 0)
		perror("onna: pinning isp_fw_process to CPU0");
	return (int)pid;
}
