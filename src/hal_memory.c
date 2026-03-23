/*
 * hal_memory.c -- Raptor HAL memory management implementation
 *
 * Implements all memory vtable functions: alloc/free, cache flush,
 * physical/virtual address translation, and memory pool operations.
 *
 * The Ingenic SDK provides two memory management paths:
 *
 *   1. General allocation: IMP_Alloc / IMP_Free for physically
 *      contiguous DMA-capable memory.  Available on all SoCs.
 *
 *   2. Memory pool (T23+ only): IMP_PoolAlloc / IMP_PoolFree /
 *      IMP_PoolFlushCache / IMP_PoolPhys_to_Virt / IMP_PoolVirt_to_Phys
 *      for pool-based allocation from reserved memory.
 *      On older SoCs (T20/T21/T30), pool functions fall back to the
 *      non-pool equivalents or return RSS_ERR_NOTSUP.
 *
 * Physical/virtual address translation uses the vendor
 * IMP_Phys_to_Virt / IMP_Virt_to_Phys functions which operate
 * on the rmem region mapped by the SDK.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: MIT
 */

#include "hal_internal.h"

#ifdef __mips__
#define _GNU_SOURCE
#include <sys/cachectl.h>
#endif

/* Extern declarations for functions present in libimp.so
 * but missing from SDK headers */
extern void *IMP_Alloc(uint32_t size);
extern void  IMP_Free(void *ptr);
extern int   IMP_FlushCache(void *ptr, uint32_t size);
extern void *IMP_Phys_to_Virt(uint32_t phys_addr);
extern uint32_t IMP_Virt_to_Phys(void *virt_addr);

#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
extern void *IMP_PoolAlloc(uint32_t pool_id, uint32_t size);
extern void  IMP_PoolFree(void *ptr);
extern int   IMP_PoolFlushCache(void *ptr, uint32_t size);
extern void *IMP_PoolPhys_to_Virt(uint32_t phys_addr);
extern uint32_t IMP_PoolVirt_to_Phys(void *virt_addr);
#endif

/* ================================================================
 * GENERAL ALLOCATION
 *
 * Uses IMP_Alloc / IMP_Free for DMA-capable memory.
 * ================================================================ */

void *hal_mem_alloc(void *ctx, uint32_t size, const char *name)
{
	(void)ctx;

	if (size == 0)
		return NULL;

	void *ptr = IMP_Alloc(size);
	if (!ptr) {
		HAL_LOG_ERR("IMP_Alloc(%u, %s) failed", size,
		            name ? name : "(null)");
		return NULL;
	}

	HAL_LOG_DBG("mem_alloc(%u, %s) = %p", size,
	            name ? name : "(null)", ptr);
	return ptr;
}

void hal_mem_free(void *ctx, void *ptr)
{
	(void)ctx;
	if (ptr)
		IMP_Free(ptr);
}

/* ================================================================
 * CACHE FLUSH
 *
 * T23+ has IMP_FlushCache.  On older SoCs, fall back to the
 * cacheflush() syscall on MIPS.
 * ================================================================ */

#ifdef __mips__
#include <sys/cachectl.h>
#endif

int hal_mem_flush_cache(void *ctx, void *ptr, uint32_t size)
{
	(void)ctx;

	if (!ptr || size == 0)
		return RSS_ERR_INVAL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_FlushCache(ptr, size);
#elif defined(__mips__)
	/* MIPS cacheflush: flush data cache */
	if (cacheflush(ptr, (int)size, DCACHE) != 0) {
		HAL_LOG_ERR("cacheflush(%p, %u) failed: %s",
		            ptr, size, strerror(errno));
		return RSS_ERR_IO;
	}
	return RSS_OK;
#else
	/* Non-MIPS: assume cache coherent */
	(void)ptr;
	(void)size;
	return RSS_OK;
#endif
}

/* ================================================================
 * PHYSICAL / VIRTUAL ADDRESS TRANSLATION
 *
 * Uses vendor IMP_Phys_to_Virt / IMP_Virt_to_Phys which operate
 * on the rmem/DMA region mapped by the SDK at init time.
 * ================================================================ */

void *hal_mem_phys_to_virt(void *ctx, uint32_t phys_addr)
{
	(void)ctx;

	void *virt = IMP_Phys_to_Virt(phys_addr);
	if (!virt) {
		HAL_LOG_WARN("IMP_Phys_to_Virt(0x%08x) returned NULL",
		             phys_addr);
	}
	return virt;
}

uint32_t hal_mem_virt_to_phys(void *ctx, void *virt_addr)
{
	(void)ctx;

	if (!virt_addr) {
		HAL_LOG_WARN("mem_virt_to_phys(NULL)");
		return 0;
	}

	return IMP_Virt_to_Phys(virt_addr);
}

/* ================================================================
 * MEMORY POOL OPERATIONS
 *
 * T23+: IMP_PoolAlloc / IMP_PoolFree for pool-based allocation.
 * T20/T21/T30: Fall back to IMP_Alloc / IMP_Free (no pool concept).
 * T31: Has IMP_System_MemPoolRequest but also supports IMP_PoolAlloc.
 * ================================================================ */

void *hal_mem_pool_alloc(void *ctx, uint32_t pool_id, uint32_t size)
{
	(void)ctx;

	if (size == 0)
		return NULL;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	void *ptr = IMP_PoolAlloc(pool_id, size);
	if (!ptr) {
		HAL_LOG_ERR("IMP_PoolAlloc(%u, %u) failed", pool_id, size);
		return NULL;
	}
	HAL_LOG_DBG("mem_pool_alloc(pool=%u, size=%u) = %p",
	            pool_id, size, ptr);
	return ptr;
#else
	/* T20/T21/T30: no pool API, fall back to general alloc */
	(void)pool_id;
	void *ptr = IMP_Alloc(size);
	if (!ptr) {
		HAL_LOG_ERR("IMP_Alloc(%u) [pool fallback] failed", size);
		return NULL;
	}
	HAL_LOG_DBG("mem_pool_alloc(pool=%u, size=%u) = %p [fallback]",
	            pool_id, size, ptr);
	return ptr;
#endif
}

void hal_mem_pool_free(void *ctx, void *ptr)
{
	(void)ctx;

	if (!ptr)
		return;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	IMP_PoolFree(ptr);
#else
	/* T20/T21/T30: fall back to general free */
	IMP_Free(ptr);
#endif
}

/* ================================================================
 * MEMORY POOL CACHE FLUSH
 *
 * T23+: IMP_PoolFlushCache
 * T20/T21/T30: Fall back to regular cache flush
 * ================================================================ */

int hal_mem_pool_flush_cache(void *ctx, void *ptr, uint32_t size)
{
#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	(void)ctx;
	if (!ptr || size == 0)
		return RSS_ERR_INVAL;
	return IMP_PoolFlushCache(ptr, size);
#else
	/* T20/T21/T30: fall back to regular flush */
	return hal_mem_flush_cache(ctx, ptr, size);
#endif
}

/* ================================================================
 * MEMORY POOL PHYSICAL / VIRTUAL ADDRESS TRANSLATION
 *
 * T23+: IMP_PoolPhys_to_Virt / IMP_PoolVirt_to_Phys
 * T20/T21/T30: Fall back to non-pool equivalents
 * ================================================================ */

void *hal_mem_pool_phys_to_virt(void *ctx, uint32_t phys_addr)
{
	(void)ctx;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	void *virt = IMP_PoolPhys_to_Virt(phys_addr);
	if (!virt)
		HAL_LOG_WARN("IMP_PoolPhys_to_Virt(0x%08x) returned NULL",
		             phys_addr);
	return virt;
#else
	/* T20/T21/T30: fall back to non-pool */
	return hal_mem_phys_to_virt(ctx, phys_addr);
#endif
}

uint32_t hal_mem_pool_virt_to_phys(void *ctx, void *virt_addr)
{
	(void)ctx;

	if (!virt_addr)
		return 0;

#if defined(PLATFORM_T23) || defined(PLATFORM_T31) || \
    defined(PLATFORM_T32) || defined(PLATFORM_T40) || defined(PLATFORM_T41)
	return IMP_PoolVirt_to_Phys(virt_addr);
#else
	/* T20/T21/T30: fall back to non-pool */
	return hal_mem_virt_to_phys(ctx, virt_addr);
#endif
}
