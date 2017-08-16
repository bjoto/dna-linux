/*
 *  tpacket v4
 *  Copyright(c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _LINUX_TPACKET4_H
#define _LINUX_TPACKET4_H

#include <asm/barrier.h>
#include <linux/if_packet.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#define TP4_UMEM_MIN_FRAME_SIZE 2048
#define TP4_UMEM_MIN_PACKET_DATA 1536

struct tp4_umem {
	struct pid *pid;
	struct page **pgs;
	unsigned int npgs;
	size_t size;
	unsigned long address;
	unsigned int frame_size;
	unsigned int frame_size_log2;
	unsigned int nframes;
	unsigned int nfpplog2; /* num frames per page in log2 */
	unsigned int header_headroom;
	unsigned int data_headroom;
};

struct tp4_dma_info {
	dma_addr_t dma;
	struct page *page;
};

struct tp4_queue {
	struct tpacket4_desc *vring;

	unsigned int used_idx;
	unsigned int last_avail_idx;
	unsigned int ring_size;
	unsigned int num_free;

	struct tp4_umem *umem;
	struct tp4_dma_info *dma_info;
	enum dma_data_direction direction;
	spinlock_t tx_lock; /* used in copy mode when completing from skb dtor */
};

enum tp4_netdev_command {
	/* Enable the AF_PACKET V4 zerocopy support. When this is enabled,
	 * packets will arrive to the socket without being copied resulting
	 * in better performance. Note that this also means that no packets
	 * are sent to the kernel stack after this feature has been enabled.
	 */
	TP4_ENABLE,
	/* Disables the PACKET_ZEROCOPY support. */
	TP4_DISABLE,
};

struct tp4_netdev_parms {
	enum tp4_netdev_command command;

	struct tp4_queue *tp4q_rx;
	struct tp4_queue *tp4q_tx;
	void (*readable)(void *);
	void *readable_opaque;
	void (*writable)(void *);
	void *writable_opaque;
};

/**
 *
 **/
static inline void tp4q_init(struct tp4_queue *tp4q, unsigned int nentries,
			     struct tp4_umem *umem,
			     struct tpacket4_desc *buffer)
{
	tp4q->vring = buffer;
	tp4q->used_idx = 0;
	tp4q->last_avail_idx = 0;
	tp4q->ring_size = nentries;
	tp4q->num_free = 0;
	tp4q->umem = umem;
	spin_lock_init(&tp4q->tx_lock);
}

/**
 *
 **/
static inline struct tp4_umem *tp4q_new_umem(unsigned long addr, size_t size,
					     unsigned int frame_size,
					     unsigned int header_headroom,
					     unsigned int data_headroom)
{
	struct tp4_umem *umem;
	unsigned int nframes;

	if (frame_size < TP4_UMEM_MIN_FRAME_SIZE || frame_size > PAGE_SIZE) {
		/* Strictly speaking we could support this, if:
		 * - huge pages, or*
		 * - using an IOMMU, or
		 * - making sure the memory area is consecutive
		 * but for now, we simply say "computer says no".
		 */
		return ERR_PTR(-EINVAL);
	}

	if (!is_power_of_2(frame_size))
		return ERR_PTR(-EINVAL);

	if (!PAGE_ALIGNED(addr)) {
		/* Memory area has to be page size aligned. For
		 * simplicity, this might change.
		 */
		return ERR_PTR(-EINVAL);
	}

	if ((addr + size) < addr)
		return ERR_PTR(-EINVAL);

	nframes = size / frame_size;
	if (nframes == 0)
		return ERR_PTR(-EINVAL);

	/* require 4B alignment, for u32 access */
	if (!IS_ALIGNED(header_headroom, 4))
		return ERR_PTR(-EINVAL);

	data_headroom =
		ALIGN(header_headroom + TPACKET4_HDRLEN + data_headroom, 64) -
		(header_headroom + TPACKET4_HDRLEN);

	if (frame_size - header_headroom - TPACKET4_HDRLEN -
	    data_headroom - TP4_UMEM_MIN_PACKET_DATA <= 0) {
		return ERR_PTR(-EINVAL);
	}

	umem = kzalloc(sizeof(*umem), GFP_KERNEL);
	if (!umem)
		return ERR_PTR(-ENOMEM);

	umem->pid = get_task_pid(current, PIDTYPE_PID);
	umem->size = size;
	umem->address = addr;
	umem->frame_size = frame_size;
	umem->frame_size_log2 = ilog2(frame_size);
	umem->nframes = nframes;
	umem->nfpplog2 = ilog2(PAGE_SIZE / frame_size);
	umem->header_headroom = header_headroom;
	umem->data_headroom = data_headroom;

	return umem;
}

/**
 *
 **/
static inline int tp4q_is_free(struct tp4_queue *q)
{
	unsigned int idx = q->used_idx & (q->ring_size - 1);
	unsigned int prev_idx;

	if (!idx)
		prev_idx = q->ring_size - 1;
	else
		prev_idx = idx - 1;

	/* previous frame is already consumed by userspace
	 * meaning ring is free
	 */
	if (q->vring[prev_idx].flags & DESC_HW)
		return 1;

	/* there is some data that userspace can read immediately */
	return 0;
}

/**
 *
 **/
static inline int tp4q_nb_avail(struct tp4_queue *q, int dcnt)
{
	unsigned int idx, last_avail_idx = q->last_avail_idx;
	int i, entries = 0;

	for (i = 0; i < dcnt; i++) {
		idx = (last_avail_idx++) & (q->ring_size - 1);
		if (!(q->vring[idx].flags & DESC_HW))
			break;
		entries++;
	}

	return entries;
}

/**
 *
 **/
static inline int tp4q_dequeue(struct tp4_queue *q,
			       struct tpacket4_desc *d, int dcnt)
{
	unsigned int idx;
	int i, entries;

	entries = tp4q_nb_avail(q, dcnt);
	q->num_free += entries;

	/* Order flags and data */
	smp_rmb();

	for (i = 0; i < entries; i++) {
		idx = (q->last_avail_idx++) & (q->ring_size - 1);
		d[i] = q->vring[idx];
	}
	return entries;
}

/**
 *
 **/
static inline int tp4q_enqueue(struct tp4_queue *q,
			       const struct tpacket4_desc *d, int dcnt)
{
	unsigned int used_idx = q->used_idx;
	int i;

	if (q->num_free < dcnt)
		return -ENOSPC;

	q->num_free -= dcnt;

	for (i = 0; i < dcnt; i++) {
		unsigned int idx = (used_idx++) & (q->ring_size - 1);

		q->vring[idx].addr = d[i].addr;
		q->vring[idx].len = d[i].len;
		q->vring[idx].error = d[i].error;
	}

	/* Order flags and data */
	smp_wmb();

	for (i = 0; i < dcnt; i++) {
		unsigned int idx = (q->used_idx++) & (q->ring_size - 1);

		q->vring[idx].flags = d[i].flags & ~DESC_HW;
	}
	return 0;
}

/**
 *
 **/
static inline bool tp4q_validate_header(struct tp4_queue *tp4q,
					struct tpacket4_hdr *hdr)
{
	unsigned int max_off = tp4q->umem->frame_size -
			       tp4q->umem->header_headroom;
	unsigned int min_off = TPACKET4_HDRLEN;

	if (hdr->data >= max_off || hdr->data < min_off ||
	    hdr->data_end > max_off || hdr->data_end < min_off ||
	    hdr->data_end < hdr->data) {
		return false;
	}

	return true;
}

/**
 *
 **/
static inline bool tp4_get_page_offset(struct tp4_queue *tp4q, u64 addr,
				       u64 *pg, u64 *off)
{
	if (addr >= tp4q->umem->nframes)
		return false;

	*pg = addr >> tp4q->umem->nfpplog2;
	*off = (addr - (*pg << tp4q->umem->nfpplog2))
	       << tp4q->umem->frame_size_log2;

	return true;
}

/**
 *
 **/
static inline struct tpacket4_hdr *tp4q_get_header(struct tp4_queue *tp4q,
						   u64 addr)
{
	u64 pg, off;
	u8 *hdr;

	if (!tp4_get_page_offset(tp4q, addr, &pg, &off))
		return NULL;

	hdr = page_address(tp4q->umem->pgs[pg]);
	return (struct tpacket4_hdr *)
		(hdr + off + tp4q->umem->header_headroom);
}

/**
 *
 **/
static inline struct tpacket4_hdr *tp4q_get_validated_header(
	struct tp4_queue *tp4q, u64 addr)
{
	struct tpacket4_hdr *hdr;

	hdr = tp4q_get_header(tp4q, addr);
	if (!hdr || !tp4q_validate_header(tp4q, hdr))
		return NULL;

	return hdr;
}

/**
 *
 **/
static inline void tp4q_write_header(struct tp4_queue *tp4q,
				     struct tpacket4_hdr *hdr, u32 size)
{
	hdr->data = TPACKET4_HDRLEN + tp4q->umem->data_headroom;
	hdr->data_end = hdr->data + size;
}

/**
 *
 **/
static inline void tp4q_write_desc(struct tpacket4_desc *desc, u64 addr,
				   u32 len)
{
	desc->addr = addr;
	desc->len = len;
	desc->flags = 0;
	desc->error = 0;
}

/**
 *
 **/
static inline void tp4q_set_error(struct tpacket4_desc *desc,
				  int errno)
{
	desc->error = errno;
}

/**
 *
 **/
static inline void tp4q_return_error(struct tp4_queue *tp4q, int errno)
{
	struct tpacket4_desc desc;
	int ndescs;
	int err;

	ndescs = tp4q_dequeue(tp4q, &desc, 1);
	if (ndescs == 0) {
		/* Do not call this function if you have no entries left
		 * in the queue to use for errors.
		 */
		WARN_ON_ONCE(1);
		return;
	}

	tp4q_set_error(&desc, errno);
	err = tp4q_enqueue(tp4q, &desc, 1);
	WARN_ON(err);
}

/**
 *
 **/
static inline void tp4q_disable(struct device *dev,
				struct tp4_queue *tp4q)
{
	int i;

	/* Already been cleared, so nothing to do */
	if (!tp4q->umem)
		return;

	if (tp4q->dma_info) {
		/* Unmap DMA */
		for (i = 0; i < tp4q->umem->npgs; i++)
			dma_unmap_page(dev, tp4q->dma_info[i].dma, PAGE_SIZE,
				       tp4q->direction);

		kfree(tp4q->dma_info);
		tp4q->dma_info = NULL;
	}

	tp4q->umem = NULL;
}

/**
 *
 **/
static inline int tp4q_enable(struct device *dev,
			      struct tp4_queue *tp4q,
			      enum dma_data_direction direction)
{
	int i, j;

	/* Empty umem signifies application wants only one of RX or TX */
	if (!tp4q->umem)
		return 0;

	/* DMA map all the buffers in bufs up front, and sync prior
	 * kicking userspace. Is this sane? Strictly user land owns
	 * the buffer until they show up on the avail queue. However,
	 * mapping should be ok.
	 */
	if (direction != DMA_NONE) {
		tp4q->dma_info = kzalloc(sizeof(*tp4q->dma_info) *
					 tp4q->umem->npgs, GFP_KERNEL);
		if (!tp4q->dma_info)
			return -ENOMEM;

		for (i = 0; i < tp4q->umem->npgs; i++) {
			dma_addr_t dma;

			dma = dma_map_page(dev, tp4q->umem->pgs[i], 0,
					   PAGE_SIZE, direction);
			if (dma_mapping_error(dev, dma)) {
				for (j = 0; j < i; j++)
					dma_unmap_page(dev,
						       tp4q->dma_info[j].dma,
						       PAGE_SIZE, direction);
				kfree(tp4q->dma_info);
				tp4q->dma_info = NULL;
				return -EBUSY;
			}

			tp4q->dma_info[i].page = tp4q->umem->pgs[i];
			tp4q->dma_info[i].dma = dma;
		}
	} else {
		tp4q->dma_info = NULL;
	}

	tp4q->direction = direction;
	return 0;
}

/**
 *
 **/
static inline unsigned int tp4q_get_frame_size(struct tp4_queue *tp4q)
{
	return tp4q->umem->frame_size;
}

/**
 *
 **/
static inline unsigned int tp4q_max_data_size(struct tp4_queue *tp4q)
{
	return tp4q->umem->frame_size - tp4q->umem->header_headroom -
		TPACKET4_HDRLEN - tp4q->umem->data_headroom;
}

/**
 *
 **/
static inline void *tp4q_get_data(struct tp4_queue *tp4q,
				  struct tpacket4_hdr *hdr)
{
	return (u8 *)hdr + TPACKET4_HDRLEN + tp4q->umem->data_headroom;
}


#endif /* _LINUX_TPACKET4_H */
