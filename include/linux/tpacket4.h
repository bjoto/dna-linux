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
#include <linux/log2.h>

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

/**
 * struct tp4_packet_array - An array of packets
 * @tp4q: the tp4q associated with this packet array. Flushes and
 *	  populates will operate on this.
 * @start: the first packet that has not been processed
 * @curr: the packet that is currently being processed
 * @end: the last packet in the array
 * @mask: convenience variable for internal operations on the array
 * @items: the actual descriptors to frames/packets that are in the array
 **/
struct tp4_packet_array {
	struct tp4_queue *tp4q;
	u32 start;
	u32 curr;
	u32 end;
	u32 mask;
	struct tpacket4_desc items[0];
};

/**
 * struct tp4_packet - A packet consisting of one or more frames
 * @pkt_arr: the packet array this packet is located in
 * @start: the first frame that has not been processed
 * @curr: the frame that is currently being processed
 * @end: the last frame in the packet
 **/
struct tp4_packet {
	struct tp4_packet_array *pkt_arr;
	u32 start;
	u32 curr;
	u32 end;
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
static inline int tp4q_dequeue_to_ring(struct tp4_queue *q,
				       struct tpacket4_desc *d,
				       u32 start, u32 dcnt, u32 mask)
{
	unsigned int idx;
	int i, entries;

	entries = tp4q_nb_avail(q, dcnt);
	q->num_free += entries;

	/* Order flags and data */
	smp_rmb();

	for (i = 0; i < entries; i++) {
		idx = (q->last_avail_idx++) & (q->ring_size - 1);
		d[start++ & mask] = q->vring[idx];
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
static inline int tp4q_enqueue_from_ring(struct tp4_queue *q,
					 const struct tpacket4_desc *d,
					 u32 start, u32 dcnt, u32 mask)
{
	unsigned int used_idx = q->used_idx;
	u32 didx = start;
	int i;

	if (q->num_free < dcnt)
		return -ENOSPC;

	q->num_free -= dcnt;

	for (i = 0; i < dcnt; i++) {
		unsigned int idx = (used_idx++) & (q->ring_size - 1);

		q->vring[idx].addr = d[didx & mask].addr;
		q->vring[idx].len = d[didx & mask].len;
		q->vring[idx].error = d[didx & mask].error;
		didx++;
	}

	/* Order flags and data */
	smp_wmb();

	for (i = 0; i < dcnt; i++) {
		unsigned int idx = (q->used_idx++) & (q->ring_size - 1);

		q->vring[idx].flags = d[start++ & mask].flags & ~DESC_HW;
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
static inline void tp4q_write_header_headroom(struct tpacket4_hdr *hdr,
					      u32 size, u32 headroom)
{
	hdr->data = TPACKET4_HDRLEN + headroom;
	hdr->data_end = hdr->data + size;
}

/**
 *
 **/
static inline void tp4q_write_header(struct tp4_queue *tp4q,
				     struct tpacket4_hdr *hdr, u32 size)
{
	tp4q_write_header_headroom(hdr, size, tp4q->umem->data_headroom);
}

/**
 *
 **/
static inline
struct tpacket4_hdr *tp4q_get_header_from_headroom(void *data)
{
	return (struct tpacket4_hdr *)(data - TPACKET4_HDRLEN);
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

/**
 *
 **/
static inline int tp4q_get_dma_page_offset(struct tp4_queue *tp4q, u64 addr,
					   dma_addr_t *dma, struct page **page,
					   u32 *page_offset)
{
	u64 pg, off;

	if (!tp4_get_page_offset(tp4q, addr, &pg, &off))
		return -1;

	*dma = tp4q->dma_info[pg].dma;
	*page = tp4q->dma_info[pg].page;
	*page_offset = off + tp4q->umem->header_headroom +
		       TPACKET4_HDRLEN + tp4q->umem->data_headroom;

	return 0;
}

/**
 *
 **/
static inline int tp4q_get_header_dma(struct tp4_queue *tp4q, u64 addr,
				      dma_addr_t *dma)
{
	u64 pg, off;

	if (!tp4_get_page_offset(tp4q, addr, &pg, &off)) {
		*dma = 0;
		return -1;
	}

	*dma = tp4q->dma_info[pg].dma + off + tp4q->umem->header_headroom;
	return 0;
}

/**
 *
 **/
static inline unsigned int tp4q_get_data_headroom(struct tp4_queue *tp4q)
{
	return tp4q->umem->data_headroom;
}

/**
 * tp4a_packet_array_new - Create new packet array
 * @tp4q: The tp4_queue object to be associated with this packet array
 * @elems: number of elements in the packet array
 *
 * Returns a reference to the new packet array or NULL for failure
 **/
static inline
struct tp4_packet_array *
tp4a_packet_array_new(struct tp4_queue *tp4q,
		      size_t elems)
{
	struct tp4_packet_array *arr;

	if (!is_power_of_2(elems))
		return NULL;

	arr = kzalloc(sizeof(*arr) + elems * sizeof(struct tpacket4_desc),
		      GFP_KERNEL);
	if (!arr)
		return NULL;

	arr->tp4q = tp4q;
	arr->mask = elems - 1;
	return arr;
}

/**
 * tp4a_num_items - Number of packets in array
 * @a: pointer to packet array
 *
 * Returns the number of packets currently in the array
 **/
static inline
u32 tp4a_num_items(struct tp4_packet_array *a)
{
	return a->end - a->curr;
}

/**
 * tp4a_packet_array_free - Destroy packet array
 * @a: pointer to packet array
 **/
static inline
void tp4a_packet_array_free(struct tp4_packet_array *a)
{
	kfree(a);
}

/**
 * tp4a_populate - Populate an array with packets from associated tp4q
 * @a: pointer to packet array
 **/
static inline
void tp4a_populate(struct tp4_packet_array *a)
{
	u32 cnt, free = a->mask + 1 - (a->end - a->start);

	if (free == 0)
		return; /* no space! */

	cnt = tp4q_dequeue_to_ring(a->tp4q, &a->items[0], a->end, free,
				   a->mask);
	a->end += cnt;
}

/**
 * tp4a_discard - Drop all packets in packet array up to current packet
 * @a: pointer to packet array
 **/
static inline
void tp4a_discard(struct tp4_packet_array *a)
{
	a->start = a->curr;
}

/**
 * tp4a_reset - Start to traverse the packets in the array from the beginning
 * @a: pointer to packet array
 **/
static inline
void tp4a_reset(struct tp4_packet_array *a)
{
	a->curr = a->start;
}

/**
 * tp4a_end - Skip traversing rest of packets, but do not drop them
 * @a: pointer to packet array
 **/
static inline
void tp4a_end(struct tp4_packet_array *a)
{
	a->curr = a->end;
}

/**
 * tp4a_flush - Flush processed packets to associated tp4q
 * @a: pointer to packet array
 *
 * Returns 0 for success and -1 for failure
 **/
static inline
int tp4a_flush(struct tp4_packet_array *a)
{
	u32 avail = a->curr - a->start;
	int ret;

	if (avail == 0)
		return 0; /* nothing to flush */

	ret = tp4q_enqueue_from_ring(a->tp4q, &a->items[0], a->start, avail,
				     a->mask);
	if (ret < 0)
		return -1;

	tp4a_discard(a);
	return 0;
}

/**
 * tp4a_next_packet - Get next packet in array
 * @a: pointer to packet array
 * @p: supplied pointer to packet structure that is filled in by function
 *
 * Returns 0 for success and -1 for failure
 **/
static inline
int tp4a_next_packet(struct tp4_packet_array *a, struct tp4_packet *p)
{
	u32 avail = a->end - a->curr;

	if (avail == 0)
		return -1; /* empty */

	p->pkt_arr = a;
	p->start = a->curr;
	p->curr = a->curr;
	p->end = a->curr;

	/* XXX Sanity check for too-many-frames packets? */
	while (a->items[p->end++ & a->mask].flags & DESC_NEXT) {
		avail--;
		if (avail == 0)
			return -1;
	}

	a->curr += (p->end - p->start);
	return 0;
}


/**
 * tp4a_next_packet_populate - Get next packet and populate array if empty
 * @a: pointer to packet array
 * @p: supplied pointer to packet structure that is filled in by function
 *
 * Returns 0 for success and -1 for failure
 **/
static inline
int tp4a_next_packet_populate(struct tp4_packet_array *a, struct tp4_packet *p)
{
	int err;

	err = tp4a_next_packet(a, p);
	if (err) {
		tp4a_discard(a);
		tp4a_populate(a);
		err = tp4a_next_packet(a, p);
	}

	return err;
}

/**
 * tp4a_add_frame - Add a frame to a packet array
 * @a: pointer to packet array
 * @addr: index of in packet buffer that this frame should point to
 * @len: the length in bytes of the data in the frame
 * @errno: >0 if an errno should be returned, otherwise 0
 * @is_last_frame: Set if this is the last frame of the packet
 *
 * Returns 0 for success and -1 for failure
 **/
static inline
int tp4a_add_frame(struct tp4_packet_array *a,
		   u64 addr, u32 len, int errno, bool is_last_frame)
{
	u32 free = a->mask + 1 - (a->end - a->start);

	if (free == 0)
		return -1; /* full */

	a->items[a->end & a->mask].addr = addr;
	a->items[a->end & a->mask].len = len;
	a->items[a->end & a->mask].flags = is_last_frame ? 0 : DESC_NEXT;
	a->items[a->end & a->mask].error = errno;

	a->end++;
	return 0;
}

/**
 * tp4a_flush_frame - Adds frame and flushes it to associated tp4q
 * @a: pointer to packet array
 * @addr: index of in packet buffer that this frame should point to
 * @len: the length in bytes of the data in the frame
 * @errno: >0 if an errno should be returned, otherwise 0
 * @is_last_frame: Set if this is the last frame of the packet
 *
 * Returns 0 for success and -1 for failure
 **/
static inline
int tp4a_flush_frame(struct tp4_packet_array *a,
		     u64 addr, u32 len, int errno, bool is_last_frame)
{
	struct tpacket4_desc desc;
	int err;

	desc.addr = addr;
	desc.len = len;
	desc.flags = is_last_frame ? 0 : DESC_NEXT;
	desc.error = errno;

	err = tp4q_enqueue(a->tp4q, &desc, 1);
	return err;
}

/* Packet operations; One packet is one or more frames. */

/**
 * tp4p_reset - Start to traverse the frames in the packet from the beginning
 * @p: pointer to packet
 **/
static inline
void tp4p_reset(struct tp4_packet *p)
{
	p->curr = p->start;
}

/**
 * tp4p_last - Go to last frame in packet
 * @p: pointer to packet
 **/
static inline
void tp4p_last(struct tp4_packet *p)
{
	p->curr = p->end - 1;
}

/**
 * tp4p_next_frame - Go to next frame in packet
 * @p: pointer to packet
 *
 * Returns -1 if there are no more frames in the packet, otherwise 0
 **/
static inline
int tp4p_next_frame(struct tp4_packet *p)
{
	if (p->curr + 1 == p->end)
		return -1;

	p->curr++;
	return 0;
}

/**
 * tp4p_prev_frame - Go to previous frame in packet
 * @p: pointer to packet
 *
 * Returns 1 if there is no earlier frame in this packet, otherwise 0
 **/
static inline
int tp4p_prev_frame(struct tp4_packet *p)
{
	if (p->curr == p->start)
		return -1;

	p->curr--;
	return 0;
}

/**
 * tp4p_get_frame_id - Get packet buffer id of frame
 * @p: pointer to packet
 *
 * Returns the id of the packet buffer of the current frame
 **/
static inline
u64 tp4p_get_frame_id(struct tp4_packet *p)
{
	return p->pkt_arr->items[p->curr & p->pkt_arr->mask].addr;
}

/**
 * tp4p_get_frame_len - Get length of data in current frame
 * @p: pointer to packet
 *
 * Returns the length of data in the packet buffer of the current frame
 **/
static inline
u32 tp4p_get_frame_len(struct tp4_packet *p)
{
	return p->pkt_arr->items[p->curr & p->pkt_arr->mask].len;
}

/**
 * tp4p_set_error - Set an error on the current frame
 * @p: pointer to packet
 * @errno: the errno to be assigned.
 **/
static inline
void tp4p_set_error(struct tp4_packet *p, int errno)
{
	p->pkt_arr->items[p->curr & p->pkt_arr->mask].error = errno;
}

/**
 * tp4p_is_last_frame - Is this the last frame of the packet
 * @p: pointer to packet
 *
 * Returns true if this is the last frame of the packet, otherwise 0
 **/
static inline
bool tp4p_is_last_frame(struct tp4_packet *p)
{
	return p->curr + 1 == p->end;
}

/**
 * tp4p_num_frames - Number of frames in a packet
 * @p: pointer to packet
 *
 * Returns the number of frames this packet consists of
 **/
static inline
u32 tp4p_num_frames(struct tp4_packet *p)
{
	return p->end - p->start;
}

/**
 * tp4a_add_packet - Adds a packet to a packet array
 * @a: pointer to packet array
 * @p: pointer to packet to insert
 * @errno: >0 if packet should be marked with an errno, otherwise 0
 *
 * Returns 0 for success and -1 for failure
 **/
static inline
int tp4a_add_packet(struct tp4_packet_array *a, struct tp4_packet *p, int errno)
{
	u32 free = a->mask + 1 - (a->end - a->start);
	u32 nframes = p->end - p->start;

	if (nframes > free)
		return -1;

	tp4p_reset(p);

	do {
		a->items[a->end & a->mask].addr = tp4p_get_frame_id(p);
		a->items[a->end & a->mask].len = tp4p_get_frame_len(p);
		a->items[a->end & a->mask].flags = tp4p_is_last_frame(0) ?
						   0 : DESC_NEXT;
		a->items[a->end & a->mask].error = errno;
		a->end++;
	} while (tp4p_next_frame(p) == 0);

	return 0;
}

#endif /* _LINUX_TPACKET4_H */
