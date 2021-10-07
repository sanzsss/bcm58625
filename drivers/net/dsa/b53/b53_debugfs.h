/*
 * B53 switch driver debugfs support
 *
 * Copyright (C) 2017 Broadcom Limited
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_DEBUG_FS
void b53_dbg_init(struct b53_device *dev);
void b53_dbg_exit(void);
#else
static inline void b53_dbg_init(struct b53_device *dev) {}
static inline void b53_dbg_exit(void) {}
#endif /* CONFIG_DEBUG_FS */
