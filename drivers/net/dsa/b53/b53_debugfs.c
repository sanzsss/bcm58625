/*
 * B53 switch driver debugfs support
 *
 * Copyright (C) 2017 Broadcom Limited
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/types.h>
#include <linux/debugfs.h>

#include "b53_priv.h"

static struct dentry *b53_dbg_root;

static char b53_dbg_reg_ops_buf[256] = "";

/* b53_dbg_reg_ops_read - read for reg_ops datum
 * @filp: the opened file
 * @buffer: where to write the data for the user to read
 * @count: the size of the user's buffer
 * @ppos: file position offset
 */
static ssize_t b53_dbg_reg_ops_read(struct file *filp, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	char *buf;
	int len;

	/* Don't allow partial reads */
	if (*ppos != 0)
		return 0;

	buf = kasprintf(GFP_KERNEL, "%s\n",
			b53_dbg_reg_ops_buf);
	if (!buf)
		return -ENOMEM;

	if (count < strlen(buf)) {
		kfree(buf);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));

	kfree(buf);
	return len;
}

/* b53_dbg_reg_ops_write - write into reg_ops datum
 * @filp: the opened file
 * @buffer: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 */
static ssize_t b53_dbg_reg_ops_write(struct file *filp,
				     const char __user *buffer,
				     size_t count, loff_t *ppos)
{
	struct b53_device *dev = filp->private_data;
	int len;
	u32 reg, value;
	char *pbuf, *token;
	int rc_reg, rc_val;

	/* Don't allow partial writes */
	if (*ppos != 0)
		return 0;
	if (count >= sizeof(b53_dbg_reg_ops_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(b53_dbg_reg_ops_buf,
				     sizeof(b53_dbg_reg_ops_buf) - 1,
				     ppos,
				     buffer,
				     count);
	if (len < 0)
		return len;

	b53_dbg_reg_ops_buf[len] = '\0';

	pbuf = b53_dbg_reg_ops_buf;
	token = strsep(&pbuf, " ");
	if (!token)
		goto error;

	/* Cmds supported:
	 *
	 *   Write: "write <0xREG> <0xVALUE>
	 *   Read: "read <0xREG>
	 */
	if (strncmp(token, "write", 5) == 0) {
		token = strsep(&pbuf, " ");
		if (!token)
			goto error;
		rc_reg = kstrtou32(token, 0, &reg);

		token = strsep(&pbuf, " ");
		if (!token)
			goto error;
		rc_val = kstrtou32(token, 0, &value);

		if (rc_reg || rc_val) {
			goto error;
		} else {
			b53_write32(dev, reg >> 24,
				    (reg >> 16) & 0xFF, value);
			sprintf(b53_dbg_reg_ops_buf, "%d", 0);
		}
	} else if (strncmp(token, "read", 4) == 0) {
		token = strsep(&pbuf, " ");
		if (!token)
			goto error;

		rc_reg = kstrtou32(token, 0, &reg);
		if (rc_reg) {
			goto error;
		} else {
			b53_read32(dev, reg >> 24, (reg >> 16) & 0xFF, &value);
			sprintf(b53_dbg_reg_ops_buf, "0x%x", value);
		}
	} else {
		goto error;
	}

	return count;

error:
	pr_info("Incorrect command format\n");
	pr_info("Available commands:\n");
	pr_info("   read <0xREG 32bit>\n");
	pr_info("   write <0xREG 32bit> <0xVALUE 32bit>\n");
	sprintf(b53_dbg_reg_ops_buf, "%d", -1);
	return count;
}

static const struct file_operations b53_dbg_reg_ops_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read =  b53_dbg_reg_ops_read,
	.write = b53_dbg_reg_ops_write,
};

/* b53_dbg_init - start up debugfs for the driver
 * @dev: device
 */
void b53_dbg_init(struct b53_device *dev)
{
	struct dentry *pfile;

	b53_dbg_root = debugfs_create_dir(dev->dev->driver->name, NULL);

	if (!b53_dbg_root) {
		pr_err("b53_dbg_init: init of debugfs failed\n");
	} else {
		pfile = debugfs_create_file("reg_ops", 0600,
					    b53_dbg_root, dev,
					    &b53_dbg_reg_ops_fops);
		if (!pfile)
			pr_err("reg_ops %s failed\n", dev->dev->driver->name);
	}
}

/* b53_dbg_exit - clean out the drivers debugfs entries
 */
void b53_dbg_exit(void)
{
	debugfs_remove_recursive(b53_dbg_root);
	b53_dbg_root = NULL;
}
