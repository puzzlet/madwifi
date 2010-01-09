/* This file is used for a trick to determine the version of the kernel
 * build tree. Simply grepping <linux/version.h> doesn't work, since
 * some distributions have multiple UTS_RELEASE definitions in that
 * file.
 * Taken from the lm_sensors project.
 *
 * $Id$
 */
#include <linux/version.h>

#ifndef UTS_RELEASE
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,33)
/* Linux 2.6.33+ uses <generated/utsrelease.h> */
#include <generated/utsrelease.h>
#else
/* Linux 2.6.18 - 2.6.32 uses <linux/utsrelease.h> */
#include <linux/utsrelease.h>
#endif
#endif

char *uts_release = UTS_RELEASE;
