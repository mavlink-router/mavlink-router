#pragma once

#pragma GCC diagnostic push
/*
 * All structs in mavlink are packed since they are "on-the-wire" definition,
 * but then pymavlink generates helper functions that take the address of
 * members of those structs - that's not good, but there's nothing we can do,
 * so shut up the warnings.
 *
 * Unaligned access is generally slower, but should work in most of today
 * platforms, so this is not harmful. Also, pymavlink supposedly uses just
 * memcpy() to access the fields so, but if you have a variant of sparc,
 * mips, arm, etc that doesn't silently handle unaligned access,
 * you may be interested checking case by case in mavlink/pymavlink.
 */
#if defined(HAVE_WADDRESS_OF_PACKED_MEMBER) && HAVE_WADDRESS_OF_PACKED_MEMBER
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif
#include <mavlink.h>
#pragma GCC diagnostic pop

