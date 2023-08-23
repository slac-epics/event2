#ifndef __FIDMATH_H_
#define __FIDMATH_H_
/*
 * A few fiducial helper definitions.
 * FID_ROLL(a, b) is true if we have rolled over from fiducial a to fiducial b.  (That is, a
 * is large, and b is small.)
 * FID_GT(a, b) is true if fiducial a is greater than fiducial b, accounting for rollovers.
 * FID_DIFF(a, b) is the difference between two fiducials, accounting for rollovers.
 */
#define FID_MAX        0x1ffe0
#define FID_INVALID    0x1ffff
#define FID_ROLL_LO    0x00200
#define FID_ROLL_HI    (FID_MAX-FID_ROLL_LO)
#define FID_ROLL(a,b)  ((b) < FID_ROLL_LO && (a) > FID_ROLL_HI)
#define FID_GT(a,b)    (FID_ROLL(b, a) || ((a) > (b) && !FID_ROLL(a, b)))
#define FID_DIFF(a,b)  ((FID_ROLL(b, a) ? FID_MAX : 0) + (int)(a) - (int)(b) - (FID_ROLL(a, b) ? FID_MAX : 0))
#endif
