/*
 * The map the navigation is given (docs/map-navigation.md): a grid of the space around the drone, in the map's own
 * axes, each cell unknown, free or occupied. A mapper on the ground (Room Survey, the simulator) makes it; it comes
 * to the drone as a file (map.dmp, file_rx.h) and the navigation looks things up in it: what a point is, and how far
 * the drone can go along a line before it would touch something.
 *
 * The file, little-endian:
 *   "DFMP"  u32 version (1)  f32 resolution [m]  f32 origin[3] [m] (the corner of cell 0,0,0)  u32 nx ny nz
 *   runs of cells, x fastest, then y, then z: a kind byte, then the run's length as an unsigned LEB128 (7 bits a
 *     byte, low first, the top bit set on all but the last). Kinds: 0 unknown, 1 free, 2 occupied (that many cells of
 *     it); 3 each as the cell a row back (y − 1), 4 each as the cell a layer back (z − 1): walls and floors repeat, so
 *     a room comes to a few hundred bytes
 *   u32 CRC-32 of everything before it (rn_crc32)
 *
 * On loading, the occupied cells are grown by the drone's radius (with its margin): a cell within that distance of
 * one is taken as occupied too, so the look-ups can treat the drone as a point. Growing is by a box (a little more
 * along the diagonals: on the safe side).
 *
 * Portable C without the C library (the simulator's boards are freestanding); a fixed store (MAP_MAX_CELLS).
 */
#ifndef MAP_CORE_H
#define MAP_CORE_H
#include <stdint.h>

#define MAP_MAX_CELLS 524288u               /* the most cells a map may have (e.g. 10 × 10 × 3 m at 10 cm: 300 000) */
#define MAP_VERSION 1
enum { MAP_UNKNOWN = 0, MAP_FREE = 1, MAP_OCC = 2 };

typedef struct {
  int ok;                                   /* a map is loaded */
  float res, origin[3]; int n[3];
  float radius;                             /* what the occupied cells were grown by [m] */
  uint32_t crc;                             /* the file's CRC (which map this is) */
  uint32_t n_occ, n_free, n_unknown;        /* cells of each state, as sent */
  uint8_t raw[MAP_MAX_CELLS / 4];           /* the cells as sent, 2 bits each */
  uint8_t grown[MAP_MAX_CELLS / 4];         /* occupied grown by the radius */
} map_grid;

/* A map file. 0, or −1 with why (what's wrong) filled. radius: the drone's, with its margin [m]. Keeps the map it had
 * on a bad file. */
int map_load(map_grid *M, const uint8_t *buf, uint32_t len, float radius, char *why, int wn);
/* Grow the occupied cells again for another radius (the map as sent is kept). */
void map_grow(map_grid *M, float radius);
/* The state at a point of the map [m] (outside it: unknown); grown: with the radius. */
int map_at(const map_grid *M, const float p[3], int grown);
/* Along the unit direction d from p (both in the map's axes), up to maxd [m]: how far until the grown map is
 * occupied (*d_occ) and until it is unknown (*d_unk), each maxd if nothing is met. From a point already inside a grown
 * cell, d_occ is 0. */
void map_look(const map_grid *M, const float p[3], const float d[3], float maxd, float *d_occ, float *d_unk);

/* Write a map file from cells (one state byte each, x fastest), for the tests and the tools: its length, or 0 if it
 * doesn't fit cap. */
uint32_t map_encode(uint8_t *out, uint32_t cap, float res, const float origin[3], const int n[3], const uint8_t *cells);

#endif
