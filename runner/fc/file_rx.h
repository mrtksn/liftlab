/*
 * Files up the radio, as text commands (rc_core.h RC_CMD_TEXT): how LiftLab's drone card puts the design's files on
 * the drone's Pi (dfb_pi writes them into its folder; docs/pi-suite.md), and the simulator's the same way. Each line
 * fits one text command (57 characters):
 *   file NAME SIZE CRC   a file begins: NAME (letters, digits, - _ and ., no folders; at most FILE_RX_NAME
 *                        characters), SIZE bytes (at most FILE_RX_MAX), CRC its CRC-32 in hex (rn_crc32)
 *   fd OFF DATA          a piece: DATA base64, FILE_RX_PIECE bytes (the last piece fewer) at byte OFF, a multiple of
 *                        FILE_RX_PIECE. Pieces come in any order, again or not at all; they say nothing back
 *   file end             it's all sent: whole and its CRC right, the file is done; or which pieces are missing (the
 *                        first few: the answer fits one radio message, FILE_RX_SAY; send them and end again)
 *   file                 where it stands
 * One file at a time: a new `file` drops the one before. The text link is reliable over a packet link (plink) but
 * not over an ExpressLRS module, so `file end` lists the missing pieces for the sender to send again.
 * No C library (the simulator's boards are freestanding).
 */
#ifndef FILE_RX_H
#define FILE_RX_H
#include <stdint.h>

#define FILE_RX_MAX 32768           /* [bytes] the largest file */
#define FILE_RX_PIECE 36            /* [bytes] a piece: 48 base64 characters, "fd 32760 " before them */
#define FILE_RX_NAME 31
#define FILE_RX_SAY 56              /* [characters] the missing list's answer at most: one message down (tlm_core.h TLM_TEXT) */

typedef struct {
  char name[FILE_RX_NAME + 1];
  uint32_t size, crc, n_got;           /* the file's size, its CRC-32, the pieces here */
  int active;                          /* 1: a file is coming; 2: it's done (buf, size, name) */
  uint8_t got[FILE_RX_MAX / FILE_RX_PIECE / 8 + 1];
  uint8_t buf[FILE_RX_MAX];
} file_rx;

/* A line. Returns 0 if it isn't one of these (reply untouched); 1 taken (reply set; empty for a piece); 2 the file is
 * whole and its CRC right (R->name, R->buf, R->size; reply says so): the caller keeps it. */
int file_rx_line(file_rx *R, const char *line, char *reply, int cap);
/* The pieces a file has: (size + PIECE − 1) / PIECE. */
uint32_t file_rx_pieces(uint32_t size);
/* A name this takes (letters, digits, - _ ., not starting with a dot): 1, or 0. */
int file_rx_name_ok(const char *s);
/* A `file NAME SIZE CRC` line read (before file_rx_line takes it, to answer that the same file is already here): 1,
 * or 0 if the line isn't one. */
int file_rx_begins(const char *line, char *name, uint32_t *size, uint32_t *crc);

/* Files down the radio, the other way (LiftLab's drone card reads the drone's files: Compare with the drone), one
 * radio message (tlm_core.h TLM_TEXT, 56 characters) a piece:
 *   fget NAME              the drone answers "fget NAME: SIZE CRC" (CRC-32 in hex), or that it has no such file
 *   fget NAME OFF [N]      N pieces (1 to FILE_TX_N) from byte OFF, a multiple of FILE_TX_PIECE, each "fg:OFF:DATA"
 *                          (DATA base64, FILE_TX_PIECE bytes, the last piece fewer), separated by spaces. No space in
 *                          a piece: a long answer is cut into messages at its spaces (tlm_core.c), a piece each
 * The reader asks for what it's missing again: messages over an ExpressLRS module may not all arrive. */
#define FILE_TX_PIECE 33            /* [bytes] a piece down: 44 base64 characters, "fg:32736:" before them: 53 */
#define FILE_TX_N 3                 /* pieces an answer: three messages (dfb_pi sends at most 3 of a reply) */
/* The pieces of b (n bytes) from off, at most count (≤ FILE_TX_N), into out: how many, 0 past the end, or −1 if off
 * isn't a piece's or out is too small. */
int file_tx_pieces(const uint8_t *b, uint32_t n, uint32_t off, int count, char *out, int cap);

#endif
