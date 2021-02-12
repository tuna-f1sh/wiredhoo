#include <stdio.h>
#include "utilities.h"

void print_hex(char *string, size_t len) {
  unsigned char *p = (unsigned char *) string;

  for (int i = 0; i < len; i++) {
    /* if (! (i % 16) && i) */
    /*   printf("\r\n"); */

    printf("0x%02x ", p[i]);
  }

  printf("\r\n");
}

uint8_t calculate_crc(uint8_t *buffer, size_t len) {
  uint8_t crc = 0;

  for (int i = 0; i < len; i++)
    crc ^= buffer[i];

  return crc;
}
