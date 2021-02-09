#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#include "usart.h"

int _write(int file, char *data, int len)
{
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
  {
    errno = EBADF;
    return -1;
  }

  HAL_StatusTypeDef status;
  status = HAL_UART_Transmit(&huart1, (uint8_t*) data, len, 100);
  return (status == HAL_OK ? len : 0);
}
