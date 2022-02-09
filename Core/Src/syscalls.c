#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#include "main.h"
#include "usart.h"

int _write(int file, char *data, int len)
{
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
  {
    errno = EBADF;
    return -1;
  }

  HAL_StatusTypeDef status;

  if (BOARD == BOARD_FEATHER) {
    status = HAL_UART_Transmit(&huart3, (uint8_t*) data, len, 10);
  } else {
    status = HAL_UART_Transmit(&huart1, (uint8_t*) data, len, 10);
  }
  return (status == HAL_OK ? len : 0);
}
