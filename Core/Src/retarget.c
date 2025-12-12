// 標準輸入輸出結合 UART 傳輸

// 法 1 : 修改 fputc()、fgetc() 

#include <retarget.h>
#include <stdio.h>
#include <errno.h>

// 文件描述符 (defined by system call)
#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

UART_HandleTypeDef *gHuart;
UART_SYNC_MODE current_uart_sync_mode = UART_POLLING;

void RetargetInit(UART_HandleTypeDef *huart) {
    gHuart = huart;

    // 定義 stream 如何緩衝。
    // STM32CubeIDE 自動產生的預設 syscalls.c 檔案在啟用輸入流的內部快取時會導致意外行為，因此這裡停用快取。
    // https://forum.digikey.com/t/stm32-scanf/37746#:~:text=STM32CubeIDE%20%E8%87%AA%E5%8B%95%E7%94%A2%E7%94%9F%E7%9A%84%E9%A0%90%E8%A8%AD%20syscalls.c%20%E6%AA%94%E6%A1%88%E5%9C%A8%E5%95%9F%E7%94%A8%E8%BC%B8%E5%85%A5%E6%B5%81%E7%9A%84%E5%85%A7%E9%83%A8%E5%BF%AB%E5%8F%96%E6%99%82%E6%9C%83%E5%B0%8E%E8%87%B4%E6%84%8F%E5%A4%96%E8%A1%8C%E7%82%BA
    // 當設置為無緩衝 (_IONBF) 時 buffer 與 size 忽略。
    setvbuf(stdout, NULL, _IONBF, 0);
}

// call by printf()
int _write(int file, char *ptr, int len)
{
    HAL_StatusTypeDef hstatus;
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        
        if (current_uart_sync_mode == UART_POLLING) {
            // Polling 模式
            hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
            if (hstatus == HAL_OK) {
                return len;
            } else {
                return EIO; // EIO is defined in errno.h of glibc
            }
        } else if (current_uart_sync_mode == UART_INTERRUPT) {
            // Interrupt 模式：呼叫 Ring Buffer 寫入函數
            while ((HAL_UART_GetState(gHuart) == HAL_UART_STATE_BUSY_TX) || (HAL_UART_GetState(gHuart) == HAL_UART_STATE_BUSY_TX_RX));
            hstatus = HAL_UART_Transmit_IT(gHuart, (uint8_t *) ptr, len);
            if (hstatus == HAL_OK) {
                return len;
            } else {
                return EIO; // EIO is defined in errno.h of glibc
            }
        } else if (current_uart_sync_mode == UART_DMA) {
            // DMA 模式：呼叫 Ring Buffer 寫入函數
            hstatus = HAL_UART_Transmit_DMA(gHuart, (uint8_t *) ptr, len);
            if (hstatus == HAL_OK) {
                return len;
            } else {
                return EIO; // EIO is defined in errno.h of glibc
            }
        } else {
            // 未知的 UART 傳輸模式配置！請檢查 retarget.h
            return -1;
        }
    }

    errno = EBADF;
    return -1;
}

// call by scanf()
int _read(int file, char *ptr, int len)
{
    HAL_StatusTypeDef hstatus;
    if (file == STDIN_FILENO) {

        if (current_uart_sync_mode == UART_POLLING) {
            // Polling 模式
            hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
            if (hstatus == HAL_OK) {
                return 1;
            } else {
                return EIO; // EIO is defined in errno.h of glibc
            }
        } else if (current_uart_sync_mode == UART_INTERRUPT) {
            // Interrupt 模式：呼叫 Ring Buffer 寫入函數
            hstatus = HAL_UART_Receive_IT(gHuart, (uint8_t *) ptr, 1);
            if (hstatus == HAL_OK) {
                return 1;
            } else {
                return EIO; // EIO is defined in errno.h of glibc
            }
        } else if (current_uart_sync_mode == UART_DMA) {
            // DMA 模式：呼叫 Ring Buffer 寫入函數
            hstatus = HAL_UART_Receive_DMA(gHuart, (uint8_t *) ptr, 1);
            if (hstatus == HAL_OK) {
                return 1;
            } else {
                return EIO; // EIO is defined in errno.h of glibc
            }
        } else {
            // 未知的 UART 傳輸模式配置！請檢查 retarget.h
            return -1;
        }
    }

    errno = EBADF;
    return -1;
}
