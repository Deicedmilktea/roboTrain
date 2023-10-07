# Uart收发数据练习
* 使用阻塞性uart发送接收串口助手数据
  ```C++
  // 接收数据
    HAL_UART_Receive(&huart2, rxData, sizeof(rxData), 1000);

    // 检查接收到的数据
    if (strcmp((char *)rxData, "0\r\n") == 0)
    {
      // 如果接收到"0\r\n"，发送"Hello"
      uint8_t txData[] = "Hello\r\n";
      HAL_UART_Transmit(&huart2, txData, sizeof(txData), 1000);
    }
    else if (strcmp((char *)rxData, "1\r\n") == 0)
    {
      // 如果接收到"1\r\n"，发送"World"
      uint8_t txData[] = "World\r\n";
      HAL_UART_Transmit(&huart2, txData, sizeof(txData), 1000);
    }

    // 清空接收缓冲区
    memset(rxData, 0, sizeof(rxData));
  ```

* 使用中断uart的方式接收串口助手的数据并使用串口助手点灯
  ```C++
  //中断回调函数
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
      // 处理接收到的数据
      if (huart2.Instance->DR == '1') {
        // 点亮LED
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
      } else if (huart2.Instance->DR == '0') {
        // 熄灭LED
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
      }
      // 启动下一次接收
      HAL_UART_Receive_IT(&huart2, (uint8_t *)&huart2.Instance->DR, 1);
    }
  }

  /*****中断接收******/
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&huart2.Instance->DR, 1);
  ```

* 使用dma的方式接收串口助手的数据并使用串口助手点灯
  ```C++
  //DMA回调函数
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // 处理接收到的数据
        if (strcmp((char*)rxData, "1\r\n") == 0) {
            // 点亮LED
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        }else if (strcmp((char*)rxData, "0\r\n") == 0) {
            // 熄灭LED
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        }
        // 清空接收缓冲区
        memset(rxData, 0, sizeof(rxData));
        // 启动下一次DMA接收
        HAL_UART_Receive_DMA(&huart2, rxData, sizeof(rxData));
    }
  }

  /******DMA*******/
  HAL_UART_Receive_DMA(&huart2, rxData, sizeof(rxData));
  ```