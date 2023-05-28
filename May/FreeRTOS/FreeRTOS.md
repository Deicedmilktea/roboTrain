# 使用FreeRTOS点灯
## 主要代码
```C++
//红灯闪烁task
void LEDRTask(void const * argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_11);
    osDelay(500);
  }
}
//绿灯闪烁task
void LEDGTask(void const * argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
    osDelay(500);
  }
}
```