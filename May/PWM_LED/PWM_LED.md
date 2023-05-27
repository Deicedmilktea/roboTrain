  # PWM呼吸灯
  ## 主要代码
  ```C++
while (1)
{
    HAL_Delay(10);
    //占空比从0依次增大到100%，实现效果为led灯逐渐变亮
    for(uint16_t i=0;i<500;i++){
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,i);
        HAL_Delay(1);
    }
    //占空比从100%依次减小到0，实现效果为led灯逐渐变暗
    for(uint16_t i=499;i>0;i++){
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,i);
        HAL_Delay(1);
    }
}
  ```
## 分析
  占空比计算\
  PWM频率=$\frac{72000000}{72*500}$=2000Hz=2kHz\
  2kHz频率周期更快，人眼难以分辨，实现效果更自然

  通过调节占空比来控制灯的亮度，加上循环，让亮度依次变化，可以实现呼吸灯的效果