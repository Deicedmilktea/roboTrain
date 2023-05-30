# KEY控制开关，切换颜色
## 控制开关
```C++
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    while(1){
        //当开关处于未按下状态，将灯关闭
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0){
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
        }
        //当开关处于按下状态，将灯开启
        else{
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
        }
    }
}
```
## 切换颜色
```C++
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    while(1){
        //当开关处于未按下状态，将红灯开启，绿灯关闭
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)==0){
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
        }
        //当开关处于按下状态，将红灯关闭，绿灯开启
        else{
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
        }
    }
}
```