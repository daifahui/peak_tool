#include <assert.h>
#include <stm32f0xx_hal.h>

#define TIM_BUS_FREQ (48000000)
#define TIM_US_PRESCALER ( ( TIM_BUS_FREQ / 1000000u ) - 1u )

static uint32_t us64_last;
static uint64_t us64_total;

void pcan_timestamp_init( void )
{

  switch( TIM_BUS_FREQ )
  {
    case 48000000:
      /* TIM3 on APB1 bus */
      __HAL_RCC_TIM3_CLK_ENABLE();

      TIM3->PSC = (2048-1); /* => tick = 42.666uS */
      /* set clock division to zero: */
      TIM3->CR1 &= (uint16_t)(~TIM_CR1_CKD);
      TIM3->CR1 |= TIM_CLOCKDIVISION_DIV1;
      /* enable timer */
      TIM3->CR1 |= TIM_CR1_CEN;

      /* TIM2: 32 位、1 MHz 自由运行计数器，用于 µs 时间戳。
       * 不产生中断，需要时直接读 CNT；约 71.6 分钟回绕一次。 */
      __HAL_RCC_TIM2_CLK_ENABLE();

      TIM2->PSC = TIM_US_PRESCALER;
      TIM2->CR1 = TIM_CLOCKDIVISION_DIV1;
      TIM2->EGR = TIM_EGR_UG;      /* 立即装载预分频值 */
      TIM2->SR = 0;
      TIM2->CR1 |= TIM_CR1_CEN;

      us64_last = TIM2->CNT;
      us64_total = 0;
    break;
    default:
      assert( 0 );
    break;
  }
}

uint16_t pcan_timestamp_millis( void )
{
  return (HAL_GetTick()&0xFFFF);
}

uint16_t pcan_timestamp_ticks( void )
{
  /* 1 pcan tick => 42.666 us */
  return TIM3->CNT;
}

uint32_t pcan_timestamp_us( void )
{
  /* TIM2 以 1 MHz 计数，1 count = 1 µs，直接读寄存器 */
  return TIM2->CNT;
}

uint64_t pcan_timestamp_us64( void )
{
  uint32_t now = TIM2->CNT;

  us64_total += (uint32_t)( now - us64_last );
  us64_last = now;

  return us64_total;
}
