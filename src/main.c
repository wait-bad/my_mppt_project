/**
  **************************************************************************
  * @file     main.c
  * @version  v2.0.9
  * @date     2022-09-28
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "at32f421_board.h"

#include "at32f421_clock.h"

#include "at32f421_dma.h"
#include "at32f421_dma.c"
#include "at32f421_adc.h"
#include "at32f421_adc.c"
#include "mean_processing.h"


/*************system_value*************/

uint16_t time_cnt = 0;
__IO uint16_t adc1_ordinary_value = 0;

__IO uint16_t adc1_ordinary_valuetab[3] = {0};
uint16_t ad_value[3]={0};
__IO uint16_t vmor_flag_index = 0;


// ctrl_parameter ;
float set_voltage       = 0;
float set_current       = 0;
double voltage_cailbration  = 0.016918;
double current_cailbration  = 0.0058547;
int32_t constant            = 10000;//constant = 0.0001;
uint16_t set_Reload_value   = 1399; //  freq = 150k
uint16_t set_duty_max_value = 1299;    
uint16_t set_duty_min_value = 0;

// un_ctrl_parameter
uint32_t set_duty_voltage = 0;
uint32_t set_duty_currten = 0;
int32_t currten_duty      = 0;
int32_t voltage_duty      = 0;
uint32_t set_duty_max     = 0;
uint32_t set_duty_min     = 0;
double really_duty      = 0;
uint16_t now_current      = 0;

// flag
uint16_t tim_add_counter = 0;
uint16_t refresh_flag    = 0;
/**************************************/



/*******************function*******************/
void gpio_configuration(void);
void tim3_configuration(void);

static void dma_config(void);
static void adc_config(void);
static void adc_gpio_config(void);
void MPPT_PerturbObserve(void);
/**************************************/

uint16_t prescaler_value = 0;

void count_un_ctrl_parameter()
{
  set_duty_currten = 4000;
  set_duty_voltage = set_voltage / voltage_cailbration;
  set_duty_max     = set_duty_max_value*constant ; 
  set_duty_min     = set_duty_min_value*constant+constant/2;
}

/**
  * @brief  gpio configuration.
  * @param  none
  * @retval none
  */
static void adc_gpio_config(void)
{
  gpio_init_type gpio_initstructure;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_initstructure);
  gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
  gpio_initstructure.gpio_pins = GPIO_PINS_1 | GPIO_PINS_2;
  gpio_init(GPIOA, &gpio_initstructure);
}

/**
  * @brief  dma configuration.
  * @param  none
  * @retval none
  */
static void dma_config(void)
{
  dma_init_type dma_init_struct;
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 3;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)ad_value;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

/**
  * @brief  adc configuration.
  * @param  none
  * @retval none
  */
static void adc_config(void)
{
  adc_base_config_type adc_base_struct;
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_adc_clock_div_set(CRM_ADC_DIV_6);
  nvic_irq_enable(ADC1_CMP_IRQn, 0, 0);

  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = TRUE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 3;
  adc_base_config(ADC1, &adc_base_struct);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_239_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 2, ADC_SAMPLETIME_239_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 3, ADC_SAMPLETIME_239_5);
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);
  adc_voltage_monitor_enable(ADC1, ADC_VMONITOR_SINGLE_ORDINARY);
  adc_voltage_monitor_threshold_value_set(ADC1, 0xBBB, 0xAAA);
  adc_voltage_monitor_single_channel_select(ADC1, ADC_CHANNEL_5);
  adc_interrupt_enable(ADC1, ADC_VMOR_INT, FALSE);

  adc_enable(ADC1, TRUE);
  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1));
}

/**
  * @brief  configure the tim3_channel_1.
  * @param  none
  * @retval none
  */
void gpio_configuration(void)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
  /* gpioa gpiob clock enable */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_init_struct.gpio_pins = GPIO_PINS_6;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOA, &gpio_init_struct);

  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE6, GPIO_MUX_1);
}

void tim3_configuration(void)
{
  tmr_output_config_type tmr_oc_init_structure;

  /* tmr3 clock enable */
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  /* compute the prescaler value */
  prescaler_value = (uint16_t)(system_core_clock / 24000000) - 1;

  /* tmr3 time base configuration */
  tmr_base_init(TMR3, set_Reload_value, 0);
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);

  tmr_output_default_para_init(&tmr_oc_init_structure);
  tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_oc_init_structure.oc_idle_state = FALSE;
  tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_oc_init_structure.oc_output_state = TRUE;
  tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_1, &tmr_oc_init_structure);
  tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, 1);
  tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_1, TRUE);


  tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_2, &tmr_oc_init_structure);
  tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_2,499);
  tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_2, TRUE);

  tmr_period_buffer_enable(TMR3, TRUE);

  /* tmr enable counter */
  tmr_counter_enable(TMR3, TRUE);

  /* tmr3 int enable */
  tmr_interrupt_enable(TMR3, TMR_C2_INT, TRUE);
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 0, 0);
}


void system_init(void)
{
  //system_clock_config();
  wk_system_clock_config();
  /* gpio configuration */
  gpio_configuration();
  tim3_configuration();
  delay_init();
  uart_print_init(115200);
  adc_gpio_config();
  dma_config();
  adc_config();

}


/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  system_init();
  tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, 1);
  set_current=10;
  set_voltage=50;
  count_un_ctrl_parameter();
  printf("usart printf example: retarget the c library printf function to the usart\r\n");
  
  /***********************/
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
  /***********************/
  while(1)
  {
    delay_ms(1000);
    //printf("usart printf counter: %u\r\n",time_cnt++);
    printf("voltage value = %.3fV\r\n", ad_value[0]*voltage_cailbration);
    printf("now_current value = %dA\r\n", now_current);
    printf("mean_current value = %dA\r\n", ad_value[1]);
    printf("mean_current  = %.3fA\r\n", ad_value[1]*current_cailbration);
    printf("power   value = %.3fW\r\n", ad_value[0]*voltage_cailbration*ad_value[1]*current_cailbration);
    printf("reload  value  = %d\r\n", currten_duty/constant);
    really_duty = (currten_duty/constant);
    printf("duty____value = %.3f%%\r\n", really_duty/set_Reload_value*100);

  }

}

/**
  * @brief  this function handles adc1_2 handler.
  * @param  none
  * @retval none
  */
/*void ADC1_CMP_IRQHandler(void)
{
  if(adc_flag_get(ADC1, ADC_VMOR_FLAG) != RESET)
  {
    at32_led_toggle(LED3);
    adc_flag_clear(ADC1, ADC_VMOR_FLAG);
    vmor_flag_index = 1;
  }
}*/

/**
 * @brief 
 * 将电压扰动的限制移到了函数内部，确保不会超出范围。
  增加了对电压扰动的限制。
  去掉了不必要的全局变量，直接使用 set_duty_voltage 来进行电压扰动。
  简化了一些变量名，以提高可读性。
 * 
 */
void MPPT_PerturbObserve(void) {
    static double perturbAmount = 0.1;  // 扰动幅度，根据实际情况调整

    // 计算电压和电流的实际值
    double voltage = ad_value[0] * voltage_cailbration;
    double current = ad_value[1] * current_cailbration;

    // 计算功率
    double power = voltage * current;

    // 扰动电压
    double perturbedVoltage = voltage + perturbAmount;

    // 计算扰动后的功率
    double perturbedPower = perturbedVoltage * current;

    // 比较扰动前后的功率
    if (perturbedPower > power) {
        // 如果功率增加，则增大电压扰动
        set_duty_voltage += perturbAmount;  // 根据实际情况调整增加的电压扰动
    } else {
        // 如果功率减小，则减小电压扰动
        set_duty_voltage -= perturbAmount;  // 根据实际情况调整减小的电压扰动
    }

    // 限制电压扰动范围
    if (set_duty_voltage > set_duty_max) {
        set_duty_voltage = set_duty_max;
    } else if (set_duty_voltage < set_duty_min) {
        set_duty_voltage = set_duty_min;
    }

    // 更新电压扰动到 PWM 输出
    voltage_duty += set_duty_voltage;
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, voltage_duty / constant);
}


void TMR3_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR3_GLOBAL_IRQ 0 */
  if(tmr_flag_get(TMR3, TMR_C2_FLAG) != RESET)
  {
    time_cnt++;
    voltage_duty += set_duty_voltage-ad_value[0];
    currten_duty += set_duty_currten-ad_value[1];
    now_current   = ad_value[1];

    if (voltage_duty > set_duty_max)
    {
      voltage_duty = set_duty_max;
    }
    else if (voltage_duty < set_duty_min)
    {
      voltage_duty = set_duty_min;
    }
      
    if (currten_duty >voltage_duty)
    {
      currten_duty = voltage_duty;
    }
    else if (currten_duty < set_duty_min)
    {
      currten_duty = set_duty_min;
    }

    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, currten_duty/constant);

    tmr_flag_clear(TMR3, TMR_C2_FLAG);
  }

  /* add user code end TMR3_GLOBAL_IRQ 0 */
  /* add user code begin TMR3_GLOBAL_IRQ 1 */

  /* add user code end TMR3_GLOBAL_IRQ 1 */
}
