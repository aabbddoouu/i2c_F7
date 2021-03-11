#include "i2c_oled.h"

void *_sbrk(int incr) { return (void *)-1; }

char buffer[50];


static inline void
uart_putc(char ch) {
	usart_send_blocking(USART3,ch);  //USART3 is connected to ST-link serial com
}

static int
uart_printf(const char *format,...) {
	va_list args;
	int rc;

	va_start(args,format);
	rc = mini_vprintf_cooked(uart_putc,format,args);
	va_end(args);
	return rc;
}


uint32_t ticks=0;

static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_3v3[RCC_CLOCK_3V3_216MHZ]);
	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOD);
	
}

void init_usart(void) {

	//////////////////////////////////////////////////////////////
	// STM32F7:
	//	RX:	-
	//	TX:	D8
	//	Baud:	115200
	//////////////////////////////////////////////////////////////
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOD,GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO8);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8); //set PD8 to UART3_tx

	rcc_periph_clock_enable(RCC_USART3);

	usart_set_baudrate(USART3,9600);
	usart_set_databits(USART3,8);
	usart_set_stopbits(USART3,USART_STOPBITS_1);
	usart_set_mode(USART3,USART_MODE_TX);
	usart_set_parity(USART3,USART_PARITY_NONE);
	usart_set_flow_control(USART3,USART_FLOWCONTROL_NONE);
	usart_enable(USART3);
}

static void gpio_setup(void)
{
	/* Set GPIOB0 LED to 'output' */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,
		      GPIO_PUPD_NONE, GPIO0);

	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,GPIO0);


}

static void i2c1_setup(){
	RCC_DCKCFGR2|= (RCC_DCKCFGR2_UARTxSEL_HSI<<RCC_DCKCFGR2_I2C1SEL_SHIFT); //set i2c1 clock to HSI = 16MHz
	rcc_periph_clock_enable(RCC_I2C1);
	
	i2c_reset(I2C_OLED);
	gpio_mode_setup(
		GPIOB,
      	GPIO_MODE_AF,
      	GPIO_PUPD_NONE,
      	GPIO8|GPIO9		//  SCL|SDA
	);
	gpio_set_output_options(
		GPIOB,
		GPIO_OTYPE_OD,
		GPIO_OSPEED_50MHZ,
		GPIO8|GPIO9
	);
	gpio_set_af(GPIOB,GPIO_AF4,GPIO8|GPIO9); //set to AF4 => i2c1
	
	i2c_peripheral_disable(I2C_OLED);
	i2c_enable_analog_filter(I2C_OLED);
	i2c_set_digital_filter(I2C_OLED, 0);
	i2c_set_speed(I2C_OLED,i2c_speed_fm_400k,rcc_get_spi_clk_freq(I2C_OLED));
	
	i2c_peripheral_enable(I2C_OLED);
}
void delay_setup(void)
{
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM6);
	/* microsecond counter */
	timer_set_prescaler(TIM6, 54000); //TIMx are clocked 2*apbx ; increment each 1us
	timer_set_period(TIM6, 0xFFFF); //OF each ms
	timer_one_shot_mode(TIM6);

	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM7);
	/* microsecond counter */
	timer_set_prescaler(TIM7, 108); //TIMx are clocked 2*apbx ; increment each 1us
	timer_set_period(TIM7, 0x3E8); //OF each ms
	timer_continuous_mode(TIM7);
	nvic_enable_irq(NVIC_TIM7_IRQ);
	timer_enable_irq(TIM7,TIM_DIER_UIE);
	timer_enable_counter(TIM7);
	
}

void tim7_isr(){
	ticks++;
	timer_clear_flag(TIM7,TIM_DIER_UIE);
}

void delay_ms(uint32_t ms)
{
	TIM_ARR(TIM6) = 2*ms;
	TIM_EGR(TIM6) = TIM_EGR_UG;
	timer_enable_counter(TIM6);
	while (TIM_CR1(TIM6) & TIM_CR1_CEN);
}


/**
 * Run a write/read transaction to a given 7bit i2c address
 * If both write & read are provided, the read will use repeated start.
 * Both write and read are optional
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param w buffer of data to write
 * @param wn length of w
 * @param autoend 
 */
void i2c_xfer7(uint32_t i2c, uint8_t addr, uint8_t command, uint8_t *w, size_t wn, bool autoend)
{	
	i2c_set_7bit_address(i2c, addr);
	i2c_set_write_transfer_dir(i2c);
	i2c_set_bytes_to_transfer(i2c, wn);
	if (autoend)
		i2c_enable_autoend(i2c);
	else
		i2c_disable_autoend(i2c);

	i2c_send_start(i2c);

	while (wn--) {
		bool wait = true;
		while (wait) {
			if (i2c_transmit_int_status(i2c)) {
				wait = false;
			}
			while (i2c_nack(i2c)); /* FIXME Some error */
		}
		i2c_send_data(i2c, *w++);
	}
}


int main(){
    clock_setup();
	delay_setup();
    gpio_setup();
    init_usart();
	i2c1_setup();
	uart_printf("testing... \n");
	
	//ssd1306_TestAll();
	
    for(;;){
		
  		sprintf((char *)buffer, "Ticks : %lu\n",ticks);
		uart_printf(buffer);
        gpio_toggle(GPIOB, GPIO0); //blink green led
		delay_ms(1000);
    }
	return 0;
}