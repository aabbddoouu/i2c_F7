#include <uart_test.h>

#define CS_pin		GPIO14

void *_sbrk(int incr) { return (void *)-1; }

void lsm6ds3_self_test(void);

void open_spi(){
	gpio_clear(GPIOA, GPIO4);
}
void close_spi(){
	gpio_set(GPIOA, GPIO4);
}

void delay_setup(void);
void delay_ms(uint32_t us);

static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_3v3[RCC_CLOCK_3V3_216MHZ]);
	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOD);
	
}

static void gpio_setup(void)
{
	/* Set GPIOB0 LED to 'output' */
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,
		      GPIO_PUPD_NONE, GPIO0);

	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,GPIO0);
	
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOD,GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO8);
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8); //set PD8 to UART3_tx
	
}

void init_usart(void) {

	//////////////////////////////////////////////////////////////
	// STM32F7:
	//	RX:	-
	//	TX:	D8
	//	Baud:	115200
	//////////////////////////////////////////////////////////////

	rcc_periph_clock_enable(RCC_USART3);

	usart_set_baudrate(USART3,9600);
	usart_set_databits(USART3,8);
	usart_set_stopbits(USART3,USART_STOPBITS_1);
	usart_set_mode(USART3,USART_MODE_TX);
	usart_set_parity(USART3,USART_PARITY_NONE);
	usart_set_flow_control(USART3,USART_FLOWCONTROL_NONE);
	usart_enable(USART3);
}

void init_spi1(){
	rcc_periph_clock_disable(SPI1);
	rcc_periph_clock_enable(RCC_GPIOA);
	delay_setup();

	

	gpio_mode_setup(
		GPIOA,
        GPIO_MODE_OUTPUT,
        GPIO_PUPD_NONE,
        GPIO4		// NSS=PA4 (slave select) SCK=PA5,MOSI=PA7
	);
	
	gpio_set_output_options(
		GPIOA,
		GPIO_OTYPE_PP,
		GPIO_OSPEED_50MHZ,
		GPIO4
	);
	close_spi();
  delay_ms(100);

	rcc_periph_clock_enable(RCC_SPI1);


	gpio_mode_setup(
		GPIOA,
      GPIO_MODE_AF,
      GPIO_PUPD_NONE,
      GPIO5|GPIO6|GPIO7		// NSS=PA4 (slave select) SCK=PA5,MOSI=PA7
	);
	gpio_set_output_options(
		GPIOA,
		GPIO_OTYPE_PP,
		GPIO_OSPEED_50MHZ,
		GPIO5|GPIO7
	);
	gpio_set_af(GPIOA,GPIO_AF5,GPIO5|GPIO6|GPIO7); //set to AF5 => spi1

/*
	gpio_mode_setup(
		GPIOD,
            GPIO_MODE_OUTPUT,
        	GPIO_PUPD_NONE,
            GPIO14		// CS=PD14
	);

	gpio_set_output_options(
		GPIOD,
		GPIO_OTYPE_PP,
		GPIO_OSPEED_50MHZ,
		GPIO14
	);
*/
	/*
	gpio_mode_setup(
		GPIOA,
		GPIO_MODE_INPUT,
		GPIO_PUPD_NONE,
		GPIO6				// MISO=PA6
	);
	*/

	spi_reset(SPI1); 
	spi_init_master(
		SPI1,
        SPI_CR1_BAUDRATE_FPCLK_DIV_256,
        SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
		    SPI_CR1_CPHA_CLK_TRANSITION_2,
        SPI_CR1_MSBFIRST
	);
	
	spi_set_crcl_8bit(SPI1);
	spi_fifo_reception_threshold_8bit(SPI1);
	spi_enable_software_slave_management(SPI1);
	//spi_disable_ss_output(SPI1); not needed : included in the previous line
	spi_set_nss_high(SPI1);
	//spi_enable_ss_output(SPI1);
	
	
	//gpio_set(GPIOA,GPIO4);		//put slave on IDLE
	spi_enable(SPI1);

}



static void
pause(void) {
	int x;
	for ( x = 0; x < 1000000; x++ )	// Wait for 1 M clock cycles
		__asm__("NOP");
}

int main(void)
{
	
	int i;
	int j;
	clock_setup();
	gpio_setup();
	init_usart();
	init_spi1();

	uart_printf("My son will type AMEN\n");
	uart_printf("IGNORE for bad luck\n");
	delay_ms(100);
	//SPI_DR8(SPI1);

	lsm6ds3_self_test();
/*
	{
		char temp[256];
		int c2;

		c2 = mini_snprintf(temp,sizeof temp,"[c = %d]",c);
		uart_printf("Formatted '%s', with c2=%d.\n",temp,c2);
	}
*/
	uint8_t reg = 0x00;
	uint8_t reg_c = 0x00;
	char str_buf[100];
	///spi_set_nss_low(SPI1);
  
	for(;;){
		//reg |= 0x80; //for reading OPs
		
    gpio_toggle(GPIOB, GPIO0); //blink green led
		//xd
    /*
		open_spi();
		spi_send8(SPI1, reg);
		spi_read8(SPI1);
		spi_send8(SPI1, 0xFF);
		reg_c=spi_read8(SPI1);

		close_spi();
		sprintf((char *)str_buf, "sending : %u\n",reg++);
		uart_printf(str_buf);

		sprintf((char *)str_buf, "received : %u\n",reg_c);
		uart_printf(str_buf);
		
		*/
		//lsm6ds3_self_test();
		delay_ms(1000);
	}
	return 0;
}


void delay_setup(void)
{
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM6);
	/* microsecond counter */
	timer_set_prescaler(TIM6, rcc_3v3[RCC_CLOCK_3V3_216MHZ].apb1_frequency / 1000 ); //1.000.000
	timer_set_period(TIM6, 0xffff);
	timer_one_shot_mode(TIM6);
}


void delay_ms(uint32_t ms)
{
	TIM_ARR(TIM6) = 2*ms;
	TIM_EGR(TIM6) = TIM_EGR_UG;
	TIM_CR1(TIM6) |= TIM_CR1_CEN;
	//timer_enable_counter(TIM6);
	while (TIM_CR1(TIM6) & TIM_CR1_CEN);
}





#define    BOOT_TIME   20 //ms
#define    WAIT_TIME_A     200 //ms
#define    WAIT_TIME_G_01  800 //ms
#define    WAIT_TIME_G_02   60 //ms

/* Self test limits. */
#define    MIN_ST_LIMIT_mg         90.0f
#define    MAX_ST_LIMIT_mg       1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(const char* tx_buffer);
static void platform_delay(uint32_t ms);

/* Main Example --------------------------------------------------------------*/
void lsm6ds3_self_test()
{
  char tx_buffer[100];
  int16_t data_raw[3];
  stmdev_ctx_t dev_ctx;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t whoamI=0x00;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;

  uint32_t SPI_handle=SPI1; //cant use '&' on macro defines (they aren't variables) => waste of 4bytes space but fuck it
  dev_ctx.handle = &SPI_handle;
  
  uart_printf("Section 1\n");

  /* Init test platform */

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);
  sprintf((char *)tx_buffer, "ID : %x\n",whoamI);
  uart_printf(tx_buffer);
  uart_printf("Section 2\n");
  //return;
  if (whoamI != LSM6DS3_ID)
    while (1);

  

  /* Restore default configuration */
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
  uart_printf("Section 3\n");
  do {
    lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /**
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * BIG ASS FUCKING COMMENT TO TELL YOU THAT :
   * 
   * ALL USER REGISTER WRITES SHALL 
   * BE DONE AFTER THIS COMMENT 
   * 
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   * /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   */

  /* Enable auto increment for fast reads */
  lsm6ds3_auto_increment_set(&dev_ctx, PROPERTY_ENABLE); //by default it's enabled but : dakir ina dikra ..

  /* Enable Block Data Update */
  lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_52Hz);
  /* Set full scale */
  lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
  /* Wait stable output */
  platform_delay(WAIT_TIME_A);
  uart_printf("Section 4\n");
  /* Check if new value available */
  do {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  uart_printf("Section 5\n");

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6ds3_from_fs4g_to_mg(data_raw[j]);
    }
  }

  

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }
  uart_printf("Section 6\n");
  /* Enable Self Test positive (or negative) */
  lsm6ds3_xl_self_test_set(&dev_ctx, LSM6DS3_XL_ST_NEGATIVE);
  //lsm6ds3_xl_self_test_set(&dev_ctx, LSM6DS3_XL_ST_POSITIVE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_A);

  /* Check if new value available */
  do {
    lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  uart_printf("Section 7\n");

  /* Read dummy data and discard it */
  lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6ds3_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6ds3_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6ds3_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }
 
  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }
  uart_printf("Section 8\n");
  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

	if (st_result == ST_PASS) {
    sprintf((char *)tx_buffer, "Self Test - PASS\n" );
  }

  else {
    sprintf((char *)tx_buffer, "Self Test - FAIL\n" );
  }
	
  tx_com(tx_buffer);

  /* Disable Self Test */
  lsm6ds3_xl_self_test_set(&dev_ctx, LSM6DS3_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);
  /*
   * Gyroscope Self Test
   */
  /* Set Output Data Rate */
  lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_208Hz);
  /* Set full scale */
  lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);
  /* Wait stable output */
  platform_delay(WAIT_TIME_G_01);

  /* Check if new value available */
  do {
    lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6ds3_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6ds3_gy_self_test_set(&dev_ctx, LSM6DS3_GY_ST_POSITIVE);
  //lsm6ds3_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(WAIT_TIME_G_02);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6ds3_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mdps > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6ds3_gy_self_test_set(&dev_ctx, LSM6DS3_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_OFF);

  if (st_result == ST_PASS) {
    sprintf((char *)tx_buffer, "Self Test - PASS\n" );
  }

  else {
    sprintf((char *)tx_buffer, "Self Test - FAIL\n" );
  }

  tx_com(tx_buffer);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{
  //uart_printf("Init write\n");
  open_spi();	//enable spi comm w/ slave
  spi_send8(SPI1, reg); //reg where we gonna start writing 
  spi_read8(SPI1);
  // add delay ? send dummy data ?
  uint16_t i=0;   //send the whole buffer : bufp[i] is written in #{reg+i}
  for (;i<len;i++){   //stupid way to do it w/out timeout or security => may crash the program
    spi_send8(SPI1,bufp[i]); //no bound check for speed maybe revised ?
	spi_read8(SPI1);
  }
  close_spi();
  //uart_printf("write done\n");
  return 0; //non 0 return means an error
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  //char str_buf[20];
  reg |= 0x80; //wtf is this ?  0x80 = 1000 0000b
              //to tell the sensor this is a READ operation : 8th bit should be 1
  //uart_printf("Init read\n");
  open_spi();
  //uart_printf("read sec 1\n");
  spi_send8(SPI1, reg); //reg where we gonna start reading 
  //delay_ms(1);
  spi_read8(SPI1);

  //uart_printf("read sec 2\n");
  // add delay ? send dummy data ? 
  uint16_t i=0;   //start reading from #reg => #{reg+i} is written in bufp[i]
  for (;i<len;i++){   //stupid way to do it w/out timeout or security => may crash the program
    spi_send8(SPI1, 0xFF);
    //delay_ms(1);
	  bufp[i]=spi_read8(SPI1); //no bound check for speed maybe revised ?
  }
  close_spi();
  
  //sprintf((char *)str_buf, "read : %x\n", bufp[0]);
  //uart_printf(str_buf);

  //uart_printf("read done\n");
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(const char *tx_buffer)
{
  uart_printf((const char*)tx_buffer);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  delay_ms(ms);
}

