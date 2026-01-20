/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "oled.h"
#include "BSP_KEY.h"

///////////////////////////////函数声明//////////////////////////////
void Task();
void Shift_gears(uint8_t Shift);
void Set_num(s8* goal,s8 index,u8 op);
void average();
double Get_R(double dat_AD, uint8_t shift);
void OLED_Show_Scale(uint8_t shift);
void OLED_Show_Mode(uint8_t mode);
double Get_And_Show_Rd(double R,uint16_t adc_value,uint8_t shift);
double average2();
double average3();
////////////////////////////////////////////////////////////////////

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//??
#define AD_JZY 3.295	
#define R_WC	 0.1	
#define ADC_ITER_NUM 60

u8 oled_buf[16];
char temp[64];  
unsigned char cnt = 0;
double dat_AD = 0;  
double R = 0;  
double R_goal=0;
double R_error=0;
uint8_t shift = 1;
double dat_AD1 = 0;
uint32_t R1 = 0;  
uint8_t mode = 2;
uint32_t R_set = 0;
uint16_t R1_temp[128] = {63};
uint16_t adc_value=0;
double Rd=0;
double angle;
int angle_count=0;
u8 angle_direction=1;
u8 update_flag=1;//屏幕需要1刷新(GOAL)
u8 update_flag1=1;//屏幕需要1刷新(ERROR)
u8 mode_change_flag=0;//模式切换标志位

s8 goal[8];
s8 error[8];
s8 index_goal=7;
s8 index_error=7;
#define MODE_SET_GOAL 0
#define MODE_SET_ERROR 1
#define MODE_RUNNING 2
#define MODE_MANUAL 3
#define MODE_AUTO 4
#define CLEAR(x) Oled_Display_String((x),0,"                ")
#define ALL_MODE 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  u8 timer_count=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器输出
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)//ADC校准
  {
    Error_Handler();
  }
  Oled_Init();//OLED初始化
  OLED_Clear(0x00);//清屏
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //方法1:
    // for(timer_count=0;timer_count<50;timer_count++)
    // {
    //     HAL_Delay(1);
    //     average();
    // }
	for(timer_count=0;timer_count<50;timer_count++)
{
HAL_Delay(1);
adc_value=average3();
}
    //方法2:
    //adc_value=(uint16_t)average2();

    Task();//main task
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCL_Pin|SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin BUTTON3_Pin BUTTON4_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin|BUTTON2_Pin|BUTTON3_Pin|BUTTON4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
///////////////////////////////////////TASK////////////////////////////////////////
void Task()
{
  OLED_Show_Mode(mode);//显示当前模式

//////////////////////////////////////////////
  if(KEY_Scan()==KEYD_PRES){//模式切换
      mode++;
      mode%=ALL_MODE;
      mode_change_flag=1;
  }
//////////////////////////////////////////////

//////////////////////////////////SET GOAL///////////////////////////////////
  if(mode==MODE_SET_GOAL){
    if(mode_change_flag){
      mode_change_flag=0;update_flag=1;
      CLEAR(6);
    }
    if(KEY_Scan()==KEYU_PRES){Set_num(goal,index_goal,1);update_flag=1;}
    if(KEY_Scan()==KEYL_PRES){index_goal--;update_flag=1;}
    if(KEY_Scan()==KEYR_PRES){index_goal++;update_flag=1;}
    if(index_goal<0)index_goal=7;
    if(index_goal>7)index_goal=0;
    //show the goal and error
    if(update_flag){
      update_flag=0;
      CLEAR(2);Oled_Display_String(2,0,"GOAL:    ");
      CLEAR(4);Oled_Display_String(4,0,"ERROR:   ");
      Oled_Display_Num_with_Index(goal,index_goal,2,8*8);
      Oled_Display_Num_with_Index(error,index_error,4,8*8);
    }
  }//if(mode==MODE_SET_GOAL)

//////////////////////////////////SET ERROR///////////////////////////////////
  if(mode==MODE_SET_ERROR){
    if(mode_change_flag){
      mode_change_flag=0;update_flag1=1;
      CLEAR(6);
    }
    if(KEY_Scan()==KEYU_PRES){Set_num(error,index_error,1);update_flag1=1;}
    if(KEY_Scan()==KEYL_PRES){index_error--;update_flag1=1;}
    if(KEY_Scan()==KEYR_PRES){index_error++;update_flag1=1;}
    if(index_error<0)index_error=7;
    if(index_error>7)index_error=0;
    //Show the goal and error
    if(update_flag1){
      update_flag1=0;
      CLEAR(2);Oled_Display_String(2,0,"GOAL:    ");
      CLEAR(4);Oled_Display_String(4,0,"ERROR:   ");
      Oled_Display_Num_with_Index(goal,index_goal,2,8*8);
      Oled_Display_Num_with_Index(error,index_error,4,8*8);
    }
    
  }//if(mode==MODE_SET_ERROR)

  //Calculate the goal and error
  R_goal=goal[0]*10000000+goal[1]*1000000+goal[2]*100000+goal[3]*10000+goal[4]*1000+goal[5]*100+goal[6]*10+goal[7];
  R_error=error[0]*10000000+error[1]*1000000+error[2]*100000+error[3]*10000+error[4]*1000+error[5]*100+error[6]*10+error[7];

//////////////////////////////////RUNNING///////////////////////////////////
  if(mode==MODE_RUNNING){
    
    if(mode_change_flag){
      mode_change_flag=0;
      CLEAR(6);
    }

    dat_AD=(double)adc_value;
    R=Get_R(dat_AD,shift);
    Rd=Get_And_Show_Rd(R,adc_value,shift);
    
    //判断阻值是否符合标准
    if(R_goal&&R_error){
    if(R<=(R_goal-R_error)){//实际阻值过小
          Oled_Display_String(6,0,"too little!     ");
          }else if(R>=(R_goal+R_error)){//实际阻值过大
          Oled_Display_String(6,0,"too big!        ");
        }else{//实际阻值符合标准
          Oled_Display_String(6,0,"      PASS      ");
        }
    }else{
      Oled_Display_String(6,0,"----------------");
    }
    


    if(R<500)shift=1;//100  
    else if(R<9000)shift=2;//1k  1000 vs 10000
    else if(R<110000)shift=3;//10k 10 000   vs 100 000
    else shift=4;//10M

    if(R<1){
      //shift=4;
    }
    Shift_gears(shift);//切换下次档位
  }//if(mode==MODE_RUNNING)

//////////////////////////////////MANUAL///////////////////////////////////
  if(mode==MODE_MANUAL){
    OLED_Show_Scale(shift);//显示量程 row=6
    if(adc_value==0){
      //R = 0;//无效阻值
    }
    else{
      dat_AD=(double)adc_value;
      R=Get_R(dat_AD,shift);
      Rd=Get_And_Show_Rd(R,adc_value,shift);//计算并显示实际合适量程的阻值
    }

    if(KEY_Scan()==KEYU_PRES){//手动切换档位
      shift++;
      if(shift>4){
        shift=1;
      }
      Shift_gears(shift);
    }
  }//if(mode==MODE_MANUAL)

//////////////////////////////////AUTO///////////////////////////////////
   if(mode==MODE_AUTO){
		 
		 if(mode_change_flag){
      mode_change_flag=0;
      CLEAR(6);
    }
		 
    shift=1;//根据实际电阻值 量程设置为1
    Shift_gears(shift);//执行切换档位
    angle_count++;//测量分频计数器计数
    if(angle_count>=4){//修改分频值here
      angle_count=0;

      if(angle_direction){//++
        angle+=9;
        if(angle>=180-9) {
          angle_direction=0;
					while(1);
        }
      }else{//--
        angle-=9;
        if(angle<=0+9) {
          angle_direction=1;
        }
      }
      
      //设置新的舵机值
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)(20000.0/40.0+(angle/180.0)*(20000.0/40.0*4.0)));
    }
    if(adc_value==0){
      R = 0;//无效阻值
    }
    else{
      dat_AD=(double)adc_value;//获取ADC原始值
      R=Get_R(dat_AD,shift);//计算实际阻值
      Rd=Get_And_Show_Rd(R,adc_value,shift);//计算并显示实际合适量程的阻值
    }
    //串口发送阻值数据
    //格式:角度值,实际阻值 
    sprintf(temp, "%d,%lf\r\n",(uint16_t)angle,R);
    HAL_UART_Transmit(&huart1, (uint8_t*)temp, strlen(temp), 0xffff);
  }//if(mode==MODE_AUTO)

}//TASK

////////////////////////////////////////底层函数定义///////////////////////////////////


double average3()
{
static uint8_t i=0;
static double sum=0;
static double data[10];
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1,10);
data[i] = (double)HAL_ADC_GetValue(&hadc1);
sum+=data[i];
i=(i+1)%10;
if(i==0)
{
double avg=sum/10.0;
sum=0;
return avg;
}
return 0;
}


//根据实际电阻范围选择合适的显示方式显示出来 并计算实际应显示的值
//在第三行显示原始ADC值  
//并显示当前档位
double Get_And_Show_Rd(double R,uint16_t adc_value,uint8_t shift)
{
  double Rd;
  if(R<1000){
  Rd=R;
  sprintf(oled_buf, "R:%10.3lfR", Rd);
  }else if(R<1000000){
  Rd=R/1000.0;
  sprintf(oled_buf, "R:%10.3lfK", Rd);
  }else if(R<1000000000){
  Rd=R/1000000.0;
  sprintf(oled_buf, "R:%10.3lfM", Rd);
  }else {
    //Rd=0.0;
		//Rd=0.0;
    sprintf(oled_buf, "R:              ");
  }
  //Show on the OLED
  CLEAR(2);
  Oled_Display_String(2,0,oled_buf);
  CLEAR(4);
  sprintf(oled_buf, "ADC:%4d Shift:%d", adc_value,shift);
  Oled_Display_String(4,0,oled_buf);
  return Rd;
}


//OLED在顶部显示当前模式
void OLED_Show_Mode(uint8_t mode)
{
  if(mode==MODE_SET_GOAL){
    Oled_Display_String(0,0,"  MODE SET GOAL ");
  }
  if(mode==MODE_SET_ERROR){
    Oled_Display_String(0,0," MODE SET ERROR ");
  }
  if(mode==MODE_RUNNING){
    Oled_Display_String(0,0,"  MODE RUNNING  ");
  }
  if(mode==MODE_MANUAL){
    Oled_Display_String(0,0,"  MODE MANUAL   ");
  }if(mode==MODE_AUTO){
    Oled_Display_String(0,0,"  MODE AUTO     ");
  }
}
  
  
//根据量程在OLED底部显示(row=6)
void OLED_Show_Scale(uint8_t shift)
{
  if(shift==1){
    Oled_Display_String(6,0,"SCALE--------100");
  }
  if(shift==2){
    Oled_Display_String(6,0,"SCALE---------1K");
  }
  if(shift==3){
    Oled_Display_String(6,0,"SCALE--------10K");
  }
  if(shift==4){
    Oled_Display_String(6,0,"SCALE--------10M");
  }
}

//设置目标值和误差值通用函数
//输入参数:
//goal:要修改的数字的各位数组
//index:要修改的位
//op:固定为1 单向增加
void Set_num(s8* goal,s8 index,u8 op){
  if(index<0)return;
  if(index>=8)return;

  if(op==0){
    goal[index]--;
    if(goal[index]<=0)goal[index]=9;
  }
  else {
    goal[index]++;
    if(goal[index]>=10)goal[index]=0;
  }

}

//切换量程档位
void Shift_gears(uint8_t Shift)
{
		if(Shift == 1)			
		{
			 HAL_GPIO_WritePin(GPIOB, SW1_Pin, 0);
				HAL_GPIO_WritePin(GPIOB, SW2_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW3_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW4_Pin, 1);
		}
		else if(Shift == 2)			
		{
				HAL_GPIO_WritePin(GPIOB, SW1_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW2_Pin, 0);
				HAL_GPIO_WritePin(GPIOB, SW3_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW4_Pin, 1);
		}
		else if(Shift == 3)			
		{
				HAL_GPIO_WritePin(GPIOB, SW1_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW2_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW3_Pin, 0);   
				HAL_GPIO_WritePin(GPIOB, SW4_Pin, 1);
		}
		else if(Shift == 4)			
		{
        HAL_GPIO_WritePin(GPIOB, SW1_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW2_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW3_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW4_Pin, 0);
		}
		else	
		{
        HAL_GPIO_WritePin(GPIOB, SW1_Pin, 1);
				HAL_GPIO_WritePin(GPIOB, SW2_Pin, 1);        
				HAL_GPIO_WritePin(GPIOB, SW3_Pin, 1);  
        HAL_GPIO_WritePin(GPIOB, SW4_Pin, 1);
		}
}

//根据ADC原始值和量程计算实际电阻值
double Get_R(double dat_AD, uint8_t shift)
{
  double R=0;
	if(dat_AD==0)return 99999999999;
  if(shift == 1)	R = (double)(4096.0-dat_AD)*99/dat_AD;
  else if(shift == 2)	R = (double)(4096.0-dat_AD)*985.0/dat_AD;
  else if(shift == 3)	R = (double)(4096.0-dat_AD)*10000.0/dat_AD;
  else if(shift == 4)	R = (double)(4096.0-dat_AD)*11020000.0/dat_AD;
  return R;
}

uint8_t num = 0;
int v_data[30];
void average()
{

  HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
		
		v_data[num] = (int)HAL_ADC_GetValue(&hadc1);
		if(++num == 30)
		{
				num = 0;
				for(uint8_t i = 0; i < 30; i ++)
				for(uint8_t j = i; j < 30; j ++)
				{
			    if(v_data[j] <  v_data[i])
					{
							double t  = v_data[i];
							v_data[i] = v_data[j];
							v_data[j] = t;
					}		
				}
				double sum = 0;
				for(uint8_t i = 10; i < 20; i ++)
				{
						sum+=v_data[i];
				}
				adc_value = (uint16_t)(sum/10.0);
		}
}

//计算方法2:
//直接取平均值
double average2()
{
  int sum=0;
  for(int i=0;i<ADC_ITER_NUM;i++){
    HAL_Delay(1);
    HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1,10);
	  sum+= (int)HAL_ADC_GetValue(&hadc1);
  }
  return (double)sum/ADC_ITER_NUM;
  //return (double)HAL_ADC_GetValue(&hadc1);
}

// void TASK(void)
// {
// 		if(Shift == 1)			R = (double)(AD_JZY-dat_AD)*100/dat_AD;
// 		else if(Shift == 2)	R = (double)(AD_JZY-dat_AD)*1000/dat_AD;
// 		else if(Shift == 3)	R = (double)(AD_JZY-dat_AD)*10000/dat_AD;
// 		else if(Shift == 4)	R = (double)(AD_JZY-dat_AD)*1000000/dat_AD;
// 		OLED_ShowString(1,1,(uint8_t *)WS_Format("SHIFT:%d ", Shift),  strlen(WS_Format("SHIFT:%d ", Shift)));		
// 		if(R > 1000000)																		OLED_ShowString(1,2,(uint8_t *)WS_Format("R:%d     ", 0), 		 strlen(WS_Format("R:%d     ", 0)));	
// 		else if(R<1000)																		OLED_ShowString(1,2,(uint8_t *)WS_Format("R:%d     ", R), 		 strlen(WS_Format("R:%d     ", R)));		
// 		else 																							OLED_ShowString(1,2,(uint8_t *)WS_Format("R:%.1fK    ", (double)R/1000), strlen(WS_Format("R:%.1fK    ", (double)R/1000)));	
// 		double R_WC_num = (double)R_set*R_WC;
// 		if(R > 1000000)																		OLED_ShowString(1,5,(uint8_t *)WS_Format("    "), strlen(WS_Format("    ")));
// 		else
// 		{
// 				if(R>(R_set-R_WC_num) && R<(R_set+R_WC_num))	OLED_ShowString(1,5,(uint8_t *)WS_Format("PASS"), strlen(WS_Format("PASS")));	
// 				else 																					OLED_ShowString(1,5,(uint8_t *)WS_Format("NG  "), strlen(WS_Format("NG  ")));					
// 		}		


		
// 		if(R<190) 							Shift = 1;
// 		if(R>200 	&& R<900) 		Shift = 2;
// 		if(R>1000 && R<500000) 	Shift = 3;
// 		if(R>400000) 						Shift = 4;
// 		Shift_gears(Shift);
// }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
