/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== LoRa AIR: jembatan Pixhawk TELEM <-> E32 =====
   Parameter E32 HARUS SAMA PERSIS dengan LoRa GCS (lihat tabel SPED di LoRa GCS). */
#define E32_ADDH    0x00
#define E32_ADDL    0x00
#define E32_SPED    (0x30 | 0x04)   /* UART 8N1 57600 bps, air rate 9.6 kbps */
#define E32_CHAN    0x17            /* 410 + 0x17 = 433 MHz */
#define E32_OPTION  0x44            /* transparan, push-pull, FEC on, daya 30 dBm (1 W) */
#define E32_UART_BAUD  57600        /* harus sesuai bit baud di E32_SPED */

#define AIR_CYCLE_MS  140     /* 1 kiriman + jeda dengar. Turunkan pelan2 jika sudah stabil */
#define RADIO_Q_PRIO  2000    /* batas antrean paket penting */

#define PIX_BAUD      57600   /* = SERIALx_BAUD 57 di Pixhawk */
#define E32_BURST     58      /* maks. byte per kiriman ke E32 (= 1 sub-paket radio) */
#define RING_SIZE     4096    /* ukuran antrean (harus pangkat 2) */
#define RADIO_Q_MAX   1024    /* antrean radio maks. (~1 s di 9.6k); lebih dari ini paket baru dibuang
                                 supaya telemetri tidak makin terlambat */
#define MAV_MAX       280     /* ukuran maks. paket MAVLink2 (termasuk signature) */

/* Nilai e32_status (lihat di Live Expressions) */
#define E32_ST_BELUM        0
#define E32_ST_OK           1
#define E32_ST_AUX_TIMEOUT  2   /* AUX tidak pernah HIGH: cek kabel AUX / power */
#define E32_ST_NO_REPLY     3   /* tidak ada balasan: cek TX/RX tertukar, M0/M1 */
#define E32_ST_WRITE_FAIL   4   /* parameter gagal ditulis */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1;		//Pixhawk TELEM: PA9 (TX) -> RX Pixhawk, PA10 (RX) <- TX Pixhawk

uint8_t e32_cfg[6];				//hasil baca parameter E32 (harus C0 00 00 34 17 44)
uint8_t e32_ver[4];				//versi modul (C3 32 xx yy, 0x32 = 433 MHz)
volatile uint8_t e32_status = E32_ST_BELUM;

/* Antrean satu-produsen satu-konsumen (aman dipakai interrupt + main loop) */
typedef struct {
  uint8_t buf[RING_SIZE];
  volatile uint32_t head;   /* ditulis produsen */
  volatile uint32_t tail;   /* ditulis konsumen */
} Ring;

static Ring pix_rx;     //byte mentah dari Pixhawk (diisi interrupt USART1)
static Ring radio_tx;   //paket MAVLink UTUH yang menunggu dikirim ke E32
static Ring radio_rx;   //byte dari E32 / Mission Planner (diisi interrupt USART2) -> ke Pixhawk

uint8_t e32_tx[E32_BURST];		//buffer kirim ke E32
uint8_t pix_tx[64];				//buffer kirim ke Pixhawk

/* Pemotong paket MAVLink (supaya yang dibuang selalu paket utuh, bukan potongan) */
static uint8_t mav_pkt[MAV_MAX];
static uint16_t mav_len = 0, mav_need = 0;
static uint32_t mav_tick = 0;

/* Statistik - dilaporkan tiap 2 detik lewat USB-C (Live Expressions juga bisa) */
volatile uint32_t pix_bytes = 0;	//byte dari Pixhawk
volatile uint32_t mav_ok = 0;		//paket MAVLink diantrekan ke radio
volatile uint32_t mav_drop = 0;		//paket dibuang karena radio penuh -> turunkan SRx_ di Pixhawk
volatile uint32_t mav_junk = 0;		//byte/paket rusak dari Pixhawk (cek baud & kabel)
volatile uint32_t up_bytes = 0;		//byte pesawat -> radio
volatile uint32_t down_bytes = 0;	//byte radio -> Pixhawk (perintah Mission Planner)
volatile uint32_t pix_noise = 0;	//byte rusak di USART1 (cek kabel Pixhawk)
volatile uint32_t ring_drop = 0;	//byte terbuang karena antrean mentah penuh

char out[200];					//buffer teks laporan

extern USBD_HandleTypeDef hUsbDeviceFS;	//didefinisikan di usb_device.c
extern volatile uint32_t rx_noise;		//byte rusak dibuang (stm32f4xx_it.c) - harus tetap 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Tunggu AUX HIGH (modul siap). Return 1 jika siap, 0 jika timeout. */
static uint8_t E32_WaitAux(uint32_t timeout)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_RESET)
  {
    if (HAL_GetTick() - t0 > timeout) return 0;
  }
  return 1;
}

/* Buang byte sisa & flag overrun di USART2, supaya balasan E32 tidak bergeser */
static void E32_FlushRx(void)
{
  for (int i = 0; i < 64; i++)
  {
    if (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) && !__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) break;
    (void)huart2.Instance->DR;   /* baca SR lalu DR = clear RXNE & ORE */
  }
}

/* Ganti mode: (0,0)=normal, (1,1)=sleep/konfigurasi */
static void E32_SetMode(uint8_t m0, uint8_t m1)
{
  E32_WaitAux(1000);   /* jangan ganti mode saat modul masih sibuk */
  HAL_GPIO_WritePin(E32_M0_GPIO_Port, E32_M0_Pin, m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E32_M1_GPIO_Port, E32_M1_Pin, m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_Delay(10);
  E32_WaitAux(1000);
  HAL_Delay(5);        /* datasheet: tunggu >= 2 ms setelah AUX HIGH */
  E32_FlushRx();
}

/* Kirim perintah konfigurasi, terima 6 byte balasan (diawali 0xC0) */
static uint8_t E32_Command(const uint8_t *cmd, uint16_t len, uint8_t *resp)
{
  /* E32 menolak perintah (balas FF FF FF) jika dikirim terlalu cepat setelah perintah sebelumnya */
  E32_WaitAux(1000);
  HAL_Delay(100);
  E32_FlushRx();
  HAL_UART_Transmit(&huart2, (uint8_t *)cmd, len, 100);
  if (HAL_UART_Receive(&huart2, resp, 6, 1000) != HAL_OK) return 0;
  return resp[0] == 0xC0;
}

/* Baca parameter E32; jika beda dari yang diinginkan, tulis (disimpan permanen) */
static void E32_Init(void)
{
  const uint8_t want[6] = {0xC0, E32_ADDH, E32_ADDL, E32_SPED, E32_CHAN, E32_OPTION};
  const uint8_t rd[3] = {0xC1, 0xC1, 0xC1};
  uint8_t ok = 0;

  if (!E32_WaitAux(2000))
  {
    e32_status = E32_ST_AUX_TIMEOUT;
    return;
  }

  E32_SetMode(1, 1);
  for (int i = 0; i < 3 && !ok; i++) ok = E32_Command(rd, 3, e32_cfg);

  if (!ok)
  {
    e32_status = E32_ST_NO_REPLY;
  }
  else if (memcmp(e32_cfg, want, 6) != 0)
  {
    E32_Command(want, 6, e32_cfg);
    E32_WaitAux(1000);
    HAL_Delay(50);
    ok = E32_Command(rd, 3, e32_cfg);
    e32_status = (ok && memcmp(e32_cfg, want, 6) == 0) ? E32_ST_OK : E32_ST_WRITE_FAIL;
  }
  else
  {
    e32_status = E32_ST_OK;
  }

  /* Baca versi modul */
  const uint8_t ver[3] = {0xC3, 0xC3, 0xC3};
  E32_WaitAux(1000);
  HAL_Delay(100);
  E32_FlushRx();
  HAL_UART_Transmit(&huart2, (uint8_t *)ver, 3, 100);
  for (int i = 0; i < 4; i++)
  {
    if (HAL_UART_Receive(&huart2, &e32_ver[i], 1, 300) != HAL_OK) break;
  }

  E32_SetMode(0, 0);
}

/* ---------- Antrean ---------- */
static inline uint32_t Ring_Count(const Ring *r) { return (r->head - r->tail) & (RING_SIZE - 1); }
static inline uint32_t Ring_Free(const Ring *r)  { return RING_SIZE - 1 - Ring_Count(r); }

static inline uint8_t Ring_Put(Ring *r, uint8_t b)
{
  uint32_t next = (r->head + 1) & (RING_SIZE - 1);
  if (next == r->tail) return 0;   /* penuh */
  r->buf[r->head] = b;
  r->head = next;
  return 1;
}

static inline uint8_t Ring_Get(Ring *r, uint8_t *b)
{
  if (r->head == r->tail) return 0;
  *b = r->buf[r->tail];
  r->tail = (r->tail + 1) & (RING_SIZE - 1);
  return 1;
}

/* Salin maks. n byte ke dst TANPA mengambilnya (diambil dengan Ring_Drop) */
static uint32_t Ring_Peek(const Ring *r, uint8_t *dst, uint32_t n)
{
  uint32_t cnt = Ring_Count(r), t = r->tail;
  if (n > cnt) n = cnt;
  for (uint32_t i = 0; i < n; i++) dst[i] = r->buf[(t + i) & (RING_SIZE - 1)];
  return n;
}

static inline void Ring_Drop(Ring *r, uint32_t n) { r->tail = (r->tail + n) & (RING_SIZE - 1); }

/* USART1 ke Pixhawk TELEM: PA9 TX, PA10 RX, 57600 8N1 + interrupt terima
   (GPIO diatur di sini karena USART1 tidak diaktifkan di CubeMX) */
static void PIX_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = PIX_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* Pindah USART2 ke baud kerja + aktifkan interrupt terima (setelah konfigurasi E32 di 9600) */
static void E32_StartBridge(void)
{
  huart2.Init.BaudRate = E32_UART_BAUD;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
  E32_FlushRx();
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* Laporan teks HANYA lewat USB-C (USART1 dipakai Pixhawk, jangan dikotori teks) */
static void Log(const char *s, uint16_t n)
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;   /* USB tidak dicolok */

  uint32_t t0 = HAL_GetTick();
  while (CDC_Transmit_FS((uint8_t *)s, n) == USBD_BUSY)
  {
    if (HAL_GetTick() - t0 > 20) return;   /* terminal belum dibuka */
  }
  t0 = HAL_GetTick();
  while (((USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData)->TxState != 0)
  {
    if (HAL_GetTick() - t0 > 20) break;   /* buffer 's' dipakai ulang, tunggu kirim selesai */
  }
}

/* Dipanggil dari USART2_IRQHandler (stm32f4xx_it.c) untuk setiap byte dari E32 */
void E32_RxByte(uint8_t b)
{
  if (!Ring_Put(&radio_rx, b)) ring_drop++;
}

static uint8_t Mav_IsPriority(uint32_t id)
{
  switch (id)
  {
    case 0:                                   /* HEARTBEAT */
    case 22:                                  /* PARAM_VALUE */
    case 39: case 40: case 41: case 44:       /* MISSION_ITEM/REQUEST/SET_CURRENT/COUNT */
    case 47: case 51: case 73:                /* MISSION_ACK/REQUEST_INT/ITEM_INT */
    case 77:                                  /* COMMAND_ACK */
    case 253:                                 /* STATUSTEXT */
      return 1;
    default:
      return 0;
  }
}

/* Susun byte dari Pixhawk menjadi paket MAVLink utuh, lalu antrekan ke radio.
   v1: FE len seq sys comp msgid [payload] crc(2)          -> 6 + len + 2
   v2: FD len incompat compat seq sys comp msgid(3) [payload] crc(2) [signature 13]
                                                            -> 10 + len + 2 (+13) */
static void Mav_Feed(uint8_t b)
{
  /* Cari magic byte MAVLink1 (0xFE) atau MAVLink2 (0xFD). */
  if (mav_len == 0)
  {
    if (b != 0xFE && b != 0xFD)
    {
      mav_junk++;
      return;
    }
    mav_pkt[0] = b;
    mav_len = 1;
    mav_need = 0;
    mav_tick = HAL_GetTick();
    return;
  }

  /* Pulih jika frame rusak atau ukurannya melewati buffer. */
  if (mav_len >= MAV_MAX)
  {
    mav_len = 0;
    mav_need = 0;
    mav_junk++;
    if (b == 0xFE || b == 0xFD)
    {
      mav_pkt[0] = b;
      mav_len = 1;
    }
    mav_tick = HAL_GetTick();
    return;
  }

  mav_pkt[mav_len++] = b;
  mav_tick = HAL_GetTick();

  /* Setelah byte length diterima, hitung ukuran frame total. */
  if (mav_len == 2)
  {
    mav_need = (mav_pkt[0] == 0xFE ? 8U : 12U) + mav_pkt[1];
  }
  /* MAVLink2 incompat_flags bit 0 menandakan ada signature 13 byte. */
  else if (mav_len == 3 && mav_pkt[0] == 0xFD && (mav_pkt[2] & 0x01U))
  {
    mav_need += 13U;
  }

  if (mav_need != 0 && mav_len >= mav_need)
  {
    uint32_t id = (mav_pkt[0] == 0xFE)
                ? mav_pkt[5]
                : ((uint32_t)mav_pkt[7] |
                   ((uint32_t)mav_pkt[8] << 8) |
                   ((uint32_t)mav_pkt[9] << 16));
    uint32_t limit = Mav_IsPriority(id) ? RADIO_Q_PRIO : RADIO_Q_MAX;

    if (Ring_Count(&radio_tx) + mav_len <= limit)
    {
      for (uint16_t i = 0; i < mav_len; i++)
      {
        if (!Ring_Put(&radio_tx, mav_pkt[i])) break;
      }
      mav_ok++;
    }
    else
    {
      mav_drop++;
    }

    mav_len = 0;
    mav_need = 0;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  E32_Init();

  //Konfigurasi gagal -> LED kedip cepat 2 detik sebagai tanda
  if (e32_status != E32_ST_OK)
  {
    for (int i = 0; i < 20; i++)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
  }

  E32_StartBridge();
  PIX_UART_Init();

  uint32_t last_burst = 0;    //waktu kiriman terakhir ke E32
  uint32_t last_act = 0;      //waktu ada lalu lintas data (untuk LED)
  uint32_t last_report = 0;   //waktu laporan statistik ke USB
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint32_t now = HAL_GetTick();
	uint8_t b;

	//1) Pixhawk -> potong jadi paket MAVLink utuh -> antrean radio
	while (Ring_Get(&pix_rx, &b))
	{
		pix_bytes++;
		Mav_Feed(b);
	}
	if (mav_len > 0 && (int32_t)(HAL_GetTick() - mav_tick) > 20)   /* perbaikan bug waktu */
	{
		mav_len = 0;
		mav_junk++;
	}

	if (Ring_Count(&radio_tx) > 0 && (int32_t)(now - last_burst) >= AIR_CYCLE_MS &&
	    HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET)
	{
		uint32_t n = Ring_Peek(&radio_tx, e32_tx, E32_BURST);
		HAL_UART_Transmit(&huart2, e32_tx, n, 50);
		Ring_Drop(&radio_tx, n);
		up_bytes += n;
		last_burst = last_act = HAL_GetTick();
	}

	//3) E32 (perintah dari Mission Planner) -> Pixhawk
	if (Ring_Count(&radio_rx) > 0)
	{
		uint32_t n = Ring_Peek(&radio_rx, pix_tx, sizeof(pix_tx));
		HAL_UART_Transmit(&huart1, pix_tx, n, 50);
		Ring_Drop(&radio_rx, n);
		down_bytes += n;
		last_act = HAL_GetTick();
	}

	//4) LED: nyala saat ada data lewat; kedip lambat jika konfigurasi E32 gagal
	if (e32_status != E32_ST_OK)
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (now / 250) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (now - last_act < 30) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	//5) Laporan tiap 2 detik lewat USB-C (hanya jika dicolok ke laptop saat uji di darat)
	if (now - last_report >= 2000)
	{
		last_report = now;
		int n = snprintf(out, sizeof(out),
				"[AIR] st=%u cfg=%02X %02X %02X %02X %02X %02X | pix=%lu mav_ok=%lu mav_drop=%lu junk=%lu | up=%lu down=%lu | noise=%lu/%lu drop=%lu\r\n",
				e32_status, e32_cfg[0], e32_cfg[1], e32_cfg[2], e32_cfg[3], e32_cfg[4], e32_cfg[5],
				pix_bytes, mav_ok, mav_drop, mav_junk, up_bytes, down_bytes,
				pix_noise, rx_noise, ring_drop);
		Log(out, n);
	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E32_M0_Pin|E32_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E32_M0_Pin E32_M1_Pin */
  GPIO_InitStruct.Pin = E32_M0_Pin|E32_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E32_AUX_Pin */
  GPIO_InitStruct.Pin = E32_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E32_AUX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Interrupt USART1 (Pixhawk): setiap byte langsung masuk antrean pix_rx.
   CATATAN: jangan aktifkan USART1 di CubeMX tanpa memindahkan handler ini. */
void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
  {
    uint8_t b = (uint8_t)USART1->DR;   /* baca SR lalu DR = clear semua flag */
    if (sr & (USART_SR_FE | USART_SR_NE)) pix_noise++;   /* byte rusak -> buang */
    else if (sr & USART_SR_RXNE)
    {
      if (!Ring_Put(&pix_rx, b)) ring_drop++;
    }
  }
}
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
#ifdef USE_FULL_ASSERT
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
