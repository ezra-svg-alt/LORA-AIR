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
/* Pelacak batas paket MAVLink (tanpa menyimpan isi paket) */
typedef struct {
  uint16_t len, need, pe;   /* pe = posisi checksum */
  uint16_t crc;
  uint8_t  magic, sys, lo;
  uint8_t  ok;              /* 1 = CRC benar (hanya dicek untuk HEARTBEAT & RADIO_STATUS) */
  uint32_t msg;
  uint32_t tick;
} MavTrack;
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

#define FW_VER        "v6"
#define BLOCK_MAVFTP  1       /* 1 = sembunyikan MAVFTP dari Mission Planner, supaya parameter diunduh
                                 dengan PARAM_REQUEST_LIST biasa. MAVFTP mengirim paket 263 byte beruntun
                                 tanpa pengatur kecepatan -> di LoRa lambat lalu "unexpected error". */
#define PIX_BAUD      57600   /* = SER_TEL1_BAUD di PX4 (dicoba pertama; autobaud mencoba yang lain) */
#define PIX_AUTOBAUD  1       /* 1 = cari sendiri baud Pixhawk dari HEARTBEAT yang valid */
#define PIX_SCAN_MS   1500    /* lama mencoba satu baud */
#define PIX_LOST_MS   5000    /* HEARTBEAT hilang selama ini -> cari baud lagi */
#define LINK_TURNS    1       /* 1 = giliran radio aktif; 0 = transparan biasa (untuk perbandingan).
                                 HARUS sama dengan LINK_TURNS di LoRa GCS */
#define E32_BURST     58      /* maks. byte per kiriman ke E32 (= 1 sub-paket radio) */
/* Antrean radio dengan prioritas (paket baru dibuang bila antreannya penuh):
   0 = URGENT : HEARTBEAT, COMMAND_ACK            -> selalu dikirim duluan
   1 = PENTING: PARAM_VALUE, misi, STATUSTEXT ... -> TIDAK boleh kalah oleh telemetri
   2 = BIASA  : telemetri (attitude, GPS, dll.)    -> hanya dikirim bila 0 & 1 kosong
   Antrean PENTING dibuat BESAR (64 KB): ArduPilot mengirim parameter ~30% baud TELEM
   (~47 param/detik di 57600) dan TIDAK melambat karena RADIO_STATUS, sedangkan radio 9.6k
   hanya ~15-20 param/detik. Antrean kecil -> ratusan parameter dibuang -> Mission Planner
   meminta satu per satu -> sangat lambat lalu "unexpected error". 64 KB cukup menampung
   seluruh parameter ArduPilot (~1300 x 37 byte = ~48 KB), jadi tidak ada yang dibuang. */
#define URG_Q_MAX     512
#define PRIO_Q_MAX    2048   /* buffer 65536 byte */
#define NORM_Q_MAX    128     /* kecil: telemetri lama tidak ada gunanya */
#define MAV_MAX       280     /* ukuran maks. paket MAVLink2 (termasuk signature) */

/* ===== Giliran radio (E32 half-duplex: saat memancar TIDAK bisa menerima) =====
   AIR memancar maks. AIR_TX_SLOT_MS, lalu mengirim RADIO_STATUS sebagai tanda "giliran GCS"
   dan diam mendengarkan AIR_LISTEN_MS. Tanpa ini perintah Mission Planner (PARAM_REQUEST_xxx)
   selalu bertabrakan dengan telemetri -> macet di "Getting params". */
#define AIR_TX_SLOT_MS     500
#define AIR_LISTEN_MS      150    /* cukup untuk menangkap awal kiriman GCS; bila GCS memang
                                     mengirim, waktu dengar diperpanjang (AIR_LISTEN_EXT_MS) */
#define AIR_LISTEN_EXT_MS  150    /* diperpanjang tiap ada data masuk dari GCS */
#define AIR_LISTEN_MAX_MS  800
#define UP_GAP_MS          1500   /* perintah MP bisa terpotong di 2 giliran GCS (>500 ms);
                                     RADIO_STATUS tidak boleh disisipkan di tengahnya */
#define AIR_PH_TX          0
#define AIR_PH_MARK        1      /* tanda giliran sedang dipancarkan */
#define AIR_PH_LISTEN      2

/* RADIO_STATUS ke Pixhawk (seperti radio SiK): PX4 memperlambat stream & unduhan parameter
   bila txbuf (persen antrean radio yang kosong) kecil -> antrean tidak penuh, paket tidak dibuang */
#define PIX_RS_MS          50     /* sering, supaya PX4 cepat berhenti saat antrean parameter penuh */
#define RS_SYSID           51     /* '3' - id radio SiK */
#define RS_COMPID          68     /* 'D' */

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
  uint8_t *buf;
  uint32_t mask;            /* ukuran buffer - 1 (ukuran harus pangkat 2) */
  volatile uint32_t head;   /* ditulis produsen */
  volatile uint32_t tail;   /* ditulis konsumen */
} Ring;

static uint8_t pix_rx_buf[4096], radio_rx_buf[4096];
static uint8_t q_urg_buf[1024], q_prio_buf[65536], q_norm_buf[1024];

static Ring pix_rx = {pix_rx_buf, sizeof(pix_rx_buf) - 1, 0, 0};	//byte mentah dari Pixhawk (diisi interrupt USART1)
//paket MAVLink UTUH yang menunggu dikirim ke E32 (0 urgent, 1 penting, 2 biasa)
static Ring radio_q[3] = {
  {q_urg_buf,  sizeof(q_urg_buf) - 1,  0, 0},
  {q_prio_buf, sizeof(q_prio_buf) - 1, 0, 0},
  {q_norm_buf, sizeof(q_norm_buf) - 1, 0, 0},
};
static const uint32_t radio_q_max[3] = {URG_Q_MAX, PRIO_Q_MAX, NORM_Q_MAX};
volatile uint32_t q_drop[3] = {0};	//paket dibuang per antrean (q_drop[1] harus tetap 0)
static uint8_t tx_q = 0;		//antrean yang sedang dikirim (ganti hanya di batas paket)
volatile uint32_t par_ok = 0;		//PARAM_VALUE dari Pixhawk yang masuk antrean radio
volatile uint8_t ap_type = 0;		//HEARTBEAT.autopilot: 3 = ArduPilot, 12 = PX4, 0 = belum tahu
volatile uint32_t ftp_strip = 0;	//AUTOPILOT_VERSION yang kemampuan MAVFTP-nya dihapus
static Ring radio_rx = {radio_rx_buf, sizeof(radio_rx_buf) - 1, 0, 0};	//byte dari E32 / Mission Planner (diisi interrupt USART2) -> ke Pixhawk

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

volatile uint32_t rs_pix = 0;		//RADIO_STATUS dikirim ke Pixhawk
volatile uint32_t marks = 0;		//tanda giliran dikirim ke GCS
volatile uint32_t listen_hit = 0;	//data GCS diterima saat giliran mendengar

static MavTrack up_trk;		//batas paket radio -> Pixhawk (RADIO_STATUS hanya disisipkan di antara paket)
static MavTrack tx_trk;		//batas paket di aliran ke E32 (tanda giliran hanya di antara paket)
static MavTrack pix_trk;	//cek HEARTBEAT dari Pixhawk (untuk autobaud & diagnosis)

static const uint32_t pix_bauds[] = {57600, 115200, 921600, 460800, 38400, 19200, 9600};
volatile uint32_t pix_baud = PIX_BAUD;	//baud USART1 yang sedang dipakai
volatile uint8_t pix_locked = 0;	//1 = HEARTBEAT valid dari Pixhawk diterima di baud ini
volatile uint32_t pix_hb = 0;		//HEARTBEAT valid dari Pixhawk (harus naik ~1/detik)
volatile uint32_t gcs_hb = 0;		//HEARTBEAT valid dari Mission Planner lewat radio (bukti uplink jalan)
uint32_t last_pix_hb = 0;

char out[256];					//buffer teks laporan

extern USBD_HandleTypeDef hUsbDeviceFS;	//didefinisikan di usb_device.c
extern volatile uint32_t rx_noise;		//byte rusak dibuang (stm32f4xx_it.c) - harus tetap 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Mav_Crc(uint16_t *crc, uint8_t d);
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
static inline uint32_t Ring_Count(const Ring *r) { return (r->head - r->tail) & r->mask; }

static inline uint8_t Ring_Put(Ring *r, uint8_t b)
{
  uint32_t next = (r->head + 1) & r->mask;
  if (next == r->tail) return 0;   /* penuh */
  r->buf[r->head] = b;
  r->head = next;
  return 1;
}

static inline uint8_t Ring_Get(Ring *r, uint8_t *b)
{
  if (r->head == r->tail) return 0;
  *b = r->buf[r->tail];
  r->tail = (r->tail + 1) & r->mask;
  return 1;
}

/* Salin maks. n byte ke dst TANPA mengambilnya (diambil dengan Ring_Drop) */
static uint32_t Ring_Peek(const Ring *r, uint8_t *dst, uint32_t n)
{
  uint32_t cnt = Ring_Count(r), t = r->tail;
  if (n > cnt) n = cnt;
  for (uint32_t i = 0; i < n; i++) dst[i] = r->buf[(t + i) & r->mask];
  return n;
}

static inline void Ring_Drop(Ring *r, uint32_t n) { r->tail = (r->tail + n) & r->mask; }

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

/* Ganti baud USART1 (autobaud). HAL_UART_Init mematikan interrupt terima -> nyalakan lagi */
static void PIX_SetBaud(uint32_t baud)
{
  pix_baud = baud;
  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
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

/* Golongan prioritas paket dari Pixhawk (lihat URG_Q_MAX dkk.) */
static uint8_t Mav_Class(uint32_t id)
{
  switch (id)
  {
    case 0:     /* HEARTBEAT */
    case 77:    /* COMMAND_ACK */
      return 0;
    case 22:    /* PARAM_VALUE */
    case 322:   /* PARAM_EXT_VALUE */
    case 324:   /* PARAM_EXT_ACK */
    case 39: case 40: case 44: case 47: case 51: case 73:   /* MISSION_ITEM/REQUEST/COUNT/ACK/REQUEST_INT/ITEM_INT */
    case 110:   /* FILE_TRANSFER_PROTOCOL */
    case 148:   /* AUTOPILOT_VERSION */
    case 253:   /* STATUSTEXT */
    case 300:   /* PROTOCOL_VERSION */
      return 1;
    default:
      return 2;
  }
}

/* Periksa paket utuh dari Pixhawk sebelum diantrekan:
   - HEARTBEAT        : catat jenis autopilot (ArduPilot / PX4)
   - AUTOPILOT_VERSION: hapus bit MAV_PROTOCOL_CAPABILITY_FTP (0x20) + hitung ulang CRC */
static void Mav_Inspect(uint8_t *p, uint32_t id)
{
  uint16_t hdr = (p[0] == 0xFE) ? 6 : 10;
  uint16_t len = p[1];
  if (id == 0 && len >= 6) ap_type = p[hdr + 5];
#if BLOCK_MAVFTP
  uint8_t signed_pkt = (p[0] == 0xFD) && (p[2] & 0x01);
  if (id == 148 && len >= 1 && !signed_pkt && (p[hdr] & 0x20))
  {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 1; i < hdr + len; i++) Mav_Crc(&crc, p[i]);
    Mav_Crc(&crc, 178);   /* CRC_EXTRA AUTOPILOT_VERSION */
    if (crc != (uint16_t)(p[hdr + len] | ((uint16_t)p[hdr + len + 1] << 8))) return;   /* paket rusak: biarkan */

    p[hdr] &= (uint8_t)~0x20;
    crc = 0xFFFF;
    for (uint16_t i = 1; i < hdr + len; i++) Mav_Crc(&crc, p[i]);
    Mav_Crc(&crc, 178);
    p[hdr + len] = crc & 0xFF;
    p[hdr + len + 1] = crc >> 8;
    ftp_strip++;
  }
#endif
}

/* Susun byte dari Pixhawk menjadi paket MAVLink utuh, lalu antrekan ke radio.
   v1: FE len seq sys comp msgid [payload] crc(2)          -> 6 + len + 2
   v2: FD len incompat compat seq sys comp msgid(3) [payload] crc(2) [signature 13]
                                                            -> 10 + len + 2 (+13) */
static void Mav_Feed(uint8_t b)
{
  if (mav_len == 0)
  {
    if (b != 0xFD && b != 0xFE) { mav_junk++; return; }   /* tunggu awal paket */
    mav_need = 0;
  }
  mav_pkt[mav_len++] = b;
  mav_tick = HAL_GetTick();

  if (mav_len == 2 && mav_pkt[0] == 0xFE)
    mav_need = 6 + mav_pkt[1] + 2;
  else if (mav_len == 3 && mav_pkt[0] == 0xFD)
    mav_need = 10 + mav_pkt[1] + 2 + ((mav_pkt[2] & 0x01) ? 13 : 0);

  if (mav_need && mav_len >= mav_need)
  {
    /* Masuk antrean sesuai prioritas; antrean penuh -> buang paket UTUH */
    uint32_t id = (mav_pkt[0] == 0xFE) ? mav_pkt[5]
                : (mav_pkt[7] | ((uint32_t)mav_pkt[8] << 8) | ((uint32_t)mav_pkt[9] << 16));
    Mav_Inspect(mav_pkt, id);
    uint8_t k = Mav_Class(id);
    if (Ring_Count(&radio_q[k]) + mav_len <= radio_q_max[k])
    {
      for (uint16_t i = 0; i < mav_len; i++) Ring_Put(&radio_q[k], mav_pkt[i]);
      mav_ok++;
      if (id == 22) par_ok++;
    }
    else
    {
      mav_drop++;
      q_drop[k]++;
    }
    mav_len = 0;
    mav_need = 0;
  }
  else if (mav_len >= MAV_MAX)
  {
    mav_len = 0;   /* tidak masuk akal -> mulai ulang */
    mav_junk++;
  }
}

/* CRC MAVLink (X.25) */
static void Mav_Crc(uint16_t *crc, uint8_t d)
{
  uint8_t t = d ^ (uint8_t)(*crc & 0xFF);
  t ^= (uint8_t)(t << 4);
  *crc = (*crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4);
}

/* CRC_EXTRA pesan yang dicek; 0 = tidak dicek */
static uint8_t Mav_Extra(uint32_t msg)
{
  if (msg == 0) return 50;      /* HEARTBEAT */
  if (msg == 109) return 185;   /* RADIO_STATUS */
  return 0;
}

/* Ikuti batas paket MAVLink v1/v2. Return 1 saat byte terakhir paket diterima
   (t->sys, t->msg, t->ok berisi milik paket itu). gap = timeout paket terputus (0 = tanpa timeout). */
static uint8_t MavTrack_Feed(MavTrack *t, uint8_t b, uint32_t now, uint32_t gap)
{
  if (t->len && gap && now - t->tick > gap) t->len = 0;
  t->tick = now;
  if (t->len == 0)
  {
    if (b != 0xFD && b != 0xFE) return 0;
    t->magic = b;
    t->need = 0;
    t->msg = 0;
    t->crc = 0xFFFF;
    t->ok = 0;
  }
  uint16_t i = t->len++;
  if (i == 1)
  {
    t->pe = (t->magic == 0xFE ? 6 : 10) + b;   /* header + payload */
    t->need = t->pe + 2;
  }
  if (t->magic == 0xFE)
  {
    if (i == 3) t->sys = b;
    else if (i == 5) t->msg = b;
  }
  else
  {
    if (i == 2 && (b & 0x01)) t->need += 13;   /* signature */
    else if (i == 5) t->sys = b;
    else if (i >= 7 && i <= 9) t->msg |= (uint32_t)b << (8 * (i - 7));
  }
  if (i >= 1 && i < t->pe) Mav_Crc(&t->crc, b);
  else if (i == t->pe) t->lo = b;
  else if (i == t->pe + 1)
  {
    uint8_t x = Mav_Extra(t->msg);
    if (x)
    {
      uint16_t c = t->crc;
      Mav_Crc(&c, x);
      t->ok = (c == (uint16_t)(t->lo | ((uint16_t)b << 8)));
    }
  }
  if (t->need && t->len >= t->need) { t->len = 0; return 1; }
  if (t->len >= MAV_MAX) t->len = 0;
  return 0;
}

static uint8_t MavTrack_Busy(const MavTrack *t, uint32_t now, uint32_t gap)
{
  return t->len && !(gap && now - t->tick > gap);
}

/* Susun paket MAVLink1 RADIO_STATUS (#109) - 17 byte */
static uint8_t RS_Build(uint8_t *p, uint8_t txbuf, uint16_t rxerr)
{
  static uint8_t seq = 0;
  uint8_t i = 0;
  p[i++] = 0xFE; p[i++] = 9; p[i++] = seq++; p[i++] = RS_SYSID; p[i++] = RS_COMPID; p[i++] = 109;
  p[i++] = rxerr & 0xFF; p[i++] = rxerr >> 8;   /* rxerrors */
  p[i++] = 0; p[i++] = 0;                       /* fixed */
  p[i++] = 0; p[i++] = 0;                       /* rssi, remrssi (tidak tersedia di E32) */
  p[i++] = txbuf;                               /* % antrean radio kosong */
  p[i++] = 0; p[i++] = 0;                       /* noise, remnoise */
  uint16_t crc = 0xFFFF;
  for (uint8_t k = 1; k < i; k++) Mav_Crc(&crc, p[k]);
  Mav_Crc(&crc, 185);                           /* CRC_EXTRA RADIO_STATUS */
  p[i++] = crc & 0xFF;
  p[i++] = crc >> 8;
  return i;
}

/* Persen antrean PENTING (parameter) yang masih kosong (0..100) -> txbuf RADIO_STATUS.
   Telemetri sengaja tidak dihitung: kalau dihitung, PX4 berhenti mengirim parameter
   hanya karena antrean telemetri penuh. */
static uint8_t Radio_FreePct(void)
{
  uint32_t c = Ring_Count(&radio_q[1]);
  uint8_t f = (c >= PRIO_Q_MAX) ? 0 : (uint8_t)(100 - c * 100 / PRIO_Q_MAX);

  /* ArduPilot: txbuf hanya memperlambat STREAM telemetri (unduhan parameter tidak ikut berhenti),
     jadi antrean telemetri ikut dihitung -> paket telemetri yang dibuang berkurang,
     "link quality" di Mission Planner naik. */
  if (ap_type != 12)
  {
    uint32_t c2 = Ring_Count(&radio_q[2]);
    uint8_t f2 = (c2 >= NORM_Q_MAX) ? 0 : (uint8_t)(100 - c2 * 100 / NORM_Q_MAX);
    if (f2 < f) f = f2;
  }
  return f;
}

static uint8_t Radio_Pending(void)
{
  return Ring_Count(&radio_q[0]) || Ring_Count(&radio_q[1]) || Ring_Count(&radio_q[2]);
}

/* Antrean prioritas tertinggi yang berisi */
static uint8_t Radio_Pick(void)
{
  for (uint8_t k = 0; k < 3; k++) if (Ring_Count(&radio_q[k])) return k;
  return 2;
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
  uint32_t last_rs = 0;       //waktu RADIO_STATUS terakhir ke Pixhawk
  uint8_t  air_phase = AIR_PH_TX;
  uint8_t  slot_active = 0;   //sedang dalam giliran memancar
  uint32_t slot_start = 0, listen_start = 0, listen_end = 0;
  uint32_t pix_scan_tick = HAL_GetTick();   //awal mencoba baud sekarang (autobaud)
  uint8_t  baud_idx = 0;
#if !PIX_AUTOBAUD
  pix_locked = 1;
#endif

  int nb = snprintf(out, sizeof(out), "\r\n[AIR " FW_VER "] start, E32 st=%u, Pixhawk %lu baud, giliran=%u\r\n",
                    e32_status, pix_baud, LINK_TURNS);
  HAL_Delay(1500);   //beri waktu USB terdeteksi laptop
  Log(out, nb);
  pix_scan_tick = HAL_GetTick();
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
		if (MavTrack_Feed(&pix_trk, b, now, 20) && pix_trk.msg == 0 && pix_trk.ok)
		{
			pix_hb++;			//HEARTBEAT valid -> baud & kabel Pixhawk benar
			last_pix_hb = now;
			pix_locked = 1;
		}
	}

#if PIX_AUTOBAUD
	//1b) Autobaud: belum ada HEARTBEAT valid -> coba baud berikutnya
	if (pix_locked && (int32_t)(now - last_pix_hb) > PIX_LOST_MS)
	{
		pix_locked = 0;		//Pixhawk hilang / ganti baud -> cari lagi
		pix_scan_tick = now;
	}
	if (!pix_locked && now - pix_scan_tick >= PIX_SCAN_MS)
	{
		baud_idx = (baud_idx + 1) % (sizeof(pix_bauds) / sizeof(pix_bauds[0]));
		PIX_SetBaud(pix_bauds[baud_idx]);
		pix_scan_tick = now;
	}
#endif
	//(signed: mav_tick bisa lebih baru dari 'now' -> tanpa ini paket utuh ikut terbuang)
	if (mav_len > 0 && (int32_t)(HAL_GetTick() - mav_tick) > 20)
	{
		mav_len = 0;	//paket terpotong (byte hilang di kabel) -> buang
		mav_junk++;
	}

	//2) Antrean radio -> E32 dengan giliran: memancar maks. AIR_TX_SLOT_MS, kirim tanda giliran
	//   (RADIO_STATUS), lalu diam AIR_LISTEN_MS supaya perintah Mission Planner bisa masuk.
	//   Kirim hanya saat AUX HIGH (buffer E32 kosong), maks. 1 sub-paket; jeda 3 ms agar AUX sempat turun.
	uint8_t aux_ready = (now - last_burst >= 3) &&
	                    HAL_GPIO_ReadPin(E32_AUX_GPIO_Port, E32_AUX_Pin) == GPIO_PIN_SET;
	if (air_phase == AIR_PH_LISTEN)
	{
		if ((int32_t)(now - listen_end) >= 0) air_phase = AIR_PH_TX;
	}
	else if (air_phase == AIR_PH_MARK)
	{
		if (aux_ready)	//tanda giliran selesai dipancarkan -> mulai mendengar
		{
			air_phase = AIR_PH_LISTEN;
			listen_start = now;
			listen_end = now + AIR_LISTEN_MS;
		}
	}
	else if (aux_ready)
	{
		if (LINK_TURNS && slot_active && now - slot_start >= AIR_TX_SLOT_MS && !MavTrack_Busy(&tx_trk, now, 0))
		{
			uint32_t n = RS_Build(e32_tx, Radio_FreePct(), (uint16_t)mav_drop);
			HAL_UART_Transmit(&huart2, e32_tx, n, 50);
			up_bytes += n;
			marks++;
			slot_active = 0;
			air_phase = AIR_PH_MARK;
			last_burst = last_act = HAL_GetTick();
		}
		else if (Radio_Pending())
		{
			if (!slot_active) { slot_active = 1; slot_start = now; }
			//ganti antrean hanya di batas paket (potongan paket dari antrean lain tidak boleh tersisip)
			if (!MavTrack_Busy(&tx_trk, now, 0) || Ring_Count(&radio_q[tx_q]) == 0)
			{
				tx_trk.len = 0;
				tx_q = Radio_Pick();
			}
			uint32_t n = Ring_Peek(&radio_q[tx_q], e32_tx, E32_BURST);
			HAL_UART_Transmit(&huart2, e32_tx, n, 50);
			Ring_Drop(&radio_q[tx_q], n);
			for (uint32_t k = 0; k < n; k++) MavTrack_Feed(&tx_trk, e32_tx[k], now, 0);
			up_bytes += n;
			last_burst = last_act = HAL_GetTick();
		}
	}

	//3) E32 (perintah dari Mission Planner) -> Pixhawk
	if (Ring_Count(&radio_rx) > 0)
	{
		uint32_t n = 0;
		while (n < sizeof(pix_tx) && Ring_Get(&radio_rx, &b))
		{
			pix_tx[n++] = b;
			if (MavTrack_Feed(&up_trk, b, now, UP_GAP_MS) && up_trk.msg == 0 && up_trk.ok)
				gcs_hb++;	//HEARTBEAT Mission Planner sampai di pesawat -> uplink jalan
		}
		HAL_UART_Transmit(&huart1, pix_tx, n, 50);
		down_bytes += n;
		last_act = HAL_GetTick();

		if (air_phase == AIR_PH_LISTEN)	//GCS sedang bicara -> perpanjang waktu mendengar
		{
			listen_hit++;
			uint32_t e = now + AIR_LISTEN_EXT_MS;
			if ((int32_t)(e - listen_end) > 0) listen_end = e;
			if (listen_end - listen_start > AIR_LISTEN_MAX_MS) listen_end = listen_start + AIR_LISTEN_MAX_MS;
		}
	}

	//3b) RADIO_STATUS ke Pixhawk (flow control PX4) - hanya di antara paket perintah GCS
	if (pix_locked && now - last_rs >= PIX_RS_MS && Ring_Count(&radio_rx) == 0 && !MavTrack_Busy(&up_trk, now, UP_GAP_MS))
	{
		uint32_t n = RS_Build(pix_tx, Radio_FreePct(), (uint16_t)mav_drop);
		HAL_UART_Transmit(&huart1, pix_tx, n, 20);
		last_rs = now;
		rs_pix++;
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
				"[AIR " FW_VER "] st=%u cfg=%02X %02X %02X %02X %02X %02X | baud=%lu lock=%u ap=%u ftp=%lu pix_hb=%lu mav_ok=%lu par=%lu drop=%lu/%lu/%lu junk=%lu"
				" | up=%lu down=%lu gcs_hb=%lu | pq=%lu txq=%lu%% rs=%lu mark=%lu hit=%lu | noise=%lu/%lu rdrop=%lu\r\n",
				e32_status, e32_cfg[0], e32_cfg[1], e32_cfg[2], e32_cfg[3], e32_cfg[4], e32_cfg[5],
				pix_baud, pix_locked, ap_type, ftp_strip, pix_hb, mav_ok, par_ok, q_drop[0], q_drop[1], q_drop[2], mav_junk,
				up_bytes, down_bytes, gcs_hb, Ring_Count(&radio_q[1]),
				(unsigned long)(100 - Radio_FreePct()), rs_pix, marks, listen_hit,
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
