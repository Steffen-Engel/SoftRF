
/* Peripherals */
#define SOC_GPIO_PIN_CONS_T2000_RX    _PINNUM(1,  8) // P1.08
#define SOC_GPIO_PIN_CONS_T2000_TX    _PINNUM(0,  4) // P0.04

/* Quectel L76K */
#define SOC_GPIO_PIN_GNSS_T2000_RX    _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_GNSS_T2000_TX    _PINNUM(0, 27) // P0.27

#define SOC_GPIO_PIN_GNSS_T2000_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_T2000_EN    _PINNUM(1,  5) // P1.05 active HIGH
#define SOC_GPIO_PIN_GNSS_T2000_WKE   _PINNUM(1,  9) // P1.09
#define SOC_GPIO_PIN_GNSS_T2000_RST   _PINNUM(1,  6) // P1.06

/* SPI */
#define SOC_GPIO_PIN_T2000_MOSI       _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_T2000_MISO       _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_T2000_SCK        _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_T2000_SS         _PINNUM(1, 14) // P1.14

/* SX1262 with SKY13453 RF switch */
#define SOC_GPIO_PIN_T2000_RST        _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_T2000_DIO1       _PINNUM(0,  7) // P0.07
#define SOC_GPIO_PIN_T2000_BUSY       _PINNUM(1, 10) // P1.10

/* I2C */
#define SOC_GPIO_PIN_T2000_SDA        _PINNUM(0,  6) // P0.06
#define SOC_GPIO_PIN_T2000_SCL        _PINNUM(0,  5) // P0.05

/* buttons */
#define SOC_GPIO_PIN_T2000_BUTTON     _PINNUM(0, 11) // P0.11 active LOW
#define SOC_GPIO_PIN_T2000_TP         _PINNUM(0,  8) // P0.08 TI DRV5032
#define SOC_GPIO_PIN_T2000_DFU        _PINNUM(1,  1) // P1.01

/* LED */
#define SOC_GPIO_LED_T2000_GREEN      _PINNUM(1,  2) // P1.02 active HIGH
#define SOC_GPIO_LED_T2000_RED        _PINNUM(0, 19) // P0.19 active HIGH
#define SOC_GPIO_LED_T2000_BLUE       _PINNUM(1,  3) // P1.03 active HIGH

/* ADC */
#define SOC_GPIO_PIN_T2000_BATTERY    _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_T2000_VBAT_EN    _PINNUM(0,  2) // P0.02 active HIGH

#define SOC_ADC_T2000_VOLTAGE_DIV     (2.0F) // 100K + 100K voltage divider on VBAT

/* battery charger CN3165 */
#define SOC_GPIO_PIN_T2000_DCDC       _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_T2000_CHG_STATUS _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_T2000_CHG_DONE   _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_T2000_VBUS_DET   _PINNUM(0, 29) // P0.29

/* LSM6DSOWTR */
#define SOC_GPIO_PIN_T2000_ACC_EN     _PINNUM(0, 15) // P0.15 active HIGH
#define SOC_GPIO_PIN_T2000_ACC_INT1   _PINNUM(0, 13) // P0.13
#define SOC_GPIO_PIN_T2000_ACC_INT2   _PINNUM(0, 14) // P0.14

/* POWER ON/OFF */
#define SOC_GPIO_PIN_T2000_3V3_EN     _PINNUM(0, 12) // P0.12 active HIGH

/* QSPI flash (P25Q16H) */
#if 0
#define SOC_GPIO_PIN_SFL_T2000_MOSI   _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_T2000_MISO   _PINNUM(0, 24) // P0.24
#define SOC_GPIO_PIN_SFL_T2000_SCK    _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_T2000_SS     _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_SFL_T2000_HOLD   _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_SFL_T2000_WP     _PINNUM(0, 22) // P0.22

#define SOC_GPIO_PIN_SFL_T2000_EN     _PINNUM(1, 13) // P1.13 active HIGH
#else
#define SOC_GPIO_PIN_SFL_T2000_MOSI   _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_SFL_T2000_MISO   _PINNUM(0, 21) // P0.21
#define SOC_GPIO_PIN_SFL_T2000_SCK    _PINNUM(0, 19) // P0.19 shared with red LED
#define SOC_GPIO_PIN_SFL_T2000_SS     _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_T2000_HOLD   _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_SFL_T2000_WP     _PINNUM(0, 22) // P0.22
#endif

/* UART connected to a Wi-Fi adapter */
#define SOC_GPIO_PIN_WIFI_T2000_RX    _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_WIFI_T2000_TX    _PINNUM(0, 16) // P0.16
#define SOC_GPIO_PIN_WIFI_T2000_EN    _PINNUM(1,  4) // P1.04 active HIGH
#define SOC_GPIO_PIN_WIFI_T2000_RST   _PINNUM(1,  0) // P1.00

/* misc. */
#define SOC_GPIO_PIN_T2000_MCU_RESET  _PINNUM(0, 18) // P0.18
