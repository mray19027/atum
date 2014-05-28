
#include "contiki.h"
#include "rv3049.h"
#include "spi-arch.h"
#include "spi.h"
#include "dev/ssi.h"

uint8_t rv3049_binary_to_bcd (uint8_t binary) {
  uint8_t out = 0;

  if (binary >= 40) {
    out |= 0x40;
    binary -= 40;
  }
  if (binary >= 20) {
    out |= 0x20;
    binary -= 20;
  }
  if (binary >= 10) {
    out |= 0x10;
    binary -= 10;
  }
  out |= binary;
  return out;
}

void
rv3049_init()
{
  /* Set the HOLD_N and WP_N pins to outputs and high */
  GPIO_SET_INPUT(GPIO_PORT_TO_BASE(RV3049_INT_N_PORT_NUM),
                 GPIO_PIN_MASK(RV3049_INT_N_PIN));

  spi_cs_init(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);
}

int
rv3049_read_time(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  spi_set_mode(SSI_CR0_FRF_MOTOROLA, 0, 0, 8);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Tell the RTC we want to read the clock
  SPI_WRITE(RV3049_SET_READ_BIT(RV3049_PAGE_ADDR_CLOCK));

  SPI_FLUSH();

  // Then actually read the clock
  for (i=0; i<RV3049_READ_LEN_TIME; i++) {
    SPI_READ(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Convert the values
  time->seconds = BCD_TO_BINARY(buf[0]);
  time->minutes = BCD_TO_BINARY(buf[1]);
  time->hours   = BCD_TO_BINARY((buf[2])&0x3F);
  time->days    = BCD_TO_BINARY(buf[3]);
  time->weekday = buf[4];
  time->month   = buf[5];
  time->year    = BCD_TO_BINARY(buf[6])+2000;

  return 0;
}

int
rv3049_set_time(rv3049_time_t* time)
{
  uint8_t buf[8];
  int i;

  buf[0] = rv3049_binary_to_bcd(time->seconds);
  buf[1] = rv3049_binary_to_bcd(time->minutes);
  buf[2] = rv3049_binary_to_bcd(time->hours); // 24 hour mode
  buf[3] = rv3049_binary_to_bcd(time->days);
  buf[4] = time->weekday;
  buf[5] = time->month;
  buf[6] = rv3049_binary_to_bcd(time->year - 2000);

  SPI_CS_SET(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  // Signal a write to the clock
  SPI_WRITE(RV3049_SET_WRITE_BIT(RV3049_PAGE_ADDR_CLOCK));

  // Write the clock values
  for (i=0; i<RV3049_WRITE_LEN_TIME; i++) {
    SPI_WRITE(buf[i]);
  }

  SPI_CS_CLR(RV3049_CS_PORT_NUM, RV3049_CS_PIN);

  return 0;
}
