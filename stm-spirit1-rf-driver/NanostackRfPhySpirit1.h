#ifndef NANOSTACK_RF_PHY_SPIRIT1_H_
#define NANOSTACK_RF_PHY_SPIRIT1_H_

#include <stdint.h>

#include "NanostackRfPhy.h"
#include "PinNames.h"

// Arduino pin defaults for convenience
#if !defined(SPIRIT1_SPI_MOSI)
#define SPIRIT1_SPI_MOSI   D11
#endif
#if !defined(SPIRIT1_SPI_MISO)
#define SPIRIT1_SPI_MISO   D12
#endif
#if !defined(SPIRIT1_SPI_SCLK)
#define SPIRIT1_SPI_SCLK   D13
#endif
#if !defined(SPIRIT1_DEV_IRQ)
#define SPIRIT1_DEV_IRQ    D9
#endif
#if !defined(SPIRIT1_DEV_CS)
#define SPIRIT1_DEV_CS     D10
#endif
#if !defined(SPIRIT1_DEV_SDN)
#define SPIRIT1_DEV_SDN    D2
#endif
#if !defined(SPIRIT1_BRD_LED)
#define SPIRIT1_BRD_LED    NC
#endif

class NanostackRfPhySpirit1 : public NanostackRfPhy {
public:
    NanostackRfPhySpirit1(PinName spi_mosi, PinName spi_miso, PinName spi_sclk,
    		PinName dev_irq,  PinName dev_cs, PinName dev_sdn, PinName brd_led);
    ~NanostackRfPhySpirit1();
    int8_t rf_register();
    void rf_unregister();
    void get_mac_address(uint8_t *mac);
    void set_mac_address(uint8_t *mac);

private:
    void rf_init(void);

    const PinName _spi_mosi;
    const PinName _spi_miso;
    const PinName _spi_sclk;
    const PinName _dev_irq;
    const PinName _dev_cs;
    const PinName _dev_sdn;
    const PinName _brd_led;
};

#endif /* NANOSTACK_RF_PHY_SPIRIT1_H_ */
