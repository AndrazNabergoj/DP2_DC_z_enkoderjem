/**************************************************************
* FILE:         SPI_dajalnik.c 
* DESCRIPTION:  SPI_dajalnik Initialization & Support Functions
* VERSION:      1.0
* AUTHOR:       Mitja Nemec
**************************************************************/
#include 	"DSP28x_Project.h"
#include	"SPI_dajalnik.h"
#include    "define.h"

// prototipi funkcij
long  SPI_getkot(void);
void SPI_init(void);

/**************************************************************
* Funckija ki inicializira komunikacijo
**************************************************************/
void SPI_init(void)
{
    long spi_modul_clock = 1;
    long spi_brr = 1;

    int  spi_psc = 0;

    if (SysCtrlRegs.LOSPCP.bit.LSPCLK == 0)
    {
        spi_psc = 1;
    }
    else
    {
        spi_psc = SysCtrlRegs.LOSPCP.bit.LSPCLK << 1;
    }

    // izraèunam vrednost za baud rate register
    spi_modul_clock = ((long)CPU_FREQ) / spi_psc;

    spi_brr = (spi_modul_clock / SPI_CLK) - 1;

    if (spi_brr < 0)
    {
        spi_brr = 0;
    }

    // najprej nastavmo GPIO pine
    EALLOW;
    SPI_PIN_DAT = SPI_MUX_VALUE;
    SPI_PIN_CLK = SPI_MUX_VALUE;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;

    EDIS;

    // zdaj pa inicializiramo SPI vmesnik za FIFO delovanje
    SPI_MODUL.SPICCR.bit.SPISWRESET = 0;     // reset SPI

    // clock polarity
    SPI_MODUL.SPICCR.bit.CLKPOLARITY = 0;    // clock polarity
    SPI_MODUL.SPICCR.bit.SPILBK = 0;         // loopback disabled
    SPI_MODUL.SPICCR.bit.SPICHAR = SPI_BITS - 1;       // bit lenght
    SPI_MODUL.SPICTL.bit.OVERRUNINTENA = 0;  // no overrun interrupta

    // clock phase
    SPI_MODUL.SPICTL.bit.CLK_PHASE = 0;      // clock phase
    SPI_MODUL.SPICTL.bit.MASTER_SLAVE = 1;   // SPI is master
    SPI_MODUL.SPICTL.bit.TALK = 1;           // enable transmit
    SPI_MODUL.SPICTL.bit.SPIINTENA = 0;      // no interrupts
    SPI_MODUL.SPISTS.bit.BUFFULL_FLAG = 1;   // clear flags
    SPI_MODUL.SPISTS.bit.INT_FLAG = 1;       // clear flags
    SPI_MODUL.SPISTS.bit.OVERRUN_FLAG = 1;   // clear flags

    SPI_MODUL.SPIBRR = spi_brr;          // SPICLK =   Mhz

    SPI_MODUL.SPIFFTX.bit.SPIRST = 0;        // SPI FIFO reset
    SPI_MODUL.SPIFFTX.bit.SPIFFENA = 1;      // enable FIFO
    SPI_MODUL.SPIFFTX.bit.TXFIFO = 0;        // reset FIFO cnt
    SPI_MODUL.SPIFFTX.bit.TXFIFO = 1;        // enable FIFO cnt
    SPI_MODUL.SPIFFTX.bit.TXFFIENA = 0;      // no FIFO interrupts
    SPI_MODUL.SPIFFCT.bit.TXDLY = 0;         // 0 delay bits
    SPI_MODUL.SPIFFTX.bit.TXFFINTCLR = 1;    // clear flags
    SPI_MODUL.SPIFFRX.bit.RXFIFORESET = 0;   // put out of reset
    SPI_MODUL.SPIFFRX.bit.RXFFIENA = 0;      // no RX interrupts
    SPI_MODUL.SPIFFRX.bit.RXFFINTCLR = 1;    // clear flags
    SPI_MODUL.SPIFFRX.bit.RXFFOVFCLR = 1;    // clear flags
    SPI_MODUL.SPIFFRX.bit.RXFIFORESET = 1;   // put out of reset
    SPI_MODUL.SPIFFTX.bit.SPIRST = 1;        // SPI FIFO out of reset
    SPI_MODUL.SPICCR.bit.SPISWRESET = 1;     // put out ofreset SPI

    // zazenem komunikacijo
    SPI_getkot();
    DELAY_US(50);
    SPI_getkot();
    DELAY_US(50);
    SPI_getkot();
    DELAY_US(50);

}

/**************************************************************
* Funckija ki komunicira z sPI dajalnikom položaja
* returns:  koda, ki jo vrne dajalnik
**************************************************************/
long SPI_getkot(void)
{
#if SPI_WORDS == 2
    long temp1 = 0;
    long temp2 = 0;

    long koda;

    // preberem podatek
    temp1 = SPI_MODUL.SPIRXBUF;
    temp2 = SPI_MODUL.SPIRXBUF;

    // izlocim podatek
    koda = (temp1 << 16);
    koda = koda | temp2;

    koda = koda & ((((long)1) << (SPI_MSB)) - 1);

    koda = koda >> SPI_LSB;

    // posljem nov paket, da znova dobim podatek
    SPI_MODUL.SPITXBUF = 0x0000;
    SPI_MODUL.SPITXBUF = 0x0000;
#endif

#if SPI_WORDS == 1
    long temp1 = 0;

    long koda;

    // preberem podatek
    temp1 = SPI_MODUL.SPIRXBUF;

    // izlocim podatek
    koda = temp1;

    koda = koda & ((((long)1) << (SPI_MSB)) - 1);

    koda = koda >> SPI_LSB;

    // posljem nov paket, da znova dobim podatek
    SPI_MODUL.SPITXBUF = 0x0000;
#endif

    return(koda);
}
