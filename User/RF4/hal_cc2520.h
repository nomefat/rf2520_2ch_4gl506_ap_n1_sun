#ifndef _HAL_CC2520_H
#define _HAL_CC2520_H



/***********************************************************************************
* CONSTANTS AND DEFINES
*/

// TXPOWER values
#define CC2520_TXPOWER_MIN_4_DBM        0x88  // Gives ~ -3.1 dBm TX output power
#define CC2520_TXPOWER_0_DBM            0x32  // Gives ~ 0.3 dBm TX output power
#define CC2520_TXPOWER_4_DBM            0xF7  // Gives ~ 4.6 dBm TX output power

#define CC2520_RSSI_OFFSET                76  // dBm (from data sheet)

// CC2520 pin assignments
#define CC2520_RX_FRM_DONE_PIN              CC2520_GPIO0_IPIN
#define CC2520_SAMPLED_CCA_PIN              CC2520_GPIO1_IPIN
// GPIO2 are reused for RSSI Valid or TX_FRM_DONE
#define CC2520_RSSI_VALID_PIN               CC2520_GPIO2_IPIN
#define CC2520_TX_FRM_DONE_PIN              CC2520_GPIO2_IPIN
#define CC2520_SFD_PIN                      CC2520_GPIO3_IPIN
#define CC2520_GPIO_SNIFFER_DATA_PIN        CC2520_GPIO4_IPIN
#define CC2520_GPIO_SNIFFER_CLK_PIN         CC2520_GPIO5_IPIN


#define CC2520_MEMORY_SIZE                  0x400

// Startup time values (in microseconds)
#define CC2520_XOSC_MAX_STARTUP_TIME        300
#define CC2520_VREG_MAX_STARTUP_TIME        200
#define CC2520_SRXON_TO_RANDOM_READY_TIME   144

// Address definitions

// FREG definitions (BSET/BCLR supported)
#define CC2520_FRMFILT0                0x000
#define CC2520_FRMFILT1                0x001
#define CC2520_SRCMATCH                0x002
#define CC2520_SRCSHORTEN0             0x004
#define CC2520_SRCSHORTEN1             0x005
#define CC2520_SRCSHORTEN2             0x006
#define CC2520_SRCEXTEN0               0x008
#define CC2520_SRCEXTEN1               0x009
#define CC2520_SRCEXTEN2               0x00A
#define CC2520_FRMCTRL0                0x00C
#define CC2520_FRMCTRL1                0x00D
#define CC2520_RXENABLE0               0x00E
#define CC2520_RXENABLE1               0x00F
#define CC2520_EXCFLAG0                0x010
#define CC2520_EXCFLAG1                0x011
#define CC2520_EXCFLAG2                0x012
#define CC2520_EXCMASKA0               0x014
#define CC2520_EXCMASKA1               0x015
#define CC2520_EXCMASKA2               0x016
#define CC2520_EXCMASKB0               0x018
#define CC2520_EXCMASKB1               0x019
#define CC2520_EXCMASKB2               0x01A
#define CC2520_EXCBINDX0               0x01C
#define CC2520_EXCBINDX1               0x01D
#define CC2520_EXCBINDY0               0x01E
#define CC2520_EXCBINDY1               0x01F
#define CC2520_GPIOCTRL0               0x020
#define CC2520_GPIOCTRL1               0x021
#define CC2520_GPIOCTRL2               0x022
#define CC2520_GPIOCTRL3               0x023
#define CC2520_GPIOCTRL4               0x024
#define CC2520_GPIOCTRL5               0x025
#define CC2520_GPIOPOLARITY            0x026
#define CC2520_GPIOCTRL                0x028
#define CC2520_DPUCON                  0x02A
#define CC2520_DPUSTAT                 0x02C
#define CC2520_FREQCTRL                0x02E
#define CC2520_FREQTUNE                0x02F
#define CC2520_TXPOWER                 0x030
#define CC2520_TXCTRL                  0x031
#define CC2520_FSMSTAT0                0x032
#define CC2520_FSMSTAT1                0x033
#define CC2520_FIFOPCTRL               0x034
#define CC2520_FSMCTRL                 0x035
#define CC2520_CCACTRL0                0x036
#define CC2520_CCACTRL1                0x037
#define CC2520_RSSI                    0x038
#define CC2520_RSSISTAT                0x039
#define CC2520_TXFIFO_BUF              0x03A
#define CC2520_RXFIRST                 0x03C
#define CC2520_RXFIFOCNT               0x03E
#define CC2520_TXFIFOCNT               0x03F

// SREG definitions (BSET/BCLR unsupported)
#define CC2520_CHIPID                  0x040
#define CC2520_VERSION                 0x042
#define CC2520_EXTCLOCK                0x044
#define CC2520_MDMCTRL0                0x046
#define CC2520_MDMCTRL1                0x047
#define CC2520_FREQEST                 0x048
#define CC2520_RXCTRL                  0x04A
#define CC2520_FSCTRL                  0x04C
#define CC2520_FSCAL0                  0x04E
#define CC2520_FSCAL1                  0x04F
#define CC2520_FSCAL2                  0x050
#define CC2520_FSCAL3                  0x051
#define CC2520_AGCCTRL0                0x052
#define CC2520_AGCCTRL1                0x053
#define CC2520_AGCCTRL2                0x054
#define CC2520_AGCCTRL3                0x055
#define CC2520_ADCTEST0                0x056
#define CC2520_ADCTEST1                0x057
#define CC2520_ADCTEST2                0x058
#define CC2520_MDMTEST0                0x05A
#define CC2520_MDMTEST1                0x05B
#define CC2520_DACTEST0                0x05C
#define CC2520_DACTEST1                0x05D
#define CC2520_ATEST                   0x05E
#define CC2520_DACTEST2                0x05F
#define CC2520_PTEST0                  0x060
#define CC2520_PTEST1                  0x061
#define CC2520_RESERVED                0x062
#define CC2520_DPUBIST                 0x07A
#define CC2520_ACTBIST                 0x07C
#define CC2520_RAMBIST                 0x07E

// RAM size definitions
#define CC2520_RAM_START_ADDR          0x100
#define CC2520_RAM_SIZE                640

// RX and TX buffer definitions
#define CC2520_RAM_TXBUF               0x100
#define CC2520_RAM_RXBUF               0x180

// Security work buffer definitions
#define CC2520_RAM_CBCTEMPL            0x360
#define CC2520_RAM_CBCTEMPH            0x370

// Source address matching definitions
#define CC2520_RAM_SRCTABLEBASE        0x380
#define CC2520_RAM_SRCRESMASK0         0x3E0
#define CC2520_RAM_SRCRESMASK1         0x3E1
#define CC2520_RAM_SRCRESMASK2         0x3E2
#define CC2520_RAM_SRCRESINDEX         0x3E3
#define CC2520_RAM_SRCEXTPENDEN0       0x3E4
#define CC2520_RAM_SRCEXTPENDEN1       0x3E5
#define CC2520_RAM_SRCEXTPENDEN2       0x3E6
#define CC2520_RAM_SRCSHORTPENDEN0     0x3E7
#define CC2520_RAM_SRCSHORTPENDEN1     0x3E8
#define CC2520_RAM_SRCSHORTPENDEN2     0x3E9

// Local address definitions
#define CC2520_RAM_EXTADDR             0x3EA
#define CC2520_RAM_PANID               0x3F2
#define CC2520_RAM_SHORTADDR           0x3F4

// Command strobes (identifiers to be used with GPIO and exception binding)
#define CC2520_STR_SIBUFEX             0x00
#define CC2520_STR_SRXMASKBITCLR       0x01
#define CC2520_STR_SRXMASKBITSET       0x02
#define CC2520_STR_SRXON               0x03
#define CC2520_STR_SSAMPLECCA          0x04
#define CC2520_STR_SACK                0x05
#define CC2520_STR_SACKPEND            0x06
#define CC2520_STR_SNACK               0x07
#define CC2520_STR_STXON               0x08
#define CC2520_STR_STXONCCA            0x09
#define CC2520_STR_SFLUSHRX            0x0A
#define CC2520_STR_SFLUSHTX            0x0B
#define CC2520_STR_SRXFIFOPOP          0x0C
#define CC2520_STR_STXCAL              0x0D
#define CC2520_STR_SRFOFF              0x0E
#define CC2520_STR_SXOSCOFF            0x0F

// Exceptions
#define CC2520_EXC_RF_IDLE             0
#define CC2520_EXC_TX_FRM_DONE         1
#define CC2520_EXC_TX_ACK_DONE         2
#define CC2520_EXC_TX_UNDERFLOW        3
#define CC2520_EXC_TX_OVERFLOW         4
#define CC2520_EXC_RX_UNDERFLOW        5
#define CC2520_EXC_RX_OVERFLOW         6
#define CC2520_EXC_RXENABLE_ZERO       7
#define CC2520_EXC_RX_FRM_DONE         8
#define CC2520_EXC_RX_FRM_ACCEPTED     9
#define CC2520_EXC_SRC_MATCH_DONE      10
#define CC2520_EXC_SRC_MATCH_FOUND     11
#define CC2520_EXC_FIFOP               12
#define CC2520_EXC_SFD                 13
#define CC2520_EXC_DPU_DONE_L          14
#define CC2520_EXC_DPU_DONE_H          15
#define CC2520_EXC_MEMADDR_ERROR       16
#define CC2520_EXC_USAGE_ERROR         17
#define CC2520_EXC_OPERAND_ERROR       18
#define CC2520_EXC_SPI_ERROR           19
#define CC2520_EXC_RF_NO_LOCK          20
#define CC2520_EXC_RX_FRM_ABORTED      21
#define CC2520_EXC_RXBUFMOV_TIMEOUT    22

// Predefined exception channels
#define CC2520_EXC_CH_RX_BV           \
    (BV(CC2520_EXC_RX_UNDERFLOW) | BV(CC2520_EXC_RX_OVERFLOW) \
        | BV(CC2520_EXC_RX_FRM_ABORTED) | BV(CC2520_EXC_RXBUFMOV_TIMEOUT))
#define CC2520_EXC_CH_ERR_BV          \
    (BV(CC2520_EXC_MEMADDR_ERROR) | BV(CC2520_EXC_USAGE_ERROR) \
        | BV(CC2520_EXC_OPERAND_ERROR) | BV(CC2520_EXC_SPI_ERROR))

// CC2520 FSM state defintions
#define CC2520_FSM_IDLE                 0
#define CC2520_FSM_FS_RX_CAL            2
#define CC2520_FSM_RX_RESET             3
#define CC2520_FSM_RX_S_CLK             4
#define CC2520_FSM_RX_ON                5
#define CC2520_FSM_RX_W_SFD             6
#define CC2520_FSM_RD_LEN               7
#define CC2520_FSM_RX_RD_FCF0           8
#define CC2520_FSM_RX_RD_FCF1           9
#define CC2520_FSM_RX_RD_SEQ           10
#define CC2520_FSM_RX_PKT              12
#define CC2520_FSM_RX_FINAL            13
#define CC2520_FSM_RXRX_WAIT           14
#define CC2520_FSM_FIFO_RESET          16
#define CC2520_FSM_RX_OVERFLOW         17
#define CC2520_FSM_FS_LB_CAL_WAIT      20
#define CC2520_FSM_LB_RESET            21
#define CC2520_FSM_LB_S_CLK            22
#define CC2520_FSM_LB_PRE              23
#define CC2520_FSM_LB_PKT              24
#define CC2520_FSM_LB_FINAL            25
#define CC2520_FSM_RX_INF              31
#define CC2520_FSM_FS_TX_CAL           32
#define CC2520_FSM_FS_TX_CAL_WAIT      33
#define CC2520_FSM_TX_RESET            34
#define CC2520_FSM_TX_S_CLK            35
#define CC2520_FSM_TX_PRE              36
#define CC2520_FSM_LEN                 37
#define CC2520_FSM_PKT                 38
#define CC2520_FSM_TX_FINAL            39
#define CC2520_FSM_TXRX_TRANSIT        40
#define CC2520_FSM_PA_SETTLE           41
#define CC2520_FSM_TX_MOD_DOWNRAMP     42
#define CC2520_FSM_FS_ACK_CAL          48
#define CC2520_FSM_TX_ACK_RESET        49
#define CC2520_FSM_TX_ACK_S_CLK        50
#define CC2520_FSM_TX_ACK_PRE          51
#define CC2520_FSM_TX_ACK_LEN          52
#define CC2520_FSM_TX_ACK_PKT          53
#define CC2520_FSM_TX_ACK_FIN_PKT      54
#define CC2520_FSM_TX_ACK_DELAY        55
#define CC2520_FSM_TX_UNDERFLOW        56
#define CC2520_FSM_TX_TURNOFF          57
#define CC2520_FSM_TX_INF              63

// CC2520 FSMSTAT0/1 register bits
#define CC2520_FSMSTAT_FSM_STATE_BV    0x003F
#define CC2520_FSMSTAT_CAL_RUNNING_BV  0x0040
#define CC2520_FSMSTAT_CAL_DONE_BV     0x0080
#define CC2520_FSMSTAT_RX_ACTIVE_BV    0x0100
#define CC2520_FSMSTAT_TX_ACTIVE_BV    0x0200
#define CC2520_FSMSTAT_LOCK_STATUS_BV  0x0400
#define CC2520_FSMSTAT_SAMPLED_CCA_BV  0x0800
#define CC2520_FSMSTAT_CCA_BV          0x1000
#define CC2520_FSMSTAT_SFD_BV          0x2000
#define CC2520_FSMSTAT_FIFOP_BV        0x4000
#define CC2520_FSMSTAT_FIFO_BV         0x8000

// CC2520 status byte
#define CC2520_STB_XOSC_STABLE_BV      0x80
#define CC2520_STB_RSSI_VALID_BV       0x40
#define CC2520_STB_EXC_CHB_BV          0x20
#define CC2520_STB_EXC_CHA_BV          0x10
#define CC2520_STB_DPUH_ACTIVE_BV      0x08
#define CC2520_STB_DPUL_ACTIVE_BV      0x04
#define CC2520_STB_TX_ACTIVE_BV        0x02
#define CC2520_STB_RX_ACTIVE_BV        0x01

// CC2520 GPIO output definitions
#define CC2520_GPIO_EXC_CH_A           0x21
#define CC2520_GPIO_EXC_CH_B           0x22
#define CC2520_GPIO_EXC_CH_INVA        0x23
#define CC2520_GPIO_EXC_CH_INVB        0x24
#define CC2520_GPIO_EXC_CH_RX          0x25
#define CC2520_GPIO_EXC_CH_ERR         0x26
#define CC2520_GPIO_FIFO               0x27
#define CC2520_GPIO_FIFOP              0x28
#define CC2520_GPIO_CCA                0x29
#define CC2520_GPIO_SFD                0x2A
#define CC2520_GPIO_RSSI_VALID         0x2C
#define CC2520_GPIO_SAMPLED_CCA        0x2D
#define CC2520_GPIO_SNIFFER_CLK        0x31
#define CC2520_GPIO_SNIFFER_DATA       0x32
#define CC2520_GPIO_RX_ACTIVE          0x43
#define CC2520_GPIO_TX_ACTIVE          0x44
#define CC2520_GPIO_LOW                0x7E
#define CC2520_GPIO_HIGH               0x7F

// Instruction implementation
#define CC2520_INS_SNOP                0x00
#define CC2520_INS_IBUFLD              0x02
#define CC2520_INS_SIBUFEX             0x03
#define CC2520_INS_SSAMPLECCA          0x04
#define CC2520_INS_SRES                0x0F
#define CC2520_INS_MEMRD               0x10
#define CC2520_INS_MEMWR               0x20
#define CC2520_INS_RXBUF               0x30
#define CC2520_INS_RXBUFCP             0x38
#define CC2520_INS_RXBUFMOV            0x32
#define CC2520_INS_TXBUF               0x3A
#define CC2520_INS_TXBUFCP             0x3E
#define CC2520_INS_RANDOM              0x3C
#define CC2520_INS_SXOSCON             0x40
#define CC2520_INS_STXCAL              0x41
#define CC2520_INS_SRXON               0x42
#define CC2520_INS_STXON               0x43
#define CC2520_INS_STXONCCA            0x44
#define CC2520_INS_SRFOFF              0x45
#define CC2520_INS_SXOSCOFF            0x46
#define CC2520_INS_SFLUSHRX            0x47
#define CC2520_INS_SFLUSHTX            0x48
#define CC2520_INS_SACK                0x49
#define CC2520_INS_SACKPEND            0x4A
#define CC2520_INS_SNACK               0x4B
#define CC2520_INS_SRXMASKBITSET       0x4C
#define CC2520_INS_SRXMASKBITCLR       0x4D
#define CC2520_INS_RXMASKAND           0x4E
#define CC2520_INS_RXMASKOR            0x4F
#define CC2520_INS_MEMCP               0x50
#define CC2520_INS_MEMCPR              0x52
#define CC2520_INS_MEMXCP              0x54
#define CC2520_INS_MEMXWR              0x56
#define CC2520_INS_BCLR                0x58
#define CC2520_INS_BSET                0x59
#define CC2520_INS_CTR                 0x60
#define CC2520_INS_CBCMAC              0x64
#define CC2520_INS_UCBCMAC             0x66
#define CC2520_INS_CCM                 0x68
#define CC2520_INS_UCCM                0x6A
#define CC2520_INS_ECB                 0x70
#define CC2520_INS_ECBO                0x72
#define CC2520_INS_ECBX                0x74
#define CC2520_INS_ECBXO               0x76
#define CC2520_INS_INC                 0x78
#define CC2520_INS_ABORT               0x7F
#define CC2520_INS_REGRD               0x80
#define CC2520_INS_REGWR               0xC0








#define RF2411_RX_BYPASS   0
#define RF2411_RX_LNA      1


#define RF2411_ANT_A 0
#define RF2411_ANT_B 1

















#endif

