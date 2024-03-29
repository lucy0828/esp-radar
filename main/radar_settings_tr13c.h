/* XENSIV BGT60TRXX register configurator, SDK versionv3.4.0+.e7acc0f10 */

#ifndef XENSIV_BGT60TRXX_CONF_H
#define XENSIV_BGT60TRXX_CONF_H

#define XENSIV_BGT60TRXX_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define XENSIV_BGT60TRXX_CONF_START_FREQ_HZ (58000000000)
#define XENSIV_BGT60TRXX_CONF_END_FREQ_HZ (62000000000)
#define XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP (128)
#define XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME (1)
#define XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS (1)
#define XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS (1)
#define XENSIV_BGT60TRXX_CONF_SAMPLE_RATE (200000)
#define XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S (0.000650037)
#define XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S (0.00100058)
#define XENSIV_BGT60TRXX_CONF_NUM_REGS (38)

#if defined(XENSIV_BGT60TRXX_CONF_IMPL)
const uint32_t register_list[] = { 
    0x11e8270UL, 
    0x3640210UL, 
    0x9e967fdUL, 
    0xb0805b4UL, 
    0xdf02c3fUL, 
    0xf010f00UL, 
    0x11000000UL, 
    0x13000000UL, 
    0x15000000UL, 
    0x17000be0UL, 
    0x19000000UL, 
    0x1b000000UL, 
    0x1d000000UL, 
    0x1f000b60UL, 
    0x21103c51UL, 
    0x231ff40aUL, 
    0x25004e73UL, 
    0x2d000490UL, 
    0x3b000480UL, 
    0x49000480UL, 
    0x57000480UL, 
    0x5911be0eUL, 
    0x5b25e20aUL, 
    0x5d000000UL, 
    0x5f787e1eUL, 
    0x61a9b596UL, 
    0x6300007fUL, 
    0x65001932UL, 
    0x67000080UL, 
    0x69000000UL, 
    0x6b000000UL, 
    0x6d000000UL, 
    0x6f00c110UL, 
    0x7f000100UL, 
    0x8f000100UL, 
    0x9f000100UL, 
    0xad000000UL, 
    0xb7000000UL
};
#endif /* XENSIV_BGT60TRXX_CONF_IMPL */

#endif /* XENSIV_BGT60TRXX_CONF_H */