#ifndef __W25N01G_H
#define __W25N01G_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "main.h"
#include "spi.h"
#include "lfs.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct lfs_config cfg_lfs;

// define CS pin
#define W25N01_CS_Pin          CS_FLASH_GPIO_PIN
#define W25N01_CS_GPIO_Port    CS_FLASH_GPIO_PORT
#define HANDLER_FLASH          hspi2
#define W25NXX_TIMOUT         500
#define W25NXX_DEFAULT_TIMEOUT  200

// Device size parameters
#define W25N01G_PAGE_SIZE                2048
#define W25N01G_PAGES_PER_BLOCK            64
#define W25N01G_BLOCKS_PER_DIE            1024

#define W25N01G_STATUS_REGISTER_SIZE          8
#define W25N01G_STATUS_PAGE_ADDRESS_SIZE      16
#define W25N01G_STATUS_COLUMN_ADDRESS_SIZE    16

#define W25N01GXX_ID                          (0xEFAA21U)

#define W25N01G_INSTR_READ_DATA                   0x03
#define W25N01G_POWER_DOWN                        0xB9
#define W25N01G_INSTR_DEVICE_RESET                0xFF
#define W25N01G_INSTR_JEDEC_ID                    0x9F
#define W25N01G_INSTR_READ_STATUS_REG              0x05
#define W25N01G_INSTR_READ_STATUS_ALTERNATE_REG    0x0F
#define W25N01G_INSTR_WRITE_STATUS_REG            0x01
#define W25N01G_INSTR_WRITE_STATUS_ALTERNATE_REG  0x1F
#define W25N01G_INSTR_WRITE_ENABLE                0x06
#define W25N01G_INSTR_WRITE_DISABLE                0x04
#define W25N01G_INSTR_BB_MANAGEMENT                0xA1
#define W25N01G_INSTR_READ_BBM_LUT                0xA5
#define W25N01G_INSTR_LAST_ECC_FAIL_PAGE_ADR      0xA9
#define W25N01G_INSTR_BLOCK_ERASE                  0xD8
#define W25N01G_INSTR_PROGRAM_DATA_LOAD            0x02
#define W25N01G_INSTR_RANDOM_PROGRAM_DATA_LOAD    0x84
#define W25N01G_INSTR_QUAD_PROGRAM_DATA_LOAD      0x32
#define W25N01G_INSTR_QUAD_RANDOM_PROGRAM_DATA_LOAD  0x34
#define W25N01G_INSTR_PROGRAM_EXECUTE              0x10
#define W25N01G_INSTR_PAGE_DATA_READ              0x13


#define W25N01G_PROT_REG  0xA0
#define W25N01G_CONF_REG  0xB0
#define W25N01G_STAT_REG  0xC0

// Bits in config/status register 1 (W25N01G_PROT_REG)
typedef struct protRegBits_structure {
  uint8_t srp1: 1;
  uint8_t wp_e: 1;
  uint8_t tb: 1;
  uint8_t pb0: 1;
  uint8_t pb1: 1;
  uint8_t pb2: 1;
  uint8_t pb3: 1;
  uint8_t srp2: 1;
} protRegBits;

// Bits in config/status register 1 (W25N01G_PROT_REG)
#define W25N01G_PROT_CLEAR          (0)
#define W25N01G_PROT_SRP1_ENABLE      (1 << 0)
#define W25N01G_PROT_WP_E_ENABLE      (1 << 1)
#define W25N01G_PROT_TB_ENABLE        (1 << 2)
#define W25N01G_PROT_PB0_ENABLE        (1 << 3)
#define W25N01G_PROT_PB1_ENABLE        (1 << 4)
#define W25N01G_PROT_PB2_ENABLE        (1 << 5)
#define W25N01G_PROT_PB3_ENABLE        (1 << 6)
#define W25N01G_PROT_SRP0_ENABLE      (1 << 7)

// Bits in config/status register 2 (W25N01G_CONF_REG)
#define W25N01G_CONFIG_ECC_ENABLE          (1 << 4)
#define W25N01G_CONFIG_BUFFER_READ_MODE    (1 << 3)

// Bits in config/status register 3       (W25N01G_STATREG)
#define W25N01G_STATUS_BBM_LUT_FULL        (1 << 6)
#define W25N01G_STATUS_FLAG_ECC_POS        4
#define W25N01G_STATUS_FLAG_ECC_MASK      ((1 << 5)|(1 << 4))
#define W25N01G_STATUS_FLAG_ECC(status)    (((status) & W25N01G_STATUS_FLAG_ECC_MASK) >> 4)
#define W25N01G_STATUS_PROGRAM_FAIL        (1 << 3)
#define W25N01G_STATUS_ERASE_FAIL          (1 << 2)
#define W25N01G_STATUS_FLAG_WRITE_ENABLED  (1 << 1)
#define W25N01G_STATUS_FLAG_BUSY          (1 << 0)


bool w25n_init();
void W25n01g_select(void);
void W25n01g_deselect(void);
int8_t w25n01g_deviceReset(void);
uint8_t W25n01g_wait_for_ready(void);
void W25n01g_read_sr(uint8_t ADD, uint8_t *SR1);
void W25n01g_read_jedec(uint8_t *idBuffer);
void W25n01g_write_sr(uint8_t ADD, uint8_t SR);
bool W25n01g_write_enable();
void W25n01g_write_disable();
bool w25n01g_block_erase(uint32_t address);
void w25n01g_erase_completely();
bool W25n01g_program_data_load(uint16_t columnAddress, const uint8_t *data, uint32_t length);
bool w25n01g_page_program(uint32_t address, const uint8_t *data, uint32_t length);
bool w25n01g_write_flash(uint32_t address, const uint8_t *data, uint32_t length);
uint32_t w25n01g_read_bytes(uint32_t address, uint8_t *buffer, uint32_t length);


int lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, void *buffer, lfs_size_t size);
int lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, const void *buffer, lfs_size_t size);
int lfs_erase(const struct lfs_config *c, lfs_block_t block);
int lfs_sync(const struct lfs_config *c);

#ifdef __cplusplus
}
#endif

#endif /* __W25N01G_H */
