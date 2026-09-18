/**
 * @file w25n01g.c
 *
 * @brief Librairie de la puce flash NAND W25N01Gxxx
 * Gestion complète de la puce avec fonction d'init de lecture de suppression
 * Ajout des wrappers de fonction pour LittleFS
 *
 * Utilisation de la puce en mode READ_BUF=1 non continious
 *
 * @warning Fonctions dipo avec le flag à la compilation USE_W25N01G
 *
 */


#include "w25n01g.h"


#define LFS_BUFFER_SIZE     512
static uint8_t read_buffer[LFS_BUFFER_SIZE];
static uint8_t prog_buffer[LFS_BUFFER_SIZE];
static uint8_t lookahead_buffer[LFS_BUFFER_SIZE];

/**
 * @struct cfg_lfs
 * Structure de configuration pour LittleFS
 * Spécifique à la puce w25n01gxxx
 */
struct lfs_config cfg_lfs = {
        .read = lfs_read,
        .prog = lfs_prog,
        .erase = lfs_erase,
        .sync = lfs_sync,

        .read_size = LFS_BUFFER_SIZE,
        .prog_size = LFS_BUFFER_SIZE,
        .block_size = W25N01G_PAGE_SIZE * W25N01G_PAGES_PER_BLOCK,
        .block_count = W25N01G_BLOCKS_PER_DIE,
        .cache_size = LFS_BUFFER_SIZE,
        .lookahead_size = LFS_BUFFER_SIZE,
        .block_cycles = 500,

        .read_buffer = read_buffer,
        .prog_buffer = prog_buffer,
        .lookahead_buffer = lookahead_buffer
};

#define W25N01G_LINEAR_TO_COLUMN(laddr) ((laddr) % W25N01G_PAGE_SIZE)
#define W25N01G_LINEAR_TO_PAGE(laddr) (((laddr) / W25N01G_PAGE_SIZE) & 0xffff)
#define W25N01G_LINEAR_TO_BLOCK(laddr) (W25N01G_LINEAR_TO_PAGE(laddr) / W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_PAGE(block) ((block) * W25N01G_PAGES_PER_BLOCK)
#define W25N01G_BLOCK_TO_LINEAR(block) (W25N01G_BLOCK_TO_PAGE(block) * W25N01G_PAGE_SIZE)

#define W25N01G_STATUS_FLAG_ECC(status)   (((status) & W25N01G_STATUS_FLAG_ECC_MASK) >> 4)

static bool W25n01g_perform_command_with_page_address(SPI_HandleTypeDef *hspi, uint8_t command, uint32_t pageAddress);

/**
 * @brief Initialisation de la puce
 * Puce en mode read mode et avec ECC activé 0x18 registre de conf
 *
 * @return True si success, False sinon
 */
bool w25n_init()
{
  bool success = true;
  uint8_t buffer[3];

  w25n01g_deviceReset();

  // W25n01g_write_sr(W25N01G_PROT_REG, 0x00);
  // W25n01g_write_sr(W25N01G_CONF_REG, 0x18);

  W25n01g_read_jedec(buffer);

  uint32_t id_puce = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
  if (id_puce == W25N01GXX_ID) {
    success = true;
  } else {
    success = false;
  }

  HAL_Delay(100);
  return success;
}

int8_t w25n01g_deviceReset(void)
{
  uint32_t tick_start = HAL_GetTick();
  while (W25n01g_wait_for_ready()) {
    if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
      return -1;
    }
  }

  uint8_t cmd = W25N01G_INSTR_DEVICE_RESET;

  W25n01g_select();
  HAL_SPI_Transmit(&HANDLER_FLASH, &cmd, 1, W25NXX_DEFAULT_TIMEOUT);
  W25n01g_deselect();
  HAL_Delay(500);

  W25n01g_write_sr(W25N01G_PROT_REG, 0x00);
  W25n01g_write_sr(W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE|W25N01G_CONFIG_BUFFER_READ_MODE);

  return 0;
}

void W25n01g_select(void)
{
  HAL_GPIO_WritePin(W25N01_CS_GPIO_Port, W25N01_CS_Pin, GPIO_PIN_RESET);
}

void W25n01g_deselect(void)
{
  HAL_GPIO_WritePin(W25N01_CS_GPIO_Port, W25N01_CS_Pin, GPIO_PIN_SET);
}

void W25n01g_read_jedec(uint8_t *idBuffer)
{
  uint8_t cmd[2] = {W25N01G_INSTR_JEDEC_ID, 0x00};

  W25n01g_select();
  HAL_SPI_Transmit(&HANDLER_FLASH, cmd, sizeof(cmd), W25NXX_DEFAULT_TIMEOUT);
  HAL_SPI_Receive(&HANDLER_FLASH, idBuffer, 3, W25NXX_DEFAULT_TIMEOUT);
  W25n01g_deselect();
}

void W25n01g_write_sr(uint8_t ADD, uint8_t SR)
{
  uint32_t tick_start;
  uint8_t cmd[3] = {W25N01G_INSTR_WRITE_STATUS_REG, ADD, SR};
  W25n01g_select();
  HAL_SPI_Transmit(&HANDLER_FLASH, cmd, sizeof(cmd), W25NXX_TIMOUT);
  W25n01g_deselect();
  tick_start = HAL_GetTick();
  while (W25n01g_wait_for_ready()) {
    if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
      break;
    }
  }
}

void W25n01g_read_sr(uint8_t ADD, uint8_t *SR1)
{
  uint8_t cmd[3] = {W25N01G_INSTR_READ_STATUS_REG, ADD, 0};
  uint8_t temp[3];
  W25n01g_select();
  HAL_SPI_Transmit(&HANDLER_FLASH, cmd, sizeof(cmd), W25NXX_TIMOUT);
  HAL_SPI_Receive(&HANDLER_FLASH, temp, sizeof(cmd), W25NXX_TIMOUT);
  W25n01g_deselect();
  *SR1 = temp[2];
}


uint8_t W25n01g_wait_for_ready(void)
{
  uint8_t quad_bk1_sr;
  W25n01g_read_sr(W25N01G_STAT_REG, &quad_bk1_sr);
  return (quad_bk1_sr & 0x01);

  return 0;
}


bool W25n01g_write_enable()
{
  uint8_t cmd[1] = {W25N01G_INSTR_WRITE_ENABLE};
  W25n01g_select();
  HAL_SPI_Transmit(&HANDLER_FLASH, cmd, 1, W25NXX_TIMOUT);
  W25n01g_deselect();

  uint8_t statusReg;
  W25n01g_read_sr(W25N01G_STAT_REG, &statusReg);
  if (!(statusReg & W25N01G_STATUS_FLAG_WRITE_ENABLED)) {
    return false;
  } else {
    return true;
  }
}

void W25n01g_write_disable()
{
  uint8_t cmd[1] = {W25N01G_INSTR_WRITE_DISABLE};
  W25n01g_select();
  HAL_SPI_Transmit(&HANDLER_FLASH, cmd, 1, W25NXX_TIMOUT);
  W25n01g_deselect();
}

//semble ok
bool w25n01g_block_erase(uint32_t address)
{
  bool success = true;
  uint16_t pageAddress = W25N01G_LINEAR_TO_PAGE(address) & 0xffff;


  success = W25n01g_write_enable();
  if (success) {
    uint32_t tick_start = HAL_GetTick();
    while (W25n01g_wait_for_ready()) {
      if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
        return false;
      }
    }

    W25n01g_perform_command_with_page_address(&HANDLER_FLASH, W25N01G_INSTR_BLOCK_ERASE,
                                              W25N01G_LINEAR_TO_PAGE(address));


    /*uint8_t cmd[4] = {W25N01G_INSTR_BLOCK_ERASE, 0x00, (pageAddress >> 8) & 0xff, (pageAddress & 0xff)};

    W25N01_Select();
    HAL_SPI_Transmit(&HANDLER_SPI_FLASH, cmd, 4, W25NXX_TIMOUT);
    W25N01_Deselect();*/

    tick_start = HAL_GetTick();
    uint8_t quad_bk1_sr;
    W25n01g_read_sr(W25N01G_STAT_REG, &quad_bk1_sr);
    while (quad_bk1_sr & 0x01) {
      W25n01g_read_sr(W25N01G_STAT_REG, &quad_bk1_sr);
      if (HAL_GetTick() - tick_start > W25NXX_TIMOUT || (quad_bk1_sr & 0x04)) {
        return false;
      }
    }
    HAL_Delay(1);
    return true;
  }

  return success;
}

void w25n01g_erase_completely()
{
  for (uint32_t block = 0; block < W25N01G_BLOCKS_PER_DIE; block++) {
    w25n01g_block_erase(W25N01G_BLOCK_TO_LINEAR(block));
  }
}

//semble ok
static bool W25n01g_perform_command_with_page_address(SPI_HandleTypeDef *hspi, uint8_t command, uint32_t pageAddress)
{
  bool success = true;

  uint32_t tick_start = HAL_GetTick();
  while (W25n01g_wait_for_ready()) {
    if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
      return false;
    }
  }

  if (success) {
    uint8_t cmd[] = {command, 0, (pageAddress >> 8) & 0xff, (pageAddress >> 0) & 0xff};
    W25n01g_select();
    if (HAL_SPI_Transmit(&HANDLER_FLASH, cmd, sizeof(cmd), W25NXX_TIMOUT) != HAL_OK) {
      success = false;
    }
    W25n01g_deselect();
  }

  return success;
}

//semble ok
bool W25n01g_program_data_load(uint16_t columnAddress, const uint8_t *data, uint32_t length)
{
  bool success = false;
  success = W25n01g_write_enable();

  if (success) {
    uint32_t tick_start = HAL_GetTick();
    while (W25n01g_wait_for_ready()) {
      if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
        return false;
      }
    }

    uint8_t cmd[] = {W25N01G_INSTR_PROGRAM_DATA_LOAD, columnAddress >> 8, columnAddress & 0xff};

    W25n01g_select();
    if (HAL_SPI_Transmit(&HANDLER_FLASH, cmd, sizeof(cmd), W25NXX_TIMOUT) == HAL_OK) {
      if (HAL_SPI_Transmit(&HANDLER_FLASH, (uint8_t *) data, length, W25NXX_TIMOUT) == HAL_OK) {
        success = true;
      } else {
        success = false;
      }
    } else {
      success = false;
    }
    W25n01g_deselect();

  }

  return success;
}

bool w25n01g_page_program(uint32_t address, const uint8_t *data, uint32_t length)
{
  bool success = true;

  uint16_t columnAddress = W25N01G_LINEAR_TO_COLUMN(address);
  uint16_t pageAddress = W25N01G_LINEAR_TO_PAGE(address);

  success = W25n01g_program_data_load(columnAddress, data, length);

  if (success) {
    success = W25n01g_perform_command_with_page_address(&HANDLER_FLASH, W25N01G_INSTR_PROGRAM_EXECUTE, pageAddress);
  }

  if (success) {
    uint32_t tick_start = HAL_GetTick();
    while (W25n01g_wait_for_ready()) {
      if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
        return false;
      }
    }

    uint8_t quad_bk1_sr;
    W25n01g_read_sr(W25N01G_STAT_REG, &quad_bk1_sr);
    success = ((W25N01G_STATUS_PROGRAM_FAIL & quad_bk1_sr) != W25N01G_STATUS_PROGRAM_FAIL);
  }

  return success;
}

//semble ok...
bool w25n01g_write_flash(uint32_t address, const uint8_t *data, uint32_t length)
{
  bool success = true;

  uint32_t numberOfPages = length / W25N01G_PAGE_SIZE;
  uint32_t pageIndex;
  uint32_t pageAddress;

  for (pageIndex = 0; (pageIndex < numberOfPages) && success; pageIndex++) {
    pageAddress = address + W25N01G_PAGE_SIZE * pageIndex;
    success = w25n01g_page_program(pageAddress, &(data[W25N01G_PAGE_SIZE * pageIndex]), W25N01G_PAGE_SIZE);
  }

  uint32_t notFullPageSize = (length % W25N01G_PAGE_SIZE);

  if (success && (notFullPageSize != 0)) {
    pageAddress = address + W25N01G_PAGE_SIZE * pageIndex;
    success = w25n01g_page_program(pageAddress, &(data[W25N01G_PAGE_SIZE * pageIndex]), notFullPageSize);
  }

  return success;
}

//semble etre ok...
uint32_t w25n01g_read_bytes(uint32_t address, uint8_t *buffer, uint32_t length)
{

  uint32_t targetPage = W25N01G_LINEAR_TO_PAGE(address);

  uint32_t tick_start = HAL_GetTick();
  while (W25n01g_wait_for_ready()) {
    if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
      return 0;
    }
  }

  W25n01g_perform_command_with_page_address(&HANDLER_FLASH, W25N01G_INSTR_PAGE_DATA_READ, targetPage);

  uint16_t column = W25N01G_LINEAR_TO_COLUMN(address);
  uint16_t transferLength;

  if (length > (W25N01G_PAGE_SIZE - column)) {
    transferLength = W25N01G_PAGE_SIZE - column;
  } else {
    transferLength = length;
  }

  tick_start = HAL_GetTick();
  while (W25n01g_wait_for_ready()) {
    if (HAL_GetTick() - tick_start > W25NXX_TIMOUT) {
      return false;
    }
  }

  uint8_t cmd[4];
  cmd[0] = W25N01G_INSTR_READ_DATA;
  cmd[1] = (column >> 8) & 0xff;
  cmd[2] = (column >> 0) & 0xff;
  cmd[3] = 0;

  W25n01g_select();
  if (HAL_SPI_Transmit(&HANDLER_FLASH, cmd, sizeof(cmd), W25NXX_DEFAULT_TIMEOUT) != HAL_OK) {
    return 0;
  }

  if (HAL_SPI_Receive(&HANDLER_FLASH, buffer, length, W25NXX_DEFAULT_TIMEOUT) != HAL_OK) {
    return 0;
  }
  W25n01g_deselect();

  tick_start = HAL_GetTick();
  while (W25n01g_wait_for_ready()) {
    if (HAL_GetTick() - tick_start > 2) {
      return 0;
    }
  }

  // HAL_Delay(2);

  uint8_t quad_bk1_sr;
  W25n01g_read_sr(W25N01G_STAT_REG, &quad_bk1_sr);
  uint8_t eccCode = W25N01G_STATUS_FLAG_ECC(quad_bk1_sr);

  switch (eccCode) {
    case 0: // Successful read, no ECC correction
      break;
    case 1: // Successful read with ECC correction
    case 2: // Uncorrectable ECC in a single page
    case 3: // Uncorrectable ECC in multiple pages
      /*w25n01g_addError(address, eccCode);
      w25n01g_deviceReset(fdevice);*/
      int8_t res =  w25n01g_deviceReset();
      break;
  }


  return transferLength;
}

/**
 * Wrapper des fonctions utiliser par LittleFs
 * Fonction de sync, ecriturre, lecture et effacement.
 *
 */

int lfs_sync(const struct lfs_config *c)
{
  return 0;
}

int lfs_erase(const struct lfs_config *c, lfs_block_t block)
{
  w25n01g_block_erase(block * c->block_size);
  return 0;
}

int lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, const void *buffer, lfs_size_t size)
{
  w25n01g_write_flash((block * c->block_size + offset), (uint8_t *) buffer, size);
  return 0;
}

int lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, void *buffer, lfs_size_t size)
{
  w25n01g_read_bytes((block * c->block_size + offset), (uint8_t *) buffer, size);
  return 0;
}


