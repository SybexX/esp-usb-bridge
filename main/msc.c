/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// The mass storage class creates a mountable USB device into which UF2 formatted files can be dropped to flash the
// target ESP device. Some necessary initialization is done by running the msc_task(). The task is deleted after the
// initialization. The module contains the following callbacks from the tinyusb USB stack.
// - tud_msc_inquiry_cb - returns string identifiers about the device.
// - tud_msc_test_unit_ready_cb - return the availability of the device. It is available in the beginning and while it
//   is mounted. It becomes unavailable after ejecting the device.
// - tud_msc_capacity_cb - returns the device capacity.
// - tud_msc_start_stop_cb - handles disc ejection.
// - tud_msc_scsi_cb - desired actions to SCSI disc commands can be handler there.
// - tud_msc_read10_cb - invoked in order to read from the disc. A skeleton structure of FAT16 file system is
//   pre-defined by variables msc_disk_boot_sector, msc_disk_fat_table_sector0, msc_disk_readme_sector0 and
//   msc_disk_root_directory_sector0. A disc read outside of these variables returns all zeroes.
// - tud_msc_write10_cb - invoked in order to write the disc. The above mentioned file system structure is not modified.
//   Each write is interpreted as a block for flashing. UF2 block format is used where the flashing address is encoded
//   among other information. The flashing is done by the esp-serial-flasher IDF component.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "tusb.h"
#include "msc.h"
#include "util.h"
#include "esp_loader.h"
#include "esp_timer.h"
#include "serial.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_io.h"

#define KB(x) ((x) * 1024)
#define FAT_CLUSTERS                    KB(6)
#define FAT_SECTORS_PER_CLUSTER         8
#define FAT_SECTORS                     (FAT_CLUSTERS * FAT_SECTORS_PER_CLUSTER)
#define FAT_SECTOR_SIZE                 512
#define FAT_ROOT_SECTORS                2
#define FAT_ROOT_ENTRY_SIZE             32
#define FAT_VOLUME_NAME_SIZE            11

// Windows will generate a volume information file on the first mount. And it uses long file names, therefore, will
// use three entries per file. So only three new files could be added if using one 512B partition and 16 root
// entries.

#define FAT_ROOT_ENTRIES                (FAT_ROOT_SECTORS * FAT_SECTOR_SIZE / FAT_ROOT_ENTRY_SIZE)
#define FAT16_CLUSTER_BYTES             2
#define FAT_TABLE_SECTORS               (FAT_CLUSTERS * FAT16_CLUSTER_BYTES / FAT_SECTOR_SIZE)
#define FAT_BOOT_SECTORS                1

typedef struct __attribute__((__packed__))
{
    uint8_t jump_instructions[3];
    uint8_t oem_name[8];
    uint16_t bytes_per_sector;
    // BIOS parameter block (25 bytes)
    uint8_t sectors_per_cluster;
    uint16_t reserved_sectors;
    uint8_t fat_table_copies;
    uint16_t root_entries;
    uint16_t no_small_sectors;
    uint8_t media_type;
    uint16_t fat_table_sectors;
    uint16_t sectors_per_track;
    uint16_t heads;
    uint32_t hidden_sectors;
    uint32_t large_sectors;
    // Extended BIOS parameter block (26 bytes)
    uint8_t physical_disco_no;
    uint8_t current_head;
    uint8_t signature;
    uint32_t serial_no;
    uint8_t volume[FAT_VOLUME_NAME_SIZE];
    uint8_t system_id[8];

    uint8_t bootstrap_code[448];

    uint16_t end_marker;

} msc_boot_sector_t;

static const char *TAG = "bridge_msc";
static bool ejected = false;

_Static_assert(FAT_SECTORS == (FAT_SECTORS & 0xFFFF), "Large sectors should be used instead of small ones");
_Static_assert(FAT_SECTOR_SIZE == (FAT_SECTOR_SIZE & 0xFFFF), "FAT Sector Size must fit into a 16-bit field");
_Static_assert(FAT_SECTORS_PER_CLUSTER == (FAT_SECTORS_PER_CLUSTER & 0xFF), "FAT Sectors per Cluster must fit into a 8-bit field");
_Static_assert(FAT_ROOT_ENTRIES == (FAT_ROOT_ENTRIES & 0xFFFF), "FAT ROOT entries must fit into a 16-bit field");
_Static_assert(FAT_TABLE_SECTORS == (FAT_TABLE_SECTORS & 0xFFFF), "FAT table sectors must fit into a 16-bit field");
_Static_assert(FAT_BOOT_SECTORS == (FAT_BOOT_SECTORS & 0xFFFF), "FAT boot sectors must fit into a 16-bit field");
_Static_assert(sizeof(msc_boot_sector_t) == FAT_SECTOR_SIZE, "The boot sector has incorrect size!");
_Static_assert(strlen(CONFIG_BRIDGE_MSC_VOLUME_LABEL) <= FAT_VOLUME_NAME_SIZE, "BRIDGE_MSC_VOLUME_LABEL is too long");

static msc_boot_sector_t msc_disk_boot_sector = {
    .jump_instructions = {0xEB, 0x3C, 0x90},
    .oem_name = {'m', 'k', 'f', 's', '.', 'f', 'a', 't'},
    .bytes_per_sector = FAT_SECTOR_SIZE,
    .sectors_per_cluster = FAT_SECTORS_PER_CLUSTER,
    .reserved_sectors = FAT_BOOT_SECTORS,
    .fat_table_copies = 1,
    .root_entries = FAT_ROOT_ENTRIES,
    .no_small_sectors = FAT_SECTORS, // Small sectors if the number fits here
    .media_type = 0xF8, // hard disk
    .fat_table_sectors = FAT_TABLE_SECTORS,
    .sectors_per_track = 0,
    .heads = 0,
    .hidden_sectors = 0,
    .large_sectors = 0,
    .physical_disco_no = 0x80, // Hard drives are numbered from 0x80.
    .current_head = 0, // not used by FAT
    .signature = 0x29, // Must be 0x28 or 0x29 for Windows.
    .serial_no = 0x563d0c93,  // Random number created upon formatting.
    .volume = {' '},
    .system_id = {'F', 'A', 'T', '1', '6', ' ', ' ', ' '},

    // The bootstrap code was generated with mkfs.fat and it prints "This is not a bootable disk. Please insert a
    // bootable floppy and press any key to try again".
    .bootstrap_code = {
        0x0e, 0x1f, 0xbe, 0x5b, 0x7c, 0xac, 0x22, 0xc0, 0x74, 0x0b, 0x56, 0xb4, 0x0e, 0xbb, 0x07, 0x00, 0xcd, 0x10, 0x5e,
        0xeb, 0xf0, 0x32, 0xe4, 0xcd, 0x16, 0xcd, 0x19, 0xeb, 0xfe, 0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, 0x6e,
        0x6f, 0x74, 0x20, 0x61, 0x20, 0x62, 0x6f, 0x6f, 0x74, 0x61, 0x62, 0x6c, 0x65, 0x20, 0x64, 0x69, 0x73, 0x6b, 0x2e,
        0x20, 0x20, 0x50, 0x6c, 0x65, 0x61, 0x73, 0x65, 0x20, 0x69, 0x6e, 0x73, 0x65, 0x72, 0x74, 0x20, 0x61, 0x20, 0x62,
        0x6f, 0x6f, 0x74, 0x61, 0x62, 0x6c, 0x65, 0x20, 0x66, 0x6c, 0x6f, 0x70, 0x70, 0x79, 0x20, 0x61, 0x6e, 0x64, 0x0d,
        0x0a, 0x70, 0x72, 0x65, 0x73, 0x73, 0x20, 0x61, 0x6e, 0x79, 0x20, 0x6b, 0x65, 0x79, 0x20, 0x74, 0x6f, 0x20, 0x74,
        0x72, 0x79, 0x20, 0x61, 0x67, 0x61, 0x69, 0x6e, 0x20, 0x2e, 0x2e, 0x2e, 0x20, 0x0d, 0x0a, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0
    },

    .end_marker = 0xAA55,
};

static const uint8_t msc_disk_fat_table_sector0[] = {
    0xF8, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF, // Cluster no. 2 - Readme file start and end
};

static const uint8_t msc_disk_readme_sector0[] =
    "Use 'idf.py uf2' to generate an UF2 binary. Drop the generated file into this disk in order to flash the device. \
\r\n";

#define MSC_README_SIZE     (sizeof(msc_disk_readme_sector0) - 1)
_Static_assert(MSC_README_SIZE < FAT_SECTOR_SIZE, "Only the first sector of the README is stored in RAM");

static uint8_t msc_disk_root_directory_sector0[] = {
    ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
    0x08, // attribute byte where volume bit is set
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // time and date for creation & modification
    0, 0, // starting cluster in the FAT table
    0, 0, 0, 0, // size
    // readme file
    'R', 'E', 'A', 'D', 'M', 'E', ' ', ' ', 'T', 'X', 'T',
    0x01, // attribute byte where read-only bit is set
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // time and date for creation & modification
    0x02, 0, // starting cluster in the FAT table
    GET_BYTE(MSC_README_SIZE, 0), GET_BYTE(MSC_README_SIZE, 1), GET_BYTE(MSC_README_SIZE, 2), GET_BYTE(MSC_README_SIZE, 3), // size
};

void tud_msc_inquiry_cb(const uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;

    const char vid[8] = "ESP";
    const char pid[16] = "Flash Storage";
    const char rev[4] = "0.1";

    ESP_LOGD(TAG, "tud_msc_inquiry_cb() invoked");

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

bool tud_msc_test_unit_ready_cb(const uint8_t lun)
{
    ESP_LOGD(TAG, "tud_msc_test_unit_ready_cb() invoked");

    if (ejected) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
        return false;
    }

    return true;
}

void tud_msc_capacity_cb(const uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void) lun;

    ESP_LOGD(TAG, "tud_msc_capacity_cb() invoked");
    *block_count = FAT_SECTORS;
    *block_size  = FAT_SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(const uint8_t lun, const uint8_t power_condition, const bool start, const bool load_eject)
{
    (void) lun;

    ESP_LOGI(TAG, "tud_msc_start_stop_cb() invoked, power_condition=%d, start=%d, load_eject=%d", power_condition,
             start, load_eject);

    if (load_eject) {
        if (start) {
            // load disk storage
        } else {
            // unload disk storage
            ejected = true;
        }
    }

    return true;
}

#define FIRST_FAT_SECTOR      FAT_BOOT_SECTORS
#define FIRST_ROOT_SECTOR     (FIRST_FAT_SECTOR + FAT_TABLE_SECTORS)
#define FIRST_README_SECTOR   (FIRST_ROOT_SECTOR + FAT_ROOT_SECTORS)
#define FIRST_ELSE_SECTOR     (FIRST_README_SECTOR + FAT_SECTORS_PER_CLUSTER)
#define IS_LBA_BOOT(lba)      ((lba) < FIRST_FAT_SECTOR)
#define IS_LBA_FAT(lba)       ((lba) >= FIRST_FAT_SECTOR && (lba) < FIRST_ROOT_SECTOR)
#define IS_LBA_ROOT(lba)      ((lba) >= FIRST_ROOT_SECTOR && (lba) < FIRST_README_SECTOR)
#define IS_LBA_README(lba)    ((lba) >= FIRST_README_SECTOR && (lba) < FIRST_ELSE_SECTOR)
#define IS_LBA_ELSE(lba)      ((lba) >= FIRST_ELSE_SECTOR)

int32_t tud_msc_read10_cb(const uint8_t lun, const uint32_t lba, const uint32_t offset, void *buffer, const uint32_t bufsize)
{
    ESP_LOGD(TAG, "tud_msc_read10_cb() invoked, lun=%d, lba=%" PRId32 ", offset=%" PRId32 ", bufsize=%" PRId32, lun, lba, offset, bufsize);

    const uint8_t *addr = NULL;
    size_t size = FAT_SECTOR_SIZE;

    if (IS_LBA_BOOT(lba)) {
        addr = (const uint8_t *) &msc_disk_boot_sector;
    } else if (lba == FIRST_FAT_SECTOR) {
        addr = msc_disk_fat_table_sector0;
        size = sizeof(msc_disk_fat_table_sector0);
    } else if (lba == FIRST_ROOT_SECTOR) {
        addr = msc_disk_root_directory_sector0;
        size = sizeof(msc_disk_root_directory_sector0);
    } else if (lba == FIRST_README_SECTOR) {
        addr = msc_disk_readme_sector0;
        size = sizeof(msc_disk_readme_sector0);
    } // else lba sector is not kept in RAM

    int done = 0;
    int left_to_do = bufsize;

    if (addr) {
        const int available = size - offset;
        if (available > 0) {
            const int len = MIN(available, left_to_do);
            memcpy(buffer, addr + offset, len);
            done = len;
            left_to_do -= len;
        }
    }

    if (left_to_do > 0) {
        memset((uint8_t *)buffer + done, 0, left_to_do);
    }

    return bufsize;
}

#define UF2_BLOCK_SIZE                  512
#define UF2_DATA_SIZE                   476
#define UF2_MD5_SIZE                    24
#define UF2_FIRST_MAGIC                 0x0A324655
#define UF2_SECOND_MAGIC                0x9E5D5157
#define UF2_FINAL_MAGIC                 0x0AB16F30
#define UF2_FLAG_FAMILYID_PRESENT       0x00002000
#define UF2_FLAG_MD5_PRESENT            0x00004000

typedef struct {
    uint32_t magic0;
    uint32_t magic1;
    uint32_t flags;
    uint32_t addr;
    uint32_t payload_size;
    uint32_t block_no;
    uint32_t blocks;
    uint32_t chip_id;
    uint8_t data[UF2_DATA_SIZE];
    uint32_t magic3;
} uf2_block_t;

#define UF2_ESP8266_ID      0x7eab61ed

static const char *chipid_to_name(const uint32_t id)
{
    // IDs can be found at https://github.com/Microsoft/uf2
    switch (id) {
    case UF2_ESP8266_ID:
        return "ESP8266";
    case 0x1c5f21b0:
        return "ESP32";
    case 0xbfdd4eee:
        return "ESP32-S2";
    case 0xd42ba06c:
        return "ESP32-C3";
    case 0xc47e5767:
        return "ESP32-S3";
    case 0x2b88d29c:
        return "ESP32-C2";
    case 0x332726f6:
        return "ESP32-H2";
    case 0x540ddf62:
        return "ESP32-C6";
    case 0x3d308e94:
        return "ESP32-P4";
    case 0xf71c0343:
        return "ESP32-C5";
    case 0x77d850c4:
        return "ESP32-C61";
    case 0xb6dd00af:
        return "ESP32-H21";
    case 0x9e0baa8a:
        return "ESP32-H4";
    default:
        return "unknown";
    }
}

#define BUFFER_SIZE                         KB(16)
#define FLASH_BLOCK_SIZE                    KB(4)

#define MSC_FLASH_HIGH_BAUDRATE             230400
#define MSC_FLASH_DEFAULT_BAUDRATE          115200

static int msc_last_block_written = -1;
static esp_timer_handle_t reset_timer;

static bool msc_change_baudrate(const uint32_t chip_id, const uint32_t baud)
{
    if (chip_id == UF2_ESP8266_ID) {
        return true;
    }
    return (esp_loader_change_baudrate(baud) == ESP_LOADER_SUCCESS) && serial_set_baudrate(baud);
}

static esp_loader_error_t flash_data(uint32_t addr, uint8_t *data, uint32_t datalen)
{
    const uint32_t block_size = KB(4); // Seems like max for ROM flash
    if (esp_loader_flash_start(addr, datalen, block_size) != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "Erasing flash failed at addr %" PRId32 " of length %" PRId32, addr, datalen);
        return ESP_LOADER_ERROR_FAIL;
    }

    while (datalen > 0) {
        uint32_t bytes_to_write = MIN(datalen, block_size);
        ESP_LOGD(TAG, "Writing flash at addr %" PRId32 " of length %" PRId32, addr, bytes_to_write);
        if (esp_loader_flash_write(data, bytes_to_write) != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Writing flash failed at addr %" PRId32 " of length %" PRId32, addr, datalen);
            return ESP_LOADER_ERROR_FAIL;
        }
        datalen -= bytes_to_write;
        data += bytes_to_write;
        addr += bytes_to_write;
    }
    return ESP_LOADER_SUCCESS;
}

static bool handle_data(uint32_t addr, uint8_t *data, uint32_t datalen, bool flush_buffer)
{
    static uint8_t buffer[BUFFER_SIZE];
    static uint32_t last_written_pos = 0;
    static uint32_t aligned_addr = 0;

    if (flush_buffer) {
        // If we have data in buffer, pad it to block boundary and write it
        if (last_written_pos > 0) {
            uint32_t end_addr = aligned_addr + last_written_pos;
            uint32_t pad_size = (FLASH_BLOCK_SIZE - (end_addr % FLASH_BLOCK_SIZE)) % FLASH_BLOCK_SIZE;

            // Read existing data for padding if needed
            if (pad_size > 0) {
                if (esp_loader_flash_read(buffer + last_written_pos, end_addr, pad_size) != ESP_LOADER_SUCCESS) {
                    return false;
                }
                last_written_pos += pad_size;
            }

            // Write the complete block
            if (flash_data(aligned_addr, buffer, last_written_pos) != ESP_LOADER_SUCCESS) {
                return false;
            }
            last_written_pos = 0;
        }
        return true;
    }

    // If buffer is empty, check if the address is aligned to the block size,
    // if not, pad the buffer with the existing data from the flash.
    if (last_written_pos == 0) {
        aligned_addr = addr & ~(FLASH_BLOCK_SIZE - 1);
        if (addr != aligned_addr) {
            uint32_t bytes_to_read = addr - aligned_addr;
            if (esp_loader_flash_read(buffer, aligned_addr, bytes_to_read) != ESP_LOADER_SUCCESS) {
                return false;
            }
            last_written_pos = bytes_to_read;
        }
    }

    // Handle buffer overflow by writing current buffer and continuing with remaining data
    uint32_t remaining_space = BUFFER_SIZE - last_written_pos;
    if (datalen > remaining_space) {
        uint32_t bytes_to_copy = remaining_space;
        memcpy(buffer + last_written_pos, data, bytes_to_copy);

        if (flash_data(aligned_addr, buffer, BUFFER_SIZE) != ESP_LOADER_SUCCESS) {
            return false;
        }

        // Update the remaining data to be written
        datalen -= bytes_to_copy;
        data += bytes_to_copy;
        addr += bytes_to_copy;
        last_written_pos = 0;

        // Recursively handle remaining data
        return handle_data(addr, data, datalen, false);
    }

    // Add new data to buffer
    memcpy(buffer + last_written_pos, data, datalen);
    last_written_pos += datalen;

    return true;
}

static bool init_flash(uint32_t chip_id)
{
    serial_set(false);

    if (!serial_set_baudrate(MSC_FLASH_DEFAULT_BAUDRATE)) {
        ESP_LOGW(TAG, "BRIDGE UART failed to change baudrate to %d", MSC_FLASH_DEFAULT_BAUDRATE);
        return false;
    }

    esp_loader_connect_args_t connect_config = ESP_LOADER_CONNECT_DEFAULT();
    if (esp_loader_connect(&connect_config) != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "ESP LOADER connection failed!");
        return false;
    }
    ESP_LOGD(TAG, "ESP LOADER connection success!");

    if (!msc_change_baudrate(chip_id, MSC_FLASH_HIGH_BAUDRATE)) {
        ESP_LOGE(TAG, "Failed to change baudrate to %d", MSC_FLASH_HIGH_BAUDRATE);
        return false;
    }
    ESP_LOGD(TAG, "Baudrate changed to %d", MSC_FLASH_HIGH_BAUDRATE);

    return true;
}

static bool finish_flash(void)
{
    // Flush any remaining data in the buffer
    if (!handle_data(0, NULL, 0, true)) {
        ESP_LOGE(TAG, "Failed to flush buffer");
        return false;
    }

    if (!serial_set_baudrate(MSC_FLASH_DEFAULT_BAUDRATE)) {
        ESP_LOGW(TAG, "ESP LOADER cannot change baudrate to %d", MSC_FLASH_DEFAULT_BAUDRATE);
    }

    esp_loader_flash_finish(false);
    serial_set(true);
    gpio_set_level(GPIO_RST, false);
    ESP_ERROR_CHECK(esp_timer_start_once(reset_timer, SERIAL_FLASHER_RESET_HOLD_TIME_MS * 1000));
    return true;
}

int32_t tud_msc_write10_cb(const uint8_t lun, const uint32_t lba, const uint32_t offset, uint8_t *buffer, const uint32_t bufsize)
{
    ESP_LOGD(TAG, "tud_msc_write10_cb() invoked, lun=%d, lba=%" PRId32 ", offset=%" PRId32, lun, lba, offset);
    ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, bufsize, ESP_LOG_DEBUG);

    assert(bufsize == UF2_BLOCK_SIZE);

    if (esp_timer_is_active(reset_timer)) {
        ESP_LOGD(TAG, "Waiting for the target to reset");
        return 0;
    }

    // Linux and Windows write files differently. Windows also creates system volume information files on the first
    // mount. In an ideal case, FAT and ROOT content would be analyzed and the flash file detected.
    // However, the only reliable way to detect files for flashing is look at the content.
    if (IS_LBA_ELSE(lba)) {
        uf2_block_t *p = (uf2_block_t *) buffer;

        if (p->magic0 == UF2_FIRST_MAGIC && p->magic1 == UF2_SECOND_MAGIC && p->magic3 == UF2_FINAL_MAGIC) {
            // UF2 block detected
            const char *chip_name = (p->flags & UF2_FLAG_FAMILYID_PRESENT) ? chipid_to_name(p->chip_id) : "???";

            ESP_LOGD(TAG, "LBA %" PRId32 ": UF2 block %" PRId32 " of %" PRId32 " for chip %s at %#08" PRIx32 " with length %" PRId32, lba, p->block_no, p->blocks,
                     chip_name, p->addr, p->payload_size);

            // Not connected to chip, initialize
            if (msc_last_block_written == -1) {
                if (!init_flash(p->chip_id)) {
                    ESP_LOGE(TAG, "Failed to initialize flash");
                    eub_abort();
                }
            }
            // The block number is not sequential, flush the buffer
            else if (p->block_no - 1 != msc_last_block_written) {
                if (!handle_data(0, NULL, 0, true)) {
                    ESP_LOGE(TAG, "Failed to flush buffer");
                    eub_abort();
                }
            }

            const uint32_t real_payload_size = (p->flags & UF2_FLAG_MD5_PRESENT) ? (UF2_DATA_SIZE - UF2_MD5_SIZE) : UF2_DATA_SIZE;
            if (!handle_data(p->addr, p->data, real_payload_size, false)) {
                ESP_LOGE(TAG, "Failed to handle data");
                eub_abort();
            }
            msc_last_block_written = p->block_no;
        }
    }
    return bufsize;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
void tud_msc_write10_complete_cb(uint8_t lun)
{
    if (msc_last_block_written != -1) {
        if (!finish_flash()) {
            ESP_LOGE(TAG, "Failed to finish flash");
            eub_abort();
        }
        msc_last_block_written = -1;
    }
    ESP_LOGD(TAG, "tud_msc_write10_complete_cb() invoked, lun=%" PRIu8, lun);
}

int32_t tud_msc_scsi_cb(const uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, const uint16_t bufsize)
{
    (void) buffer;
    int32_t ret;

    ESP_LOGD(TAG, "tud_msc_scsi_cb() invoked. bufsize=%d", bufsize);
    ESP_LOG_BUFFER_HEXDUMP("scsi_cmd", scsi_cmd, 16, ESP_LOG_DEBUG);

    switch (scsi_cmd[0]) {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        ESP_LOGD(TAG, "tud_msc_scsi_cb() invoked: SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL");
        ret = 0;
        break;

    default:
        ESP_LOGW(TAG, "tud_msc_scsi_cb() invoked: %d", scsi_cmd[0]);

        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

        ret = -1;
        break;
    }

    return ret;
}

static void reset_timer_cb(void *arg)
{
    (void) arg;
    gpio_set_level(GPIO_RST, true);
    serial_set(true);
}

static void init_reset_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = reset_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "msc_reset",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &reset_timer));
}

void msc_init(void)
{
    char volume_label[FAT_VOLUME_NAME_SIZE + 1] = CONFIG_BRIDGE_MSC_VOLUME_LABEL; // +1 because the config value is 0-terminated
    // fill the volume_label with spaces up to length FAT_VOLUME_NAME_SIZE
    memset(volume_label + strlen(CONFIG_BRIDGE_MSC_VOLUME_LABEL), ' ',
           FAT_VOLUME_NAME_SIZE - strlen(CONFIG_BRIDGE_MSC_VOLUME_LABEL));
    memcpy(msc_disk_boot_sector.volume, volume_label, FAT_VOLUME_NAME_SIZE);
    memcpy(msc_disk_root_directory_sector0, volume_label, FAT_VOLUME_NAME_SIZE);

    ESP_LOG_BUFFER_HEXDUMP("boot", &msc_disk_boot_sector, sizeof(msc_boot_sector_t), ESP_LOG_DEBUG);
    ESP_LOG_BUFFER_HEXDUMP("fat", msc_disk_fat_table_sector0, sizeof(msc_disk_fat_table_sector0), ESP_LOG_DEBUG);
    ESP_LOG_BUFFER_HEXDUMP("root", msc_disk_root_directory_sector0, sizeof(msc_disk_root_directory_sector0), ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "MSC disk RAM usage: %d bytes", sizeof(msc_boot_sector_t) + sizeof(msc_disk_fat_table_sector0) +
             sizeof(msc_disk_root_directory_sector0) + sizeof(msc_disk_readme_sector0));

    init_reset_timer();
}
