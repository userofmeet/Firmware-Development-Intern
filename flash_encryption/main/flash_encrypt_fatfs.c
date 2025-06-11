#include <stdio.h>
#include <string.h>
#include "esp_flash.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"

#include "flash_encrypt_fatfs.h"  

static size_t example_fatfs_partition_test(const esp_partition_t* partition, const size_t text_data_offset);

static const char* TAG = "example_fatfs";

void example_read_write_fatfs(void)
{
    const esp_partition_t* partition = NULL;

    partition = esp_partition_find_first(
                    ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, CUSTOM_FAT_PART_NAME_NE);
    assert(partition);

    size_t open_test_string_offset = example_fatfs_partition_test(partition, SIZE_MAX);
    assert(open_test_string_offset != SIZE_MAX);

    partition = esp_partition_find_first(
                    ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, CUSTOM_FAT_PART_NAME_E);
    assert(partition);

    example_fatfs_partition_test(partition, open_test_string_offset);
}
static size_t example_fatfs_partition_test(const esp_partition_t* partition, const size_t text_data_offset)
{
    const char* TEST_FAT_STRING = "hi, this is mj from marshee";
    const char *base_path = "/spiflash";
    wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
    esp_err_t err = ESP_FAIL;

    ESP_LOGI(TAG, "FAT partition \"%s\" is %sencrypted. Size is (0x%" PRIx32 " bytes) ",
             partition->label,
             (partition->encrypted) ? "" : "not ",
             (long unsigned int)partition->size
            );

    ESP_LOGI(TAG, "Formatting FAT filesystem");
    err = esp_vfs_fat_spiflash_format_rw_wl(base_path, partition->label);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(err));
        return SIZE_MAX;
    }

    ESP_LOGI(TAG, "Mounting FAT filesystem");
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = false,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, partition->label, &mount_config, &s_wl_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return SIZE_MAX;
    }

    const size_t RD_BUFF_LEN = 64;
    char read_buffer[RD_BUFF_LEN];
    char *device_filename;

    device_filename = "/spiflash/inner.txt";

    ESP_LOGI(TAG, "Opening file");
    FILE *f;
    f = fopen(device_filename, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return SIZE_MAX;
    }
    fprintf(f, TEST_FAT_STRING);
    fclose(f);

    ESP_LOGI(TAG, "Written to file: '%s'", TEST_FAT_STRING);

    ESP_LOGI(TAG, "Reading file");
    f = fopen(device_filename, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return SIZE_MAX;
    }
    fgets(read_buffer, sizeof(read_buffer), f);
    fclose(f);
    char *pos = strchr(read_buffer, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", read_buffer);

    ESP_LOGI(TAG, "Unmounting FAT filesystem");
    ESP_ERROR_CHECK(esp_vfs_fat_spiflash_unmount_rw_wl(base_path, s_wl_handle));

    if (text_data_offset == SIZE_MAX) {
        ESP_LOGI(TAG, "Read partition using esp_flash_read until test string is found");

        size_t read_offset = 0;
        size_t read_len = RD_BUFF_LEN;
        void* text_addr = NULL;

        assert(partition->size > RD_BUFF_LEN);
        assert(RD_BUFF_LEN > strlen(TEST_FAT_STRING));
        while (true) {
            ESP_ERROR_CHECK(esp_flash_read(NULL, read_buffer, partition->address + read_offset, read_len));
            text_addr = memmem(read_buffer, read_len, TEST_FAT_STRING, strlen(TEST_FAT_STRING));
            if (text_addr != NULL) {
                ESP_LOG_BUFFER_HEXDUMP(TAG, text_addr, strlen(TEST_FAT_STRING), ESP_LOG_INFO);
                size_t test_str_part_offset = (text_addr - (void*)read_buffer) + read_offset;
                ESP_LOGI(TAG, "Test string was found at offset (0x%" PRIx32 ")", (long unsigned int)test_str_part_offset);
                return test_str_part_offset;
            }
            read_offset += (RD_BUFF_LEN - strlen(TEST_FAT_STRING));
            if ((read_offset + strlen(TEST_FAT_STRING)) > partition->size) {
                break;
            } else {
                read_len = (partition->size >= (read_offset + RD_BUFF_LEN)) ? RD_BUFF_LEN : partition->size - read_offset;
            }
        }
    } else {
        ESP_LOGI(TAG, "Read partition using esp_flash_read at expected offset (0x%" PRIx32 ") ", (long unsigned int)text_data_offset);

        assert(text_data_offset <= partition->size);
        assert((text_data_offset + strlen(TEST_FAT_STRING)) <= partition->size);
        ESP_ERROR_CHECK(esp_flash_read(NULL, read_buffer, partition->address + text_data_offset, strlen(TEST_FAT_STRING)));

        ESP_LOG_BUFFER_HEXDUMP(TAG, read_buffer, strlen(TEST_FAT_STRING), ESP_LOG_INFO);
        if (memcmp(read_buffer, TEST_FAT_STRING, strlen(TEST_FAT_STRING)) == 0) {
            ESP_LOGI(TAG, "Data matches test string");
            return text_data_offset;
        }
        ESP_LOGI(TAG, "Data does not match test string");
    }

    return SIZE_MAX;
}
