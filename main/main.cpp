/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <string>
#include <atomic>

using namespace std;

static constexpr int RX_BUF_SIZE = 1024;

#if CONFIG_IDF_TARGET_ESP32S3
/* IOMUX pins */
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#elif CONFIG_IDF_TARGET_ESP32
/* no UARTs available with IOMUX on esp32; uart 1 IOMUX pins are occupied by flash on WROOM/WROVER and uart 2 IOMUX pins are occupied by PSRAM on WROVER */
#define TXD_PIN (GPIO_NUM_32)
#define RXD_PIN (GPIO_NUM_34)
#endif

static atomic<int> sentBytes = 0;
static atomic<int> receivedBytes = 0;
static atomic<int> receivedCorrectSequences = 0;
static constexpr std::string_view abab = "abababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababab";
static constexpr std::string_view baba = "babababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababababa";

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 20000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, 40*1024, 40*1024, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1)
    {
        sentBytes += sendData(TX_TASK_TAG, abab.data());
    }
}

int substring_count(const std::string_view &haystack, const std::string_view &needle)
{
    std::string::size_type pos = 0;
    auto sz = needle.size();
    int n = 0;
    while ((pos = haystack.find(needle, pos)) != std::string::npos)
    {
        n++;
        pos += sz;
    }
    return n;
}


static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1)
    {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length));
        length = min(length, RX_BUF_SIZE);
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, length, 100 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            receivedBytes += rxBytes;
            data[rxBytes] = 0;

            std::string_view datastr(reinterpret_cast<const char *>(data));

            /* naive comparison -- we may have started receiving at 'b' instead of 'a' */
            if (!memcmp(datastr.data(), abab.data(), rxBytes))
            {
                receivedCorrectSequences += rxBytes;
            }
            else if (!memcmp(data, baba.data(), rxBytes))
            {
                receivedCorrectSequences += rxBytes;
            }
            else
            {
                int found = 0;
                /* count the total number of times the correct sequence appears in the string */
                if (data[0] == 'a')
                    found = substring_count(datastr, "ab");
                else if (data[0] == 'b')
                    found = substring_count(datastr, "ba");

                receivedCorrectSequences += found * 2;
            }
        }
    }
    free(data);
}

static void speed_test_task(void *arg)
{
    static const char *TAG = "speedy";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int rx, tx, correct;

        /* std::atomic operations to set the value to 0 and return what was there before, in order to avoid locks / data races */
        rx = receivedBytes.exchange(0);
        tx = sentBytes.exchange(0);
        correct = receivedCorrectSequences.exchange(0);

        ESP_LOGI(TAG, "%d tx, %d rx, %d good (%f)", tx, rx, correct, (float)correct / rx);
    }
}

extern "C" void app_main(void)
{
    init();
    xTaskCreatePinnedToCore(speed_test_task, "speed_test_task", 1024*4, NULL, tskIDLE_PRIORITY, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(rx_task, "uart_rx_task", 1024*4, NULL, tskIDLE_PRIORITY, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(tx_task, "uart_tx_task", 1024*4, NULL, tskIDLE_PRIORITY, NULL, tskNO_AFFINITY);
}
