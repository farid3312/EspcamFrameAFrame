// =====================================================
//      ESP32-CAM (AI Thinker) → Cliente de Video OPTIMIZADO
//      Solución Error -1 (Connection Refused) + Keep Alive
// =====================================================

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ----------- CONFIGURACIÓN DE RED -------------
const char* ssid     = "MiRed1234578";
const char* password = "Tucontraseña123";

// Dirección de tu servidor (Asegúrate que sea la IP actual de tu PC)
const char* serverURL = "http://TuIp123:TuPuerto123/video";

// ----------- PINES ESP32-CAM AI THINKER -----------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_BUILTIN_GPIO   4 

// ----------- BUFFERS Y ESTADO -----------
// Buffer global protegido por semáforo
static uint8_t * global_jpg_buf = NULL;
static size_t global_jpg_len = 0;
bool frame_ready_for_send = false; 

SemaphoreHandle_t buf_mutex = NULL;

// ----------- FPS y DIAGNÓSTICO -----------
unsigned long lastFpsTime = 0;
int framesSent = 0;

// =====================================================
//              CONFIGURACIÓN DE LA CÁMARA
// =====================================================
void configureCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Configuración optimizada para streaming fluido
    if (psramFound()) {
        config.frame_size = FRAMESIZE_VGA; // 320x240 (Equilibrio velocidad/calidad)
        config.jpeg_quality = 10;           // Menor numero = Mayor calidad (10-15 es bueno)
        config.fb_count = 2;
        Serial.println("✓ PSRAM Detectada: Calidad Alta");
    } else {
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        Serial.println("⚠ Sin PSRAM: Calidad Baja");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Error al iniciar cámara: 0x%x\n", err);
        delay(5000);
        ESP.restart();
    }
}

// =====================================================
//              CONEXIÓN WIFI ROBUSTA
// =====================================================
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("📡 Conectando a WiFi");
    
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        intentos++;
        if (intentos > 40) { // 20 segundos timeout
            Serial.println("\n❌ No se pudo conectar al WiFi. Reiniciando...");
            ESP.restart();
        }
    }
    Serial.println("\n✓ WiFi Conectado!");
    Serial.print("IP del ESP32: ");
    Serial.println(WiFi.localIP());
}

// =====================================================
//      TAREA 1: CAPTURA (Cámara)
// =====================================================
void cameraTask(void *param) {
    while (true) {
        // Solo capturamos si el buffer anterior ya fue enviado o está libre
        // para evitar saturar la memoria y el mutex
        if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(200))) {
            bool ready = frame_ready_for_send;
            xSemaphoreGive(buf_mutex);
            
            if (ready) {
                // Si el frame anterior aun no se envía, esperamos un poco
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("❌ Fallo captura de cámara");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Copiamos al buffer global
        if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(200))) {
            if (global_jpg_buf) {
                free(global_jpg_buf); // Liberar memoria anterior
                global_jpg_buf = NULL;
            }
            
            // Asignar nueva memoria
            global_jpg_buf = (uint8_t*) malloc(fb->len);
            if (global_jpg_buf) {
                memcpy(global_jpg_buf, fb->buf, fb->len);
                global_jpg_len = fb->len;
                frame_ready_for_send = true; // Avisar al enviador
            } else {
                Serial.println("❌ Error malloc (Memoria llena)");
            }
            xSemaphoreGive(buf_mutex);
        }

        esp_camera_fb_return(fb); // Devolver frame al driver
        
        // Pequeña pausa para no sobrecalentar
        vTaskDelay(pdMS_TO_TICKS(9)); 
    }
}

// =====================================================
//      TAREA 2: ENVÍO HTTP (WiFi)
// =====================================================
void senderTask(void *param) {
    // Usamos WiFiClient para configurar timeouts
    WiFiClient client;
    
    while (true) {
        // Verificar conexión WiFi
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("⚠ WiFi perdido, reconectando...");
            WiFi.disconnect();
            WiFi.reconnect();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint8_t *send_buf = NULL;
        size_t send_len = 0;

        // 1. Obtener frame del buffer global
        if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(100))) {
            if (frame_ready_for_send && global_jpg_buf) {
                // Hacemos una copia local para liberar el mutex rápido
                // OJO: Si hay poca RAM, podríamos enviar directamente bloqueando el mutex,
                // pero copiar es más seguro para la concurrencia.
                send_buf = (uint8_t*) malloc(global_jpg_len);
                if (send_buf) {
                    memcpy(send_buf, global_jpg_buf, global_jpg_len);
                    send_len = global_jpg_len;
                    
                    // Ya tomamos el dato, marcamos como "no listo" para que la cámara tome otro
                    frame_ready_for_send = false; 
                }
            }
            xSemaphoreGive(buf_mutex);
        }

        // 2. Si no hay datos, esperar
        if (!send_buf) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 3. ENVIAR HTTP POST
        HTTPClient http;
        
        // Configurar timeout (importante para evitar bloqueos)
        client.setTimeout(2000); // 2 segundos max
        
        if (http.begin(client, serverURL)) {
            http.addHeader("Content-Type", "image/jpeg");
            
            // Habilitar Keep-Alive para reusar conexión TCP
            http.setReuse(true); 

            int httpCode = http.POST(send_buf, send_len);

            if (httpCode > 0) {
                // Éxito (200)
                if (httpCode == HTTP_CODE_OK) {
                    framesSent++;
                    String payload = http.getString(); // Leer respuesta (opcional)
                } else {
                    Serial.printf("⚠ HTTP Code: %d\n", httpCode);
                }
            } else {
                Serial.printf("❌ Error HTTP POST: %s (%d)\n", http.errorToString(httpCode).c_str(), httpCode);
                // Si el error es -1, es conexión rechazada. Revisa IP y Firewall.
            }
            
            http.end();
        } else {
            Serial.println("❌ No se pudo conectar al servidor");
        }

        free(send_buf); // Liberar memoria local

        // 4. Cálculo de FPS cada 5 segundos
        unsigned long now = millis();
        if (now - lastFpsTime > 5000) {
            float fps = (float)framesSent / 5.0;
            Serial.printf("📊 Estado: %d FPS | Heap Libre: %d bytes\n", (int)fps, ESP.getFreeHeap());
            framesSent = 0;
            lastFpsTime = now;
        }
    }
}

// =====================================================
//                      SETUP
// =====================================================
void setup() {
    Serial.begin(115200);
    
    // Desactivar detector de caídas de voltaje (Brownout)
    #include "soc/soc.h"
    #include "soc/rtc_cntl_reg.h"
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.println("\n\n=== INICIANDO ESP32-CAM ===");

    configureCamera();
    connectWiFi();

    // Crear mutex para proteger la memoria compartida
    buf_mutex = xSemaphoreCreateMutex();

    // Crear tareas
    // Prioridad Sender un poco más alta para evitar lag de red
    xTaskCreatePinnedToCore(cameraTask, "camera_task", 4096, NULL, 1, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(senderTask, "sender_task", 8192, NULL, 2, NULL, 0); // Core 0 (WiFi corre aquí)
    
    Serial.println("=== SISTEMA LISTO ===");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // El loop principal no hace nada
}
