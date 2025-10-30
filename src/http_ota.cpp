#include "http_ota.hpp"
#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_http_server.h"

static const char* TAG = "HTTP_OTA";
static httpd_handle_t s_server = nullptr;

// Página simple de subida
static const char kIndexHtml[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'><title>OTA</title></head>"
"<body><h3>Actualizar firmware</h3>"
"<form method='POST' action='/update' enctype='multipart/form-data'>"
"<input type='file' name='firmware' accept='.bin' required>"
"<button type='submit'>Subir</button></form>"
"<hr><form action='/reboot' method='POST'><button>Reiniciar</button></form>"
"</body></html>";

static esp_err_t root_get_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t reboot_post_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Reiniciando...");
    esp_restart();
    return ESP_OK;
}

// Lee multipart “a mano” y extrae el binario del campo 'firmware'
static esp_err_t update_post_handler(httpd_req_t* req) {
    esp_err_t err;
    const esp_partition_t* update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No hay particion OTA"); return ESP_FAIL; }

    esp_ota_handle_t ota = 0;
    if ((err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &ota)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin fallo");
        return err;
    }

    // Buffer de recepción
    static const int BUF_SZ = 4096;
    uint8_t* buf = (uint8_t*)malloc(BUF_SZ);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "sin RAM"); return ESP_ERR_NO_MEM; }

    // Multipart: buscar inicio de datos del archivo y luego escribir todo a OTA
    bool in_file = false;
    size_t total_written = 0;
    int remaining = req->content_len;

    while (remaining > 0) {
        int to_read = remaining > BUF_SZ ? BUF_SZ : remaining;
        int r = httpd_req_recv(req, (char*)buf, to_read);
        if (r <= 0) {
            free(buf);
            esp_ota_end(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv error");
            return ESP_FAIL;
        }
        remaining -= r;

        // Si aún no ubicamos el comienzo del bin, localizar "\r\n\r\n" de multipart
        int offset = 0;
        if (!in_file) {
            // Buscar fin de cabeceras de la parte: \r\n\r\n
            for (int i = 0; i + 3 < r; ++i) {
                if (buf[i]=='\r' && buf[i+1]=='\n' && buf[i+2]=='\r' && buf[i+3]=='\n') {
                    offset = i + 4;
                    in_file = true;
                    break;
                }
            }
            // si no apareció, seguimos leyendo sin escribir
            if (!in_file) continue;
        }

        // Dentro del archivo: el límite final es "\r\n------" del boundary.
        // Escribir todo el chunk y, si el próximo contiene el boundary, recortar.
        // Heurística simple: si encontramos "\r\n--" al final, puede ser cierre.
        int write_len = r - offset;

        // Intento de detectar boundary en ESTE buffer:
        // busca "\r\n--" desde el final; si aparece, recorta hasta antes.
        for (int i = r - 6; i >= offset && i >= 0; --i) {
            if (buf[i]=='\r' && buf[i+1]=='\n' && buf[i+2]=='-' && buf[i+3]=='-') {
                write_len = i - offset;
                remaining = 0; // forzar salida
                break;
            }
        }

        if (write_len > 0) {
            if ((err = esp_ota_write(ota, buf + offset, write_len)) != ESP_OK) {
                free(buf);
                esp_ota_end(ota);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_write fallo");
                return err;
            }
            total_written += write_len;
        }
    }

    free(buf);

    if ((err = esp_ota_end(ota)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_end fallo");
        return err;
    }

    if ((err = esp_ota_set_boot_partition(update_part)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set_boot_partition fallo");
        return err;
    }

    ESP_LOGW(TAG, "OTA OK, escrito=%u bytes, proxima particion: %s", (unsigned)total_written, update_part->label);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<html><body><h3>Actualizacion exitosa</h3>"
        "<p>Tamano escrito OK. Reinicia para aplicar.</p>"
        "<form action='/reboot' method='POST'><button>Reiniciar ahora</button></form>"
        "</body></html>");
    return ESP_OK;
}

esp_err_t http_ota_start(uint16_t port) {
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = port;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 6144;       // subir si tu html crece
    cfg.recv_wait_timeout = 30;
    cfg.send_wait_timeout = 30;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;

    httpd_uri_t root = { .uri="/", .method=HTTP_GET,  .handler=root_get_handler, .user_ctx=NULL };
    httpd_uri_t updt = { .uri="/update", .method=HTTP_POST, .handler=update_post_handler, .user_ctx=NULL };
    httpd_uri_t reb  = { .uri="/reboot", .method=HTTP_POST, .handler=reboot_post_handler, .user_ctx=NULL };

    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &updt);
    httpd_register_uri_handler(s_server, &reb);

    ESP_LOGI(TAG, "HTTP OTA en puerto %u", (unsigned)port);
    return ESP_OK;
}

void http_ota_stop() {
    if (s_server) {
        httpd_stop(s_server);
        s_server = nullptr;
    }
}
