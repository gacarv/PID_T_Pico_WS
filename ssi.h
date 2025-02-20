#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"

// Referência à variável definida em outro lugar
extern double setpoint; 
extern double current_temperature;

const char *ssi_tags[] = {"setpoint", "current_temperature"};

u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
  size_t printed;
  switch (iIndex) {
  case 0:
    {
      printed = snprintf(pcInsert, iInsertLen, "%.2f", setpoint);
    }
    break;
  case 1:
    {
    printed = snprintf(pcInsert, iInsertLen, "%.2f", current_temperature);
    }
    break;
  default:
    printed = 0;
    break;
  }

  return (u16_t)printed;
}

void ssi_init() {
  // Inicializa o manipulador SSI
  http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
}
