#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"

// Referência à variável definida em outro lugar
extern double setpoint;
extern double error;

// CGI handler para atualizar o setpoint
const char * cgi_setpoint_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
    // Verifica se o parâmetro "setpoint" foi passado na URL (ex: /set_setpoint.cgi?setpoint=100)
    if (strcmp(pcParam[0], "setpoint") == 0) {
        // Converte o valor de setpoint recebido para float
        setpoint = atof(pcValue[0]);

        // Exibe o novo setpoint no log
        printf("\n-------------------------------------------------------");
        printf("\nNew setpoint: %.2f °C", setpoint);
        printf("\n-------------------------------------------------------\n");
        error = 1000.0; // reset error for new read
    }

    // Retorna o caminho para a página principal ou página de confirmação
    return "/index.shtml";  // Redireciona para a página principal
}


// Estrutura tCGI que mapeia as requisições para seus respectivos manipuladores
static const tCGI cgi_handlers[] = {
    {
        "/set_setpoint.cgi", cgi_setpoint_handler  // Manipulador para /set_setpoint.cgi
    },
};


void cgi_init(void) {
    // Registra os manipuladores de CGI
    http_set_cgi_handlers(cgi_handlers, sizeof(cgi_handlers) / sizeof(tCGI));
}
