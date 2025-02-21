/* =================================================================
* Development of Temperature PID Controller for Industrial Thermal Plant Model, using Raspberry Pi Pico W and Web Server
* Final project of the Embedded Systems Training - IFMA / Embarca Tech
* Author: Guilherme Abreu Carvalho
* E-mail: eng.guilherme.ac@gmail.com
* Date: February 8, 2025.
==================================================================== */

// --- Importing libraries --- //
#include <stdio.h>            // Standard input/output functions for printing and managing I/O
#include <string.h>           // String manipulation functions
#include <stdlib.h>           // Standard library functions like memory allocation
#include <ctype.h>            // Character classification functions
#include "lwip/apps/httpd.h"  // LWIP library for HTTP server support
#include "pico/stdlib.h"      // Standard Raspberry Pi Pico library for input/output operations
#include "pico/cyw43_arch.h"  // Library for communication with the CYW43 chip (Wi-Fi on Raspberry Pi Pico W)
#include "lwipopts.h"         // LWIP configuration options (for the lightweight TCP/IP stack used in embedded systems)
#include "ssi.h"              // Library for Server-Side Includes (SSI), used to dynamically insert content into HTML pages served by the HTTP server
#include "cgi.h"              // Library for Common Gateway Interface (CGI) support, used to handle HTTP requests and interact with scripts or specific functions
#include <math.h>             // Standard C math library
#include "pico/binary_info.h" // Binary information functions to help with debugging and providing runtime information
#include "inc/ssd1306.h"      // Library to control an SSD1306 OLED display, used for visual output
#include "hardware/i2c.h"     // I2C communication functions for interaction with external devices
#include "hardware/pwm.h"     // PWM (Pulse Width Modulation) functions to control actuators such as fans or heaters
#include "hardware/clocks.h"  // Clock management functions, necessary for accurate timing and synchronization in the system

// --- Definitions --- //
#define N 2                    // ARX order
#define MIN_TEMPERATURE 25.0   // Minimum system temperature
#define MAX_TEMPERATURE 200.0  // Maximum system temperature
#define MIN_CONTROL_SIGNAL 0   // Minimum voltage value
#define MAX_CONTROL_SIGNAL 100 // Maximum voltage value
#define Kp 0.5                 // Proportional gain of PID Controller
#define Ki 0.1                 // Integral gain of PID Controller
#define Kd 0.05                // Derivative gain of PID Controller
#define LED_G 11               // Green LED pin
#define LED_B 12               // Green LED pin
#define I2C_SDA 14             // Pin number for the I2C data line (SDA)
#define I2C_SCL 15             // Pin number for the I2C clock line (SCL)
#define BUZZER_PIN 21          // Pin number for controlling the buzzer
#define BUZZER_FREQUENCY 100   // Frequency of the buzzer sound (in Hz)

// --- Constants --- //
const char WIFI_SSID[] = "YOU_SSID";       // Wi-Fi network name
const char WIFI_PASSWORD[] = "YOU_PASSWORD"; // Wi-Fi network password
static double previous_error = 0.0;          // Previous error for derivative term
static double integral = 0.0;                // Integral sum
double current_temperature = 25.0;           // Initial temperature
double control_signal = 0.0;                 // Initial control signal (voltage)
double error = 1.0;                          // Initial error
double setpoint = 100;                       // Initial target temperature
bool beeped = false;                         // Variable to control the beep

// --- Function to initialize LEDs --- //
void init_leds()
{
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_init(LED_B);
    gpio_set_dir(LED_B, GPIO_OUT);
}

// --- Function to initialize OLED Display --- //
void init_display()
{
    // i2c initialization
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Complete initialization process of OLED SSD1306
    ssd1306_init();
}

void pwm_init_buzzer()
{
    // Configure the pin as PWM output
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    // Get the PWM slice associated with the pin
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Set PWM to desired frequency
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));
    pwm_init(slice_num, &config, true);

    // Start PWM at low level
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

// --- Function to emit a beep with a specified duration --- //
void beep(uint duration_ms)
{
    // Get the PWM slice associated with the pin
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Set duty cycle to 50% (active)
    pwm_set_gpio_level(BUZZER_PIN, 2048);

    // Timing
    sleep_ms(duration_ms);

    // Disable the PWM signal (duty cycle 0)
    pwm_set_gpio_level(BUZZER_PIN, 0);

    // Pause between beeps
    sleep_ms(100);
}

// --- Function to update display --- //
void update_display(bool controlled, double setpoint, double current_temperature, double error)
{
    struct render_area frame_area = {
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1};

    calculate_render_area_buffer_length(&frame_area);
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);

    // Format strings for display on the display
    char temperature_str[20], error_str[20], setpoint_str[20], status_str[20];
    snprintf(status_str, sizeof(status_str), "%s", controlled ? "  Controlled  " : "  Controlling  ");
    snprintf(setpoint_str, sizeof(setpoint_str), "S: %.2f C", setpoint);
    snprintf(temperature_str, sizeof(temperature_str), "T: %.2f C", current_temperature);
    snprintf(error_str, sizeof(error_str), "E: %.2f", error);

    // Render strings on the display
    ssd1306_draw_string(ssd, 5, 0, status_str);
    ssd1306_draw_string(ssd, 5, 16, setpoint_str);
    ssd1306_draw_string(ssd, 5, 32, temperature_str);
    ssd1306_draw_string(ssd, 5, 48, error_str);

    render_on_display(ssd, &frame_area);
}

// --- Function to connect to Wi-Fi --- //
void wifi_configure()
{
    // Initialize the Wi-Fi chip
    cyw43_arch_init();

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    // Attempts to connect to the specified Wi-Fi network, with a timeout of 30 seconds
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Trying to connect to Wi-Fi...\n");
    }

    // Get the IP address assigned to the device and display it in the console
    uint8_t *ip_address = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    printf("Connected with IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
}

// --- Web Server configuration function --- //
void web_server_configure()
{
    // Initialize the built-in HTTP server
    httpd_init();
    // Configure the SSI (Server-Side Includes) handler
    ssi_init();
    // Configure the CGI (Common Gateway Interface) handler
    cgi_init();
}

// --- Function that simulates a thermal plant with ARX model --- //
double simulate_arx(double u)
{
    // An ARX (Auto-Regressive with eXogenous Input) model is defined by:
    // y(k) = a1 * y(k - 1) + a2 * y(k - 2) + b1 * u(k - 1) + b2 * u(k - 2)

    // ARX ​​model parameters
    const double a[N] = {0.8, -0.2}; // Output coefficients
    const double b[N] = {0.7, 0.2};  // Input coefficients

    // History of last inputs and outputs
    static double y_hist[N] = {25.0, 25.0}; // Temperature history (initialized with 25°C)
    static double u_hist[N] = {0.0, 0.0};   // Voltage history (initialized with 0 V)

    double y = 0.0; // Initial output

    // Application of the ARX model
    for (int i = 0; i < N; i++)
    {
        y += a[i] * y_hist[i];
        y += b[i] * u_hist[i];
    }

    // Update histories
    for (int i = N - 1; i > 0; i--)
    {
        y_hist[i] = y_hist[i - 1];
        u_hist[i] = u_hist[i - 1];
    }

    y_hist[0] = y; // Update the latest output
    u_hist[0] = u; // Update the latest entry

    // Limits the temperature to stay between 0 and 200
    if (y > MAX_TEMPERATURE)
    {
        y = MAX_TEMPERATURE;
    }
    else if (y < MIN_TEMPERATURE)
    {
        y = MIN_TEMPERATURE;
    }

    return y; // Returns the output for each instant
}

// --- Function that implements the PID controller --- //
double pid_controller(double error)
{
    // Calculate integral (sum of errors over time)
    integral += error;

    // Calculate derivative (change in error over time)
    double derivative = error - previous_error;

    // PID output (control signal)
    double control_signal = Kp * error + Ki * integral + Kd * derivative;

    // Limits the control signal to stay between 0 and 100
    if (control_signal > MAX_CONTROL_SIGNAL)
    {
        control_signal = MAX_CONTROL_SIGNAL;
    }
    else if (control_signal < MIN_CONTROL_SIGNAL)
    {
        control_signal = MIN_CONTROL_SIGNAL;
    }

    // Save the current error for the next derivative calculation
    previous_error = error;

    return control_signal;
}

int main()
{
    // Initialize standard input and output
    stdio_init_all();

    // Initialize LEDs
    init_leds();

    // Initialize display
    init_display();

    // Initialize buzzer
    pwm_init_buzzer();

    // Configure and connect to the Wi-Fi network
    wifi_configure();

    // Configure and initialize the web server
    web_server_configure();

    while (true)
    {
        // Calculate the error (difference between setpoint and current temperature)
        error = setpoint - current_temperature;

        // Get the control signal from the PID controller
        control_signal = pid_controller(error);

        // Use the control signal (voltage) as the input for the ARX model
        current_temperature = simulate_arx(control_signal);

        // Check if the temperature is within an acceptable range of the setpoint
        bool controlled = fabs(setpoint - current_temperature) < 0.001;
        
        // Update the display with the current system status
        update_display(controlled, setpoint, current_temperature, error);

        if (controlled)
        {
            // Turn off the blue LED and turn on the green LED to indicate stability
            gpio_put(LED_B, false);
            gpio_put(LED_G, true);

            // Beep once when reaching the setpoint, if not already beeped
            if (!beeped)
            {
                beep(500);
                beeped = true;
            }
        }
        else
        {
            // Turn off the green LED and turn on the blue LED to indicate adjustment in progress
            gpio_put(LED_G, false);
            gpio_put(LED_B, true);
            beeped = false; // Reset beep flag for the next stable condition
        }

        // Wait for 500 milliseconds before the next control iteration
        sleep_ms(500);
    }
}
