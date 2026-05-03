#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* ---------------- GPIO ---------------- */
#define PIR_GPIO     GPIO_NUM_15
#define LED_GPIO     GPIO_NUM_2
#define BUZZER_GPIO  GPIO_NUM_23

/* ---------------- I2C / LCD ---------------- */
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

#define PCF8574_ADDR 0x27
#define LCD_EN        0x04
#define LCD_RW        0x02
#define LCD_RS        0x01
#define LCD_BACKLIGHT 0x08

#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME   0x02
#define LCD_CMD_ENTRY_MODE    0x06
#define LCD_CMD_DISPLAY_ON    0x0C
#define LCD_CMD_FUNCTION_SET  0x28

#define cmd_delay   500
#define pulse_delay 10

/* ---------------- Keypad ---------------- */
#define ROW0 GPIO_NUM_32
#define ROW1 GPIO_NUM_33
#define ROW2 GPIO_NUM_25
#define ROW3 GPIO_NUM_26
#define COL0 GPIO_NUM_27
#define COL1 GPIO_NUM_14
#define COL2 GPIO_NUM_13
#define COL3 GPIO_NUM_12
#define ADMIN_PIN "1245"
#define PIN_LEN 4

int admin_mode = 0;
char pin_buf[PIN_LEN + 1];
int pin_index = 0;
int64_t disarm_time = 0;
char last_key = 0;
int show_admin_prompt = 0;

static void alarm_on(void);
static void alarm_off(void);
int pir_ready = 1;

/* YOUR requested inverted order – unchanged */
gpio_num_t row_pins[4] = { ROW0, ROW1, ROW2, ROW3 };
gpio_num_t col_pins[4] = { COL0, COL1, COL2, COL3 };

const char keys[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

/* ---------------- Globals ---------------- */
int64_t alarm_disable_until = 0;
int alarm_latched = 0;

/* ---------------- Delay ---------------- */
static void delay_ms(uint32_t ms){
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ---------------- I2C ---------------- */
static void i2c_master_init(){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* ---------------- LCD ---------------- */
static void pcf8574_write(uint8_t data){
    data |= LCD_BACKLIGHT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8574_ADDR<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, cmd_delay/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

static void lcd_pulse(uint8_t data){
    pcf8574_write(data | LCD_EN);
    delay_ms(pulse_delay);
    pcf8574_write(data & ~LCD_EN);
    delay_ms(pulse_delay);
}

static void lcd_send(uint8_t value, bool isData){
    uint8_t high = value & 0xF0;
    uint8_t low  = (value << 4) & 0xF0;
    if(isData){ high |= LCD_RS; low |= LCD_RS; }
    lcd_pulse(high);
    lcd_pulse(low);
}

static void lcd_command(uint8_t cmd){ lcd_send(cmd, false); }
static void lcd_data(uint8_t data){ lcd_send(data, true); }

static void lcd_init(){
    delay_ms(500);
    lcd_command(LCD_CMD_FUNCTION_SET);
    lcd_command(LCD_CMD_DISPLAY_ON);
    lcd_command(LCD_CMD_ENTRY_MODE);
    lcd_command(LCD_CMD_RETURN_HOME);
    lcd_command(LCD_CMD_CLEAR_DISPLAY);
}

static void lcd_cursor(uint8_t r, uint8_t c){
    lcd_command((r==0 ? 0x80 : 0xC0) + c);
}

static void lcd_print(char *s){
    while(*s) lcd_data(*s++);
}

/* ---------------- Keypad ---------------- */
static void keypad_init(){
    gpio_config_t io = {0};

    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask =
        (1ULL<<ROW0)|(1ULL<<ROW1)|(1ULL<<ROW2)|(1ULL<<ROW3);
    gpio_config(&io);

    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pin_bit_mask =
        (1ULL<<COL0)|(1ULL<<COL1)|(1ULL<<COL2)|(1ULL<<COL3);
    gpio_config(&io);

    for(int i=0;i<4;i++) gpio_set_level(row_pins[i],1);
}

static char scan_keypad(){
    for(int r=0;r<4;r++){
        for(int i=0;i<4;i++)
            gpio_set_level(row_pins[i], i==r ? 0 : 1);

        vTaskDelay(pdMS_TO_TICKS(5));

        for(int c=0;c<4;c++){
            if(gpio_get_level(col_pins[c])==0){
                while(gpio_get_level(col_pins[c])==0)
                    vTaskDelay(pdMS_TO_TICKS(10));
                return keys[r][c];
            }
        }
    }
    return 0;
}

/* ---------------- PIR & Alarm ---------------- */
static void pir_init(){
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<PIR_GPIO),
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&io);
}

static int pir_read_stable(){
    int cnt=0;
    for(int i=0;i<5;i++){
        if(gpio_get_level(PIR_GPIO)) cnt++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return (cnt>=3);
}

static void alarm_init(){
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<LED_GPIO)|(1ULL<<BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
    alarm_off();
}

static void alarm_on(){
    gpio_set_level(LED_GPIO,1);
    gpio_set_level(BUZZER_GPIO,1);
}

static void alarm_off(){
    gpio_set_level(LED_GPIO,0);
    gpio_set_level(BUZZER_GPIO,0);
}

/* ---------------- MAIN ---------------- */
void app_main() {
    i2c_master_init();
    lcd_init();
    keypad_init();
    pir_init();
    alarm_init();

    lcd_cursor(0, 0);
    lcd_print("PIR SENSOR      ");
    lcd_cursor(1, 0);
    lcd_print("PIR WARMING UP  ");
    vTaskDelay(pdMS_TO_TICKS(30000));   // 30 sec warm-up

    lcd_cursor(1, 0);
    lcd_print("ALARM ARMED     ");

    while (1) {
        char key = scan_keypad();

        // ---- key latch ----
        if (key == last_key) key = 0;
        if (key) last_key = key;
        else last_key = 0;

        /* ===== Motion detect ONLY when armed ===== */
        if (!alarm_latched && !admin_mode && disarm_time == 0) {
            if (!gpio_get_level(PIR_GPIO)) {
                pir_ready = 1;
            }

            if (pir_ready && pir_read_stable()) {
                pir_ready = 0;
                alarm_latched = 1;
                show_admin_prompt = 0;
            }
        }

        /* ===== Alarm active → wait for A ===== */
        if (alarm_latched && !admin_mode) {
            if (!show_admin_prompt) {
                lcd_cursor(0, 0);
                lcd_print("MOTION DETECTED ");
                lcd_cursor(1, 0);
                lcd_print("Press A Admin   ");
                show_admin_prompt = 1;
            }

            if (key == 'C' || key == 'D' || key == '#' || key == '*' ) {
                admin_mode = 1;
                pin_index = 0;
                show_admin_prompt = 0;

                lcd_cursor(0, 0);
                lcd_print("Enter 4Digit Pin");
                lcd_cursor(1, 0);
                lcd_print("                ");
            }
        }

        /* ===== Admin Mode: PIN entry ===== */
        else if (admin_mode) {
            if (key >= '0' && key <= '9') {
                if (pin_index < PIN_LEN) {
                    pin_buf[pin_index++] = key;
                    lcd_cursor(1, pin_index - 1);
                    lcd_data('*');
                }

                if (pin_index == PIN_LEN) {
                    pin_buf[PIN_LEN] = '\0';

                    if (strcmp(pin_buf, ADMIN_PIN) == 0) {
                        // ✔ correct PIN
                        alarm_latched = 0;
                        admin_mode = 0;
                        alarm_off();
                        pir_ready = 1;
                        disarm_time = esp_timer_get_time();

                        lcd_cursor(0, 0);
                        lcd_print("PIR SENSOR      ");
                        lcd_cursor(1, 0);
                        lcd_print("ALARM DISARMED  ");
                    } else {
                        // ✘ wrong PIN
                        lcd_cursor(0, 0);
                        lcd_print("TRY AGAIN !     ");
                        lcd_cursor(1, 0);
                        lcd_print("                ");
                        pin_index = 0;
                    }
                }
            }
        }

        /* ===== After disarm → re-arm after 30 sec ===== */
        else if (!alarm_latched && disarm_time > 0) {
            if (esp_timer_get_time() - disarm_time > 60000000) {
                disarm_time = 0;
                pir_ready = 1;

                lcd_cursor(0, 0);
                lcd_print("PIR SENSOR      ");
                lcd_cursor(1, 0);
                lcd_print("ALARM ARMED     ");
            }
        }

        /* ===== Alarm output ===== */
        if (alarm_latched) {
            alarm_on();
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
