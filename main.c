#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "myLcd.h"
#include <string.h>

#define AS7341_ADDR     0x39
#define AS7341_ID_REG   0x92
#define AS7341_ENABLE      0x80
#define AS7341_ATIME       0x81
#define AS7341_CFG1        0xAA
#define AS7341_CFG6        0xAF
#define AS7341_STATUS2     0xA3
#define AS7341_ASTEP_L     0xCA
#define AS7341_ASTEP_H     0xCB
#define AS7341_CH0_DATA_L  0x95

#define AS7341_ENABLE_PON     0x01
#define AS7341_ENABLE_SP_EN   0x02
#define AS7341_ENABLE_SMUXEN  0x10

#define AS7341_STATUS2_AVALID 0x40
#define AVG_SAMPLES 5

#define AS7341_CFG0       0xA9
#define AS7341_CONFIG     0x70
#define AS7341_LED        0x74

#define CAL_POINTS 5
#define LAB_FEATURES 4

typedef enum
{
    CAL_9005 = 0,
    CAL_9016,
    CAL_3020,
    CAL_6018,
    CAL_5002,
    CAL_DONE
} calibration_state_t;

typedef enum
{
    MODE_SELECT = 0,
    MODE_RAL_RECOGNITION,
    MODE_LAB_MEASUREMENT
} system_mode_t;

volatile system_mode_t system_mode = MODE_SELECT;

volatile calibration_state_t cal_state = CAL_9005;
volatile bool calibration_done = false;

const char *lcd_scroll_msg = 0;
uint16_t lcd_scroll_pos = 0;
uint16_t lcd_scroll_counter = 0;
uint16_t lcd_scroll_speed = 80;
bool lcd_scroll_active = false;

float raw_lab_cal[CAL_POINTS][3];
float lab_corr[3][LAB_FEATURES];

float target_lab[CAL_POINTS][3] =
{
    {  4.02f,   0.36f,  -0.99f},   // RAL 9005 black
    { 94.72f,  -0.71f,   3.01f},   // RAL 9016 white
    { 40.47f,  59.14f,  48.29f},   // RAL 3020 red
    { 57.69f, -35.32f,  42.65f},   // RAL 6018 yellow green
    { 24.51f,  12.6f, -42.5f}    // RAL 5002 ultramarine blue
};

typedef struct
{
    uint16_t ral_number;
    uint16_t n[8];
} ral_spec_t;

const ral_spec_t ral_database[] =
{
    {1000, {64,214,363,610,731,815,1000,742}},
    {1001, {61,198,350,542,650,813,1000,674}},
    {1002, {58,172,298,514,679,819,1000,661}},
    {1003, {49,93,170,373,670,832,1000,684}},
    {1004, {51,100,190,403,691,853,1000,687}},
    {1005, {55,113,196,459,691,868,1000,728}},
    {1006, {49,94,160,375,588,837,1000,684}},
    {1007, {48,93,153,313,585,807,1000,693}},
    {1011, {59,175,286,466,630,827,1000,678}},
    {1012, {55,119,234,511,719,825,1000,712}},
    {1013, {72,296,459,663,736,839,1000,671}},
    {1014, {65,230,371,601,736,843,1000,672}},
    {1015, {68,256,426,620,718,841,1000,673}},
    {1016, {55,106,313,644,734,831,1000,670}},
    {1017, {51,121,195,382,609,809,1000,684}},
    {1018, {51,103,223,507,681,817,1000,667}},
    {1019, {69,258,400,592,696,826,1000,675}},
    {1020, {67,224,371,617,746,835,1000,692}},
    {1021, {48,85,161,436,659,787,1000,708}},
    {1023, {49,86,176,458,656,811,1000,658}},
    {1024, {59,164,282,500,690,830,1000,668}},
    {1027, {60,147,247,531,738,838,1000,717}},
    {1028, {45,80,136,291,569,821,1000,672}},
    {1032, {52,104,216,430,668,837,1000,703}},
    {1033, {46,93,157,311,557,781,1000,637}},
    {1034, {50,118,192,362,534,770,1000,697}},
    {1037, {48,96,157,299,635,839,1000,690}},
    {2000, {44,90,148,262,398,754,1000,677}},
    {2001, {45,103,154,224,314,635,1000,746}},
    {2002, {40,90,135,176,233,535,1000,733}},
    {2003, {42,85,133,205,371,696,1000,681}},
    {2004, {42,85,128,166,321,724,1000,702}},
    {2008, {42,87,131,201,358,708,1000,698}},
    {2009, {42,86,131,187,298,674,1000,716}},
    {2010, {43,94,145,222,347,721,1000,675}},
    {2011, {43,85,132,234,359,695,1000,701}},
    {2012, {46,113,172,224,342,698,1000,738}},
    {2017, {41,80,122,158,321,752,1000,681}},
    {3000, {48,110,162,207,248,554,1000,998}},
    {3001, {42,101,153,197,228,516,1000,718}},
    {3002, {43,102,150,192,223,466,1000,806}},
    {3003, {47,123,181,231,257,452,1000,753}},
    {3004, {44,110,160,205,236,311,753,1000}},
    {3005, {61,193,292,386,411,568,1000,667}},
    {3007, {76,257,400,535,567,715,1000,660}},
    {3009, {64,192,297,392,458,685,1000,700}},
    {3011, {54,149,226,294,328,565,1000,708}},
    {3012, {56,175,276,380,505,785,1000,663}},
    {3013, {48,113,166,216,262,504,951,1000}},
    {3014, {50,149,214,290,315,589,1000,803}},
    {3015, {58,217,324,396,443,733,1000,665}},
    {3016, {48,122,182,235,306,645,1000,762}},
    {3017, {43,118,168,205,244,551,1000,710}},
    {3018, {40,99,148,176,201,520,1000,711}},
    {3020, {38,83,123,157,192,458,1000,733}},
    {3022, {44,114,170,247,314,611,1000,683}},
    {3027, {41,103,147,176,200,482,1000,730}},
    {3028, {37,79,118,144,183,496,1000,709}},
    {3031, {49,129,191,238,269,603,1000,779}},
    {4001, {69,272,363,417,418,569,1000,885}},
    {4002, {54,166,239,286,314,584,1000,862}},
    {4003, {52,182,227,244,270,576,1000,791}},
    {4004, {55,170,243,302,317,435,1000,761}},
    {4005, {73,321,412,424,414,530,860,1000}},
    {4006, {57,196,239,254,271,533,1000,816}},
    {4007, {77,249,356,444,470,582,933,1000}},
    {4008, {67,280,336,355,370,655,1000,685}},
    {4009, {75,316,447,562,633,824,1000,661}},
    {4010, {46,139,170,184,208,487,1000,757}},
    {5000, {112,550,753,844,767,829,1000,682}},    
    {5001, {111,523,758,882,771,807,1000,693}},
    {5002, {125,706,816,792,743,813,1000,700}},
    {5003, {108,494,681,773,741,810,1000,695}},
    {5004, {95,355,540,704,725,808,1000,688}},
    {5005, {124,739,1000,905,720,769,976,684}},
    {5007, {111,568,799,882,741,766,1000,712}},
    {5008, {95,376,569,736,733,808,1000,685}},
    {5009, {118,562,825,959,819,835,1000,686}},
    {5010, {121,678,944,896,740,791,1000,700}},
    {5011, {100,401,595,738,733,811,1000,690}},
    {5012, {116,704,983,1000,716,679,876,622}},
    {5013, {105,460,632,742,720,796,1000,685}},
    {5014, {94,455,649,770,692,737,1000,684}},
    {5015, {116,715,1000,939,679,650,793,566}},
    {5017, {111,681,1000,849,646,675,849,602}},
    {5018, {92,448,789,1000,727,643,757,519}},
    {5019, {119,657,1000,956,774,799,984,700}},
    {5020, {103,425,766,842,750,806,1000,701}},
    {5021, {97,446,896,1000,714,666,791,546}},
    {5022, {105,466,604,696,689,785,1000,692}},
    {5023, {107,559,768,841,718,770,1000,693}},
    {5024, {112,587,844,1000,860,851,999,677}},
    {6000, {94,394,700,1000,934,792,857,638}},
    {6001, {90,313,567,1000,977,795,901,667}},
    {6002, {95,306,575,1000,912,832,996,791}},
    {6003, {86,308,494,735,806,852,1000,703}},
    {6004, {102,403,680,872,791,820,1000,694}},
    {6005, {98,365,631,872,825,828,1000,701}},
    {6006, {88,322,503,698,743,826,1000,684}},
    {6007, {90,313,507,748,754,824,1000,672}},
    {6008, {88,303,480,683,720,813,1000,685}},
    {6009, {93,331,532,755,769,823,1000,694}},
    {6010, {94,300,574,1000,989,875,985,693}},
    {6011, {86,306,567,879,891,886,1000,708}},
    {6012, {93,339,542,756,751,804,1000,701}},
    {6013, {78,274,433,711,790,843,1000,678}},
    {6014, {83,277,446,652,687,785,1000,725}},
    {6015, {88,321,505,697,738,818,1000,678}},
    {6016, {91,359,718,1000,811,712,833,597}},
    {6017, {82,267,495,1000,919,777,883,622}},
    {6018, {70,202,459,1000,916,694,719,555}},
    {6019, {85,349,600,921,970,923,1000,722}},
    {6020, {91,331,535,778,829,850,1000,709}},
    {6021, {87,340,575,904,986,932,1000,730}},
    {6022, {88,304,477,673,741,828,1000,690}},
    {6024, {78,286,626,1000,823,639,693,510}},
    {6025, {83,256,425,811,831,753,1000,844}},
    {6026, {96,381,800,1000,775,732,882,619}},
    {6027, {93,457,759,1000,902,810,837,631}},
    {6028, {93,347,575,851,768,785,1000,695}},
    {6029, {77,269,608,1000,719,607,711,513}},
    {6032, {78,300,615,1000,845,638,698,517}},
    {6033, {94,436,701,1000,835,751,879,597}},
    {6034, {97,461,747,1000,878,821,949,645}},
    {6037, {63,188,464,1000,744,540,596,436}},
    {6039, {70,167,427,916,925,884,1000,757}},
    {7000, {88,393,593,766,752,812,1000,712}},
    {7001, {85,380,567,744,744,814,1000,710}},
    {7002, {75,277,453,657,725,832,1000,664}},
    {7003, {79,307,486,690,743,829,1000,678}},
    {7004, {82,354,528,704,748,838,1000,659}},
    {7005, {85,351,534,730,763,839,1000,663}},
    {7006, {78,293,459,640,715,842,1000,676}},
    {7008, {70,225,378,582,676,817,1000,672}},
    {7009, {86,328,513,732,765,832,1000,680}},
    {7010, {87,346,530,724,762,834,1000,693}},
    {7011, {90,374,564,743,752,820,1000,692}},
    {7012, {87,359,542,715,727,803,1000,713}},
    {7013, {81,292,473,657,701,816,1000,679}},
    {7015, {89,373,555,712,725,820,1000,692}},
    {7016, {94,368,561,738,750,823,1000,680}},
    {7021, {92,351,535,704,729,810,1000,692}},
    {7022, {86,328,503,691,729,827,1000,670}},
    {7023, {83,338,518,723,770,846,1000,676}},
    {7024, {89,361,542,705,713,796,1000,687}},
    {7026, {93,370,563,754,764,825,1000,677}},
    {7030, {77,316,485,678,728,820,1000,689}},
    {7031, {93,402,598,789,793,844,1000,671}},
    {7032, {74,295,474,676,736,836,1000,669}},
    {7033, {80,323,532,730,749,831,1000,673}},
    {7034, {73,263,430,653,727,828,1000,670}},
    {7035, {78,343,524,716,748,829,1000,684}},
    {7036, {79,336,507,678,716,834,1000,660}},
    {7037, {83,350,531,716,752,839,1000,661}},
    {7038, {78,330,514,707,747,834,1000,669}},
    {7039, {81,318,488,686,734,824,1000,683}},
    {7040, {83,362,552,710,723,810,1000,755}},
    {7042, {83,365,548,732,760,839,1000,663}},
    {7043, {90,359,549,734,757,831,1000,674}},
    {7044, {75,309,484,675,728,830,1000,670}},
    {7045, {83,364,550,725,738,820,1000,697}},
    {7046, {86,373,559,731,737,811,1000,708}},
    {7047, {78,340,520,684,716,815,1000,724}},
    {8000, {66,207,333,535,680,835,1000,671}},
    {8001, {61,163,259,421,592,820,1000,692}},
    {8002, {66,212,329,452,549,774,1000,698}},
    {8003, {66,190,307,440,572,815,1000,714}},
    {8004, {59,168,261,355,451,706,1000,817}},
    {8007, {69,219,337,492,594,794,1000,678}},
    {8008, {69,214,343,501,628,813,1000,695}},
    {8011, {73,241,377,518,607,796,1000,682}},
    {8012, {64,202,311,415,482,699,1000,683}},
    {8014, {80,273,430,597,669,807,1000,684}},
    {8015, {72,238,364,479,549,757,1000,744}},
    {8016, {76,259,400,547,610,781,1000,660}},
    {8017, {81,279,432,582,650,785,1000,672}},
    {8019, {84,302,467,638,666,776,1000,680}},
    {8022, {89,317,496,662,693,796,1000,684}},
    {8023, {52,137,212,325,438,712,1000,715}},
    {8024, {68,223,346,496,613,804,1000,668}},
    {8025, {71,242,377,534,630,807,1000,680}},
    {8028, {78,266,423,580,658,808,1000,680}},
    {9001, {72,301,469,657,721,833,1000,670}},
    {9002, {75,321,498,694,740,833,1000,669}},
    {9003, {76,340,513,686,719,813,1000,701}},
    {9004, {91,330,512,682,709,803,1000,688}},
    {9005, {93,333,521,691,718,807,1000,688}},
    {9006, {77,333,508,689,732,822,1000,647}},
    {9010, {74,320,493,686,735,830,1000,669}},
    {9011, {92,338,522,696,722,812,1000,680}},
    {9012, {72,307,478,682,744,835,1000,669}},
    {9016, {77,343,519,704,744,833,1000,670}},
    {9017, {92,329,511,684,713,805,1000,684}},
    {9018, {78,341,536,730,763,841,1000,669}}
};

#define RAL_COUNT (sizeof(ral_database) / sizeof(ral_database[0]))

volatile uint8_t as7341_id = 0;
volatile uint8_t lcd_button_event = 0; 
/* 0 = none, 1 = S1, 2 = S2 */

void clock_init(void);
void i2c_init(void);
void uart_init(void);
void delay_ms(unsigned int ms);

void uart_putc(char c);
void uart_print(const char *str);
void uart_print_uint8(uint8_t value);

uint8_t i2c_read8(uint8_t reg);

bool i2c_write8(uint8_t reg, uint8_t value);
uint16_t i2c_read16(uint8_t reg_low);

void as7341_init_basic(void);
void as7341_setup_smux_f1f4(void);
void as7341_setup_smux_f5f8(void);
void as7341_start_measurement(void);
void as7341_wait_data(void);
void as7341_read_channels(uint16_t *ch0, uint16_t *ch1, uint16_t *ch2, uint16_t *ch3, uint16_t *clear, uint16_t *nir);
void uart_print_uint16(uint16_t value);

void as7341_read_spectrum_averaged(uint16_t spectrum[10]);

void as7341_led_on(void);
void as7341_led_off(void);

void normalize_spectrum(uint16_t spectrum[10], uint16_t norm[8]);
void print_normalized_spectrum(uint16_t norm[8]);

void print_lab(float L, float a, float b);
void print_float_simple(float value);

void buttons_init(void);
bool s1_pressed(void);
bool s2_pressed(void);
void print_calibration_prompt(void);

void spectrum_to_xyz(float refl[8], float *x_val, float *y_val, float *z_val);
void xyz_to_lab(float x_val, float y_val, float z_val, float *L, float *a, float *b);
float cube_root_approx(float t);
float lab_f(float t);

void save_lab_calibration_point(uint8_t index, float L, float a, float b);
void compute_lab_correction(void);
void apply_lab_correction(float L, float a, float b, float *Lc, float *ac, float *bc);
bool solve_4x4(float A[LAB_FEATURES][LAB_FEATURES], float y[LAB_FEATURES], float x[LAB_FEATURES]);
void print_calibration_prompt(void);
void spectrum_to_xyz_from_norm(uint16_t norm[8], float *x_val, float *y_val, float *z_val);

uint16_t find_nearest_ral_by_spectrum(uint16_t norm[8], uint32_t *best_distance);
void print_normalized_spectrum(uint16_t norm[8]);
void uart_print_uint32(uint32_t value);
void export_ral_entry(uint16_t norm[8]);

void lcd_display_text(const char *txt);
void lcd_scroll_text(const char *text, uint16_t scroll_delay_ms);
void lcd_show_ral(uint16_t ral_number);
void lcd_show_lab(float L, float a, float b);
void lcd_scroll_text_repeat(const char *text, uint16_t scroll_delay_ms, uint8_t repeat_count);
bool both_buttons_pressed(void);
void lcd_start_scroll(const char *text, uint16_t speed);
void lcd_scroll_update(void);
void lcd_stop_scroll(void);
uint8_t get_button_event(void);
bool s1_long_pressed(void);
bool menu_combo_pressed(void);
void lcd_add_lab_value(char *msg, uint8_t *p, float value);

int main(void)
{
    uint16_t spectrum[10];
    uint16_t norm[8];

    float x_val, y_val, z_val;
    float L, a, b;
    float Lc, ac, bc;

    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    clock_init();
    uart_init();
    i2c_init();
    buttons_init();
    myLCD_init();
    lcd_scroll_text_repeat("AS7341 COLOR SCANNER", 300, 1);

    delay_ms(500);
    uart_print("\r\nAS7341 COLOR SCANNER\r\n");

    as7341_init_basic();

    system_mode = MODE_SELECT;
    cal_state = CAL_9005;
    calibration_done = false;
    lcd_start_scroll("S1-RAL MODE S2-LAB MODE", 15);

    uart_print("\r\nSelect mode:\r\n");
    uart_print("S1 = RAL recognition mode\r\n");
    uart_print("S2 = Lab measurement mode\r\n\r\n");

    while (1)
    {
        lcd_scroll_update();
        if (system_mode != MODE_SELECT)
        {
            if (menu_combo_pressed())
            {
                lcd_stop_scroll();
                system_mode = MODE_SELECT;
                lcd_scroll_text_repeat("MAIN MENU", 300, 1);
                lcd_start_scroll("S1-RAL MODE S2-LAB MODE", 15);

                uart_print("\r\nBack to Main Menu\r\n");
                continue;
            }
        }

        if (system_mode == MODE_SELECT)
        {
            if (s1_pressed())
            {
                lcd_stop_scroll();

                system_mode = MODE_RAL_RECOGNITION;

                lcd_scroll_text_repeat("RAL MODE S1-SAVE S2-SCAN", 300, 1);
                lcd_start_scroll("S1-SAVE S2-SCAN", 15);

                uart_print("RAL recognition mode selected.\r\n");
                uart_print("S1 = export RAL entry\r\n");
                uart_print("S2 = identify nearest RAL\r\n\r\n");
            }
            else if (s2_pressed())
            {
                lcd_stop_scroll();

                system_mode = MODE_LAB_MEASUREMENT;

                uart_print("Lab measurement mode selected.\r\n");

                if (!calibration_done)
                {
                    cal_state = CAL_9005;
                    print_calibration_prompt();
                }
                else
                {
                    uart_print("Lab calibration already available.\r\n");
                    uart_print("Press S2 to measure Lab.\r\n\r\n");
                }
            }
        }
        else if (system_mode == MODE_RAL_RECOGNITION)
        {
            if (s1_pressed())
            {
                as7341_led_on();
                delay_ms(100);
                as7341_read_spectrum_averaged(spectrum);
                normalize_spectrum(spectrum, norm);

                export_ral_entry(norm);
                as7341_led_off();
                uart_print("Press S1 to export another RAL entry.\r\n");
                uart_print("Press S2 to identify nearest RAL.\r\n\r\n");
            }

            else if (s2_pressed())
            {
                as7341_led_on();
                delay_ms(100);
                uint32_t dist;
                uint16_t ral_number;

                lcd_scroll_text_repeat("SCANNING", 300, 1);

                as7341_read_spectrum_averaged(spectrum);
                normalize_spectrum(spectrum, norm);

                ral_number = find_nearest_ral_by_spectrum(norm, &dist);

                uart_print("Nearest RAL: ");
                uart_print_uint16(ral_number);

                uart_print("  Distance: ");
                uart_print_uint32(dist);
                uart_print("\r\n");
                lcd_show_ral(ral_number);

                print_normalized_spectrum(norm);
                uart_print("\r\n");
                as7341_led_off();
                uart_print("Press S1 to export RAL entry.\r\n");
                uart_print("Press S2 to scan another RAL color.\r\n\r\n");
            }
        }

        else if (system_mode == MODE_LAB_MEASUREMENT)
        {  
            if (!calibration_done)
            {
                if (s1_pressed())
                {
                    as7341_led_on();
                    delay_ms(100);
                    lcd_scroll_text_repeat("CALIBRATING", 300, 1);
                    as7341_read_spectrum_averaged(spectrum);
                    normalize_spectrum(spectrum, norm);

                    spectrum_to_xyz_from_norm(norm, &x_val, &y_val, &z_val);
                    xyz_to_lab(x_val, y_val, z_val, &L, &a, &b);

                    save_lab_calibration_point((uint8_t)cal_state, L, a, b);

                    as7341_led_off();

                    uart_print("Calibration color saved OK\r\n\r\n");
                    cal_state++;

                    if (cal_state == CAL_DONE)
                    {
                        lcd_scroll_text_repeat("CALIBRATION DONE", 300, 1);
                        compute_lab_correction();
                        calibration_done = true;

                        uart_print("5-point Lab correction complete.\r\n");
                        uart_print("Press S2 to measure Lab.\r\n\r\n");
                        lcd_start_scroll("PRESS S2 TO MEASURE COLOR", 15);

                    }
                    else
                    {
                        print_calibration_prompt();
                    }
                }
            }
            else
            {
                if (s2_pressed())
                {
                    as7341_led_on();
                    delay_ms(100);
                    lcd_scroll_text_repeat("SCANNING", 300, 1);
                    as7341_read_spectrum_averaged(spectrum);
                    normalize_spectrum(spectrum, norm);

                    spectrum_to_xyz_from_norm(norm, &x_val, &y_val, &z_val);
                    xyz_to_lab(x_val, y_val, z_val, &L, &a, &b);

                    apply_lab_correction(L, a, b, &Lc, &ac, &bc);
                    
                    as7341_led_off();

                    uart_print("Corrected ");
                    print_lab(Lc, ac, bc);
                    uart_print("\r\n");
                    lcd_show_lab(Lc, ac, bc);
                    lcd_start_scroll("PRESS S2 TO MEASURE ANOTHER COLOR", 15);
                    uart_print("Press S2 to measure another color.\r\n\r\n");
                }
            }
        }
        delay_ms(20);
    }
}

void clock_init(void)
{
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__VLOCLK;
    CSCTL3 = DIVM__1 | DIVS__1 | DIVA__1;
    CSCTL0_H = 0;
}

void i2c_init(void)
{
    P1SEL0 |= BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);

    UCB0CTLW0 = UCSWRST;
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC;
    UCB0CTLW0 |= UCSSEL__SMCLK;
    UCB0BRW = 10;
    UCB0I2CSA = AS7341_ADDR;
    UCB0CTLW0 &= ~UCSWRST;
}

void uart_init(void)
{
    P3SEL0 |= BIT4 | BIT5;
    P3SEL1 &= ~(BIT4 | BIT5);

    UCA1CTLW0 = UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;

    UCA1BRW = 6;
    UCA1MCTLW = UCOS16 | UCBRF_8 | 0x2000;

    UCA1CTLW0 &= ~UCSWRST;
}

void delay_ms(unsigned int ms)
{
    while (ms--)
    {
        __delay_cycles(1000);
    }
}

void uart_putc(char c)
{
    while (!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = c;
}

void uart_print(const char *str)
{
    while (*str)
    {
        uart_putc(*str++);
    }
}

void uart_print_uint8(uint8_t value)
{
    char buffer[4];
    int i = 0;

    if (value == 0)
    {
        uart_putc('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = (value % 10) + '0';
        value /= 10;
    }

    while (i > 0)
    {
        uart_putc(buffer[--i]);
    }
}

uint8_t i2c_read8(uint8_t reg)
{
    uint8_t value;
    uint32_t timeout;

    /* Write register address */
    timeout = 50000;
    while ((UCB0STATW & UCBBUSY) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    UCB0IFG &= ~(UCTXIFG0 | UCRXIFG0 | UCNACKIFG);
    UCB0I2CSA = AS7341_ADDR;

    UCB0CTLW0 |= UCTR;
    UCB0CTLW0 |= UCTXSTT;

    timeout = 50000;
    while (!(UCB0IFG & UCTXIFG0) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    UCB0TXBUF = reg;

    timeout = 50000;
    while (!(UCB0IFG & UCTXIFG0) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    UCB0CTLW0 |= UCTXSTP;

    timeout = 50000;
    while ((UCB0CTLW0 & UCTXSTP) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    delay_ms(1);

    /* Read one byte */
    UCB0IFG &= ~(UCRXIFG0 | UCNACKIFG);

    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;

    timeout = 50000;
    while ((UCB0CTLW0 & UCTXSTT) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    UCB0CTLW0 |= UCTXSTP;

    timeout = 50000;
    while (!(UCB0IFG & UCRXIFG0) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    value = UCB0RXBUF;

    timeout = 50000;
    while ((UCB0CTLW0 & UCTXSTP) && timeout)
        timeout--;

    if (!timeout)
        return 0xFF;

    return value;
}

bool i2c_write8(uint8_t reg, uint8_t value)
{
    uint32_t timeout;

    timeout = 50000;
    while ((UCB0STATW & UCBBUSY) && timeout)
        timeout--;

    if (!timeout)
        return false;

    UCB0IFG &= ~(UCTXIFG0 | UCNACKIFG);
    UCB0I2CSA = AS7341_ADDR;

    UCB0CTLW0 |= UCTR;
    UCB0CTLW0 |= UCTXSTT;

    timeout = 50000;
    while (!(UCB0IFG & UCTXIFG0) && timeout)
        timeout--;

    if (!timeout)
        return false;

    UCB0TXBUF = reg;

    timeout = 50000;
    while (!(UCB0IFG & UCTXIFG0) && timeout)
        timeout--;

    if (!timeout)
        return false;

    UCB0TXBUF = value;

    timeout = 50000;
    while (!(UCB0IFG & UCTXIFG0) && timeout)
        timeout--;

    if (!timeout)
        return false;

    UCB0CTLW0 |= UCTXSTP;

    timeout = 50000;
    while ((UCB0CTLW0 & UCTXSTP) && timeout)
        timeout--;

    if (!timeout)
        return false;

    return true;
}

uint16_t i2c_read16(uint8_t reg_low)
{
    uint8_t low;
    uint8_t high;

    low = i2c_read8(reg_low);
    high = i2c_read8(reg_low + 1);

    return ((uint16_t)high << 8) | low;
}

void uart_print_uint16(uint16_t value)
{
    char buffer[6];
    int i = 0;

    if (value == 0)
    {
        uart_putc('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = (value % 10) + '0';
        value /= 10;
    }

    while (i > 0)
    {
        uart_putc(buffer[--i]);
    }
}

void as7341_init_basic(void)
{
    i2c_write8(AS7341_ENABLE, AS7341_ENABLE_PON);
    delay_ms(10);

    /* Longer integration time */
    /* ATIME = 149 */
    i2c_write8(AS7341_ATIME, 149);

    /* ASTEP = 1499 */
    i2c_write8(AS7341_ASTEP_L, 1499 & 0xFF);
    i2c_write8(AS7341_ASTEP_H, 1499 >> 8);

    /* Gain = 64x */
    i2c_write8(AS7341_CFG1, 7);
}

void as7341_setup_smux_f1f4(void)
{
    i2c_write8(AS7341_ENABLE, AS7341_ENABLE_PON);
    delay_ms(5);

    i2c_write8(AS7341_CFG6, 0x10);
    delay_ms(5);

    i2c_write8(0x00, 0x30);
    i2c_write8(0x01, 0x01);
    i2c_write8(0x02, 0x00);
    i2c_write8(0x03, 0x00);
    i2c_write8(0x04, 0x00);
    i2c_write8(0x05, 0x42);
    i2c_write8(0x06, 0x00);
    i2c_write8(0x07, 0x00);
    i2c_write8(0x08, 0x50);
    i2c_write8(0x09, 0x00);
    i2c_write8(0x0A, 0x00);
    i2c_write8(0x0B, 0x00);
    i2c_write8(0x0C, 0x20);
    i2c_write8(0x0D, 0x04);
    i2c_write8(0x0E, 0x00);
    i2c_write8(0x0F, 0x30);
    i2c_write8(0x10, 0x01);
    i2c_write8(0x11, 0x50);
    i2c_write8(0x12, 0x00);
    i2c_write8(0x13, 0x06);
}

void as7341_setup_smux_f5f8(void)
{
    i2c_write8(AS7341_ENABLE, AS7341_ENABLE_PON);
    delay_ms(5);

    i2c_write8(AS7341_CFG6, 0x10);
    delay_ms(5);

    i2c_write8(0x00, 0x00);
    i2c_write8(0x01, 0x00);
    i2c_write8(0x02, 0x00);
    i2c_write8(0x03, 0x40);
    i2c_write8(0x04, 0x02);
    i2c_write8(0x05, 0x00);
    i2c_write8(0x06, 0x10);
    i2c_write8(0x07, 0x03);
    i2c_write8(0x08, 0x50);
    i2c_write8(0x09, 0x10);
    i2c_write8(0x0A, 0x03);
    i2c_write8(0x0B, 0x00);
    i2c_write8(0x0C, 0x00);
    i2c_write8(0x0D, 0x00);
    i2c_write8(0x0E, 0x24);
    i2c_write8(0x0F, 0x00);
    i2c_write8(0x10, 0x00);
    i2c_write8(0x11, 0x50);
    i2c_write8(0x12, 0x00);
    i2c_write8(0x13, 0x06);
}

void as7341_start_measurement(void)
{
    i2c_write8(AS7341_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SMUXEN);
    delay_ms(20);

    i2c_write8(AS7341_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SP_EN);
    delay_ms(150);

    i2c_init();
    delay_ms(5);
}

void as7341_wait_data(void)
{
    uint32_t timeout = 10000;

    while (!(i2c_read8(AS7341_STATUS2) & 0x40) && timeout)
    {
        timeout--;
    }
}

void as7341_read_channels(uint16_t *ch0, uint16_t *ch1,
                          uint16_t *ch2, uint16_t *ch3,
                          uint16_t *clear, uint16_t *nir)
{
    *ch0 = i2c_read16(AS7341_CH0_DATA_L);
    *ch1 = i2c_read16(AS7341_CH0_DATA_L + 2);
    *ch2 = i2c_read16(AS7341_CH0_DATA_L + 4);
    *ch3 = i2c_read16(AS7341_CH0_DATA_L + 6);
    *clear = i2c_read16(AS7341_CH0_DATA_L + 8);
    *nir = i2c_read16(AS7341_CH0_DATA_L + 10);
}

void as7341_led_on(void)
{
    /* Select low register bank */
    i2c_write8(AS7341_CFG0, 0x10);
    delay_ms(5);

    /* Select LED driver mode */
    i2c_write8(AS7341_CONFIG, 0x08);
    delay_ms(5);

    /* LED on, current setting high */
    i2c_write8(AS7341_LED, 0x8F);
    delay_ms(5);

    /* Return to normal high register bank */
    i2c_write8(AS7341_CFG0, 0x00);
    delay_ms(5);
}

void as7341_led_off(void)
{
    i2c_write8(AS7341_CFG0, 0x10);
    delay_ms(5);

    i2c_write8(AS7341_LED, 0x00);
    delay_ms(5);

    i2c_write8(AS7341_CFG0, 0x00);
    delay_ms(5);
}

void as7341_read_spectrum_averaged(uint16_t spectrum[10])
{
    uint8_t i;
    uint32_t sum[10] = {0};

    uint16_t f1, f2, f3, f4;
    uint16_t f5, f6, f7, f8;
    uint16_t clear1, nir1;
    uint16_t clear2, nir2;

    for (i = 0; i < AVG_SAMPLES; i++)
    {
        as7341_setup_smux_f1f4();
        as7341_start_measurement();
        as7341_wait_data();
        as7341_read_channels(&f1, &f2, &f3, &f4, &clear1, &nir1);

        as7341_setup_smux_f5f8();
        as7341_start_measurement();
        as7341_wait_data();
        as7341_read_channels(&f5, &f6, &f7, &f8, &clear2, &nir2);

        sum[0] += f1;
        sum[1] += f2;
        sum[2] += f3;
        sum[3] += f4;
        sum[4] += f5;
        sum[5] += f6;
        sum[6] += f7;
        sum[7] += f8;
        sum[8] += clear2;
        sum[9] += nir2;

        delay_ms(100);
    }

    for (i = 0; i < 10; i++)
    {
        spectrum[i] = sum[i] / AVG_SAMPLES;
    }
}

void normalize_spectrum(uint16_t spectrum[10], uint16_t norm[8])
{
    uint8_t i;
    uint16_t max_value = 0;

    for (i = 0; i < 8; i++)
    {
        if (spectrum[i] > max_value)
        {
            max_value = spectrum[i];
        }
    }

    if (max_value == 0)
    {
        for (i = 0; i < 8; i++)
        {
            norm[i] = 0;
        }

        return;
    }

    for (i = 0; i < 8; i++)
    {
        norm[i] = ((uint32_t)spectrum[i] * 1000) / max_value;
    }
}

void print_float_simple(float value)
{
    int16_t whole;
    uint16_t frac;

    if (value < 0)
    {
        uart_putc('-');
        value = -value;
    }

    whole = (int16_t)value;
    frac = (uint16_t)((value - whole) * 10.0f);

    uart_print_uint16(whole);
    uart_putc('.');
    uart_print_uint16(frac);
}

void print_lab(float L, float a, float b)
{
    uart_print("Lab: L=");
    print_float_simple(L);

    uart_print(" a=");
    print_float_simple(a);

    uart_print(" b=");
    print_float_simple(b);

    uart_print("\r\n");
}

void buttons_init(void)
{
    /* S1 = P1.1, S2 = P1.2 */
    P1DIR &= ~(BIT1 | BIT2);
    P1REN |=  (BIT1 | BIT2);
    P1OUT |=  (BIT1 | BIT2);
}

bool s1_pressed(void)
{
    if (!(P1IN & BIT1))
    {
        delay_ms(30);

        if (!(P1IN & BIT1))
        {
            while (!(P1IN & BIT1));
            delay_ms(30);
            return true;
        }
    }

    return false;
}

bool s2_pressed(void)
{
    if (!(P1IN & BIT2))
    {
        delay_ms(30);

        if (!(P1IN & BIT2))
        {
            while (!(P1IN & BIT2));
            delay_ms(30);
            return true;
        }
    }

    return false;
}

void spectrum_to_xyz(float refl[8], float *x_val, float *y_val, float *z_val)
{
    const float xw[8] = {0.014f, 0.134f, 0.095f, 0.032f, 0.433f, 0.916f, 0.631f, 0.165f};
    const float yw[8] = {0.000f, 0.004f, 0.139f, 0.503f, 0.995f, 0.757f, 0.265f, 0.017f};
    const float zw[8] = {0.067f, 0.645f, 0.813f, 0.158f, 0.004f, 0.000f, 0.000f, 0.000f};

    uint8_t i;
    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;

    for (i = 0; i < 8; i++)
    {
        sx += refl[i] * xw[i];
        sy += refl[i] * yw[i];
        sz += refl[i] * zw[i];
    }

    *x_val = sx * 37.31f;
    *y_val = sy * 37.31f;
    *z_val = sz * 37.31f;
}

float cube_root_approx(float t)
{
    float x = t;
    uint8_t i;

    if (x <= 0.0f)
        return 0.0f;

    if (x < 0.01f)
        x = 0.01f;

    for (i = 0; i < 8; i++)
        x = (2.0f * x + t / (x * x)) / 3.0f;

    return x;
}

float lab_f(float t)
{
    if (t > 0.008856f)
        return cube_root_approx(t);
    else
        return 7.787f * t + 16.0f / 116.0f;
}

void xyz_to_lab(float x_val, float y_val, float z_val, float *L, float *a, float *b)
{
    float Xn = 95.047f;
    float Yn = 100.000f;
    float Zn = 108.883f;

    float fx = lab_f(x_val / Xn);
    float fy = lab_f(y_val / Yn);
    float fz = lab_f(z_val / Zn);

    *L = 116.0f * fy - 16.0f;
    *a = 500.0f * (fx - fy);
    *b = 200.0f * (fy - fz);

    if (*L < 0.0f) *L = 0.0f;
    if (*L > 100.0f) *L = 100.0f;
}

void save_lab_calibration_point(uint8_t index, float L, float a, float b)
{
    raw_lab_cal[index][0] = L;
    raw_lab_cal[index][1] = a;
    raw_lab_cal[index][2] = b;
}

void compute_lab_correction(void)
{
    uint8_t i, r, c, ch;
    float features[CAL_POINTS][LAB_FEATURES];

    for (i = 0; i < CAL_POINTS; i++)
    {
        features[i][0] = 1.0f;
        features[i][1] = raw_lab_cal[i][0];
        features[i][2] = raw_lab_cal[i][1];
        features[i][3] = raw_lab_cal[i][2];
    }

    for (ch = 0; ch < 3; ch++)
    {
        float A[LAB_FEATURES][LAB_FEATURES] = {0};
        float y[LAB_FEATURES] = {0};
        float x[LAB_FEATURES] = {0};

        for (i = 0; i < CAL_POINTS; i++)
        {
            for (r = 0; r < LAB_FEATURES; r++)
            {
                y[r] += features[i][r] * target_lab[i][ch];

                for (c = 0; c < LAB_FEATURES; c++)
                {
                    A[r][c] += features[i][r] * features[i][c];
                }
            }
        }

        if (!solve_4x4(A, y, x))
        {
            uart_print("Lab correction error\r\n");
            return;
        }

        for (c = 0; c < LAB_FEATURES; c++)
        {
            lab_corr[ch][c] = x[c];
        }
    }

    uart_print("Lab correction OK\r\n");
}

void apply_lab_correction(float L, float a, float b, float *Lc, float *ac, float *bc)
{
    float f[LAB_FEATURES];

    f[0] = 1.0f;
    f[1] = L;
    f[2] = a;
    f[3] = b;

    *Lc = lab_corr[0][0]*f[0] + lab_corr[0][1]*f[1] + lab_corr[0][2]*f[2] + lab_corr[0][3]*f[3];
    *ac = lab_corr[1][0]*f[0] + lab_corr[1][1]*f[1] + lab_corr[1][2]*f[2] + lab_corr[1][3]*f[3];
    *bc = lab_corr[2][0]*f[0] + lab_corr[2][1]*f[1] + lab_corr[2][2]*f[2] + lab_corr[2][3]*f[3];

    if (*Lc < 0.0f) *Lc = 0.0f;
    if (*Lc > 100.0f) *Lc = 100.0f;
}

bool solve_4x4(float A[LAB_FEATURES][LAB_FEATURES], float y[LAB_FEATURES], float x[LAB_FEATURES])
{
    uint8_t i, j, k, max_row;
    float max_val, temp, factor;

    for (i = 0; i < LAB_FEATURES; i++)
    {
        max_row = i;
        max_val = A[i][i];
        if (max_val < 0.0f) max_val = -max_val;

        for (k = i + 1; k < LAB_FEATURES; k++)
        {
            float v = A[k][i];
            if (v < 0.0f) v = -v;

            if (v > max_val)
            {
                max_val = v;
                max_row = k;
            }
        }

        if (max_val < 0.000001f)
            return false;

        if (max_row != i)
        {
            for (j = 0; j < LAB_FEATURES; j++)
            {
                temp = A[i][j];
                A[i][j] = A[max_row][j];
                A[max_row][j] = temp;
            }

            temp = y[i];
            y[i] = y[max_row];
            y[max_row] = temp;
        }

        for (k = i + 1; k < LAB_FEATURES; k++)
        {
            factor = A[k][i] / A[i][i];

            for (j = i; j < LAB_FEATURES; j++)
                A[k][j] -= factor * A[i][j];

            y[k] -= factor * y[i];
        }
    }

    for (i = LAB_FEATURES; i > 0; i--)
    {
        uint8_t row = i - 1;
        float sum = y[row];

        for (j = row + 1; j < LAB_FEATURES; j++)
            sum -= A[row][j] * x[j];

        x[row] = sum / A[row][row];
    }

    return true;
}

void print_calibration_prompt(void)
{
    switch (cal_state)
    {
        case CAL_9005:
            lcd_scroll_text_repeat("LAB MODE", 300, 1);
            lcd_scroll_text_repeat("CALIBRATION NEEDED", 300, 1);
            lcd_start_scroll("PLACE RAL 9005 AND PRESS S1",15);
            uart_print("Place RAL 9005 and press S1\r\n");
            break;

        case CAL_9016:
            lcd_start_scroll("PLACE RAL 9016 AND PRESS S1",15);
            uart_print("Place RAL 9016 and press S1\r\n");
            break;

        case CAL_3020:
            lcd_start_scroll("PLACE RAL 3020 AND PRESS S1",15);
            uart_print("Place RAL 3020 and press S1\r\n");
            break;

        case CAL_6018:
            lcd_start_scroll("PLACE RAL 6018 AND PRESS S1",15);
            uart_print("Place RAL 6018 and press S1\r\n");
            break;

        case CAL_5002:
            lcd_start_scroll("PLACE RAL 5002 AND PRESS S1",15);
            uart_print("Place RAL 5002 and press S1\r\n");
            break;

        default:
            break;
    }
}

void spectrum_to_xyz_from_norm(uint16_t norm[8], float *x_val, float *y_val, float *z_val)
{
    float refl[8];
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        refl[i] = (float)norm[i] / 1000.0f;
    }

    spectrum_to_xyz(refl, x_val, y_val, z_val);
}

uint16_t find_nearest_ral_by_spectrum(uint16_t norm[8], uint32_t *best_distance)
{
    uint16_t i, j;
    uint16_t best_ral = 0;

    *best_distance = 0xFFFFFFFF;

    for (i = 0; i < RAL_COUNT; i++)
    {
        uint32_t distance = 0;

        for (j = 0; j < 8; j++)
        {
            int32_t diff = (int32_t)norm[j] - (int32_t)ral_database[i].n[j];
            distance += (uint32_t)(diff * diff);
        }

        if (distance < *best_distance)
        {
            *best_distance = distance;
            best_ral = ral_database[i].ral_number;
        }
    }

    return best_ral;
}

void print_normalized_spectrum(uint16_t norm[8])
{
    uart_print("N: ");
    uart_print_uint16(norm[0]); uart_print(",");
    uart_print_uint16(norm[1]); uart_print(",");
    uart_print_uint16(norm[2]); uart_print(",");
    uart_print_uint16(norm[3]); uart_print(",");
    uart_print_uint16(norm[4]); uart_print(",");
    uart_print_uint16(norm[5]); uart_print(",");
    uart_print_uint16(norm[6]); uart_print(",");
    uart_print_uint16(norm[7]);
    uart_print("\r\n");
}

void uart_print_uint32(uint32_t value)
{
    char buffer[11];
    int i = 0;

    if (value == 0)
    {
        uart_putc('0');
        return;
    }

    while (value > 0)
    {
        buffer[i++] = (value % 10) + '0';
        value /= 10;
    }

    while (i > 0)
    {
        uart_putc(buffer[--i]);
    }
}

void export_ral_entry(uint16_t norm[8])
{
    uart_print("\r\nCOPY THIS TO ral_database[]:\r\n");

    uart_print("{\"RAL XXXX\", {");

    uart_print_uint16(norm[0]); uart_print(",");
    uart_print_uint16(norm[1]); uart_print(",");
    uart_print_uint16(norm[2]); uart_print(",");
    uart_print_uint16(norm[3]); uart_print(",");
    uart_print_uint16(norm[4]); uart_print(",");
    uart_print_uint16(norm[5]); uart_print(",");
    uart_print_uint16(norm[6]); uart_print(",");
    uart_print_uint16(norm[7]);

    uart_print("}},\r\n\r\n");
}

void lcd_display_text(const char *txt)
{
    uint8_t i;

    for (i = 0; i < 6; i++)
    {
        if (txt[i] != '\0')
            myLCD_showChar(txt[i], i + 1);
        else
            myLCD_showChar(' ', i + 1);
    }
}

void lcd_scroll_text(const char *text, uint16_t scroll_delay_ms)
{
    char window[7];
    uint16_t len;
    uint16_t pos;
    uint8_t i;

    len = strlen(text);

    for (pos = 0; pos < len + 12; pos++)
    {
        for (i = 0; i < 6; i++)
        {
            int16_t idx = (int16_t)pos + i - 6;

            if (idx < 0 || idx >= len)
                window[i] = ' ';
            else
                window[i] = text[idx];
        }

        window[6] = '\0';

        lcd_display_text(window);
        delay_ms(scroll_delay_ms);
    }

    lcd_display_text("      ");
}

void lcd_show_ral(uint16_t ral_number)
{
    char msg[16];

    msg[0] = 'R';
    msg[1] = 'A';
    msg[2] = 'L';
    msg[3] = ' ';
    msg[4] = (ral_number / 1000) % 10 + '0';
    msg[5] = (ral_number / 100)  % 10 + '0';
    msg[6] = (ral_number / 10)   % 10 + '0';
    msg[7] = ral_number % 10 + '0';
    msg[8] = '\0';

    lcd_scroll_text_repeat(msg, 300, 1);
}

void lcd_show_lab(float L, float a, float b)
{
    char msg[64];
    uint8_t p = 0;

    msg[p++] = 'L';
    msg[p++] = '=';
    lcd_add_lab_value(msg, &p, L);

    msg[p++] = ' ';
    msg[p++] = 'A';
    msg[p++] = '=';
    lcd_add_lab_value(msg, &p, a);

    msg[p++] = ' ';
    msg[p++] = 'B';
    msg[p++] = '=';
    lcd_add_lab_value(msg, &p, b);

    msg[p] = '\0';

    lcd_scroll_text_repeat(msg, 250, 1);
}

void lcd_scroll_text_repeat(const char *text, uint16_t scroll_delay_ms, uint8_t repeat_count)
{
    uint8_t r;

    for (r = 0; r < repeat_count; r++)
    {
        lcd_scroll_text(text, scroll_delay_ms);
        if(s1_pressed() || s2_pressed())
        {
            return;
        }
        delay_ms(10);
    }
}

bool both_buttons_pressed(void)
{
    if (!(P1IN & BIT1) && !(P1IN & BIT2))
    {
        delay_ms(100);

        if (!(P1IN & BIT1) && !(P1IN & BIT2))
        {
            delay_ms(1500);

            if (!(P1IN & BIT1) && !(P1IN & BIT2))
            {
                while (!(P1IN & BIT1) || !(P1IN & BIT2));

                return true;
            }
        }
    }

    return false;
}

void lcd_start_scroll(const char *text, uint16_t speed)
{
    lcd_scroll_msg = text;
    lcd_scroll_pos = 0;
    lcd_scroll_counter = 0;
    lcd_scroll_speed = speed;
    lcd_scroll_active = true;
}

void lcd_stop_scroll(void)
{
    lcd_scroll_active = false;
}

void lcd_scroll_update(void)
{
    char window[7];
    uint8_t i;
    uint16_t len;

    if (!lcd_scroll_active || lcd_scroll_msg == 0)
        return;

    lcd_scroll_counter++;

    if (lcd_scroll_counter < lcd_scroll_speed)
        return;

    lcd_scroll_counter = 0;

    len = strlen(lcd_scroll_msg);

    for (i = 0; i < 6; i++)
    {
        int16_t idx = (int16_t)lcd_scroll_pos + i - 6;

        if (idx < 0 || idx >= len)
            window[i] = ' ';
        else
            window[i] = lcd_scroll_msg[idx];
    }

    window[6] = '\0';

    lcd_display_text(window);

    lcd_scroll_pos++;

    if (lcd_scroll_pos >= len + 12)
        lcd_scroll_pos = 0;
}

uint8_t get_button_event(void)
{
    uint8_t event = lcd_button_event;

    lcd_button_event = 0;

    if (event != 0)
        return event;

    if (s1_pressed())
        return 1;

    if (s2_pressed())
        return 2;

    return 0;
}

bool menu_combo_pressed(void)
{
    uint16_t t;

    if (!(P1IN & BIT1))          // първо е натиснат S1
    {
        for (t = 0; t < 15; t++) // 30 x 10ms = 300ms
        {
            if (!(P1IN & BIT2))
            {
                while (!(P1IN & BIT1) || !(P1IN & BIT2));
                delay_ms(50);
                return true;
            }
            delay_ms(10);
        }
    }

    if (!(P1IN & BIT2))          // първо е натиснат S2
    {
        for (t = 0; t < 15; t++)
        {
            if (!(P1IN & BIT1))
            {
                while (!(P1IN & BIT1) || !(P1IN & BIT2));
                delay_ms(50);
                return true;
            }
            delay_ms(10);
        }
    }

    return false;
}

void lcd_add_lab_value(char *msg, uint8_t *p, float value)
{
    int16_t v;
    uint16_t abs_v;

    v = (int16_t)(value * 10.0f + ((value >= 0.0f) ? 0.5f : -0.5f));

    if (v < 0)
    {
        msg[(*p)++] = '-';
        abs_v = (uint16_t)(-v);
    }
    else
    {
        abs_v = (uint16_t)v;
    }

    if (abs_v >= 1000)
    {
        msg[(*p)++] = (abs_v / 1000) % 10 + '0';
    }

    if (abs_v >= 100)
    {
        msg[(*p)++] = (abs_v / 100) % 10 + '0';
    }

    msg[(*p)++] = (abs_v / 10) % 10 + '0';
    msg[(*p)++] = '.';
    msg[(*p)++] = abs_v % 10 + '0';
}
