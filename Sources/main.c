#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "lpit1.h"
#include "adConv1.h"
#include "lpi2c1.h"
#include "canCom1.h"
#include "interrupt_manager.h"
#include "lpuart1.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 1. KHAI BAO CAC HANG SO
 * ============================================================================ */

/* --- THONG SO CAM BIEN DONG DIEN (INA226) --- */
#define INA226_ADDR 0x40        // Dia chi I2C mac dinh cua chip INA226 tren duong truyen
#define SHUNT_REG   0x01        // Dia chi thanh ghi ben trong INA226 chua du lieu dien ap roi tren dien tro Shunt

/* --- THONG SO HE THONG PIN (Dung de tinh %) --- */
#define BATTERY_CAPACITY_MAH 8000
#define BATTERY_CAPACITY_MAS (BATTERY_CAPACITY_MAH * 3600)

/* --- THONG SO CAM BIEN NHIET DO (NTC 10K) --- */
#define SERIES_RESISTOR 10000.0f    // Dien tro 10k Ohm mac noi tiep voi cam bien NTC tren mach chia ap
#define NOMINAL_RESISTANCE 10000.0f // Cam bien NTC nay co gia tri chuan la 10k Ohm o nhiet do 25 do C
#define NOMINAL_TEMPERATURE 298.15f // Nhiet do 25 do C doi sang thang do Kelvin (25 + 273.15)
#define B_COEFFICIENT 3950.0f


/* ============================================================================
 * 2. KHAI BAO CAC BIEN TOAN CUC
 * ============================================================================ */

/* --- BIEN GIAO TIEP MAY TINH (UART) --- */
char uart_buffer[150];
uint8_t uart_counter = 0;

/* --- BIEN DO DONG DIEN --- */
int16_t Shunt_Raw = 0;      // Con so tho (chua tinh toan) doc ve tu mat cam bien INA226.
int32_t Current_mA = 0;     // Dong dien thuc te dang chay ra/vao pin (don vi mili-Ampe).

/* --- BIEN DO DUNG LUONG (SOC - State of Charge) --- */
int32_t Remaining_Capacity_mAs = 0; // Dung luong Pin.
uint8_t SOC_Percent = 0;            // Phan tram pin .
bool is_Charging = false;           // Co bao hieu xem pin dang duoc cam sac (true) hay dang xa (false).

/* --- BIEN DO DIEN AP (VOLTAGE) --- */
uint16_t ADC_Raw[4] = {0};      // Du lieu tho .
uint16_t Node_Voltage[4] = {0}; // Dien ap tai cac Node .
uint16_t Cell_Voltage[4] = {0}; // Dien ap tai cac Cell pin .

/* --- BIEN DO NHIET DO --- */
float Temperature_C = 0;        // Nhiet do thuc te cua khoi pin (do C).

/* --- BIEN DIEU KHIEN SAC --- */
int16_t Target_Charge_Current_mA = 0; // Dong dien sac mong muon .

/* --- BIEN THOI GIAN (TIMER) --- */
volatile bool Flag_100ms = false; // Co thong bao den thoi gian do .

/* --- BIEN TRANG THAI HE THONG --- */
lpi2c_master_state_t lpi2c1State; // Cau truc luu trang thai cua bo truyen du lieu I2C.


/* ============================================================================
 * 3. KHAI BAO TEN CAC HAM (PROTOTYPES)
 * ============================================================================ */
void Read_Battery_Voltage(void);
void Read_Battery_Current(void);
void Read_Battery_Temperature(void);
void Calculate_SOC(void);
void Manage_Charging(void);
void Print_BMS_Data(void);


/* ============================================================================
 * 4. HAM PHUC VU NGAT TIMER (CHAY NGAM)
 * ============================================================================ */
void LPIT0_Ch0_IRQHandler(void)
{
    // Cu 0.1s chay vong lap main 1 lan .
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << 0));
    Flag_100ms = true;
}


/* ============================================================================
 * 5. CHI TIET CAC HAM XU LY CHUC NANG CUA BMS
 * ============================================================================ */

/* --- HAM 1: DO DIEN AP TUNG CUC PIN --- */
void Read_Battery_Voltage(void)
{

    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0); // Mo kenh 0
    ADC_DRV_WaitConvDone(INST_ADCONV1);                       // Doi chip do xong
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[0]);      // Cat so lieu vao mang ADC_Raw

    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig1);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[1]);

    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig2);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[2]);

    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig3);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[3]);

    // BUOC 2: Dich so lieu tho (0-4095) ra so Von (mV)( 12 bit)
    // Chip S32K144 dung dien 5V nen moc toi da la 5000mV.
    Node_Voltage[0] = (ADC_Raw[0] * 5000) / 4095;
    Node_Voltage[1] = ((ADC_Raw[1] * 5000) / 4095) * 2;
    Node_Voltage[2] = ((ADC_Raw[2] * 5000) / 4095) * 3;
    Node_Voltage[3] = ((ADC_Raw[3] * 5000) / 4095) * 4;

    // BUOC 3: Tim dien ap cua tung cuc pin le (Phep toan tru don)
    Cell_Voltage[0] = Node_Voltage[0];
    Cell_Voltage[1] = Node_Voltage[1] - Node_Voltage[0];
    Cell_Voltage[2] = Node_Voltage[2] - Node_Voltage[1];
    Cell_Voltage[3] = Node_Voltage[3] - Node_Voltage[2];
}

/* --- HAM 2: DO NHIET DO KHOI PIN --- */
void Read_Battery_Temperature(void)
{
    uint16_t adc_raw_temp = 0; // Bien tam de chua so lieu tho do tu cam bien nhiet

    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig4);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &adc_raw_temp);

   // Kiem tra dut giay hoac doan mach .
    if(adc_raw_temp == 0 || adc_raw_temp >= 4095) return;

    // Tinh dien tro cua 10K là bao nhieu Om .
    float R_NTC = SERIES_RESISTOR / ((4095.0f / (float)adc_raw_temp) - 1.0f);

    // Ap dung Phuong trinh Steinhart-Hart (Cong thuc vat ly chuyen dung de dich Dien tro ra Nhiet do cho NTC).
    float temp_K = R_NTC / NOMINAL_RESISTANCE;
    temp_K = log(temp_K);                          // Ham log nay la Logarit tu nhien (ln)
    temp_K /= B_COEFFICIENT;
    temp_K += 1.0f / NOMINAL_TEMPERATURE;
    temp_K = 1.0f / temp_K;                        // Ket qua ra do Kelvin

    Temperature_C = temp_K - 273.15f;              // Phep tru don gian de chuyen Kelvin sang do C quen thuoc.
}

/* --- HAM 3: BAO VE PIN (CAT SAC NEU QUA AP) --- */
void Manage_Charging(void)
{
    uint16_t max_cell_voltage = 0; // Bien chua dien ap cua cuc pin dang day nhat

    // Dung vong lap tim xem trong 4 cuc, cuc nao co so Von cao nhat
    for (int i = 0; i < 4; i++) {
        if (Cell_Voltage[i] > max_cell_voltage) {
            max_cell_voltage = Cell_Voltage[i];
        }
    }


    // 1. Duoi 4.1V sac binh thuong
    if (max_cell_voltage < 4100) {
        Target_Charge_Current_mA = 3000;
        PINS_DRV_WritePin(PTD, 0, 1); // 1 = Xuat dien 5V ra chan PTD0 (de kich mo khoa con MOSFET tren mach sac).
    }
    // 2. 4.1 den 4.2 sac cham
    else if (max_cell_voltage >= 4100 && max_cell_voltage < 4200) {
        Target_Charge_Current_mA = 500;
        PINS_DRV_WritePin(PTD, 0, 1);
    }
    // 3. ngat sac
    else {
        Target_Charge_Current_mA = 0;
        PINS_DRV_WritePin(PTD, 0, 0); // 0 = Dap chan PTD0 ve 0V (Dong chat khoa MOSFET, cat dut dong sac).
    }
}

/* --- HAM 4: DO DONG DIEN BANG I2C --- */
void Read_Battery_Current(void)
{
    uint8_t reg_addr = SHUNT_REG;
    uint8_t rx_buffer[2] = {0};
    status_t status;

    // Go cua con INA226 (bang dia chi 0x40)
    LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, INA226_ADDR, false);
    // Nhet to giay ghi so tu (0x01) qua khe cua cho no.
    status = LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &reg_addr, 1, true, 100);

    // Neu go cua thanh cong
    if (status == STATUS_SUCCESS)
    {
        // Nhan lai 2 hop du lieu no nem ra.
        status = LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C1, rx_buffer, 2, true, 100);
        if (status == STATUS_SUCCESS)
        {
            // Ghep 2 hop du lieu (moi hop 8-bit) thanh 1 con so nguyen 16-bit.
            Shunt_Raw = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
            Current_mA = (int32_t)(Shunt_Raw * 1.666f);
        }
    }
    else {
        Current_mA = 0; // Neu ket noi I2C bi loi , gan dong dien = 0 cho an toan.
    }
}

/* --- HAM 5: TINH % PIN (THUAT TOAN DEM COULOMB) --- */
void Calculate_SOC(void)
{
    // Xac dinh xem dang sac hay xa
    if (Current_mA > 0) { is_Charging = true; }
    else { is_Charging = false; }

    // THUAT TOAN COT LOI (Coulomb Counting):
    // Vi cai ham nay cu 100ms (0.1 giay) lai chay 1 lan. Tuc la 1 giay no chay 10 lan.
    // Nen ta chia dong dien cho 10 de ra luong dien thuc su da chay qua trong 0.1 giay do.
    int32_t delta_Charge_mAs = Current_mA / 10;

    // Tinh dung luong pin
    Remaining_Capacity_mAs = Remaining_Capacity_mAs + delta_Charge_mAs;

    if (Remaining_Capacity_mAs > BATTERY_CAPACITY_MAS) {
        Remaining_Capacity_mAs = BATTERY_CAPACITY_MAS;
    }
    else if (Remaining_Capacity_mAs < 0) {
        Remaining_Capacity_mAs = 0;
    }

    // Tinh % Pin
    SOC_Percent = (uint8_t)(Remaining_Capacity_mAs / (BATTERY_CAPACITY_MAS / 100));
}

/* --- HAM 6: HIEN THI LEN MAN HINH MAY TINH --- */
void Print_BMS_Data(void)
{

    // %3d nghia la danh san 3 khoang trang de in so, giup cac dong khi in ra thang cot voi nhau trong rat chuyen nghiep.
    sprintf(uart_buffer, "SOC: %3d %% | TEMP: %2d C | DONG: %5ld mA | AP TONG: %5d mV | C1:%4d | C2:%4d | C3:%4d | C4:%4d \r\n",
            SOC_Percent,
            (int)Temperature_C,  // Ep kieu ve so nguyen (int) vut bo phan thap phan de in cho gon.
            Current_mA,
            Node_Voltage[3],
            Cell_Voltage[0], Cell_Voltage[1], Cell_Voltage[2], Cell_Voltage[3]);

    LPUART_DRV_SendDataBlocking(INST_LPUART1, (uint8_t *)uart_buffer, strlen(uart_buffer), 100);
}


/* ============================================================================
 * 6. Main
 * ============================================================================ */
int main(void)
{
    int exit_code = 0; // Bien do phan mem yeu cau (bat buoc phai co de tranh bao loi)
    bool is_First_Read = true; // Co hieu de khoa lenh doan % pin sau khi chay lan dau tien.

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    /* --- BUOC KHOI TAO  --- */

    // 1. Cau hinh Clock (Nhip tim) va chan cam (Muxing)
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    // 2. Cau hinh Timer (Bao thuc 100ms)
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    LPIT_DRV_InitChannel(INST_LPIT1, 0, &lpit1_ChnConfig0);
    INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, LPIT0_Ch0_IRQHandler, (isr_t*) 0); // Kich hoat he thong Ngat
    INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << 0)); // Bat dau dem nguoc!

    // 3. Khoi tao khoi do Von (ADC)
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);

    // 4. Khoi tao khoi giao tiep I2C (De noi chuyen voi con do dong)
//  LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1State); // Van tam khoa de test ap truoc

    // 5. Khoi tao cong truyen UART (De in chu len PuTTY)
    LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);


    /* =========================================
     * VONG LAP CHINH (CHAY VINH VIEN KHONG DUNG)
     * ========================================= */
    for(;;)
    {
        // He thong se dung im o day. Cho den khi bo Timer dem du 100ms va vay co (Flag) len.
        if (Flag_100ms == true)
        {
            Flag_100ms = false;     // Ha co xuong ngay lap tuc de chuan bi cho chu ky sau.

            // --- GOI CAC HAM XU LY THEO THU TU ---

            Read_Battery_Voltage(); // 1. Do ap
            Read_Battery_Temperature(); // 2. Do nhiet do

            /* --- DOAN % PIN BAN DAU DUA VAO DIEN AP (Chi lam dung 1 lan dau) --- */
            if (is_First_Read == true)
            {
                uint16_t Total_V = Node_Voltage[3]; // Lay ap tong vua do duoc.
                if (Total_V >= 16800) { SOC_Percent = 100; }     // Ap day 16.8V -> 100%
                else if (Total_V <= 12000) { SOC_Percent = 0; }  // Ap can 12.0V -> 0%
                else {
                    // Nam o khoang giua thi dung toan hoc duong thang de doan
                    SOC_Percent = (uint8_t)(((Total_V - 12000) * 100) / (16800 - 12000));
                }

                // Gan % doan duoc vao thuat toan Coulomb de no bat dau dem.
                Remaining_Capacity_mAs = (BATTERY_CAPACITY_MAS / 100) * SOC_Percent;

                is_First_Read = false; // Tat co, doan code doan % nay bi khoa vinh vien khong bao gio chay lai nua.
            }
            /* ------------------------------------------------------------------ */

//          Read_Battery_Current(); // 3. Do dong dien (Tam khoa)
            Calculate_SOC();        // 4. Tinh lai % pin dua vao dong dien
            Manage_Charging();      // 5. Kiem tra an toan xem co can ngat sac khong

            /* --- DIEU TOC VIEC IN RA MAN HINH --- */
            uart_counter++; // Tang bien dem len 1. (Moi lan chay mat 0.1 giay)
            if (uart_counter >= 10) // Du 10 lan = 1 giay
            {
                Print_BMS_Data(); // Goi ham in ra man hinh
                uart_counter = 0; // Tra ve 0 de dem lai tu dau
            }
        }
    }

  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
