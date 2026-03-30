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

#define MASTER_BMS        // Bo mach Man hinh (Nhan so lieu pin, gui lenh sac)
// #define SLAVE_BMS      // Bo mach quan ly Pin (Doc cam bien, nhan lenh sac)

#define CAN_ID_BMS_OVERVIEW   0x100  // SOC, Dong, Ap tong, Nhiet do
#define CAN_ID_BMS_CELLS      0x101  // Ap 4 Cell
#define CAN_ID_CMD_CHARGE     0x200  // Lenh cai dat dong sac/Cat relay


#if defined(SLAVE_BMS)
    // Co 2 hom thu di (TX) va 1 hom thu den (RX)
    #define MB_TX_OVERVIEW    0
    #define MB_TX_CELLS       1
    #define MB_RX_CMD         2

#elif defined(MASTER_BMS)
    // Co 2 hom thu den (RX) va 1 hom thu di (TX)
    #define MB_RX_OVERVIEW    0
    #define MB_RX_CELLS       1
    #define MB_TX_CMD         2
#endif

#if defined(MASTER_BMS)
    uint8_t  rx_soc = 0;
    uint8_t  rx_temp = 0;
    int16_t  rx_current = 0;
    uint16_t rx_total_v = 0;
    uint16_t rx_cell_v[4] = {0};
#endif

/* Bien chua du lieu nhan ve (CAN) */
flexcan_msgbuff_t rx_msg_buffer;

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
int32_t Remaining_Capacity_mAs = 0; // Dung luong Pin thuc te con lai.
uint8_t SOC_Percent = 0;            // Phan tram pin.
bool is_Charging = false;           // Co bao hieu xem pin dang duoc cam sac (true) hay dang xa (false).

/* --- BIEN DO DIEN AP (VOLTAGE) --- */
uint16_t ADC_Raw[4] = {0};      // Du lieu tho do duoc tu ADC.
uint16_t Node_Voltage[4] = {0}; // Dien ap tai cac Node tren mach chia ap.
uint16_t Cell_Voltage[4] = {0}; // Dien ap thuc te cua tung Cell pin.

/* --- BIEN DO NHIET DO --- */
float Temperature_C = 0;        // Nhiet do thuc te cua khoi pin (do C).

/* --- BIEN DIEU KHIEN SAC --- */
int16_t Target_Charge_Current_mA = 0; // Dong dien sac mong muon.

/* --- BIEN THOI GIAN (TIMER) --- */
volatile bool Flag_100ms = false; // Co thong bao da het 100ms.

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
void Send_BMS_Data_CAN(void);

/* ============================================================================
 * 4. HAM PHUC VU NGAT TIMER (CHAY NGAM)
 * ============================================================================ */
void LPIT0_Ch0_IRQHandler(void)
{
    // Cu 0.1s (100ms) chay vong lap main 1 lan.
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << 0));
    Flag_100ms = true;
}


/* ============================================================================
 * 5. CHI TIET CAC HAM XU LY CHUC NANG CUA BMS
 * ============================================================================ */

/* --- HAM 1: DO DIEN AP TUNG CUC PIN --- */
void Read_Battery_Voltage(void)
{
    // BUOC 1: Do ADC tung kenh
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[0]);

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

   // Kiem tra dut giay hoac doan mach.
    if(adc_raw_temp == 0 || adc_raw_temp >= 4095) return;

    // Tinh dien tro thuc te cua cam bien NTC la bao nhieu Ohm.
    float R_NTC = SERIES_RESISTOR / ((4095.0f / (float)adc_raw_temp) - 1.0f);

    // Ap dung Phuong trinh Steinhart-Hart de quy doi Dien tro ra Nhiet do do C.
    float temp_K = R_NTC / NOMINAL_RESISTANCE;
    temp_K = log(temp_K);                          // Ham log nay la Logarit tu nhien (ln)
    temp_K /= B_COEFFICIENT;
    temp_K += 1.0f / NOMINAL_TEMPERATURE;
    temp_K = 1.0f / temp_K;                        // Ket qua ra do Kelvin

    Temperature_C = temp_K - 273.15f;              // Chuyen Kelvin sang do C.
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

    // 1. Duoi 4.1V: Sac binh thuong (Dong cao)
    if (max_cell_voltage < 4100) {
        Target_Charge_Current_mA = 3000;
        PINS_DRV_WritePin(PTD, 0, 1); // 1 = Xuat dien 5V ra chan PTD0 de mo MOSFET.
    }
    // 2. Tu 4.1V den 4.2V: Sac cham (Ep dong nho lai)
    else if (max_cell_voltage >= 4100 && max_cell_voltage < 4200) {
        Target_Charge_Current_mA = 500;
        PINS_DRV_WritePin(PTD, 0, 1);
    }
    // 3. Tren 4.2V: Ngat sac bao ve pin
    else {
        Target_Charge_Current_mA = 0;
        PINS_DRV_WritePin(PTD, 0, 0); // 0 = Dap chan PTD0 ve 0V de dong MOSFET.
    }
}

/* --- HAM 4: DO DONG DIEN BANG GIAO TIEP I2C --- */
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
            // Ghep 2 hop du lieu (moi hop 8-bit) thanh 1 con so nguyen 16-bit co dau.
            Shunt_Raw = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
            Current_mA = (int32_t)(Shunt_Raw * 1.666f);
        }
    }
    else {
        Current_mA = 999; // Neu ket noi I2C bi loi, gan dong dien bang so 999 de nhan biet loi.
    }
}

/* --- HAM 5: TINH % PIN (THUAT TOAN DEM COULOMB) --- */
void Calculate_SOC(void)
{
    // Xac dinh xem dang sac hay xa
    if (Current_mA > 0) { is_Charging = true; }
    else { is_Charging = false; }

    // THUAT TOAN COT LOI (Coulomb Counting):
    // Vi ham nay cu 100ms (0.1 giay) chay 1 lan. Tuc la 1 giay no chay 10 lan.
    // Nen ta chia dong dien cho 10 de ra luong dien thuc su da chay qua trong 0.1 giay do.
    int32_t delta_Charge_mAs = Current_mA / 10;

    // Cong/tru luong dien vao dung luong pin tong
    Remaining_Capacity_mAs = Remaining_Capacity_mAs + delta_Charge_mAs;

    // Gioi han tren va duoi
    if (Remaining_Capacity_mAs > BATTERY_CAPACITY_MAS) {
        Remaining_Capacity_mAs = BATTERY_CAPACITY_MAS;
    }
    else if (Remaining_Capacity_mAs < 0) {
        Remaining_Capacity_mAs = 0;
    }

    // Tinh % Pin
    SOC_Percent = (uint8_t)(Remaining_Capacity_mAs / (BATTERY_CAPACITY_MAS / 100));
}

/* --- HAM 6: HIEN THI LEN MAN HINH MAY TINH QUA UART --- */
void Print_BMS_Data(void)
{
    // %3d nghia la danh san 3 khoang trang de in so, giup cac dong khi in ra thang cot voi nhau.
    sprintf(uart_buffer, "SOC: %3d %% | TEMP: %2d C | DONG: %5ld mA | AP TONG: %5d mV | C1:%4d | C2:%4d | C3:%4d | C4:%4d \r\n",
            SOC_Percent,
            (int)Temperature_C,  // Ep kieu ve so nguyen (int) vut bo phan thap phan de in cho gon.
            Current_mA,
            Node_Voltage[3],
            Cell_Voltage[0], Cell_Voltage[1], Cell_Voltage[2], Cell_Voltage[3]);

    LPUART_DRV_SendDataBlocking(INST_LPUART1, (uint8_t *)uart_buffer, strlen(uart_buffer), 100);
}

/* --- HAM 7: GUI DU LIEU BMS LEN MANG CAN (CHI DANH CHO SLAVE) --- */
#if defined(SLAVE_BMS)
void Send_BMS_Data_CAN(void)
{
    flexcan_data_info_t tx_info = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8,
        .fd_enable = false, .fd_padding = 0, .enable_brs = false, .is_remote = false
    };

    uint8_t tx_data_1[8] = {0};
    uint8_t tx_data_2[8] = {0};

    // GOI SO 1 (Tong quan)
    tx_data_1[0] = SOC_Percent;
    tx_data_1[1] = (uint8_t)Temperature_C;

    // Ep dong dien ve 16 bit de tiet kiem duong truyen
    int16_t short_current = (int16_t)Current_mA;
    tx_data_1[2] = (short_current >> 8) & 0xFF;
    tx_data_1[3] = short_current & 0xFF;

    uint16_t Total_V = Node_Voltage[3];
    tx_data_1[4] = (Total_V >> 8) & 0xFF;
    tx_data_1[5] = Total_V & 0xFF;

    // Phat goi 1 len mang CAN
    FLEXCAN_DRV_Send(INST_CANCOM1, MB_TX_OVERVIEW, &tx_info, CAN_ID_BMS_OVERVIEW, tx_data_1);

    // GOI SO 2 (Ap 4 Cell)
    tx_data_2[0] = (Cell_Voltage[0] >> 8) & 0xFF;
    tx_data_2[1] = Cell_Voltage[0] & 0xFF;
    tx_data_2[2] = (Cell_Voltage[1] >> 8) & 0xFF;
    tx_data_2[3] = Cell_Voltage[1] & 0xFF;
    tx_data_2[4] = (Cell_Voltage[2] >> 8) & 0xFF;
    tx_data_2[5] = Cell_Voltage[2] & 0xFF;
    tx_data_2[6] = (Cell_Voltage[3] >> 8) & 0xFF;
    tx_data_2[7] = Cell_Voltage[3] & 0xFF;

    // Phat goi 2 len mang CAN
    FLEXCAN_DRV_Send(INST_CANCOM1, MB_TX_CELLS, &tx_info, CAN_ID_BMS_CELLS, tx_data_2);
}
#endif

/* ============================================================================
 * 6. Main
 * ============================================================================ */
int main(void)
{
    int exit_code = 0; // Bien do phan mem yeu cau
    bool is_First_Read = true; // Co hieu de khoa lenh doan % pin sau khi chay lan dau tien.

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    /* --- BUOC KHOI TAO PHAN CUNG --- */

    // 1. Cau hinh Clock va chan cam (Muxing)
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    // 2. Cau hinh Timer (Bao thuc 100ms)
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    LPIT_DRV_InitChannel(INST_LPIT1, 0, &lpit1_ChnConfig0);
    INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, LPIT0_Ch0_IRQHandler, (isr_t*) 0);
    INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << 0));

    // 3. Khoi tao khoi do Von (ADC)
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);

    // 4. Khoi tao khoi giao tiep I2C
    LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1State);

    // 5. Khoi tao cong truyen UART
    LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);

    // 6. Khoi tao giao tiep CAN Bus
    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);

    /* --- CAU HINH HOM THU NHAN CAN THEO VAI TRO BO MACH --- */
    flexcan_data_info_t rx_info = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = 8,
        .fd_enable = false,
        .fd_padding = 0,
        .enable_brs = false,
        .is_remote = false
    };

    #if defined(SLAVE_BMS)
        // Mach BMS chi can mo cua 1 hom thu de cho nhan Lenh Dieu Khien (0x200) tu Man Hinh
        FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, MB_RX_CMD, &rx_info, CAN_ID_CMD_CHARGE);
        FLEXCAN_DRV_Receive(INST_CANCOM1, MB_RX_CMD, &rx_msg_buffer);
    #endif

    #if defined(MASTER_BMS)
        // Mach Man hinh mo cua 2 hom thu de hung du lieu tu BMS (0x100 va 0x101)
        FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, MB_RX_OVERVIEW, &rx_info, CAN_ID_BMS_OVERVIEW);
        FLEXCAN_DRV_Receive(INST_CANCOM1, MB_RX_OVERVIEW, &rx_msg_buffer);

        FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, MB_RX_CELLS, &rx_info, CAN_ID_BMS_CELLS);
        FLEXCAN_DRV_Receive(INST_CANCOM1, MB_RX_CELLS, &rx_msg_buffer);
    #endif

    /* =========================================
     * VONG LAP CHINH
     * ========================================= */
    for(;;)
    {
        /* --- 1. LOGIC NHAN LENH / DU LIEU LIEN TUC KHONG CHO TIMER --- */

        #if defined(SLAVE_BMS)
            // Lang nghe lenh cai dong sac tu Man hinh gui xuong
            if (FLEXCAN_DRV_GetTransferStatus(INST_CANCOM1, MB_RX_CMD) == STATUS_SUCCESS) {
                Target_Charge_Current_mA = (rx_msg_buffer.data[0] * 100);
                // Doc xong phai mo lai hom thu de don lenh moi
                FLEXCAN_DRV_Receive(INST_CANCOM1, MB_RX_CMD, &rx_msg_buffer);
            }
        #endif

        #if defined(MASTER_BMS)
            // Lang nghe goi Tong quan (ID: 0x100)
            if (FLEXCAN_DRV_GetTransferStatus(INST_CANCOM1, MB_RX_OVERVIEW) == STATUS_SUCCESS) {
                rx_soc     = rx_msg_buffer.data[0];
                rx_temp    = rx_msg_buffer.data[1];
                rx_current = (int16_t)((rx_msg_buffer.data[2] << 8) | rx_msg_buffer.data[3]);
                rx_total_v = (uint16_t)((rx_msg_buffer.data[4] << 8) | rx_msg_buffer.data[5]);

                FLEXCAN_DRV_Receive(INST_CANCOM1, MB_RX_OVERVIEW, &rx_msg_buffer);
            }

            // Lang nghe goi Ap 4 Cell (ID: 0x101)
            if (FLEXCAN_DRV_GetTransferStatus(INST_CANCOM1, MB_RX_CELLS) == STATUS_SUCCESS) {
                rx_cell_v[0] = (uint16_t)((rx_msg_buffer.data[0] << 8) | rx_msg_buffer.data[1]);
                rx_cell_v[1] = (uint16_t)((rx_msg_buffer.data[2] << 8) | rx_msg_buffer.data[3]);
                rx_cell_v[2] = (uint16_t)((rx_msg_buffer.data[4] << 8) | rx_msg_buffer.data[5]);
                rx_cell_v[3] = (uint16_t)((rx_msg_buffer.data[6] << 8) | rx_msg_buffer.data[7]);

                FLEXCAN_DRV_Receive(INST_CANCOM1, MB_RX_CELLS, &rx_msg_buffer);
            }
        #endif

        /* --- 2. LOGIC XU LY DINH KY MOI 100ms --- */
        if (Flag_100ms == true)
        {
            Flag_100ms = false;     // Ha co xuong ngay lap tuc

            #if defined(SLAVE_BMS)
                // --- SLAVE GOI CAC HAM XU LY DO DAC ---
                Read_Battery_Voltage();
                Read_Battery_Temperature();

                /* Doan % pin lan dau */
                if (is_First_Read == true) {
                    uint16_t Total_V = Node_Voltage[3];
                    if (Total_V >= 16800) { SOC_Percent = 100; }
                    else if (Total_V <= 12000) { SOC_Percent = 0; }
                    else {
                        SOC_Percent = (uint8_t)(((Total_V - 12000) * 100) / (16800 - 12000));
                    }
                    Remaining_Capacity_mAs = (BATTERY_CAPACITY_MAS / 100) * SOC_Percent;
                    is_First_Read = false;
                }

                Read_Battery_Current();
                Calculate_SOC();
                Manage_Charging();

                // --- DIEU TOC VIEC GUI DU LIEU (1 giay / 1 lan) ---
                uart_counter++;
                if (uart_counter >= 10)
                {
                    Print_BMS_Data();    // In ra PuTTY
                    Send_BMS_Data_CAN(); // Ban 2 goi CAN len mang cho Master doc
                    uart_counter = 0;
                }
            #endif

            #if defined(MASTER_BMS)
                // O day ban co the bo sung doan code de in cac bien rx_... ra LCD
            #endif
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
