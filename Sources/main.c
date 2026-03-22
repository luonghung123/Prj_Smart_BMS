#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "lpit1.h"
#include "adConv1.h"
#include "lpi2c1.h"
#include "canCom1.h"
#include "interrupt_manager.h"

/* --- BIEN VA DIA CHI CHO I2C & INA226 (DO DONG DIEN) --- */
#define INA226_ADDR 0x40        // Dia chi I2C mac dinh cua INA226
#define SHUNT_REG   0x01        // Thanh ghi chua dien ap Shunt

/* --- DIEN TRO SHUNT LOAI 50A- 75mV --- */
int16_t Shunt_Raw = 0;          // Gia tri tho tu cam bien
int32_t Current_mA = 0;         // Dong dien thuc te (mili-Ampe)

/* --- BIEN CHO THUAT TOAN DEM COULOMB (TINH % PIN) --- */
#define BATTERY_CAPACITY_MAH 8000 // He pin 4S4P dung luong 8000mAh
#define BATTERY_CAPACITY_MAS (BATTERY_CAPACITY_MAH * 3600)

int32_t Remaining_Capacity_mAs = BATTERY_CAPACITY_MAS; // Mac dinh bat len la 100%
uint8_t SOC_Percent = 100;                             // Phan tram pin
bool is_Charging = false;

/* Cac mang chua du lieu dien ap pin */
uint16_t ADC_Raw[4] = {0};      // Luu gia tri tho 0-4095 tu ADC 12 bit
uint16_t Node_Voltage[4] = {0}; // Dien ap tai cac diem do (sau mach chia ap)
uint16_t Cell_Voltage[4] = {0}; // Dien ap thuc te cua TUNG cell (mV)

/* Bien trang thai toan cuc cho I2C */
lpi2c_master_state_t lpi2c1State;

void Read_Battery_Voltage(void);
int16_t Target_Charge_Current_mA = 0; // Dong sac yeu cau gui cho Tram sac
volatile bool Flag_100ms = false;
void Manage_Charging(void);
void Read_Battery_Current(void);
void Calculate_SOC(void);

void LPIT0_Ch0_IRQHandler(void)
{
    /* Xoa co ngat phan cung cua kenh 0 */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << 0));
    Flag_100ms = true; /* Bao cho vong lap chinh biet da du 100ms */
}

int main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    /* 1. KHOI TAO CLOCK & PIN */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* 2. KHOI TAO LPIT (TIMER 100MS) */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    LPIT_DRV_InitChannel(INST_LPIT1, 0, &lpit1_ChnConfig0);

    /* Cai dat ngat cho LPIT trong he thong NVIC (Cuc ky quan trong) */
    INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, LPIT0_Ch0_IRQHandler, (isr_t*) 0);
    INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);

    /* Khoi dong dem thoi gian */
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << 0));

    /* 3. KHOI TAO ADC0 (DO DIEN AP TUNG CELL PIN) */
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);

    /* 4. KHOI TAO LPI2C (DE DOC INA226) */
    LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1State);

    /* 5. KHOI TAO CAN (DE GIAO TIEP ESP32/TRAM SAC) */
    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);

    /* =========================================
     * VONG LAP CHINH
     * ========================================= */
    for(;;)
    {
        if (Flag_100ms)
        {
            Flag_100ms = false;     // Xoa co
            Read_Battery_Voltage(); // Doc ADC de tinh ra Von cua 4 Cell
            Read_Battery_Current(); // Doc I2C tu INA226 de lay Ampe
            Calculate_SOC();        // Tinh dung luong pin theo %
            Manage_Charging();      // Quan ly sac
        }
    }

    /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
    #ifdef PEX_RTOS_START
      PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
    #endif
    /*** End of RTOS startup code.  ***/
    /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
    return 0;
}

void Read_Battery_Voltage(void)
{
    /* --- 1. KICH HOAT VA DOC ADC LAN LUOT 4 KENH --- */
    // Kenh 0 (Cell 1)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0); // Bat dau do
    ADC_DRV_WaitConvDone(INST_ADCONV1);                       // Cho do xong
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[0]);      // Lay ket qua

    // Kenh 1 (Cell 2)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig1);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[1]);

    // Kenh 2 (Cell 3)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig2);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[2]);

    // Kenh 3 (Cell 4)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig3);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[3]);

    /* --- 2. TINH TOAN DIEN AP TAI CAC CELL --- */
    // Cong thuc: V_Node = (ADC_Raw * 5000 / 4095) * He_so_chia_ap
    Node_Voltage[0] = (ADC_Raw[0] * 5000) / 4095;       //  Dien ap tai Cell 1
    Node_Voltage[1] = ((ADC_Raw[1] * 5000) / 4095) * 2; // Tong Cell 1+2
    Node_Voltage[2] = ((ADC_Raw[2] * 5000) / 4095) * 3; // Tong Cell 1+2+3
    Node_Voltage[3] = ((ADC_Raw[3] * 5000) / 4095) * 4; // Tong 4 Cell

    /* --- 3. THUAT TOAN TRU DON DE TIM DIEN AP TUNG CELL --- */
    Cell_Voltage[0] = Node_Voltage[0];
    Cell_Voltage[1] = Node_Voltage[1] - Node_Voltage[0];
    Cell_Voltage[2] = Node_Voltage[2] - Node_Voltage[1];
    Cell_Voltage[3] = Node_Voltage[3] - Node_Voltage[2];
}

/* Ham dieu tiet dong sac va bao ve */
void Manage_Charging(void)
{
    uint16_t max_cell_voltage = 0;

    // 1. Tim ra Cell co dien ap cao nhat (Cell co nguy co day truoc/bi chai)
    for (int i = 0; i < 4; i++) {
        if (Cell_Voltage[i] > max_cell_voltage) {
            max_cell_voltage = Cell_Voltage[i];
        }
    }

    // 2. LOGIC DIEU TIET DONG SAC THONG MINH
    if (max_cell_voltage < 4100) {
        // Tat ca cac cell duoi 4.1V ; sac binh thuong
        Target_Charge_Current_mA = 3000;
        PINS_DRV_WritePin(PTD, 0, 1); // Dong MOSFET cho phep sac
    }
    else if (max_cell_voltage >= 4100 && max_cell_voltage < 4200) {
        // Bat dau co Cell cham nguong 4.1V -> EP DONG SAC CHAM LAI de cho cac cell khac cung sac
        Target_Charge_Current_mA = 500; // HA DONG XUONG VA SAC CHAM
        PINS_DRV_WritePin(PTD, 0, 1);
    }
    else {
        // Vuot 4.2V
        Target_Charge_Current_mA = 0;
        PINS_DRV_WritePin(PTD, 0, 0); // KICH HOAT MOSFET NGAT SAC
    }
}

/* --- HAM 1: DOC DONG DIEN TU INA226 QUA I2C ---*/
void Read_Battery_Current(void)
{
    uint8_t reg_addr = SHUNT_REG; // Muon doc thanh ghi Shunt Voltage 0x01
    uint8_t rx_buffer[2] = {0};   // Doc 2 byte du lieu
    status_t status;

    /* CAI DAT DIA CHI SLAVE TRUOC KHI GIAO TIEP  */
    // Tham so: Bo LPI2C1, Dia chi INA226, false
    LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, INA226_ADDR, false);

    /* Gui lenh yeu cau doc (5 tham so: Bo I2C, data gui, do dai 1, Stop bit, Timeout 100ms) */
    status = LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &reg_addr, 1, true, 100);

    /* Neu gui thanh cong, tien hanh doc ve */
    if (status == STATUS_SUCCESS)
    {
        /* Lay 2 byte du lieu ve */
        status = LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C1, rx_buffer, 2, true, 100);

        if (status == STATUS_SUCCESS)
        {
            /* 3. Ghep 2 byte thanh 1 so nguyen co dau 16-bit */
            Shunt_Raw = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);

            /* 4. Quy doi ra mili-Ampe (Dua tren tro Shunt 50A/75mV) */
            Current_mA = (int32_t)(Shunt_Raw * 1.666f);
        }
    }
    else
    {
        // Loi giao tiep (Dut day, sai dia chi) thi set dong bang 0
        Current_mA = 0;
    }
}

/* HAM 2: THUAT TOAN DEM COULOMB DE TINH % PIN (SOC) */
void Calculate_SOC(void)
{
    /* 1. Nhan biet sac hay xa */
    if (Current_mA > 0) {
        is_Charging = true;
    } else {
        is_Charging = false;
    }

    /* 2. Tinh luong dien thay doi trong 100ms (0.1s) */
    // Q = I * t -> Dien luong = Dong dien / 10
    int32_t delta_Charge_mAs = Current_mA / 10;

    /* 3. Cong don vao "be chua" tong */
    Remaining_Capacity_mAs = Remaining_Capacity_mAs + delta_Charge_mAs;

    /* 4. Khoa an toan (Khong cho dung luong vuot 100% hoac tut qua 0%) */
    if (Remaining_Capacity_mAs > BATTERY_CAPACITY_MAS) {
        Remaining_Capacity_mAs = BATTERY_CAPACITY_MAS;
    }
    else if (Remaining_Capacity_mAs < 0) {
        Remaining_Capacity_mAs = 0;
    }

    /* 5. Tinh % hien thi */
    SOC_Percent = (uint8_t)((Remaining_Capacity_mAs * 100) / BATTERY_CAPACITY_MAS);
}
