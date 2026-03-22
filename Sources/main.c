#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "lpit1.h"
#include "adConv1.h"
#include "lpi2c1.h"
#include "canCom1.h"
#include "interrupt_manager.h"
/* --- BIẾN VÀ �?ỊA CHỈ CHO I2C & INA226 (�?O DÒNG �?IỆN) --- */
#define INA226_ADDR 0x40        // �?ịa chỉ I2C mặc định của INA226
#define SHUNT_REG   0x01        // Thanh ghi chứa điện áp Shunt

/* --- �?IỆN TRỞ SHUNT LOẠI 50A- 75mV --- */
int16_t Shunt_Raw = 0;          // Giá trị thô từ cảm biến
int32_t Current_mA = 0;         // Dòng điện thực tế (mili-Ampe)

/* --- BIẾN CHO THUẬT TO�?N �?ẾM COULOMB (T�?NH % PIN) --- */
#define BATTERY_CAPACITY_MAH 8000 // Hệ pin 4S4P dung lượng 8000mAh
#define BATTERY_CAPACITY_MAS (BATTERY_CAPACITY_MAH * 3600)

int32_t Remaining_Capacity_mAs = BATTERY_CAPACITY_MAS; // Mặc định bật lên là 100%
uint8_t SOC_Percent = 100;                             // Phần trăm pin
bool is_Charging = false;

/* Các mảng chứa dữ liệu điện áp pin */
uint16_t ADC_Raw[4] = {0};      // Lưu giá trị thô 0-4095 từ ADC 12 bit
uint16_t Node_Voltage[4] = {0}; // �?iện áp tại các điểm đo (sau mạch chia áp)
uint16_t Cell_Voltage[4] = {0}; // �?iện áp thực tế của TỪNG cell (mV)

/* Biến trạng thái toàn cục cho I2C */
lpi2c_master_state_t lpi2c1State;

void Read_Battery_Voltage(void);
int16_t Target_Charge_Current_mA = 0; // Dòng sạc yêu cầu gửi cho Trạm sạc
volatile bool Flag_100ms = false;
void Manage_Charging(void);
void Read_Battery_Current(void);
void Calculate_SOC(void);
void LPIT0_Ch0_IRQHandler(void)
{
    /* Xóa c�? ngắt phần cứng của kênh 0 */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << 0));
    Flag_100ms = true; /* Báo cho vòng lặp chính biết đã đủ 100ms */
}

int main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/
    /* 1. KHỞI TẠO CLOCK & PIN */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* 2. KHỞI TẠO LPIT (TIMER 100MS) */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    LPIT_DRV_InitChannel(INST_LPIT1, 0, &lpit1_ChnConfig0);

    /* Cài đặt ngắt cho LPIT trong hệ thống NVIC (Cực kỳ quan tr�?ng) */
    INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, LPIT0_Ch0_IRQHandler, (isr_t*) 0);
    INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);

    /* Khởi động đếm th�?i gian */
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << 0));

    /* 3. KHỞI TẠO ADC0 (�?O �?IỆN �?P TỪNG CELL PIN) */
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);

    /* 4. KHỞI TẠO LPI2C (�?Ể �?ỌC INA226) */
    LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1State);

    /* 5. KHỞI TẠO CAN (�?Ể GIAO TIẾP ESP32/TRẠM SẠC) */
    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);

    /* =========================================
     * VÒNG LẶP CH�?NH
     * ========================================= */
    for(;;)
    {
        if (Flag_100ms)
        {
            Flag_100ms = false; // Xóa c�?
            Read_Battery_Voltage(); // �?�?c ADC để tính ra Vôn của 4 Cell
            Read_Battery_Current(); // �?�?c I2C từ INA226 để lấy Ampe
            Calculate_SOC();		// Tính dung lượng pin theo %
            Manage_Charging();

        }
    }

    return 0;
}
void Read_Battery_Voltage(void)
{
    /* --- 1. K�?CH HOẠT VÀ �?ỌC ADC LẦN LƯỢT 4 KÊNH --- */
    // Kênh 0 (Cell 1)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0); // Bắt đầu đo
    ADC_DRV_WaitConvDone(INST_ADCONV1);                       // Ch�? đo xong
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[0]);      // Lấy kết quả

    // Kênh 1 (Cell 2)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig1);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[1]);

    // Kênh 2 (Cell 3)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig2);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[2]);

    // Kênh 3 (Cell 4)
    ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig3);
    ADC_DRV_WaitConvDone(INST_ADCONV1);
    ADC_DRV_GetChanResult(INST_ADCONV1, 0, &ADC_Raw[3]);

    /* --- 2. T�?NH TO�?N �?IỆN �?P TẠI C�?C CELL --- */
    // Công thức: V_Node = (ADC_Raw * 5000 / 4095) * Hệ_số_chia_áp
    Node_Voltage[0] = (ADC_Raw[0] * 5000) / 4095;       //  �?iện áp tại Cell 1
    Node_Voltage[1] = ((ADC_Raw[1] * 5000) / 4095) * 2; // Tổng Cell 1+2
    Node_Voltage[2] = ((ADC_Raw[2] * 5000) / 4095) * 3; // Tổng Cell 1+2+3
    Node_Voltage[3] = ((ADC_Raw[3] * 5000) / 4095) * 4; // Tổng 4 Cell

    /* --- 3. THUẬT TO�?N TRỪ DỒN �?Ể TÌM �?IỆN �?P TỪNG CELL --- */
    Cell_Voltage[0] = Node_Voltage[0];
    Cell_Voltage[1] = Node_Voltage[1] - Node_Voltage[0];
    Cell_Voltage[2] = Node_Voltage[2] - Node_Voltage[1];
    Cell_Voltage[3] = Node_Voltage[3] - Node_Voltage[2];
}



/* Hàm đi�?u tiết dòng sạc và bảo vệ */
void Manage_Charging(void)
{
    uint16_t max_cell_voltage = 0;

    // 1. Tìm ra Cell có điện áp cao nhất (Cell có nguy cơ đầy trước/bị chai)
    for (int i = 0; i < 4; i++) {
        if (Cell_Voltage[i] > max_cell_voltage) {
            max_cell_voltage = Cell_Voltage[i];
        }
    }

    // 2. LOGIC �?IỀU TIẾT DÒNG SẠC THÔNG MINH
    if (max_cell_voltage < 4100) {
        // Tất cả các cell dưới 4,1V ; sạc bình thư�?ng
        Target_Charge_Current_mA = 3000;
        PINS_DRV_WritePin(PTD, 0, 1); // �?óng MOSFET cho phép sạc
    }
    else if (max_cell_voltage >= 4100 && max_cell_voltage < 4200) {
        // Bắt đầu có Cell chạm ngưỡng 4.1V -> ÉP DÒNG SẠC CHẬM LẠI để cho các cell khác cùng sạc
        Target_Charge_Current_mA = 500; // HẠ DÒNG XU�?NG VÀ SẠC CHẬM
        PINS_DRV_WritePin(PTD, 0, 1);
    }
    else {
        // Vượt 4.2V
        Target_Charge_Current_mA = 0;
        PINS_DRV_WritePin(PTD, 0, 0); // K�?CH HOẠT MOSFET NGẮT SẠC
    }
}

/* --- HÀM 1: �?ỌC DÒNG �?IỆN TỪ INA226 QUA I2C ---*/

void Read_Battery_Current(void)
{
    uint8_t reg_addr = SHUNT_REG; // Muốn đ�?c thanh ghi Shunt Voltage 0x01
    uint8_t rx_buffer[2] = {0};   // �?�?c 2 byte dữ liệu
    status_t status;

    /* CÀI �?ẶT �?ỊA CHỈ SLAVE TRƯỚC KHI GIAO TIẾP  */
    // Tham số: Bộ LPI2C1, �?ịa chỉ INA226, false
    LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, INA226_ADDR, false);

    /* Gửi lệnh yêu cầu đ�?c (5 tham số: Bộ I2C, data gửi, độ dài 1, Stop bit, Timeout 100ms) */
    status = LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &reg_addr, 1, true, 100);

    /* Nếu gửi thành công, tiến hành đ�?c v�? */
    if (status == STATUS_SUCCESS)
    {
        /* Lấy 2 byte dữ liệu v�? */
        status = LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C1, rx_buffer, 2, true, 100);

        if (status == STATUS_SUCCESS)
        {
            /* 3. Ghép 2 byte thành 1 số nguyên có dấu 16-bit */
            Shunt_Raw = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);

            /* 4. Quy đổi ra mili-Ampe (Dựa trên trở Shunt 50A/75mV) */
            Current_mA = (int32_t)(Shunt_Raw * 1.666f);
        }
    }
    else
    {
        // Lỗi giao tiếp (�?ứt dây, sai địa chỉ) thì set dòng bằng 0
        Current_mA = 0;
    }
}

/*  HÀM 2: THUẬT TO�?N �?ẾM COULOMB �?Ể T�?NH % PIN (SOC) */
void Calculate_SOC(void)
{
    /* 1. Nhận biết sạc hay xả */
    if (Current_mA > 0) {
        is_Charging = true;
    } else {
        is_Charging = false;
    }

    /* 2. Tính lượng điện thay đổi trong 100ms (0.1s) */
    // Q = I * t -> �?iện lượng = Dòng điện / 10
    int32_t delta_Charge_mAs = Current_mA / 10;

    /* 3. Cộng dồn vào "bể chứa" tổng */
    Remaining_Capacity_mAs = Remaining_Capacity_mAs + delta_Charge_mAs;

    /* 4. Khóa an toàn (Không cho dung lượng vượt 100% hoặc tụt quá 0%) */
    if (Remaining_Capacity_mAs > BATTERY_CAPACITY_MAS) {
        Remaining_Capacity_mAs = BATTERY_CAPACITY_MAS;
    }
    else if (Remaining_Capacity_mAs < 0) {
        Remaining_Capacity_mAs = 0;
    }

    /* 5. Tính % hiển thị */
    SOC_Percent = (uint8_t)((Remaining_Capacity_mAs * 100) / BATTERY_CAPACITY_MAS);
}
