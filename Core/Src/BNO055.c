/*
 * BNO055.c
 *
 *  Created on: Mar 6, 2024
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "BNO055_STM32.h"
#include <string.h>
#include "can_topic.h"

//CÁC BƯỚC:  
// 
//1. Khởi tạo cảm biến(Init – BNO055_Init)

//- Set mode về CONFIG_MODE → để cho phép cấu hình.
//- Chuyển sang Page 1, cấu hình : Dải đo gia tốc(SET_Accel_Range()).
//- Chuyển về Page 0. 
//- Chọn nguồn xung nhịp(Clock Source – nội bộ hay ngoài).
//- Cấu hình lại trục(remap trục).Chọn đơn vị đo(m / s², độ, ...).
//- Cài đặt chế độ nguồn(normal / low - power).
//- Đặt mode hoạt động chính(VD : NDOF).
//- Sau bước này, cảm biến bắt đầu hoạt động và sẵn sàng cung cấp dữ liệu.

//2. ĐỌC DỮ LIỆU (READ_DATA) 

//Dựa vào loại sensor được yêu cầu(GYRO, ACCEL, EULER, QUATERNION, ...), hàm sẽ :
//Gửi lệnh I2C_Mem_Read để đọc đúng địa chỉ thanh ghi dữ liệu(6 hoặc 8 byte).
//Chuyển dữ liệu uint8_t → int16_t → float đúng theo scaling.
//Ví dụ :
//Gia tốc : RAW / 100.0
//Gyro : RAW / 16.0
//Quaternion : RAW / (1 << 14)

//3.  Kiểm tra trạng thái hệ thống (CHECK_STATUS) 
//Đọc các thanh ghi :
//ST_RESULT_ADDR: kiểm tra self - test.
//SYS_STATUS_ADDR : hệ thống đã sẵn sàng chưa.
//SYS_ERR_ADDR : có lỗi hệ thống không.

//4. Đặt mode hoạt động(Set_Operation_Mode())
//Chuyển đổi giữa các chế độ hoạt động :
//NDOF → full sensor fusion(9 DOF)
//IMU, COMPASS, MAG_ONLY, ...
//Phải đặt về CONFIG_MODE trước khi đổi mode.

//5. Hiệu chuẩn cảm biến(Calibrate_BNO055())
//Đặt chế độ hoạt động.
//Hướng dẫn người dùng đặt thiết bị đúng cách.
//Đợi từng thành phần đạt mức calib == 3:
//Gyro, Accel, Mag, System.
//Ghi hoặc đọc lại offset nếu cần.

//6. Ghi và đọc offset(calib dữ liệu)
//getSensorOffsets() → đọc 22 byte offset.
//setSensorOffsets() → ghi 22 byte offset vào cảm biến.
//Giúp giữ lại hiệu chuẩn giữa các lần tắt / mở nguồn.

//7. Các tiện ích khác
//SelectPage() → chuyển giữa Page 0 & Page 1.
//ResetBNO055() → phần mềm reset cảm biến.
//SetPowerMODE() → đổi giữa normal / low power / suspend. 

/*!
 *   @brief  Gets the latest system status info
 *
 *   @param  BNO_status_t structure that contains status information
 *           STresult, SYSError and SYSStatus
 *
 *   @retval None
 */
void Check_Status(BNO_Status_t *result){  
	//Kiểm tra và xử lý trạng thái
	//Đọc các thanh ghi:
		//ST_RESULT_ADDR: Kết quả tự kiểm tra
		// SYS_STATUS_ADDR : Trạng thái hệ thống
		//SYS_ERR_ADDR : Mã lỗi hệ thống
	HAL_StatusTypeDef status;
	uint8_t value;

	  /* Self Test Results
	     1 = test passed, 0 = test failed

	     Bit 0 = Accelerometer self test
	     Bit 1 = Magnetometer self test
	     Bit 2 = Gyroscope self test
	     Bit 3 = MCU self test
	     0x0F = all good!
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, ST_RESULT_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: ST_RESULT_ADDR\n");
	}
	HAL_Delay(50);
	result->STresult = value;
	value=0;

	  /* System Status (see section 4.3.58)
	     0 = Idle
	     1 = System Error
	     2 = Initializing Peripherals
	     3 = System Iniitalization
	     4 = Executing Self-Test
	     5 = Sensor fusio algorithm running
	     6 = System running without fusion algorithms
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_STATUS_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: SYS_STATUS_ADDR\n");
	}
	HAL_Delay(50);
	result->SYSStatus = value;
	value=0;
	  /* System Error (see section 4.3.59)
	     0 = No error
	     1 = Peripheral initialization error
	     2 = System initialization error
	     3 = Self test result failed
	     4 = Register map value out of range
	     5 = Register map address out of range
	     6 = Register map write error
	     7 = BNO low power mode not available for selected operation mode
	     8 = Accelerometer power mode not available
	     9 = Fusion algorithm configuration error
	     A = Sensor configuration error
	   */
	status = HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_ERR_ADDR, 1, &value, 1, 100);
	if (status != HAL_OK) {
	    printf("I2C Read Error: SYS_ERR_ADDR\n");
	}
	HAL_Delay(50);
	result->SYSError = value;
}

/*!
 *   @brief  Changes register page
 *
 *   @param  Page number
 *   		Possible Arguments
 * 			[PAGE_0
 * 			 PAGE_1]
 *
 * 	 @retval None
 */
void SelectPage(uint8_t page){  //BNO055 có 2 page thanh ghi: PAGE 0 và PAGE 1 → Chuyển qua lại giữa chúng.

	if(HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, PAGE_ID_ADDR, 1, &page, 1, 100) != HAL_OK){
		printf("Register page replacement could not be set\n");
	}
	HAL_Delay(50);
}

/**
  * @brief  Software Reset to BNO055
  *
  * @param  None
  *
  * @retval None
  */
void ResetBNO055(void){ 
	//Gửi lệnh reset phần mềm (0x20 vào SYS_TRIGGER_ADDR)
	//Kiểm tra chip ID(phải bằng 0xA0) để đảm bảo reset thành công

	uint8_t reset = 0x20;
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, SYS_TRIGGER_ADDR, 1, &reset, 1, 100);
	HAL_Delay(500);

	//Checking for is reset process done
	uint8_t chip_id=0;
	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, CHIP_ID_ADDR, 1, &chip_id, 1, 100);

	//If value of id register is not equal to BNO055 chip id which is 0xA0, wait until equal to each other
	while(chip_id != BNO055_ID) {
		printf("BNO055-> Undefined chip id\n");
		HAL_Delay(500);
	}
}

/*!
 *   @brief  Reads various data measured by BNO055
 *
 *   @param  Register base address of the data to be read
 * 			Possible arguments
 * 			[SENSOR_ACCEL
 *			 SENSOR_GYRO
 * 			 SENSOR_MAG
 *			 SENSOR_EULER
 *			 SENSOR_LINACC
 *			 SENSOR_GRAVITY
 *			 SENSOR_QUATERNION]
 *
 *   @retval Structure containing the values ​​of the read data
 */
void ReadData(BNO055_Sensors_t *sensorData,BNO055_Sensor_Type sensors){ //Đọc dữ liệu cảm biến

	//Dựa vào kiểu dữ liệu được chọn (SENSOR_ACCEL, SENSOR_EULER, v.v.), đọc đúng thanh ghi từ BNO055.
	//Dữ liệu đọc được scale đúng đơn vị(e.g.chia cho 100.0 hoặc 16.0).
	//Dữ liệu gán vào struct sensorData
	   uint8_t buffer[8];

	    if (sensors & SENSOR_GRAVITY) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_GRAVITY, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Gravity.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/100.0); // chia cho 100 để quy đổi ra đơn vị chuẩn 
	        sensorData->Gravity.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->Gravity.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_QUATERNION) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_QUATERNION, 1, buffer, 8, HAL_MAX_DELAY);
	        sensorData->Quaternion.W = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/(1<<14));
	        sensorData->Quaternion.X = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/(1<<14));
	        sensorData->Quaternion.Y = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/(1<<14));
	        sensorData->Quaternion.Z = (float)(((int16_t)((buffer[7] << 8) | buffer[6]))/(1<<14));
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_LINACC) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_LINACC, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->LineerAcc.X = (float)(((int16_t)((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->LineerAcc.Y = (float)(((int16_t)((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->LineerAcc.Z = (float)(((int16_t)((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }

	    if (sensors & SENSOR_GYRO) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_GYRO, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Gyro.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Gyro.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Gyro.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_ACCEL) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_ACCEL, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Accel.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/100.0);
	        sensorData->Accel.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/100.0);
	        sensorData->Accel.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/100.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_MAG) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_MAG, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Magneto.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Magneto.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Magneto.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
	    if (sensors & SENSOR_EULER) {

	    	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_EULER, 1, buffer, 6, HAL_MAX_DELAY);
	        sensorData->Euler.X = (float)(((int16_t) ((buffer[1] << 8) | buffer[0]))/16.0);
	        sensorData->Euler.Y = (float)(((int16_t) ((buffer[3] << 8) | buffer[2]))/16.0);
	        sensorData->Euler.Z = (float)(((int16_t) ((buffer[5] << 8) | buffer[4]))/16.0);
	        memset(buffer, 0, sizeof(buffer));
	    }
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  Operation modes
 *          Mode Values;
 *           [CONFIG_MODE
 *            ACC_ONLY
 *            MAG_ONLY
 *            GYR_ONLY
 *            ACC_MAG
 *            ACC_GYRO
 *            MAG_GYRO
 *            AMG
 *            IMU
 *            COMPASS
 *            M4G
 *            NDOF_FMC_OFF
 *            NDOF]
 *
 *  @retval None
 */
void Set_Operation_Mode(Op_Modes_t Mode){ //Đặt chế độ hoạt động (e.g. NDOF, IMU, CONFIG_MODE...)

	SelectPage(PAGE_0);
	if(	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, OPR_MODE_ADDR, 1, &Mode, 1, 100) !=HAL_OK){
		printf("Operation mode could not be set!\n");
	}
	else printf("Operation mode switching succeeded.\n");

	if(Mode == CONFIG_MODE) HAL_Delay(19);

	else HAL_Delay(9);

}

/*!
 *  @brief  Set the power mode of BNO055
 *  @param  power modes
 *          possible values
 *           [BNO055_NORMAL_MODE
 *            BNO055_LOWPOWER_MODE
 *            BNO055_SUSPEND_MODE]
 *
 *  @retval None
 */
void SetPowerMODE(uint8_t BNO055_){ //Cấu hình power mode: Normal, Low-power, Suspend.

	if(	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, PWR_MODE_ADDR, 1, &BNO055_, 1, 100) != HAL_OK)
	{
		printf("Power mode could not be set!\n");
	}
	else
	{
		printf("Power mode switching succeeded.\n");
	}
	HAL_Delay(50);

}

/*!
 *  @brief  Selects the chip's clock source
 *  @param  Source
 *          possible values
 *           [CLOCK_EXTERNAL
 *            CLOCK_INTERNAL]
 *
 *  @retval None
 */
void Clock_Source(uint8_t source) { //Chọn xung clock nội/ngoại.

	//7th bit: External Crystal=1; Internal Crystal=0
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, SYS_TRIGGER_ADDR, 1, &source, sizeof(source), 100);
}

/*!
 *  @brief  Changes the chip's axis signs and remap
 *  @param  remapcode and signcode
 *         	Default Parameters:[DEFAULT_AXIS_REMAP(0x24), DEFAULT_AXIS_SIGN(0x00)]
 *
 *  @retval None
 */
void BNO055_Axis(uint8_t remap, uint8_t sign){ // Chuyển đổi trục (Axis remap)

	//Gán lại trục X–Y–Z hoặc đảo dấu (khi lắp cảm biến không đúng hướng).
	//Ví dụ : nếu gắn nghiêng 90°, có thể đổi trục để kết quả đúng.

	HAL_I2C_Mem_Write(&bno_i2c,P_BNO055, AXIS_MAP_CONFIG_ADDR, 1, &remap, 1, 100);
	HAL_Delay(20);
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, AXIS_MAP_SIGN_ADDR, 1, &sign, 1, 100);
	HAL_Delay(100);
}

/*!
 *  @brief  Sets the accelerometer range
 *  @param  range
 *          possible values
 *           [Range_2G
 *            Range_4G
 *            Range_8G
 *            Range_16G]
 *
 *  @retval None
 */
void SET_Accel_Range(uint8_t range){

	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, ACC_CONFIG_ADDR, 1, &range, 1, 100);
	HAL_Delay(100);

}

/**
  * @brief  Initialization of BNO055
  *
  * @param  Init argument to a BNO055_Init_t structure that contains
  *         the configuration information for the BNO055 device.
  *
  * @retval None
  */
void BNO055_Init(void){  //Khởi tạo toàn bộ cảm biến
	// Gồm các bước:

	//Vào chế độ cấu hình
	//Đặt page = 1, cấu hình dải gia tốc
	//Đặt page = 0, chọn clock, map trục, chọn đơn vị(m / s² hay g…)
	//Đặt chế độ nguồn
	//Đặt chế độ hoạt động chính(ví dụ NDOF)
    BNO055_Init_t Init;

    Init.Unit_Sel     = UNIT_ORI_WINDOWS | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2;
    Init.Axis         = DEFAULT_AXIS_REMAP;
    Init.Axis_sign    = DEFAULT_AXIS_SIGN;
    Init.Mode         = BNO055_NORMAL_MODE;
    Init.OP_Modes     = NDOF;
    Init.Clock_Source = CLOCK_INTERNAL;
    Init.ACC_Range    = Range_2G;

	//Set operation mode to config_mode for initialize all register
	Set_Operation_Mode(CONFIG_MODE); // cấu hình 
	HAL_Delay(50);
	/*
	 * Set register page number to 1
	 * Configure Accelerometer range
	 */
	SelectPage(PAGE_1);
	SET_Accel_Range(Init.ACC_Range);
	HAL_Delay(50);

	//Set register page number to 0
	SelectPage(PAGE_0);
	HAL_Delay(50);

	//Read clock status. If status=0 then it is free to configure the clock source
	uint8_t status;
	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, SYS_CLK_STATUS_ADDR, 1, &status, 1, 100);
	HAL_Delay(50);
	//Checking if the status bit is 0
	if(status == 0)
	{
		//Changing clock source
		Clock_Source(Init.Clock_Source);
		HAL_Delay(100);
	}

	//Configure axis remapping and signing
	BNO055_Axis(Init.Axis, Init.Axis_sign);
	HAL_Delay(100);

	//Configure data output format and the measurement unit
	HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, UNIT_SEL_ADDR, 1, &Init.Unit_Sel, sizeof(Init.Unit_Sel), 100);
	HAL_Delay(100);

	//Set power mode
	SetPowerMODE(Init.Mode);
	HAL_Delay(100);

	//Set operation mode
	Set_Operation_Mode(Init.OP_Modes);
	HAL_Delay(100);

	printf("BNO055 Initialization process is done!\n");
}

/**
  * @brief  Gets calibration status of accel, gyro, mag and system
  *
  * @param  None
  *
  * @retval Calib_status_t structure that contains
  *         the calibration status of accel, gyro, mag and system.
  */
void getCalibration(Calib_status_t *calib) { // Hiệu chuẩn (calibration)

	//Đọc thanh ghi CALIB_STAT_ADDR(1 byte) : Trích xuất tình trạng hiệu chuẩn(0–3) của từng sensor.
	//Calibrate_BNO055()
	//	Hướng dẫn người dùng thực hiện hiệu chuẩn từng phần(Gyro → Accel → Mag → System).
	//In ra trạng thái khi đủ 3 / 3 (fully calibrated).

    uint8_t calData;

    // Read calibration status register using I2C
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, P_BNO055, CALIB_STAT_ADDR, 1, &calData, 1, HAL_MAX_DELAY);

    // Check if read was successful
    if (status == HAL_OK) {

        // Extract calibration status values

        	calib->System= (calData >> 6) & 0x03;


        	calib->Gyro = (calData >> 4) & 0x03;


        	calib->Acc = (calData >> 2) & 0x03;


        	calib->MAG = calData & 0x03;

    } else {
        printf("Failed to read calibration status register.\n");
    }
}

/**
  * @brief  Gets sensor offsets
  *
  * @param  22 byte long buffer to hold offset data
  *
  * @retval None
  *
  */
void getSensorOffsets(uint8_t *calibData) { //Ghi & đọc thông số offset

        // Save the current mode
        uint8_t lastMode = getCurrentMode();

        // Switch to CONFIG mode
        Set_Operation_Mode(CONFIG_MODE); // Vào CONFIG_MODE, đọc 22 bytes offset data từ cảm biến.
        printf("Switched to CONFIG mode.\n");

        // Read the offset registers
        HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, ACC_OFFSET_X_LSB_ADDR, 1, calibData, 22, 100); // Viết lại 22 bytes offset vào thanh ghi offset.
        printf("Calibration data obtained.\n");

        // Restore the previous mode
        Set_Operation_Mode(lastMode);
        printf("Restored to previous mode.\n");
}

/**
  * @brief  Sets sensor offsets
  *
  * @param  22 byte long buffer containing offset data
  *
  * @retval None
  *
  */
void setSensorOffsets(const uint8_t *calibData) {
    uint8_t lastMode = getCurrentMode();

    // Switch to CONFIG mode
    Set_Operation_Mode(CONFIG_MODE);
    printf("Switched to CONFIG mode.\n");

    // Write calibration data to the sensor's offset registers using memory write
    HAL_I2C_Mem_Write(&bno_i2c, P_BNO055, ACC_OFFSET_X_LSB_ADDR, 1, (uint8_t *)calibData, 22, 100);
    printf("Wrote calibration data to sensor's offset registers.\n");

    // Restore the previous mode
    Set_Operation_Mode(lastMode);
    printf("Restored to previous mode.\n");
}

/**
  * @brief  Checks the calibration status of the sensor
  *
  * @param  None
  *
  * @retval True of False
  *
  */
bool isFullyCalibrated(void) { //Dựa vào getCurrentMode() để xác định xem cần kiểm tra calibration loại nào (e.g. chỉ Accel, hay Acc+Gyro…)
//    Calib_status_t calib ={0};
    Calib_status_t calib ={0};
    getCalibration(&calib);


    switch (getCurrentMode()) {
        case ACC_ONLY:
            return (calib.Acc == 3);
        case MAG_ONLY:
            return (calib.MAG == 3);
        case GYRO_ONLY:
        case M4G: /* No magnetometer calibration required. */
            return (calib.Gyro == 3);
        case ACC_MAG:
        case COMPASS:
            return (calib.Acc == 3 && calib.MAG == 3);
        case ACC_GYRO:
        case IMU:
            return (calib.Acc == 3 && calib.Gyro == 3);
        case MAG_GYRO:
            return (calib.MAG == 3 && calib.Gyro == 3);
        default:
            return (calib.System == 3 && calib.Gyro == 3 && calib.Acc == 3 && calib.MAG == 3);
    }
}

/**
  * @brief  Gets the current operating mode of the chip
  *
  * @param  None
  *
  * @retval Operating mode
  *
  */
Op_Modes_t getCurrentMode(void) { //Trả về chế độ hiện tại của cảm biến.

	Op_Modes_t mode;

	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, OPR_MODE_ADDR, 1, &mode, 1, 100);

    return mode;
}

/**
  * @brief  Calibrates BNO055
  *
  * @param  None
  *
  * @retval None
  *
  */
bool Calibrate_BNO055(void) {

		Calib_status_t calib={0};
        printf("Calibrating BNO055 sensor...\n");

        // Set operation mode to FUSION_MODE or appropriate mode for calibration
        Set_Operation_Mode(NDOF);
    	HAL_Delay(100);
        // Gyroscope calibration
        printf("Calibrating gyroscope...\n");
        printf("Place the device in a single stable position\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.Gyro !=3);
        printf("Gyroscope calibration complete.\n");

        // Accelerometer calibration
        printf("Calibrating accelerometer...\n");
        printf("Place the device in 6 different stable positions\n");
        for (int i = 0; i < 6; i++) {
            printf("Position %d\n", i + 1);
            HAL_Delay(1500);  // Simulated accelerometer calibration time
        }

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.Acc !=3);
        printf("Accelerometer calibration complete.\n");

        // Magnetometer calibration
        printf("Calibrating magnetometer...\n");
        printf("Make some random movements\n");
        HAL_Delay(1000);  // Simulated gyroscope calibration time

        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.MAG !=3);
        printf("Magnetometer calibration complete.\n");

        // System calibration
        printf("Calibrating system...\n");
        printf("Keep the device stationary until system calibration reaches level 3\n");
        do {
            getCalibration(&calib);
        	HAL_Delay(500);
		} while (calib.System !=3);
        HAL_Delay(500);

        // Check calibration status
        while(!isFullyCalibrated()) HAL_Delay(500);
        printf("Sensor is fully calibrated.\n");

        printf("System: %d      Gyro: %d       Accel: %d       MAG: %d\n",calib.System,calib.Gyro , calib.Acc, calib.MAG);
        if(isFullyCalibrated()) return true;
        else return false;
}

BNO055_EulerRaw_t BNO055_ReadEulerRaw(void)
{
	BNO055_EulerRaw_t raw = {0};
	uint8_t buffer[6];

	HAL_I2C_Mem_Read(&bno_i2c, P_BNO055, BNO_EULER, 1, buffer, 6, HAL_MAX_DELAY);

	raw.Yaw   = (int16_t)((buffer[1] << 8) | buffer[0]);
	raw.Roll  = (int16_t)((buffer[3] << 8) | buffer[2]);
	raw.Pitch = (int16_t)((buffer[5] << 8) | buffer[4]);

	return raw;
}
BNO055_EulerFloat_t BNO055_ReadEulerFloat(void)
{
	BNO055_EulerRaw_t raw = BNO055_ReadEulerRaw();
	BNO055_EulerFloat_t angle;

	angle.Yaw   = raw.Yaw / 16.0f;
	angle.Roll  = raw.Roll / 16.0f;
	angle.Pitch = raw.Pitch / 16.0f;

	return angle;
}

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;


HAL_StatusTypeDef CAN_SendTopicData(uint16_t topic_id, uint8_t *data, uint8_t len);
void BNO055_PrintEulerRaw(void)
{
    BNO055_EulerRaw_t raw = BNO055_ReadEulerRaw();

    // Đổi sang độ: raw = int16_t, đơn vị là 1/16 độ
    float rawYaw   = raw.Yaw   / 16.0f;
    float rawPitch = raw.Pitch / 16.0f;
    float rawRoll  = raw.Roll  / 16.0f;

    char msg[128];
    sprintf(msg,
        "RAW Angle: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f° (raw hex: R=0x%04X, P=0x%04X, Y=0x%04X)\r\n",
        rawRoll, rawPitch, rawYaw,
        (uint16_t)raw.Roll, (uint16_t)raw.Pitch, (uint16_t)raw.Yaw);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void BNO055_SendEulerCAN(void)
{
    BNO055_Sensors_t sensorData;
    ReadData(&sensorData, SENSOR_EULER);

    // Giữ 2 chữ số thập phân
    int16_t roll  = (int16_t)(sensorData.Euler.Z *100);
    int16_t pitch = (int16_t)(sensorData.Euler.Y *100);
    int16_t yaw   = (int16_t)((sensorData.Euler.X) *100);

    // Trừ offset để đưa hướng Tây Bắc về 0 độ
//    float yaw_deg = sensorData.Euler.X ;
//
//    // CHUẨN HÓA về [-180, 180] để đảm bảo máy tính nhúng xử lý đúng
//    if (yaw_deg > 180.0f)
//        yaw_deg -= 360.0f;
//     if (yaw_deg - 160.0f)
//        yaw_deg -= 160.0f;
//
    // Nhân 100 để đưa về dạng int16_t gửi CAN
//    int16_t yaw = (int16_t)(yaw_deg * 100);


    uint8_t data[8] = {
            (roll >> 8) & 0xFF,  roll & 0xFF,
            (pitch >> 8) & 0xFF, pitch & 0xFF,
			(yaw >> 8) & 0xFF,   yaw & 0xFF,
//			yaw & 0xFF,         // byte thấp
//			0, 0, 0, 0, 0, 0    // phần còn lại không dùng
    };
    CAN_SendTopicData(TOPIC_ID_IMU_EULER, data, 8);
}


void BNO055_SendGyroCAN(void)
{
    BNO055_Sensors_t sensorData;
    ReadData(&sensorData, SENSOR_GYRO);

    // Scale về int16_t để gửi qua CAN (độ chính xác 0.01 °/s)
    int16_t gyro_x = (int16_t)(sensorData.Gyro.X );
    int16_t gyro_y = (int16_t)(sensorData.Gyro.Y);
    int16_t gyro_z = (int16_t)(sensorData.Gyro.Z);

    uint8_t data[6] = {
        (gyro_x >> 8) & 0xFF, gyro_x & 0xFF,
        (gyro_y >> 8) & 0xFF, gyro_y & 0xFF,
        (gyro_z >> 8) & 0xFF, gyro_z & 0xFF
    };

    CAN_SendTopicData(TOPIC_ID_IMU_Gyro, data, 8);  // Gửi với ID mới

    // Optional: debug UART
    char msg[128];
    sprintf(msg, "Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s\r\n",
            sensorData.Gyro.X, sensorData.Gyro.Y, sensorData.Gyro.Z);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void BNO055_PrintEulerDebug(void)
{
    BNO055_Sensors_t sensorData;
    ReadData(&sensorData, SENSOR_EULER);

    float roll  = sensorData.Euler.Z;
    float pitch = sensorData.Euler.Y;
    float yaw   = sensorData.Euler.X;

//    if (yaw > 180.0f)
//        yaw -= 360.0f;
//    else if (yaw < -180.0f)
//        yaw += 360.0f;

    char msg[128];
    sprintf(msg, "DEBUG: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\r\n", roll, pitch, yaw);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

}


