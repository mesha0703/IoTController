/*	
This is code for implementation of lithium battery fuel gauge IC MAX17055.
MAX17055 uses I2C for receiving orders from the host.
This file includes set of functions, that can request specific info or change parameters of the IC.
This code is created with the help of MAX17055 Datasheet, User-Guide and Software-Implementation-Guide (see links below).

MAX17055 Links
Datasheet: https://eu.mouser.com/datasheet/2/609/MAX17055-3468925.pdf
User-Guide: https://www.analog.com/media/en/technical-documentation/user-guides/max17055-user-guide.pdf
Software-Implementation-Guide: https://www.analog.com/media/en/technical-documentation/user-guides/max17055-software-implementation-guide.pdf
*/

#include <stdio.h>
#include "driver/i2c_master.h"
#include "battery_fuelGauge.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "portmacro.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

// Set this variable true if you want to enter DEBUG mode
const bool DEBUG = true;

/* USER SETTINGS */
// The time between reading MAX17055 data in ms
#define GETDATA_PAUSE 10*1000
#define UPDATENVSPARAM_PAUSE 5*60*1000000
// Rsense Resistor Value (in mOhm)
#define RSENSE_VALUE 10
// Expected Battery Capacitance (in mAh)
#define EXPECTED_BATTERY_CAP 1100
// Current when charge cycle has completed (in mA). Check battery-charging IC for this setting
#define ENDOFCHARGE_CURRENT 250
// This is value calculated for the VEmpty register. VE=3.3V, VR=3.88V. See User-Guide for more details.
#define VEMPTY_REG_VALUE 0xA561

#define SDA_PIN 15
#define SCL_PIN 7
#define I2C_DEVICE_ADDRESS 0x36
#define I2C_SCL_SPEED 100000
#define I2C_WAITTIMEOUT 1000

#define TAG "fuel_gauge"

/* 
Registers and Data-Bits in the registers. 
Format for registers: "REGISTERNAME"_REG
Format for data bits in registers: "REGISTERNAME"_REG_"DATABITNAME"
*/
#define STATUS_REG 0x00
#define STATUS_REG_POR (1<<1)
#define FSTAT_REG 0x3D
#define FSTAT_REG_DNR (1<<0)
#define HIBCFG_REG 0xBA
#define HIBCFG_REG_ENHIB (1<<15)
#define DESIGNCAP_REG 0x18
#define DQACC_REG 0x05
#define ICHRGTERM_REG 0x1E
#define VEMPTY_REG 0x3A
#define DPACC_REG 0x46
#define MODELCFG_REG 0xDB
#define MODELCFG_REG_REFRESH (1<<15)
#define REPCAP_REG 0x05
#define REPSOC_REG 0x06
#define TTE_REG 0x11
#define RCOMP0_REG 0x38
#define TEMPCO_REG 0x39
#define FULLCAPREP_REG 0x10
#define CYCLES_REG 0x17
#define FULLCAPNOM_REG 0x23
#define MIXCAP_REG 0x0F
#define MIXSOC_REG 0x0D

static i2c_master_dev_handle_t dev_handle;
static i2c_master_bus_handle_t bus_handle;
static bool i2c_isinit = false;
static esp_err_t init_i2c(void);
static int8_t checkPOR(void);
static esp_err_t readRegister(uint8_t reg, uint16_t *data_rd);
static esp_err_t writeRegister(uint8_t reg, uint16_t data_wr);
static esp_err_t writeAndVerifyRegister(uint8_t reg, uint16_t data_wr);
static uint16_t convCapToStFormat(uint16_t capacity);
static uint16_t convStFormatToCap(uint16_t capacityStFormat);
static uint16_t convCurrentToStFormat(uint16_t current);
static uint8_t convStFormatToPerc(uint16_t percentageStFormat);
static esp_err_t checkAndWrite_nvs(nvs_handle_t handle, char *key, uint16_t value);
static esp_err_t openFuelGaugeNVSHandle(nvs_handle_t handle, nvs_open_mode_t mode);
esp_err_t readNVSParam_fuelGauge(void);
esp_err_t loadNVSParamAfterPOR_fuelGauge(void);

// Variables for loading saved parameters from NVS for MAX17055
static uint16_t Loaded_RCOMP0, Loaded_TempCo, Loaded_FullCapRep, Loaded_Cycles, Loaded_FullCapNom;
static nvs_handle_t nvs_fuelGauge_handle;

// Global mutex for I2C resource protection
static SemaphoreHandle_t i2c_mutex;

void getData_fuelGauge(void *pvParameters) {
	int8_t status_reg_por_rd;
	uint16_t RepCapStFormat_rd, RepSOCStFormat_rd, TTEStFormat;
	
	for (;;) {
		// Check for reset
		status_reg_por_rd = checkPOR();
		
		if (status_reg_por_rd == -1) {
			ESP_LOGE(TAG, "Error occurred while checking POR status.");
		}
		else if ((status_reg_por_rd == 1) || (status_reg_por_rd == 0)) {
			
			if (status_reg_por_rd == 1) {
				ESP_LOGI(TAG, "POR flag is set. Initializing fuel gauge IC again...");
				init_fuelGauge();
			}
			
			// Read the reported capacity and state of charge (SOC)
			ESP_ERROR_CHECK_WITHOUT_ABORT( readRegister(REPCAP_REG, &RepCapStFormat_rd) );
			uint16_t RepCap = convStFormatToCap(RepCapStFormat_rd);
			if (DEBUG) ESP_LOGI(TAG, "RepCap:\t%d mAh", RepCap);
			
			ESP_ERROR_CHECK_WITHOUT_ABORT( readRegister(REPSOC_REG, &RepSOCStFormat_rd) );
			uint8_t RepSOC = convStFormatToPerc(RepSOCStFormat_rd);
			if (DEBUG) ESP_LOGI(TAG, "RepSOC:\t%d%%", RepSOC);
			
			// Read the remaining time to empty (TTE)
			ESP_ERROR_CHECK_WITHOUT_ABORT( readRegister(TTE_REG, &TTEStFormat) );
			if (DEBUG) ESP_LOGI(TAG, "TTE:\t%d", TTEStFormat);
			
			ESP_LOGI(TAG, "Done reading data from MAX17055");
		}
		else ESP_LOGE(TAG, "Invalid state of POR returned.");
		
		vTaskDelay(GETDATA_PAUSE / portTICK_PERIOD_MS);
	}
}

esp_err_t loadNVSParamAfterPOR_fuelGauge(void) {
	esp_err_t err_check;
	uint16_t MixCap_wr, MixSOC_rd, dQacc_wr;
	int8_t status_reg_por_rd;
	
	// Open handle
	err_check = openFuelGaugeNVSHandle(nvs_fuelGauge_handle, NVS_READONLY);
    if (err_check != ESP_OK) return err_check;
    
    // Read Loaded_RCOMP0, Loaded_TempCo, Loaded_FullCapRep, Loaded_Cycles, Loaded_FullCapNom from NVS
    err_check = readNVSParam_fuelGauge();
    if (err_check != ESP_OK) return err_check;
    
    status_reg_por_rd = checkPOR();
	if (status_reg_por_rd == -1) {
		ESP_LOGE(TAG, "Error occurred while checking POR status.");
		return ESP_FAIL;
	}
	else if (status_reg_por_rd == 1) {
		ESP_LOGI(TAG, "POR flag is set. Initializing fuel gauge IC again...");
		init_fuelGauge();
	}
    
    // Write loaded RCOMP0, TempCo and FullCapNom values to MAX17055
    if (DEBUG) ESP_LOGI(TAG, "Writing MAX17055 values to the chip registers.");
    err_check = writeAndVerifyRegister(RCOMP0_REG, Loaded_RCOMP0);
    if (err_check != ESP_OK) return err_check;
    err_check = writeAndVerifyRegister(TEMPCO_REG, Loaded_TempCo);
    if (err_check != ESP_OK) return err_check;
	err_check = writeAndVerifyRegister(FULLCAPNOM_REG, Loaded_FullCapNom);
	if (err_check != ESP_OK) return err_check;
	
	// Wait 350ms (specified by Software-Impl.-Guide)
	vTaskDelay(350 / portTICK_PERIOD_MS);
	
	/* Restore FullCap */
	
	// Read MixSOC, which is needed for calculating MixCap
	err_check = readRegister(MIXSOC_REG, &MixSOC_rd);
	if (err_check != ESP_OK) return err_check;
	
	// Calculating MixCap using formula from Software-Impl.-Guide
	MixCap_wr = (MixSOC_rd * Loaded_FullCapNom) / 25600;
	// Write MixCap value to MAX17055
	err_check = writeAndVerifyRegister(MIXCAP_REG, MixCap_wr);
	if (err_check != ESP_OK) return err_check;
	
	// Write FullCapRep
	err_check = writeAndVerifyRegister(FULLCAPREP_REG, Loaded_FullCapRep);
	if (err_check != ESP_OK) return err_check;
	
	// Write dQacc to 200% of Capacity and dPacc to 200% (according to Software-Impl.-Guide).
	// Calculating trough 16 because percentage in these Registers is 1/16% per LSB.
	dQacc_wr = Loaded_FullCapNom / 16;
	
	// Write 200% to dPacc. 0x0C80 / 16 = 3200 / 16 = 200%.
	err_check = writeAndVerifyRegister(DPACC_REG, 0x0C80);
	if (err_check != ESP_OK) return err_check;
	
	// Write dQacc value to the register
	err_check = writeAndVerifyRegister(DQACC_REG, dQacc_wr);
	if (err_check != ESP_OK) return err_check;
	
	// Wait 350ms (specified by Software-Impl.-Guide)
	vTaskDelay(350 / portTICK_PERIOD_MS);
	
	err_check = writeAndVerifyRegister(CYCLES_REG, Loaded_Cycles);
	if (err_check != ESP_OK) return err_check;
	
	ESP_LOGI(TAG, "Finished loading MAX17055 values from NVS and writing them to the chip registers.");
	
	return ESP_OK;
}

// This function reads parameters from NVS and updates the global variables Loaded_RCOMP0, ...
esp_err_t readNVSParam_fuelGauge(void) {
	esp_err_t err_check;
    
    // Open NVS handle
    err_check = openFuelGaugeNVSHandle(nvs_fuelGauge_handle, NVS_READONLY);
    if (err_check != ESP_OK) return err_check;
    if (DEBUG) ESP_LOGI(TAG, "Reading MAX17055 parameters from NVS ... ");
    
    // Read
    err_check = nvs_get_u16(nvs_fuelGauge_handle, "RCOMP0", &Loaded_RCOMP0);
    if (err_check == ESP_ERR_NVS_NOT_FOUND) ESP_LOGI(TAG, "The value RCOMP0 is not initialized yet!");
    if (err_check != ESP_OK) return err_check;
    ESP_LOGI(TAG, "Loaded_RCOMP0 = %04X", Loaded_RCOMP0 & 0xFFFF);
    
    err_check = nvs_get_u16(nvs_fuelGauge_handle, "TEMPCO", &Loaded_TempCo);
    if (err_check == ESP_ERR_NVS_NOT_FOUND) ESP_LOGI(TAG, "The value TEMPCO is not initialized yet!");
    if (err_check != ESP_OK) return err_check;
    ESP_LOGI(TAG, "Loaded_TemPCo = %04X", Loaded_TempCo & 0xFFFF);
	
	err_check = nvs_get_u16(nvs_fuelGauge_handle, "FULLCAPREP", &Loaded_FullCapRep);
    if (err_check == ESP_ERR_NVS_NOT_FOUND) ESP_LOGI(TAG, "The value FULLCAPREP is not initialized yet!");
    if (err_check != ESP_OK) return err_check;
    ESP_LOGI(TAG, "Loaded_FullCapRep = %04X", Loaded_FullCapRep & 0xFFFF);
    
    err_check = nvs_get_u16(nvs_fuelGauge_handle, "CYCLES", &Loaded_Cycles);
    if (err_check == ESP_ERR_NVS_NOT_FOUND) ESP_LOGI(TAG, "The value CYCLES is not initialized yet!");
    if (err_check != ESP_OK) return err_check;
    ESP_LOGI(TAG, "Loaded_Cycles = %04X", Loaded_Cycles & 0xFFFF);
    
    err_check = nvs_get_u16(nvs_fuelGauge_handle, "FULLCAPNOM", &Loaded_FullCapNom);
    if (err_check == ESP_ERR_NVS_NOT_FOUND) ESP_LOGI(TAG, "The value FULLCAPNOM is not initialized yet!");
    if (err_check != ESP_OK) return err_check;
    ESP_LOGI(TAG, "Loaded_FullCapNom = %04X", Loaded_FullCapNom & 0xFFFF);

    // Close
    nvs_close(nvs_fuelGauge_handle);

	return ESP_OK;
}

void updateNVSParam_fuelGauge(void *pvParameters) {
	esp_err_t err_check;
	uint16_t new_RCOMP0, new_TempCo, new_FullCapRep, new_Cycles, new_FullCapNom;
	bool update_RCOMP0 = false, update_TempCo = false, update_FullCapRep = false,\
		 update_Cycles = false, update_FullCapNom = false;
 	int8_t status_reg_por_rd;
	
	for (;;) {
	
		if (DEBUG) ESP_LOGI(TAG, "Saving MAX17055 parameters to NVS...");
		
		// Check for reset
		status_reg_por_rd = checkPOR();
		if (status_reg_por_rd == -1) {
			ESP_LOGE(TAG, "Error occurred while checking POR status.");
		}
		else if ((status_reg_por_rd == 1) || (status_reg_por_rd == 0)) {
			if (status_reg_por_rd == 1) {
				ESP_LOGI(TAG, "POR flag is set. Initializing fuel gauge IC again...");
				init_fuelGauge();
			}
		
			/* Save important values in case of a power loss */
			
			// The RComp0 register holds characterization information critical to computing the
			// open-circuit voltage of a cell under loaded conditions.
			err_check = readRegister(RCOMP0_REG, &new_RCOMP0);
			if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while reading RCOMP0 Register: (%s)", esp_err_to_name(err_check));
			
			// The TempCo register holds temperature compensation information for the RComp0 register value.
			err_check = readRegister(TEMPCO_REG, &new_TempCo);
			if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while reading TempCo Register: (%s)", esp_err_to_name(err_check));
			
			// This register reports the full capacity that goes with RepCap.
			err_check = readRegister(FULLCAPREP_REG, &new_FullCapRep);
			if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while reading FullCapRep Register: (%s)", esp_err_to_name(err_check));
			
			// The Cycles register maintains a total count of the number of charge/discharge cycles of the cell.
			err_check = readRegister(CYCLES_REG, &new_Cycles);
			if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while reading Cycles Register: (%s)", esp_err_to_name(err_check));
			
			// This register holds the calculated full capacity of the cell, not including temperature and empty compensation.
			// A new full-capacity nominal value is calculated each time a cell relaxation event is detected.
			err_check = readRegister(FULLCAPNOM_REG, &new_FullCapNom);
			if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while reading FullCapNom Register: (%s)", esp_err_to_name(err_check));
			
			// Compare new data to old data
			if (new_RCOMP0 != Loaded_RCOMP0) update_RCOMP0 = true;
			if (new_TempCo != Loaded_TempCo) update_TempCo = true;
			if (new_FullCapRep != Loaded_FullCapRep) update_FullCapRep = true;
			if (new_Cycles != Loaded_Cycles) update_Cycles = true;
			if (new_FullCapNom != Loaded_FullCapNom) update_FullCapNom = true;
			
			// If there is new data available, write it to NVS
			if (update_RCOMP0 || update_TempCo || update_FullCapRep || update_Cycles || update_FullCapNom) {
				// Open
			    if (DEBUG) ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
			    err_check = nvs_open("FUELGAUGE", NVS_READWRITE, &nvs_fuelGauge_handle);
			    if (err_check != ESP_OK) {
			        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err_check));
			        nvs_close(nvs_fuelGauge_handle);
			    }
			    else {
				    if (DEBUG) ESP_LOGI(TAG, "Done opening NVS handle.");
				    
				    // Write to NVS
				    if (DEBUG) ESP_LOGI(TAG, "Writing MAX17055 parameters to NVS...");
				    
				    if (update_RCOMP0 == true) {
					    err_check = checkAndWrite_nvs(nvs_fuelGauge_handle, "RCOMP0", new_RCOMP0);
					    if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while writing RCOMP0 to NVS: (%s)", esp_err_to_name(err_check));
					}
					
					if (update_TempCo == true) {
						err_check = checkAndWrite_nvs(nvs_fuelGauge_handle, "TEMPCO", new_TempCo);
					    if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while writing TEMPCO to NVS: (%s)", esp_err_to_name(err_check));
					}
					
					if (update_FullCapRep == true) {
						err_check = checkAndWrite_nvs(nvs_fuelGauge_handle, "FULLCAPREP", new_FullCapRep);
					    if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while writing FULLCAPREP to NVS: (%s)", esp_err_to_name(err_check));
					}
					
					if (update_Cycles == true) {
						err_check = checkAndWrite_nvs(nvs_fuelGauge_handle, "CYCLES", new_Cycles);
					    if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while writing CYCLES to NVS: (%s)", esp_err_to_name(err_check));
					}
					
					if (update_FullCapNom == true) {
						err_check = checkAndWrite_nvs(nvs_fuelGauge_handle, "FULLCAPNOM", new_FullCapNom);
					    if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while writing FULLCAPNOM to NVS: (%s)", esp_err_to_name(err_check));
					}
				
				    // Commit written value.
				    // After setting any values, nvs_commit() must be called to ensure changes are written
				    // to flash storage. Implementations may write to storage at other times,
				    // but this is not guaranteed.
				    if (DEBUG) ESP_LOGI(TAG, "Committing updates in NVS ... ");
				    err_check = nvs_commit(nvs_fuelGauge_handle);	
				    if (err_check != ESP_OK) ESP_LOGE(TAG, "Error occurred while commiting nvs data: (%s)", esp_err_to_name(err_check));  
				      
				    // Close
			   		nvs_close(nvs_fuelGauge_handle);
			   		
			   		ESP_LOGI(TAG, "Done updating MAX17055 learned parameters to NVS.");
		   		}
	   		}
   		}
   		vTaskDelay(UPDATENVSPARAM_PAUSE / portTICK_PERIOD_MS);
    }
}

esp_err_t init_fuelGauge(void) {
	esp_err_t err_check;
	
	// Initialize I2C
	err_check = init_i2c();
	if (err_check != ESP_OK) return err_check;
	
	// Check for Power-On Reset (POR)
	int8_t status_reg_por = checkPOR();
	ESP_LOGI(TAG, "POR Flag: %d", status_reg_por);
	
	// Store original hibernate-configuration value
	uint16_t HibCFG;
	err_check = readRegister(HIBCFG_REG, &HibCFG);
	if (err_check != ESP_OK) return err_check;
	
	/* 
	If POR flag is set:
	- check the Data-Not-Read (DNR) Flag in FSTAT Register
	- initialize configuration
	*/
	if (status_reg_por == 1) {
		
		// Request FSTAT reg and keep requesting until (DNR == 0)
		ESP_LOGI(TAG, "Checking the DNR Flag in FSTAT Register...");
		uint16_t fstat_reg_rd;
		bool fstat_reg_dnr = 1;
		do {
			err_check = readRegister(FSTAT_REG, &fstat_reg_rd);
			if (err_check != ESP_OK) {
				ESP_LOGE(TAG, "Reading FSTAT Register failed!");
				return err_check;
			}
		
			// Check DNR Flag
			fstat_reg_dnr = (fstat_reg_rd & FSTAT_REG_DNR);
			
			// If data is still not ready, wait for 10ms and request data again
			if (fstat_reg_dnr == 1) vTaskDelay(10 / portTICK_PERIOD_MS);
		} while (fstat_reg_dnr == 1);
		ESP_LOGI(TAG, "DNR Flag: %d", fstat_reg_dnr);
		
		/* Initialize Configuration */
		
		// Soft-Wake and exiting Hibernate Mode (see User-Guide "1.10 Soft-Wakeup" p.34)
		// Exit Hibernate Mode step 1
		err_check = writeAndVerifyRegister(0x60, 0x90);
		if (err_check != ESP_OK) return err_check;
		// Exit Hibernate Mode step 2
		err_check = writeAndVerifyRegister(0xBA, 0x0);
		if (err_check != ESP_OK) return err_check;
		// Exit Hibernate Mode step 3
		err_check = writeAndVerifyRegister(0x60, 0x0);
		if (err_check != ESP_OK) return err_check;
		
		/* EZ Config */
		// Write DesignCap (expected battery capacity)
		uint16_t DesignCapStFormat = convCapToStFormat(EXPECTED_BATTERY_CAP);
		err_check = writeAndVerifyRegister(DESIGNCAP_REG, DesignCapStFormat);
		if (err_check != ESP_OK) return err_check;
		
		// Write dQAcc (change in battery charge between relaxation points).
		// DesignCap / 32 is standard value from Software-Impl.-Guide
		err_check = writeAndVerifyRegister(DQACC_REG, DesignCapStFormat/32);
		if (err_check != ESP_OK) return err_check;
		
		// Write IchgTerm (The current when a charge cycle of the cell has completed)
		// This value depends on the battery-charging IC
		uint16_t IChgTermStFormat = convCurrentToStFormat(ENDOFCHARGE_CURRENT);
		err_check = writeAndVerifyRegister(ICHRGTERM_REG, IChgTermStFormat);
		if (err_check != ESP_OK) return err_check;
		
		// Write VEmpty (treshold related to empty detection during operation and recovery voltage)
		err_check = writeAndVerifyRegister(VEMPTY_REG, VEMPTY_REG_VALUE);
		if (err_check != ESP_OK) return err_check;
		
		// Write dPACC according to Software-Impl.-Guide
		uint16_t dQAcc = (uint16_t)(DesignCapStFormat / 32);
		err_check = writeAndVerifyRegister(DPACC_REG, (dQAcc * 44138) / DesignCapStFormat);
		if (err_check != ESP_OK) return err_check;
		
		// In ModellCfg Register is Battery type and charge voltage configured. 
		// For LiPo, standard values are used. REFRESH bit is set.
		err_check = writeAndVerifyRegister(MODELCFG_REG, 0x8000);
		if (err_check != ESP_OK) return err_check;
		
		// Wait until REFRESH Bit is 0
		uint16_t modelcfg_reg_rd;
		bool modelcfg_reg_refresh_rd = 1;
		do {
			err_check = readRegister(MODELCFG_REG, &modelcfg_reg_rd);
			if (err_check != ESP_OK) return err_check;
			modelcfg_reg_refresh_rd = modelcfg_reg_rd & MODELCFG_REG_REFRESH;
			if (modelcfg_reg_refresh_rd) vTaskDelay(10 / portTICK_PERIOD_MS);
		} while(modelcfg_reg_refresh_rd == 1);
		ESP_LOGI(TAG, "Refreshing finished. REFRESH Flag: %d", modelcfg_reg_refresh_rd);
		
		// Restore original HibCfg value
		err_check = writeAndVerifyRegister(HIBCFG_REG, HibCFG);
		if (err_check != ESP_OK) return err_check;
	}
	// Read STATUS Register (again)
	uint16_t status_reg_rd;
	err_check = readRegister(STATUS_REG, &status_reg_rd);
	if (err_check != ESP_OK) return err_check;
	ESP_LOGI(TAG, "STATUS REG: %d", status_reg_rd);
	
	// Clear the POR bit to indicate that the custom model and parameters were successfully loaded.
	err_check = writeAndVerifyRegister(STATUS_REG, status_reg_rd & 0xFFFD);
	if (err_check != ESP_OK) return err_check;
	else ESP_LOGI(TAG, "Power-On Reset (POR) Bit successfully cleared.");
	
	ESP_LOGI(TAG, "Finished initializing fuel gauge.");
	
	return ESP_OK;
}

static esp_err_t init_i2c(void) {
	
	if (i2c_isinit == false) {
		esp_err_t err_check;
		
		// Master bus configuration
		i2c_master_bus_config_t i2c_mst_config = {
			.i2c_port = I2C_NUM_0,
			.sda_io_num = SDA_PIN,
			.scl_io_num = SCL_PIN,
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7, // typical value is 7
			.intr_priority = 0, // if set to 0, driver will select the default priority (1,2,3)
			.flags.enable_internal_pullup = false,
		};
		
		// Create new master bus using config and check for error
		err_check = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
		if (err_check != ESP_OK) return err_check;
		
		// Device (slave) configuration
		i2c_device_config_t dev_cfg = {
	    	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    	.device_address = I2C_DEVICE_ADDRESS, // 7MSbit slave address for MAX17055
	    	.scl_speed_hz = I2C_SCL_SPEED,
	    	.scl_wait_us = 0,
	    	.flags.disable_ack_check = false,
		};
		
		// Add new device on the bus and check for error
		err_check = i2c_master_bus_add_device( bus_handle, &dev_cfg, &dev_handle );
		if (err_check != ESP_OK) return err_check;
		
		ESP_LOGI(TAG, "I2C for fuel gauge IC successfully initialized.");
		
		// Initialize the mutex
        i2c_mutex = xSemaphoreCreateBinary();
        if (i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex.");
            return ESP_FAIL;
        }
        xSemaphoreGive(i2c_mutex);
		
		// Set variable true to avoid redundant initialization
		i2c_isinit = true;
	}
	else {
		ESP_LOGI(TAG, "I2C is already initialized. Skipping initialization.");
	}
	
	return ESP_OK;
}

static esp_err_t readRegister(uint8_t reg, uint16_t *data_rd) {
	esp_err_t err_check;
	
	// Array used for splitting data in Low byte and High byte
	uint8_t data_rd_prep[2];
	
	// Take the mutex
    if (xSemaphoreTake(i2c_mutex, ( TickType_t ) 10) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex.");
        return ESP_FAIL;
    }
	
	err_check = i2c_master_transmit_receive(dev_handle, &reg, sizeof(reg),\
									data_rd_prep, 2, I2C_WAITTIMEOUT);
									
	// Release the mutex
    xSemaphoreGive(i2c_mutex);
    
	*data_rd = (data_rd_prep[1]<<8) | data_rd_prep[0];
	
	if (DEBUG) ESP_LOGI(TAG, "Read 0x%04X from register 0x%02X using readRegister function.", *data_rd, reg);
	
	
	return err_check;
}

static esp_err_t writeRegister(uint8_t reg, uint16_t data_wr) {
	esp_err_t err_check;
	
	// Array used for splitting data in Low byte and High byte
	uint8_t data_wr_prep[3];
	// Save Register to array
	data_wr_prep[0] = reg;
	// Low byte
	data_wr_prep[1] = data_wr & 0xFF;
	// High byte
	data_wr_prep[2] = (data_wr >> 8) & 0xFF;
	
	// Take the mutex
    if (xSemaphoreTake(i2c_mutex, ( TickType_t ) 10) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire I2C mutex.");
        return ESP_FAIL;
    }
	
	if (DEBUG) ESP_LOGI(TAG, "Writing 0x%04X to register 0x%02X using writeRegister function.", data_wr, reg);
	
	err_check = i2c_master_transmit(dev_handle, data_wr_prep , sizeof(uint8_t)*3, I2C_WAITTIMEOUT);
	
	// Release the mutex
    xSemaphoreGive(i2c_mutex);
    
	return err_check;
}

static esp_err_t writeAndVerifyRegister(uint8_t reg, uint16_t data_wr) {
	uint16_t *data_rd = malloc(sizeof(uint16_t));
	if (data_rd == NULL) {
		ESP_LOGE(TAG, "Allocating memory for %d Register data failed!", reg);
		return ESP_ERR_NO_MEM;
	}
	
	esp_err_t err_check;
	
	uint8_t i;
	const uint8_t max_trial_count = 3;
	for (i=0; i<max_trial_count; i++) {
		err_check = writeRegister(reg, data_wr);
		if (err_check != ESP_OK) return err_check;
		
		vTaskDelay(1 / portTICK_PERIOD_MS);
		
		err_check = readRegister(reg, data_rd);
		if (err_check != ESP_OK) return err_check;
		
		if (data_wr == *data_rd) break;
		else {
			ESP_LOGI(TAG, "Write and verify data not matching! Try %d out of %d. Writing data: %d, reading data: %d",\
			i+1, max_trial_count, data_wr, *data_rd);
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	
	free(data_rd);
	
	if (i==3) {
		ESP_LOGE(TAG, "Write and verify failed.");
		return ESP_ERR_INVALID_RESPONSE;
	}
	
	if (DEBUG) ESP_LOGI(TAG, "Data 0x%04X successfully written to register 0x%02X and verified using writeAndVerifyRegister.", data_wr & 0xFFFF, reg & 0xFF);
	
	return ESP_OK;
}

// Convert Capacity value to standard format according to User-Guide
// capacity is in mAh
static uint16_t convCapToStFormat(uint16_t capacity) {
	return (uint16_t)(capacity  / (5.0 / (float) RSENSE_VALUE));
}

// Convert standard format data to capacity in mAh
static uint16_t convStFormatToCap(uint16_t capacityStFormat) {
	uint16_t calc = (uint16_t)((float)capacityStFormat * (5.0 / (float) RSENSE_VALUE));
	return calc;
}

// Convert Current value to standard format according to User-Guide
// current in mA
static uint16_t convCurrentToStFormat(uint16_t current) {	 				// TEST IF WORKS!
	return (uint16_t)((float) current / (1.5625 / (float) RSENSE_VALUE));
}

static uint8_t convStFormatToPerc(uint16_t percentageStFormat) {
	// D8 Bit is 1% Value. Bits D7, D6, ... , D0 are irrelevant
	return (uint8_t)(percentageStFormat >> 8);
}

static esp_err_t checkAndWrite_nvs(nvs_handle_t handle, char *key, uint16_t value) {
	esp_err_t err_check;
	
	if (DEBUG) ESP_LOGI(TAG, "Writing %d to NVS using key %s", value, key);
	
    err_check = nvs_set_u16(handle, key, value);
    if (err_check != ESP_OK) {
		ESP_LOGE(TAG, "Writing %s to NVS failed", key);
		nvs_close(nvs_fuelGauge_handle);
		return err_check;
	}
	
	return ESP_OK;
}

static esp_err_t openFuelGaugeNVSHandle(nvs_handle_t handle, nvs_open_mode_t mode) {
	esp_err_t err_check;
	
	if (DEBUG) ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
    err_check = nvs_open("FUELGAUGE", mode, &nvs_fuelGauge_handle);
    if (err_check != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) occurred while opening NVS handle!", esp_err_to_name(err_check));
        nvs_close(handle);
        return err_check;
    }
    if (DEBUG) ESP_LOGI(TAG, "Done opening NVS handle.");
	
	return ESP_OK;
}

static int8_t checkPOR(void) {
	esp_err_t err_check;
	uint16_t status_reg_rd;
	bool status_reg_por_rd;
	
	// Check for reset
	err_check = readRegister(STATUS_REG, &status_reg_rd);
	if (err_check != ESP_OK) return -1;
	
	status_reg_por_rd = status_reg_rd & STATUS_REG_POR;
	
	return (int8_t) status_reg_por_rd;
}