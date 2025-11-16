/*
 * hal_translate.h
 *
 *  Created on: Nov 8, 2025
 *      Author: Mathis
 */

#ifndef INC_MY_STM32WL3X_HAL_H_
#define INC_MY_STM32WL3X_HAL_H_

#include "stm32wl3x_ll_mrsubg.h"
#include "stm32wl3x_hal_mrsubg.h"

#define HAL_MRSUBG_GET_DATABUFFER_COUNT()             READ_REG_FIELD(MR_SUBG_GLOB_STATUS->DATABUFFER_INFO, MR_SUBG_GLOB_STATUS_DATABUFFER_INFO_CURRENT_DATABUFFER_COUNT)
#define HAL_MRSUBG_GET_CURRENT_DATABUFFER()           READ_REG_FIELD(MR_SUBG_GLOB_STATUS->DATABUFFER_INFO, MR_SUBG_GLOB_STATUS_DATABUFFER_INFO_CURRENT_DATABUFFER)
#define HAL_MRSUBG_GET_NUMBER_OF_DATABUFFERS()        READ_REG_FIELD(MR_SUBG_GLOB_STATUS->DATABUFFER_INFO, MR_SUBG_GLOB_STATUS_DATABUFFER_INFO_NB_DATABUFFER_USED)

static inline uint8_t HAL_MRSUBG_GET_CHF_E(void) {
  return READ_REG_FIELD(MR_SUBG_GLOB_DYNAMIC->MOD1_CONFIG, MR_SUBG_GLOB_DYNAMIC_MOD1_CONFIG_CHFLT_E);
}
static inline uint8_t HAL_MRSUBG_GET_CHF_M(void) {
  return READ_REG_FIELD(MR_SUBG_GLOB_DYNAMIC->MOD1_CONFIG, MR_SUBG_GLOB_DYNAMIC_MOD1_CONFIG_CHFLT_M);
}

uint8_t crcBits(MRSubG_PcktCrcMode crcMode) {
	switch (crcMode) {
		case PKT_NO_CRC: return 0;
		case PKT_CRC_MODE_8BITS: return 8;
		case PKT_CRC_MODE_16BITS_1:
		case PKT_CRC_MODE_16BITS_2:
			return 16;
		case PKT_CRC_MODE_24BITS:
			return 24;
		case PKT_CRC_MODE_32BITS:
			return 32;
		default:
			return 0;
	}
}

// Debug Translates

char* tr_FSMState(MRSubGFSMState state) {
	switch (state) {
		case STATE_IDLE: return "STATE_IDLE";
		case STATE_ENA_RF_REG: return "STATE_ENA_RF_REG";
		case STATE_WAIT_ACTIVE2: return "STATE_WAIT_ACTIVE2";
		case STATE_ACTIVE2: return "STATE_ACTIVE2";
		case STATE_ENA_CURR: return "STATE_ENA_CURR";
		case STATE_SYNTH_SETUP: return "STATE_SYNTH_SETUP";
		case STATE_CALIB_VCO: return "STATE_CALIB_VCO";
		case STATE_LOCKRXTX: return "STATE_LOCKRXTX";
		case STATE_LOCKONTX: return "STATE_LOCKONTX";
		case STATE_EN_PA: return "STATE_EN_PA";
		case STATE_TX: return "STATE_TX";
		case STATE_PA_DWN_ANA: return "STATE_PA_DWN_ANA";
		case STATE_END_TX: return "STATE_END_TX";
		case STATE_LOCKONRX: return "STATE_LOCKONRX";
		case STATE_EN_RX: return "STATE_EN_RX";
		case STATE_EN_LNA: return "STATE_EN_LNA";
		case STATE_RX: return "STATE_RX";
		case STATE_END_RX: return "STATE_END_RX";
		case STATE_SYNTH_PWDN: return "STATE_SYNTH_PWDN";
		default: return "UNKNOWN_STATE";
	}
}

char* tr_ModSelect(MRSubGModSelect mod) {
	switch (mod) {
		case MOD_2FSK: return "MOD_2FSK";
		case MOD_4FSK: return "MOD_4FSK";
		case MOD_2GFSK05: return "MOD_2GFSK05";
		case MOD_2GFSK1: return "MOD_2GFSK1";
		case MOD_4GFSK05: return "MOD_4GFSK05";
		case MOD_4GFSK1: return "MOD_4GFSK1";
		case MOD_ASK: return "MOD_ASK/MOD_OOK";
		// case MOD_OOK: return "MOD_OOK";
		case MOD_POLAR: return "MOD_POLAR";
		case MOD_CW: return "MOD_CW";
		default: return "UNKNOWN_MODULATION";
	}
}

#endif /* INC_MY_STM32WL3X_HAL_H_ */
