
#pragma once

#if (__PROJECT_8266_HOST_DONGLE__)
	#include "../8266_host_dongle/app_config.h"
#elif (__PROJECT_8267_MASTER__)
	#include "../8267_master/app_config.h"
#elif (__PROJECT_8261_BEACON__)
	#include "../826x_beacon/app_config.h"
#elif (__PROJECT_8267_BEACON_PAD__)
	#include "../826x_beacon_pad/app_config.h"
#elif (__PROJECT_8267_BTUSB__)
	#include "../8267_btusb/app_config.h"
#elif (__PROJECT_8261_HCI__|__PROJECT_8267_HCI__|__PROJECT_8269_HCI__)
	#include "../826x_hci/app_config.h"
#elif (__PROJECT_8269_HCI__)
	#include "../826x_hci/8269_app_config.h"
#elif (__PROJECT_8261_MODULE__ || __PROJECT_8267_MODULE__ || __PROJECT_8269_MODULE__ || __PROJECT_8266_MODULE__)
	#include "../826x_module/app_config.h"
#elif (__PROJECT_826X_PROFILE_TEST__)
	#include "../826x_profile_test/8267_app_config.h"
#elif (__PROJECT_826X_SPI_MODULE__)
	#include "../826x_spi_module/8267_app_config.h"
#elif (__PROJECT_8267_SPI_MASTER__)
    #include "../8267_spi_master/app_config.h"
#elif (__PROJECT_DRIVER_TEST__)
	#include "../8267_driver_test/app_config.h"
#elif (__PROJECT_8261_BLE_REMOTE__ || __PROJECT_8266_BLE_REMOTE__ || __PROJECT_8267_BLE_REMOTE__ || __PROJECT_8269_BLE_REMOTE__)
	#include "../826x_ble_remote/app_config.h"
#elif(__PROJECT_8261_MASTER_KMA_DONGLE__ || __PROJECT_8266_MASTER_KMA_DONGLE__ || __PROJECT_8267_MASTER_KMA_DONGLE__ || __PROJECT_8269_MASTER_KMA_DONGLE__ )
	#include "../826x_master_kma_dongle/app_config.h"
#elif (__PROJECT_8261_FEATURE_TEST__ || __PROJECT_8266_FEATURE_TEST__ || __PROJECT_8267_FEATURE_TEST__ || __PROJECT_8269_FEATURE_TEST__)
	#include "../826x_feature_test/app_config.h"
#elif (__PROJECT_8261_DRIVER_TEST__ || __PROJECT_8266_DRIVER_TEST__ || __PROJECT_8267_DRIVER_TEST__ || __PROJECT_8269_DRIVER_TEST__)
	#include "../826x_driver_test/app_config.h"
#elif (__PROJECT_8261_HID_SAMPLE__ || __PROJECT_8266_HID_SAMPLE__ || __PROJECT_8267_HID_SAMPLE__ || __PROJECT_8269_HID_SAMPLE__)
	#include "../826x_hid_sample/app_config.h"
#elif (__PROJECT_8266_OTA_BOOT__ || __PROJECT_8261_OTA_BOOT__)
	#include "../826x_ota_boot/app_config.h"
#elif (__PROJECT_USB_HOST__)
	#include "../8267_usb_host/app_config.h"
#elif (__PROJECT_8266_AUDIO_MIC_SDM__)
	#include "../8266_ble_audio_mic_sdm/app_config.h"
#else
	#include "../common/default_config.h"
#endif

