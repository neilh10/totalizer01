//TotalizerPrj.h


#define LedActivityUpdate(ledUpdate) LedFlashInterval_mS=ledUpdate
#define LED_FLASH_SLOW_MS 2000
#define LED_FLASH_FAST_MS  500
#define LedActivityUpdateFast() LedFlashInterval_mS=LED_FLASH_FAST_MS 
#define LedActivityUpdateSlow() LedFlashInterval_mS=LED_FLASH_SLOW_MS 
#define PIN_LED_ACTIVITY PIN_LED_TXL
