#include "Platform/board.h"

#include "Services/Timebase/timebase.h"

void Board_Init(void)
{
    /*
     * CubeMX çevre birimlerini Core/Src/main.c içinde başlatıyor.
     *
     * Burada:
     * HAL_Init yok
     * SystemClock_Config yok
     * MX_SPIx_Init yok
     * MX_I2Cx_Init yok
     * MX_TIMx_Init yok
     *
     * Sadece uygulama timebase servisi başlatılır.
     */

    Timebase_Init();
}
