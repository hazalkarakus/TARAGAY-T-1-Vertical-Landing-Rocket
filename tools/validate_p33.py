#!/usr/bin/env python3
from pathlib import Path
import ast
root=Path(__file__).resolve().parents[1]
def must(path,text):
    data=(root/path).read_text(errors='ignore')
    assert text in data, f'missing {text!r} in {path}'
for p,t in [
('App/Common/app_version.h','P33-FAST-IMU-SD1BIT'),
('App/Modules/Sensors/IMU/imu.c','IMU_SW_SCK_LOW()'),
('App/Modules/Sensors/IMU/imu.c','GPIOA->BSRR'),
('App/Modules/Sensors/IMU/imu.c','IMU_REDUNDANT_BURST_COUNT     3U'),
('FATFS/Target/bsp_driver_sd.c','forced/reliable 1-bit mode'),
('FATFS/Target/bsp_driver_sd.c','hsd.Init.BusWide = SDIO_BUS_WIDE_1B;'),
('App/Core/Tasks/app_tasks.c','SDLogger_PushFastIMU();'),
('App/Core/Tasks/app_tasks.c','SDLogger_PublishSources();'),
]: must(p,t)
imu=(root/'App/Modules/Sensors/IMU/imu.c').read_text()
assert 'for (volatile uint32_t i = 0UL; i < 40UL;' not in imu
assert 'HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)' not in imu.split('static uint8_t IMU_SW_SPI_Byte',1)[1].split('static HAL_StatusTypeDef IMU_SW_SPI_Transfer',1)[0]
bsp=(root/'FATFS/Target/bsp_driver_sd.c').read_text()
init=bsp.split('__weak uint8_t BSP_SD_Init(void)',1)[1].split('/* USER CODE BEGIN AfterInitSection */',1)[0]
assert 'HAL_SD_ConfigWideBusOperation' not in init
mon=(root/'monitor_uart_v55.py').read_text()
node=ast.parse(mon); fields=None
for n in node.body:
    if isinstance(n, ast.Assign) and any(isinstance(t,ast.Name) and t.id=='FIELDS' for t in n.targets):
        fields=ast.literal_eval(n.value)
assert fields is not None and len(fields)==149
print('P33 validation: PASS (fast SW-SPI + forced SDIO 1-bit + 149 UART fields)')
